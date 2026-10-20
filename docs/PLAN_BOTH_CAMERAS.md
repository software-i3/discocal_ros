# Plan: fast two-camera intrinsic recalibration with a fixed extrinsic

Status: **implemented 2026-10-18.** Written 2026-10-18 for a fresh session;
every fact below was verified against the tree on that date. The implementation
follows this plan through both-camera collection, the shared holdout, the mono
solves, the optional joint refinement with the extrinsic held bit-constant,
`camera_info` + `rig_calibration.yaml` + `.dwecal` output in a timestamped run
directory, and the bag benchmark. Deviations and measured results are in
`README.md` and `docs/DESIGN.md`; the one design change this plan did not
anticipate is that the action server uses `actionlib::ActionServer` rather than
`SimpleActionServer`, because the latter has a single current goal and its
`setPreempted()` cancelled a replacement goal (see DESIGN §11).

## 0. The request this plan answers

> Recalibrate the intrinsics of both cameras quickly, with a predefined (never
> estimated) extrinsic. Expose the topic name, `n_x`, `n_y`, radius/diameter and
> the other useful knobs in the rig YAML. Keep the node non-blocking so the rest
> of the system can work while the calibration runs. Bring over the optimisation
> work that already exists in `~/beex_work/discocal`'s working tree.

Four decisions were already made and are binding:

| Question | Decision |
| --- | --- |
| Solve method | Mono solve per camera, then an optional joint refinement with the extrinsic held constant. Joint refine must be switchable in YAML. |
| Non-blocking | Dedicated `ros::CallbackQueue` + `AsyncSpinner` for images, solve on the existing worker thread, configurable OpenMP/Ceres thread counts, OpenCV threads and `nice`. No need to keep collecting during a solve. |
| Action API | Extend `CalibrateCamera`: accept `camera: "both"` / `"all"` and return a `CameraCalibration[]` array. Keep the scalar result fields populated for a single-camera goal. |
| Budget | Configurable, not a new hard-coded default. Expose it and document the measured trade-off. |

## 1. What exists today (verified)

### 1.1 The package

`/home/tuanpham/catkin_ws/src/discocal_ros` is a ROS Melodic package, C++17,
GCC 7.5 / OpenCV 3.2 verified. It vendors discocal under
`src/discocal/` (CC BY-NC-SA 4.0, see `THIRD_PARTY_NOTICES.md`).

Current flow, all for **one** camera:

1. `ImageSource` (`src/image_source.cpp`) subscribes to one topic, converts to
   BGR, and hands the frame to `DiscocalNode::onFrame`, which enqueues it
   (bounded at `kMaxQueuedFrames = 30`, `src/node.cpp:65`).
2. One worker thread (`workerLoop`, `src/node.cpp:625`) owns the detectors and
   the session. `processFrame` (`:407`) runs detection, feeds the goal camera to
   `CalibrateSession::on_detection` and the other camera to
   `on_other_detection`.
3. `CalibrateSession::finalize` (`src/calibrate_session.cpp:182`) splits accepted
   frames into fit/holdout, runs `Calibrator::calibrate`, re-solves the held-out
   poses with `Calibrator::update_Es`, runs `run_cross_camera_check` against the
   partner's intrinsics through the fixed extrinsic, and writes
   `<output>/<camera>_camera_info.yaml` only if the check passes.
4. Retry: `check.retries` more collections, then report.

Important properties already true:

- `ros::spin()` runs in `main` (`src/node.cpp:668`); the worker is a second
  thread. Image callbacks and action callbacks currently share that one spin
  thread.
- Detection must stay single-threaded: `TargetDetector.cpp:628` keeps a
  non-atomic `static int dbg_call`.
- `Calibrator` is mono and already has `DISC_TIME` and `DISC_SPARSE_SCHUR`
  (`src/discocal/lib_calib/CCalibrator.cpp:5-29`).
- The fixed extrinsic is already parsed, oriented, and never estimated.
- `knownIntrinsics()` (`src/node.cpp:304`) lets one camera be checked against a
  `source: file` camera or against a camera calibrated earlier in the process.
- `config/rig_example.yaml` already exposes `topic`, `n_x`, `n_y`, `radius`,
  `distance`, `n_d`, `edge_check`, the frame policy, the check and the extrinsic.

### 1.2 The upstream working-tree work (what to port)

Working tree at `/home/tuanpham/beex_work/discocal`, HEAD `500a665`, uncommitted:

```
M CMakeLists.txt
M src/lib_calib/CCalibrator.cpp
M src/lib_detect/CTargetDetector.cpp
?? DETECTION_ROBUSTNESS.md  calib_eval/  calibration_set_*/  demo_1x_subset/
```

**Detector: nothing to port.** The vendored
`src/discocal/lib_detect/CTargetDetector.cpp` (952 lines) already contains the
whole detection WIP: `DISC_EDGE_CHECK` appearance gate, relaxed pass, probes,
`DISC_SIZE_THRESHOLD` / `DISC_DIST_THRESHOLD` / `DISC_MAX_CANDIDATES` env
overrides. It differs from the working-tree file (948 lines) only by the OpenCV
3.2 build fix at 906-912, which is required here. **Do not copy the upstream
detector over it** -- the upstream `cv::Mat_<double>({2,3},{...})` form is
OpenCV 4 only.

**Calibrator: three hunks to port.** The vendored
`src/discocal/lib_calib/CCalibrator.cpp` (894 lines) lacks only the performance
work. The exact diff against the working tree is:

1. **Three env helpers** after `disc_phase` (working tree lines 21-57; insert
   after the vendored `disc_phase` at line 18):
   - `disc_block_cov()` -- `DISC_BLOCK_COV`
   - `disc_unc_dump()` -- `DISC_UNC_DUMP`
   - `disc_sparse_info()` -- `DISC_SPARSE_INFO`
2. **The uncertainty block**, vendored lines 240-303 replaced by the working
   tree's lines 277-469:
   - `need_dense_j` / conditional dense `J` and `J_real`
   - Tier 2 `DISC_SPARSE_INFO`: reads the Ceres `CRSMatrix` directly, blob `k`
     owns rows `2k`/`2k+1`, column map `n_keep = 9 - n_skip_cols`, whiten with
     `Eigen::LLT<Matrix2d>` on the inverse 2x2 covariance, accumulate
     `info += wa wa^T + wb wb^T`
   - Tier 1 `DISC_BLOCK_COV`: `rankUpdate` on the lower triangle, mirror before
     `inverse()`
   - dense `else` branch retained, with its own `t_gemm` mark
3. **Sigma dump**: after `params_cov[i] = sqrt(cov_matrix(i,i))` (vendored line
   300), print the same values at 17 digits when `DISC_UNC_DUMP=1`.

**Do not port** the four `params.to_table(...)` hunks (vendored lines 128, 766,
784, 794). `tabulate` was deliberately removed from the vendored copy; keep
`to_string_detailed()`.

**Preserve CRLF.** Both files are CRLF. Writing LF back produces a ~900-line
phantom diff. After editing, verify with `file`; if needed, normalise with a
CRLF-preserving pass.

The measured payoff (from `DETECTION_ROBUSTNESS.md` §3.3-3.4, SSE2 build,
`calibration_set_big`, 275 frames):

| Path | calib | wall | peak RSS |
| --- | --- | --- | --- |
| dense (stock) | OOM at 486 s | -- | ~9.5 GB |
| `DISC_BLOCK_COV=1` | 120.6 s | 143.9 s | 0.67 GB |
| `DISC_SPARSE_INFO=1` | 22.2 s | 42.7 s | 0.16 GB |

`DISC_SPARSE_INFO` is algebraically the same `J' M_cov^-1 J`; sigmas agree with
the dense path to 3.3e-11 / 6.7e-12. Cost is linear in frames. At Tier 2,
detection (0.076 s/frame) and calibration (0.086 s/frame) are comparable, so
**~190 frames ≈ 30 s** wall for one camera.

### 1.3 `StereoCalibrator` (not yet vendored)

`include/CStereoCalibrator.h`, `src/lib_calib/CStereoCalibrator.cpp` (119 lines)
in the upstream tree. C++11-only, OpenCV-free. It shares per-frame board poses
across cameras, which is the accuracy gain over two independent mono solves.

```cpp
StereoCalibrator(int n_x, int n_y, bool asymmetric, double distance);
void calibrate(const vector<vector<Target>>& all_targets,
               vector<se3>& Es,
               vector<se3>& all_ext,
               vector<Params>& all_params,
               const vector<double>& all_radius,
               const vector<double>& all_n_d,
               bool full_optimize);
```

Verified facts:

- `Target = std::pair<bool, std::vector<Shape>>` (`include/utils.h:130`).
- `all_ext[i-1] = T_{c_i}^{c_0}` (camera i -> camera 0). The functor builds
  `T_w^{c_i} = (T_{c_i}^{c_0})^{-1} * T_w^{c_0}` (`CFunctors.h:108`).
- `all_ext` is **always** a free parameter block today
  (`CStereoCalibrator.cpp:96-101`). `full_optimize=false` only makes `K` and
  `d` constant (`:103-106`).
- `mode = 0` is hard-coded (`:71`): circle boards only.
- `DENSE_SCHUR` is hard-coded (`:110`).
- It ignores `Solver::Summary` and returns `void`.
- It computes **no uncertainty**; `Params::s_K` / `s_d` come back untouched.
- `normalize_Es(all_ext)` runs after the solve (`:117`), and it can flip the
  translation sign and two rotation columns when `trans(2) < 0`. With a fixed
  extrinsic that must not be allowed to touch the caller's value.
- `se3 { Eigen::Vector3d rot; Eigen::Vector3d trans; }`
  (`include/CLieAlgebra.h:20`); `rot` is a Rodrigues vector.
- `CMomentsTracker.cpp` and `CLieAlgebra.cpp` are already compiled into
  `discocal_vendor`; add only `CStereoCalibrator.cpp` or every
  `MomentsTracker::` / `LieAlgebra::` symbol duplicates.
- `include/stereo.hpp` is **not** to be vendored: it uses `std::filesystem` and
  structured bindings, and has a `set_image_size(rows, cols)` swap bug at :122.
  Replicate only the relative-pose/`calibrate` call.

## 2. Target design

### 2.1 Config schema

Existing groups stay. Additions:

```yaml
board:
  type: circle
  n_x: 7
  n_y: 5
  radius: 0.0145       # metres, dot RADIUS (the one canonical size field)
  distance: 0.039
  n_d: 2
  detection_mode: ""
  edge_check: true

detector:                # new; numeric detector knobs
  size_threshold_px2: 0       # 0 -> library default
  dist_threshold_px: 0        # 0 -> library default
  max_candidates: 0           # 0 -> library default
  edge_contrast: 0            # 0 -> library default (25)
  edge_inner_uniformity: 0    # 0 -> library default (25)
  edge_outer_uniformity: 0    # 0 -> library default (80)
  probe: false                # DISC_DEBUG_PROBE
  edge_stats: false           # DISC_EDGE_STATS

performance:             # new
  sparse_info: true           # DISC_SPARSE_INFO   (Tier 2, linear in frames)
  block_cov: false            # DISC_BLOCK_COV    (only when sparse_info=false)
  sparse_schur: true          # DISC_SPARSE_SCHUR
  unc_dump: false             # DISC_UNC_DUMP     (17-digit sigma dump)
  time: false                 # DISC_TIME         (phase timings)
  detector_threads: 0         # 0 -> OpenMP default (all cores)
  ceres_threads: 1            # Ceres Solver::Options::num_threads
  opencv_threads: 1           # cv::setNumThreads
  nice: 0                     # worker thread; -20..19
  async_spinner_threads: 1    # image callback queue

solve:                   # new
  joint_refine: true          # only used by camera: "both"/"all"
  joint_refine_fallback: true # fall back to the mono results if it fails

frames:                  # semantics clarified
  budget: 100                 # ACCEPTED frames PER CAMERA in both mode
  max_duration_s: 300.0
  diversity_gate: true
  diversity_threshold: 0.035
  holdout_fraction: 0.15      # shared holdout in both mode

check:
  enabled: true
  max_rms_px: 4.0
  retries: 1

cameras:
  left:
    topic: /ikan/explore3d/left/compressed
    transport: compressed
    frame_id: explore3d
    source: calibrate
  right:
    topic: /ikan/explore3d/right/compressed
    transport: compressed
    frame_id: explore3d
    source: calibrate

extrinsic:
  parent: left
  child: right
  rotation: {rx: 0.005870, ry: 0.013980, rz: -0.013437}
  translation: {x: 0.099966, y: -0.001510, z: 0.006461}

output:
  directory: /tmp/discocal_ros
```

Rules:

- **One size field: `board.radius`, the dot radius in metres.** It is what
  `Calibrator`/`Shape` already take (`original_r`), what the vendored library
  calls `radius`, and what the existing config already exposes. Do not add a
  `diameter` key or an alias: two names for one physical quantity invites a
  config that silently means two different things. The dot diameter is simply
  `2 * radius`; say that in the comment, not in a second field.
- `detector.*` values are applied with `setenv` in the node constructor,
  before any `TargetDetector` is constructed. `DISC_SIZE_THRESHOLD` is read in
  the `TargetDetector` constructor (`CTargetDetector.cpp:22`), so it cannot be
  changed later; the rest are read per detection. A value of 0 means "leave
  the library default"; do **not** write `"0"` to the environment for the
  numeric knobs, because the detector applies any value that is present
  (`getenv` test, then `atof`/`atoi`). `DISC_EDGE_CHECK` and
  `DISC_EDGE_STATS` are value-based and `DISC_EDGE_CHECK` is always written
  (`"0"` means off, as today). `DISC_DEBUG_PROBE` is **presence**-based, so
  `detector.probe: false` must leave it unset rather than writing `"0"`.
- `performance.sparse_info` defaulting to `true` is a deliberate change from
  the library's default. The equivalence is ~1e-11; the speed is the point.
  Set it to `false` to reproduce the dense path.
- `performance.block_cov` is only consulted when `sparse_info` is false. It is
  kept for the A/B oracle, not as a production path.
- `solve.joint_refine` is only meaningful for `camera: "both"` / `"all"`.
  Setting it in a mono goal is ignored (log a note).
- `frames.budget` counts accepted frames per camera in both mode. The goal ends
  when **both** cameras reach it.
- Validation: `detector_threads >= 0`, `ceres_threads >= 1`,
  `opencv_threads >= 1`, `nice` in [-20, 19], `async_spinner_threads >= 1`.

### 2.2 Both-camera data flow

```
image topics ──▶ ImageSource (async image callback queue, 1 thread)
                    │  enqueue BGR + stamp
                    ▼
              bounded frame queue (30)
                    │
                    ▼
       worker thread (single detector at a time)
                    │
              RigSession::on_detection(frame)
                 ├─ left  session: on_detection / on_other_detection
                 └─ right session: on_detection / on_other_detection
                    │
        both budgets reached
                    ▼
       RigSession::solve()
         1. common holdout split (by stamp)
         2. left  mono solve  (Calibrator)
         3. right mono solve  (Calibrator)
         4. optional joint refine (StereoCalibrator, extrinsic fixed)
         5. per-camera held-out check with the FINAL params
         6. write both camera_info files
                    │
                    ▼
             action result (CameraCalibration[])
```

Each `CalibrateSession` keeps its own diversity gate and its own accepted set.
A stamp accepted by one camera and not the other is still usable for that
camera's mono fit; it simply cannot be part of the joint scene set or the
common holdout.

### 2.3 Common holdout (must not leak between the cameras)

The current per-session split is independent, so camera A could fit a frame
that camera B holds out and then scores A against B's intrinsics on it. In both
mode, use one split by stamp:

1. `paired` = sorted stamps accepted by **both** cameras.
2. `n_holdout = round(holdout_fraction * |paired|)`, clamped so each camera
   still has at least `min_fit_frames()` fit frames.
3. Pick them with the existing `spread_indices` (evenly spaced).
4. Both sessions exclude exactly those stamps from their fit sets.

Then:

- Camera A's check scores `paired ∩ holdout` using A's final intrinsics and B's
  final intrinsics.
- Camera B's check is symmetric.
- No scored frame was fitted in either camera, so the check is independent of
  both solves.

If `|paired|` is too small, the check reports why (as today) rather than
silently scoring fitted frames.

### 2.4 Solve pipeline in both mode

1. **Mono solve, per camera.** `Calibrator` with that camera's fit frames
   (accepted minus the common holdout). This produces `Params`, the board
   poses by stamp, the library reprojection error, and the 1-sigma values.
2. **Joint refine (optional).** Scene set = stamps present in **both** mono
   fit sets, sorted. Build:
   - `all_targets[0]` = parent's detections for those stamps
   - `all_targets[1]` = child's detections for those stamps
   - `Es` = parent's mono board poses, converted to `se3` from the
     `Eigen::Matrix4d` stored in the solve outcome (board -> parent camera)
   - `all_params[0]` = parent's mono `Params`, `[1]` = child's
   - `all_radius[0..1]` = `board.radius`; `all_n_d[0..1]` = `board.n_d`
   - `all_ext = { se3(LieAlgebra::to_so3(R), t) }` from the config, **parent as
     camera 0**, so the direction is exactly `p_parent = R p_child + t` and no
     inversion is needed.
   - Call `calibrate(..., full_optimize=true, fix_extrinsic=true,
     &converged)`.
3. **Validate the joint result.** Finite values, positive `fx`/`fy`, and
   `converged`. If it fails and `solve.joint_refine_fallback` is true, keep
   the mono results and report `joint_refined: false` with a warning. If the
   fallback is disabled, fail the goal.
4. **Re-check with the final params.** The held-out poses must be re-solved
   with the final intrinsics (`Calibrator::update_Es`), because the joint
   refine changed them. Do **not** reuse the mono-stage held-out poses.
5. **Verdict.** Both cameras must pass (or `allow_unchecked`). If either fails,
   the goal fails and the retry re-collects both; report both cameras' numbers.
6. **Write.** Both camera_info files, temp-then-rename; then a
   `rig_calibration.yaml` summary (optional but useful: both intrinsics, the
   extrinsic as used, the check numbers, the frame counts, the joint flag).

Uncertainty caveat: `StereoCalibrator` computes no covariance. The action
result for a joint-refined camera carries the **mono-stage** sigmas. Add a
`bool uncertainties_from_mono_stage` field to the per-camera message rather
than pretending they belong to the joint solution. The check is the real
verdict; the sigmas are an order-of-magnitude estimate.

### 2.5 Joint-refine patches to `StereoCalibrator`

Vendor `include/CStereoCalibrator.h` and `src/lib_calib/CStereoCalibrator.cpp`
from the upstream tree unchanged in substance, then apply:

```cpp
// header
void calibrate(..., bool full_optimize,
               bool fix_extrinsic = false,
               bool* converged = NULL);
```

```cpp
// .cpp
if (!full_optimize) {
    problem.SetParameterBlockConstant(all_params[camera_index].K);
    problem.SetParameterBlockConstant(all_params[camera_index].d);
}
// NEW: the extrinsic is a *given*, never a parameter.
if (fix_extrinsic) {
    for (std::size_t i = 0; i + 1 < all_ext.size(); ++i) {
        problem.SetParameterBlockConstant(all_ext[i].rot.data());
        problem.SetParameterBlockConstant(all_ext[i].trans.data());
    }
}

ceres::Solver::Options options;
options.linear_solver_type = ceres::SPARSE_SCHUR;  // NEW; or env-selectable
options.num_threads = disc_ceres_threads();        // NEW; 1 by default
options.minimizer_progress_to_stdout = false;
ceres::Solver::Summary summary;
Solve(options, &problem, &summary);
if (converged) *converged = (summary.termination_type == ceres::CONVERGENCE);

normalize_Es(Es);
if (!fix_extrinsic) normalize_Es(all_ext);   // NEW guard: never mutate a given
```

Notes:

- `disc_ceres_threads()` reads `DISC_CERES_THREADS` once; default 1. Duplicate
  the tiny helper in this file (it is `static` in `CCalibrator.cpp`).
- `SPARSE_SCHUR` is the right default for the joint problem (one pose block per
  scene plus 2 intrinsic blocks plus 1 extrinsic block). Keep `DENSE_SCHUR`
  selectable via the same `DISC_SPARSE_SCHUR` env if the A/B is ever wanted.
- `normalize_Es(all_ext)` is **not** an identity operation: it flips
  `trans(2)` and two rotation columns when `trans(2) < 0`. Guard it as above.
- `all_n_d` is `vector<double>` but the functor takes `int`; pass exact values
  (2.0), not a rounded approximation.
- `mode = 0` stays hard-coded. If `board.type != "circle"`, skip the joint
  refine with a clear log/message (mono results remain valid).
- `all_targets[0]` and `all_targets[1]` must have identical lengths and every
  `Target::first` must be true. The `camera_index > 0` loop indexes
  `all_targets[0].at(scene_index)` without checking `camera_index`'s own
  vector length.

### 2.6 Action API

Add `msg/CameraCalibration.msg`:

```
string  camera
bool    success
string  message

float64 fx
float64 fy
float64 cx
float64 cy
float64 skew
float64 d1
float64 d2
float64 d3
float64 d4

float64 s_fx
float64 s_fy
float64 s_cx
float64 s_cy
float64 s_skew
float64 s_d1
float64 s_d2
float64 s_d3
float64 s_d4
bool    uncertainties_from_mono_stage

float64 reprojection_error
uint32  n_frames
uint32  n_frames_heldout
string  camera_info_path

bool    check_ran
bool    check_passed
uint32  check_pairs
float64 check_rms_px
float64 check_max_rms_px
float64 check_worst_pair_rms_px

float64 fit_seconds
```

Optionally `msg/CameraProgress.msg` for feedback:

```
string  camera
uint32  frames_seen
uint32  frames_detected
uint32  frames_accepted
uint32  frames_duplicate
uint32  frames_unpaired
uint32  budget
float32 progress
```

`CalibrateCamera.action` result additions:

```
CameraCalibration[] cameras
bool   joint_refined
float64 joint_fit_seconds
```

Feedback additions:

```
uint32 cameras_in_goal
CameraProgress[] cameras
```

Compatibility:

- `camera: "left"` keeps every existing scalar result field populated exactly
  as today, and `cameras` has one element mirroring it.
- `camera: "both"` / `"all"`: `cameras` has one element per configured camera,
  ordered by name (`std::map` order: "left" before "right"). The scalar fields
  are zeroed in both mode, success or failure, so a consumer that ignores
  `cameras[]` and reads the old fields cannot install one camera's intrinsics
  as if they described the rig. `success`, `message` and the check scalars
  still describe the overall goal. Document that in the action comment.
- Existing feedback scalar fields mirror the first camera in both mode; the
  per-camera truth is in `cameras[]`.

`scripts/calibrate_client.py` must print the per-camera array and keep exiting
non-zero unless **all** cameras succeeded and every check passed.

### 2.7 Non-blocking execution

Three levels:

1. **Process level.** `discocal_ros_node` is a separate process. It never
   blocks another ROS node; a solve costs CPU, not locks. This was already
   true and should be stated in the README.
2. **Callback level.** Image subscriptions move to a dedicated
   `ros::CallbackQueue` spun by `ros::AsyncSpinner(performance.async_spinner_threads)`.
   The action server stays on the global queue, spun by `ros::spin()` in
   `main`. A slow decode or a saturated image queue can no longer delay a
   goal/preempt callback.
   - Construct `ros::CallbackQueue image_queue_;` and
     `ros::NodeHandle image_nh_;` as members; call
     `image_nh_.setCallbackQueue(&image_queue_)` before constructing the
     `ImageSource`s. `ImageSource` already takes a `NodeHandle&`.
   - Own the `AsyncSpinner` in the node (construct after the subscribers,
     `start()` immediately, `stop()` in the destructor).
3. **CPU level.** During a solve the worker is the only heavy consumer.
   - `performance.nice`: call `setpriority(PRIO_PROCESS, 0, nice)` as the
     **first statement in `workerLoop()`**, not in the constructor --
     `setpriority`/`nice` are per-thread on Linux, and setting it in the
     constructor would nice the main spin thread instead.
   - `performance.detector_threads`: set `OMP_NUM_THREADS` via `setenv` before
     the first detection; additionally call `omp_set_num_threads(n)` from a
     small helper in `CTargetDetector.cpp` (the only TU that includes
     `<omp.h>`) so the limit is deterministic. A static
     `TargetDetector::set_num_threads(int)` is a 3-line addition; expose it and
     call it in the node constructor.
   - `performance.ceres_threads`: `Solver::Options::num_threads` in both
     `CCalibrator` and `StereoCalibrator`.
   - `performance.opencv_threads`: `cv::setNumThreads(n)` in the node
     constructor, before the first decode. OpenCV's pool would otherwise
     compete with OpenMP.
4. **Do less work while solving.** Add an `std::atomic<bool> collecting_`. The
   image callback returns immediately (before decoding) when it is false. The
   worker sets it true around the collection loop and false during
   `RigSession::solve` and while idle. Frames that arrive during a solve belong
   to no collection anyway; the existing code drains the queue before a new
   collection.
5. **Preemption.** A solve cannot be interrupted inside a Ceres call. Check
   `preempt_requested_` between stages (after mono A, after mono B, before
   joint, before writing) so a long both-mode goal can stop between solves.
   Document the granularity in the action comment.

### 2.8 Output and camera_info

- `<output>/left_camera_info.yaml`, `<output>/right_camera_info.yaml` for both
  mode. Existing naming already does this.
- Add `frame_id` to `CameraInfo` and emit a `header` block:
  ```yaml
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: explore3d
  ```
  Use `cameras.<name>.frame_id`; fall back to the camera name when empty.
  (`rig_config.cpp` already parses `frame_id`; `camera_info_writer.cpp` does not
  emit it.)
- Add `write_text_file_atomic`: write `<path>.tmp.<pid>`, flush, `rename`.
  Write both camera_info temp files first, then rename both, so the window in
  which the rig is half-updated is two consecutive renames rather than two
  full writes.
- Optional `rig_calibration.yaml` summary in the output directory. Not part of
  the ROS contract; useful for `axis_grasp`/libCalib conversion later.

### 2.9 Raw transport lifetime bug (fix while here)

`ImageSource::onImage` uses `cv_bridge::toCvShare`, which returns a `cv::Mat`
that does not own the message buffer. `onFrame` stores that `cv::Mat` in a
queue and returns; the message can be freed before the worker decodes it. Use
`cv_bridge::toCvCopy` for the raw path too (a copy per frame is cheap next to
detection). Document the cost in the comment. This is a latent use-after-free
that both-camera work would make easier to hit.

## 3. Implementation phases

Each phase should compile and keep the existing mono tests green before the
next one starts.

### Phase 0 -- Port the calibrator performance work

Files:

- `src/discocal/lib_calib/CCalibrator.cpp` (three hunks, CRLF preserved)

Acceptance:

- Builds on GCC 7.5.
- Offline A/B with `DISC_UNC_DUMP=1` on `calib_eval/left_baseline` (48 frames)
  and `calib_eval/left_all` (275 frames): dense vs `DISC_SPARSE_INFO=1` sigmas
  agree to `< 1e-9` (measured 3.3e-11 / 6.7e-12). The 275-frame yaml is
  byte-identical to the Tier 1 result.
- `DISC_TIME=1` prints the phase table and the phases sum to the printed
  runtime (see the `t_gemm` note -- a reused mark inflates the label).

### Phase 1 -- Config schema

Files:

- `include/discocal_ros/rig_config.h`
- `src/rig_config.cpp`
- `config/rig_example.yaml`
- `config/rig_example_both.yaml` (new)
- `test/test_core.cpp`

Add `DetectorConfig`, `PerformanceConfig`, `SolveConfig`. Leave
`BoardConfig::radius` exactly as it is -- no new size key, no alias. Implement
the ranges and the defaults above. Unknown keys stay ignored.

Acceptance: new parse tests; the shipped example still parses; a both-mode
example parses with both cameras `source: calibrate`.

### Phase 2 -- Session refactor: solve / evaluate / write

Files:

- `include/discocal_ros/calibrate_session.h`
- `src/calibrate_session.cpp`
- `src/node.cpp` (only the mono call sites)

Split `finalize` into:

```cpp
struct SolveOutcome {
    bool ok;
    std::string message;
    ::Params params;
    int n_frames;
    double reprojection_error;
    double fit_seconds;
    std::string raw_output_directory;
    std::vector<std::int64_t> fit_stamps;
    std::vector<std::vector<::Shape> > fit_shapes;
    std::vector<Eigen::Matrix4d> fit_poses;       // board -> camera, input order
    std::vector<HeldOutFrame> holdout_detections; // T filled later
};

SolveOutcome solve(const std::vector<std::int64_t>* forced_holdout, int attempt);
CheckResult evaluate(const SolveOutcome& solved, const ::Params& final_params) const;
bool write_camera_info(const ::Params& params, std::string* path, std::string* error) const;
SessionResult finalize(bool allow_unchecked);  // mono wrapper, behaviour unchanged
```

Also:

- Add `set_other_params(const ::Params&)`. It sets `other_params_`,
  `have_other_params_ = true`, and recomputes `check_configured_`.
- Decouple `T_own_to_other_` from "partner intrinsics known": compute it from
  the extrinsic whenever the extrinsic involves this camera; only the
  *verdict* is gated on partner params. This is what lets both mode construct a
  session before either camera is solved.
- `check_unavailable_reason()` must distinguish "no extrinsic", "not two
  cameras", "extrinsic does not involve this camera", and "partner intrinsics
  not yet known".
- `solve(nullptr, attempt)` reproduces the current holdout selection. A
  non-null `forced_holdout` uses exactly those stamps (intersected with the
  accepted set) and ignores `holdout_fraction`.
- `evaluate` re-solves the held-out poses with `final_params`
  (`Calibrator::update_Es`) and calls `run_cross_camera_check` with
  `other_params_`.
- `finalize` for mono mode: `++attempts_`; `solve(nullptr, attempts_)`;
  `evaluate`; verdict; `write_camera_info`. This must preserve the current
  result fields, messages, retry semantics and output paths.

Acceptance: existing 12 tests pass; the mono end-to-end run on
`calibration_set_big/left` still produces the same calibration and check
numbers (within the run-to-run variation already observed).

### Phase 3 -- Vendor and patch `StereoCalibrator`

Files:

- `src/discocal/include/CStereoCalibrator.h` (new)
- `src/discocal/lib_calib/CStereoCalibrator.cpp` (new)
- `CMakeLists.txt`
- `THIRD_PARTY_NOTICES.md` (list the new vendored file and its patch)

Copy from `/home/tuanpham/beex_work/discocal`. Add
`src/discocal/lib_calib/CStereoCalibrator.cpp` to `discocal_vendor`. Do not add
`CMomentsTracker.cpp` or `CLieAlgebra.cpp` again.

Apply the `fix_extrinsic`, `SPARSE_SCHUR`, `num_threads`, `converged` and
`normalize_Es(all_ext)` guard patches described in §2.5.

Acceptance:

- Builds; no duplicate-symbol link errors.
- A small unit test or a standalone check confirms `all_ext` is bit-identical
  after `calibrate(..., fix_extrinsic=true)`.
- With `fix_extrinsic=false`, the existing upstream behaviour is unchanged.

### Phase 4 -- `RigSession` orchestration

Files:

- `include/discocal_ros/rig_session.h` (new)
- `src/rig_session.cpp` (new, ROS-free -- add to `discocal_core`)
- `CMakeLists.txt`
- `test/test_core.cpp`

API:

```cpp
struct CameraOutcome {
    std::string camera;
    SessionResult result;   // params, check, paths, counts, message
};

struct RigSessionResult {
    bool success;
    std::string message;
    bool joint_refined;
    double joint_fit_seconds;
    std::vector<CameraOutcome> cameras;
};

class RigSession {
public:
    RigSession(const RigConfig& rig, const std::vector<std::string>& cameras,
               int budget, const std::map<std::string, ::Params>& known_intrinsics);
    void set_image_size(const std::string& camera, int width, int height);
    bool on_detection(const BoardFrame& frame);   // routes to both sessions
    bool budget_reached() const;                  // all cameras at budget
    const SessionStats& stats(const std::string& camera) const;
    RigSessionResult solve(bool allow_unchecked);
    void reset();
    int attempts() const;
    const std::vector<std::string>& cameras() const;
};
```

Header hygiene (the package's existing rule): `rig_session.h` must not include
`CStereoCalibrator.h` or `CLieAlgebra.h` -- both do `using namespace std;` at
global scope. Include them only in `rig_session.cpp`. That is why `SolveOutcome`
carries `Eigen::Matrix4d` poses rather than `se3`: the conversion
(`LieAlgebra::to_se3` / `to_so3`) happens inside the .cpp.

`solve()` implements §2.3-2.4. Joint refine is attempted only when
`solve.joint_refine`, `board.type == "circle"`, both mono solves succeeded, and
the paired fit scene count is at least `min_fit_frames()`. Otherwise
`joint_refined = false` and the mono results stand.

Failure/retry semantics: `success` is true only when every camera's check
passed (or `allow_unchecked` and the check did not run). A failure retries the
whole rig, not one camera.

Acceptance:

- A unit test for the common holdout split: no stamp is in both a fit set and
  the other camera's holdout; the split is deterministic from the stamp list.
- A unit test that joint refine can be switched off and returns exactly the
  mono params.
- Real-data verification in Phase 8.

### Phase 5 -- Action and messages

Files:

- `msg/CameraCalibration.msg` (new)
- `msg/CameraProgress.msg` (new)
- `action/CalibrateCamera.action`
- `CMakeLists.txt` (`add_message_files(FILES ...)` before
  `generate_messages`)
- `src/node.cpp` (`fillResult`, `publishFeedback`)
- `scripts/calibrate_client.py`

Implement §2.6. Keep the old scalar fields populated for mono goals. Update the
client to print `cameras[]` and to require all cameras to pass.

Acceptance: `catkin_make` regenerates the action/messages; the mono client
still works; a both-mode goal fills two `CameraCalibration` entries.

### Phase 6 -- Node: both mode and non-blocking execution

Files:

- `src/node.cpp`
- `src/image_source.cpp` (raw `toCvCopy`)
- `include/discocal_ros/image_source.h` (comment only)

Implement:

- `camera == "both" || camera == "all"`: require exactly two configured
  cameras, both `source: calibrate`. If one is `source: file`, abort with a
  message that names it.
- `RigSession` instead of the single `CalibrateSession`.
- `processFrame` routes every decoded frame to `RigSession::on_detection`
  regardless of which camera it came from. Per-camera counters for feedback.
- `knownIntrinsics()`: in both mode pass an empty map (the partner is solved in
  the same goal); call `set_other_params` after each stage.
- The dedicated image `CallbackQueue` + `AsyncSpinner`.
- `collecting_` atomic, `setpriority` in `workerLoop`, `cv::setNumThreads`.
- Env application block for `detector.*`, `performance.*` at the top of the
  constructor, before detectors are created.
- Preempt checks between solve stages.
- Result filling: `cameras[]`, `joint_refined`, `joint_fit_seconds`, and
  `fit_seconds` per camera (the current `SessionResult::fit_seconds` is never
  copied to the result because the action has no field for it; the new message
  fixes that).

Acceptance: mono goal unchanged; both goal collects two cameras, solves,
checks, writes two files; the action server and image topics stay responsive
during a solve; `top` shows the worker's nice value.

### Phase 7 -- camera_info header and atomic writes

Files:

- `include/discocal_ros/camera_info_writer.h`
- `src/camera_info_writer.cpp`
- `test/test_core.cpp`

Add `frame_id` to `CameraInfo`, emit the `header` block, add
`write_text_file_atomic`, and use it from `CalibrateSession`/`RigSession`.
Emit the fallback `frame_id` (camera name) when the config leaves it empty.

Acceptance: the YAML still parses as `sensor_msgs/CameraInfo` (the existing
layout test plus a header assertion); a failed second rename leaves the first
file intact and reports the error.

### Phase 8 -- Verification, docs, defaults

Files:

- `README.md`, `docs/DESIGN.md`, `THIRD_PARTY_NOTICES.md`
- `config/rig_example.yaml`, `config/rig_example_both.yaml`
- `DETECTION_ROBUSTNESS.md` (copy or link the upstream write-up)

Document the new groups, the both-mode flow, the common holdout, the joint
refine caveats (circle only, no joint uncertainty, extrinsic fixed), the
non-blocking model and the CPU knobs. Update the "Known limits" section: the
package is no longer mono-only; the remaining limits are that the extrinsic is
still never estimated, there is no stereo *extrinsic* calibration path, and the
joint refine only supports circle boards.

## 4. Test plan

### Unit (no ROS, no images)

Existing helper in `test/test_core.cpp:87`:
`::Shape shape_at(x, y)` and `detect(proj, T, params, ...)`. Only `x`/`y` are
needed for the cross-camera check. A full synthetic calibration needs the
moments (`Kxx`, `Kxy`, `Kyy`, conic), which the helper does not provide, so do
not promise a synthetic end-to-end solve; test the split/orchestration pieces
directly and rely on real data for the solve.

Add:

- `RigConfig.DetectorKnobs`, `RigConfig.PerformanceDefaults`
- `RigConfig.JointRefineSwitch`
- `RigSession.CommonHoldoutIsDisjointFromBothFits`
- `RigSession.JointRefineOffReturnsMonoParams`
- `StereoCalibrator.FixedExtrinsicIsBitIdentical`
- `CameraInfo.HeaderFrameId`

### Integration / real data

`calibration_set_big` (294 frames per camera, 275/279 detected with the gate
on) is the fixture. Use `publish_image_dir.py` at 1 Hz and
`calibrate_client.py`.

1. **Mono regression (left).** Same config/budget as the last verified run.
   Expect `attempts: 2`, `fx` near 368.36 (node run) or 367.38 (offline
   275-frame reference), cross RMS 2.4-3.0 px, limit 4.0.
2. **Mono regression (right).** Needs a full-set right reference to compare
   against, and the one currently on disk is unusable:
   `calib_eval/right_all/calibration_results/intrinsic_parameters.yaml` is
   0 bytes. That is not a code problem -- `Calibrator::calibrate` opens the file
   with `std::ofstream` at its first statement (vendored `CCalibrator.cpp:594`)
   and writes the values after the solve (around :635), so a completed run
   writes it correctly whether or not it pre-existed. The empty file is just
   the trace of a run that died in between, and the 275-frame set is the size
   that OOMs on the dense path. Regenerate the reference with
   `DISC_SPARSE_INFO=1` (measured ~23 s, 0.16 GB) before doing the comparison,
   then record it. Expect it to land in the 360-362 neighbourhood; the 56-frame
   baseline (`fx 360.046`) and 68-frame stride4 (`fx 361.551`) stay as the
   small-set references.
3. **Both mode, joint off.** `camera: "both"`, `solve.joint_refine: false`.
   Verify each camera's result equals its mono run on the same frames (within
   run-to-run collection variation) and the common holdout is respected.
4. **Both mode, joint on.** Verify `joint_refined: true`, the extrinsic in the
   log is unchanged, both checks pass, and the intrinsics move by a sane
   amount relative to the mono solve (a few tenths of a px; investigate
   anything larger).
5. **Retry.** Force a failure (tight `check.max_rms_px`, e.g. 1.0) and confirm
   both cameras re-collect and `attempts: 2`.
6. **Non-blocking.** During a solve:
   - `rosnode ping` / `rostopic list` respond promptly;
   - a preempt goal is accepted and acted on at the next stage boundary;
   - `top -H` shows the worker's nice value and the thread count bound
     (`detector_threads`, `ceres_threads`, `opencv_threads`).
7. **Preemption.** Send a new goal during both-mode solve; confirm the old goal
   is preempted and the new one starts (the solve in flight finishes first).

## 5. Expected performance

From the measured mono numbers (Tier 2, SSE2, 35 dots/frame):

| Pairs (per camera) | Detection (2 cams) | Mono solve (2 cams) | Joint refine | Total, rough |
| --- | --- | --- | --- | --- |
| 50 | ~8 s | ~9 s | a few s | ~20-25 s |
| 100 | ~16 s | ~13 s | ~5-10 s | ~35-45 s |
| 190 | ~30 s | ~30 s | ~10-20 s | ~70-85 s |

These are estimates; the joint refine time is not measured yet. The point of
the table is to show that 100 pairs is the sensible "quick recalibration"
operating point, not 275. Set `frames.budget: 100` in
`config/rig_example_both.yaml` and comment that coverage beats count: at 96
frames the 1-sigma on `fx` is 0.49 px, at 137 it is 0.21 px, and spread poses
beat contiguous ones by ~10x at equal count.

Do not hard-code a new default in the code. The library default stays 50; the
both example sets 100 and explains it.

## 6. Risks and gotchas (read before editing)

1. **CRLF.** `CCalibrator.cpp` is CRLF in both trees. Preserve it or the diff
   becomes the whole file.
2. **Do not copy the upstream detector.** It uses the OpenCV 4 `Mat_` form and
   breaks the OpenCV 3.2 build. The vendored copy already has the WIP.
3. **Do not port `to_table()`.** The vendored copy dropped `tabulate`.
4. **`normalize_Es(all_ext)` mutates a fixed extrinsic.** Guard it.
5. **`StereoCalibrator` hard-codes `mode = 0`.** Circle boards only.
6. **`StereoCalibrator` has no uncertainty.** Report mono-stage sigmas and say
   so.
7. **Duplicate symbols.** `CMomentsTracker.cpp` and `CLieAlgebra.cpp` are
   compiled into `discocal_vendor` once; add only `CStereoCalibrator.cpp`.
8. **Detection stays single-threaded** because of `TargetDetector.cpp:628`.
   A second detector thread is a separate project: it needs that counter made
   atomic, one detector/session owner per camera, and a shared queue. Do not
   fold it into this work.
9. **Raw transport lifetime.** `toCvShare` frames do not own their buffer; use
   `toCvCopy` before queueing.
10. **`setpriority` is per-thread.** Call it inside `workerLoop`, not the
    constructor.
11. **`OMP_NUM_THREADS` timing.** Set it before the first parallel region, and
    also call `omp_set_num_threads` for determinism.
12. **Action message change is ABI-breaking for clients.** Rebuild the action
    consumers; update the Python client in the same commit.
13. **The common holdout is the correctness-critical part.** If the two
    cameras score frames the other fitted, the check stops being independent.
    Test the split explicitly.
14. **`attempts` and raw output directories.** Both cameras must use the same
    attempt number so `discocal_raw/<camera>/attempt_N` lines up across the
    rig.
15. **A failed joint refine must not silently publish a worse result.** Validate
    finite/positive intrinsics and fall back to mono with a warning; make the
    fallback configurable.

## 7. File change index

New:

```
docs/PLAN_BOTH_CAMERAS.md
include/discocal_ros/rig_session.h
src/rig_session.cpp
src/discocal/include/CStereoCalibrator.h
src/discocal/lib_calib/CStereoCalibrator.cpp
msg/CameraCalibration.msg
msg/CameraProgress.msg
config/rig_example_both.yaml
```

Modified:

```
CMakeLists.txt
package.xml                    (only if message deps need declaring)
action/CalibrateCamera.action
include/discocal_ros/rig_config.h
src/rig_config.cpp
include/discocal_ros/calibrate_session.h
src/calibrate_session.cpp
include/discocal_ros/camera_info_writer.h
src/camera_info_writer.cpp
include/discocal_ros/image_source.h   (comment)
src/image_source.cpp
src/node.cpp
src/discocal/lib_calib/CCalibrator.cpp          (three hunks, CRLF)
src/discocal/include/CTargetDetector.h          (optional set_num_threads)
src/discocal/lib_detect/CTargetDetector.cpp     (optional helper)
scripts/calibrate_client.py
config/rig_example.yaml
README.md
docs/DESIGN.md
THIRD_PARTY_NOTICES.md
test/test_core.cpp
```

## 8. Suggested change sequence

`/home/tuanpham/catkin_ws/src/discocal_ros` is **not currently a git
repository** (the repo at `~/beex_work/discocal` is the upstream checkout, not
this package). Treat the list below as change grouping; if commits are wanted,
initialise a repo or place the package in one first.

1. `perf: port sparse uncertainty accumulation and sigma dump to vendored calibrator`
2. `config: add detector knobs, performance and solve groups`
3. `refactor: split CalibrateSession into solve / evaluate / write`
4. `vendor: add StereoCalibrator with fixed-extrinsic support`
5. `feat: RigSession runs two mono solves plus an optional joint refine`
6. `action: expose per-camera results and progress`
7. `node: both-camera goals, async image queue, CPU budget`
8. `output: camera_info header and atomic writes`
9. `docs: both-camera workflow, performance knobs, verification numbers`

Keep commit 1 independent: it is the only one that touches a vendored library
file with CRLF, and it should be reviewable as a pure port.

## 9. Definition of done

- The package builds on Ubuntu 18.04 / ROS Melodic / GCC 7.5 as C++17.
- Offline A/B: dense vs sparse uncertainty agrees to < 1e-9 in sigmas; the
  275-frame yaml is unchanged.
- `camera: "left"` still behaves exactly as documented in the README.
- `camera: "both"` solves both cameras in one goal, writes both camera_info
  files, and reports a per-camera check verdict.
- Joint refine is on/off configurable; with it on the extrinsic is bit-identical
  to the config and never a free parameter; with it off the result equals the
  two mono solves.
- The common holdout guarantees no scored frame was fitted by either camera.
- Images and action callbacks stay responsive during a solve; CPU use is
  bounded by the `performance` knobs and the worker's nice value.
- At `frames.budget: 100`, a both-camera run completes in well under two
  minutes on the reference host (12 CPUs, 15 GB).
- Tests cover config parsing, the common split, the joint-refine switch,
  extrinsic immutability, and the camera_info header.
- README, DESIGN, THIRD_PARTY_NOTICES and the example configs describe the new
  behaviour and the remaining limits.

## 10. Deferred / explicitly out of scope

- Estimating the extrinsic (stereo calibration of `R, t`). The extrinsic is
  always a config input.
- Joint covariance/uncertainty. The mono-stage sigmas are reported and labelled.
- Square-board joint refinement (`mode 2` in `StereoCalibrator`).
- Parallel detection across cameras (needs the `dbg_call` atomic and a
  per-camera worker/session ownership model).
- A libCalib JSON / `axis_grasp` adapter. `rig_calibration.yaml` is only a
  summary; the conversion is a separate task.
- The unanswered "dwecal" question from an earlier session. Nothing in this
  plan depends on it.
