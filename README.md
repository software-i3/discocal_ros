# discocal_ros

Calibrate one camera -- or both at once -- from a live ROS image feed, then
prove the result is sane before handing it to anything downstream.

A node subscribes to the camera topics, detects a dot board in the frames that
come past, collects a pose-diverse subset of them, and solves the intrinsics.
The rig's relative pose is a **fixed config input**: nothing here estimates it.
For one camera, the result is checked by predicting what the *other* camera
should have seen in frames the solve never used. For both cameras, the two mono
solves share one holdout, are optionally refined jointly with the extrinsic held
bit-constant, and are checked against each other's **final** intrinsics. Results
come back through an action, and a timestamped run directory receives the
camera_info YAMLs, a `.dwecal` rectification file, and a performance report.

## Licence and relationship to upstream

This package vendors a copy of **discocal** (Dmitry Sorokin, ISP RAS), which is
**CC BY-NC-SA 4.0** -- non-commercial, ShareAlike. That binds this package too:
it is for internal research and evaluation, it cannot be relicensed as
proprietary, and it is deliberately a *separate catkin package* from
`axis_grasp` (which is proprietary) rather than merged into it. See
`THIRD_PARTY_NOTICES.md` for the attribution table, the list of files removed
from the vendored copy, and the local modifications (including the
fixed-extrinsic joint refine), and `docs/DESIGN.md` for the reasoning.

## Requirements

Verified on **Ubuntu 18.04 / ROS Melodic / GCC 7.5**, which is the constrained
target. The package builds as C++17 -- not the upstream C++20 -- because CMake
3.10.2 rejects the value `20` outright.

| Dependency | Why | Install |
| --- | --- | --- |
| `roscpp`, `sensor_msgs`, `cv_bridge`, `actionlib`, `actionlib_msgs` | The node and its interface | ROS Melodic base |
| OpenCV 3.2 | The vendored detector, and the rectification-map generation | ROS Melodic base |
| Eigen3 | The vendored calibrator | ROS Melodic base |
| **Ceres** + **glog** | The vendored solver | `apt install libceres-dev libgoogle-glog-dev` |
| **yaml-cpp** | Rig config, and the vendored `utils.h` includes it | `apt install libyaml-cpp-dev` |

Nothing needs to be vendored by hand beyond the discocal copy already in
`src/discocal/`. There is no Python dependency for the node itself; the test and
benchmark scripts need `cv2`, `rosbag` and `rosbag`'s Python bindings, and are
not needed at runtime.

`docker/Dockerfile` bakes those three apt packages onto the verification image
(`myros:melodic`, a Melodic image that already carries the ROS packages above) --
it is what this package was verified in, and it is short enough to read as the
definitive dependency list. The node and the vehicle want the same three.

On a clean machine, plain `catkin_make` fails at configure, not at link time:
there is no fallback for a missing Ceres (or OpenCV, Eigen or yaml-cpp), and
`find_package(... REQUIRED)` aborts the whole workspace build. Install the table
first -- `rosdep install --from-paths src --ignore-src -r -y` in the workspace
does all of it, because `package.xml` declares every key -- or the three apt
packages by hand. The CMake error now names the apt package rather than sending
you to an installation prefix.

## Build

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Run

The mono example (`config/rig_example.yaml`) calibrates `left` and takes
`right`'s intrinsics from the file, which is the regression this package was
originally verified against:

```bash
roslaunch discocal_ros discocal_ros.launch \
    rig_config:=$(rospack find discocal_ros)/config/rig_example.yaml \
    output_dir:=/tmp/discocal_ros
```

The both-camera example (`config/rig_example_both.yaml`) solves both cameras in
one goal and writes the full artifact set:

```bash
roslaunch discocal_ros discocal_ros.launch \
    rig_config:=$(rospack find discocal_ros)/config/rig_example_both.yaml \
    output_dir:=/output
rosrun discocal_ros calibrate_client.py --camera both --budget 100
```

Both launch arguments go to private node parameters (`~rig_config`,
`~output_dir`). The rig YAML holds everything else: board geometry, frame
policy, detector and CPU knobs, joint-refine policy, the check threshold, the
fixed extrinsic, per-camera topic/transport/source, and the `.dwecal`
resolution. Start from one of the two examples -- their numbers are measured
values for `calibration_set_big`, and every field carries a comment saying where
its value came from.

The external example (`config/rig_example_external.yaml`) does the same but
takes each frame's board pose from a measured per-camera `PoseArray` instead of
estimating it from the image (see the next section):

```bash
roslaunch discocal_ros discocal_ros.launch \
    rig_config:=$(rospack find discocal_ros)/config/rig_example_external.yaml \
    output_dir:=/tmp/discocal_ros
rosrun discocal_ros calibrate_client.py --camera both --pose-source external
```

`calibrate_client.py` is the reference client: it prints live feedback and every
result field, and exits non-zero unless every camera succeeded *and* every check
passed, so it is usable from a build script. `--camera` accepts `left`, `right`
or `both` (`all` is an alias). For an interactive GUI instead:

```bash
rosrun actionlib axclient.py /discocal_ros/calibrate_camera
```

(`axclient.py` is in `actionlib`, not `actionlib_tools`, on Melodic.)

`CalibrateCamera.action` is the full contract. The goal is `camera`, `budget`
(0 = the config value), `allow_unchecked` and `pose_source` (empty = the config
value, `image` or `external`); feedback carries live collection counters, per
camera for a both-mode goal, including how many frames had no usable external
pose; the result carries one `CameraCalibration` entry per camera (intrinsics,
1-sigma uncertainties, reprojection error, check measurement and verdict), plus
the run directory and the `.dwecal` path and hash. In both mode the scalar
result fields are zeroed -- use `cameras[]` -- so a consumer that ignores the
array cannot install one camera's numbers as the rig.

## Calibrating from measured board poses

The default path estimates each frame's board pose from the detected dots while
it solves `K` and `D`. With `solve.pose_source: external`, a second node
publishes the target dots' positions in each camera's optical frame, and this
node fits the board pose to those points rigidly. The reprojection model, the
Ceres problem over `K`/`D`, the uncertainties and the cross-camera check are the
same; only the source of the board pose changes.

This is the mode to use when the target is carried by a calibrated robot arm and
the arm's forward kinematics plus static TFs already give the dot positions. It
does **not** make the calibration automatically better: errors in the measured
positions are absorbed by `K` and `D` exactly as pose errors were before, and
time-synchronization error under continuous motion is a new bias. It removes the
unknown-pose direction from the problem, not the need for accurate geometry.

### The point-pose contract

Each `source: calibrate` camera gets a topic:

```yaml
cameras:
  left:
    point_pose_topic: /arm/calibration_target/left_points
    point_pose_frame_id: left_optical
```

The topic is a `geometry_msgs/PoseArray`:

| Field | Meaning |
| --- | --- |
| `header.stamp` | When the positions are valid. This is matched against the image stamp, not against the message's arrival time. |
| `header.frame_id` | The camera's optical frame (z forward, x right, y down), i.e. the frame `K`/`D` project into. `point_pose_frame_id` checks it; empty disables the check. |
| `poses[i]` | Position of the dot whose **original label** is `i`, in metres. Orientation is ignored. |

The array must have exactly `n_x * n_y` poses. The publisher does not need to
know how the board is mounted, but it must preserve each dot's original
pre-assembly label: flipping or rotating the target permutes labels relative to
the detector's own lattice walk, and the node resolves that per frame.

Because the pose is measured rather than inferred from the pixels, external mode
accepts a smaller board than the image path: 2 dots per side instead of 3. A
single row or column is still refused -- the detector needs a four-corner hull
to order the dots and Kabsch needs three non-collinear points -- so 2x2 is the
smallest usable board. A smaller board means fewer dots per frame, so distortion
(`board.n_d`) is informed by spatial coverage across frames rather than by dots
within one frame; keep the arm moving if you calibrate `d1`/`d2`.

### Labels, flips and matching

The detector returns a rectangular dot lattice in an order it chooses; the
physical board's orientation relative to that walk is one of the lattice's
dihedral symmetries. For every frame the node enumerates those symmetries (4 for
a rectangular grid, 8 for a square one), fits a rigid transform by Kabsch for
each, and scores the candidate by projecting the nominal lattice through the
fitted pose with the library's own conic projector. The candidate is accepted
when:

- the measured cloud is rigid to within `external.max_rigidity_rms_m` (this
  residual is invariant under relabeling, so it validates the measurement with
  no `K`/`D` involved);
- the chosen candidate reprojects below `external.max_match_rms_px`;
- it beats the runner-up by `external.min_match_margin_px`, unless
  `external.ambiguous_policy: use_best` accepts it anyway. Ambiguity means the
  competing pairings project onto nearly the same pixels -- a small, distant or
  edge-on board -- not a face-on one: the labels normally make even a
  fronto-parallel board unique.

`external.temporal_margin_px` keeps the previous frame's permutation when it is
within that margin of the best, so a borderline frame does not flip between
labelings mid-sequence. Frames that fail are counted (`frames_ambiguous`,
`frames_rejected_external`), not silently used.

Matching and solving alternate (EM): match with the current `K`/`D`, solve
`K`/`D` with the fitted poses fixed, rematch, repeat until the permutations stop
changing (`external.max_em_iterations`). `K` is bootstrapped by a linear
3D-to-2D solve from the known positions and the detected pixels
(`use_dlt_initialization`), falling back to the config intrinsics when that
system is degenerate.

### Time synchronization

The arm moves continuously, so an image at stamp `t` must be paired with the
positions valid at `t`, not with the latest array:

- a sample within `external.sync_tolerance_s` is used as-is;
- with `interpolate: true`, two samples bracketing `t` and no more than
  `external.max_gap_s` apart are interpolated per label;
- otherwise the nearest sample inside the tolerance is used;
- with no usable sample, the frame does not enter the solve and is counted in
  feedback as `pose_missing` (or `pose_invalid` for a wrong count/frame/non-finite
  point).

A tolerance much longer than the arm's motion period lets a moving board be
paired with the wrong pose, which biases the result. Tune it to image latency
plus publisher jitter.

### Both-camera mode and the check

Both cameras are still solved in one goal, each from its own PoseArray, and the
shared holdout and fixed extrinsic are unchanged. The cross-camera check is
adapted rather than skipped: each held-out frame is matched independently in
both cameras, both detections are reordered by **original label** into nominal
lattice order, and the existing check projects one camera's pose through the
fixed extrinsic and scores the other camera's pixels. That makes the check
sensitive to a wrong permutation or a mis-synchronized pose as well as to bad
intrinsics.

Joint refinement is skipped with a warning and `joint_refined: false`: the
board poses are measurements, not free parameters shared by the two mono
problems, so there is nothing to refine jointly with the extrinsic fixed. The
run is otherwise identical, and `rig_calibration.yaml` records per-camera
`pose_source`, `frames_matched`, `frames_ambiguous`,
`frames_rejected_external` and `mean_rigidity_rms_m`.

## Configuring for your rig

The rig YAML has these groups: `board`, `detector`, `performance`, `solve`,
`frames`, `check`, `cameras`, `extrinsic`, `output`, `dwecal`. What has to
change for a new rig, in the order it will bite you:

| Group | Change |
| --- | --- |
| `board` | `n_x`/`n_y`, `radius`, `distance` -- physical board measurements, and the one group whose errors look like a broken detector rather than a wrong config. `type` is `circle` (moment/conic path) or `square` (corner path). The image path needs at least 3 per side; external mode needs 2 (see above). |
| `cameras.*` | The real `topic`, `transport` (`compressed` or `raw`) and `frame_id`. |
| `cameras.*.source` | `calibrate` for each camera you are solving; in mono mode, `file` plus an `intrinsics:` block for the other one. Both mode refuses a `file` camera. |
| `solve.pose_source` / `external` | `image` (default) estimates a board pose per frame; `external` fits it to the per-camera PoseArrays and requires `point_pose_topic` on every `calibrate` camera. The `external` group sets sync tolerance, the rigidity gate, matching margins and the EM limit. A goal's `pose_source` overrides the config for that goal. |
| `cameras.*.point_pose_topic` / `point_pose_frame_id` | External mode only. The PoseArray topic, and the optical frame its header must carry (`point_pose_frame_id` falls back to `external.expected_frame_id`; empty disables the check). |
| `extrinsic` | `parent`/`child` and the rotation vector / translation from your rig. Direction matters; see `docs/DESIGN.md` section 6. |
| `check.max_rms_px` | **Must be measured, not guessed.** Run the check once, read the RMS, and set the limit above your rig's floor. `docs/DESIGN.md` section 5 has the method and the numbers for this dataset. |
| `frames.budget` / `frames.max_duration_s` | Both are ceilings: collection ends at the frame count or the time window, whichever comes first, and the solve runs on what was accepted. Fewer frames give a smaller, weaker calibration, not a failed goal -- only a count below the solver's floor (6 for `n_d=2`) is refused. In both mode the budget is **per camera**. |
| `frames.diversity_gate` / `diversity_threshold` | Frame de-duplication. A detection is rejected when its pose signature is within `diversity_threshold` of an already-accepted frame; `frames_duplicate` in the feedback counts the rejections. Raise the threshold to demand more varied poses, lower it (or disable the gate) to wave near-identical frames through. |
| `dwecal` | `output_width`/`height` must be one of 1600x1200 (UXGA), 1280x720 (HD), 800x600 (SVGA), 640x480 (VGA). `require_camera1_right` guards the file's missing left/right label. |

Two values are measured for `calibration_set_big` specifically and should be
expected to move first: `check.max_rms_px` (4.0) and `frames.max_duration_s`
(300 mono, 900 both). Both are documented in place with the measurements behind
them.

The vendored detector's own `DISC_*` tuning knobs and the reason for each are in
`DETECTION_ROBUSTNESS.md` in the upstream checkout. The numeric ones the node
exposes are the `detector:` group; `board.edge_check` is the appearance gate and
is by far the most important (see `rig_example.yaml`).

`performance.detector_threads` deserves a note: keep it at 1. OpenMP's
`nthreads-var` is a per-thread setting, and the node applies the limit on the
detection worker thread. Measured on this dataset, the OpenMP default (12
threads) spent ~9 cores spinning while detecting *fewer* frames than one thread;
the same both-mode calibration used 78.9 s of process CPU with
`detector_threads: 1` against 1415.8 s at the default (429% of one core), for
bit-identical intrinsics and `.dwecal` bytes.

## Output

A **both-mode** goal writes one timestamped directory per calibration. Nothing
in an existing directory is overwritten; a second run in the same second gets a
`_1` suffix.

```
<output_dir>/calibration_YYYYmmdd_HHMMSS/
  left_camera_info.yaml          ROS CameraInfo: header.frame_id, K/D/R/P
  right_camera_info.yaml
  rig_calibration.yaml           both intrinsics, the extrinsic used, check
                                 verdicts, attempts, baseline, camera1_is_right
  land/
    camera0_intrinsic.yaml       land/dwecal-gen-format intrinsics
    camera1_intrinsic.yaml
    relative_camera_poses.txt    R_land/T_land (degrees + metres), derived
  calibration.dwecal             1600x1200 UXGA, 30,720,030 bytes
  calibration.dwecal.sha256      "<hex>  calibration.dwecal"
  performance.yaml               timing, CPU, memory and host breakdown
  discocal_raw/{left,right}/attempt_N/   the library's own output per attempt
```

Every file is written atomically (`tmp` -> `fsync` -> `rename`), and both
camera_info files are renamed together only after both temp files exist.

The artifacts are written **even when the check fails**: `rig_calibration.yaml`
records `check_passed: false`, the verdict and the RMS, and both camera_info
files and the `.dwecal` are still produced so a failed run can be inspected.
They are skipped only when a solve produced no finite, positive intrinsics.
The action result's `success` remains false in that case -- the files are for
diagnosis, not for installation.

A **mono** goal keeps the old layout: `<output_dir>/<camera>_camera_info.yaml`,
written only when the check passes, and `discocal_raw/` directly under
`<output_dir>`; `run_directory` and the dwecal fields are empty. On a failed
check the result's numeric fields are zeroed rather than filled with the numbers
the check just rejected -- a consumer that ignores `success` must not be able to
install them by accident. The rejected values are in the log and in
`discocal_raw/`.

`distortion_model` is always `plumb_bob` and `p1 = p2 = k3 = 0`: discocal's model
is purely radial (`k = 1 + d1*r^2 + d2*r^4` for `n_d = 2`) with no tangential
term anywhere.

### The `.dwecal` file

The two cameras' final intrinsics plus the configured extrinsic are passed
through the same `cv::stereoRectify` pipeline as `dwecal-gen-land`:
`alpha = 0`, `CALIB_ZERO_DISPARITY`, and the `H.t()` map convention. Before
writing, the generator checks the byte size, magic/version/camera count, the
resolution enum against the requested dimensions, finite and positive focal
lengths, finite maps, and the fraction of map samples inside the source image.

The format itself stores **no baseline and no checksum**, and only two radial
coefficients per camera. So:

- a rig whose `n_d > 2` (or non-zero d3/d4) calibrates normally and skips the
  `.dwecal` with an explicit message;
- the baseline is reported in `rig_calibration.yaml` but must be taken from the
  extrinsic, not read back from the file;
- `calibration.dwecal.sha256` is a per-run integrity record, not a cross-version
  golden: the rectification ROI search can move the last bits between OpenCV
  versions, so the bytes are only reproducible with the same OpenCV that wrote
  them.

`require_camera1_right` refuses to write when camera1 is not physically to the
right of camera0 (positive x of its origin in camera0's frame): the file has no
field saying which map is which, so a swapped rig would silently produce
mirrored disparity.

The `land/` side output is there so the conversion is auditable and
reproducible: `rosrun discocal_ros dwecal_gen_cli <run>/land <out>.dwecal`
regenerates the file from the same two intrinsic sets and derived extrinsic, and
reports the same rectified P0 and baseline. It is not byte-identical to the
node's file: the text calibration stores the rotation as a Rodrigues vector
rounded in degrees, so the matrix it reconstructs differs in the last bits and
the rectification maps differ in their low float bits (0.4% of bytes on the
example run, with an identical header and P0). Compare structure and P0, not the
hash, when using the standalone tool as a cross-check.

## Verifying a rig

The package's own verification uses the dataset's images as a fake stereo feed.
Start the node, then in two more terminals:

```bash
rosrun discocal_ros publish_image_dir.py --dir calibration_set_big --rate 1.0
rosrun discocal_ros calibrate_client.py --camera left
```

Throttle it. `--rate` defaults to 0.0, meaning publish as fast as the disk
allows, which is *not* what you want: the node's detector runs at ~1.5 frames/s
and its queue holds 30, so an unthrottled feed is mostly evicted, and eviction is
what orphans a frame's partner (see the pairing note under Known limits). At
1 Hz the queue stays near-empty and pairing is essentially complete.

Both cameras of one index are published with the same header stamp, because that
is what the node pairs on -- an exact match. `publish_image_dir.py` loops until
killed, which is deliberate: one pass through a 294-frame set at 1 Hz is 294 s,
and a goal that fails its check and retries needs two collections, so a single
pass can run dry mid-retry and look like a node bug.

Set `frames.max_duration_s` to how long you are willing to wait for the budget.
The shipped example's 300 s is sized for 1 Hz, where the collection genuinely
takes minutes (see `docs/DESIGN.md` section 8). If the window ends first the goal
still solves from what it has -- a short feed is not a failure -- so the value
trades collection time against how few frames you accept. On a real 30 Hz rig it
is pure margin.

What a good mono run looks like, on `calibration_set_big/left` against the fixed
extrinsic in `config/rig_example.yaml`: `fx` lands within ~2 px of the offline
reference intrinsics (recent runs: 365.85 and 367.07 against 367.451), and the
held-out cross-camera RMS comes in at 2.0-3.0 px -- above the 1.4467 px floor
that the reference intrinsics themselves score, and well under the 4.0 px limit.
Expect `attempts: 2`: the first collection's solve is typically an `fx`-371-class
set that the check exists to reject (the exact frames vary with the feed).

For both cameras, the final verification run (budget 100 each, 1 Hz bag replay)
passed on the first attempt with cross-camera RMS **0.510 px left / 0.544 px
right**, and the intrinsic sets and `.dwecal` bytes reproduced exactly across
runs. `docs/DESIGN.md` section 10 has the full measured table.

### Benchmarking from a bag

`scripts/make_calibration_bag.py` turns a directory of paired `<index>_left.ppm`
/ `<index>_right.ppm` frames into a compressed ROS bag with one shared stamp per
pair. `scripts/benchmark_calibration.sh` then starts a private roscore, node and
action client, replays the bag only after the client reports `COLLECTING`, and
samples `/proc` for CPU, RSS and thread count:

```bash
python scripts/make_calibration_bag.py \
    --dir /data/calibration_set_big --out /output/calibration_set_big.bag \
    --repeats 2
bash scripts/benchmark_calibration.sh \
    --bag /output/calibration_set_big.bag --rate 1 --budget 100
```

Both scripts are Python 2.7 / bash compatible (Melodic). The report and logs go
to `--report-dir` (default `<output-dir>/benchmark_<timestamp>/`); the run
directory the node names is printed at the end. `--rate 0` replays as fast as
possible, which is useful only for measuring queue-drop behaviour -- the
bounded queue and the content-based duplicate gate make it slower and no more
thorough than 1 Hz for this dataset (see Known limits).

## Tests

```bash
catkin_make run_tests_discocal_ros
```

56 tests over the ROS-free core, in two suites. `discocal_core_tests` (35
tests) covers config parsing and validation for both examples, the board
lattice ordering (which the cross-camera check depends on), the diversity gate,
the shared holdout, timestamped directory uniqueness, the camera_info layout
and atomic writes, joint refine (fixed extrinsic bit-identical, fallback,
joint-off equals mono), the `.dwecal` generator (size/header/skew/refusals/
golden land input), the extrinsic direction and baseline, and the cross-camera
check itself against synthetic frames built from a known-good model -- including
that a wrong extrinsic *direction* is caught -- and the collection policy: a
short feed above the solver's floor still calibrates, one below it fails with
the count and the floor in the message. `discocal_external_tests` (21 tests)
covers the external path: the lattice symmetries (4 for a rectangle, 8 for a
square), the Kabsch fit and its count mismatch, matching a flipped rectangular
and a rotated square board, rejection of a non-rigid point cloud, the
fronto-parallel square resolved by its labels, a 2x2 board matched by label, the
pose buffer (exact, interpolated, nearest on a gap, tolerance, replacement),
external config parsing including the shipped example, the mode-dependent grid
floor (2 per side accepted for external, 1 per side and the image path's 3
enforced), and the known-pose solve recovering synthetic `K`/`D` from measured
point positions. No camera, dataset or ROS graph needed.

GTest is resolved by `catkin_add_gtest`: it uses an installed copy, or builds
the sources `libgtest-dev` ships under `/usr/src/googletest` when there is no
archive (the stock 18.04 case), and only warns if neither exists. Each suite
supplies its own `main()` because catkin links gtest but not `gtest_main`.
(On Melodic, `catkin_test_results` reports twice the real count for gtest XML:
it adds the root `<testsuites tests=...>` total to the per-suite totals. The
gtest output above the summary is the authoritative count.)

## Known limits

- **No extrinsic estimation.** Both-camera mode calibrates two intrinsic sets
  against a fixed, configured relative pose. There is no stereo *extrinsic*
  calibration, and the joint refine holds the extrinsic bit-constant rather than
  adjusting it.
- **Joint refine is circle-only, and has no joint covariance.** Square boards
  calibrate both cameras independently with the joint stage skipped and logged.
  After a joint solve the reported sigmas are the mono stage's, flagged with
  `uncertainties_from_mono_stage: true`; they do not describe the joint problem.
- **Joint refine moves the intrinsics more than a little on this dataset.**
  `fx` moves 3.0/5.6 px relative to the mono solves, but the cross-camera RMS
  improves (0.51/0.54 px, against 2.60/2.35 with joint off) and the result moves
  closer to the offline reference. That is the intended trade, not a regression;
  inspect `rig_calibration.yaml` if your own rig moves further.
- **External mode inherits its accuracy from the measured positions.** The
  solver treats every dot position as exact. Arm forward-kinematics error, a
  static-TF error or a target-mounting offset is *rigid*, so it leaves no
  residual signature of its own and is absorbed by `K`/`D` instead. The
  rigidity gate catches a non-rigid or mislabeled cloud, not a rigidly wrong
  one. Time-sync error under motion is the other first-order bias; see the
  external section above.
- **Joint refinement does not run in external mode.** With the board poses
  measured there are no shared free pose parameters, so the node skips the
  stage, reports `joint_refined: false` and logs a warning. The two mono
  results are the answer.
- **External mode has not been run against a live arm publisher.** It is
  covered by synthetic unit tests (matching, flips, pose buffer, known-pose
  solve, config) and shares the cross-camera check with the image path, but the
  real publisher's latency, jitter and label convention are unverified.
- **No visualization.** Nothing here needs a display or OpenCV's highgui.
- **Detector throughput, not feed rate, sets collection time.** One worker thread
  sustains roughly 1.4-1.9 frames/s on this dataset, so a 50-accepted-frame mono
  budget takes minutes at a throttled test feed even though a real 30 Hz rig
  delivers those frames in seconds. If `frames.max_duration_s` cuts collection
  short, the solve still runs on what arrived (down to the solver's floor of 6),
  so the cost of too small a window is a weaker calibration, not a failed goal.
  In both mode the budget is per camera, so the collection roughly doubles.
- **A saturated feed can orphan a frame's partner.** The worker's bounded queue
  evicts the oldest frame when full, which can drop the other camera's frame for a
  stamp whose first half was already accepted. Measured at 49/50 paired on a 1 Hz
  feed, 25/50 at 5 Hz. Harmless while the check only needs a handful of pairs, but
  it is why the pairing counters are worth watching on a real rig.
- **The check's threshold is a per-rig number.** There is no meaningful default;
  see `docs/DESIGN.md`.
- **Mono mode keeps the legacy output path.** A mono goal writes
  `<output_dir>/<camera>_camera_info.yaml` without a timestamped directory and
  writes no `.dwecal`; only a both-mode goal produces the timestamped artifact
  set.
