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

`calibrate_client.py` is the reference client: it prints live feedback and every
result field, and exits non-zero unless every camera succeeded *and* every check
passed, so it is usable from a build script. `--camera` accepts `left`, `right`
or `both` (`all` is an alias). For an interactive GUI instead:

```bash
rosrun actionlib axclient.py /discocal_ros/calibrate_camera
```

(`axclient.py` is in `actionlib`, not `actionlib_tools`, on Melodic.)

`CalibrateCamera.action` is the full contract. The goal is `camera`, `budget`
(0 = the config value) and `allow_unchecked`; feedback carries live collection
counters, per camera for a both-mode goal; the result carries one
`CameraCalibration` entry per camera (intrinsics, 1-sigma uncertainties,
reprojection error, check measurement and verdict), plus the run directory and
the `.dwecal` path and hash. In both mode the scalar result fields are zeroed --
use `cameras[]` -- so a consumer that ignores the array cannot install one
camera's numbers as the rig.

## Configuring for your rig

The rig YAML has these groups: `board`, `detector`, `performance`, `solve`,
`frames`, `check`, `cameras`, `extrinsic`, `output`, `dwecal`. What has to
change for a new rig, in the order it will bite you:

| Group | Change |
| --- | --- |
| `board` | `n_x`/`n_y`, `radius`, `distance` -- physical board measurements, and the one group whose errors look like a broken detector rather than a wrong config. `type` is `circle` (moment/conic path) or `square` (corner path). |
| `cameras.*` | The real `topic`, `transport` (`compressed` or `raw`) and `frame_id`. |
| `cameras.*.source` | `calibrate` for each camera you are solving; in mono mode, `file` plus an `intrinsics:` block for the other one. Both mode refuses a `file` camera. |
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

35 tests over the ROS-free core: config parsing and validation for both examples,
the board lattice ordering (which the cross-camera check depends on), the
diversity gate, the shared holdout, timestamped directory uniqueness, the
camera_info layout and atomic writes, joint refine (fixed extrinsic bit-identical,
fallback, joint-off equals mono), the `.dwecal` generator (size/header/skew/
refusals/golden land input), the extrinsic direction and baseline, and the
cross-camera check itself against synthetic frames built from a known-good model
-- including that a wrong extrinsic *direction* is caught -- and the collection
policy: a short feed above the solver's floor still calibrates, one below it
fails with the count and the floor in the message. No camera, dataset or
ROS graph needed. GTest has to be findable; on 18.04 `libgtest-dev` ships
headers without an archive, so `FindGTest` succeeds only after building
`/usr/src/googletest` and copying `libgtest*.a` into
`/usr/lib/x86_64-linux-gnu`.

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
