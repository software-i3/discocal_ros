# Design

Why this package is shaped the way it is, and what the measurements were that
decided the parts that could have gone either way. `README.md` is the how-to;
this is the why.

---

## 1. Why a separate package from `axis_grasp`

`THIRD_PARTY_NOTICES.md` points here for this, so it comes first.

discocal is **CC BY-NC-SA 4.0**. The ShareAlike clause is the binding one: a
derivative work must be distributed under the same licence. A ROS node that
compiles the discocal sources and links them into a binary is unambiguously a
derivative work, so this package cannot be relicensed as proprietary -- and
`axis_grasp` is proprietary.

Keeping them as two catkin packages rather than one is not tidiness, it is the
mechanism that keeps the licence boundary honest:

- The obligation lands on `discocal_ros` and stops there. `axis_grasp` depends on
  it the way it depends on any third-party package, and its own sources stay
  free of anything derived from discocal.
- Merging them would make the combined package a single derivative work, which
  would put `axis_grasp`'s proprietary code under ShareAlike too.
- The boundary is enforceable by inspection: everything under `src/discocal/` is
  upstream plus the build fixes listed in the notices, and everything else is
  ours.

There is a second, smaller reason. `axis_grasp` builds on the vehicle; this
package wants to be buildable and testable without it, and section 2 makes that
literal.

---

## 2. The core is ROS-free and image-free

The policy lives in classes that have never heard of ROS:

| Class | Owns |
| --- | --- |
| `RigConfig` | Reading and validating the YAML. |
| `DiversityGate` | Deciding whether a pose is new enough to keep. |
| `CrossCameraCheck` | Scoring held-out frames through the fixed extrinsic. |
| `CalibrateSession` | Budget, holdout split, solve, evaluate, retry, verdict for one camera. |
| `RigSession` | Both-camera orchestration: shared holdout, two mono solves, fixed-extrinsic joint refine, per-camera verdicts, artifact layout. |
| `generate_dwecal()` | The `.dwecal` bytes from two intrinsic sets plus the extrinsic. |
| `PerformanceReport` | Stage wall/CPU timing, process CPU, RSS and thread sampling, written as YAML. |
| `CameraInfoWriter` | The `camera_info` YAML. |

`CalibrateSession` and `RigSession` are handed `BoardFrame`s -- detected dot
centroids plus a stamp -- and never a `cv::Mat`. The node decodes images and runs
the detector; nothing below `node.cpp` knows what an image is. The dwecal
generator is similarly ROS-free: it takes plain arrays and returns bytes.

The payoff is `test/test_core.cpp`: 35 tests that exercise config parsing and
validation, the lattice ordering, the gate, the shared holdout, timestamped
directory naming, the collection policy (a short feed above the solver's floor
still calibrates), the `camera_info` layout and atomic writes, joint refine (the
fixed extrinsic stays bit-identical), the dwecal generator (structure, refusals,
golden land input, skew against `initUndistortRectifyMap`), and the check itself
-- including a synthetic rig where both cameras are generated from one known-good
pose, so a correct projector must score ~0 and an inverted extrinsic must score
>10 px. None of that needs a roscore, a camera, or the dataset. The alternative
-- policy tangled into the node -- is testable only by launching a graph and
feeding it images, which is how you end up verifying calibration code by looking
at log lines.

It also keeps detection on one thread, which is not a style choice. See next.

---

## 3. Threading: one worker owns the detector

`CTargetDetector.cpp` carries a `static int dbg_call` that is incremented without
a lock (`discocal_ros` inherits this from upstream). Two threads detecting
concurrently race on it. Rather than patch upstream's debug instrumentation, the
node never gives the detector to more than one thread:

- Image callbacks run on a dedicated `ros::CallbackQueue` driven by its own
  `AsyncSpinner`, decode the message there, and enqueue a `cv::Mat`; they hold a
  mutex for the length of a push, not a detection. Action goal/feedback callbacks
  stay on the global queue, so a slow image decode cannot delay a cancel.
- Exactly one worker thread pops frames and runs the detector. The image spinner
  may have more than one thread (`performance.async_spinner_threads`), but that
  only parallelises decode -- detection stays on the worker.
- `ros::spin()` runs on the main thread; the worker is a `std::thread` started at
  construction.

So a 12-core machine uses one core for detection. That is a real cost -- section 8
quantifies it -- and it is accepted because the alternative is either a data race
on shared state or a change to vendored code beyond the build fixes the notices
promise. If detection throughput ever becomes the limit on a real rig, the
correct fix is to make that counter atomic upstream, not to add a thread here.

### The queue drops the oldest, and that is deliberate

`kMaxQueuedFrames = 30`. At 30 Hz that is about a second of slack. On overflow
the node pops the front.

The alternative -- backpressure, or an unbounded queue -- is worse for what this
node does. Calibration wants *recent, diverse* frames, and it has a duration
budget. An unbounded queue turns a slow detector into a growing backlog of stale
frames that the node then works through long after the goal's deadline, so the
collected set would be a recording of the past rather than a sample of now.
Dropping the oldest keeps the collection live. Dropping frames is also not a
loss: the gate rejects most of them anyway.

When no goal is active the queue is cleared outright rather than accumulated --
queued frames have no consumer, so keeping them only delays the start of the next
collection with frames from before it.

### The pairing consequence

Pairing is by exact header stamp, and a dropped frame can orphan its partner: a
left frame is consumed while the worker is busy, and its right partner sits at
the head of the queue and can be evicted before it is looked at. Measured: 49 of
50 accepted paired on a 1 Hz feed, 25 of 50 at 5 Hz.

That is a *saturated-feed* problem, i.e. a real 30 Hz rig, not a test-harness
artefact. It is currently tolerable because the check only needs a handful of
pairs, but the honest fix is to pair at enqueue -- make the queue hold whole
stamp-matched pairs and drop whole pairs -- so that a frame can never be consumed
without its partner. That is not implemented, because it wants a feed that
actually saturates to be verified against, and inventing one in the harness would
mean pretending to have tested it.

If pairing looks broken, read the log line that the node prints *after*
`finalize()` -- `other camera (right): 143 of 146 decoded frame(s) detected; 49 of
50 accepted frame(s) had a simultaneous frame there`. That second number is the
true count. Do **not** read the *feedback* `unpaired`, which is provisional by
construction: it counts only what was known at the moment each frame was accepted,
and the publisher sends one camera and then the other, so at that moment the
partner has not arrived yet. On the last verified run the feedback said 49
unpaired and the truth was **1**. The gap is not a small bias, it is the whole
number, and treating it as real costs an hour of chasing a bug that does not
exist.

---

## 4. Frame policy: budget plus a diversity gate

Collection stops at `frames.budget` accepted frames, or at
`frames.max_duration_s`, whichever comes first. A goal can override the budget;
zero means "use the config".

Both values are ceilings. Hitting the time limit with fewer accepted frames does
not fail the goal: the solve runs on what was accepted and only refuses below
`CalibrateSession::min_fit_frames()` (6 for this board, `max(5, 4 + n_d)`),
where the calibrator cannot initialize at all. A short collection therefore
yields a smaller, more weakly constrained calibration whose reported sigmas grow
accordingly -- not an error. That failure is not retried either, since a retry
re-collects from zero and cannot manufacture frames the feed did not deliver.

The gate is what makes the word *accepted* mean something. Without it, a board
sitting still in front of a camera fills the budget with 50 near-identical poses
-- a set that constrains the focal length weakly and the distortion coefficients
not at all, while reporting a beautifully small reprojection error. The gate keeps
a frame only if its pose signature is far enough from every signature already
kept (`frames.diversity_gate` turns it on; `frames.diversity_threshold` sets how
far is far enough).

Consequences worth knowing:

- Acceptance is not detection. On the verification dataset a 50-frame budget was
  filled after 101 detections -- roughly half the board-visible frames were
  duplicates, which is what a hand-waved board looks like.
- The gate is checked against the *pose*, not the pixel content, so it does not
  care whether the board moved or the camera did.
- It only records a signature for a frame it accepts. A disabled gate therefore
  reports `accepted() == 0` despite accepting everything; that accessor counts held
  signatures, not frames, and the session tracks its own `stats_.accepted`.

---

## 5. The check: held-out cross-camera RMS

The requirement was to calibrate one camera, use the fixed extrinsic as a
reprojection check, and fall back and recalibrate if the check failed. That is
what this does, and the shape of it comes from one measurement.

### Own-camera reprojection error cannot judge intrinsics

Over the same 40 held-out frame pairs on `calibration_set_big/left`:

| Intrinsic set | Own-camera RMS | Cross-camera RMS |
| --- | --- | --- |
| Upstream reference (fx 367.451) | 0.1424 px | **1.4467 px** |
| Node solve (fx 367.006) | 0.1474 px | 2.8074 px |
| Node solve (fx 371.178) | 0.1583 px | 5.0731 px |

An 11% spread in the self-reported number spans a 3.5x spread in the real one.
The reason is structural: a mono solve has enough freedom to fit its own dots
almost regardless of the focal length, because moving fx and moving the board
pose trade off against each other. Rescaling a scene and changing the focal
length produce nearly the same image. So reprojection error is a *fit quality*
number, and a mis-calibrated camera fits its own frames fine.

The extrinsic is what breaks the tie. It is fixed and independently known, so it
pins the geometry: a wrong fx forces the board to sit at the wrong distance,
which projects into the other camera at the wrong place. Hence a check that
scores a camera by how well it predicts *the other camera's* pixels.

### How the split works

1. Of the accepted frames, the ones the other camera also saw are the candidates
   -- a frame with no partner cannot be checked. `holdout_fraction` of them are
   held out, the rest are fit. The held-out ones are picked at *evenly spaced*
   positions through the candidate list, not from one end, so they span the pose
   range that was collected. Both lists are then sorted by collection order, so
   the solve sees frames in the order they arrived.
2. `Calibrator::update_Es(Params, 0)` solves the board poses for the held-out
   frames **with the intrinsics held constant** -- upstream's `fix_intrinsic`
   primitive, used standalone.
3. Each held-out dot is projected through `T_own_to_other` into the other
   camera's image and compared with that camera's detected centroid.
4. `rms_px` is the pooled RMS over every dot of every held-out pair, against
   `check.max_rms_px`. `worst_pair_rms_px` is the worst single pair, reported
   separately: a value far above `rms_px` means one frame dominates, which is a
   different problem from a uniformly bad calibration.

A detail that matters for reading the numbers: the poses are solved with the
*candidate's own* intrinsics, not the reference's. Solving with the reference and
scoring the candidate would flatter every candidate, since the poses would have
been fitted to a different camera. The harness that produced the table above does
the same, which is why its 1.4467 px row is a real floor and not an artefact of
what it was compared against.

### The both-camera split is shared

A both-camera goal runs the same measurement twice -- once per camera -- but the
holdout is chosen **once** and forced into both sessions. Only stamps that both
cameras accepted are candidates, `holdout_fraction` of them are picked with the
same evenly-spaced rule, and each camera then fits everything else. Without that
sharing, a frame one camera fitted could be scored by the other camera's check,
and the check would no longer be independent of the fit it is judging.

The other difference from mono is *which* intrinsics score it. Each camera is
evaluated against the other camera's **final** parameter set -- after the joint
refine, when it runs -- not the mono result. So the check answers "is this pair
of cameras, as they will be installed, consistent with the fixed extrinsic?",
and a joint solve that moved the intrinsics is judged on where it landed. Both
cameras must pass; a goal succeeds only when both verdicts do.

An accepted frame without a partner is still usable for its own camera's mono
fit, but it cannot enter the shared holdout or the joint scene set. The
unpaired counters in feedback/result report how many such frames were dropped.

### Where the threshold comes from

1.4467 px is what upstream's own reference intrinsics score on this dataset --
the floor for *any* calibration, since nothing is fitted in that row. The nearest
wrong reading of the extrinsic (using `R|t` where the node uses `R^T|-R^T t`)
scores 12.66 px. The usable band is therefore (1.45, 12.66), the node's own good
mono solves land between about 2 and 3 px, and the shipped example uses **4.0**.

Two things follow. First, the `1.0` this example shipped with earlier was
unpassable -- not "strict", but below the floor, so every calibration would have
failed and retried and failed. That is the failure mode a threshold picked by
intuition produces. Second, there is no useful default in `rig_config.h` for the
same reason: the right value is a property of the rig, roughly 3x the RMS its own
trusted calibration scores. The 4.0 default there is a documented guess that at
least sits inside a real band rather than under it.

### On a retry

A failed check reports the measurement and retries: `reset()` drops the collected
frames and the gate's signatures, attempts already spent are kept, and a fresh
collection runs. Verified end to end, twice, with one goal each:

| Run | Attempt | Fit / held out | Reprojection | Cross-camera | Verdict |
| --- | --- | --- | --- | --- | --- |
| 1 | 1 | 40 / 10 | 0.136599 px | 5.27598 px | fail |
| 1 | 2 | 41 / 9 | 0.203695 px | 3.004685 px | **pass** |
| 2 | 1 | 40 / 10 | 0.136599 px | 5.27598 px | fail |
| 2 | 2 | 41 / 9 | 0.223812 px | 2.389125 px | **pass** |

Both runs rejected a solve of **fx 371.178** on the first attempt -- the exact
intrinsic set the table above measures at 5.07 px cross-camera -- and both passed
on the retry. Two things follow.

First, the check is doing what it is for. The rejected solve reported the *better*
reprojection error of the two (0.1366 against 0.2238), which is the whole point of
section 5: the number a mono solve reports about itself does not distinguish these
two, and only the extrinsic does.

Second, an `attempts: 2` result with a first-attempt failure is the *expected*
outcome on this dataset, not a symptom. Note that the two runs agree to every
digit of the first attempt -- same frame counts, same reprojection, same rejected
fx. That is not a coincidence and it is not luck: both started the publisher at
the head of the dataset, detection is deterministic, and the gate keeps the same
frames, so the first collection is the same 50 poses and the solve lands in the
same place. It also means the reproducibility here is a property of the *test
feed*, and a real rig, whose frames are never the same twice, will not behave
this identically -- the first-attempt failure is a fact about this pose set, so
expect real-rig behaviour to differ and judge it by the retry, not by this table.

Do not read the retry as flakiness, and do not "fix" it by loosening the
threshold -- it is the fallback the design calls for, working.

`allow_unchecked` exists for the case where no check is possible at all -- a rig
with one camera, or a config with no extrinsic -- and then the result is written
without a verdict. It is not a way to overrule a failed check.

The result zeroes its numeric fields when the check fails, so a consumer that
ignores `success` cannot install the rejected intrinsics by accident. The rejected
values survive in the log and in `discocal_raw/<camera>/attempt_N/`.

---

## 6. Extrinsics are fixed, never estimated

The extrinsic is read from the config and never estimated. With the both-camera
path this is enforced by the solver rather than by omission: the joint refine
uses the vendored `CStereoCalibrator`, but `calibrate()` takes `fix_extrinsic`,
and the node marks every extrinsic parameter block constant. The configured
rotation and translation therefore come out bit-identical, `test_core.cpp`
asserts that, and the `normalize_Es(all_ext)` call is skipped when fixing so the
solver cannot mutate the given extrinsic as a side effect.

What the joint refine *does* estimate is both intrinsic sets and every board
pose in one problem, with the extrinsic as a rigid link between the cameras.
That is not stereo extrinsic calibration: no parameter of the relative pose is
free. The distinction is the point of the design. The same rig is used
repeatedly, so the extrinsic is a known constant, and estimating it from the
same frames the check then scores would make the check circular -- more
parameters would absorb the residual and the check would pass by construction.
For the same reason the mono path still only *scores* through it.

The joint stage needs the moment/conic board representation, so it runs for
circle boards only. A square board still calibrates both cameras with two mono
solves; the joint stage is skipped and logged, not silently approximated.

### Direction is a trap

`T_own_to_other` maps points from the calibrated camera's frame into the other
camera's. That is a specific direction, and getting it backwards is not a
symmetric error -- it produces the 12.66 px reading.

There are three frames in play and the config only names two. The config's
`extrinsic` is `child` in `parent`'s frame, `p_parent = R * p_child + t`, with no
relationship to which camera is being calibrated. The node derives the transform
it actually needs per goal: if the calibrated camera is the parent, it inverts
(`R^T | -R^T t`); if it is the child, it uses `R | t` directly. So calibrating
`left` and calibrating `right` traverse the same extrinsic in opposite
directions, and only one of them inverts anything.

The fix for a confusing extrinsic is a single scripted run against a known board
and a look at which reading lands near the floor, not a reasoned argument from a
JSON file -- discocal's own `relative_camera_poses.txt` is a third convention
again, and reading its Euler angles as anything but a loose cross-check is how
the sign gets flipped. `test_core.cpp` pins the inverted direction as a >10 px
failure so the convention cannot silently flip.

---

## 7. Output: timestamped run directory, `camera_info`, and `.dwecal`

A both-mode goal writes one directory per calibration,
`<output.directory>/calibration_YYYYmmdd_HHMMSS/`, and appends `_1`, `_2`, ...
when that name already exists. Stable filenames inside make the layout
predictable; the timestamp means a later run cannot overwrite an earlier one,
even if a file name is reused. The suffix makes uniqueness unconditional rather
than dependent on the clock not standing still.

The run summary (`rig_calibration.yaml`) is the human-readable record: both
intrinsic sets, the extrinsic actually used, `camera1_position_in_camera0` and
`baseline_m`, the check verdict and RMS per camera, attempts, and whether the
joint refine ran. `land/` holds the same two intrinsic sets in the format
`dwecal-gen-land` reads, plus `relative_camera_poses.txt` derived from the
config, so the conversion can be reproduced by the standalone tool and audited
without reading the node's log. One caveat: the text format stores the rotation
as a Rodrigues vector in degrees, so a matrix -> vector -> matrix round trip
reconstructs the rotation only to within text precision. The standalone tool
therefore reports the same P0 and baseline but not the same file hash as the
node (about 0.4% of map bytes differed by low-order float bits on the example
run). The node's own `.dwecal` and its side output are both written from the
in-memory parameters; the hash comparison is not a cross-check. `performance.yaml`
is section 8.

### `camera_info`

`distortion_model` is always `plumb_bob`, and `p1 = p2 = k3 = 0` is not an
approximation -- discocal's model is purely radial (`k = 1 + d1*r^2 + d2*r^4` at
`n_d = 2`) with no tangential term anywhere in the library, so there is nothing
else those coefficients could be. Writing them as zeros states the truth rather
than omitting them. `P` is filled consistent with `K` and a zero baseline, which
is the correct *mono* projection; the rectified stereo product is the `.dwecal`,
not `P`.

Both files carry a `header` with `frame_id` from `cameras.<name>.frame_id`
(falling back to the camera name when empty) and a zero seq/stamp, so they paste
into a `CameraInfo` message without editing.

Mono goals keep the original rule: `camera_info` is written only on a passing
check, into `<output_dir>/<camera>_camera_info.yaml`, with no run directory.
Both-mode goals write every artifact even when the check fails -- that run's
`rig_calibration.yaml` records `check_passed: false`, and the result's `success`
is still false. The files exist so a failed calibration can be inspected; they
are not an endorsement.

All writes are atomic: `<path>.tmp.<pid>` -> flush/fsync -> `rename`. The two
camera_info temp files are written first and renamed one after the other, so a
crash cannot leave one camera's new YAML beside the other's old one.

### The `.dwecal`

The generator is a faithful port of `gen_land.cc`: `cv::stereoRectify` with
`alpha = 0` and `CALIB_ZERO_DISPARITY`, `cv::initUndistortRectifyMap`, and
`H.t()` when the maps are interleaved, with the same `ResolutionByte` enum and
30-byte header. The one thing it does differently is take the extrinsic from the
rig config instead of reading `relative_camera_poses.txt`, which means the
direction has to be converted:

```
config:         p_parent = R_config * p_child + t_config
stereoRectify:  X_cam1   = R_land   * X_cam0   + T_land
R_land = R_config^T          T_land = -R_config^T * t_config
```

Camera 1's origin in camera 0's frame comes back as `t_config` (asserted to
1e-12), and `||T_land|| == ||t_config||`, so the baseline is preserved. The
conversion is tested both ways round; feeding the un-inverted pair is rejected
by `require_camera1_right`, because the file has no field that labels the maps
and a swapped rig would silently produce mirrored disparity.

Before returning bytes the generator checks structure and content: byte size
`30 + 2*W*H*8`, magic/version/camera count, the resolution byte against the
requested dimensions, finite and positive rectified focal lengths, finite maps,
and the fraction of map samples that land inside the source image. That last
number is written to `rig_calibration.yaml`; a value far below ~99% means the
requested output resolution and the source do not fit sensibly.

The format stores no baseline and no checksum, and only `k1`/`k2` per camera.
So a run with `n_d > 2` (or non-zero d3/d4) calibrates and writes everything
else but skips the file with an explicit message; the baseline lives in the
extrinsic and in `rig_calibration.yaml`; and `calibration.dwecal.sha256` is a
per-run integrity record, not a cross-version golden. The rectification ROI
search can move the low bits between OpenCV versions, so byte identity is only
promised against the same OpenCV that wrote the file.

---

## 8. Timing, CPU, and the `max_duration_s` window

The detector is the bottleneck on a real feed. One worker thread sustains roughly
1.4-1.9 frames/s on this dataset -- a probe doing two detections and one solve per
frame measured ~0.55 s/frame of wall time.

At a 1 Hz stereo test feed, filling a 50-frame mono budget took **129 s and
154 s** on the two attempts of a single goal. The example's original
`max_duration_s: 120` was under both, so collection was cut off holding 35 of
its 50 frames. Since the budget became a ceiling, that run now solves from those
35 frames instead of aborting -- a config value should not be able to make the
node look broken. It is now 300 (900 for both mode, where the budget and the
window are per attempt and the budget is per camera), which is long enough to
hit the budget on this feed.

On a real 30 Hz rig the same 50 frames arrive in a couple of seconds and the cap
is pure margin. The number to be careful with is the *test* feed's, because a
throttled feed is the slow case, not the fast one.

### The measured both-camera run

The final verification is a 1 Hz replay of `calibration_set_big` (the 294 unique
pairs recorded twice into the bag, so each camera can fill a 100-frame budget;
one replay at 1 Hz) through `benchmark_calibration.sh`, with
`detector_threads: 1` and `ceres_threads: 1`:

| Stage | Wall | Notes |
| --- | --- | --- |
| Goal, end to end | **312.0 s** | client exit 0, first attempt, check passed |
| Collection | 268.4 s | 533 decoded, 499 detected, 200 accepted (100/camera), 285 duplicate, 108 unpaired; 1.86 detections/s |
| Mono solve, left | 11.21 s | sparse-info uncertainty path |
| Mono solve, right | 11.27 s | |
| Joint refine | 19.31 s | extrinsic constant, 90 shared fit scenes |
| Evaluate, left / right | 0.79 / 0.75 s | 10 shared holdout scenes each |
| Write artifacts / dwecal | 0.31 / 0.29 s | |

Process CPU was **74.78 s user + 4.13 s system = 78.9 s**, i.e. 25.3% of one
core and 2.1% of the 12 available. That is a feed-limited number, not a
detector-limited one: the 1 Hz feed delivers 2 messages/s, collection CPU was
25.57 s for 533 decoded frames (about 0.05 s per frame), and most of the 312 s
is wall time spent waiting for the next frame. Peak RSS was
149,564 kB by `getrusage` (120,264 kB by once-a-second `/proc` sampling), with a
peak of 19 threads. `performance.yaml` in the run directory carries the same
breakdown, plus per-stage worker CPU.

A faster replay is not a useful performance number for this package. The bounded
queue drops most of an unthrottled feed, and the content-based duplicate gate
then rejects repeated passes, so rate 0 is slower and no more diverse than
1 Hz; it is a queue-drop test, not a throughput measurement. The 1 Hz replay is
the honest benchmark for a single-worker detector on 2 messages/s.

### OpenMP's thread setting is per-thread

`performance.detector_threads` looked like it was honoured and was not. The node
originally called `omp_set_num_threads(n)` in its constructor, but OpenMP's
`nthreads-var` is an *internal control variable per thread*: setting it on the
main thread does not limit the worker thread that runs detection. With
`detector_threads: 1` the worker still inherited the default team, which on this
12-core host meant the detector's contour loop spun up 12 threads for a region
too small to pay for them. The run used 2700.9 s of CPU (825% of one core) and
detected no more frames.

The fix applies the limit where detection runs: `TargetDetector::set_num_threads()`
on the worker thread. The same calibration then used 78.9 s of CPU -- about 18x
less -- with bit-identical intrinsics and `.dwecal` bytes. The 12-thread default
run is the other side of the A/B: 1415.8 s of CPU (429% of one core) over 330 s
of wall time, again with identical output. That is why the example configs ship
`detector_threads: 1`: more threads here buy nothing and cost most of a machine.
The mono example shows the same effect even more sharply: 2997.0 s of CPU
(945% of one core) at the default against 45.6 s (16.2%) with the fix, for one
budget-50 goal.

---

## 9. Alternatives that were considered and rejected

- **Estimate the stereo extrinsic from the data.** Still rejected. The rig's
  extrinsic is known and constant, and estimating it would remove the independent
  check that is the whole point of this design.
- **Refine both cameras jointly with the extrinsic fixed.** Implemented, as an
  optional both-mode stage. It is not an alternative to the check: the check
  still scores a shared holdout that neither solve fitted. On this dataset it is
  worth having -- `fx` moves 3.0 px (left) and 5.6 px (right) away from the mono
  solutions, and the held-out cross-camera RMS falls from 2.60/2.35 px to
  0.51/0.54 px, with the result closer to the offline reference. The cost is that
  the joint problem computes no covariance, so the reported sigmas remain the
  mono stage's and are flagged as such. Circle boards only, because the stereo
  solver's moment path is the one that shares the mono representation.
- **Trust reprojection error instead of a cross-camera check.** Rejected on the
  measurement in section 5: it cannot separate a good solve from a 5 px-wrong one.
- **Judge the check against the offline reference result, frame by frame.**
  Rejected: it makes the check a fixture comparison rather than a statement about
  the rig, and it cannot run on a rig whose reference is unknown -- which is every
  rig this package will actually calibrate.
- **Multiple detection threads.** Rejected -- section 3. On a 12-core machine the
  speedup would be real; it is not worth a change to vendored code beyond the
  build fixes, or a data race.
- **An unbounded frame queue.** Rejected -- section 3. It converts a slow detector
  into a stale collection.

---

## 10. What has and has not been verified

Verified on 2026-10-18, in the Melodic container (`docker/Dockerfile`: GCC 7.5,
OpenCV 3.2, Ceres 1.13, yaml-cpp 0.5): builds warning-free; all **35 core tests**
pass; and the both-camera path was run end to end from a bag replay of
`calibration_set_big` (the 294 unique pairs recorded twice into one bag,
replayed once at 1 Hz, budget 100 per camera).

The final both-mode run (`calibration_20261018_172411`) passed on the first
attempt:

| | left | right |
| --- | --- | --- |
| fx / fy | 365.132899 / 365.632277 | 361.721924 / 361.326765 |
| cx / cy | 407.879256 / 320.860514 | 413.491718 / 319.369343 |
| skew | -0.889374 | -1.190288 |
| d1 / d2 | -0.17217479 / 0.02431255 | -0.16544979 / 0.02091491 |
| Fit / shared holdout | 90 / 10 | 90 / 10 |
| Reprojection error (fit) | 0.182518 px | 0.140938 px |
| Held-out cross-camera RMS | **0.510165 px** (worst pair 0.664991; 350 dots) | **0.543941 px** (worst pair 0.739644; 350 dots) |

`joint_refined: true`; the extrinsic in `rig_calibration.yaml` is bit-identical
to the config; baseline 0.10018596 m, `camera1_is_right: true`,
`in_bounds_fraction` 0.99994. The `.dwecal` is
30,720,030 bytes, SHA-256
`3e507b258de93ba5a8ac537479eff512b9738adc3eabe2bef4589cd497b0ed3c`, header
UXGA/2 cameras. Both the intrinsic sets and that hash came out identical on
three replays -- once at the OpenMP default, once after the per-thread fix, and
once before the action-server refactor -- which is the reproducibility claim
stated precisely: same bag, deterministic detection, same bytes.

The A/B for the joint stage, same bag and budget:

| Configuration | left fx | right fx | Held-out cross RMS (L / R) |
| --- | --- | --- | --- |
| Joint refine off | 368.160738 | 367.301917 | 2.599381 / 2.352295 px |
| Joint refine on | 365.132899 | 361.721924 | 0.510165 / 0.543941 px |

The mono regression (`config/rig_example.yaml` as shipped, `left`, budget 50)
was re-run after the refactor: exit 0, `attempts: 2`, final `fx 365.849687`,
held-out cross RMS 1.991354 px, reprojection 0.189013 px. That is the same
behaviour as the 2026-10-13 confirmation, so the mono path did not change. At
`detector_threads: 1` the goal used 45.6 s of process CPU over 281 s of wall
time (16.2% of one core), 90,992 kB peak RSS and 20 threads; the 12-thread
default had measured 2997.0 s of CPU over 317 s (945% of one core) for the same
calibration path.

Failure paths were exercised deliberately. With `check.max_rms_px: 0.1` the goal
failed both attempts (RMS 0.272682 / 0.274772), and the run directory still
contained both camera_info YAMLs, `rig_calibration.yaml` with
`check_passed: false`, `performance.yaml`, and a full-size `.dwecal` -- the
"artifacts with the verdict" rule. Preemption was tested by sending a second
both-camera goal while the first was solving (section 11).

Not verified: a live camera, a real 30 Hz feed, `sensor_msgs/Image` rather than
`CompressedImage` (the raw transport is implemented and shares the decode path,
but the verification feed is compressed), and a swapped-camera rig (the refusal
is unit-tested, not run against hardware). The right camera now *has* been
calibrated end to end from a feed (the bag), unlike the 2026-10-13 state where
only `left` had.

---

## 11. Actionlib: per-goal handles, not `SimpleActionServer`

The node originally used `SimpleActionServer`, which is the obvious choice for a
long-running calibration goal and is wrong for one that must be preemptable. A
`SimpleActionServer` tracks exactly one current goal: `acceptNewGoal()` cancels
the running goal as "another goal was received" and immediately makes the new
goal current. The worker thread, still inside the old goal's solve, then calls
`setPreempted()` on the server -- which now refers to the *new* goal -- and
cancels the replacement. The observed failure was a second goal returning exit 1
with an empty result and the first goal's counters, i.e. the newest request was
killed by the one it was supposed to replace.

The fix is `actionlib::ActionServer` with a `ServerGoalHandle` per goal.
`registerGoalCallback` and `registerCancelCallback` receive the handle, so the
node can keep `pending_goal_` and `active_goal_` separately, compare handles by
value, and call `setAccepted`/`setSucceeded`/`setAborted`/`setCanceled` and
`publishFeedback` on the handle the event belongs to. A new goal marks
`preempt_requested_`; the worker checks between solve stages and after the mono
solves (the detector is not interruptible mid-frame), sets the *old* handle
canceled, and starts the new goal. Terminal handles are copied under the mutex
and actionlib is called outside it, so a slow result publication cannot stall
the worker.

Measured: `rosnode ping` answered in **0.246 ms** and `rostopic list` returned
normally while a solve was running. Goal 2 preempted goal 1 at the next stage
boundary, **20.6 s** after the request (21.5 s after the solve began; after both
mono solves, before the joint stage), and then ran to completion and succeeded.

A note on feedback: `publishFeedback` must also go to the active handle, not the
server. A feedback message sent after a preemption would otherwise be attributed
to the replacement goal and report the old goal's counters under the new goal's
budget -- the same class of bug, visible only when two goals overlap.
