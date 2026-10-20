# Third-party notices

## Vendored: discocal (`src/discocal/`)

The sources under `src/discocal/` are a copy of the **discocal** camera-calibration
library, taken from the working tree of a local checkout of:

- https://github.com/chaehyeonsong/discocal
- upstream revision `500a665` **plus uncommitted local changes** to
  `src/lib_detect/CTargetDetector.cpp` (the detection-robustness work: candidate
  caps, contrast/edge filters, size and distance thresholds, read from the
  `DISC_*` environment variables).

### Licence, and what it means for this package

discocal is licensed **Creative Commons Attribution-NonCommercial-ShareAlike 4.0
International (CC BY-NC-SA 4.0)**, and its README states the work is *protected by
a patent*, directing commercial users to contact chaehyeon@snu.ac.kr.

That has two consequences this package does not try to hide:

1. **Non-commercial only.** This package is for internal research and evaluation.
   Neither it nor anything derived from it may be used commercially without a
   separate licence from the copyright holder.
2. **ShareAlike.** CC BY-NC-SA 4.0 requires derivative works to be distributed
   under the same licence. `discocal_ros` embeds and links the library, so it is
   a derivative work: the package as a whole is bound by CC BY-NC-SA 4.0 and
   **cannot be relicensed as Proprietary**. This is why the package is kept as a
   separate catkin package from `axis_grasp` (which is Proprietary) rather than
   merged into it. See `docs/DESIGN.md`.

### What was removed from the vendored copy

Deliberately excluded, because this package drives the classes directly and does
not need them:

| Excluded | Why |
| --- | --- |
| `include/mono.hpp`, `include/stereo.hpp` | File-based drivers. They define non-`inline` out-of-class functions, so including either from two translation units is a duplicate-symbol link error -- and this node calls `Calibrator`/`TargetDetector` directly instead. |
| `include/CImagehandler.h`, `src/lib_utils/` | Not built upstream either: its `add_subdirectory` is commented out in upstream's root `CMakeLists.txt`, and nothing in the repo references the class. `cv::initUndistortRectifyMap` + `cv::remap` replaces it. |
| `src/main_{mono,stereo,bind}.cpp`, `src/python/`, `pybind11/` | CLI and Python entry points. `pybind11` is also what otherwise forces CMake >= 3.15. |
| `include/tabulate/` | See below. |

### De-vendored: `tabulate`

Upstream vendors the [`tabulate`](https://github.com/p-ranav/tabulate) header
library (MIT) under `include/tabulate/`, 21 headers and roughly 370 KB, and uses
it in exactly one place: `Params::to_table()`, which builds a pretty console
table for `std::cout` in `printParams` and in `Calibrator::calibrate`.

A ROS node has no use for box-drawing console output -- it logs through
`ROS_INFO` -- so `to_table()` and its include were removed and the call sites now
use `Params::to_string()`, which already existed and is tab-separated. This drops
21 third-party headers from the build. Nothing on the calibration path used it.

### Local modifications to the vendored sources

Kept to the minimum needed to build on Ubuntu 18.04 / GCC 7.5 / OpenCV 3.2. Each
is marked in the source with a `discocal_ros:` comment.

1. `include/utils.h` -- `std::filesystem` does not exist on GCC 7.5
   (`fatal error: filesystem: No such file or directory`). Uses
   `filesystem_compat.h` instead, which aliases to `std::filesystem` on GCC 8+ and
   to `std::experimental::filesystem` on GCC 7.
2. `include/utils.h` -- the `tabulate` include and `to_table()` removed (above).
3. `src/lib_detect/CTargetDetector.cpp` -- the two-argument `initializer_list`
   constructor `cv::Mat_<double>({2,3}, {...})` only exists from OpenCV 4.0;
   OpenCV 3.2 has no matching overload. Rewritten as the `<<` comma form, which
   compiles on 3.2 through 4.x. The line is in `translation_blur`, which is dead
   code -- nothing in the repository calls it.
4. `src/lib_calib/CStereoCalibrator.cpp` and
   `src/discocal/include/CStereoCalibrator.h` -- `calibrate()` gained
   `fix_extrinsic` and `converged` parameters. With `fix_extrinsic` the
   extrinsic parameter blocks are marked constant, so a both-camera goal refines
   the intrinsics and board poses while the configured extrinsic stays
   bit-identical. The solver choice is selectable through `DISC_SPARSE_SCHUR`
   and `DISC_CERES_THREADS`, and `normalize_Es(all_ext)` is skipped when the
   extrinsic is fixed, because it would otherwise mutate the given pose.
   `mono.hpp`/`stereo.hpp` remain excluded.
5. `include/utils.h` -- the header now includes the standard headers it uses
   directly (`<algorithm>`, `<array>`, `<cmath>`, `<cstdio>`, `<cstdlib>`,
   `<ctime>`, `<string>`, `<vector>`) and the `sorted_random_sampling()` call is
   qualified as `std::sort`. It previously relied on `<yaml-cpp/yaml.h>` to pull
   in `<algorithm>`: yaml-cpp 0.5 does, other yaml-cpp builds need not, and on
   such a toolchain the unqualified `sort` is undeclared ("'sort' was not
   declared in this scope"). This was latent in the container build and failed
   on the vehicle's ARM workspace.

No change was made to any algorithm, threshold or numeric constant.
