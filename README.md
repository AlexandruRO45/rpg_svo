# SVO CPP

Semi Direct Visual Odometry with Python bindings.

## Installation

```bash
pip install svo-cpp
```
<details>
  <summary>TODO (Click me)</summary>


* **Implement Full `MapGraph` Creation**
    * The current `get_map_graph()` in `SVOEngine` is a placeholder that only returns the last frame.
    * **Sub-task:** Add C++ `pybind11` functions to iterate through all active keyframes in the `svo::Map`.
    * **Sub-task:** Convert the C++ keyframe data into a list of Python `MapNode` objects to provide a complete map representation to the `SLAMService`.

* **Resolve Feature Handling Mismatch**
    * The Python `VOEngine` interface provides pre-computed features to `process_frame`, but the C++ SVO library performs its own feature detection and ignores them.
    * **Decision:** Choose a long-term strategy:
        1.  **(Recommended for SVO):** Keep the current implementation and clearly document that SVO handles its own feature detection.
        2.  **(Advanced):** Modify the C++ `FrameHandlerMono` to accept and use external features, bypassing its internal FAST detector. This would allow for experimentation with different feature detectors from Python.

* **Finalize ARM-Specific Parameter Tuning**
    * Methodically test and validate a final `jetson_config` dictionary with optimal parameters for the drone's hardware and expected motion patterns.
    * **Sub-task:** Focus on finding the best balance for `reproj_thresh`, as it's the most critical parameter for tracking stability.
    * **Sub-task:** Tune `init_min_disparity` to ensure reliable initialization in real-world drone startup scenarios (e.g., slow takeoff).

* **Investigate Compiler Flag Impact**
    * The performance difference between x86 and ARM suggests sensitivity to compiler optimizations.
    * **Sub-task:** Compile and test the C++ modules using the `-O2` optimization level instead of `-O3` to see if it improves numerical stability.
    * **Sub-task:** Double-check all `CMakeLists.txt` files to ensure the `-ffast-math` flag (which can reduce precision) is not being used.

* **Integrate IMU Data for VIO**
    * The `SLAMService` is already designed to handle IMU data for visual-inertial odometry. The SVO library also has capabilities for this.
    * **Sub-task:** Create C++ bindings for SVO's IMU processing functions.
    * **Sub-task:** Implement the logic in `SVOEngine` to pass IMU data from the `_on_calib_sync` callback to the C++ backend.

* **Enable Dynamic Feature Filtering**
    * Your application can provide a dynamic mask to filter features on moving objects (e.g., other drones, people).
    * **Sub-task:** Modify the C++ `FrameHandlerMono` to accept an image mask.
    * **Sub-task:** Apply this mask during the internal feature detection step to ignore features in dynamic regions.


* **Improve Relocalization Logic**
    * The logs show the system enters a `RELOCALIZING` state frequently on ARM.
    * **Sub-task:** Expose C++ parameters related to relocalization to the Python `set_svo_config` function.
    * **Sub-task:** Tune these parameters to make relocalization faster and more reliable.

* **Refine State Management**
    * The mapping from the C++ `Stage` enum to the Python `SLAMState` enum is functional but could be more detailed to provide better system health information.
    * **Sub-task:** Provide more granular state updates from the C++ backend to the Python `SVOEngine`.

