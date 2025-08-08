#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp> 

// SVO Headers
#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame.h>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <sophus/se3.hpp>
#include <svo/NDArrayConverter.h>

namespace py = pybind11;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);

// Custom type caster to handle the conversion between
// cv::Mat and numpy arrays.
// namespace pybind11 { namespace detail {
// template <> struct type_caster<cv::Mat> {
//     public:
//         PYBIND11_TYPE_CASTER(cv::Mat, _("numpy.ndarray"));

//         bool load(handle src, bool) {
//             if (!py::isinstance<py::array>(src))
//                 return false;

//             auto buf = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>::ensure(src);
//             if (!buf)
//                 return false;
            
//             // Create a temporary Mat header that points to the numpy array's data
//             cv::Mat temp_mat(buf.shape(0), buf.shape(1), CV_8U, (void*)buf.data());
//             value = temp_mat.clone();
//             return true;
//         }

//         static handle cast(const cv::Mat &m, return_value_policy, handle defval) {
//              return py::array(py::buffer_info(
//                 m.data,
//                 sizeof(unsigned char),
//                 py::format_descriptor<unsigned char>::format(),
//                 2,
//                 { (size_t) m.rows, (size_t) m.cols },
//                 { (size_t) m.step[0], (size_t) m.step[1] }
//              )).release();
//         }
// };
// }} // namespace pybind11::detail


// Main function to define the Python module
PYBIND11_MODULE(svo_cpp, m) {
    m.doc() = "Python bindings for the SVO (Semi-direct Visual Odometry) library";

    // =================================================================================
    // 1. Bind Core Data Types (Pose, Camera)
    // =================================================================================

    NDArrayConverter::init_numpy();
    py::class_<Sophus::SE3d>(m, "SE3d")
        .def(py::init<>())
        .def("translation", static_cast<const Eigen::Vector3d& (Sophus::SE3d::*)() const>(&Sophus::SE3d::translation))
        .def("unit_quaternion", &Sophus::SE3d::unit_quaternion)
        .def("inverse", &Sophus::SE3d::inverse)
        .def("__repr__", [](const Sophus::SE3d &a) {
            std::stringstream ss;
            ss << a.translation().transpose();
            return "<svo_cpp.SE3d t: " + ss.str() + ">";
        });
    
    py::class_<vk::AbstractCamera>(m, "AbstractCamera");
    py::class_<vk::PinholeCamera, vk::AbstractCamera>(m, "PinholeCamera")
        .def(py::init<double, double, double, double, double, double>(),
             py::arg("width"), py::arg("height"), py::arg("fx"), py::arg("fy"),
             py::arg("cx"), py::arg("cy"));

    // =================================================================================
    // 2. Bind Configuration
    // =================================================================================
    
    // Expose a function to set SVO's global config singleton from a Python dict
    m.def("set_svo_config", [](const py::dict& config_dict) {
        if (config_dict.contains("n_pyr_levels")) {
            svo::Config::nPyrLevels() = config_dict["n_pyr_levels"].cast<size_t>();
        }
        if (config_dict.contains("kfselect_mindist")) {
            svo::Config::kfSelectMinDist() = config_dict["kfselect_mindist"].cast<double>();
        }
        if (config_dict.contains("reproj_thresh")) {
            svo::Config::reprojThresh() = config_dict["reproj_thresh"].cast<double>();
        }
        if (config_dict.contains("max_fts")) {
            svo::Config::maxFts() = config_dict["max_fts"].cast<size_t>();
        }
    }, "Set SVO parameters from a Python dictionary.");

    // =================================================================================
    // 3. Bind Core SVO Classes (Frame, FrameHandler)
    // =================================================================================

    py::class_<svo::Frame, boost::shared_ptr<svo::Frame>>(m, "Frame")
        .def_readonly("id_", &svo::Frame::id_)
        .def_readonly("timestamp_", &svo::Frame::timestamp_)
        .def_property_readonly("T_f_w", [](const svo::Frame& f) { return f.T_f_w_; });

    py::enum_<svo::FrameHandlerBase::Stage>(m, "Stage")
        .value("PAUSED", svo::FrameHandlerBase::Stage::STAGE_PAUSED)
        .value("FIRST_FRAME", svo::FrameHandlerBase::Stage::STAGE_FIRST_FRAME)
        .value("SECOND_FRAME", svo::FrameHandlerBase::Stage::STAGE_SECOND_FRAME)
        .value("DEFAULT_FRAME", svo::FrameHandlerBase::Stage::STAGE_DEFAULT_FRAME)
        .value("RELOCALIZING", svo::FrameHandlerBase::Stage::STAGE_RELOCALIZING)
        .export_values();

    // This is the main class we will interact with from Python
    py::class_<svo::FrameHandlerMono>(m, "SVO")
        .def(py::init<vk::AbstractCamera*>(), "Constructor takes a camera model.")
        .def("start", &svo::FrameHandlerMono::start, "Start the VO pipeline.")
        .def("reset", &svo::FrameHandlerMono::reset, "Reset the system to its initial state.")
        .def("addImage", &svo::FrameHandlerMono::addImage,
             "Process a new image. Takes a NumPy array (uint8) and a timestamp (float).",
             py::arg("image"), py::arg("timestamp"))
        .def("lastFrame", &svo::FrameHandlerMono::lastFrame,
             "Get the last processed frame object.",
             py::return_value_policy::reference) // Return a reference, not a copy
        .def("stage", &svo::FrameHandlerMono::stage, "Get the current stage of the SLAM pipeline.")
        .def("lastProcessingTime", &svo::FrameHandlerMono::lastProcessingTime, "Get processing time for the last frame in seconds.")
        .def("lastNumObservations", &svo::FrameHandlerMono::lastNumObservations, "Get the number of tracked features in the last frame.");
}