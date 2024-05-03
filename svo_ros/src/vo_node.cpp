// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <string>
#include <utility>

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo_ros/visualizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.hpp>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

namespace svo {

/// SVO Interface
class VoNode : public rclcpp::Node
{
public:
  vk::AbstractCamera* cam_{};
  svo::FrameHandlerMono* vo_{};
  std::shared_ptr<svo::Visualizer> visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  boost::shared_ptr<vk::UserInputThread> user_input_thread_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_remote_key_;
  std::string remote_input_;
  bool quit_{false};
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Subscriber sub_images_;

  VoNode();
  ~VoNode() override;
  void set_config();
  bool post_construction();
  void imgCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void processUserActions();
};

VoNode::VoNode() :
  Node("vo_node")
{
  publish_markers_= declare_parameter("svo/publish_markers", true);
  publish_dense_input_= declare_parameter("svo/publish_dense_input", false);

  // Get initial position and orientation
  auto init_rx = declare_parameter("svo/init_rx", 0.0);
  auto init_ry = declare_parameter("svo/init_ry", 0.0);
  auto init_rz = declare_parameter("svo/init_rz", 0.0);
  auto init_tx = declare_parameter("svo/init_tx", 0.0);
  auto init_ty = declare_parameter("svo/init_ty", 0.0);
  auto init_tz = declare_parameter("svo/init_tz", 0.0);

  // Start visualizer with initial position and orientation
  visualizer_ = std::make_shared<svo::Visualizer>();
  visualizer_->post_construction();
  visualizer_->T_world_from_vision_ = Sophus::SE3d(
      vk::rpy2dcm(Vector3d(init_rx, init_ry, init_rz)),
      Eigen::Vector3d(init_tx, init_ty, init_tz));

  // Set svo config from parameters
  set_config();
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != nullptr)
    user_input_thread_->stop();
}

void VoNode::set_config()
{
  auto param_config = svo::Config(
      declare_parameter("trace_name", "vo_node"),
      declare_parameter("trace_dir", "/tmp"),
      declare_parameter("n_pyr_levels", 3),
      declare_parameter("use_imu", false),
      declare_parameter("core_n_kfs", 3),
      declare_parameter("map_scale", 1.0),
      declare_parameter("grid_size", 30),
      declare_parameter("init_min_disparity", 50.0),
      declare_parameter("init_min_tracked", 50),
      declare_parameter("init_min_inliers", 40),
      declare_parameter("klt_max_level", 4),
      declare_parameter("klt_min_level", 2),
      declare_parameter("reproj_thresh", 2.0),
      declare_parameter("poseoptim_thresh", 2.0),
      declare_parameter("poseoptim_num_iter", 10),
      declare_parameter("structureoptim_max_pts", 20),
      declare_parameter("structureoptim_num_iter", 5),
      declare_parameter("loba_thresh", 2.0),
      declare_parameter("loba_robust_huber_width", 1.0),
      declare_parameter("loba_num_iter", 0),
      declare_parameter("kfselect_mindist", 0.12),
      declare_parameter("triang_min_corner_score", 20.0),
      declare_parameter("triang_half_patch_size", 4),
      declare_parameter("subpix_n_iter", 10),
      declare_parameter("max_n_kfs", 10),
      declare_parameter("img_imu_delay", 0.0),
      declare_parameter("max_fts", 120),
      declare_parameter("quality_min_fts", 50),
      static_cast<int>(declare_parameter("quality_max_drop_fts", 40)));
  svo::Config::setInstance(param_config);
}

bool VoNode::post_construction()
  {
    // Create Camera
    if(!vk::camera_loader::loadFromRosNode(shared_from_this(), cam_) || !cam_) {
      RCLCPP_ERROR(get_logger(), "No camera model");
      return false;
    }

    // subscribe to cam msgs
    std::string cam_topic = declare_parameter("svo/cam_topic", "camera/image_raw");
    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    sub_images_ = image_transport_->subscribe(
        cam_topic, 5,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) { imgCb(msg); });

    // Init VO and start
    vo_ = new svo::FrameHandlerMono(cam_);
    vo_->start();

    // Start user input thread in parallel thread that listens to console keys
    auto user_input = declare_parameter("svo/accept_console_user_input", false);
    if (user_input) {
      user_input_thread_ = boost::make_shared<vk::UserInputThread>();

      // subscribe to remote input
      sub_remote_key_ = create_subscription<std_msgs::msg::String>(
          "svo/remote_key", 5,
          [this](const std_msgs::msg::String::ConstSharedPtr& msg) -> void { remote_input_ = msg->data; });
    }

    return true;
  }

void VoNode::imgCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  cv::Mat img;
  try {
    // This will convert rgb to mono
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
  processUserActions();
  vo_->addImage(img, rclcpp::Time(msg->header.stamp).seconds());
  visualizer_->publishMinimal(img, vo_->lastFrame(), *vo_, rclcpp::Time(msg->header.stamp).seconds());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_->visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_->exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

void VoNode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != nullptr)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("SVO user input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("SVO user input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

} // namespace svo

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto vo_node = std::make_shared<svo::VoNode>();

  if(vo_node->post_construction())
  {
    // start processing callbacks
    while(rclcpp::ok() && !vo_node->quit_)
    {
      rclcpp::spin_some(vo_node);
      // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
    }
  }

  vo_node.reset();
  printf("SVO terminated.\n");
  return 0;
}
