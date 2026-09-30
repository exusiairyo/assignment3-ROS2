#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#include <atomic> 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp" 
#include "MvCameraControl.h"  
namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{

public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode();
  
private:
rcl_interfaces::msg::SetParametersResult OnSetParameters(
    const std::vector<rclcpp::Parameter> & params);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr  pass_;  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
    static void ExceptionCallback(unsigned int nMsgType, void* pUser); 
    static void ImageCallback(MV_FRAME_OUT*, void*, bool);
    void OnImage(MV_FRAME_OUT*);  
    void OnException(unsigned int nMsgType); 
    void* handle_;
     std::atomic<bool> shutting_down_{false}; 
  // TODO(student): Design the interfaces and resource ownership required by
  // your implementation. No SDK handles or camera operations are provided.
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
