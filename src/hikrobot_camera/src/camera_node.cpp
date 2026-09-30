#include "hikrobot_camera/camera_node.hpp"
#include "MvCameraControl.h"
#include <iostream>
#include <thread>
#include <chrono>
namespace hikrobot_camera
{

  rcl_interfaces::msg::SetParametersResult CameraNode::OnSetParameters(const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    for (const auto &p : params)
    {

      if (p.get_name() == "exposure_time")
      {
        if (p.as_double() > 9996427 || p.as_double() < 15)
        {
          result.successful = false;
          result.reason = "invalid range";
          return result;
        }
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
        int ret = MV_CC_SetFloatValue(handle_, "ExposureTime", p.as_double());
        if (ret != MV_OK)
        {
          result.successful = false;
          result.reason = "SDK setting fails";
          return result;
        }
      }
      if (p.get_name() == "gain")
      {
        if (p.as_double() >15 || p.as_double() < 0)
        {
          result.successful = false;
          result.reason = "invalid range";
          return result;
        }
        MV_CC_SetEnumValue(handle_, "GainAuto", 0);
        int ret = MV_CC_SetFloatValue(handle_, "Gain", p.as_double());
        if (ret != MV_OK)
        {
          result.successful = false;
          result.reason = "SDK setting fails";
          return result;
        }
      }
      if (p.get_name() == "frame_rate")
      {
        result.successful = false;
        result.reason = "本相机不支持直接设置帧率（无 AcquisitionFrameRate 节点）；"
                        "实际帧率由曝光时间和 ROI 决定";
        return result;
      }
      if (p.get_name() == "pixel_format")
      {
        if (p.as_string() != "BayerRG8")
        {
          result.successful = false;
          result.reason = "invalid range";
          return result;
        }
        int ret = MV_CC_SetEnumValueByString(handle_, "PixelFormat", p.as_string().c_str());
        if (ret != MV_OK)
        {
          result.successful = false;
          result.reason = "SDK setting fails";
          return result;
        }
      }
    }

    result.successful = true;
    return result;
  }
  void CameraNode::ImageCallback(MV_FRAME_OUT *pstFrame, void *pUser, bool)
  {
    CameraNode *self = static_cast<CameraNode *>(pUser);
    self->OnImage(pstFrame);
  }
  void CameraNode::OnImage(MV_FRAME_OUT *pstFrame)
  {
    
    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera";
    msg.width = pstFrame->stFrameInfo.nWidth;
    msg.height = pstFrame->stFrameInfo.nHeight;
    msg.encoding = "bayer_rggb8";
    msg.step = msg.width;
    msg.is_bigendian = 0;
    msg.data.resize(pstFrame->stFrameInfo.nFrameLen);
    memcpy(msg.data.data(), pstFrame->pBufAddr, pstFrame->stFrameInfo.nFrameLen);
    image_->publish(msg); // 发布
  }
  void CameraNode::ExceptionCallback(unsigned int nMsgType, void *pUser)
  {
    CameraNode *self = static_cast<CameraNode *>(pUser);
    self->OnException(nMsgType);
  }
  void CameraNode::OnException(unsigned int nMsgType)
  {
    RCLCPP_INFO(get_logger(), "收到异常 0x%x", nMsgType);
    if (nMsgType == MV_EXCEPTION_DEV_DISCONNECT)
    {
      if (handle_ != NULL)
      {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = NULL;
        RCLCPP_INFO(get_logger(), "旧句柄已清理");
      }

      while (!shutting_down_)
      {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        std::string name_ = get_parameter("image_topic").as_string();
        std::string target_serial = get_parameter("camera_serial").as_string();
        double exp_value = get_parameter("exposure_time").as_double();
        bool found = 0;
        MV_CC_DEVICE_INFO *selected = NULL;
        int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
          RCLCPP_INFO(get_logger(), "MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        }
        if (stDeviceList.nDeviceNum > 0)
        {
          for (int i = 0; i < int(stDeviceList.nDeviceNum); i++)
          {
            RCLCPP_INFO(get_logger(), "[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
              break;
            }
            if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
            {
              RCLCPP_INFO(get_logger(), "读到的序列号: [%s]", pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
              if (0 == strcmp((char *)(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber), target_serial.c_str()))
              {
                RCLCPP_INFO(get_logger(), "Device Model Name: %s\n", pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
                RCLCPP_INFO(get_logger(), "UserDefinedName: %s\n\n", pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
                found = true;
                selected = pDeviceInfo;
              }
            }
            // - Device selection and connection.
          }
          if (found)
          {
            int nRet = MV_CC_CreateHandle(&handle_, selected);
            if (MV_OK != nRet)
            {
              RCLCPP_INFO(get_logger(), "MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            }
            nRet = MV_CC_OpenDevice(handle_);
            if (MV_OK != nRet)
            {
              RCLCPP_INFO(get_logger(), "OpenDevice fail! nRet [%x]\n", nRet);
            }
            // ① 关自动曝光
            MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
            MVCC_ENUMVALUE stAuto = {};
            MV_CC_GetEnumValue(handle_, "ExposureAuto", &stAuto);
            RCLCPP_INFO(get_logger(), "ExposureAuto 现在 = %u（期望 0）", stAuto.nCurValue);
            MV_CC_SetFloatValue(handle_, "ExposureTime", exp_value);
            MVCC_FLOATVALUE stExp = {};
            MV_CC_GetFloatValue(handle_, "ExposureTime", &stExp);
            RCLCPP_INFO(get_logger(), "ExposureTime 现在 = %f（范围 %f ~ %f）",
                        stExp.fCurValue, stExp.fMin, stExp.fMax);
            MV_CC_RegisterExceptionCallBack(handle_, &CameraNode::ExceptionCallback, this);
            nRet = MV_CC_RegisterImageCallBackEx2(handle_, &CameraNode::ImageCallback, this, true);
            if (MV_OK != nRet)
            {
              RCLCPP_ERROR(get_logger(), "注册图像回调失败！nRet [%x]", nRet);
            }
            nRet = MV_CC_StartGrabbing(handle_);
            MVCC_FLOATVALUE stRate = {};
            MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &stRate);
            RCLCPP_INFO(get_logger(), "相机出图帧率 = %f fps", stRate.fCurValue);
            if (MV_OK != nRet)
            {
              RCLCPP_ERROR(get_logger(), "StartGrabbing 失败！nRet [%x]", nRet);
            }
            else
            {
              RCLCPP_INFO(get_logger(), "重连成功！");
              break;
            }
          }
        }
        else
        {
          if (handle_ != NULL)
          {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = NULL;
          }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }
  CameraNode::~CameraNode()
  {
    shutting_down_ = true;
    if (handle_ != NULL)
    {
      MV_CC_StopGrabbing(handle_);
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      handle_ = NULL;
    }
  }

  CameraNode::CameraNode(const rclcpp::NodeOptions &options)
      : Node("hikrobot_camera", options)
  {
    // 单位 µs
    handle_ = NULL;
    this->declare_parameter<double>("exposure_time", 50000);
    this->declare_parameter<double>("gain", 0);
    this->declare_parameter<double>("frame_rate", 24);
    this->declare_parameter<std::string>("pixel_format", "BayerRG8");
    this->declare_parameter<std::string>("image_topic", "image_raw");
    this->declare_parameter<std::string>("camera_serial", "DB1921834");
    std::string name_ = get_parameter("image_topic").as_string();
    std::string target_serial = get_parameter("camera_serial").as_string();
    double exp_value = get_parameter("exposure_time").as_double();
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    bool found = 0;
    MV_CC_DEVICE_INFO *selected = NULL;
    image_ = this->create_publisher<sensor_msgs::msg::Image>(name_, 10);
    // 枚举设备
    // enum device
    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
      RCLCPP_INFO(get_logger(), "MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    }
    if (stDeviceList.nDeviceNum > 0)
    {
      for (int i = 0; i < int(stDeviceList.nDeviceNum); i++)
      {
        RCLCPP_INFO(get_logger(), "[device %d]:\n", i);
        MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo)
        {
          break;
        }
        if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
        {
          RCLCPP_INFO(get_logger(), "读到的序列号: [%s]", pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
          if (0 == strcmp((char *)(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber), target_serial.c_str()))
          {
            RCLCPP_INFO(get_logger(), "Device Model Name: %s\n", pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
            RCLCPP_INFO(get_logger(), "UserDefinedName: %s\n\n", pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            found = true;
            selected = pDeviceInfo;
          }
        }
        // - Device selection and connection.
      }
      if (found)
      {
        int nRet = MV_CC_CreateHandle(&handle_, selected);
        if (MV_OK != nRet)
        {
          RCLCPP_INFO(get_logger(), "MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        }
        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet)
        {
          RCLCPP_INFO(get_logger(), "OpenDevice fail! nRet [%x]\n", nRet);
        }
        // ① 关自动曝光
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
        MVCC_ENUMVALUE stAuto = {};
        MV_CC_GetEnumValue(handle_, "ExposureAuto", &stAuto);
        RCLCPP_INFO(get_logger(), "ExposureAuto 现在 = %u（期望 0）", stAuto.nCurValue);
        MV_CC_SetFloatValue(handle_, "ExposureTime", exp_value);
        MVCC_FLOATVALUE stExp = {};
        MV_CC_GetFloatValue(handle_, "ExposureTime", &stExp);
        RCLCPP_INFO(get_logger(), "ExposureTime 现在 = %f（范围 %f ~ %f）",
                    stExp.fCurValue, stExp.fMin, stExp.fMax);
        MV_CC_RegisterExceptionCallBack(handle_, &CameraNode::ExceptionCallback, this);
        nRet = MV_CC_RegisterImageCallBackEx2(handle_, &CameraNode::ImageCallback, this, true);
        if (MV_OK != nRet)
        {
          RCLCPP_ERROR(get_logger(), "注册图像回调失败！nRet [%x]", nRet);
        }
        MVCC_FLOATVALUE stRate = {};
        MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &stRate);
        RCLCPP_INFO(get_logger(), "相机出图帧率 = %f fps", stRate.fCurValue);
        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet)
        {
          RCLCPP_ERROR(get_logger(), "StartGrabbing 失败！nRet [%x]", nRet);
        }
        else
        {
          RCLCPP_INFO(get_logger(), "开流成功");
        }
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "没找到设备");
    }
    // 形状

    pass_ = add_on_set_parameters_callback(std::bind(&CameraNode::OnSetParameters, this, std::placeholders::_1));

    // - Image acquisition and sensor_msgs/msg/Image publishing.
    // - Camera parameter inspection and updates.
    // - Disconnection recovery and resource cleanup.
  }

} // namespace hikrobot_camera
