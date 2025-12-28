// Copyright 2025 RbSCR
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sl_lidar.h>
#include <signal.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"


using namespace sl;  // NOLINT(*)

class RplidarInfo : public rclcpp::Node
{
public:
  RplidarInfo() : Node("rplidar_info")
  {
    do_work();
  }
//

private:
int do_work()
  {
    init_param();

    RCLCPP_INFO(this->get_logger(), "RPLIDAR SDK Version: %d.%d.%d",
                                    (int)SL_LIDAR_SDK_VERSION_MAJOR,
                                    (int)SL_LIDAR_SDK_VERSION_MINOR,
                                    (int)SL_LIDAR_SDK_VERSION_PATCH);
    //

    drv_ = *createLidarDriver();
    if (nullptr == drv_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct driver");
      return -1;
    }

    IChannel* _channel;
    if(channel_type_ == "tcp") {
        _channel = *createTcpChannel(tcp_ip_, tcp_port_);
    } else if(channel_type_ == "udp") {
      _channel = *createUdpChannel(udp_ip_, udp_port_);
    } else {
      _channel = *createSerialPortChannel(serial_port_, serial_baudrate_);
        }
    if (SL_IS_FAIL((drv_)->connect(_channel))) {
      if(channel_type_ == "tcp"){
        RCLCPP_ERROR(this->get_logger(),
          "Error, cannot connect to the ip addr %s with the tcp port %s.",
          tcp_ip_.c_str(), std::to_string(tcp_port_).c_str());
      } else if(channel_type_ == "udp") {
        RCLCPP_ERROR(this->get_logger(),
          "Error, cannot connect to the ip addr  %s with the udp port %s.",
          udp_ip_.c_str(), std::to_string(udp_port_).c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error, cannot bind to the specified serial port %s.",
                                          serial_port_.c_str());
      }
      delete drv_; drv_ = nullptr;
      return -1;
    }

    // Start retrieving/displaying actual RPLIDAR info
    RCLCPP_INFO(this->get_logger(), "=== RPLIDAR info start ===");
    if (!getRPLIDARDeviceInfo(drv_)) {
      delete drv_; drv_ = nullptr;
      return -1;
    }

    if (!getRPLIDARModelnameDescription(drv_)) {
      delete drv_; drv_ = nullptr;
      return -1;
    }

    if (!checkRPLIDARHealth(drv_)) {
      delete drv_; drv_ = nullptr;
      return -1;
    }

    if (!getRPLIDARScanmodes(drv_)) {
      delete drv_; drv_ = nullptr;
      return -1;
    }

    if (!getRPLIDARTypicalScanmode(drv_)) {
      delete drv_; drv_ = nullptr;
      return -1;
    }

    if (!getRPLIDARMotorInfo (drv_)) {
      delete drv_; drv_ = nullptr;
      return -1;
    }

    // All info has been shown
    if (drv_) {
      delete drv_;
      drv_ = nullptr;
    }

    RCLCPP_INFO(this->get_logger(), "=== RPLIDAR info end ===");

    return 0;
  }

void init_param()
  {
    this->declare_parameter<std::string>("channel_type", "serial");
    this->declare_parameter<std::string>("tcp_ip", "192.168.0.7");
    this->declare_parameter<int>("tcp_port", 20108);
    this->declare_parameter<std::string>("udp_ip", "192.168.11.2");
    this->declare_parameter<int>("udp_port", 8089);
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baudrate", 460800);  // default for C1
    this->declare_parameter<std::string>("scan_mode", std::string());

    this->get_parameter_or<std::string>("channel_type", channel_type_, "serial");
    this->get_parameter_or<std::string>("tcp_ip", tcp_ip_, "192.168.0.7");
    this->get_parameter_or<int>("tcp_port", tcp_port_, 20108);
    this->get_parameter_or<std::string>("udp_ip", udp_ip_, "192.168.11.2");
    this->get_parameter_or<int>("udp_port", udp_port_, 8089);
    this->get_parameter_or<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
    this->get_parameter_or<int>("serial_baudrate", serial_baudrate_, 460800);
    // default baudrate for C1, see respective datasheets for other types
  }

bool getRPLIDARDeviceInfo(ILidarDriver *drv)
  {
    sl_result op_result;
    sl_lidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (SL_IS_FAIL(op_result))
    {
      if (op_result == SL_RESULT_OPERATION_TIMEOUT)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Error, operation time out. SL_RESULT_OPERATION_TIMEOUT! ");
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Error, unexpected error, code: %x", op_result);
      }
      return false;
    }

    // print out the device info
    char sn_str[37] = {'\0'};
    for (int pos = 0; pos < 16; ++pos)
    {
      snprintf(sn_str + (pos * 2), 3, "%02X", devinfo.serialnum[pos]); // NOLINT(*)
      // NOLINT to prevent linting in cpplint:
      //    "If you can, use sizeof(sn_str + (pos * 2)) instead of 3 as the 2nd arg to snprintf."
      // If the suggestion is implemented, it will result in a compile warning suggesting to
      // replace the 'sizeof ...' with a fixed value.
    }
    RCLCPP_INFO(this->get_logger(), "-- Device info --");
    RCLCPP_INFO(this->get_logger(), "serialnumber     : [%s]", sn_str);
    RCLCPP_INFO(this->get_logger(), "firmware version : %d.%02d",
                devinfo.firmware_version >> 8,
                devinfo.firmware_version & 0xFF);
    RCLCPP_INFO(this->get_logger(), "hardware revision: %d", (int)devinfo.hardware_version);
    RCLCPP_INFO(this->get_logger(), "model            : %d", (int)devinfo.model);

    return true;
  }

bool checkRPLIDARHealth(ILidarDriver *drv)
  {
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;
    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result))
    {
      RCLCPP_INFO(this->get_logger(), "-- Health  --");
      RCLCPP_INFO(this->get_logger(), "health status : %d", healthinfo.status);

      switch (healthinfo.status)
      {
      case SL_LIDAR_STATUS_OK:
        RCLCPP_INFO(this->get_logger(), "              : OK.");
        return true;

      case SL_LIDAR_STATUS_WARNING:
        RCLCPP_INFO(this->get_logger(), "             : Warning.");
        return true;

      case SL_LIDAR_STATUS_ERROR:
        RCLCPP_ERROR(this->get_logger(),
                     "Error, RPLidar internal error detected. Please reboot the device to retry.");
        return false;

      default:
        RCLCPP_ERROR(this->get_logger(),
                     "Error, Unknown internal error detected. Please reboot the device to retry.");
        return false;
      }
    } else {
      // TODO(rbscr) refactor to function 'get_op_result_mnememonic'
      std::string op_mnemonic;
      switch (op_result)
      {
      case SL_RESULT_FAIL_BIT:
        op_mnemonic = "FailBit";
        break;
      case SL_RESULT_ALREADY_DONE:
        op_mnemonic = "AlreadyDone";
      break;
        case SL_RESULT_INVALID_DATA:
        op_mnemonic = "InvalidData";
        break;
      case SL_RESULT_OPERATION_FAIL :
        op_mnemonic = "OperationFail";
        break;
      case SL_RESULT_OPERATION_TIMEOUT:
        op_mnemonic = "OperationTimeout";
        break;
      case SL_RESULT_OPERATION_STOP:
        op_mnemonic = "OperationStop";
        break;
      case SL_RESULT_OPERATION_NOT_SUPPORT:
        op_mnemonic = "OperationNotSupport";
        break;
      case SL_RESULT_FORMAT_NOT_SUPPORT :
        op_mnemonic = "FormatNotSupport";
        break;
      case SL_RESULT_INSUFFICIENT_MEMORY:
        op_mnemonic = "InsufficientMemory";
        break;
      default:
        op_mnemonic = "Unknown";
        break;
      }
      RCLCPP_ERROR(this->get_logger(),
          "Error, cannot retrieve RPLidar health code: %x [%s]", op_result, op_mnemonic.c_str());
      return false;
    }
  }


bool getRPLIDARScanmodes(ILidarDriver *drv)
  {
    sl_result op_result;
    std::vector<LidarScanMode> scanmodes;

    op_result = drv->getAllSupportedScanModes(scanmodes);
    if (SL_IS_OK(op_result))
    {
      RCLCPP_INFO(this->get_logger(), "-- Supported scanmodes --");

      int size = scanmodes.size();
      for (int i = 0; i < size ; i++) {
        RCLCPP_INFO(this->get_logger(), "mode id             : %d", scanmodes[i].id);
        RCLCPP_INFO(this->get_logger(), "mode name           : %s", scanmodes[i].scan_mode);
        RCLCPP_INFO(this->get_logger(), "time cost per sample: %f", scanmodes[i].us_per_sample);
        RCLCPP_INFO(this->get_logger(), "maximum distance    : %f", scanmodes[i].max_distance);
        RCLCPP_INFO(this->get_logger(), "answer command code : %d", scanmodes[i].ans_type);
        if (i != (size - 1)) {
          RCLCPP_INFO(this->get_logger(), " - -");
        }
      }

      return true;

    } else {
      // TODO(rbscr) refactor to function 'get_op_result_mnememonic'
      std::string op_mnemonic;
      switch (op_result)
      {
      case SL_RESULT_FAIL_BIT:
        op_mnemonic = "FailBit";
        break;
      case SL_RESULT_ALREADY_DONE:
        op_mnemonic = "AlreadyDone";
      break;
        case SL_RESULT_INVALID_DATA:
        op_mnemonic = "InvalidData";
        break;
      case SL_RESULT_OPERATION_FAIL :
        op_mnemonic = "OperationFail";
        break;
      case SL_RESULT_OPERATION_TIMEOUT:
        op_mnemonic = "OperationTimeout";
        break;
      case SL_RESULT_OPERATION_STOP:
        op_mnemonic = "OperationStop";
        break;
      case SL_RESULT_OPERATION_NOT_SUPPORT:
        op_mnemonic = "OperationNotSupport";
        break;
      case SL_RESULT_FORMAT_NOT_SUPPORT :
        op_mnemonic = "FormatNotSupport";
        break;
      case SL_RESULT_INSUFFICIENT_MEMORY:
        op_mnemonic = "InsufficientMemory";
        break;
      default:
        op_mnemonic = "Unknown";
        break;
      }
      RCLCPP_ERROR(this->get_logger(),
          "Error, cannot retrieve RPLidar all supported scanmodes: %x [%s]",
          op_result, op_mnemonic.c_str());
      return false;
    }
  }

bool getRPLIDARTypicalScanmode(ILidarDriver *drv)
  {
    sl_result op_result;
    sl_u16 scanmode;

    op_result = drv->getTypicalScanMode(scanmode);
    if (SL_IS_OK(op_result))
    {
      RCLCPP_INFO(this->get_logger(), "-- Typical scanmode  --");
      RCLCPP_INFO(this->get_logger(), "mode id: %x (see above for name)", scanmode);

      return true;

    } else {
      // TODO(rbscr) refactor to function 'get_op_result_mnememonic'
      std::string op_mnemonic;
      switch (op_result)
      {
      case SL_RESULT_FAIL_BIT:
        op_mnemonic = "FailBit";
        break;
      case SL_RESULT_ALREADY_DONE:
        op_mnemonic = "AlreadyDone";
      break;
        case SL_RESULT_INVALID_DATA:
        op_mnemonic = "InvalidData";
        break;
      case SL_RESULT_OPERATION_FAIL :
        op_mnemonic = "OperationFail";
        break;
      case SL_RESULT_OPERATION_TIMEOUT:
        op_mnemonic = "OperationTimeout";
        break;
      case SL_RESULT_OPERATION_STOP:
        op_mnemonic = "OperationStop";
        break;
      case SL_RESULT_OPERATION_NOT_SUPPORT:
        op_mnemonic = "OperationNotSupport";
        break;
      case SL_RESULT_FORMAT_NOT_SUPPORT :
        op_mnemonic = "FormatNotSupport";
        break;
      case SL_RESULT_INSUFFICIENT_MEMORY:
        op_mnemonic = "InsufficientMemory";
        break;
      default:
        op_mnemonic = "Unknown";
        break;
      }
      RCLCPP_ERROR(this->get_logger(),
          "Error, cannot retrieve RPLidar typical scanmode: %x [%s]",
          op_result, op_mnemonic.c_str());
      return false;
    }
  }

bool getRPLIDARModelnameDescription(ILidarDriver *drv)
  {
    sl_result op_result;
    std::string description;

    op_result = drv->getModelNameDescriptionString (description);
    if (SL_IS_OK(op_result))
    {
      RCLCPP_INFO(this->get_logger(), "-- Modelname description  --");
      RCLCPP_INFO(this->get_logger(), "description: %s", description.c_str());

      return true;

    } else {
      // TODO(rbscr) refactor to function 'get_op_result_mnememonic'
      std::string op_mnemonic;
      switch (op_result)
      {
      case SL_RESULT_FAIL_BIT:
        op_mnemonic = "FailBit";
        break;
      case SL_RESULT_ALREADY_DONE:
        op_mnemonic = "AlreadyDone";
      break;
        case SL_RESULT_INVALID_DATA:
        op_mnemonic = "InvalidData";
        break;
      case SL_RESULT_OPERATION_FAIL :
        op_mnemonic = "OperationFail";
        break;
      case SL_RESULT_OPERATION_TIMEOUT:
        op_mnemonic = "OperationTimeout";
        break;
      case SL_RESULT_OPERATION_STOP:
        op_mnemonic = "OperationStop";
        break;
      case SL_RESULT_OPERATION_NOT_SUPPORT:
        op_mnemonic = "OperationNotSupport";
        break;
      case SL_RESULT_FORMAT_NOT_SUPPORT :
        op_mnemonic = "FormatNotSupport";
        break;
      case SL_RESULT_INSUFFICIENT_MEMORY:
        op_mnemonic = "InsufficientMemory";
        break;
      default:
        op_mnemonic = "Unknown";
        break;
      }
      RCLCPP_ERROR(this->get_logger(),
          "Error, cannot retrieve RPLidar typical scanmode: %x [%s]",
          op_result, op_mnemonic.c_str());
      return false;
    }
  }

bool getRPLIDARMotorInfo(ILidarDriver *drv)
  {
    sl_result op_result;
    LidarMotorInfo motorinfo;

    op_result = drv->getMotorInfo(motorinfo);
    if (SL_IS_OK(op_result))
    {
      RCLCPP_INFO(this->get_logger(), "-- Motor info  --");
      RCLCPP_INFO(this->get_logger(), "desired speed     : %d", (int)motorinfo.desired_speed);
      RCLCPP_INFO(this->get_logger(), "minimum speed     : %d", (int)motorinfo.min_speed);
      RCLCPP_INFO(this->get_logger(), "maximum speed     : %d", (int)motorinfo.max_speed);
      std::string support_mnemonic;
      switch (motorinfo.motorCtrlSupport)
      {
      case MotorCtrlSupportNone:
        support_mnemonic = "None";
        break;
      case MotorCtrlSupportPwm:
        support_mnemonic = "Pwm";
        break;
      case MotorCtrlSupportRpm:
        support_mnemonic = "Rpm";
        break;
      default:
        support_mnemonic = "Unknown";
        break;
      }
      RCLCPP_INFO(this->get_logger(), "motor ctrl support: %d [%s]",
                                      (int)motorinfo.motorCtrlSupport, support_mnemonic.c_str());
       return true;
    } else {
      // TODO(rbscr) refactor to function 'get_op_result_mnememonic'
      std::string op_mnemonic;
      switch (op_result)
      {
      case SL_RESULT_FAIL_BIT:
        op_mnemonic = "FailBit";
        break;
      case SL_RESULT_ALREADY_DONE:
        op_mnemonic = "AlreadyDone";
      break;
        case SL_RESULT_INVALID_DATA:
        op_mnemonic = "InvalidData";
        break;
      case SL_RESULT_OPERATION_FAIL :
        op_mnemonic = "OperationFail";
        break;
      case SL_RESULT_OPERATION_TIMEOUT:
        op_mnemonic = "OperationTimeout";
        break;
      case SL_RESULT_OPERATION_STOP:
        op_mnemonic = "OperationStop";
        break;
      case SL_RESULT_OPERATION_NOT_SUPPORT:
        op_mnemonic = "OperationNotSupport";
        break;
      case SL_RESULT_FORMAT_NOT_SUPPORT :
        op_mnemonic = "FormatNotSupport";
        break;
      case SL_RESULT_INSUFFICIENT_MEMORY:
        op_mnemonic = "InsufficientMemory";
        break;
      default:
        op_mnemonic = "Unknown";
        break;
      }
      RCLCPP_ERROR(this->get_logger(),
          "Error, cannot retrieve RPLidar motor info: %x [%s]", op_result, op_mnemonic.c_str());
      return false;
    }
  }

  std::string channel_type_;
  std::string tcp_ip_;
  std::string udp_ip_;
  std::string serial_port_;
  int tcp_port_ = 20108;
  int udp_port_ = 8089;
  int serial_baudrate_ = 460800;

  ILidarDriver *drv_ = nullptr;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RplidarInfo>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
