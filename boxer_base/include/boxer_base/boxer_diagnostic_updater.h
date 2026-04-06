/**
Software License Agreement (BSD)

\file      boxer_diagnostic_updater.h
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
           Mike O'Driscoll <modriscoll@clearpath.ai>
\copyright Copyright (c) 2014, Rockwell Automation Technologies, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Rockwell Automation Technologies nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef BOXER_BASE_BOXER_DIAGNOSTIC_UPDATER_H
#define BOXER_BASE_BOXER_DIAGNOSTIC_UPDATER_H

#include <string>

#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "boxer_msgs/Status.h"
#include "boxer_msgs/TractionStatus.h"
#include "boxer_msgs/MotorFeedback.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"

namespace boxer_base
{

class BoxerDiagnosticUpdater : private diagnostic_updater::Updater
{
public:
  BoxerDiagnosticUpdater();

  void generalLspcuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void generalTdcuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void batteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void voltageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void currentDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void leftMotorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void rightMotorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void liftMotorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void safetyLoopDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void eStopDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void sonarDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  void lspcuStatusCallback(const boxer_msgs::Status::ConstPtr& status);
  void tdcuStatusCallback(const boxer_msgs::TractionStatus::ConstPtr& status);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
  void leftMotorCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg);
  void rightMotorCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg);
  void liftMotorCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg);

private:
  ros::NodeHandle nh_;

  ros::Timer timer_;

  ros::Subscriber status_lspcu_sub_;
  ros::Subscriber status_tdcu_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber left_feedback_sub_;
  ros::Subscriber right_feedback_sub_;
  ros::Subscriber lift_feedback_sub_;

  diagnostic_updater::TopicDiagnostic* imu_diagnostic_;

  boxer_msgs::Status::ConstPtr last_status_lspcu_;
  boxer_msgs::TractionStatus::ConstPtr last_status_tdcu_;
  sensor_msgs::BatteryState::ConstPtr last_battery_state_;
  boxer_msgs::MotorFeedback::ConstPtr last_left_motor_state_;
  boxer_msgs::MotorFeedback::ConstPtr last_right_motor_state_;
  boxer_msgs::MotorFeedback::ConstPtr last_lift_motor_state_;

  bool left_bridge_over_temp_fault_;
  bool left_motor_over_temp_fault_;
  bool right_bridge_over_temp_fault_;
  bool right_motor_over_temp_fault_;
  bool lift_bridge_over_temp_fault_;
  bool lift_motor_over_temp_fault_;

  double motor_temperature_warn_;
  double motor_temperature_error_;
  double bridge_temperature_warn_;
  double bridge_temperature_error_;
  double battery_temperature_warn_;
  double battery_temperature_error_;

  double expected_imu_frequency_;

  uint32_t lift_current_stop_count_;
  bool last_current_stop_status_;

  std::string expected_firmware_package_ver_;

  /**
   * @brief Take a hardware_id string coming from the firmware and determine if it is a developer version.
   * Split a "x.x.x::REV x" to get the version, and if a developer SHA is present, then flag so.
   * @param hw_id Is the string provided by the firmware
   * @param firmware_ver This is set to the stripped version.
   * @param is_dev Is set to true if a developer SHA is found.
   */
  void processHardwareId(const std::string& hw_id, std::string& firmware_ver, bool& is_dev) const;

  /**
   * @brief Take a mcu reset reason message and convert it to a readable
   * string "Low Power, Pin Reset, Power on Reset" combining all stated
   * reset reasons.
   * @param reason A MCUResetReason msg provided from the mcu.
   * @return The resulting string combination.
   */
  std::string getResetReasonString(const boxer_msgs::MCUResetReason& reset_reason);
};

}  // namespace boxer_base

#endif  // BOXER_BASE_BOXER_DIAGNOSTIC_UPDATER_H
