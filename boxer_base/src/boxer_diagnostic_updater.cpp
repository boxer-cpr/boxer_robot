/**
Software License Agreement (BSD)

\file      boxer_diagnostic_updater.cpp
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

#include <string>
#include <sstream>
#include <vector>
#include <sys/types.h>
#include <ifaddrs.h>

#include "boost/algorithm/string/predicate.hpp"
#include "diagnostic_updater/update_functions.h"
#include "boxer_base/boxer_diagnostic_updater.h"

namespace boxer_base
{

BoxerDiagnosticUpdater::BoxerDiagnosticUpdater()
{
  lift_current_stop_count_ = 0;
  last_current_stop_status_ = false;

  left_bridge_over_temp_fault_ = false;
  left_motor_over_temp_fault_ = false;
  right_bridge_over_temp_fault_ = false;
  right_motor_over_temp_fault_ = false;
  lift_bridge_over_temp_fault_ = false;
  lift_motor_over_temp_fault_ = false;

  setHardwareID("unknown");

  add("General Logic", this, &BoxerDiagnosticUpdater::generalLspcuDiagnostics);
  add("General Drivetrain", this, &BoxerDiagnosticUpdater::generalTdcuDiagnostics);
  add("Battery", this, &BoxerDiagnosticUpdater::batteryDiagnostics);
  add("User voltage supplies", this, &BoxerDiagnosticUpdater::voltageDiagnostics);
  add("Current consumption", this, &BoxerDiagnosticUpdater::currentDiagnostics);
  add("Left Motor", this, &BoxerDiagnosticUpdater::leftMotorDiagnostics);
  add("Right Motor", this, &BoxerDiagnosticUpdater::rightMotorDiagnostics);
  add("Lift Motor", this, &BoxerDiagnosticUpdater::liftMotorDiagnostics);
  add("Safety Loop", this, &BoxerDiagnosticUpdater::safetyLoopDiagnostics);
  add("Sonar", this, &BoxerDiagnosticUpdater::sonarDiagnostics);
  add("Emergency Stop", this, &BoxerDiagnosticUpdater::eStopDiagnostics);

  // The arrival of this message runs the update() method and triggers the above callbacks.
  status_lspcu_sub_ = nh_.subscribe("mcu/status", 5, &BoxerDiagnosticUpdater::lspcuStatusCallback, this);
  status_tdcu_sub_ = nh_.subscribe("mcu/traction/status", 5, &BoxerDiagnosticUpdater::tdcuStatusCallback, this);
  battery_sub_ = nh_.subscribe("bms/battery_status", 5, &BoxerDiagnosticUpdater::batteryCallback, this);
  left_feedback_sub_ = nh_.subscribe("mcu/left_feedback", 5, &BoxerDiagnosticUpdater::leftMotorCallback, this);
  right_feedback_sub_ = nh_.subscribe("mcu/right_feedback", 5, &BoxerDiagnosticUpdater::rightMotorCallback, this);
  lift_feedback_sub_ = nh_.subscribe("mcu/lift_feedback", 5, &BoxerDiagnosticUpdater::liftMotorCallback, this);

  // Thermal Limits
  ros::param::param("~motor_temperature_warn", motor_temperature_warn_, 50.0);
  ros::param::param("~motor_temperature_error", motor_temperature_error_, 60.0);
  ros::param::param("~bridge_temperature_warn", bridge_temperature_warn_, 50.0);
  ros::param::param("~bridge_temperature_error", bridge_temperature_error_, 60.0);
  ros::param::param("~battery_temperature_warn", battery_temperature_warn_, 40.0);
  ros::param::param("~battery_temperature_error", battery_temperature_error_, 45.0);

  // Firmware Version
  ros::NodeHandle nh_private("~");
  nh_private.getParam("current_firmware_version", expected_firmware_package_ver_);
  // Strip extra whitespace from param server.
  std::size_t pos = expected_firmware_package_ver_.find('\n');
  if (std::string::npos != pos)
  {
    expected_firmware_package_ver_.erase(expected_firmware_package_ver_.begin()+pos,
                                         expected_firmware_package_ver_.end());
  }
  ROS_INFO("Current expected firmware version: %s", expected_firmware_package_ver_.c_str());

  // These message frequencies are reported on separately.
  ros::param::param("~expected_imu_frequency", expected_imu_frequency_, 50.0);
  imu_diagnostic_ = new diagnostic_updater::TopicDiagnostic("/imu/data_raw", *this,
      diagnostic_updater::FrequencyStatusParam(&expected_imu_frequency_, &expected_imu_frequency_, 0.15),
      diagnostic_updater::TimeStampStatusParam(-1, 1.0));
  imu_sub_ = nh_.subscribe("/imu/data_raw", 5, &BoxerDiagnosticUpdater::imuCallback, this);
}

void BoxerDiagnosticUpdater::generalLspcuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!last_status_lspcu_)
  {
    return;
  }

  // Add a counter for Elastic reporting for number of current stops
  // seen since startup. Only count false to true transitions
  if (last_status_lspcu_->stop & boxer_msgs::Status::STOP_CURRENT &&
      last_current_stop_status_ == false)
  {
    lift_current_stop_count_++;
  }
  last_current_stop_status_ = last_status_lspcu_->stop & boxer_msgs::Status::STOP_CURRENT;


  // LSPCU Info
  stat.add("MCU uptime (s)", last_status_lspcu_->mcu_uptime.toSec());
  stat.add("MCU connection uptime (s)", last_status_lspcu_->connection_uptime.toSec());
  stat.add("PCB Temperature (C)", last_status_lspcu_->pcb_temperature);
  stat.add("MCU Temperature (C)", last_status_lspcu_->mcu_temperature);

  stat.add("Firmware actual", last_status_lspcu_->hardware_id);
  stat.add("Firmware expected", expected_firmware_package_ver_.c_str());

  // Add stats as estop flags dictate
  stat.add("Emergency Stop", last_status_lspcu_->stop & boxer_msgs::Status::STOP_ESTOPPED ? "True" : "False");
  stat.add("Traction Emergency Stop",
      last_status_lspcu_->stop & boxer_msgs::Status::STOP_ESTOP_TRACTION ? "True" : "False");
  stat.add("PC Comms Loss Emergency Stop",
      last_status_lspcu_->stop & boxer_msgs::Status::STOP_ESTOP_PC_COMMS_LOST ? "True" : "False");
  stat.add("BMU Error Emergency Stop",
      last_status_lspcu_->stop & boxer_msgs::Status::STOP_ESTOP_BMU_STATUS ? "True" : "False");
  stat.add("Reset Required", last_status_lspcu_->stop &
  boxer_msgs::Status::STOP_ESTOP_RESET_REQUIRED ? "True" : "False");
  stat.add("Lidar Safety Stop", last_status_lspcu_->stop & boxer_msgs::Status::STOP_LIDAR ? "True" : "False");
  stat.add("Sonar Stop", last_status_lspcu_->stop & boxer_msgs::Status::STOP_SONAR ? "True" : "False");
  stat.add("Temperature Stop", last_status_lspcu_->stop & boxer_msgs::Status::STOP_TEMPERATURE ? "True" : "False");
  stat.add("Current Stop", last_status_lspcu_->stop & boxer_msgs::Status::STOP_CURRENT ? "True" : "False");
  stat.add("Current Stop Count", lift_current_stop_count_);
  stat.add("Motor Precharging Stop", last_status_lspcu_->stop &
  boxer_msgs::Status::STOP_PRECHARGING ? "True" : "False");
  stat.add("Charging Stop", last_status_lspcu_->stop & boxer_msgs::Status::STOP_CHARGING ? "True" : "False");
  stat.add("Computer Commanded Emergency Stop",
      last_status_lspcu_->stop & boxer_msgs::Status::STOP_ESTOP_COMPUTER_COMMANDED ? "True" : "False");

  std::string reset_reason = getResetReasonString(last_status_lspcu_->reset_reason);
  stat.add("MCU Last Reset Reasons", reset_reason);

  // Set the overall status.
  if (last_status_lspcu_->stop == boxer_msgs::Status::STOP_CLEAR)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Ready to move");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Stopped");
  }

  std::string found_version;
  bool is_dev = false;
  processHardwareId(last_status_lspcu_->hardware_id, found_version, is_dev);
  if (!last_status_lspcu_->hardware_id.empty() && is_dev)
  {
    stat.mergeSummaryf(stat.WARN, "Developer firmware present (%s)", last_status_lspcu_->hardware_id.c_str());
  }
  else if (expected_firmware_package_ver_ != found_version)
  {
    stat.mergeSummaryf(stat.WARN, "Firmware version is %s, expected %s.",
        found_version.c_str(), expected_firmware_package_ver_.c_str());
  }
  else
  {
    stat.mergeSummaryf(stat.OK, "Expected firmware present (%s)", last_status_lspcu_->hardware_id.c_str());
  }
}

void BoxerDiagnosticUpdater::sonarDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!last_status_lspcu_)
  {
    return;
  }

  stat.summary(stat.OK, "Sonars OK");

  if (last_status_lspcu_->sonar_device == boxer_msgs::Status::SONAR_DEVICE_MB7047)
  {
    stat.add("Sonar Device", "MB7047");
  }
  else if (last_status_lspcu_->sonar_device == boxer_msgs::Status::SONAR_DEVICE_SRF235)
  {
    stat.add("Sonar Device", "SRF235");
  }
  else
  {
    stat.add("Sonar Device", "Unknown");
    stat.mergeSummary(stat.ERROR, "Sonar type not detected.");
  }
}

void BoxerDiagnosticUpdater::generalTdcuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!last_status_tdcu_)
  {
    return;
  }

  // TDCU Info
  stat.add("MCU uptime (s)", last_status_tdcu_->mcu_uptime.toSec());
  stat.add("MCU connection uptime (s)", last_status_tdcu_->connection_uptime.toSec());
  stat.add("PCB Temperature (C)", last_status_tdcu_->pcb_temperature);
  stat.add("MCU Temperature (C)", last_status_tdcu_->mcu_temperature);

  stat.add("Firmware actual", last_status_tdcu_->hardware_id);
  stat.add("Firmware expected", expected_firmware_package_ver_.c_str());

  stat.add("Emergency Stop", last_status_tdcu_->stop != boxer_msgs::TractionStatus::STOP_CLEAR ? "True" : "False");
  stat.add("PC Comms Loss Emergency Stop",
      last_status_tdcu_->stop & boxer_msgs::TractionStatus::STOP_ESTOP_PC_COMMS_LOST ? "True" : "False");
  stat.add("Temperature Stop",
      last_status_tdcu_->stop & boxer_msgs::TractionStatus::STOP_OVER_TEMPERATURE ? "True" : "False");

  std::string reset_reason = getResetReasonString(last_status_tdcu_->reset_reason);
  stat.add("MCU Last Reset Reasons", reset_reason);

  // TODO(modriscoll): Add stat's when firmware implementation available:
  // STOP_DRIVER_FAULT
  // STOP_OVER_CURRENT
  // STOP_PRECHARGING
  // STOP_COMMANDS_STALE

  // Set the overall status.
  if (last_status_tdcu_->stop == boxer_msgs::TractionStatus::STOP_CLEAR)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Ready to move");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Stopped");
  }

  std::string found_version;
  bool is_dev = false;
  processHardwareId(last_status_tdcu_->hardware_id, found_version, is_dev);

  if (!last_status_tdcu_->hardware_id.empty() && is_dev)
  {
    stat.mergeSummaryf(stat.WARN, "Developer firmware present (%s)", last_status_tdcu_->hardware_id.c_str());
  }
  else if (expected_firmware_package_ver_ != found_version)
  {
    stat.mergeSummaryf(stat.WARN, "Firmware version is %s, expected %s.",
        found_version.c_str(), expected_firmware_package_ver_.c_str());
  }
  else
  {
    stat.mergeSummaryf(stat.OK, "Expected firmware present (%s)", last_status_tdcu_->hardware_id.c_str());
  }
}

void BoxerDiagnosticUpdater::batteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!last_status_lspcu_ || !last_battery_state_)
  {
    return;
  }

  stat.add("Voltage (V)", last_battery_state_->voltage);
  stat.add("Current (A)", last_battery_state_->current);
  stat.add("Charge (Ah)", last_battery_state_->charge);
  stat.add("Capacity (Ah)", last_battery_state_->capacity);
  stat.add("Design Capacity (Ah)", last_battery_state_->design_capacity);
  stat.add("Percent Charge (%)", last_battery_state_->percentage);
  stat.add("Present", last_battery_state_->present);

  // Add cell voltages
  uint32_t cell_num = 0;
  for (const float& i : last_battery_state_->cell_voltage)
  {
    std::stringstream cell_name;
    cell_name << "Cell Voltage: " << ++cell_num << " (V)";
    stat.add(cell_name.str(), i);
  }

  // Add cell temperatures
  cell_num = 0;
  for (const float& i : last_status_lspcu_->battery_cell_temperature)
  {
    std::stringstream cell_name;
    cell_name << "Battery Cell " << ++cell_num << " Temperature (C)";
    stat.add(cell_name.str(), i);
  }

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery Voltage OK.");

  // Check battery cell voltage
  if (last_status_lspcu_->measured_battery > 32.4)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery overvoltage.");
  }
  else if (last_status_lspcu_->measured_battery < 1.0)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery voltage not detected, check BATT fuse.");
  }
  else if (last_status_lspcu_->measured_battery < 24.0)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery low voltage.");
  }
  else if (last_status_lspcu_->measured_battery < 22.0)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery critically under voltage.");
  }

  // Check CMU status
  if (last_status_lspcu_->battery_cmu_failure)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery CMU Failure.");
  }
  else
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Battery CMU OK.");
  }

  // Check if we have any failed battery cells
  if (last_status_lspcu_->battery_failed_cells.size() > 0)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery Cell failure.");
    for (const float& i : last_status_lspcu_->battery_failed_cells)
    {
      stat.add("Cell # Failed", i);
    }
  }

  // Check battery temperature
  bool battery_temp_error = false;
  bool battery_temp_warn = false;
  for (const float& i : last_status_lspcu_->battery_cell_temperature)
  {
    if (i >= battery_temperature_warn_)
    {
      battery_temp_warn = true;
    }
    if (i >= battery_temperature_error_)
    {
      battery_temp_error = true;
    }
  }
  if (battery_temp_error)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery Temperature Error.");
  }
  else if (battery_temp_warn)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery Temperature Warn.");
  }
  else
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Battery Temperature OK.");
  }

  // Check supply health enum from LSPCU.
  if (last_battery_state_->power_supply_health == sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery State Unknown");
  }
  else if (last_battery_state_->power_supply_health != sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery Error");
  }
  else
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK.");
  }
}

void BoxerDiagnosticUpdater::voltageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!last_status_lspcu_)
  {
    return;
  }

  stat.add("12V Supply (V)", last_status_lspcu_->measured_12v);
  stat.add("5V Supply (V)", last_status_lspcu_->measured_5v);
  stat.add("Measured Battery Voltage (V)", last_status_lspcu_->measured_battery);

  if (last_status_lspcu_->measured_12v > 12.5 || last_status_lspcu_->measured_5v > 5.5)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "User supply overvoltage. Accessories may be damaged.");
  }
  else if (last_status_lspcu_->measured_12v < 1.0 || last_status_lspcu_->measured_5v < 1.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "User supplies absent. Check tray fuses.");
  }
  else if (last_status_lspcu_->measured_12v < 11.0 || last_status_lspcu_->measured_5v < 4.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Voltage supplies undervoltage. Check loading levels.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "User supplies OK.");
  }
}

void BoxerDiagnosticUpdater::currentDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!last_status_lspcu_)
  {
    return;
  }

  stat.add("Drive current (A)", last_status_lspcu_->drive_current);
  stat.add("User current (A)", last_status_lspcu_->user_current);
  stat.add("Computer current (A)", last_status_lspcu_->computer_current);
  stat.add("Total current (A)", last_status_lspcu_->total_current);

  if (last_status_lspcu_->total_current > 32.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Current draw critical.");
  }
  else if (last_status_lspcu_->total_current > 20.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Current draw warning.");
  }
  else if (last_status_lspcu_->total_current > 10.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw requires monitoring.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw nominal.");
  }
}

void BoxerDiagnosticUpdater::leftMotorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (last_left_motor_state_)
  {
    stat.add("Left Motor Current (A)", last_left_motor_state_->current);
    stat.add("Left Motor Bridge Temperature (C)", last_left_motor_state_->bridge_temperature);
    stat.add("Left Motor Motor Temperature (C)", last_left_motor_state_->motor_temperature);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Left Motor OK.");

    if (last_left_motor_state_->fault)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Left Motor Faulted.");
    }
    if (last_left_motor_state_->driver_in_reset)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Left Motor in Reset.");
    }

    // If below the warn level, clear an existing fault.
    if (last_left_motor_state_->motor_temperature < motor_temperature_warn_)
    {
      left_motor_over_temp_fault_ = false;
    }
    // If above the high mark throw an error, or if we haven't come below the threshold
    // for clearing the error below.
    else if ((last_left_motor_state_->motor_temperature >= motor_temperature_error_) ||
        left_motor_over_temp_fault_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Left Motor Over Temperature.");
      left_motor_over_temp_fault_ = true;
    }
    else if (last_left_motor_state_->motor_temperature >= motor_temperature_warn_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Left Motor High Temperature.");
    }

    // If below the warn level, clear an existing fault.
    if (last_left_motor_state_->bridge_temperature < bridge_temperature_warn_)
    {
      left_bridge_over_temp_fault_ = false;
    }
    // If above the high mark throw an error, or if we haven't come below the threshold
    // for clearing the error below.
    else if ((last_left_motor_state_->bridge_temperature >= bridge_temperature_error_) ||
        left_bridge_over_temp_fault_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Left Bridge Over Temperature.");
      left_bridge_over_temp_fault_ = true;
    }
    else if (last_left_motor_state_->bridge_temperature >= bridge_temperature_warn_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Left Bridge High Temperature.");
    }
  }
}

void BoxerDiagnosticUpdater::rightMotorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (last_right_motor_state_)
  {
    stat.add("Right Motor Current (A)", last_right_motor_state_->current);
    stat.add("Right Motor Bridge Temperature (C)", last_right_motor_state_->bridge_temperature);
    stat.add("Right Motor Motor Temperature (C)", last_right_motor_state_->motor_temperature);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Right Motor OK.");

    if (last_right_motor_state_->fault)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Right Motor Faulted.");
    }
    if (last_right_motor_state_->driver_in_reset)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Right Motor in Reset.");
    }

    // If below the warn level, clear an existing fault.
    if (last_right_motor_state_->motor_temperature < motor_temperature_warn_)
    {
      right_motor_over_temp_fault_ = false;
    }
    // If above the high mark throw an error, or if we haven't come below the threshold
    // for clearing the error below.
    else if ((last_right_motor_state_->motor_temperature >= motor_temperature_error_) ||
        right_motor_over_temp_fault_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Right Motor Over Temperature.");
      right_motor_over_temp_fault_ = true;
    }
    else if (last_right_motor_state_->motor_temperature >= motor_temperature_warn_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Right Motor High Temperature.");
    }

    // If below the warn level, clear an existing fault.
    if (last_right_motor_state_->bridge_temperature < bridge_temperature_warn_)
    {
      right_bridge_over_temp_fault_ = false;
    }
    // If above the high mark, throw an error, or if we haven't come below the threshold
    // for clearing the error below.
    else if ((last_right_motor_state_->bridge_temperature >= bridge_temperature_error_) ||
        right_bridge_over_temp_fault_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Right Bridge Over Temperature.");
      right_bridge_over_temp_fault_ = true;
    }
    else if (last_right_motor_state_->bridge_temperature >= bridge_temperature_warn_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Right Bridge High Temperature.");
    }
  }
}

void BoxerDiagnosticUpdater::liftMotorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (last_lift_motor_state_)
  {
    stat.add("Lift Motor Current (A)", last_lift_motor_state_->current);
    stat.add("Lift Motor Bridge Temperature (C)", last_lift_motor_state_->bridge_temperature);
    stat.add("Lift Motor Motor Temperature (C)", last_lift_motor_state_->motor_temperature);
    stat.add("Lift Lower Limit",
        last_lift_motor_state_->limit & boxer_msgs::MotorFeedback::LIMIT_LOWER
        ? "Tripped"
        : "Clear");
    stat.add("Lift Upper Limit",
        last_lift_motor_state_->limit & boxer_msgs::MotorFeedback::LIMIT_UPPER
        ? "Tripped"
        : "Clear");

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Lift Motor OK.");

    // Check for the most severe fault flags first.  Less severe flags that are also set are almost ceratinly
    // a result of the more severe faults.
    if (last_lift_motor_state_->fault & boxer_msgs::MotorFeedback::FAULT_DRIVER)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Lift Motor Driver Faulted.");
    }
    else if (last_lift_motor_state_->fault & boxer_msgs::MotorFeedback::FAULT_HOME_TIMEOUT)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                        "Lift did not trigger limit switch in expected timeframe!");
    }

    if (last_lift_motor_state_->driver_in_reset)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Lift Motor in Reset.");
    }

    if (last_lift_motor_state_->fault & boxer_msgs::MotorFeedback::FAULT_OVERHEAT)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
                        "Lift is overheating, waiting for cool-down before allowing new drive commands.");
    }

    // If below the warn level, clear an existing fault.
    if (last_lift_motor_state_->motor_temperature < motor_temperature_warn_)
    {
      lift_motor_over_temp_fault_ = false;
    }
    // If above the high mark throw an error, or if we haven't come below the threshold
    // for clearing the error below.
    else if ((last_lift_motor_state_->motor_temperature >= motor_temperature_error_) ||
             lift_motor_over_temp_fault_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Lift Motor Over Temperature.");
      lift_motor_over_temp_fault_ = true;
    }
    else if (last_lift_motor_state_->motor_temperature >= motor_temperature_warn_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Lift Motor High Temperature.");
    }

    // If below the warn level, clear an existing fault.
    if (last_lift_motor_state_->bridge_temperature < bridge_temperature_warn_)
    {
      lift_bridge_over_temp_fault_ = false;
    }
    // If above the high mark, throw an error, or if we haven't come below the threshold
    // for clearing the error below.
    else if ((last_lift_motor_state_->bridge_temperature >= bridge_temperature_error_) ||
             lift_bridge_over_temp_fault_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Lift Bridge Over Temperature.");
      lift_bridge_over_temp_fault_ = true;
    }
    else if (last_lift_motor_state_->bridge_temperature >= bridge_temperature_warn_)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Lift Bridge High Temperature.");
    }

    if (!last_lift_motor_state_->homing_complete)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Lift Motor Not Homed.");
    }

    if (last_lift_motor_state_->limit == (boxer_msgs::MotorFeedback::LIMIT_UPPER |
      boxer_msgs::MotorFeedback::LIMIT_LOWER))
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Both Lift limit switches triggered.");
    }
  }
}

void BoxerDiagnosticUpdater::safetyLoopDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!last_status_lspcu_)
  {
    return;
  }

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Safety Loop OK.");

  stat.add("OSSD1 Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_OSSD1_STATUS ? "Clear" : "Tripped");
  stat.add("OSSD2 Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_OSSD2_STATUS ? "Clear" : "Tripped");
  stat.add("LSPCU Stop Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_LSPCU_STOP_STATUS ?
    "Stopped" : "Clear");
  stat.add("TDCU Stop Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_TDCU_STOP_STATUS ?
    "Stopped" : "Clear");
  stat.add("K1 Coil Monitor Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_K1_COIL_MONITOR_STATUS ?
    "Closed" : "Open");
  stat.add("K2 Coil Monitor Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_K2_COIL_MONITOR_STATUS ?
    "Closed" : "Open");
  stat.add("Coil Net Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_K_COIL_NET_STATUS ?
    "OK" : "Mismatched");
  stat.add("EStop Power Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_ESTOP_POWER_CHECK_STATUS ?
    "Power OK" : "Error");
  stat.add("EStop External Channel Net Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_ESTOP_IMPLAUSIBLE_STATUS ?
    "OK" : "Error");
  stat.add("EStop Reset Enabled",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_ESTOP_RESET_ENABLED_STATUS ?
    "Enabled" : "Disabled");
  stat.add("EStop Reset Button State",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_ESTOP_RESET_BUTTON_STATUS ?
    "Depressed" : "Released");
  stat.add("Safety Relay Aux Check",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_AUX_CHECK_STATUS ?
    "Both Open" : "Both Closed");
  stat.add("Safety Relay OSSD1 Coil Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_SAFETY_RELAY_COIL_STATUS_OSSD1 ?
    "True" : "False");
  stat.add("Safety Relay OSSD2 Coil Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_SAFETY_RELAY_COIL_STATUS_OSSD2 ?
    "True" : "False");
  stat.add("Safety Relay Ground Path Enabled",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_SAFETY_RELAY_GROUND_PATH ?
    "True" : "False");
  stat.add("LSPCU Safety Relay Watchdog Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_LSPCU_SAFETY_RELAY_WATCHDOG_STATUS ?
    "Disabled" : "OK");
  stat.add("TDCU Safety Relay Watchdog Status",
    last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_TDCU_SAFETY_RELAY_WATCHDOG_STATUS ?
    "Disabled" : "OK");
  stat.add("External E-Stop Loop Channel 1",
      last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_ESTOP_EXTERNAL_CHANNEL1 ?
      "Closed" : "Open");
  stat.add("External E-Stop Loop Channel 2",
      last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_ESTOP_EXTERNAL_CHANNEL2 ?
      "Closed" : "Open");

  // Update summary with various status options.
  if (last_status_lspcu_->safety_loop_status &
    boxer_msgs::Status::SAFETY_LOOP_STATUS_LSPCU_SAFETY_RELAY_WATCHDOG_STATUS)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "LSPCU Safety Relay Watchdog Disabled.");
  }
  if (last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_TDCU_SAFETY_RELAY_WATCHDOG_STATUS)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "TDCU Safety Relay Watchdog Disabled.");
  }
  // Invert read 0 == bad, flag true to PC when wrong
  if (!(last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_ESTOP_IMPLAUSIBLE_STATUS))
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "External Safety Channel Mismatch.");
  }

  // OSSD Qualification state, electrical edge case notification when the following
  // is true.
  // 1) Both OSSDs are clear
  // AND
  // 2) Main Reset is enabled
  // AND
  // 3) Either Safety Relay Coil status is not good
  // AND
  // 4) Both K_COILs are closed.
  if ((last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_OSSD1_STATUS &&
      last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_OSSD2_STATUS) &&
      last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_ESTOP_RESET_ENABLED_STATUS &&
      (!(last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_SAFETY_RELAY_COIL_STATUS_OSSD1) ||  // NOLINT(whitespace/line_length)
       !(last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_SAFETY_RELAY_COIL_STATUS_OSSD1)) &&  // NOLINT(whitespace/line_length)
      (last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_K1_COIL_MONITOR_STATUS &&
      last_status_lspcu_->safety_loop_status & boxer_msgs::Status::SAFETY_LOOP_STATUS_K1_COIL_MONITOR_STATUS))
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Safety Relay Unable to close, missed OSSD edge transition.");
  }
}

void BoxerDiagnosticUpdater::eStopDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!last_status_lspcu_)
  {
    return;
  }

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Platform safe.");

  if (last_status_lspcu_->stop & boxer_msgs::Status::STOP_ESTOPPED)
  {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "E-stopped!");
  }
}

std::string BoxerDiagnosticUpdater::getResetReasonString(const boxer_msgs::MCUResetReason& reset_reason)
{
  std::vector<std::string> reset_strs;
  std::stringstream ss;
  boxer_msgs::MCUResetReason::_reason_type reason = reset_reason.reason;

  // Gather all the reset reasons
  if (reason & boxer_msgs::MCUResetReason::RESET_LOW_POWER)
  {
    reset_strs.push_back("Low Power");
  }
  if (reason & boxer_msgs::MCUResetReason::RESET_WINDOW_WATCHDOG)
  {
    reset_strs.push_back("Window Watchdog");
  }
  if (reason & boxer_msgs::MCUResetReason::RESET_INDEPENDENT_WATCHDOG)
  {
    reset_strs.push_back("Independent Watchdog");
  }
  if (reason & boxer_msgs::MCUResetReason::RESET_SOFTWARE_RESET)
  {
    reset_strs.push_back("Software Reset");
  }
  if (reason & boxer_msgs::MCUResetReason::RESET_POWER_ON_RESET)
  {
    reset_strs.push_back("Power on Reset");
  }
  if (reason & boxer_msgs::MCUResetReason::RESET_PIN_RESET)
  {
    reset_strs.push_back("Pin Reset");
  }
  if (reason & boxer_msgs::MCUResetReason::RESET_BROWN_OUT_RESET)
  {
    reset_strs.push_back("Brown Out Reset");
  }

  // Generate if no reset sources, return none.
  if (reset_strs.size() == 0)
  {
    return std::string("None");
  }

  auto iter = reset_strs.begin();
  // First reasons has no punctuation so just add it to the string.
  ss << *iter;
  iter++;
  for (; iter != reset_strs.end(); iter++)
  {
    // Additional reasons need some punctuation
    ss << ", " << *iter;
  }

  // Return the combined string.
  return ss.str();
}

void BoxerDiagnosticUpdater::lspcuStatusCallback(const boxer_msgs::Status::ConstPtr& status)
{
  // Fresh data from the MCU, publish a diagnostic update.
  last_status_lspcu_ = status;
  setHardwareID(last_status_lspcu_->hardware_id);
  update();
}

void BoxerDiagnosticUpdater::tdcuStatusCallback(const boxer_msgs::TractionStatus::ConstPtr& status)
{
  // Fresh data from the MCU, publish a diagnostic update.
  last_status_tdcu_ = status;
  update();
}

void BoxerDiagnosticUpdater::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_diagnostic_->tick(msg->header.stamp);
}

void BoxerDiagnosticUpdater::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  last_battery_state_ = msg;
  update();
}

void BoxerDiagnosticUpdater::leftMotorCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg)
{
  last_left_motor_state_ = msg;
  update();
}

void BoxerDiagnosticUpdater::rightMotorCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg)
{
  last_right_motor_state_ = msg;
  update();
}

void BoxerDiagnosticUpdater::liftMotorCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg)
{
  last_lift_motor_state_ = msg;
  update();
}

void BoxerDiagnosticUpdater::processHardwareId(const std::string& hw_id, std::string& firmware_ver, bool& is_dev) const
{
  std::string found_version = "Unknown";
  is_dev = false;

  size_t split_count = std::count(hw_id.begin(), hw_id.end(), ':');
  if (split_count >= 2)
  {
    found_version = hw_id.substr(0, hw_id.find("::"));
  }

  if (split_count > 2)
  {
     is_dev = true;
  }

  firmware_ver = found_version;
}

}  // namespace boxer_base
