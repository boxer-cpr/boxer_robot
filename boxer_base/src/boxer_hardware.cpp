/**
Software License Agreement (BSD)

\file      boxer_hardware.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2013, Rockwell Automation Technologies, Inc., All rights reserved.

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

#include <boost/assign.hpp>
#include <limits>
#include <urdf/model.h>
#include "boxer_base/boxer_hardware.h"
#include "std_msgs/Bool.h"

#include "boxer_msgs/Lift.h"

namespace boxer_base
{

BoxerHardware::BoxerHardware() :
  estop_(false),
  safety_stop_(false),
  charging_spinner_(2, &charging_service_queue_)
{
  hold_state_ = HoldState::DISABLED;
  drive_state_ = DriveState::VELOCITY;

  ros::V_string joint_names = boost::assign::list_of("wheel_left_joint")
      ("wheel_right_joint")("lift_joint");

  urdf::Model urdf;
  urdf.initParam("robot_description");
  lower_limit_position_ = urdf.getJoint("lift_joint")->limits->lower;
  upper_limit_position_ = urdf.getJoint("lift_joint")->limits->upper;

  hardware_interface::JointStateHandle left_joint_state_handle(joint_names[0],
      &joints_[0].position, &joints_[0].velocity, &joints_[0].effort);
  joint_state_interface_.registerHandle(left_joint_state_handle);
  hardware_interface::JointHandle left_joint_handle(
      left_joint_state_handle, &joints_[0].velocity_command);
  velocity_joint_interface_.registerHandle(left_joint_handle);

  hardware_interface::JointStateHandle right_joint_state_handle(joint_names[1],
      &joints_[1].position, &joints_[1].velocity, &joints_[1].effort);
  joint_state_interface_.registerHandle(right_joint_state_handle);
  hardware_interface::JointHandle right_joint_handle(
      right_joint_state_handle, &joints_[1].velocity_command);
  velocity_joint_interface_.registerHandle(right_joint_handle);

  hardware_interface::JointStateHandle lift_joint_state_handle(joint_names[2],
      &joints_[2].position, &joints_[2].velocity, &joints_[2].effort);
  joint_state_interface_.registerHandle(lift_joint_state_handle);
  hardware_interface::JointHandle lift_joint_handle(
      lift_joint_state_handle, &joints_[2].position_command);
  position_joint_interface_.registerHandle(lift_joint_handle);

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
  registerInterface(&position_joint_interface_);

  // Zero the caster/dummy wheel joints position, velocity and effort
  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    joints_[i].position = 0;
    joints_[i].velocity = 0;
    joints_[i].effort = 0;
  }

  travel_offset_[0] = 0;
  travel_offset_[1] = 0;

  nh_.param<float>("position_hold_enable_delay", wait_enable_timeout_, 0.1);
  nh_.param<float>("position_hold_time", hold_position_timeout_, 15.0);

  lspcu_status_sub_ = nh_.subscribe("mcu/status", 1, &BoxerHardware::lspcuStatusCallback, this);
  left_feedback_sub_ = nh_.subscribe("mcu/left_feedback", 1, &BoxerHardware::leftFeedbackCallback, this);
  right_feedback_sub_ = nh_.subscribe("mcu/right_feedback", 1, &BoxerHardware::rightFeedbackCallback, this);
  lift_feedback_sub_ = nh_.subscribe("mcu/lift_feedback", 1, &BoxerHardware::liftFeedbackCallback, this);

  // Provide service functionality to home lift
  home_service_ = nh_.advertiseService("home_lift", &BoxerHardware::homeServiceCallback, this);
  force_raise_service_ = nh_.advertiseService("force_raise_lift", &BoxerHardware::raiseServiceCallback, this);
  home_requested_ = boxer_msgs::LiftPosition::HOME_NONE;

  // Service client for footprint check for auto-homing the lift
  footprint_check_service_ = nh_.serviceClient<cpr_common_srvs::Bool>("platform_footprint_check");

  // Provide functionality to e-stop the platform from the PC
  // Setup a latching publisher and set it to false initially
  // This is then changed by the emergency_stop service when called.
  estop_service_ = nh_.advertiseService("emergency_stop", &BoxerHardware::setEStopCallback, this);
  mcu_estop_pub_ = nh_.advertise<std_msgs::Bool>("mcu/cmd_emergency_stop", 1);
  set_estop_.data = false;

  // emergency and safety stop unified topic publishers
  platform_emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("platform/emergency_stop", 1, true);
  platform_safety_stop_pub_ = nh_.advertise<std_msgs::Bool>("platform/safety_stop", 1, true);

  // Provide functionality to autocharge
  // Setup a latching publisher and set it to false initially
  // This is then changed by the charging_set service when called.
  mcu_charge_enable_pub_ = nh_.advertise<std_msgs::Bool>("mcu/cmd_charging_set", 1);

  // Put the service server on it's own Async spinner as it interacts with it's own
  // thread when disabling charging from velocity interface.
  charging_service_nh_.setCallbackQueue(&charging_service_queue_);
  charge_service_ = charging_service_nh_.advertiseService("charging_set", &BoxerHardware::setCharging, this);
  enable_autocharge_.data = false;

  // service to initiate power down sequence
  shutdown_service_ = nh_.advertiseService("bms/poweroff", &BoxerHardware::poweroffServiceCallback, this);
  mcu_poweroff_pub_ = nh_.advertise<std_msgs::Bool>("mcu/cmd_bmu_poweroff", 1);

  // Publish the charge command and the e-stop command to the MCU at 1hz as service calls against
  // rosserial are undesirable.
  platform_pub_timer_ = nh_.createTimer(ros::Duration(1), &BoxerHardware::setPlatformPubCallback, this);

  // Setup timer to auto home on startup if required.
  autohome_timer_ = nh_.createTimer(ros::Duration(10), &BoxerHardware::autoHomeTimerCallback, this, true);

  // Realtime publisher, initializes differently from regular ros::Publisher
  cmd_drive_pub_.init(nh_, "mcu/cmd_drive", 1);
  lift_pos_pub_.init(nh_, "mcu/cmd_lift_position", 1);

  charging_spinner_.start();
}

void BoxerHardware::autoHomeTimerCallback(const ros::TimerEvent& event)
{
  autohome_timer_.stop();
  autohome_timer_.setPeriod(ros::Duration(10));

  // Check we've received at least one feedback message.
  if (!lift_feedback_msg_)
  {
    ROS_WARN("No lift feedback message received, unable to auto home lift.");
    autohome_timer_.start();
    return;
  }

  // Check we have at least one status message from the MCU.
  if (!last_lspcu_status_msg_)
  {
    ROS_WARN("No LSPCU status message received, unable to auto home lift.");
    autohome_timer_.start();
    return;
  }

  // If homing has not been completed, check the surrounding footprint being clear
  // before allowing the lift to home automatically.
  if (!lift_feedback_msg_->homing_complete)
  {
    // Check the footprint service exists.
    if (!footprint_check_service_.exists())
    {
      ROS_WARN("Footprint check service unavailable, unable to auto home lift.");
      autohome_timer_.start();
      return;
    }

    if (last_lspcu_status_msg_->stop != 0)
    {
      ROS_WARN("Robot currently safety/e-stopped, unable to auto home lift.");
      autohome_timer_.start();
      return;
    }

    cpr_common_srvs::Bool srv;
    srv.request.data = true;
    // Check the call succeeded.
    if (!footprint_check_service_.call(srv))
    {
      ROS_WARN("Footprint check service call failed, unable to auto home lift.");
      autohome_timer_.start();
      return;
    }

    // Check our response code.
    if (srv.response.data)
    {
      // Set homing down for next spin of the joint code.
      home_requested_ = boxer_msgs::LiftPosition::HOME_DOWN;
      ROS_INFO("Requesting Autohoming of the lift.");
      // Don't reset the timer... this code is done as homing has completed
      // once on startup.
    }
    else
    {
      // Call the timer again.
      ROS_WARN("Footprint check service reports not clear, unable to auto home lift.");
      autohome_timer_.start();
      return;
    }
  }
  else
  {
     ROS_INFO("Lift already homed, no auto homing required");
  }
}

/**
 * Populates the internal joint state struct from the most recent Feedback message
 * received from the MCU.
 *
 * Called from the controller thread.
 */
void BoxerHardware::copyJointsFromHardware()
{
  boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  if (left_feedback_msg_ && feedback_msg_lock)
  {
    // Detect a wheel travel jump (ie, due to an MCU reset), and adjust our
    // travel offset value accordingly. Threshold is arbitrarily some amount of travel
    // greater than 1rad per 0.02s period. Typical travel per control period at full
    // speed is 0.2rad, so this should be safe from false positives.
    double updated_position = left_feedback_msg_->measured_pose + travel_offset_[0];
    double diff = joints_[0].position - updated_position;
    if (std::abs(diff) > TRAVEL_OFFSET_THRESHOLD_RADS)
    {
      travel_offset_[0] += diff;
      ROS_WARN("Jump detected. Travel offset for joint %d is now %f", 0, travel_offset_[0]);
    }

    joints_[0].position = left_feedback_msg_->measured_pose + travel_offset_[0];
    joints_[0].velocity = left_feedback_msg_->measured_velocity;
    joints_[0].effort = 0;  // TODO(mikepurvis): determine this from amperage data.
  }

  if (right_feedback_msg_ && feedback_msg_lock)
  {
    // Detect a wheel travel jump (ie, due to an MCU reset), and adjust our
    // travel offset value accordingly. Threshold is arbitrarily some amount of travel
    // greater than 1rad per 0.02s period. Typical travel per control period at full
    // speed is 0.2rad, so this should be safe from false positives.
    double updated_position = right_feedback_msg_->measured_pose + travel_offset_[1];
    double diff = joints_[1].position - updated_position;
    if (std::abs(diff) > TRAVEL_OFFSET_THRESHOLD_RADS)
    {
      travel_offset_[1] += diff;
      ROS_WARN("Jump detected. Travel offset for joint %d is now %f", 1, travel_offset_[1]);
    }

    joints_[1].position = right_feedback_msg_->measured_pose + travel_offset_[1];
    joints_[1].velocity = right_feedback_msg_->measured_velocity;
    joints_[1].effort = 0;  // TODO(mikepurvis): determine this from amperage data.
  }

  if (lift_feedback_msg_ && feedback_msg_lock)
  {
    // If we've hit the upper limit switch for the position to max height.
    if (lift_feedback_msg_->limit & boxer_msgs::MotorFeedback::LIMIT_UPPER)
    {
      joints_[2].position = upper_limit_position_;
    }
    else
    {
      joints_[2].position = lift_feedback_msg_->measured_pose;
    }
    joints_[2].velocity = lift_feedback_msg_->measured_velocity;
    joints_[2].effort = 0;  // TODO(modriscoll): determine this from amperage data.
  }
}

void BoxerHardware::updateDriveState()
{
    switch (hold_state_)
    {
      // Normal mode, drive set to velocity mode
      case HoldState::DISABLED:
        ROS_DEBUG_THROTTLE(1, "HOLD DISABLED");
        // If commanded stop, go wait to hold.
        if (isVelocityStopped())
        {
          hold_state_ = HoldState::WAIT_ENABLE;
          position_hold_time_ = ros::Time::now();
        }
        // Publish last drive msg.
        drive_state_ = DriveState::VELOCITY;
        break;
      case HoldState::WAIT_ENABLE:
        ROS_DEBUG_THROTTLE(1, "HOLD WAITING");
        // If any movement is commanded, publish that and go back to disabled.
        if (isVelocityCommanded())
        {
          drive_state_ = DriveState::VELOCITY;
          hold_state_ = HoldState::DISABLED;
        }
        // Otherwise wait for the elapsed period at continued 0 speed,
        // then get the current position and go to hold.
        else if ((ros::Time::now() - position_hold_time_) > ros::Duration(wait_enable_timeout_))
        {
          hold_state_ = HoldState::ENABLE;
          position_hold_time_ = ros::Time::now();
          // Check we have a message, if not go back to disabled for now and re-set.
          if (left_feedback_msg_ && right_feedback_msg_)
          {
            joints_[0].position_command = left_feedback_msg_->measured_pose;
            joints_[1].position_command = right_feedback_msg_->measured_pose;
          }
          else
          {
            drive_state_ = DriveState::VELOCITY;
            hold_state_ = HoldState::DISABLED;
          }
        }
        break;
      case HoldState::ENABLE:
        ROS_DEBUG_THROTTLE(1, "HOLD ENABLE");
        // If any movement is commanded, move and go to disabled state.
        if (isVelocityCommanded())
        {
          hold_state_ = HoldState::DISABLED;
          drive_state_ = DriveState::VELOCITY;
        }
        // Otherwise go wait for the time to elapse on disabling hold after 15 seconds
        else
        {
          position_hold_time_ = ros::Time::now();
          hold_state_ = HoldState::POST_ENABLE;
          drive_state_ = DriveState::POSITION;
        }
        break;
      case HoldState::POST_ENABLE:
        ROS_DEBUG_THROTTLE(1, "HOLD POST ENABLE");
        // If any movement, go move and disable hold state.
        if (isVelocityCommanded())
        {
          hold_state_ = HoldState::DISABLED;
          drive_state_ = DriveState::VELOCITY;
        }
        // Otherwise if 15 seconds hasn't elapsed, Hold position.
        else if (ros::Time::now() - position_hold_time_ < ros::Duration(hold_position_timeout_))
        {
          drive_state_ = DriveState::POSITION;
        }
        // If not told to move, and holding place has elapsed, just use backEMF breaking.
        else
        {
          drive_state_ = DriveState::VELOCITY;
        }
        break;
      default:
        ROS_FATAL("Invalid Hold State: %u", static_cast<uint8_t>(hold_state_));
        break;
    }
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void BoxerHardware::publishDriveFromController()
{
  if (cmd_drive_pub_.trylock())
  {
    switch (drive_state_)
    {
      case DriveState::VELOCITY:
        cmd_drive_pub_.msg_.mode = boxer_msgs::Drive::MODE_VELOCITY;
        cmd_drive_pub_.msg_.drivers[boxer_msgs::Drive::LEFT] = joints_[0].velocity_command;
        cmd_drive_pub_.msg_.drivers[boxer_msgs::Drive::RIGHT] = joints_[1].velocity_command;
        break;
      case DriveState::POSITION:
        cmd_drive_pub_.msg_.mode = boxer_msgs::Drive::MODE_POSITION;
        cmd_drive_pub_.msg_.drivers[boxer_msgs::Drive::LEFT] = joints_[0].position_command;
        cmd_drive_pub_.msg_.drivers[boxer_msgs::Drive::RIGHT] = joints_[1].position_command;
        break;
      default:
        ROS_FATAL("Invalid Drive State: %u", static_cast<uint8_t>(drive_state_));
    }
    cmd_drive_pub_.unlockAndPublish();
  }
}

void BoxerHardware::publishLiftPositionFromController()
{
  if (lift_pos_pub_.trylock())
  {
    lift_pos_pub_.msg_.height = joints_[2].position_command;
    lift_pos_pub_.msg_.home_lift = home_requested_;
    if (home_requested_ != boxer_msgs::LiftPosition::HOME_NONE)
    {
      home_requested_ = boxer_msgs::LiftPosition::HOME_NONE;
    }
    lift_pos_pub_.unlockAndPublish();
  }
}

void BoxerHardware::lspcuStatusCallback(const boxer_msgs::Status::ConstPtr& msg)
{
  last_lspcu_status_msg_ = msg;
  static constexpr int ESTOP_MASK = boxer_msgs::Status::STOP_ESTOPPED;
  static constexpr int SAFETY_STOP_MASK = boxer_msgs::Status::STOP_LIDAR | boxer_msgs::Status::STOP_SONAR |
                                          boxer_msgs::Status::STOP_TEMPERATURE | boxer_msgs::Status::STOP_CURRENT;

  std_msgs::Bool bool_msg;

  if (msg->stop & ESTOP_MASK)
  {
    bool_msg.data = true;
  }
  else
  {
    bool_msg.data = false;
  }

  if (estop_ != bool_msg.data)
  {
    platform_emergency_stop_pub_.publish(bool_msg);
    estop_ = bool_msg.data;
  }

  if (msg->stop & SAFETY_STOP_MASK)
  {
    bool_msg.data = true;
  }
  else
  {
    bool_msg.data = false;
  }

  if (safety_stop_ != bool_msg.data)
  {
    platform_safety_stop_pub_.publish(bool_msg);
    safety_stop_ = bool_msg.data;
  }
}

void BoxerHardware::leftFeedbackCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  left_feedback_msg_ = msg;
}

void BoxerHardware::rightFeedbackCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  right_feedback_msg_ = msg;
}

void BoxerHardware::liftFeedbackCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  lift_feedback_msg_ = msg;
}

bool BoxerHardware::homeServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  home_requested_ = boxer_msgs::LiftPosition::HOME_DOWN;
  return true;
}

bool BoxerHardware::raiseServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  home_requested_ = boxer_msgs::LiftPosition::HOME_UP;
  return true;
}

bool BoxerHardware::poweroffServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  std_msgs::Bool msg;
  msg.data = true;
  mcu_poweroff_pub_.publish(msg);
  return true;
}

bool BoxerHardware::setCharging(cpr_common_srvs::Bool::Request &req, cpr_common_srvs::Bool::Response &res)
{
  // Set the requested autocharge state and publish
  // it to the LSPCU immediately and on the 1hz timer.
  enable_autocharge_.data = req.data;
  mcu_charge_enable_pub_.publish(enable_autocharge_);

  res.data = true;
  return true;
}

bool BoxerHardware::setEStopCallback(cpr_common_srvs::Bool::Request &req, cpr_common_srvs::Bool::Response &res)
{
  // Set the requested estop state and publish it to the LSPCU
  set_estop_.data = req.data;
  mcu_estop_pub_.publish(set_estop_);

  res.data = true;
  return true;
}

void BoxerHardware::setPlatformPubCallback(const ros::TimerEvent& event)
{
  mcu_charge_enable_pub_.publish(enable_autocharge_);
  mcu_estop_pub_.publish(set_estop_);
}

/**
 * @brief Determine if any velocity is commanded to either wheel of the robot.
 * This checks if either the left OR right wheel has been commanded any motion above 0.
 * @return True if any motion is commanded, false otherwise.
 */
bool BoxerHardware::isVelocityCommanded() const
{
  if (std::abs(joints_[0].velocity_command) >= std::numeric_limits<double>::epsilon() ||
      std::abs(joints_[1].velocity_command) >= std::numeric_limits<double>::epsilon())
  {
    return true;
  }
  return false;
}

/**
 * @brief Determine both the left and right wheel are commanded no motion..
 * This checks if both the left and right wheel has been commanded to cease movement.
 * @return True if commanded to stop, false otherwise.
 */
bool BoxerHardware::isVelocityStopped() const
{
  if (std::abs(joints_[0].velocity_command) <= std::numeric_limits<double>::epsilon() &&
      std::abs(joints_[1].velocity_command) <= std::numeric_limits<double>::epsilon())
  {
    return true;
  }
  return false;
}

}  // namespace boxer_base
