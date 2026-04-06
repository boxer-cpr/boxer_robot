/**
Software License Agreement (BSD)

\file      boxer_hardware.h
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

#ifndef BOXER_BASE_BOXER_HARDWARE_H
#define BOXER_BASE_BOXER_HARDWARE_H

#include "boost/thread.hpp"

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "realtime_tools/realtime_publisher.h"

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

#include "cpr_common_srvs/Bool.h"
#include "boxer_msgs/Drive.h"
#include "boxer_msgs/LiftPosition.h"
#include "boxer_msgs/MotorFeedback.h"
#include "boxer_msgs/Status.h"
#include "sensor_msgs/JointState.h"


namespace boxer_base
{

class BoxerHardware : public hardware_interface::RobotHW
{
public:
  BoxerHardware();
  void copyJointsFromHardware();
  void updateDriveState();
  void publishDriveFromController();
  void publishLiftPositionFromController();

private:
  void lspcuStatusCallback(const boxer_msgs::Status::ConstPtr& msg);
  void leftFeedbackCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg);
  void rightFeedbackCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg);
  void liftFeedbackCallback(const boxer_msgs::MotorFeedback::ConstPtr& msg);

  bool homeServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool raiseServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool setCharging(cpr_common_srvs::Bool::Request &req, cpr_common_srvs::Bool::Response &res);
  bool setEStopCallback(cpr_common_srvs::Bool::Request &req, cpr_common_srvs::Bool::Response &res);

  void setPlatformPubCallback(const ros::TimerEvent& event);
  bool poweroffServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void autoHomeTimerCallback(const ros::TimerEvent& event);

  bool isVelocityCommanded() const;
  bool isVelocityStopped() const;

  ros::NodeHandle nh_;

  ros::Subscriber lspcu_status_sub_;
  ros::Subscriber left_feedback_sub_;
  ros::Subscriber right_feedback_sub_;
  ros::Subscriber lift_feedback_sub_;

  ros::Publisher mcu_charge_enable_pub_;
  ros::Publisher mcu_poweroff_pub_;
  ros::Publisher mcu_estop_pub_;
  ros::Publisher platform_emergency_stop_pub_;
  ros::Publisher platform_safety_stop_pub_;

  ros::ServiceServer home_service_;
  ros::ServiceServer force_raise_service_;
  ros::ServiceServer charge_service_;
  ros::ServiceServer shutdown_service_;
  ros::ServiceServer estop_service_;

  ros::ServiceClient footprint_check_service_;

  realtime_tools::RealtimePublisher<boxer_msgs::Drive> cmd_drive_pub_;
  realtime_tools::RealtimePublisher<boxer_msgs::LiftPosition> lift_pos_pub_;

  ros::Timer platform_pub_timer_;
  ros::Timer autohome_timer_;

  /**
   * @brief State defines for which mode the robot should be in.
   * Velocity: This is the conventional movement mode or stopped but not holding position.
   * Position: This is a position converge and hold mode where the robot keeps it's position as
   * accurately as possible.
   */
  enum class DriveState
  {
    VELOCITY,
    POSITION
  };

  /**
   * @brief Position hold states
   * DISABLED: Normal operation, generally stopped idle or velocity mode.
   * WAIT_ENABLE: After a stop condition is found wait to ensure we don't engage hold position to soon.
   * ENABLE: Read the current position and command hold.
   * POST_ENABLE: Continue to hold position for the defined period, after which go to backEMF breaking.
   */
  enum class HoldState : uint8_t
  {
    DISABLED = 0,
    WAIT_ENABLE,
    ENABLE,
    POST_ENABLE
  };

  ros::Time position_hold_time_;
  HoldState hold_state_;
  DriveState drive_state_;
  float wait_enable_timeout_;
  float hold_position_timeout_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  bool estop_;
  bool safety_stop_;

  // These are mutated on the controls thread only.
  // Boxer has: 2 traction wheels + 2 caster (dummy) wheels.
  // and a lift position joint.
  struct Joint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;
    double position_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0), position_command(0)
    {
    }
  }
  joints_[3];

  // If the MCU resets or rolls over, its travel numbers return to zero. To have odometry
  // that is robust to this, we detect the travel numbers jumping back to zero and add
  // the "old" travel number to this offset.
  double travel_offset_[2];
  static constexpr double TRAVEL_OFFSET_THRESHOLD_RADS = 6.28;

  float lower_limit_position_;
  float upper_limit_position_;

  // This pointer is set from the ROS thread.
  boxer_msgs::Status::ConstPtr last_lspcu_status_msg_;
  boxer_msgs::MotorFeedback::ConstPtr left_feedback_msg_;
  boxer_msgs::MotorFeedback::ConstPtr right_feedback_msg_;
  boxer_msgs::MotorFeedback::ConstPtr lift_feedback_msg_;
  boost::mutex feedback_msg_mutex_;
  boxer_msgs::LiftPosition::_home_lift_type home_requested_;
  std_msgs::Bool enable_autocharge_;
  std_msgs::Bool set_estop_;

  // A separate spinner is needed for the service
  // server "charge_service_" providing "charging_set"
  // service. This is due to the velocity interface
  // needing to call the service to disable charging
  // when the robot wants to move. Without
  // the ASyncSpinner the service call within
  // the velocity interface never gets responded to
  // as it's in the same ROS thread as velocity interface.
  ros::CallbackQueue charging_service_queue_;
  ros::NodeHandle charging_service_nh_;
  ros::AsyncSpinner charging_spinner_;
};

}  // namespace boxer_base

#endif  // BOXER_BASE_BOXER_HARDWARE_H
