/**
Software License Agreement (BSD)

\file      boxer_base.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2016, Rockwell Automation Technologies, Inc., All rights reserved.

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

#include <memory>
#include <string>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "controller_manager/controller_manager.h"
#include "boxer_base/boxer_diagnostic_updater.h"
#include "boxer_base/boxer_hardware.h"
#include "ros/ros.h"
// TODO(modriscoll) CORE-8051 Restore when bind() crash resolved.
// #include "rosserial_server/serial_session.h"
// #include "rosserial_server/udp_socket_session.h"
#include "urg_node/urg_node_driver.h"

// TODO(modriscoll) CORE-8051 Restore when bind() crash resolved.
// using boost::asio::ip::udp;
// using boost::asio::ip::address;
typedef boost::chrono::steady_clock time_source;

void controlThread(ros::Rate rate, boxer_base::BoxerHardware* robot, controller_manager::ControllerManager* cm)
{
  time_source::time_point last_time = time_source::now();

  while (ros::ok())
  {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    robot->copyJointsFromHardware();
    cm->update(ros::Time::now(), elapsed);

    // TODO(mpurvis): LIDAR speed limiting.
    robot->updateDriveState();
    robot->publishDriveFromController();
    robot->publishLiftPositionFromController();
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "boxer_node");
  boxer_base::BoxerHardware boxer;

  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("");
  controller_manager::ControllerManager cm(&boxer, controller_nh);
  boost::thread control_thread(boost::bind(controlThread, ros::Rate(50), &boxer, &cm));

  // Boost ASIO IO service for rosserial sessions.
  boost::asio::io_service io_service;

  // TODO(modriscoll) CORE-8051 Restore when bind() crash resolved.
  // std::unique_ptr<rosserial_server::UdpSocketSession> lspcu_session;
  // std::unique_ptr<rosserial_server::UdpSocketSession> tdcu_session;

  // Dual Ethernet-connected MCUs on the BOXER Beta and beyond.
  // lspcu_session.reset(new rosserial_server::UdpSocketSession(
    // io_service,
    // udp::endpoint(address::from_string("10.253.253.1"), 11411),
    // udp::endpoint(address::from_string("10.253.253.2"), 11411)));
  // lspcu_session->set_require_param("~require_lspcu");
  // tdcu_session.reset(new rosserial_server::UdpSocketSession(
    // io_service,
    // udp::endpoint(address::from_string("10.253.253.1"), 11412),
    // udp::endpoint(address::from_string("10.253.253.3"), 11412)));
  //  tdcu_session->set_require_param("~require_tdcu");

  // Create the laser scanner node
  ros::CallbackQueue laser_service_queue;
  ros::NodeHandle lnh;
  lnh.setCallbackQueue(&laser_service_queue);
  ros::NodeHandle laser_nh("~/laser");
  laser_nh.setCallbackQueue(&laser_service_queue);
  ros::AsyncSpinner laser_spinner_(2, &laser_service_queue);
  urg_node::UrgNode urg(lnh, laser_nh);
  urg.run();

  // Create diagnostic updater, to update itself on a ROS timer.
  boxer_base::BoxerDiagnosticUpdater boxer_diagnostic_updater;

  // Service ASIO and ROS callbacks.
  boost::thread asio_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  // Start async spinners before ros spin.
  laser_spinner_.start();

  ros::spin();

  // Stop async spinners after ros exists.
  laser_spinner_.stop();

  return 0;
}
