#!/usr/bin/env python
#
# Software License Agreement (BSD)
#
# \file      net_logger_node.py
# \authors   Justin Godson <jgodson@clearpathrobotics.com>
#            Mike O'Driscoll <modriscoll@clearpathrobotics.com>
# \copyright Copyright (c) 2016, Rockwell Automation Technologies, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Rockwell Automation Technologies nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import re
import raven
import rospy
import socket
import select
import logging
import time
from std_msgs.msg import Header
from rosgraph_msgs.msg import Log
from collections import deque


LOG_LEVEL_MAP = {
  Log.DEBUG: logging.DEBUG,
  Log.INFO: logging.INFO,
  Log.WARN: logging.WARN,
  Log.ERROR: logging.ERROR,
  Log.FATAL: logging.CRITICAL
}


# --------------------------------------------------------------------------------
# Helper Functions
# --------------------------------------------------------------------------------
class LineParser(object):
    def __init__(self):
        # Create regex compiled search
        self.search_regex = re.compile(r'^\[(.+)\] (.+):(\d+) (.+)$')

    def process_log_line(self, line, node_name):
        """Process a log line and format it into a rosgraph message.
        :param line: The line to process for level, message, lineno
        :type line: String
        :param node_name: The node name for the rosout message.
        :type node_name: String
        """

        match = self.search_regex.match(line)
        if not match:
            return None  # evaluates to false

        level, file_name, line_no, msg = match.groups()
        return Log(header=Header(stamp=rospy.Time.now()),
                   level=getattr(Log, level),
                   file=file_name,
                   line=int(line_no),
                   name=node_name,
                   msg=msg)


# --------------------------------------------------------------------------------
# NetLoggerNode
# --------------------------------------------------------------------------------
class NetLoggerNode:
    def __init__(self, name_prefix='Firmware'):
        """Create an firmware logging node class
        :param name_prefix: The ROS node name prefix to use.
        :type name_prefix: string
        """
        self.cmd_port = rospy.get_param('~cmd_port', 11414)
        self.snapshot_port = rospy.get_param('~snapshot_port', 11415)
        self.logger_port = rospy.get_param('~logger_port', 11413)

        self.logger_nodes = rospy.get_param('~loggers', dict())
        if not self.logger_nodes:
            rospy.logerror('No loggers configured for firmware logging.')

        # Configure socket and logger
        self.log_sock = socket.socket(socket.AF_INET,  # Internet
                                      socket.SOCK_DGRAM)  # UDP
        self.log_sock.bind(("", self.logger_port))

        for logger_name in self.logger_nodes:
            self.logger_nodes[logger_name]['logger'] = logging.getLogger(logger_name)
            self.logger_nodes[logger_name]['logger'].setLevel(logging.DEBUG)

        self.log_buffer = deque(maxlen=50)

        self.log_debug = rospy.get_param('~enable_debug_publishing')
        # Create sentry client to capture errors with this python class itself.
        try:
            self.sclient_qa = self.enable_sentry_client(rospy.get_param('~sentry_client_dsn_qa'), True)
        except KeyError:
            rospy.logwarn('Logger code sentry client disabled')

        self.line_parser = LineParser()

        self.rosout_pub = rospy.Publisher('rosout', Log, queue_size=1000)

    def enable_sentry_client(self, dsn_path, sys_hook=False):
        """Create a sentry client instance.
        :param dsn_path: The sentry DSN to report to.
        :type dsn_path: string
        :param sys_hook: Wether the created sentry client should hook for exceptions.
        :type sys_hook: bool
        """
        # Setup sentry reporting client
        client = raven.Client(
          dsn=dsn_path,
          # TODO pass along the version application
          exclude_paths=['sentry', 'raven'],
          install_sys_hook=sys_hook,
          string_max_length=10000  # Increase max str len so we can get the full encoded tombstone
        )

        return client

    def log_message(self, msg):
        """This node is bypassing normal ROS logging, by publishing to /rosout on it's own, it bypasses
        normal ros logging, so it needs to be passed into the logger to the ROS logger will put it into the log
        files in ~/.ros/log/....
        :param msg: The message to log out to the ros log.
        :type msg: A rosgraph_msg.Log class
        """
        logger = self.logger_nodes[msg.name]['logger']
        logger.log(LOG_LEVEL_MAP[msg.level], msg.msg)

    def process_log(self, data, addr):
        """Receive socket data coming in from the board, and publish
        to /rosout.
        :param data: The log line coming in over the socket.
        :type pub: A rospy.Publisher handle.
        :param addr: The address the data came from
        :type addr: IP address as a string.
        """
        for logger_name in self.logger_nodes:
            if self.logger_nodes[logger_name]['target_ip'] == addr[0]:

                result = self.line_parser.process_log_line(data, logger_name)

                # Log the received if we got a result match process it
                # otherwise send the raw socket data to the logger.
                if result:
                    self.log_message(result)
                else:
                    self.logger_nodes[logger_name]['logger'].debug(data)

                if not result:
                    rospy.logdebug('Received line does not match expected format')
                    return

                # Send error and context to sentry along with publishing
                if result.level == Log.FATAL:
                    if self.logger_nodes[logger_name]['sentry_client_dsn']:
                        client = self.enable_sentry_client(self.logger_nodes[logger_name]['sentry_client_dsn'])
                        client.captureMessage(result.msg,
                                              data={'fingerprint': [result.file, result.line]},
                                              extra={'context_lines': list(self.log_buffer)})

                if result.level == Log.DEBUG:
                    if self.log_debug:
                        self.rosout_pub.publish(result)
                else:
                    self.rosout_pub.publish(result)

                # Store the last few lines in a ring
                # buffer to pass out on error level
                # for context to the event along with the time.
                self.log_buffer.append(time.strftime("[%x %H:%M:%S] ") + data)

    def run(self):
        # Continue to read from socket until shutdown
        try:
            while not rospy.is_shutdown():
                # Select the sockets to we can listen to multiple ports
                # continuously to get the recv notification.
                # The timeout is so that the thread has an opportunity
                # to check if it needs to exit.
                try:
                    readable, _, _ = select.select([self.log_sock], [], [], 5.0)
                    for sock in readable:
                        data, addr = sock.recvfrom(1024)
                        self.process_log(data, addr)
                except socket.error as e:
                    rospy.logdebug('Unable to receive data from socket %s' % e.message)
                except select.error as e:
                    rospy.logdebug('Select failed: %s' % e.message)

        except KeyboardInterrupt:
            pass  # No sentry reports generated
        except select.error as e:
            rospy.logdebug('Select failed: %s' % e.message)
