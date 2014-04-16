#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, UC Regents
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the University of California nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('simulated_quadrotor')
import rospy
import numpy as np
from math import sin, cos, radians, degrees
import tf.transformations as tft
from tf import TransformListener

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from flyer_controller.msg import control_mode_output

from starmac_roslib.pid import PidController

from simulated_quadrotor.models.simple_nonlinear import SimpleNonlinearModel
from simulated_quadrotor.models.linear import LinearModel

class SimAdapter(object):

    def __init__(self):
        pass
        
    def start(self):
        rospy.init_node('sim_adapter')
        self.init_params()
        self.init_state()
        self.init_vars()
        self.init_publishers()
        self.init_subscribers()
        self.init_timers()
        rospy.spin()
        
    def init_params(self):
        self.param_model = rospy.get_param("~model", "simple_nonlinear")
        self.rate = rospy.get_param("~rate", 40)
        self.publish_state_estimate = rospy.get_param("~publish_state_estimate", True)
        self.publish_odometry_messages = rospy.get_param("~publish_odometry", True)
        self.sim_odom_topic = rospy.get_param("~sim_odom_topic", "odom")
        
    def init_state(self):
        if self.param_model == "simple_nonlinear":
            self.model = SimpleNonlinearModel()
        elif self.param_model == "linear":
            self.model = LinearModel()
        else:
            rospy.logerror("Model type '%s' unknown" % self.model)
            raise Exception("Model type '%s' unknown" % self.model)

    def init_vars(self):
        self.latest_cmd_msg = control_mode_output()
        self.motor_enable = False
        self.thrust_cmd = 0.0
        self.tfl = TransformListener()
        self.T_vicon_imu = None
        self.T_imu_vicon = None
        self.T_enu_ned = None
        
    def init_publishers(self):
        # Publishers
        if self.publish_odometry_messages: 
            self.pub_odom = rospy.Publisher(self.sim_odom_topic, Odometry)
        if self.publish_state_estimate:
            self.pub_state = rospy.Publisher('state', TransformStamped)
        
    def init_subscribers(self):
        # Subscribers
        self.control_input_sub = rospy.Subscriber('controller_mux/output', control_mode_output, self.control_input_callback)
        self.motor_enable_sub = rospy.Subscriber('teleop_flyer/motor_enable', Bool, self.motor_enable_callback)
      
    def init_timers(self):
        self.simulation_timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.simulation_timer_callback)
                                      
    # Subscriber callbacks:
    def control_input_callback(self, msg):
        rospy.logdebug('Current command is: ' + str(msg))
        self.latest_cmd_msg = msg
    
    def motor_enable_callback(self, msg):
        if msg.data != self.motor_enable:
            #rospy.loginfo('Motor enable: ' + str(msg.data))
            self.motor_enable = msg.data
    
    # Timer callbacks:
    def simulation_timer_callback(self, event):
        if False:
            print event.__dict__
#            print 'last_expected:        ', event.last_expected
#            print 'last_real:            ', event.last_real
#            print 'current_expected:     ', event.current_expected
#            print 'current_real:         ', event.current_real
#            print 'current_error:        ', (event.current_real - event.current_expected).to_sec()
#            print 'profile.last_duration:', event.last_duration.to_sec()
#            if event.last_real:
#                print 'last_error:           ', (event.last_real - event.last_expected).to_sec(), 'secs'
#            print
        if event.last_real is None:
            dt = 0.0
        else:
            dt = (event.current_real - event.last_real).to_sec()
            self.update_controller(dt)
            self.update_state(dt)
        #rospy.loginfo("position: " + str(self.position) + " velocity: " + str(self.velocity) + " thrust_cmd: " + str(self.thrust_cmd))
        if self.publish_odometry_messages:
            self.publish_odometry()
        if self.publish_state_estimate:
            self.publish_state(event.current_real)
        
    def update_state(self, dt):
        # The following model is completely arbitrary and should not be taken to be representative of
        # real vehicle performance!
        # But, it should be good enough to test out control modes etc.
        self.model.update(dt)
            
    def update_controller(self, dt):
        lcm = self.latest_cmd_msg
        self.model.set_input(lcm.motors_on, lcm.roll_cmd, lcm.pitch_cmd, lcm.direct_yaw_rate_commands, 
                  lcm.yaw_cmd, lcm.yaw_rate_cmd, lcm.direct_thrust_commands, lcm.alt_cmd, lcm.thrust_cmd)
        #rospy.loginfo("thrust_cmd = %f, dt = %f" % (self.thrust_cmd, dt))
                    
    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "/ned"
        odom_msg.child_frame_id = "/simflyer1/flyer_imu"
        oppp = odom_msg.pose.pose.position
        oppp.x, oppp.y, oppp.z  = self.model.get_position()
        ottl = odom_msg.twist.twist.linear
        ottl.x, ottl.y, ottl.z = self.model.get_velocity()
        oppo = odom_msg.pose.pose.orientation
        oppo.x, oppo.y, oppo.z, oppo.w = self.model.get_orientation()
        otta = odom_msg.twist.twist.angular
        otta.x, otta.y, otta.z = self.model.get_angular_velocity()
        self.pub_odom.publish(odom_msg)
        
    def publish_state(self, t):
        state_msg = TransformStamped()
        t_ned_imu = tft.translation_matrix(self.model.get_position())
        R_ned_imu = tft.quaternion_matrix(self.model.get_orientation())
        T_ned_imu = tft.concatenate_matrices(t_ned_imu, R_ned_imu)
        #rospy.loginfo("T_ned_imu = \n" + str(T_ned_imu))
        if self.T_imu_vicon is None:
            # grab the static transform from imu to vicon frame from param server:
            frames = rospy.get_param("frames", None)
            ypr = frames['vicon_to_imu']['rotation']
            q_vicon_imu = tft.quaternion_from_euler(*[radians(x) for x in ypr], axes='rzyx') # xyzw
            R_vicon_imu = tft.quaternion_matrix(q_vicon_imu)
            t_vicon_imu = tft.translation_matrix(frames['vicon_to_imu']['translation'])
#            rospy.loginfo(str(R_vicon_imu))
#            rospy.loginfo(str(t_vicon_imu))
            self.T_vicon_imu = tft.concatenate_matrices(t_vicon_imu, R_vicon_imu)
            self.T_imu_vicon = tft.inverse_matrix(self.T_vicon_imu)
            self.T_enu_ned = tft.euler_matrix(radians(90), 0, radians(180), 'rzyx')
            rospy.loginfo(str(self.T_enu_ned))
            rospy.loginfo(str(self.T_imu_vicon))
            #rospy.loginfo(str(T_vicon_imu))
        # we have /ned -> /imu, need to output /enu -> /vicon:
        T_enu_vicon = tft.concatenate_matrices(self.T_enu_ned, T_ned_imu, self.T_imu_vicon )
        state_msg.header.stamp  = t
        state_msg.header.frame_id = "/enu"
        state_msg.child_frame_id = "/simflyer1/flyer_vicon"
        stt = state_msg.transform.translation
        stt.x, stt.y, stt.z = T_enu_vicon[:3,3]
        stro = state_msg.transform.rotation
        stro.x, stro.y, stro.z, stro.w = tft.quaternion_from_matrix(T_enu_vicon)
        
        self.pub_state.publish(state_msg)
        
    def get_transform(self, tgt, src):
        t = self.tfl.getLatestCommonTime(tgt, src)
        t_src_tgt, q_src_tgt = self.tfl.lookupTransform(tgt, src, t)
        T_src_tgt =self.tfl.fromTranslationRotation(t_src_tgt, q_src_tgt)
        return T_src_tgt 
        
if __name__ == "__main__":
  self = SimAdapter()
  self.start()