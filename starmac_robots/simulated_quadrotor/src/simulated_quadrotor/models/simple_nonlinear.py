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

from math import sin, cos, radians, degrees
import numpy as np

import tf.transformations as tft

from starmac_roslib.pid import PidController

from model_base import ModelBase

from simulated_quadrotor.msg import SimDebug


GRAVITY = 9.81 # m/s/s
MASS = 1.48 # kg
FUDGE = 1500
THRUST_SCALING = MASS*GRAVITY/FUDGE
LINEAR_DRAG = 1.0 # N/(m/s)
QUADRATIC_DRAG = 0.3 # N/(m/(s^2))
EPS = 0.0001
IMU_HEIGHT = 0.17 # how far off the ground the IMU frame is when the vehicle is landed

# Identified closed-loop attitude dynamics linear model:
n0 = 40.0
d1 = 8.0
d0 = 80.0

# Angular velocity filter parameters
QUAT_FILT_A = 0.2
QUAT_FILT_B = 0.8

def clip(x, lb, ub):
    if x > ub:
        return ub, True
    elif x < lb:
        return lb, True
    else:
        return x, False
    
def limitSlewRate(x, x_prev, dt, rate_limit):
    limit = rate_limit * dt;
    return max(x_prev - limit, min(x_prev + limit, x));
  

class SimpleNonlinearModel(ModelBase):
    """
    Simple nonlinear model of arbitrary quadrotor type vehicle.
    13 states:
    index    quantity                    units
    0        x position                    m
    1        y position                    m
    2        z position                    m
    
    3        x velocity                    m/s
    4        y velocity                    m/s
    5        z velocity                    m/s
    
    6        orientation quaternion (w)   
    7        orientation quaternion (x)   
    8        orientation quaternion (y)   
    9        orientation quaternion (z)
       
    10       angular velocity (x)          rad/s
    11       angular velocity (y)          rad/s
    12       angular velocity (z)          rad/s
    
    4 inputs:
    index    quantity  units
    0        yaw       deg
    1        pitch     deg
    2        roll      deg
    3        altitude  m
    4        enable    (bool, True if motors are on)
    
    NOTE: transformations.py (tf.transformations) represents quaternions 
          as (x, y, z, w) tuples!
    """
    def __init__(self):
        self.x = np.zeros(13)
        self.x[6] = 1.0 # identity quaternion
        #self.u = np.zeros(5)
        self.q_dot_prev = np.zeros(4)
        # Grab parameters:
        self.alt_KP = rospy.get_param("~alt_KP", 600.0)
        self.alt_KI = rospy.get_param("~alt_KI", 400.0)
        self.alt_KD = rospy.get_param("~alt_KD", 600.0)
        #self.alt_KDD = rospy.get_param("~alt_KDD",50.0)
        self.alt_Ilimit = rospy.get_param("~alt_Ilimit", 400.0)
        #self.yaw_KP = rospy.get_param("~yaw_KP", 4.0)
        #self.yaw_KI = rospy.get_param("~yaw_KI", 0.0)
        #self.yaw_KD = rospy.get_param("~yaw_KD", 0.0)
        #self.yaw_Ilimit = rospy.get_param("~yaw_Ilimit", 50.0)
        self.nominal_thrust = rospy.get_param("~nominal_thrust", 1500)
        self.thrust_mult = rospy.get_param("~thrust_mult", 1.0)
        self.max_thrust = rospy.get_param("~max_thrust", 2100)
        temp = rospy.get_param("~periodic_accel_disturbance/vector", None)
        if temp is None:
            self.periodic_accel_disturbance_vector = np.zeros(3)
        else:
            self.periodic_accel_disturbance_vector = np.array(temp)
        self.periodic_accel_disturbance_period = rospy.get_param("~periodic_accel_disturbance/period", 10.0)
        self.periodic_accel_disturbance_duration = rospy.get_param("~periodic_accel_disturbance/duration", 1.0)
        self.pitch_bias = rospy.get_param("~pitch_bias", 0.0) # degrees
        self.roll_bias = rospy.get_param("~roll_bias", 0.0) # degrees
        self.roll_slew_rate_limit = rospy.get_param("~roll_slew_rate_limit", 10.0) # deg/s
        self.pitch_slew_rate_limit = rospy.get_param("~pitch_slew_rate_limit", 10.0) # deg/s
        temp = rospy.get_param("~gaussian_accel_noise_covariance", None) #[0.0]*9)
        if temp is not None:
            self.gaussian_accel_noise_covariance = np.array(temp).reshape(3,3)
        else:
            self.gaussian_accel_noise_covariance = None
        self.prev_orientation = (0.0, 0.0, 0.0, 1.0)
        self.prev_roll_cmd = 0.0
        self.prev_pitch_cmd = 0.0
        
        self.alt_pid = PidController(self.alt_KP, self.alt_KI, self.alt_KD, self.alt_Ilimit)
        self.t = 0
        
        self.debug_publisher = rospy.Publisher('~debug', SimDebug)
        
    def set_input(self, motors_on, roll_cmd, pitch_cmd, direct_yaw_rate_commands, 
                  yaw_cmd, yaw_rate_cmd, direct_thrust_commands, alt_cmd, thrust_cmd):
        self.motors_on = motors_on # bool
        self.roll_cmd = roll_cmd # deg
        self.pitch_cmd = pitch_cmd # deg
        self.direct_yaw_rate_commands = direct_yaw_rate_commands # bool
        self.yaw_cmd = yaw_cmd # deg
        self.yaw_rate_cmd = yaw_rate_cmd # deg/s
        self.direct_thrust_commands = direct_thrust_commands # bool
        self.alt_cmd = alt_cmd # m
        self.thrust_cmd = thrust_cmd # N
        
        
    def update(self, dt):
        # The following model is completely arbitrary and should not be taken to be representative of
        # real vehicle performance!
        # But, it should be good enough to test out control modes etc.
        self.t += dt
        #yaw_cmd, pitch_cmd, roll_cmd, alt_cmd, motor_enable = self.u
        cur_yaw, cur_pitch, cur_roll = tft.euler_from_quaternion(self.get_orientation(), 'rzyx')
        prev_yaw, prev_pitch, prev_roll = tft.euler_from_quaternion(self.prev_orientation, 'rzyx')
        # Feedback control: attitude angles track inputs exactly; altitude is input to
        # PID controller for thrust
        
        sim_debug_msg = SimDebug()
        sim_debug_msg.header.stamp = rospy.Time.now()
        
        if self.motors_on:
            if self.direct_thrust_commands:
                thrust = self.thrust_mult*self.thrust_cmd/THRUST_SCALING
            else:
                thrust = self.thrust_mult*(self.nominal_thrust + \
                  self.alt_pid.update(-self.get_position()[2] - self.alt_cmd, dt))
            thrust = min(self.max_thrust, thrust)
            ang_vel_inertial = self.x[10:13]
            R_inertial_body = tft.quaternion_matrix(self.get_orientation())[:3,:3]
            ang_vel_body = np.dot(R_inertial_body.T, ang_vel_inertial)
            # accelerations due to pitch/roll in body frame:
            A = np.array([[-d1, 1.0],[-d0, 0.0]])
            B = np.array([0, n0])
            # Roll:
            roll_cmd_slew_limited =  limitSlewRate(self.roll_cmd, self.prev_roll_cmd, dt, self.roll_slew_rate_limit)         
            self.prev_roll_cmd = roll_cmd_slew_limited
            roll_state_cur = np.array([cur_roll, ang_vel_body[0]])
            xdot = np.dot(A,roll_state_cur) + B*radians(roll_cmd_slew_limited)
            roll_state_new = roll_state_cur + xdot*dt #+ radians(self.roll_bias)  
            # Pitch:
            pitch_cmd_slew_limited =  limitSlewRate(self.pitch_cmd, self.prev_pitch_cmd, dt, self.pitch_slew_rate_limit)         
            self.prev_pitch_cmd = pitch_cmd_slew_limited
            pitch_state_cur = np.array([cur_pitch, ang_vel_body[1]])
            xdot = np.dot(A,pitch_state_cur) + B*radians(pitch_cmd_slew_limited)
            pitch_state_new = pitch_state_cur + xdot*dt #+ radians(self.pitch_bias)
                 
            pitch_clipped, pwas_clipped = clip(degrees(pitch_state_new[0]), -50, 50)
            roll_clipped, rwas_clipped = clip(degrees(roll_state_new[0]), -50, 50)
            # Yaw (no dynamics here for now -- tracks perfectly)
            yaw_cmd = self.yaw_cmd
        else:
            thrust = 0.0
            yaw_cmd = degrees(cur_yaw)
            pitch_clipped = degrees(cur_pitch)
            roll_clipped = degrees(cur_roll)
            roll_state_new = np.array([radians(roll_clipped), 0])
            pitch_state_new = np.array([radians(pitch_clipped), 0])
            pwas_clipped = rwas_clipped = False
            
        sim_debug_msg.thrust_in_counts = thrust
        
        # Propagate dynamics
        # Velocity
        accel_thrust = thrust*THRUST_SCALING/MASS
        #rospy.loginfo("accel_thrust = " + str(accel_thrust))
        if pwas_clipped or rwas_clipped:
            rospy.logwarn('Pitch and/or roll clipped! Pre-clip (commanded) values %f %f' % (self.pitch_cmd, self.roll_cmd))
        accel_body = np.array([-sin(radians(pitch_clipped))*accel_thrust, 
                              sin(radians(roll_clipped))*accel_thrust, 
                              0.0]) # vertical accel will be added in inertial frame
        # rotate body frame accelerations into inertial frame, and add drag and vertical forces:
        Ryaw = tft.euler_matrix(cur_yaw, 0, 0, 'rzyx')
        accel_inertial_lateral = np.dot(Ryaw[:3,:3], accel_body)
        cur_velocity = self.get_velocity()
        # add periodic disturbance:
        tmod = self.t % self.periodic_accel_disturbance_period
        if tmod <= self.periodic_accel_disturbance_duration:
            accel_disturbance = self.periodic_accel_disturbance_vector
        else:
            accel_disturbance = np.zeros(3)
        if self.direct_thrust_commands:
            tfactor = cos(radians(pitch_clipped))*cos(radians(roll_clipped))
        else:
            tfactor = 1.0 # asctec_adapter altitude controller accounts for cosine losses

        accel_inertial =    accel_disturbance + \
                            accel_inertial_lateral + \
                            np.array([# X:
                                      - LINEAR_DRAG*cur_velocity[0]
                                      - QUADRATIC_DRAG*np.sign(cur_velocity[0])*(cur_velocity[0]**2),
                                      # Y:
                                      - LINEAR_DRAG*cur_velocity[1]
                                      - QUADRATIC_DRAG*np.sign(cur_velocity[1])*(cur_velocity[1]**2),
                                      # Z:
                                      GRAVITY - accel_thrust*tfactor
                                      - LINEAR_DRAG*cur_velocity[2]
                                      - QUADRATIC_DRAG*np.sign(cur_velocity[2])*(cur_velocity[2]**2)])
                            
        #rospy.loginfo('accel_inertial: ' + str(accel_inertial))
        # Add Gaussian noise
        if self.gaussian_accel_noise_covariance is not None:
            accel_inertial += np.random.multivariate_normal((0,0,0), self.gaussian_accel_noise_covariance)
            
        sim_debug_msg.accel_inertial = accel_inertial
        velocity = cur_velocity + accel_inertial*dt
        #rospy.loginfo('velocity: ' + str(velocity))
       
        # Angles
        if self.get_position()[2] >= -IMU_HEIGHT and velocity[2] > 0:
            # On the ground..
            velocity = np.zeros(3)
            orientation = tft.quaternion_from_euler(radians(yaw_cmd), 0, 0, 'rzyx')
        else:
            # assume perfect yaw rate, and pitch and roll tracking:
            orientation = tft.quaternion_from_euler(radians(yaw_cmd), radians(pitch_clipped), radians(roll_clipped), 'rzyx')

        # Position
        position = self.get_position() + velocity*dt
        position[2] = min(-IMU_HEIGHT, position[2]) # can't go lower than the ground
        sim_debug_msg.position = position
            
        # Angular rate
        # from J. Diebel, "Representing attitude: Euler angles, unit quaternions, and rotation vectors," 2006
        q_cur = np.array((orientation[3],) + tuple(orientation[:3])) # convert to wxyz 
        q_prev = np.array((self.prev_orientation[3],) + tuple(self.prev_orientation[:3])) # convert to wxyz
        if dt > EPS:
            if q_cur[0]*q_prev[0] < 0:
                mult = -1
            else:
                mult = 1
            q_dot = QUAT_FILT_A*(mult*q_cur - q_prev)/dt + QUAT_FILT_B*self.q_dot_prev
        else:
            q_dot = np.zeros(4)  
        self.q_dot_prev = q_dot
        Wprime = np.array([[-q_cur[1],  q_cur[0],  q_cur[3], -q_cur[2]],
                           [-q_cur[2], -q_cur[3],  q_cur[0],  q_cur[1]],
                           [-q_cur[3],  q_cur[2], -q_cur[1],  q_cur[0]]])
        ang_vel_body = 2*np.dot(Wprime,q_dot)
        ang_vel_body[0] = roll_state_new[1]
        ang_vel_body[1] = pitch_state_new[1]
        R_inertial_body = tft.quaternion_matrix(orientation)[:3,:3]
        ang_vel_inertial = np.dot(R_inertial_body, ang_vel_body)
        # Assemble new state vector
        self.x = np.concatenate([position, velocity, q_cur, ang_vel_inertial])
        
        # Save for future finite differences
        self.prev_orientation = orientation[:] # be sure to copy not reference...
        
        self.debug_publisher.publish(sim_debug_msg)
            
    def get_position(self):
        return self.x[0:3]
            
    def get_velocity(self):
        return self.x[3:6]
            
    def get_orientation(self):
        """
        Return orientation quaternion in (x, y, z, w) convention
        """
        return tuple(self.x[7:10]) + (self.x[6],)
    
    def get_angular_velocity(self):
        return self.x[10:13]
            

        