/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <diagnostic_updater/diagnostic_updater.h>
#include "flyer_controller/controller_status.h"
#include "flyer_controller/controller_cmd.h"
#include "std_msgs/Bool.h"
#include "starmac_msgs/OperatorCommands.h"

#include <sensor_msgs/Joy.h>

namespace flyer_controller
{
// Button definitions
// TRIGGER
const int BTN_TRIGGER = 0;
const int BTN_PROCEED = 8;
const int BTN_CANCEL = 6;
const int BTN_ESTOP = 11;
// Standby Mode
const int BTN_STANDBY_GO_OPERATIONAL = 2;
// Operational Mode
const int BTN_OPERATIONAL_MODE_ATTITUDE = 2;
const int BTN_OPERATIONAL_MODE_HOVER = 3;

namespace TeleopTypes
{
enum TeleopStates
{
  ERROR = 0, OFF = 1, INITIALIZE = 2, STANDBY = 3, OPERATIONAL = 4
};
typedef TeleopStates TeleopState;

}

using namespace TeleopTypes;
using namespace std;

class TeleopFlyer : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Publishers
  ros::Publisher oper_cmd_pub;
  ros::Publisher controller_cmd_pub;
  ros::Publisher motor_enable_pub;
  ros::Publisher estop_pub;
  // Subscribers
  ros::Subscriber joy_sub;
  ros::Subscriber controller_status_sub;
  // Timer
  ros::Timer republish_timer;
  // Members
  bool got_first_joy;
  sensor_msgs::Joy prev_joy;
  sensor_msgs::Joy latest_joy;
  ros::Time latest_joy_time;

  // Diagnostic Updater
  diagnostic_updater::Updater diag_updater;

  // Parameters
  double joy_republish_rate; // [Hz] rate to republish Joy messages at
  double max_interval; // [s] max time between received Joy messages
  bool use_udp; // whether to ask for unreliable transport (UDP) in Joy subscriber
  double pitch_deadband; // joystick units, 0.1 = 10% of half-range
  double roll_deadband; // joystick units, 0.1 = 10% of half-range
  double yaw_deadband; // joystick units, 0.1 = 10% of half-range

  // Members
  TeleopState state;
  bool command_pending; // is there a command pending?
  string pending_command; // the pending command
  bool lost_joystick; // Latches true if no joystic message seen for too long
  double last_trigger_duration;
  controller_status latest_controller_status;
  double max_joystick_dt; // maximum amount of time seen between joystick messages
  bool motors_on;

public:
  TeleopFlyer() :
    got_first_joy(false),
        diag_updater(), //
        joy_republish_rate(20), max_interval(2.0),
        use_udp(true), //
        pitch_deadband(0.05), roll_deadband(0.05),
        yaw_deadband(0.1), //
        state(OFF), command_pending(false), pending_command(""), lost_joystick(false), last_trigger_duration(0),
        max_joystick_dt(0), motors_on(false)
  {
  }

  void onInit()
  {
    nh = getMTNodeHandle();
    nh_priv = getMTPrivateNodeHandle();

    // Diagnostics
    diag_updater.add("TeleopFlyer Status", this, &TeleopFlyer::diagnostics);
    diag_updater.setHardwareID("none");
    diag_updater.force_update();
    // Parameters
    nh_priv.param("joy_republish_rate", joy_republish_rate, joy_republish_rate);
    nh_priv.param("max_interval", max_interval, max_interval);
    nh_priv.param("use_udp", use_udp, use_udp);
    nh_priv.param("pitch_deadband", pitch_deadband, pitch_deadband);
    nh_priv.param("roll_deadband", roll_deadband, roll_deadband);
    nh_priv.param("yaw_deadband", yaw_deadband, yaw_deadband);

    // Publishers
    oper_cmd_pub = nh_priv.advertise<starmac_msgs::OperatorCommands> ("operator_cmds", 1);
    //controller_cmd_pub = nh_priv.advertise<controller_cmd> ("controller_cmd", 1, true);
    motor_enable_pub = nh_priv.advertise<std_msgs::Bool> ("motor_enable", 1);
    estop_pub = nh_priv.advertise<std_msgs::Bool> ("estop", 1);
    // Subscribers
    ros::TransportHints thints;
    if (use_udp)
    {
      thints.udp();
    }
    else
    {
      thints.tcp().tcpNoDelay();
    }
    joy_sub = nh_priv.subscribe("joy", 10, &TeleopFlyer::joyCallback, this, thints);
    controller_status_sub = nh.subscribe("controller/status", 10, &TeleopFlyer::controllerStatusCallback, this); //,
    //                                         ros::TransportHints().unreliable().tcpNoDelay());
    // Timers
    republish_timer = nh.createTimer(ros::Duration(1 / joy_republish_rate), &TeleopFlyer::republishCallback, this);
  }

  std::string stateToString(TeleopState t)
  {
    switch (t)
    {
      case OFF:
        return "OFF";
      case ERROR:
        return "ERROR";
      case INITIALIZE:
        return "INITIALIZE";
      case STANDBY:
        return "STANDBY";
      case OPERATIONAL:
        return "OPERATIONAL";
    }
    return "Unknown";
  }

private:

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    int status;
    status = ((state != ERROR && !lost_joystick) ? int(diagnostic_msgs::DiagnosticStatus::OK)
        : int(diagnostic_msgs::DiagnosticStatus::ERROR));
    stat.summary(status, stateToString(state));
    stat.add("Command Pending", command_pending);
    stat.add("Pending Command", pending_command);
    stat.add("Lost Joystick", lost_joystick);
    stat.add("Trigger Duration", last_trigger_duration);
    stat.add("Max time between joystick messages", max_joystick_dt);
  }

  void controllerStatusCallback(const controller_statusConstPtr& msg)
  {
    state = TeleopState(msg->state); // make teleop's state match that of Controller
    latest_controller_status = *msg;
  }

  void estop()
  {
    static bool issued_warning = false;
    std_msgs::BoolPtr estop_msg(new std_msgs::Bool);
    estop_msg->data = true;
    estop_pub.publish(estop_msg);
    if (not issued_warning)
    {
      ROS_WARN("E-STOP triggered");
      issued_warning = true;
    }
  }

  void joyCallback(const sensor_msgs::JoyConstPtr msg)
  {
    //ROS_DEBUG("Joy callback called");
    ros::Time rightnow = ros::Time::now();
    //latest_joy = msg.get();
    if (msg->axes.size() == 0)
    {
      // didn't get a useful message, so ignore it
      NODELET_WARN_STREAM_THROTTLE(0.5, "Received joystick message with zero-size axes array, ignoring...");
      return;
    }
    prev_joy = latest_joy;
    latest_joy = *msg;
    if (not got_first_joy)
    {
      got_first_joy = true;
      ROS_INFO("Got first joystick message");
    }
    else
    {
      double dT = (rightnow - latest_joy_time).toSec();
      max_joystick_dt = max(max_joystick_dt, dT);
      if (dT > max_interval and (state == OPERATIONAL))
      {
        ROS_ERROR_STREAM("Time (" << dT << " s) between Joy messages exceeded allowable (" << max_interval << ")");
        lost_joystick = true;
      }

      if (state == OPERATIONAL)
      {
        // Look for motor start/stop
        // TODO
        if (not motors_on and //
            (latest_joy.axes[2] <= -0.95) and (latest_joy.axes[3] <= -0.95) // full right yaw and full min altitude
        )
        {
          motors_on = true;
          NODELET_INFO("Motors ON");
        }

        if (motors_on and //
            (latest_joy.axes[2] >= 0.95) and (latest_joy.axes[3] <= -0.95) // full left yaw and full min altitude
        )
        {
          motors_on = false;
          NODELET_INFO("Motors OFF");
        }
      }
      else
      {
        motors_on = false;
      }

      latest_joy_time = rightnow;
    }

  }

  void republishCallback(const ros::TimerEvent& e)
  {
    ROS_DEBUG_STREAM("Timer callback triggered " << e.current_real.toSec());
    if (got_first_joy)
    {
      starmac_msgs::OperatorCommandsPtr opercmd_msg_ptr(new starmac_msgs::OperatorCommands);

      double roll_in = double(-latest_joy.axes[0]);
      double pitch_in = double(-latest_joy.axes[1]);
      double yaw_in = double(-latest_joy.axes[2]);
      double roll_sign = (roll_in > 0) ? 1.0 : -1.0;
      double pitch_sign = (pitch_in > 0) ? 1.0 : -1.0;
      double yaw_sign = (yaw_in > 0) ? 1.0 : -1.0;
      opercmd_msg_ptr->roll_cmd = roll_sign * roll_in > roll_deadband ? (max(-1.0, min(1.0, roll_in)) - roll_sign
          * roll_deadband) / (1.0 - roll_deadband) : 0.0;
      opercmd_msg_ptr->pitch_cmd = pitch_sign * pitch_in > pitch_deadband ? (max(-1.0, min(1.0, pitch_in)) - pitch_sign
          * pitch_deadband) / (1.0 - pitch_deadband) : 0.0;
      opercmd_msg_ptr->alt_cmd = (min(1.0, max(-1.0, double(latest_joy.axes[3]))) + 1.0) / 2.0; // clip, offset, scale to 0..1 range

      if (opercmd_msg_ptr->alt_cmd > 0.05)
      {
        opercmd_msg_ptr->yaw_cmd = yaw_sign * yaw_in > yaw_deadband ? (max(-1.0, min(1.0, yaw_in)) - yaw_sign
            * yaw_deadband) / (1.0 - yaw_deadband) : 0.0;
      }
      else
      {
        opercmd_msg_ptr->yaw_cmd = 0; // don't issue yaw command if alt lever is bottomed out, user probably wants to turn on/off motors
      }
      opercmd_msg_ptr->motors_on = motors_on;
      oper_cmd_pub.publish(opercmd_msg_ptr);
    }
    else
    {
      NODELET_WARN_STREAM_THROTTLE(10, "No joystick messages received yet - if this message recurs, check launchfile configuration");
      // nothing to republish yet
      return;
    }
    diag_updater.update();
    ros::Time now = ros::Time::now();
    double joy_interval = (now - latest_joy_time).toSec();
    bool joystick_is_recent = (not got_first_joy) or (joy_interval < max_interval);
    std_msgs::BoolPtr mtr_enable_msg(new std_msgs::Bool);
    if (lost_joystick)
    {
      estop();
      mtr_enable_msg->data = false;
      motor_enable_pub.publish(mtr_enable_msg);
    }
    else
    {
      if (joystick_is_recent)
      {
        if (state == OPERATIONAL)
        {
          mtr_enable_msg->data = motors_on;
          motor_enable_pub.publish(mtr_enable_msg);
        }
        else
        {
          mtr_enable_msg->data = false;
          motor_enable_pub.publish(mtr_enable_msg);
        }
      }
      else
      {
        if (got_first_joy and (state == OPERATIONAL)) // can't lose it until we've first had it

        {
          lost_joystick = true;
          ROS_ERROR_STREAM("Too large (" << joy_interval << " s) an interval between joystick messages!");
          mtr_enable_msg->data = false;
          motor_enable_pub.publish(mtr_enable_msg);
        }
      }
    }
  }

};
PLUGINLIB_DECLARE_CLASS(flyer_controller, TeleopFlyer, flyer_controller::TeleopFlyer, nodelet::Nodelet)
;

}
// namespace

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "teleop_flyer");
//  flyer_controller::TeleopFlyer teleop_flyer;
//
//  ros::spin();
//
//  return 0;
//}
