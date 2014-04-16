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
#include "flyer_controller/control_mode.h"
#include "flyer_controller/hover_modes.h"
// msg:
#include "flyer_controller/control_mode_hover_info.h"

namespace flyer_controller
{
class ControlModeHover : public HoverMode
{
protected:
  // Parameters
  double pitch_deadband; // joystick units, 0.1 = 10% of half-range
  double roll_deadband; // joystick units, 0.1 = 10% of half-range
  double yaw_deadband; // joystick units, 0.1 = 10% of half-range

  double max_yaw_rate_cmd; // [deg/s] yaw rate corresponding to full deflection
  double waypoint_speed; // [m/s] lateral speed at which to move the controller setpoint between autosequence points
  bool external_command_frame; // should commands be interpreted w.r.t. an external frame?
  double external_frame_heading; // [deg] heading that will correspond to a -pitch command when using external frame

  // Members
  ros::Time last_control_output_time;
  bool first_joystick;
  bool moving_hover_point;
  bool adjusting_yaw;

public:
  ControlModeHover() :
    HoverMode("hover"), //
        pitch_deadband(0.05), roll_deadband(0.05), yaw_deadband(0.1), //
        max_yaw_rate_cmd(10), waypoint_speed(0.25), external_command_frame(false), external_frame_heading(0), //
        first_joystick(true), moving_hover_point(false), adjusting_yaw(false)
  {
  }

  void onInit()
  {
    HoverMode::onInit();
    NODELET_WARN("Joystick must be properly calibrated using jscal.. have you done this?");
    // Parameters
    nh_priv.param("pitch_deadband", pitch_deadband, pitch_deadband);
    nh_priv.param("roll_deadband", roll_deadband, roll_deadband);
    nh_priv.param("yaw_deadband", yaw_deadband, yaw_deadband);
    CHECK_PARAMETER((pitch_deadband >= 0) and (pitch_deadband <= 1), "parameter value out of range");
    CHECK_PARAMETER((roll_deadband >= 0) and (roll_deadband <= 1), "parameter value out of range");
    CHECK_PARAMETER((yaw_deadband >= 0) and (yaw_deadband <= 1), "parameter value out of range");
    nh_priv.param("max_yaw_rate_cmd", max_yaw_rate_cmd, max_yaw_rate_cmd);
    CHECK_PARAMETER(max_yaw_rate_cmd >= 0, "parameter value out of range");
    nh_priv.param("waypoint_speed", waypoint_speed, waypoint_speed);
    nh_priv.param("external_command_frame", external_command_frame, external_command_frame);
    nh_priv.param("external_frame_heading", external_frame_heading, external_frame_heading);
  }

  bool parseControlModeCmdDerived(const string cmd)
  {
    vector<string> words;
    boost::split(words, cmd, boost::is_any_of(" "));
    int nw = words.size();
    if (nw > 0)
    {
      if (words[0] == "adjust")
      {
        if ((nw == 3) and (words[1] == "hover_point"))
        {
          if (words[2] == "on")
          {
            if (not moving_hover_point)
            {
              moving_hover_point = true;
              NODELET_INFO("Hover point adjustment by joystick enabled");
            }
            else
              NODELET_ERROR("Hover point adjustment already enabled");
          }
          else if (words[2] == "off")
          {
            if (moving_hover_point)
            {
              moving_hover_point = false;
              NODELET_INFO("Hover point adjustment by joystick disabled");
            }
            else
              NODELET_ERROR("Hover point adjustment already disabled");

          }
          else
            NODELET_ERROR_STREAM("Invalid command '" << cmd << "'");
        }
        else if ((nw == 3) and (words[1] == "yaw"))
        {
          if (words[2] == "on")
          {
            if (not adjusting_yaw)
            {
              adjusting_yaw = true;
              NODELET_INFO("Yaw adjustment by joystick enabled");
            }
            else
              NODELET_ERROR("Yaw adjustment already enabled");
          }
          else if (words[2] == "off")
          {
            if (adjusting_yaw)
            {
              NODELET_INFO("Yaw adjustment by joystick disabled");
              adjusting_yaw = false;
            }
            else
              NODELET_ERROR("Yaw adjustment already disabled");
          }
          else
          {
            NODELET_ERROR_STREAM("Invalid command '" << cmd << "'");
          }
        }
        else
        {
          NODELET_ERROR_STREAM("Invalid command '" << cmd << "'");
        }
      }
      else
        return false; // unknown first word
    }
    else
      return false; // command empty

    return true;
  }

  // TODO: rework the following in light of joystick overhaul..

  void outputControl()
  {
    ros::Time now_time = ros::Time::now();
    if (first_joystick)
    {
      last_control_output_time = now_time;
      first_joystick = false;
      return;
    }
    double dt = (now_time - last_control_output_time).toSec();

    if (moving_hover_point)
    {
      // Now, interpret pitch and roll as translational velocity commands:
      double cos_ang, sin_ang;
      if (external_command_frame)
      {
        cos_ang = cos(angles::from_degrees(external_frame_heading));
        sin_ang = sin(angles::from_degrees(external_frame_heading));
      }
      else
      {
        double current_yaw, current_pitch, current_roll;
        odom_msg_to_ypr(boost::make_shared<nav_msgs::Odometry>(latest_state), current_yaw, current_pitch, current_roll);
        cos_ang = cos(current_yaw);
        sin_ang = sin(current_yaw);
      }
      double north_cmd_normalized = -sin_ang * latest_opercmd.roll_cmd - cos_ang * latest_opercmd.pitch_cmd;
      double east_cmd_normalized = cos_ang * latest_opercmd.roll_cmd - sin_ang * latest_opercmd.pitch_cmd;
      double north_vel_cmd_new = north_cmd_normalized * waypoint_speed;
      double east_vel_cmd_new = east_cmd_normalized * waypoint_speed;
      double north_cmd_new = north_cmd + north_vel_cmd_new * dt;
      double east_cmd_new = east_cmd + east_vel_cmd_new * dt;
      if (hover_point_within_bounds(north_cmd_new, east_cmd_new))
      {
        north_vel_cmd = north_vel_cmd_new;
        east_vel_cmd = east_vel_cmd_new;
        north_cmd = north_cmd_new;
        east_cmd = east_cmd_new;
      }
      else
      {
        north_vel_cmd = 0.0;
        east_vel_cmd = 0.0;
        NODELET_ERROR_THROTTLE(1, "Cannot move hover point out of bounds");
      }
    }

    // Adjust yaw setpoint if applicable:

    if (adjusting_yaw)
      yaw_cmd = angles::to_degrees(
                                   angles::normalize_angle(
                                                           angles::from_degrees(
                                                                                yaw_cmd + latest_opercmd.yaw_cmd
                                                                                    * max_yaw_rate_cmd * dt)));

    //    else if (moving_hover_point)
    //    {
    //      moving_hover_point = false;
    //      double current_x, current_y, current_z;
    //      odom_msg_to_xyz(boost::make_shared<nav_msgs::Odometry>(latest_state), current_x, current_y, current_z);
    //      north_cmd = current_x;
    //      east_cmd = current_y;
    //    }

    last_control_output_time = now_time;

    HoverMode::outputControl();
  }
  //  void process_joystick(const sensor_msgs::JoyConstPtr & joy_msg)
  //  {
  //  }

}; // class
PLUGINLIB_DECLARE_CLASS(flyer_controller, ControlModeHover, flyer_controller::ControlModeHover, nodelet::Nodelet)
;

}
// namespace
