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
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include "flyer_controller/control_mode.h"
#include "flyer_controller/control_mode_hover_info.h"
#include "flyer_controller/control_utils.h"
#include "flyer_controller/pid.h"
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>

using std::string;
using std::vector;
using std::map;
using std::max;
using std::min;
using angles::to_degrees;
using angles::from_degrees;

const double GRAVITY = 9.81; // m/s/s

namespace flyer_controller
{

struct hover_point
{
  string name;
  double north; // [m]
  double east; // [m]
  double north_vel; // [m/s]
  double east_vel; // [m/s]
  double yaw; // [deg]
};

class HoverMode : public ControlMode
{
protected:
  // Params
  double max_alt_cmd; // [m] commanded altitude corresponding to full + deflection
  double min_alt_cmd; // [m] commanded altitude corresponding to full - deflection
  // next two meaningless unless/until 'manual hover' implemented:
  //  bool external_command_frame; // should commands be interpreted w.r.t. an external frame?
  //  double external_frame_heading; // [deg] heading that will correspond to a -pitch command when using external frame
  double KP; // deg/m
  double KI; // deg/(m-s)
  double KD; // deg/m/s
  double Ilimit; // deg
  bool direct_thrust_control; // set true to do thrust control directly in hover controller
  double alt_KP; // N per meter
  double alt_KI; // N per meter-second
  double alt_KD; // N counts per m/s
  double alt_Ilimit; // N
  double mass; // kg - nominal thrust will be based on this value
  double north_cmd_max; // m - disallow commanding north values greater than this
  double north_cmd_min; // m - disallow commanding north values less than this
  double east_cmd_max; // m - disallow commanding east values greater than this
  double east_cmd_min; // m - disallow commanding east values less than this

  // Publisher
  ros::Publisher info_pub;
  //  ros::Publisher marker_pub;
  //  ros::Publisher marker_array_pub;
  // Member vars
  string mode_name_;
  control_mode_output control_out;
  double yaw_cmd; // [deg]
  flyer_controller::Pid north_pid;
  flyer_controller::Pid east_pid;
  flyer_controller::Pid alt_pid;
  double north_cmd;
  double east_cmd;
  double alt_cmd;
  double north_vel_cmd;
  double east_vel_cmd;
  double north_err;
  double east_err;
  double alt_err; // used with direct_thrust_control only
  double north_vel_err;
  double east_vel_err;
  map<string, hover_point> hover_points;
  hover_point current_hover_point;
  double alt_override;
  double yaw_override;
  bool yaw_override_active;
  ros::Time last_time;
  bool first;

public:

  HoverMode(string mode_name) :
        //    external_command_frame(false), external_frame_heading(0),
        max_alt_cmd(1.5), min_alt_cmd(0.0), KP(12), KI(1),
        KD(8),
        Ilimit(2), //
        direct_thrust_control(false), alt_KP(2), alt_KI(1), alt_KD(2),
        alt_Ilimit(5),
        mass(1.3), //
        mode_name_(mode_name), yaw_cmd(0), north_cmd(0), east_cmd(0), alt_cmd(0), north_vel_cmd(0), east_vel_cmd(0),
        north_err(0), east_err(0), north_vel_err(0), east_vel_err(0), alt_override(9999), yaw_override(0),
        yaw_override_active(false), first(true)
  {

  }

  void onInit()
  {
    NODELET_INFO("ControlModeHover onInit() called");
    ControlMode::onInit();
    // Parameters
    nh_priv.param("max_alt_cmd", max_alt_cmd, max_alt_cmd);
    nh_priv.param("min_alt_cmd", min_alt_cmd, min_alt_cmd);
    nh_priv.param("KP", KP, KP);
    nh_priv.param("KI", KI, KI);
    nh_priv.param("KD", KD, KD);
    nh_priv.param("Ilimit", Ilimit, Ilimit);
    nh_priv.param("direct_thrust_control", direct_thrust_control, direct_thrust_control);
    nh_priv.param("alt_KP", alt_KP, alt_KP);
    nh_priv.param("alt_KI", alt_KI, alt_KI);
    nh_priv.param("alt_KD", alt_KD, alt_KD);
    nh_priv.param("alt_Ilimit", alt_Ilimit, alt_Ilimit);
    nh_priv.param("mass", mass, mass);
    nh_priv.param("north_cmd_max", north_cmd_max, north_cmd_max);
    nh_priv.param("north_cmd_min", north_cmd_min, north_cmd_min);
    nh_priv.param("east_cmd_max", east_cmd_max, east_cmd_max);
    nh_priv.param("east_cmd_min", east_cmd_min, east_cmd_min);

    // Publishers
    //    marker_pub = nh_priv.advertise<visualization_msgs::Marker> ("marker", 10, true);
    //    marker_array_pub = nh_priv.advertise<visualization_msgs::MarkerArray> ("marker_array", 10, true);

    // North and East PID control
    set_gains(KP, KI, KD, Ilimit);
    // Altitude PID control (optional)
    if (direct_thrust_control)
      alt_pid.initPid(alt_KP, alt_KI, alt_KD, alt_Ilimit, -alt_Ilimit);

    requestRegistration(mode_name_);
    control_out.control_mode = mode_name_;
    control_out.motors_on = false;
  }

private:
  // Parses (and acts upon if appropriate) a control_mode_command.Returns true if the command
  // was recognized and false otherwise. Note that an invalid (but still recognized) command
  // should only emit an error message but *not* also return false.
  bool parseControlModeCmd(const string cmd)
  {
    vector<string> words;
    boost::split(words, cmd, boost::is_any_of(" "));
    int nw = words.size();
    if (nw > 0)
    {
      if (words[0] == "mode")
      {
        parse_mode(words);
      }
      else if (words[0] == "define")
      // define hover_point point_name north_coord east_coord [north_vel east_vel [yaw_coord]]

      {
        if (words[1] == "hover_point")
        {
          parse_define_hover_point(words);
        }
        else
        {
          return false;
        }
      }
      else if (words[0] == "set")
      {
        if (nw > 1)
        {
          if (words[1] == "hover_point")
          {
            parse_set_hover_point(words);

          }
          else if (words[1] == "gains")
          {
            parse_set_gains(words);
          }
        }
        else
        {
          return false;
        }
      }
      else if (words[0] == "alt_override")
      {
        if (nw == 2)
        {
          alt_override = atof(words[1].c_str());
          NODELET_INFO_STREAM("alt_override set to " << alt_override);
        }
        else
        {
          NODELET_ERROR_STREAM("Invalid command '" << cmd << "'");
        }
      }
      else if (words[0] == "yaw_override")
      {
        if (nw == 2)
        {
          if (words[1] == "off" and yaw_override_active)
          {
            yaw_override_active = false;
            NODELET_INFO("yaw_override OFF");
          }
          else
          {
            yaw_override_active = true;
            yaw_override = atof(words[1].c_str());
            yaw_cmd = yaw_override;
            NODELET_INFO_STREAM("yaw_override set to " << yaw_override);
          }
        }
        else
        {
          NODELET_ERROR_STREAM("Invalid command '" << cmd << "'");
        }
      }
      else
      {
        return false;
      }
    }
    return true;
  }

protected:
  // Actual hover mode classes should implement this method if they are to respond
  // to additional commands. It is called if parseControlModeCmd returns false.
  // The default implementation returns false so that an unrecognized command
  // will result in an error even if the derived class doesn't implement this method.
  virtual bool parseControlModeCmdDerived(const string cmd)
  {
    return false;
  }

private:
  void controlModeCmdCallback(const control_mode_cmdConstPtr& msg)
  {
    NODELET_DEBUG_STREAM("Heard command: " << msg->cmd);
    if (!parseControlModeCmd(msg->cmd))
    {
      if (!parseControlModeCmdDerived(msg->cmd))
      {
        NODELET_ERROR_STREAM("Unrecognized command: " << msg->cmd);
      }
    }
  }

  // Virtual methods from base class
protected:
  void outputControl()
  {
    //control_mode_output control_out;
    bool do_calcs = false;
    bool do_publish = false;
    ros::Time now_time = ros::Time::now();
    ros::Duration dt;
    if (first)
    {
      first = false;
      last_time = now_time;
      return;
    }
    else if (state_updated)
    {
      dt = now_time - last_time;
      last_time = now_time;
    }
    static int prev_trigger = 0;
    control_out.header.stamp = now_time;

    switch (state)
    {
      case ControlModeTypes::STANDBY:
        do_calcs = (got_first_joy and got_first_state);
        do_publish = true;
        break;
      case ControlModeTypes::ACTIVE:
        do_calcs = state_updated; // if no state updates, keep last command
        do_publish = true;
        break;
      default:
        break;
    }

    if (do_calcs)
    {
      //      if (not state_updated and state == ControlModeTypes::ACTIVE)
      //      {
      //        ROS_INFO("state not updated since last iteration");
      //      }
      double current_yaw, current_pitch, current_roll; // radians
      double current_vx, current_vy, current_vz;
      odom_msg_to_ypr(boost::make_shared<nav_msgs::Odometry>(latest_state), current_yaw, current_pitch, current_roll);
      odom_msg_to_lin_vel(boost::make_shared<nav_msgs::Odometry>(latest_state), current_vx, current_vy, current_vz);
      state_updated = false;
      get_current_lateral_position_errors(north_err, east_err, alt_err);
      north_vel_err = current_vx - north_vel_cmd;
      east_vel_err = current_vy - east_vel_cmd;
      if (latest_opercmd.motors_on and prev_trigger == 0)
      {
        north_pid.reset();
        east_pid.reset();
        if (direct_thrust_control)
          alt_pid.reset();
      }
      north_pid.updatePid(north_err, north_vel_err, dt);
      east_pid.updatePid(east_err, east_vel_err, dt);
      if (direct_thrust_control)
        alt_pid.updatePid(alt_err, dt);
      double tilt_north_cmd = north_pid.getCurrentCmd();
      double tilt_east_cmd = east_pid.getCurrentCmd();
      // Rotate into body frame:
      double cos_yaw = cos(current_yaw);
      double sin_yaw = sin(current_yaw);
      control_out.roll_cmd = tilt_east_cmd * cos_yaw - tilt_north_cmd * sin_yaw;
      control_out.pitch_cmd = -tilt_east_cmd * sin_yaw - tilt_north_cmd * cos_yaw;
      prev_trigger = latest_opercmd.motors_on;
      control_out.direct_yaw_rate_commands = false;
      control_out.yaw_cmd = yaw_cmd;
      control_out.yaw_rate_cmd = 0; //std::numeric_limits<double>::quiet_NaN();
      control_out.direct_thrust_commands = direct_thrust_control;
      if (direct_thrust_control)
      {
        control_out.alt_cmd = 0.0;
        control_out.thrust_cmd = GRAVITY * mass + alt_pid.getCurrentCmd();
      }
      else
      {
        control_out.alt_cmd = min(alt_override, min_alt_cmd + (max_alt_cmd - min_alt_cmd) * latest_opercmd.alt_cmd);
      }
    } // do_calcs

    control_out.motors_on = (got_first_joy and (latest_opercmd.motors_on));
    if (do_publish)
    {
      control_out.control_mode = mode_name_;
      output_pub.publish(control_out);
    }

  }

protected:
  void get_current_lateral_position_errors(double &north_err, double &east_err, double &alt_err)
  {
    double current_x, current_y, current_z;
    odom_msg_to_xyz(boost::make_shared<nav_msgs::Odometry>(latest_state), current_x, current_y, current_z);
    north_err = current_x - north_cmd;
    east_err = current_y - east_cmd;
    alt_err = -current_z - alt_cmd;
  }

  void get_current_lateral_position(double &current_x, double &current_y)
  {
    double current_z;
    odom_msg_to_xyz(boost::make_shared<nav_msgs::Odometry>(latest_state), current_x, current_y, current_z);
  }

  void get_current_error_to_point(const hover_point& point, double &north_err, double &east_err, double &yaw_err)
  {
    double current_x, current_y, current_z;
    odom_msg_to_xyz(boost::make_shared<nav_msgs::Odometry>(latest_state), current_x, current_y, current_z);
    north_err = current_x - point.north;
    east_err = current_y - point.east;
    get_current_yaw_error(point.yaw, yaw_err);
  }

  void get_current_yaw_error(const double yaw_cmd, double &yaw_err)
  {
    if (not isnan(yaw_cmd))
    {
      double current_yaw, current_pitch, current_roll;
      odom_msg_to_ypr(boost::make_shared<nav_msgs::Odometry>(latest_state), current_yaw, current_pitch, current_roll);
      yaw_err = angles::shortest_angular_distance(current_yaw, angles::from_degrees(yaw_cmd));
    }
    else
    {
      yaw_err = 0.0;
    }
  }

protected:
  void reportStatusTimerCallback(const ros::TimerEvent& e)
  {
    control_mode_status msg;
    //NODELET_INFO_STREAM(__PRETTY_FUNCTION__);
    msg.state = state;
    msg.info = info;
    msg.header.stamp = e.current_real;
    ready = (got_first_joy and got_first_state);
    //    NODELET_INFO_STREAM("got_first_joy = " << got_first_joy << " got_first_state = " << got_first_state);
    msg.ready = ready; // TODO: check some more conditions..
    control_mode_status_pub.publish(msg);

    if ((state == ControlModeTypes::STANDBY) or (state == ControlModeTypes::ACTIVE))
    {
      control_mode_hover_info info_msg;
      info_msg.header.stamp = e.current_real;
      info_msg.hover_point = current_hover_point.name;

      info_msg.north_cmd = north_cmd;
      info_msg.east_cmd = east_cmd;
      info_msg.north_vel_cmd = north_vel_cmd;
      info_msg.east_vel_cmd = east_vel_cmd;
      info_msg.yaw_cmd = yaw_cmd;
      info_msg.alt_override = alt_override;

      info_msg.north_err = north_err;
      info_msg.east_err = east_err;
      info_msg.north_vel_err = north_vel_err;
      info_msg.east_vel_err = east_vel_err;
      double yaw_err;
      get_current_yaw_error(yaw_cmd, yaw_err);
      info_msg.yaw_err = yaw_err;

      info_pub.publish(info_msg);
    }
    diag_updater.update();
  }

private:
  // Command parsers
  void parse_mode(const vector<string> &words)
  {
    int nw = words.size();
    if (nw == 2)
    {
      string newmode = words[1];
      if (newmode == "idle")
      {
        state = ControlModeTypes::IDLE;
        info = "";
      }
      else if (newmode == "standby")
      {
        info = "";
        info_pub = nh_priv.advertise<control_mode_hover_info> ("info", 10);
        state = ControlModeTypes::STANDBY;
      }
      else if (newmode == "active")
      {
        if (state == ControlModeTypes::STANDBY)
        {
          state = ControlModeTypes::ACTIVE;
          double current_yaw, current_pitch, current_roll;
          double current_x, current_y, current_z;
          odom_msg_to_ypr(boost::make_shared<nav_msgs::Odometry>(latest_state), current_yaw, current_pitch,
                          current_roll);
          odom_msg_to_xyz(boost::make_shared<nav_msgs::Odometry>(latest_state), current_x, current_y, current_z);
          hover_point point_here;
          point_here.yaw = to_degrees(current_yaw);
          point_here.north = current_x;
          point_here.east = current_y;
          point_here.east_vel = 0;
          point_here.north_vel = 0;
          point_here.name = "_hover_mode_entry";
          set_hover_point(point_here, true);
          alt_override = 9999;
          info = "";
        }
      }
      else
      {
        NODELET_ERROR("Unknown mode");
      }
    }
    else
    {
      NODELET_ERROR("Invalid command");
    }
  }

  void parse_define_hover_point(const vector<string> &words)
  {
    /*
     * Examples:
     * a) Define a named point 'foo' with north = 0.1 m, east = 0.2 m, yaw and velocities unspecified.
     *          define hover_point foo 0.1 0.2
     *
     * b) Define a named point 'bar' with north = 0.1 m, east = 0.2 m, yaw = 45 deg and velocities unspecified.
     *          define hover_point bar 0.1 0.2 45.0
     *
     */
    int nw = words.size();
    if (not ((nw == 5) or (nw == 7) or (nw == 8)))
    {
      NODELET_ERROR("Invalid command");
    }
    else
    {
      string point_name = words[2];
      if (hover_point_exists(point_name))
      {
        NODELET_WARN_STREAM("hover_point '" << point_name << "' already exists and will be redefined");
      }
      hover_points[point_name].name = point_name;
      double north = atof(words[3].c_str());
      double east = atof(words[4].c_str());
      if (not hover_point_within_bounds(north, east))
        NODELET_WARN_STREAM("Hover point " << point_name << " has out-of-bounds coordinates. Point will be defined but cannot be used.");
      hover_points[point_name].north = north;
      hover_points[point_name].east = east;
      if ((nw == 7) or (nw == 8)) // north_vel, east_vel and perhaps yaw_coord specified:

      {
        hover_points[point_name].north_vel = atof(words[5].c_str());
        hover_points[point_name].east_vel = atof(words[6].c_str());
        if (nw == 7)
        {
          hover_points[point_name].yaw = NAN;
          NODELET_DEBUG_STREAM(
                               "New hover_point defined: name='" << point_name << "' north="
                                   << hover_points[point_name].north << " east=" << hover_points[point_name].east
                                   << " north_vel=" << hover_points[point_name].north_vel << " east_vel="
                                   << hover_points[point_name].east_vel << ". Yaw unspecified.");
        }
        else if (nw == 8)
        {
          hover_points[point_name].yaw = atof(words[7].c_str());
          NODELET_DEBUG_STREAM(
                               "New hover_point defined: name='" << point_name << "' north="
                                   << hover_points[point_name].north << " east=" << hover_points[point_name].east
                                   << " north_vel=" << hover_points[point_name].north_vel << " east_vel="
                                   << hover_points[point_name].east_vel << " yaw=" << hover_points[point_name].yaw);
        }
      }
      else
      {
        hover_points[point_name].north_vel = 0;
        hover_points[point_name].east_vel = 0;
        hover_points[point_name].yaw = NAN;
        NODELET_DEBUG_STREAM(
                             "New hover_point defined: name='" << point_name << "' north="
                                 << hover_points[point_name].north << " east=" << hover_points[point_name].east
                                 << " north_vel=" << hover_points[point_name].north_vel << " east_vel="
                                 << hover_points[point_name].east_vel << ". Yaw unspecified.");
      }
    }
  }

protected:
  bool hover_point_within_bounds(const double north, const double east)
  {
    return ((north >= north_cmd_min) and (north <= north_cmd_max) //
        and (east >= east_cmd_min) and (east <= east_cmd_max));
  }

  bool hover_point_exists(const string point_name)
  {
    map<string, hover_point>::iterator it;
    it = hover_points.find(point_name);
    return (not (it == hover_points.end()));
  }

  void set_gains(double KP, double KI, double KD, double Ilimit)
  {
    north_pid.initPid(KP, KI, KD, Ilimit, -Ilimit);
    east_pid.initPid(KP, KI, KD, Ilimit, -Ilimit);
  }

  // syntax:
  // set gains KP KI KD Ilimit
  void parse_set_gains(const vector<string> &words)
  {
    int nw = words.size();
    if (nw == 6)
    {
      double KP_new = atof(words[2].c_str());
      double KI_new = atof(words[3].c_str());
      double KD_new = atof(words[4].c_str());
      double Ilimit_new = atof(words[5].c_str());
      // TODO: do some checks on what was entered
      KP = KP_new;
      KI = KI_new;
      KD = KD_new;
      Ilimit = Ilimit_new;
      set_gains(KP, KI, KD, Ilimit);
      NODELET_INFO("Set new gains: KP=%f, KI=%f, KD=%f, Ilimit=%f", KP, KI, KD, Ilimit);
    }
    else
    {
      NODELET_ERROR("Invalid command");
    }
  }

  void parse_set_hover_point(const vector<string> &words)
  {
    int nw = words.size();
    if (nw == 3)
    {
      string point_name = words[2];
      if (hover_point_exists(point_name))
      {
        set_hover_point(hover_points[point_name]);
      }
      else
      {
        NODELET_ERROR("Invalid hover_point name");
      }
    }
    else
    {
      NODELET_ERROR("Invalid command");
    }
  }

  void set_hover_point(const hover_point& point, bool reset = false)
  {
    if (hover_point_within_bounds(point.north, point.east))
    {
      north_cmd = point.north;
      east_cmd = point.east;
      north_vel_cmd = point.north_vel;
      east_vel_cmd = point.east_vel;
      if (reset)
      {
        north_pid.reset();
        east_pid.reset();
      }
      current_hover_point = point;
      //    NODELET_INFO_STREAM("Setting hover setpoint to hover_point '" << point.name << "': north="
      //        << north_cmd << " east=" << east_cmd
      //        << " north_vel_cmd=" << north_vel_cmd << " east_vel_cmd=" << east_vel_cmd);
      if (not isnan(point.yaw))
      {
        yaw_cmd = point.yaw;
        //      NODELET_INFO_STREAM("Setting yaw setpoint to " << yaw_cmd);
      }
      else
      {
        //      NODELET_INFO("Hover point does not define a yaw value, yaw command unmodified.");
      }
      //    send_viz_marker(north_cmd, east_cmd, yaw_cmd);
    }
    else
    {
      NODELET_ERROR("Cannot set hover point to %f, %f (out of bounds)", point.north, point.east);
    }
  }

};

}
