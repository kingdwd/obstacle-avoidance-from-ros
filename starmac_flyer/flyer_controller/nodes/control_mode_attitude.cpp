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
#include "flyer_controller/control_mode_attitude.h"

namespace flyer_controller
{
  void ControlModeAttitude::onInit()
  {
    JoystickMode::onInit();
    // Parameters
    double max_pitch_cmd = 10; // [deg] commanded pitch corresponding to full deflection
    double max_roll_cmd = 10; // [deg] commanded roll corresponding to full deflection
    bool external_command_frame = false; // should commands be interpreted w.r.t. an external frame?
    double external_frame_heading = 0; // [deg] heading that will correspond to a -pitch command when using external frame
    bool direct_yaw_rate_control = false; // joystick yaw rate commands sent directly

    nh_priv.param("max_pitch_cmd", max_pitch_cmd, max_pitch_cmd);
    nh_priv.param("max_roll_cmd", max_roll_cmd, max_roll_cmd);
    nh_priv.param("external_command_frame", external_command_frame, external_command_frame);
    nh_priv.param("external_frame_heading", external_frame_heading, external_frame_heading);
    nh_priv.param("direct_yaw_rate_control", direct_yaw_rate_control, direct_yaw_rate_control);
    nh_priv.param("step_input_mode_enabled", step_input_mode_enabled, false);
    nh_priv.param("step_input_magnitude", step_input_magnitude, 0.5);
    nh_priv.param("step_input_joystick_roll_axis", step_input_joystick_roll_axis, 4);
    nh_priv.param("step_input_joystick_pitch_axis", step_input_joystick_pitch_axis, 5);
    CHECK_PARAMETER(max_pitch_cmd >= 0, "parameter value out of range");
    CHECK_PARAMETER(max_roll_cmd >= 0, "parameter value out of range");

    CHECK_PARAMETER((external_frame_heading >= -360) && (external_frame_heading <= 360), "parameter value out of range");
    NODELET_INFO_STREAM("max_pitch_cmd = " << max_pitch_cmd);

    att_control.configure(external_command_frame, external_frame_heading, direct_yaw_rate_control, max_pitch_cmd,
                          max_roll_cmd, max_yaw_rate_cmd, min_alt_cmd, max_alt_cmd);

    requestRegistration(mode_name_);
  }

  // Virtual methods from base class
  void ControlModeAttitude::outputControl()
  {
    control_mode_outputPtr output_msg(new control_mode_output);
    bool do_calcs = false;
    bool do_publish = false;
    output_msg->header.stamp = ros::Time::now();

    switch (state)
    {
      case ControlModeTypes::STANDBY:
        do_calcs = got_first_joy and got_first_state;
        do_publish = true;
        break;
      case ControlModeTypes::ACTIVE:
        do_calcs = true;
        do_publish = true;
        break;
      default:
        break;
    }

    if (do_calcs)
    {
      att_control.outputControl(boost::make_shared<nav_msgs::Odometry>(latest_state), boost::make_shared<
          starmac_msgs::OperatorCommands>(latest_opercmd), output_msg);
      if (step_input_mode_enabled)
      {
        //applyStepInputs(output_msg);
      }
    }

    if (do_publish)
    {
      output_msg->control_mode = "attitude";
      output_pub.publish(output_msg);
    }
  }


  // TODO: rework step inputs implementation for new way of handling joystick

//  void ControlModeAttitude::applyStepInputs(control_mode_outputPtr& control_out)
//  {
//    // Check joystick for inputs, and modify control_out accordingly
//    if (latest_opercmd.axes[step_input_joystick_roll_axis] != 0)
//    {
//      double step_input_value = step_input_magnitude * latest_opercmd.axes[step_input_joystick_roll_axis];
//      control_out->roll_cmd = step_input_value;
//      ROS_WARN("Applying roll axis step input of %f degrees", step_input_value);
//    }
//    else if (latest_opercmd.axes[step_input_joystick_pitch_axis] != 0)
//    {
//      double step_input_value = step_input_magnitude * latest_opercmd.axes[step_input_joystick_pitch_axis];
//      control_out->pitch_cmd = step_input_magnitude * latest_opercmd.axes[step_input_joystick_pitch_axis];
//      ROS_WARN("Applying pitch axis step input of %f degrees", step_input_value);
//    }
//  }

  void ControlModeAttitude::reportStatusTimerCallback(const ros::TimerEvent& e)
  {
    control_mode_statusPtr msg(new control_mode_status);
    //NODELET_INFO_STREAM(__PRETTY_FUNCTION__);
    msg->state = state;
    msg->info = info;
    msg->header.stamp = e.current_real;
    ready = (got_first_joy and got_first_state);
    msg->ready = ready; // TODO: check some more conditions..
    control_mode_status_pub.publish(msg);
    diag_updater.update();
  }

  void ControlModeAttitude::controlModeCmdCallback(const control_mode_cmdConstPtr& msg)
  {
    NODELET_INFO_STREAM("Heard command: " << msg->cmd);
    if (msg->cmd == "mode idle")
    {
      state = IDLE;
      info = "";
    }
    else if (msg->cmd == "mode standby")
    {
      state = STANDBY;
      info = "";
    }
    else if (msg->cmd == "mode active")
    {
      if (state == STANDBY)
      {
        state = ACTIVE;
        att_control.useCurrentYaw(boost::make_shared<nav_msgs::Odometry>(latest_state));
        info = "";
      }
      else
      {
        NODELET_ERROR("Invalid transition");
      }
    }
    else
    {
      NODELET_ERROR_STREAM("Command unknown: " << msg->cmd);
    }
  }

PLUGINLIB_DECLARE_CLASS(flyer_controller, ControlModeAttitude, flyer_controller::ControlModeAttitude, nodelet::Nodelet)

} // namespace
