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
#ifndef JOYSTICK_MODES_H
#define JOYSTICK_MODES_H

#include "flyer_controller/control_mode.h"
#include <math.h>
#include <algorithm>
#include <tf/transform_datatypes.h>

#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)

using namespace flyer_controller;
using namespace std;
using namespace ControlModeTypes;
using angles::to_degrees;

namespace flyer_controller
{

struct joystick_command
{
  double roll; // -1..1 (clipped, deadbanded, scaled)
  double pitch; // -1..1 (clipped, deadbanded, scaled)
  double yaw; // -1..1 (clipped, deadbanded, scaled)
  double alt; // -1..1 (clipped, deadbanded, scaled)
};

// This class is to be used as a base class for modes that use the joystick for input
// (i.e. for more than just altitude and the buttons)
class JoystickMode : public ControlMode
{
protected:
  string mode_name_;
  // Parameters
  double max_yaw_rate_cmd; // [deg/s] yaw rate corresponding to full deflection
  double max_alt_cmd; // [m] commanded altitude corresponding to full + deflection
  double min_alt_cmd; // [m] commanded altitude corresponding to full - deflection

  // Members
//  joystick_command latest_cmd;
//  double latest_pitch_cmd; // -1..1 (clipped, deadbanded, scaled)
//  double latest_roll_cmd; // -1..1 (clipped, deadbanded, scaled)
//  double latest_yaw_cmd; // -1..1 (clipped, deadbanded, scaled)
  double yaw_cmd; // degrees

public:
  JoystickMode(string mode_name) :
    mode_name_(mode_name), max_yaw_rate_cmd(10), max_alt_cmd(1.5),
        min_alt_cmd(0.0),
        //latest_cmd(),
        yaw_cmd(0)
  {

  }

  void onInit()
  {
    ControlMode::onInit();
    NODELET_WARN("Joystick must be properly calibrated using jscal.. have you done this?");
    // Parameters
    nh_priv.param("max_yaw_rate_cmd", max_yaw_rate_cmd, max_yaw_rate_cmd);
    CHECK_PARAMETER(max_yaw_rate_cmd >= 0, "parameter value out of range");
    nh_priv.param("max_alt_cmd", max_alt_cmd, max_alt_cmd);
    nh_priv.param("min_alt_cmd", min_alt_cmd, min_alt_cmd);
    CHECK_PARAMETER(max_alt_cmd >= min_alt_cmd, "parameter value out of range");

  }

}; // class
}// namespace
#endif
