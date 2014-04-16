/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, UC Regents
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
#ifndef CONTROL_MODE_ATTITUDE_H
#define CONTROL_MODE_ATTITUDE_H

#include "flyer_controller/joystick_modes.h"
#include "flyer_controller/attitude_controller.h"
#include <math.h>
#include <algorithm>
#include <tf/transform_datatypes.h>

#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)

using namespace flyer_controller;
using namespace std;
using namespace ControlModeTypes;

namespace flyer_controller
{
class ControlModeAttitude : public JoystickMode
{
private:
  // Parameters
  bool step_input_mode_enabled; //
  int step_input_joystick_roll_axis; // which joystick axis will trigger a roll axis step
  int step_input_joystick_pitch_axis; // which joystick axis will trigger a pitch axis step
  double step_input_magnitude; // [deg] size of step attitude input when one is commanded

  // Members
  AttitudeController att_control;

public:
  ControlModeAttitude(string mode_name = "attitude") :
    JoystickMode(mode_name), att_control()
  {
  }

  void onInit();

private:
  // Virtual methods from base class
  void outputControl();

private:
  void applyStepInputs(control_mode_outputPtr& control_out);
  void reportStatusTimerCallback(const ros::TimerEvent& e);
  void controlModeCmdCallback(const control_mode_cmdConstPtr& msg);

}; // class

} // namespace

#endif
