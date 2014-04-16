# This message conveys normalized operator joystick commands, expressed in the
# joystick frame of reference. I.e., roll_cmd = -1.0 corresponds to full left deflection
# of the joystick's roll axis. Any deadbands should already be applied. Any semantic
# interpretation of these commands occurs downstream of this message. E.g., in
# one context yaw_cmd = 0.5 could mean a certain yaw angle is being commanded
# or it could mean a certain yaw rate is commanded. In another context yaw might
# be entirely repurposed.

float64 roll_cmd    # -1.0 .. 1.0
float64 pitch_cmd   # -1.0 .. 1.0
float64 yaw_cmd     # -1.0 .. 1.0
float64 alt_cmd     #  0.0 .. 1.0
bool motors_on      # true if motors should be running
