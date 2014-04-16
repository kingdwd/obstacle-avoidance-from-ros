#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, UC Regents
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
import roslib; roslib.load_manifest('starmac_tui')
import rospy
import curses
from starmac_tui.curses_app import CursesApp

from rosgraph_msgs.msg import Log
from flyer_controller.msg import controller_status
from flyer_controller.msg import controller_cmd
from flyer_controller.msg import control_mode_hover_info
from flyer_controller.msg import control_mode_cmd
from nav_msgs.msg import Odometry

from asctec_hl_comm.msg import mav_status

class Bag:
    pass

spinner = "\|/-"
POS_FMT = "% 7.3f"

class StarmacTuiNode(CursesApp):
    
    def __init__(self):
        rospy.init_node('tui')
        self.init_curses()
        self.spinner_state = 0
        self.blink_state = True
        self.scroll_state = 0
        self.scroll_incr = 1
        self.cur_page = 0
        self.prev_page = 0
        self.page_names = ('Home', 'Control', 'Mode', 'Health', 'Console', 'Help')
        self.num_pages = len(self.page_names)
        self.msg_buf = []
        self.moving_hover_point = False
        self.moving_yaw_in_hover = False
        self.latest = Bag()
        self.latest.controller_status = controller_status()
        self.latest.estimator_output = Odometry()
        self.latest.hover_info = control_mode_hover_info()
        self.latest.mav_status = mav_status()
        self.init_publishers()
        self.hover_info_sub = None
        self.hover_cmd_pub = None
        self.init_subscribers()                            
        
        
    def init_publishers(self):
        self.controller_cmd_pub = rospy.Publisher('controller/cmd', controller_cmd)
        
    def init_subscribers(self):
        self.rosout_sub = rospy.Subscriber('/rosout_agg', Log, self.rosout_cb)
        self.controller_status_sub = rospy.Subscriber('controller/status', controller_status, self.controller_status_cb)
        self.est_sub = rospy.Subscriber('estimator/output', Odometry, self.est_cb)
        self.mav_status_sub = rospy.Subscriber('fcu/status', mav_status, self.mav_status_cb)
        
    def est_cb(self, msg):
        self.latest.estimator_output = msg
        
    def rosout_cb(self, msg):
        self.msg_buf.append((msg.level, msg.msg))
        
    def controller_status_cb(self, msg):
        self.latest.controller_status = msg
        
    def hover_info_cb(self, msg):
        self.latest.hover_info = msg
        
    def mav_status_cb(self, msg):
        self.latest.mav_status = msg
        
    def volts_color(self, volts):
        if volts < 4.0:
            vc = self.colors.blue + curses.A_BOLD
        elif volts < 10.0:
            if self.blink_state:
                vc = self.colors.red + curses.A_BOLD + curses.A_BLINK
            else:
                vc = self.colors.black
                curses.beep()
        elif volts < 11.0:
            vc = self.colors.yellow + curses.A_BOLD
        else:
            vc = self.colors.green
        return vc
    
    def update(self):
        self.blink_state = not self.blink_state
        maxy, maxx = self.stdscr.getmaxyx()
        addstr = self.stdscr.addstr
        
        if self.cur_page != self.prev_page:
            self.stdscr.clear()
            self.stdscr.setscrreg(0,maxy-2)
            
            
        volts = self.latest.mav_status.battery_voltage #11.85 #self.iloop_data.battVoltage
        if self.cur_page == self.page_names.index('Home'):
            # Altitude:
            y, x = self.stdscr.getyx()
            self.update_alt(0, 0)
            # Mode:
            cur_mode = self.latest.controller_status.active_mode
            self.update_mode(1, cur_mode)
            # Position:
            self.update_pos(2)
                
        elif self.cur_page == self.page_names.index('Control'):
            # State:
            if self.latest.controller_status.state == controller_status.STATE_OPERATIONAL:
                state = 'Operational'
                color = self.colors.green
            elif self.latest.controller_status.state == controller_status.STATE_STANDBY:
                state = 'Standby'
                color = self.colors.yellow
            elif self.latest.controller_status.state == controller_status.STATE_INITIALIZING:
                state = 'Initializing'
                color = self.colors.yellow
            elif self.latest.controller_status.state == controller_status.STATE_ERROR:
                state = 'Off'
                color = self.colors.blue
            else:
                state = '?'
                color = self.colors.red
            addstr(0,0,state,color)
            self.stdscr.clrtoeol()
            
            # Mode:
            addstr(1,0,'Active:',self.colors.cyan)
            cur_mode = self.latest.controller_status.active_mode
            self.update_mode(2, cur_mode)
            addstr(3,0,'Standby:',self.colors.cyan)
            for i in range(len(self.latest.controller_status.standby_modes)):
                addstr(4+i,0,'%d %s' % (i+1, self.latest.controller_status.standby_modes[i]))
                self.stdscr.clrtoeol()
            self.stdscr.clrtobot()
            
            
        elif self.cur_page == self.page_names.index('Mode'):
            # Mode:
            addstr(0,0,'Active:',self.colors.cyan)
            cur_mode = self.latest.controller_status.active_mode
            self.update_mode(1, cur_mode)

            if cur_mode == "idle":
                pass
            elif cur_mode == "attitude":
                pass
            elif cur_mode == "hover":
                if self.hover_info_sub is None:
                   self.hover_info_sub = rospy.Subscriber('control_mode_hover/info', control_mode_hover_info, self.hover_info_cb)
                   self.hover_cmd_pub = rospy.Publisher('control_mode_hover/cmd', control_mode_cmd)
                   self.stdscr.clear()
                else:
                    row = 2
                    self.add_field('Pt: ', self.latest.hover_info.hover_point, row, 0)
                    self.add_field('    Cmd     Act    Err  [m]', '', row+1, 0)
                    cmd_act_err_x = (POS_FMT % self.latest.hover_info.north_cmd) + " " \
                        + (POS_FMT % self.latest.estimator_output.pose.pose.position.x) + " " \
                        + (POS_FMT % self.latest.hover_info.north_err)
                    self.add_field('X:', cmd_act_err_x, row+2, 0)
                    cmd_act_err_y = (POS_FMT % self.latest.hover_info.east_cmd) + " " \
                        + (POS_FMT % self.latest.estimator_output.pose.pose.position.y) + " " \
                        + (POS_FMT % self.latest.hover_info.east_err)
                    self.add_field('Y:', cmd_act_err_y, row+3, 0)
                    cmd_act_err_yaw = (POS_FMT % self.latest.hover_info.yaw_cmd) + " " \
                        + (POS_FMT % 0.0) + " " \
                        + (POS_FMT % self.latest.hover_info.yaw_err)
                    self.add_field('Yaw:', cmd_act_err_yaw, row+4, 0)
                    row += 5
                    if self.moving_hover_point:
                        if self.blink_state:
                            color = self.colors.yellow
                        else:
                            color = self.colors.black
                        addstr(row, 0, 'MOVING HOVER POINT', color)
                        self.stdscr.clrtoeol()
                    else:
                        addstr(row, 0, 'm - move hover pt')
                        self.stdscr.clrtoeol()
                    if self.moving_yaw_in_hover:
                        addstr(row+1, 0, 'ADJUSTING YAW', self.colors.yellow)
                        self.stdscr.clrtoeol()
                    else:
                        addstr(row+1, 0, 'y - adjust yaw')
                        self.stdscr.clrtoeol()
            
        elif self.cur_page == self.page_names.index('Health'): 
            addstr(0,0,'Bat:')
            self.update_voltage(0,5,volts)
            
        elif self.cur_page == self.page_names.index('Console'): 
            for level, msg in self.msg_buf:
                self.print_console(maxy-2, level, msg)
                self.stdscr.scrollok(True)
                self.stdscr.scroll()
                self.stdscr.scrollok(False)

            self.msg_buf = []
            
        elif self.cur_page == self.page_names.index('Help'): 
            addstr(0,0,'Instructions:')
            addstr(2,0,'l/r arrows: chg. page')
            addstr(4,0,'q: quit')
        else:
            pass
        
        # Footer:
        # Page number:
        for i in range(self.num_pages):
            if i == self.cur_page:
                color = self.colors.black_on_white
            else:
                color = self.colors.white
            addstr(maxy-1,i,str(i),color)
        # Console:
        if self.cur_page != self.page_names.index('Console'):
            if len(self.msg_buf) > 0:
                self.print_console(maxy-2, self.msg_buf[-1][0], self.msg_buf[-1][1])
        # Separator:
        addstr(maxy-1,self.num_pages,'-'*(maxx-self.num_pages-1))
        addstr(maxy-1,self.num_pages+2,self.page_names[self.cur_page], self.colors.cyan)
        
        # Voltage:
        self.update_voltage(maxy-1,maxx-8,volts)

        # Spinner:
        self.stdscr.addch(maxy-1,maxx-2,ord(spinner[self.spinner_state]))
        self.spinner_state = (self.spinner_state + 1) % len(spinner)

        
        self.prev_page = self.cur_page
        
    # Individual page elements:
    def update_pos(self, row):
        x = self.latest.estimator_output.pose.pose.position.x
        y = self.latest.estimator_output.pose.pose.position.y
        self.add_field("X:", POS_FMT % x, row, 0)
#        self.stdscr.addstr(row, 0, "X:", self.colors.cyan)
#        self.stdscr.addstr(POS_FMT % x)
        self.stdscr.addstr(" Y:", self.colors.cyan)
        self.stdscr.addstr(POS_FMT % y)
        self.stdscr.clrtoeol()
        
    def update_alt(self, row, col):
        alt = -self.latest.estimator_output.pose.pose.position.z
        self.stdscr.addstr(row, col, "Alt:", self.colors.cyan)
        self.stdscr.addstr(POS_FMT % alt)
        self.stdscr.clrtoeol()
        
    def update_voltage(self, row, col, volts):
        self.stdscr.addstr(row,col,"%-3.2fV" % volts, self.volts_color(volts))
        
    def update_mode(self, row, mode):
        maxy, maxx = self.stdscr.getmaxyx()
        self.stdscr.addstr(row,0,mode[self.scroll_state:self.scroll_state+maxx])
        if len(mode) > maxx:
            if (self.scroll_incr == 1 and self.scroll_state == len(mode) - maxx) \
                or (self.scroll_incr == -1 and self.scroll_state == 0):
                self.scroll_incr = -self.scroll_incr
            self.scroll_state = self.scroll_state + self.scroll_incr
        else:
            self.stdscr.clrtoeol()
    
    def print_console(self, row, level, msg):
        maxy, maxx = self.stdscr.getmaxyx()
        if level == Log.INFO:
            color = self.colors.white
        elif level == Log.WARN:
            color = self.colors.yellow
        elif level == Log.ERROR:
            color = self.colors.red
        elif level == Log.DEBUG:
            color = self.colors.cyan
        self.stdscr.addstr(row,0,' '*(maxx-1))
        self.stdscr.addstr(row,0,msg.replace('\n',r'\n')[:maxx-1], color)
        
        
    def mode_to_active(self, mode_num):
        if mode_num < len(self.latest.controller_status.standby_modes):
            new_mode = self.latest.controller_status.standby_modes[mode_num]
            cmd = controller_cmd()
            cmd.cmd = "control_mode to_active %s" % new_mode
            self.controller_cmd_pub.publish(cmd)
            
    def toggle_modify_hover(self):
        if self.hover_cmd_pub is not None:
            msg = control_mode_cmd()
            if self.moving_hover_point:
                msg.cmd = "adjust hover_point off"
            else:
                msg.cmd = "adjust hover_point on"
            self.hover_cmd_pub.publish(msg)
            self.moving_hover_point = not self.moving_hover_point
        else:
            pass
        
    def toggle_modify_yaw_in_hover(self):
        if self.hover_cmd_pub is not None:
            msg = control_mode_cmd()
            if self.moving_yaw_in_hover:
                msg.cmd = "adjust yaw off"
            else:
                msg.cmd = "adjust yaw on"
            self.hover_cmd_pub.publish(msg)
            self.moving_yaw_in_hover = not self.moving_yaw_in_hover
        else:
            pass
        
    def go(self, *args, **kwargs):
        r = rospy.Rate(20)
        count = 0
        while not rospy.is_shutdown():
            c = self.stdscr.getch()
            if c == ord('q'):
                self.curses_msg("Shutting down...")
                self.restore_screen()
                rospy.signal_shutdown("Quitting..")
            elif c == ord('r'):
                self.reset()
            elif c == curses.KEY_RIGHT:
                self.cur_page = (self.cur_page + 1) % self.num_pages
            elif c == curses.KEY_LEFT:
                self.cur_page = (self.cur_page - 1) % self.num_pages
            else:
                # Page-specific keys:
                if self.cur_page == self.page_names.index("Control"):
                    if c >= ord('1') and c <= ord('9'):
                        self.mode_to_active(c - ord('1'))
                if self.cur_page == self.page_names.index('Mode'):
                    cur_mode = self.latest.controller_status.active_mode
                    if cur_mode == "hover":
                        if c == ord('m'):
                            self.toggle_modify_hover()
                        elif c == ord('y'):
                            self.toggle_modify_yaw_in_hover()
                
            # Refresh display:    
            count = (count + 1) % 4
            if count == 0 or (self.cur_page != self.prev_page):
                self.update()
            r.sleep()

if __name__ == "__main__":
    stn = StarmacTuiNode()
    try:
        stn.go()
    finally:
        stn.restore_screen()


