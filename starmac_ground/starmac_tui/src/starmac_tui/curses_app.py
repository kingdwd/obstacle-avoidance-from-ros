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
import curses

class Bag:
    pass

class CursesApp:
    def init_curses(self):
        self.colors = Bag()
        try:
            self.stdscr = curses.initscr()
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(1, curses.COLOR_GREEN, -1)
            self.colors.green = curses.color_pair(1)
            curses.init_pair(2, curses.COLOR_YELLOW, -1)
            self.colors.yellow = curses.color_pair(2)
            curses.init_pair(3, curses.COLOR_RED, -1)
            self.colors.red = curses.color_pair(3)
            curses.init_pair(4, curses.COLOR_BLUE, -1)
            self.colors.blue = curses.color_pair(4)
            curses.init_pair(5, curses.COLOR_BLACK, -1)
            self.colors.black = curses.color_pair(5)
            curses.init_pair(6, curses.COLOR_BLACK, curses.COLOR_WHITE)
            self.colors.black_on_white = curses.color_pair(6)
            curses.init_pair(7, curses.COLOR_WHITE, -1)
            self.colors.white = curses.color_pair(7)
            curses.init_pair(8, curses.COLOR_CYAN, -1)
            self.colors.cyan = curses.color_pair(8)
            curses.noecho()
            curses.cbreak()
            curses.curs_set(0)
            self.stdscr.nodelay(True)
            self.stdscr.clear()
            self.stdscr.keypad(True)
        except curses.error:
            pass
        
    def restore_screen(self):
        curses.curs_set(1)
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        
        
    def curses_msg(self, msg, line=25):
        try:
            self.stdscr.addstr(line, 0, " "*80)
            self.stdscr.addstr(line, 0, msg)
            self.stdscr.refresh()
        except curses.error:
            pass
        
    def add_field(self, label, text, row=None, col=None, label_color=None):
        if label_color is None:
            label_color = self.colors.cyan
        if row is None and col is None:
            self.stdscr.addstr(label, label_color)
        else:
            self.stdscr.addstr(row, col, label, label_color)
        self.stdscr.addstr(text)
        self.stdscr.clrtoeol()
