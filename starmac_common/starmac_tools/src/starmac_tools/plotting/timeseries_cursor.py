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

from matplotlib import pyplot as plt

class TimeSeriesCursorSynchronizer(object):
    
    def __init__(self):
        self.axes = []
        self.cursors = {}
        self.axvlines = {}
        
    def add_axis(self, ax):
        self.axes.append(ax)
        self.axvlines[ax] = []
        self._create_cursor(ax)
        
    def _create_cursor(self, ax):    
        cur = plt.axvline(0, color='r')
        self.cursors[ax] = cur
        
    def _redraw_axis(self, ax):
        ax.get_figure().canvas.draw() # maybe not most efficient way..?
    
    def set_cursors(self, t):
        for ax in self.axes:
            cur = self.cursors[ax]
            cur.set_xdata([t, t])
            self._redraw_axis(ax)
            
    def add_axvline(self, t):
        for ax in self.axes:
            axvline = ax.axvline(t)
            self.axvlines[ax].append(axvline)
            self._redraw_axis(ax)
            
    def clear_axvlines(self):
        for ax, axvlines in self.axvlines.items():
            for axvline in axvlines:
                axvline.remove()
            self.axvlines[ax] = []
            self._redraw_axis(ax)
            
    def sync_xrange(self, src_ax):
        limits = src_ax.get_xlim()
        for ax in self.axes:
            if ax is not src_ax:
                ax.set_xlim(limits)
                self._redraw_axis(ax)
            
            