#!/usr/bin/env python
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

import roslib
roslib.load_manifest('starmac_tools')

import sys
from optparse import OptionParser
import scipy.io as sio
import numpy as np
from pylab import plt

from starmac_tools.load_bag import BagLoader
from starmac_tools.postprocessing.generic import Dummy
from starmac_tools.postprocessing.state import process_state
from starmac_tools.postprocessing.control import process_control_mode_output
from starmac_tools.postprocessing.imu import process_imu
from starmac_tools.postprocessing.asctec import process_asctec_ctrl_input
from starmac_tools.plotting.util import newfig, show, plot_traj, save_all_figs
from starmac_tools.plotting.timeseries_cursor import TimeSeriesCursorSynchronizer
from starmac_tools.timeseries import uniform_resample

TRAJ_PLOT_OPTIONS = dict(label='trajectory', ls='-', marker='.', color='blue', lw=1, ms=1.0, mec='blue', 
                         mew=0.5,  mfc='None', alpha=0.5)

class AscTecControlPlots(object):
    processing_key_cb = False
    def __init__(self, fname, t_start=None, t_end=None, do_plots=True):
        self.input_bag = BagLoader(fname)
        self.t_start = t_start
        self.t_end = t_end
        self.process_data()
        if do_plots:
            self.do_plots()
        
    def show(self):
        plt.show()
        
    def process_data(self):
        v = Dummy()
        v.state = Dummy()
        v.control = Dummy()
        v.imu = Dummy()
        v.asctec_ctrl_input = Dummy()
        
        # kludge:
        if 'odom' in self.input_bag.get_topics():
            odom_topic = 'odom'
        elif 'estimator/output' in self.input_bag.get_topics():
            odom_topic = 'estimator/output'
        # following should not be here!!!
        elif 'dual_ekf_node/output' in self.input_bag.get_topics():
            odom_topic = 'dual_ekf_node/output'
        else:
            raise "Can't find an odometry topic"
        process_state(self.input_bag, odom_topic, v.state)
        process_control_mode_output(self.input_bag, 'controller_mux/output', v.control)
        process_imu(self.input_bag, 'asctec/imu', v.imu)
        process_asctec_ctrl_input(self.input_bag, 'asctec/CTRL_INPUT', v.asctec_ctrl_input)
        tout, data_out = uniform_resample((('linear', v.imu.t[1:], v.imu.ori_ypr[1:,0]), 
                                           ('linear', v.state.t[1:], v.state.ori_ypr[1:,0])), 0.02)
        self.v = v

        # setup istart and iend for each group of data:
        vars = (self.v.control, self.v.imu, self.v.asctec_ctrl_input, self.v.state)
        if self.t_start is not None:
            for w in vars:
                w.istart = np.where(w.t >= self.t_start)[0][0]
        else:
            for w in vars:
                w.istart = 1
                
        if self.t_end is not None:
            for w in vars:
                w.iend = np.where(w.t >= self.t_end)[0][0]
        else:
            for w in vars:
                w.iend = None
                
        
    def do_plots(self):
        self.cur_sync = TimeSeriesCursorSynchronizer()
        self.plot_axis_control('roll')
        self.plot_axis_control('pitch')
        self.plot_axis_control('yaw')
        self.plot_alt_control()
        self.plot_top_view()
        self.plot_side_view_east()
        self.plot_side_view_north()
        self.plot_east_vs_time()
        self.plot_north_vs_time()
        self.plot_r_vs_time()
        self.plot_r_histogram()
        
    def _timeseries_click_cb(self, event):
        if event.button == 2:
            #print 'button=%d, x=%d, y=%d, xdata=%f, ydata=%f' \
            #    % (event.button, event.x, event.y, event.xdata, event.ydata)
            if event.key is None:
                self.cur_sync.set_cursors(event.xdata)
            elif event.key == 'shift':
                self.cur_sync.add_axvline(event.xdata)
            elif event.key == 'control':
                self.cur_sync.sync_xrange(event.inaxes)
            else:
                pass
            
    def _timeseries_key_release_cb(self, event):
        print "Handling key callback, event.key = ", event.key, " event.inaxes = ", id(event.inaxes)
        if not self.processing_key_cb:
            self.processing_key_cb = True
            if event.key == 'x': # Clear all axvlines
                self.cur_sync.clear_axvlines()
            elif event.key == 'L': # Toggle legend
                if event.inaxes.get_legend() == None:
                    event.inaxes.legend()
                else:
                    event.inaxes.legend_ = None
                event.inaxes.get_figure().canvas.draw()
            self.processing_key_cb = False
            

    def _timeseries_postplot(self):
        ax = plt.gca()
        fig = ax.get_figure()
        self.cur_sync.add_axis(ax)
        cid_btn = fig.canvas.mpl_connect('button_press_event', self._timeseries_click_cb)
        cid_key = fig.canvas.mpl_connect('key_release_event', self._timeseries_key_release_cb)
        
    def plot_axis_control(self, axis, 
                          show_mux_cmd=True, show_asctec_cmd=True, show_vicon_meas=True, show_imu_meas=True):
        axismap = {'roll': (self.v.control.roll_cmd, 2, +1, self.v.asctec_ctrl_input.roll_cmd, -1),
                   'pitch': (self.v.control.pitch_cmd, 1, -1, self.v.asctec_ctrl_input.pitch_cmd, -1),
                   'yaw': (self.v.control.yaw_cmd, 0, -1, self.v.asctec_ctrl_input.yaw_rate_cmd, +1) # not sure about the multiplier here
                   }
        control_mode_cmd, state_axis, imu_mult, asctec_cmd, asctec_cmd_mult = axismap[axis]
        newfig("%s Axis Control" % axis.capitalize(), "time [s]", "%s [deg]" % axis.capitalize())
        # np.clip() and the [1:] stuff in the following to attempt deal with bogus initial data points in IMU data:
        if show_mux_cmd:
            plt.plot(self.v.control.t[self.v.control.istart:self.v.control.iend],
                     control_mode_cmd[self.v.control.istart:self.v.control.iend], label='cmd (from mux)')
        if show_vicon_meas:
            plt.plot(self.v.state.t[self.v.state.istart:self.v.state.iend], 
                     self.v.state.ori_ypr[self.v.state.istart:self.v.state.iend, state_axis], label='meas (Vicon)')
        if show_imu_meas:
            plt.plot(np.clip(self.v.imu.t[self.v.imu.istart:self.v.imu.iend], 0, np.inf), 
                     imu_mult*self.v.imu.ori_ypr[self.v.imu.istart:self.v.imu.iend, state_axis], label='meas (IMU)')
        if show_asctec_cmd and axis is not 'yaw':
            plt.plot(self.v.asctec_ctrl_input.t[self.v.asctec_ctrl_input.istart:self.v.asctec_ctrl_input.iend], 
                     asctec_cmd_mult*asctec_cmd[self.v.asctec_ctrl_input.istart:self.v.asctec_ctrl_input.iend],
                    label='cmd (AscTec)')
        # Plot difference between vicon and imu: (broken, comment it out for now..)
#        tout, data_out = uniform_resample((('linear', self.v.imu.t[self.v.asctec_ctrl_input.istart:self.v.asctec_ctrl_input.iend], 
#                                                      self.v.imu.ori_ypr[self.v.asctec_ctrl_input.istart:self.v.asctec_ctrl_input.iend,state_axis]), 
#                                           ('linear', self.v.state.t[self.v.state.istart:self.v.state.iend], 
#                                                      self.v.state.ori_ypr[self.v.state.istart:self.v.state.iend, state_axis])), 
#                                           0.02)
#        plt.plot(tout, imu_mult*data_out[0][0] - data_out[1][0], label='IMU - Vicon')
        plt.legend()
        self._timeseries_postplot()
        
    def plot_alt_control(self):
        newfig("Altitude Control", "time [s]", "Alt [m]")
        plt.plot(self.v.control.t[self.v.control.istart:self.v.control.iend], self.v.control.alt_cmd[self.v.control.istart:self.v.control.iend], label="cmd")
        plt.plot(self.v.state.t[self.v.state.istart:self.v.state.iend], self.v.state.up[self.v.state.istart:self.v.state.iend], label="meas (Vicon)")
        plt.legend()
        self._timeseries_postplot()
        
    def plot_top_view(self):
        newfig('Top View', 'East [m]', 'North [m]', equal_axes=True)
        plot_traj(self.v.state.east[self.v.state.istart:self.v.state.iend], 
                  self.v.state.north[self.v.state.istart:self.v.state.iend], **TRAJ_PLOT_OPTIONS)
        
    def plot_side_view_east(self):
        newfig('Side View (East)', 'East [m]', 'Up [m]', equal_axes=True)
        plot_traj(self.v.state.east[self.v.state.istart:self.v.state.iend], 
                  self.v.state.up[self.v.state.istart:self.v.state.iend], **TRAJ_PLOT_OPTIONS)
        
    def plot_side_view_north(self):
        newfig('Side View (North)', 'North [m]', 'Up [m]', equal_axes=True)
        plot_traj(self.v.state.north[self.v.state.istart:self.v.state.iend], 
                  self.v.state.up[self.v.state.istart:self.v.state.iend], **TRAJ_PLOT_OPTIONS)
        
    def plot_east_vs_time(self):
        newfig('East vs. Time', 'time [s]', 'East [m]')
        plot_traj(self.v.state.t[self.v.state.istart:self.v.state.iend], 
                  self.v.state.east[self.v.state.istart:self.v.state.iend], label="meas (Vicon)")
        self._timeseries_postplot()
        
    def plot_north_vs_time(self):
        newfig('North vs. Time', 'time [s]', 'North [m]')
        plot_traj(self.v.state.t[self.v.state.istart:self.v.state.iend], 
                  self.v.state.north[self.v.state.istart:self.v.state.iend], label="meas (Vicon)")
        self._timeseries_postplot()
        
    def plot_r_vs_time(self):
        newfig('sqrt(x^2 + y^2) vs. Time', 'time [s]', 'North [m]')
        t = self.v.state.t[self.v.state.istart:self.v.state.iend]
        r = np.sqrt(self.v.state.north[self.v.state.istart:self.v.state.iend]**2 + \
                    self.v.state.east[self.v.state.istart:self.v.state.iend]**2 )
        plot_traj(t, r, label="meas (Vicon)")
        self._timeseries_postplot()
        
    def plot_r_histogram(self):
        newfig('Histogram of sqrt(x^2 + y^2)', 'distance from origin [m]', 'Count')
        t = self.v.state.t[self.v.state.istart:self.v.state.iend]
        r = np.sqrt(self.v.state.north[self.v.state.istart:self.v.state.iend]**2 + \
                    self.v.state.east[self.v.state.istart:self.v.state.iend]**2 )
        plt.hist(r,50)
        
    def save_all_figs(self, output_dir, prefix="", ext=".eps", **savefig_kwargs):
        save_all_figs(output_dir, prefix=prefix, ext=ext, **savefig_kwargs)
        
        
if __name__ == "__main__":
    parser = OptionParser(usage="usage: %prog [options] bagfile")
    parser.add_option('-s', '--start', dest='t_start')
    parser.add_option('-e', '--end', dest='t_end')
    (options, args) = parser.parse_args()
    t_start = None
    t_end = None
    if options.t_start is not None:
        t_start = float(options.t_start)
    if options.t_end is not None:
        t_end = float(options.t_end)
    self = AscTecControlPlots(args[0], t_start, t_end)
    self.show()
    