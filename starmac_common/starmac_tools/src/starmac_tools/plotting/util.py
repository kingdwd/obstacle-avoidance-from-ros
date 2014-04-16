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

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def _equal_axes(ax):
    ax.set_aspect('equal')
    ax.set_aspect('equal', adjustable='datalim')
    
def newfig(title="", xlabel="", ylabel="", equal_axes=False, grid=True, hold=True):
    fig = plt.figure()
    plt.plot(0,0,'r')
    ax = plt.gca()
    ax.set_title(title)
    if equal_axes:
        _equal_axes(ax)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    plt.grid(grid)
    plt.hold(hold)
    fmwin = plt.get_current_fig_manager().window # this might be Tkinter specific..
    fignum_txt = fmwin.title()
    fmwin.title(fignum_txt + ': ' + title)
    return ax

def new_timeseries_fig(title="", ylabel="", grid=True, hold=True):
    ax = newfig(title, xlabel='time [s]', ylabel=ylabel, equal_axes=False, grid=grid, hold=hold)
    

def plot_traj(x,y, *args, **kwargs):
    plt.plot(x, y, *args, **kwargs)
    plt.plot(x[0], y[0], 'go')
    plt.plot(x[-1], y[-1], 'rx')
    
def show():
    plt.show()
    
def save_all_figs(output_dir, prefix="", ext=".eps", **savefig_kwargs):
    fignums = plt.get_fignums()
    for fignum in fignums:
        f = plt.figure(fignum)
        fname = os.path.join(output_dir, prefix, ("%03d" % fignum) + ext)
        f.savefig(fname, orientation='landscape', **savefig_kwargs)

