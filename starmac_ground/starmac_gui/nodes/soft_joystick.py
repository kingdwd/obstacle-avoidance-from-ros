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

import roslib; roslib.load_manifest('starmac_gui')
import rospy

from std_msgs.msg import String

from sensor_msgs.msg import Joy

from PySide.QtCore import QThread 
from PySide.QtGui import QApplication, QPushButton
from PySide import QtGui, QtCore
import sys
import signal

from starmac_gui.ui.joystick import Ui_MainWindow as joystick_Ui_MainWindow

NUM_BUTTONS = 10
NUM_AXES = 5

def handle_sigint(*args):
    sys.stderr.write("\rSIGINT")
    QtGui.QApplication.quit()

class SoftJoystick(QtCore.QThread):
    ui = None
    pub = None
    update_rate = 20.0
    def run(self):
        i = 0
        while not rospy.is_shutdown():
            self.send()
            self.msleep(1000.0/self.update_rate)
            i += 1
        print "is_shutdown true, returning from run()"
        
    def send(self):
        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()
        buttons = (self.ui.button_0, self.ui.button_1, self.ui.button_2, self.ui.button_3, self.ui.button_4, 
                   self.ui.button_5, self.ui.button_6, self.ui.button_7, self.ui.button_8, self.ui.button_9, 
                   self.ui.button_10, self.ui.button_11)
        joy_msg.buttons = [b.isChecked() for b in buttons]
        joy_msg.axes = [mult*s.value()/100.0 for s, mult in 
                        zip((self.ui.rollSlider, self.ui.pitchSlider, self.ui.yawSlider, self.ui.altSlider), (-1.0, 1.0, -1.0, 1.0))]
        for label, value in zip((self.ui.rollLabel, self.ui.pitchLabel, self.ui.yawLabel, self.ui.altLabel), joy_msg.axes):
            label.setText('%0.2f' % value)
        
        self.pub.publish(joy_msg)
        #print self.ui.button_0.isChecked(), self.ui.button_1.isChecked()

class UiForm(QtGui.QMainWindow):
    def __init__(self, parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.ui = joystick_Ui_MainWindow()
        self.ui.setupUi(self)

class GUI:
    finished = False
    
    def __init__(self, app):
        self.app = app
        self.form = UiForm()
        self.form.show()
        self.uibuttons = []
        
        
        self.soft_joystick = SoftJoystick()
        self.soft_joystick.ui = self.form.ui
        self.soft_joystick.update_rate = 20
        self.soft_joystick.finished.connect(self.thread_finished_callback)
        
        self.init_connections()

        # A publisher for messages
        self.pub = rospy.Publisher('joy', Joy)
        self.soft_joystick.pub = self.pub

        self.soft_joystick.start()

    def init_connections(self):
        sfu = self.form.ui
        buttons = (sfu.button_0, sfu.button_1, sfu.button_2, sfu.button_3, sfu.button_4, 
                   sfu.button_5, sfu.button_6, sfu.button_7, sfu.button_8, sfu.button_9, 
                   sfu.button_10, sfu.button_11)
        axes = (sfu.rollSlider, sfu.pitchSlider, sfu.yawSlider, sfu.altSlider)
        for b in buttons:
            b.clicked.connect(self.soft_joystick.send)
        for s in axes:
            s.valueChanged.connect(self.soft_joystick.send)

    def pub_callback(self):
        print "clicked"
        self.pub.publish('Thank You!')

    def sub_callback(self, msg):
        print 'Got message:', msg.data
        
    def thread_finished_callback(self): 
        print 'Thread finished'
        self.finished = True
        self.app.quit()
        
    def button_callback(self, btn):
        print 'Button %d pressed, state is %s' % (btn, str(self.uibuttons[btn].isChecked()))

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('soft_joystick')

    # Initialize Qt
    app = QApplication(sys.argv)
    
    # Borrowed from rxlaunch:
    # Sets up signal handling so SIGINT closes the application,
    # following the solution given at [1].  Sets up a custom signal
    # handler, and ensures that the Python interpreter runs
    # occasionally so the signal is handled.  The email thread at [2]
    # explains why this is necessary.
    #
    # [1] http://stackoverflow.com/questions/4938723/#4939113
    # [2] http://www.mail-archive.com/pyqt@riverbankcomputing.com/msg13757.html
    signal.signal(signal.SIGINT, handle_sigint)
    timer = QtCore.QTimer()
    timer.start(250)
    timer.timeout.connect(lambda: None)  # Forces the interpreter to run every 250ms

    gui = GUI(app)

    app.exec_()