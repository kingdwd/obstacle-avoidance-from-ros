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
"""
modes_to_standby_nonrosh.py

This script is used to bring additional control modes to standby. It also necessarily brings the
controller to operational first.

This replaces the original modes_to_standby rosh nodelet and doesn't depend on rosh and its many
dependencies.

Usage:
rosrun flyer_common/modes_to_standby_nonrosh.py_modes:='mode_1 mode_2 ...'
"""
import roslib
roslib.load_manifest('flyer_common')
roslib.load_manifest('flyer_controller') # probably a no-no ...
import rospy
from rospy import loginfo
import rosgraph
from flyer_controller.msg import controller_cmd
from flyer_controller.msg import control_mode_cmd


def get_node_subscriptions(master, node_name):
    pubs, subs, svcs = master.getSystemState()
    node_subs = []
    for topic, subscribers in subs:
        if node_name in subscribers:
            node_subs.append(topic)
    return node_subs

def resolve(name):
    return rosgraph.names.resolve_name(name, '')
    
def is_node_subscribed_to_topic(master, node_name, topic_name):
    return resolve(topic_name) in get_node_subscriptions(master, node_name)
    
rospy.init_node('modes_to_standby')

ns = rospy.get_namespace()
loginfo("namespace: " + ns)
stby_modes = rospy.get_param('~modes').split() #parameters['~'].modes() #myargv(argv=sys.argv)
loginfo('The following modes will be brought to STANDBY: %s' % stby_modes)
rospy.sleep(5.0)
rospy.loginfo('Bringing nodes to standby...')

manager_node_name = resolve(ns + 'manager')

master = rosgraph.Master('')

print get_node_subscriptions(master, manager_node_name)

while not (is_node_subscribed_to_topic(master, ns + 'manager', ns + 'controller/cmd') 
            or is_node_subscribed_to_topic(master, ns + 'controller', ns + 'controller/cmd')):
    loginfo('waiting for manager (or controller) to subscribe to controller_cmd...')
    rospy.sleep(1.0)
    
loginfo('I see manager subscribed to controller/cmd, proceeding..')

# Tell the controller to go to operational:
ccmd_pub = rospy.Publisher(ns + 'controller/cmd', controller_cmd, latch=True)
ccmd_pub.publish(controller_cmd('mode operational'))

rospy.sleep(3.0) # would be better to check that the last command succeeded, oh well..

# Now tell each mode listed in ~modes to go to STANDBY:
for stby_mode in stby_modes:
    loginfo('Bringing mode %s to STANDBY...' % stby_mode)
    ccmd_pub.publish(control_mode_cmd('control_mode to_standby %s' % stby_mode))
    rospy.sleep(2.0)

loginfo('modes_to_standby script finished')