#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, UC Regents
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
static_transforms.py

Uses a tf broadcaster to broadcast static transforms as necessary. These can be specific to
a particular flyer (e.g. the Vicon-to-IMU offset) or global, as in the /enu to /ned offset.

Parameters:
~mode: specifies what to broadcast:
'flyer' - broadcast transform(s) specific to flyer (see 'flyer' parameter)
'global' - broadcast global transform(s)
'both' - broadcast both flyer and global transforms

~flyer: specifies the name of the flyer (e.g. 'grey' or 'yellow')

~period: interval at which to publish the transform(s), in milliseconds
"""
import roslib; roslib.load_manifest('flyer_common')
import rospy
import tf
from math import radians

tft = tf.transformations

global_transforms = (# parent child trans rot (Y-P-R, degrees)
                     ('/enu','/ned', (0,0,0), (90, 0, 180)),
                    )
class StaticTransformBroadcaster:
    
    def __init__(self):
        rospy.init_node('static_transforms')
        self.tfb = tf.TransformBroadcaster()
        self.mode = rospy.get_param('~mode')
        self.flyer = rospy.get_param('~flyer','')
        self.period = rospy.get_param('~period', 1000)
        self.startup_time = rospy.get_param('~startup_time', 10) # how many seconds to broadcast at faster rate
        self.startup_period = rospy.get_param('~startup_period', 100)
        self.get_frame_params()
        
    def get_frame_params(self):
        self.flyer_transforms = rospy.get_param('frames')
        
    def broadcast_one(self, parent, child, trans, rot, rot_type, rot_unit):
        flyer_prefix = '/'+self.flyer+'/'
        if parent[0] == '/':
            parent_prefix = ''
        else:
            parent_prefix = flyer_prefix
        if child[0] == '/':
            child_prefix = ''
        else:
            child_prefix = flyer_prefix

        assert len(trans) == 3
        assert len(rot_unit) == 0 or rot_unit in ('deg','rad')
        if rot_type.startswith('euler_'):
            assert len(rot) == 3
            rot_seq = rot_type.split('_')[1]
            if rot_unit == 'deg':
                rot_rad = [radians(r) for r in rot]
            else:
                rot_rad = rot
            quat = tft.quaternion_from_euler(rot_rad[0], rot_rad[1], rot_rad[2], rot_seq)
        elif rot_type == 'quaternion':
            assert len(rot) == 4
            raise NotImplementedError
        elif rot_type == 'matrix':
            assert len(rot) == 9
            raise NotImplementedError
        else:
            raise SyntaxError
        self.tfb.sendTransform(trans, quat, rospy.Time.now() + self.dur, 
                               child_prefix + child, parent_prefix + parent)
        
    def broadcast(self):
        for f in self.flyer_transforms.values():
            self.broadcast_one(f['parent'], f['child'], 
                               f['translation'], f['rotation'], f['rot_type'], 
                               f.get('rot_unit',''))
                            
    def start(self):
        # print 'TF prefix:' + self.tfb.getTFPrefix() # d'arggh.. ros-pkg #4943 still hasn't been fixed..
        start = rospy.Time.now()
        now = start
        startup = True
        self.dur = rospy.Duration.from_sec(self.startup_period/1000.0)
        while not rospy.is_shutdown():
            if startup and (now - start).to_sec() > self.startup_time:
                startup = False
                self.dur = rospy.Duration.from_sec(self.period/1000.0)
                rospy.loginfo('Lowering static transform broadcast rate (period: ' + str(self.period/1000.0) + ' s)')
            self.broadcast()
            rospy.sleep(self.dur)
            if startup:
                now = rospy.Time.now()
        
if __name__ == "__main__":
    stb = StaticTransformBroadcaster()
    stb.start()
