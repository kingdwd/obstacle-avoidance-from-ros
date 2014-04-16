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
topics.py

Functions to do with topics
"""
import roslib
roslib.load_manifest('starmac_roshlib')
from rospy import loginfo, sleep
from rosh import nodes, info, topics

def is_publishing(node_name, topic_name):
    if nodes is None:
        return False
    else:
        topics_published = [info(s).name for s in info(nodes[node_name]).pubs]
        return True in [s.find(topic_name) >= 0 for s in topics_published]

def is_subscribing(node_name, topic_name):
    if nodes is None:
        return False
    else:
        topics_subscribed = [info(s).name for s in info(nodes[node_name]).subs]
        return True in [s.find(topic_name) >= 0 for s in topics_subscribed]

def wait_for_topic(node_name, topic_name, sleep_time=0.5):
    loginfo("Waiting for node %s to publish topic %s" % (node_name, topic_name))
    while not is_publishing(node_name, topic_name):
        #loginfo("Still waiting for topic %s from node %s" % (topic_name, node_name))
        sleep(sleep_time)
    loginfo("Node %s is now publishing topic %s" % (node_name, topic_name))
    
def wait_for_topic_sub(node_name, topic_name, sleep_time=0.5):
    loginfo("Waiting for node %s to subscribe to topic %s" % (node_name, topic_name))
    while not is_subscribing(node_name, topic_name):
        #loginfo("Still waiting for topic %s from node %s" % (topic_name, node_name))
        sleep(sleep_time)
    loginfo("Node %s is now subscribing to topic %s" % (node_name, topic_name))
    
def topics_list():
    all_topics = []
    for t in topics:
        all_topics.extend(list(t._list()))
    return all_topics

def wait_for_topic_existence(topic_name, sleep_time=0.5):
    loginfo("Waiting for topic %s to show up in topics list" % topic_name)
    while not topic_name in topics_list():
        sleep(sleep_time)
    loginfo("Topic %s is now in topics list" % topic_name)
