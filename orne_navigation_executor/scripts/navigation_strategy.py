#!/usr/bin/env python
# -*- coding: utf-8 -*-

try:
    import roslib; roslib,load_manifest('rospeex_if')
except:
    pass

import rospy
import re

from rospeex_if import ROSpeexInterface
from std_msgs.msg import String

syscommand_pub = rospy.Publisher('syscommand', String, queue_size=10)
rospy.init_node('navigation_strategy', anonymous=True)
r = rospy.Rate(10)

class talk_node(object):

    def __init__(self):

        self._interface = ROSpeexInterface()

    def sr_response(self, message):

        run = re.compile('(?P<run>走行)').search(message)
        start = re.compile('(?P<start>開始)').search(message)

        print 'you said : %s' %message

        if run is not None and start is not None:
            text = u'ナビゲーションを開始します。'
            robot_msg = 'start'

            rospy.loginfo(robot_msg)
            syscommand_pub.publish(robot_msg)


            print 'rospeex reply : %s' %text
            self._interface.say(text, 'ja', 'nict')

    def run(self):
        self._interface.init()
        self._interface.register_sr_response(self.sr_response)
        self._interface.set_spi_config(language='ja',engine='nict')
        rospy.spin()

if __name__ == '__main__':
    try:
        node = talk_node()
        node.run()
    except rospy.ROSInterruptException:
        pass
