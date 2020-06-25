#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import dynamic_reconfigure.client

class VelocityChanger:
    def __init__(self, dr_server = '/move_base/TrajectoryPlannerROS', timeout = 30):
        self.dr_server = dr_server
        self.timeout = timeout
        self.client = dynamic_reconfigure.client.Client(self.dr_server, self.timeout, config_callback=self.callback)
        self.client_config = self.client.get_configuration(timeout = None)
    
    def callback(self):
        self.client_config = self.client.get_configuration(timeout = None)
        rospy.loginfo("change velocity")
    
    def change(self,request_config):
        if not type(request_config) is dict:
            rospy.logerr("Argument is not type 'dict'. Not changing.")
            return False

        for key in request_config.keys():
            if not key in self.client_config.keys():
                rospy.logerr("Key's %s is not included of %s server. Not changing.",key, self.dr_server)
                return False

        self.client.update_configuration(request_config)
        rospy.loginfo("velocity_changer is done.")
        return True
        
#test
if __name__ == '__main__':
    rospy.init_node("velocity_changer_test")

    vc = VelocityChanger(dr_server = '/move_base/DWAPlannerROS', timeout =5)

    flag = True
    r= rospy.Rate(0.5)

    while not rospy.is_shutdown():
        if flag:
            rospy.loginfo("default_mode")
            vc.change({'max_vel_x':0.6,'min_vel_x': -0.4, 'max_vel_trans':0.6, 'max_vel_theta':0.8})
        else:
            rospy.loginfo("turbo_mode")
            vc.change({'max_vel_x':0.9,'min_vel_x': -0.6, 'max_vel_trans':0.9, 'max_vel_theta':1.2})

        flag = not flag
        r.sleep()
