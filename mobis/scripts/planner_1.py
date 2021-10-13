#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
import time
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd
from lib.lateral_control.lateral_control import purePursuit
from lib.planning_control.planning import pathReader, findLocalPath

import tf
from math import pi
from mgeo_loader import mgeo_loader

class gen_planner():

    def __init__(self):        
        rospy.init_node('gen_planner', anonymous=True)

        #publisher
        global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1) ## local_path publisher
        local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1) ## Vehicl Control
        
        ctrl_msg = CtrlCmd()
        
        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB)
        
        #def
        self.is_status = False ## 차량 상태 점검

        #mgeo_loader
        self.mgeo_loader = mgeo_loader("mobis", "k-city")
        

        #path_reader
        path_reader = pathReader('mobis') ## 경로 파일의 Package 위치
        self.global_path = path_reader.read_txt('kcity')

        #pure_pursuit
        pure_pursuit = purePursuit()
    
        #time var
        rate = rospy.Rate(60)# 60hz
        count = 0
        while not rospy.is_shutdown():

            if self.is_status:
                
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint = findLocalPath(self.global_path,self.status_msg) 
                
                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용

                

                ctrl_msg.longlCmdType = 2 # 1: Throttle control, 2: Velocity control, 3: Acceleration control
                ctrl_msg.velocity = 50 #km/h

                ctrl_msg.steering = -pure_pursuit.steering_angle()/180*pi

                local_path_pub.publish(local_path) ## Local Path 출력
                ctrl_pub.publish(ctrl_msg) ## Vehicl Control 출력

                
            
                if count == 100 : ## global path 출력
                    global_path_pub.publish(self.global_path)
                    self.mgeo_loader.getAllLinks()
                    count=0
                count += 1
                rate.sleep()



    def statusCB(self,data): 
        self.is_status = True
        self.status_msg = data

        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")

      
    
if __name__ == '__main__':
    try:
        kcity_pathtracking=gen_planner()
    except rospy.ROSInterruptException:
        pass