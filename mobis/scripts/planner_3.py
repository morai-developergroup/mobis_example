#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight,CollisionData
from lib.lateral_control.lateral_control import purePursuit, pidController
from lib.planning_control.planning import pathReader, findLocalPath,cruiseControl,vaildObject,velocityPlanning

import tf
from math import pi
from mgeo_loader import mgeo_loader

class gen_planner():

    def __init__(self):
        rospy.init_node('gen_planner', anonymous=True)

        #publisher
        global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1) ## Vehicl Control
        object_pub = rospy.Publisher('/object_point',PointCloud,queue_size=1)
        
        ctrl_msg = CtrlCmd()
        
        
        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber 
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.objectInfoCB) ## Object information Subscriber
        

        #def
        self.is_status = False ## 차량 상태 점검
        self.is_obj = False ## 장애물 상태 점검

        #mgeo_loader
        self.mgeo_loader = mgeo_loader("mobis", "k-city")

        #path_reader
        path_reader = pathReader('mobis') ## 경로 파일의 위치
        self.global_path = path_reader.read_txt('kcity')

        #pure_pursuit
        pure_pursuit = purePursuit() ## purePursuit import

        #pid
        pid = pidController() ## pidController import

        #velocity_planner
        target_velocity = 80 #kph
        vel_planner = velocityPlanning(target_velocity,0.15) #target_vel, road_friction
        vel_profile = vel_planner.curveBasedVelocity(self.global_path,100)

        #cruiseControl
        self.cc = cruiseControl(0.5, 1.0) ## cruiseControl import (object_vel_gain, object_dis_gain)
        self.vo = vaildObject() ## 장애물 유무 확인 (TrafficLight)
    
        #time var        
        rate = rospy.Rate(60) # 60hz
        count = 0

        self.count = 0
        while not rospy.is_shutdown():
            if self.is_status and self.is_obj:
                
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 
                
                ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)
                
                self.vo.get_object(self.object_info_msg)
                global_obj,local_obj = self.vo.calc_vaild_obj([self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi])

                self.cc.checkObject(local_path,global_obj,local_obj)

                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용

                ctrl_msg.steering = -pure_pursuit.steering_angle()/180*pi
                
                cc_vel = self.cc.acc(local_obj,self.status_msg.velocity.x,vel_profile[self.current_waypoint]) ## advanced cruise control 적용한 속도 계획

                control_input = pid.pid(cc_vel,self.status_msg.velocity.x) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)

                if control_input > 0 :
                    ctrl_msg.accel = control_input
                    ctrl_msg.brake = 0
                else :
                    ctrl_msg.accel = 0
                    ctrl_msg.brake = -control_input


                local_path_pub.publish(local_path) ## Local Path 출력
                ctrl_pub.publish(ctrl_msg) ## Vehicl Control 출력


                object_point = PointCloud()
                object_point.header.frame_id = 'map'
                for npc_list in self.object_info_msg.npc_list :
                    tmp_point = Point32()
                    tmp_point.x = npc_list.position.x
                    tmp_point.y = npc_list.position.y
                    tmp_point.z = 0
                    object_point.points.append(tmp_point)
                    
                object_pub.publish(object_point)
            
                if count == 100 : ## global path 출력
                    global_path_pub.publish(self.global_path)
                    self.mgeo_loader.getAllLinks()
                    count = 0
                    
                count += 1
                rate.sleep()



    def statusCB(self,data): ## Vehicl Status Subscriber 
        self.is_status = True
        self.status_msg = data

        br = tf.TransformBroadcaster()
        br.sendTransform((data.position.x, data.position.y, data.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (data.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")


    def objectInfoCB(self,data): ## Object information Subscriber
        self.is_obj = True        
        self.object_info_msg = data
        

if __name__ == '__main__':
    try:
        kcity_pathtracking=gen_planner()
    except rospy.ROSInterruptException:
        pass