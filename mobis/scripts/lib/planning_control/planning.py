# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Float64,Int16,Float32MultiArray
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
import tf


class pathReader :  ## 텍스트 파일에서 경로를 출력 ##
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)



    def read_txt(self,file_name):
        full_file_name=self.file_path+'/path/' + file_name + '.txt'
        openFile = open(full_file_name, 'r')
        out_path=Path()
        
        out_path.header.frame_id='/map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            # read_pose.pose.position.z=float(tmp[2])
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path.poses.append(read_pose)
        
        
        openFile.close()
        return out_path ## 읽어온 경로를 global_path로 반환 ##
      



def findLocalPath(ref_path,status_msg): ## global_path와 차량의 status_msg를 이용해 현재waypoint와 local_path를 생성 ##
    out_path=Path()
    current_x=status_msg.position.x
    current_y=status_msg.position.y
    current_waypoint=0
    min_dis=float('inf')

    for i in range(len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i


    if current_waypoint+100 > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)
    else :
        last_local_waypoint=current_waypoint+100



    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=1
        out_path.poses.append(tmp_pose)

    return out_path,current_waypoint ## local_path와 waypoint를 반환 ##


def calcCurrentWaypoint(ref_path, status_msg):
    current_x=status_msg.position.x
    current_y=status_msg.position.y
    current_waypoint=0
    min_dis=float('inf')

    for i in range(len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i

    return current_waypoint



class velocityPlanning :
    def __init__(self,car_max_speed,road_friction):
        self.car_max_speed = car_max_speed/3.6
        self.road_friction = road_friction
 
    def curveBasedVelocity(self,global_path,point_num):
        out_vel_plan=[]
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num,len(global_path.poses)-point_num):
            x_list=[]
            y_list=[]
            for box in  range(-point_num,point_num):
                x=global_path.poses[i+box].pose.position.x
                y=global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))
            
            x_matrix=np.array(x_list)
            y_matrix=np.array(y_list)
            x_trans=x_matrix.T
            

            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)  #0.7
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
            out_vel_plan.append(self.car_max_speed)
        
        return out_vel_plan

class cruiseControl: ## ACC(advanced cruise control) 적용 ##
    def __init__(self,object_vel_gain,object_dis_gain):
        self.object=[False,0]
        self.traffic=[False,0]
        self.Person=[False,0]
        self.object_vel_gain=object_vel_gain
        self.object_dis_gain=object_dis_gain


    def checkObject(self,ref_path,global_vaild_object,local_vaild_object,tl=[]): ## 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호) ##
        self.object=[False,0]
        self.traffic=[False,0]
        self.Person=[False,0]
        if len(global_vaild_object) >0  :
            min_rel_distance=float('inf')
            for i in range(len(global_vaild_object)):
                for path in ref_path.poses :      

                    if global_vaild_object[i][0]==1 :

                        dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))

                        if dis<2.5:
                            rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                            
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.object=[True,i]
                            

                    if global_vaild_object[i][0]==0 :
                    
                        dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))

                        if dis<4.35:
                            
                            rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.Person=[True,i]


                    if global_vaild_object[i][0]==3 :
                        traffic_sign='STOP'

                        if len(tl)!=0  and  global_vaild_object[i][3] == tl[0] :
                            if tl[1] == 48 or tl[1]==16   :   #
                                traffic_sign ='GO'
                        if traffic_sign =='STOP':
                            dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                            
                            if dis<9 :
                                rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                                if rel_distance < min_rel_distance:
                                    min_rel_distance=rel_distance
                                    self.traffic=[True,i]

                         
                    

    



    def acc(self,local_vaild_object,ego_vel,target_vel): ## advanced cruise control 를 이용한 속도 계획 ##
        ego_vel = ego_vel
        out_vel=min(target_vel, ego_vel + 5)        
        
        if self.object[0] == True :
            print("ACC ON_vehicle")
            
            front_vehicle=[local_vaild_object[self.object[1]][1],local_vaild_object[self.object[1]][2],local_vaild_object[self.object[1]][3]]
            print(ego_vel,front_vehicle[2])   
            time_gap=0.8
            default_space=8
            
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))
            
            vel_rel=((front_vehicle[2]/3.6)-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel 
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel

            dx = front_vehicle[0]
            dy = front_vehicle[1]

            t_dis = sqrt(pow(dx,2)+pow(dy,2))
            print(out_vel)
            

        
        if self.Person[0]==True:
            # print("ACC ON_person")
            Pedestrian=[local_vaild_object[self.Person[1]][1],local_vaild_object[self.Person[1]][2],local_vaild_object[self.Person[1]][3]]
            time_gap=0.8
            default_space=8
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(Pedestrian[0],2)+pow(Pedestrian[1],2))-3
            
            vel_rel=(Pedestrian[2]-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel - 5
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel
            dx =  Pedestrian[0]
            dy =  Pedestrian[1]

            t_dis = sqrt(pow(dx,2)+pow(dy,2))


        if self.traffic[0] == True :
            # print("Traffic_ON")   
            front_vehicle=[local_vaild_object[self.traffic[1]][1],local_vaild_object[self.traffic[1]][2],local_vaild_object[self.traffic[1]][3]]
            time_gap=0.8
            default_space=3
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-3
            
            vel_rel=(0-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel

            if dis_rel < 3 :
                out_vel = 0
        return out_vel

class val_object :
    def __init__(self):
        self.num_of_objects=0
        self.pose_x=[]
        self.pose_y=[]
        self.velocity=[]
        self.object_type=[]
        
class vaildObject : ## 장애물 유무 확인 (차량, 사람, 정지선 신호) ##

    def __init__(self,stop_line=[]):
        self.stop_line=stop_line

    def get_object(self,object_data):
        
        self.all_object=val_object()
        
        self.all_object.num_of_objects=object_data.num_of_npcs+object_data.num_of_obstacle+object_data.num_of_pedestrian

        for num in range(object_data.num_of_npcs) :
            self.all_object.object_type.append(object_data.npc_list[num].type)
            self.all_object.pose_x.append(object_data.npc_list[num].position.x)
            self.all_object.pose_y.append(object_data.npc_list[num].position.y)
            self.all_object.velocity.append(object_data.npc_list[num].velocity.x)

        for num in range(object_data.num_of_obstacle) :
            self.all_object.object_type.append(object_data.obstacle_list[num].type)
            self.all_object.pose_x.append(object_data.obstacle_list[num].position.x)
            self.all_object.pose_y.append(object_data.obstacle_list[num].position.y)
            self.all_object.velocity.append(object_data.obstacle_list[num].velocity.x)

        for num in range(object_data.num_of_pedestrian) :
            self.all_object.object_type.append(object_data.pedestrian_list[num].type)
            self.all_object.pose_x.append(object_data.pedestrian_list[num].position.x)
            self.all_object.pose_y.append(object_data.pedestrian_list[num].position.y)
            self.all_object.velocity.append(object_data.pedestrian_list[num].velocity.x)

    def calc_vaild_obj(self,ego_pose):  # x, y, heading
        global_object_info=[]
        local_object_info=[]
        
        if self.all_object.num_of_objects > 0:
            tmp_theta=ego_pose[2]
            tmp_translation=[ego_pose[0],ego_pose[1]]
            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                            [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                            [0,0,1]])
            tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],
                                [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],
                                [0,0,1]])

            for num in range(self.all_object.num_of_objects):
                global_result=np.array([[self.all_object.pose_x[num]],[self.all_object.pose_y[num]],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_object_info.append([self.all_object.object_type[num],self.all_object.pose_x[num],self.all_object.pose_y[num],self.all_object.velocity[num]])
                    local_object_info.append([self.all_object.object_type[num],local_result[0][0],local_result[1][0],self.all_object.velocity[num]])
            
            
            for line in self.stop_line:
                global_result=np.array([[line[0]],[line[1]],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_object_info.append([3,line[0],line[1],line[2]])
                    local_object_info.append([3,local_result[0][0],local_result[1][0],line[2]])
        

        return global_object_info,local_object_info