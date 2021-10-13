#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy ,rospkg
from math import pi
from morai_msgs.msg  import EgoVehicleStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point32
import tf
from sensor_msgs.msg import PointCloud

# rospack=rospkg.RosPack()
# pkg_path = rospack.get_path('method_ex')
# current_path = pkg_path + '/scripts/'
# sys.path.append(current_path)
# mgeo_lib_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_parser/lib/mgeo/'))
# # print('mgeo_lib_path: {}'.format(mgeo_lib_path))
# sys.path.append(mgeo_lib_path)


from lib.mgeo_parser.lib.mgeo.class_defs import *

class mgeo_loader():
    def __init__(self, pkg_name , map_name):
        rospack=rospkg.RosPack()
        self.file_path = rospack.get_path(pkg_name)
        mgeo_data_path = self.file_path+'/scripts/lib/mgeo_parser/mgeo_data/'+map_name

        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(mgeo_data_path)

        node_set = mgeo_planner_map.node_set
        # link 관련 메소드 참고 👍 method_ex/scripts/lib/mgeo_parser/lib/mgeo/class_defs
        link_set = mgeo_planner_map.link_set

        self.gm_pub = rospy.Publisher('/global_map',PointCloud,queue_size=1)

        self.nodes=node_set.nodes
        self.links=link_set.lines

    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:
                # print(self.links[link_idx].get_max_speed_kph())
                tmp_point=Point32()
                tmp_point.x=link_point[0]
                tmp_point.y=link_point[1]
                tmp_point.z=0
                all_link.points.append(tmp_point)
        
        self.gm_pub.publish(all_link)

        return all_link

    def getSubLinks(self, links_list):
        out_path=Path()
        out_path.header.frame_id='/map'
        
        for links_idx in links_list :

            for links_point in self.links[links_idx].points:
                read_pose=PoseStamped()
                read_pose.pose.position.x=links_point[0]
                read_pose.pose.position.y=links_point[1]
                read_pose.pose.position.z=0
                read_pose.pose.orientation.x=0
                read_pose.pose.orientation.y=0
                read_pose.pose.orientation.z=0
                read_pose.pose.orientation.w=1
                out_path.poses.append(read_pose)
        
        return out_path



if __name__ == '__main__':
    try:
        rospy.init_node('mgeo_test', anonymous=True)
        test=mgeo_loader('method_ex', 'k-city')
        while not rospy.is_shutdown():
            test.getAllLinks()
    except rospy.ROSInterruptException:
        pass


