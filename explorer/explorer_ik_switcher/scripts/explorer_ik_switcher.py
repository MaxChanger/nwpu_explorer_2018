#!/usr/bin/env python
# coding= utf8
import numpy as np
from ikpy import chain
import rospy
import geometry_msgs
import sensor_msgs.msg
import math
from ikpy import geometry_utils as gu
import explorer_msgs.msg
import sys,os
import tf
def cur_file_dir():
     path = sys.path[0]
     
     if os.path.isdir(path):
         return path
     elif os.path.isfile(path):
         return os.path.dirname(path)
# 定义π值方便后续运算
pi = math.acos(0.0)*2


class explorer_ik_switcher(object):
    """
    explorer_arm_ik_switcher
    构造函数来自chain.Chain.from_urdf_file
    对象构造完成后请务必启用rospy.spin()使得消息接收成立

    :param urdf_file: The path of the URDF file
    :type urdf_file: string
    :param base_elements: List of the links beginning the chain
    :type base_elements: list of strings
    :param last_link_vector: Optional : The translation vector of the tip.
    :type last_link_vector: numpy.array
    :param list active_links: The active links
    :param string Name: The name of the Chain
    """
    def __init__(self,urdf_file, base_elements=["base_link"], last_link_vector=None, base_element_type="link", active_links_mask=None, name="chain"):
        self.arm = chain.Chain.from_urdf_file(urdf_file, base_elements, last_link_vector, base_element_type, active_links_mask, name)
        self.pose = [0.0] * len(self.arm.links)
        self.ik = [0.0] * len(self.arm.links)
        # 初始化ros结点
        rospy.init_node('explorer_ik_switcher',anonymous=True)
        # 初始化运动消息发布者
        self.explorer_pub = rospy.Publisher('/explorer_moveit', explorer_msgs.msg.explorer_arm,queue_size=1)
        # 初始化关节位置接收者
        self.explorer_joint_sub = rospy.Subscriber('joint_states',sensor_msgs.msg.JointState,callback=self.joint_state_callback)
        self.explorer_joy_listen =rospy.Subscriber('explorer_arm',geometry_msgs.msg.TwistStamped,callback=self.joy_callback)
        self.last_joy_time = rospy.Time.now()

    
    def joint_state_callback(self,msg):
        br = tf.TransformBroadcaster()
        # 从信息处更新数据
        for i in range(0, len(self.pose)):
            for j in range(0,len(msg.name)):
                if self.arm.links[i].name == msg.name[j]:
                    self.pose[i] = msg.position[j]
                    break
        # 进行正向运动学解析,运算当前机械臂末端的坐标和各轴旋转角度
        # 旋转角度公式来源于http://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-roational-matrix
        # 可以参考euler.pdf(就是高票答案里提到的论文)
        mat = self.arm.forward_kinematics(self.pose,full_kinematics=True)
        postion = mat[-1]
        self.matrix = gu.homogeneous_to_cartesian(postion)
        self.x = postion[0,-1]
        self.y = postion[1,-1]
        self.z = postion[2,-1]
        self.radx=math.atan2(postion[2,1],postion[2,2])
        self.rady=math.atan2(-postion[2,0],math.sqrt(postion[2,1] ** 2 + postion[2,2] ** 2))
        self.radz=math.atan2(postion[1,0],postion[0,0])
        # 发送一个用于标定二维码和其他东西的tf坐标
        matrixs = gu.homogeneous_to_cartesian(mat[7])
        matrixs = np.dot(matrixs,gu.Rz_matrix(-pi/2))
        matrixs = np.dot(matrixs,gu.Rx_matrix(-pi/2))
        
        dx=math.atan2(matrixs[2,1],matrixs[2,2])
        dy=math.atan2(-matrixs[2,0],math.sqrt(matrixs[2,1] ** 2 + matrixs[2,2] ** 2))
        dz=math.atan2(matrixs[1,0],matrixs[0,0])
        br.sendTransform((  self.x, self.y, self.z),
                            tf.transformations.quaternion_from_euler(dx,dy,dz),
                            rospy.Time.now(),
                            "camera_arm","base_link")

    def joy_callback(self,msg):
        if msg.header.stamp > self.last_joy_time:
            print("23333")
            cnt = 1
            trans = [[msg.twist.linear.y],[-msg.twist.linear.x],[-msg.twist.linear.z]]
            #trans = np.dot(np.dot(self.matrix,gu.Rx_matrix(pi/4)),trans)# 进行坐标变换
            trans = np.dot(gu.Rz_matrix(pi/2),trans)
            #print trans
            self.x += trans[0,0]*cnt
            self.y += trans[1,0]*cnt
            self.z += trans[2,0]*cnt
            #matrix = np.dot(gu.Rz_matrix(msg.twist.angular.z),matrix)
            #matrix = np.dot(gu.Ry_matrix(msg.twist.angular.y),matrix)
            #matrix = np.dot(gu.Rx_matrix(msg.twist.angular.x),matrix)
            frame_target = gu.to_transformation_matrix([self.x,self.y,self.z], np.dot(gu.rpy_matrix(msg.twist.angular.x * cnt,msg.twist.angular.y * cnt,msg.twist.angular.z * cnt),self.matrix))
            self.ik = self.arm.inverse_kinematics(frame_target,initial_position=self.pose)
            self.matest = self.arm.forward_kinematics(self.ik)
            print self.matest
            self.radx=math.atan2(self.matest[2,1],self.matest[2,2])
            self.rady=math.atan2(-self.matest[2,0],math.sqrt(self.matest[2,1] ** 2 + self.matest[2,2] ** 2))
            self.radz=math.atan2(self.matest[1,0],self.matest[0,0])
            #self.ik[8]+=msg.twist.angular.z * cnt
            #self.ik[7]-=msg.twist.angular.x * cnt
            #self.ik[6]-=msg.twist.angular.y * cnt
            #print 'ik[8]=' + str(ik[8])
            #self.ik[8]-=math.atan2(self.matest[2,1],self.matest[2,2])
            #self.ik[8]-=self.radx
            #print 'ik[7]=' + str(ik[7])
            #self.ik[7]-=math.atan2(-self.matest[2,0],math.sqrt(self.matest[2,1] ** 2 + self.matest[2,2] ** 2))
            #self.ik[7]-=self.radz
            #self.ik[7] = -self.ik[7] # 他理解的i[7]方向不一样...# 现在一样了 urdf 有个-1....
            #print 'ik[6]' + str(ik[6])
            #self.ik[6]-=math.atan2(self.matest[1,0],self.matest[0,0])
            #self.ik[6]-=self.rady
            msgs = explorer_msgs.msg.explorer_arm()
            i = 0
            while i < len(self.ik):
                if self.ik[i] > 0:
                    k = int(self.ik[i] / pi)
                    self.ik[i] -= ((k + 1) / 2) * pi * 2
                else:
                    k = int(self.ik[i] / pi)
                    self.ik[i] -= ((k + 1) / 2 ) * pi * 2
                msgs.names.append(self.arm.links[i].name)
                msgs.poses.append(self.ik[i])
                i+=1
            self.explorer_pub.publish(msgs)
            self.last_joy_time = rospy.Time.now()
            print "=========================="
if __name__ == '__main__':
    node = explorer_ik_switcher(cur_file_dir() +"/rescue_arm_only.URDF",
    base_elements=[ "base_link", "joint_front_back",
                    "front_back_base", "robot_left_right",
                    "frame_Link","arm1_bearing_joint"], 
    active_links_mask=[ False ,False ,False ,True ,True ,True ,True ,True ,True ])
    rospy.spin()
