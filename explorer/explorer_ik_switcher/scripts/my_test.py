#!/usr/bin/env python
# coding= utf8
import numpy as np
import sys
from ikpy import chain
from ikpy import plot_utils
import rospy
import geometry_msgs
import sensor_msgs.msg
import math
from ikpy import geometry_utils as gu
import explorer_msgs.msg

pi = math.acos(0.0)*2
arm = chain.Chain.from_urdf_file("./rescue_arm_only.URDF",
base_elements=[ "base_link", "joint_front_back",
                "front_back_base", "robot_left_right",
                "frame_Link","arm1_bearing_joint"], 
active_links_mask=[ False ,False ,False ,True ,True ,True ,True ,True ,True ,False])
print arm.links
pose = [0.0] * len(arm.links)
ik = [0.0] * len(arm.links)
matrix = gu.rpy_matrix(0.0,0.0,0.0)
x = 0
x_cu = 0
y = 0
y_cu = 0
z = 0
z_cu = 0
matrix_test = gu.rpy_matrix(0.0,0.0,0.0)
def joint_state_callback(msg):
    #print arm.links
    for i in range(0, len(pose)):
        for j in range(0,len(msg.name)):
            if arm.links[i].name == msg.name[j]:
                #print msg.name[j] + '==' + arm.links[i].name
                pose[i] = msg.position[j]
                break
    postion = arm.forward_kinematics(pose)
    global matrix
    matrix = gu.homogeneous_to_cartesian(postion)
    global x
    x = postion[0,-1]
    global y
    y = postion[1,-1]
    global z
    z = postion[2,-1]
    print 'radx=' + str(math.atan2(postion[2,1],postion[2,2]))
    print 'rady=' + str(math.atan2(-postion[2,0],math.sqrt(postion[2,1] ** 2 + postion[2,2] ** 2)))
    print 'radz=' + str(math.atan2(postion[1,0],postion[0,0]))
    #print "get joint call back"
    #for i in range(0, len(pose)):
    #    print arm.links[i].name + " == " + str (pose[i]) + " -> " + str(ik[i])
    print matrix
    print matrix_test
    print str(x) + " -> " + str(x_cu)
    print str(y) + " -> " + str(y_cu) 
    print str(z) + " -> " + str(z_cu)
    print "===================="


def joy_callback(msg):
    global x
    global y
    global z
    global matrix
    cnt = 0.005
    trans = [[msg.linear.x],[msg.linear.y],[msg.linear.z]]
    #trans = np.dot(matrix,trans)
    x += msg.linear.x*cnt#trans[0,0]*cnt
    y += msg.linear.y*cnt#trans[1,0]*cnt
    z += msg.linear.z*cnt#trans[2,0]*cnt
    #matrix = np.dot(gu.Rz_matrix(msg.angular.z),matrix)
    #matrix = np.dot(gu.Ry_matrix(msg.angular.y),matrix)
    #matrix = np.dot(gu.Rx_matrix(msg.angular.x),matrix)
    frame_target = gu.to_transformation_matrix([x,y,z], matrix)
    global ik
    ik = arm.inverse_kinematics(frame_target)

    matest = arm.forward_kinematics(ik)
    global matest
    #print 'ik[8]=' + str(ik[8])
    ik[8]-=math.atan2(matest[2,1],matest[2,2])
    print 'radx=' + str(math.atan2(matest[2,1],matest[2,2])) 
    #print 'ik[7]=' + str(ik[7])
    ik[7]-=math.atan2(-matest[2,0],math.sqrt(matest[2,1] ** 2 + matest[2,2] ** 2))
    ik[7] = -ik[7] # 他理解的i[7]方向不一样...
    print 'rady=' + str(math.atan2(-matest[2,0],math.sqrt(matest[2,1] ** 2 + matest[2,2] ** 2)))
    #print 'ik[6]' + str(ik[6])
    ik[6]-=math.atan2(matest[1,0],matest[0,0])
    print 'radz=' + str(math.atan2(matest[1,0],matest[0,0]))
    (tty,matrix_test) = gu.from_transformation_matrix(matest)
    global x_cu
    global y_cu
    global z_cu
    x_cu = tty[0]
    y_cu = tty[1]
    z_cu = tty[2]
    print str(x_cu) + ' ---> ' + str(x)
    print str(y_cu) + ' ---> ' + str(y)
    print str(z_cu) + ' ---> ' + str(z)
    msgs = explorer_msgs.msg.explorer_arm()
    i = 0
    while i < len(ik):
        if ik[i] > 0:
            k = int(ik[i] / pi)
            ik[i] -= ((k + 1) / 2) * pi * 2
        else:
            k = int(ik[i] / pi)
            ik[i] -= ((k + 1) / 2 ) * pi * 2
        msgs.names.append(arm.links[i].name)
        msgs.poses.append(ik[i])
        print arm.links[i].name + " \t pose = " + str(ik[i])
        i+=1
    explorer_pub.publish(msgs)


rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
explorer_pub = rospy.Publisher('/explorer_moveit', explorer_msgs.msg.explorer_arm,queue_size=1)

i=0
while i<len(arm.links):
    #print arm.active_links_mask[i]
    #print arm.links[i]
    i+=1
    #print "========================================"
test = arm.forward_kinematics([0] * len(arm.links),full_kinematics=True)
#print test
#print "========================================"

plot = True
#explorer_sub = rospy.Subscriber('joint_states',sensor_msgs.msg.JointState,callback=joint_state_callback)
explorer_joy_listen =rospy.Subscriber('explorer_arm_cof',geometry_msgs.msg.Twist,callback=joy_callback)
ax = plot_utils.init_3d_figure()
matrix = gu.rpy_matrix(1.5,0,0)
x=0.3
y=0
z=0.5

frame_target = gu.to_transformation_matrix([x,y,z],
#[[0,0,0],[0,0,0],[0,0,0]])
#[[ 0.58277065,0.65325703,0.48335661], [-0.05946241,0.62748519,-0.77635466], [-0.81045826,0.42369516,0.40452419]])
matrix)
ik = arm.inverse_kinematics(frame_target)
ik = [0.0] * len(arm.links)
ik[3] = 0
ik[5] = -0.9
matest = arm.forward_kinematics(ik)
(tty,matrix_test) = gu.from_transformation_matrix(matest)
x_cu = tty[0]
y_cu = tty[1]
z_cu = tty[2]
msg = explorer_msgs.msg.explorer_arm()
#print "==========================="
#print "ik========"+ str(ik)
#print "name======"+ str(arm.links[0].name)


#print msg
#print len(ik)
#print len(arm.links)


'''
We can get euler angles from rotation matrix using following formula.

Given a 3×3 rotation matrix
R = [[r11,r12,r13],
     [r21,r22,r23],
     [r31,r32,r33]]
ps:
Rz=[[cz,    sz,     0],
    [-sz,   cz,     0],
    [0,     0,      1]]

Ry=[[cy,    0,      sy],
    [0,     1,      0],
    [-sy,   0,      cy]]

Rz=[[1,     0,      0],
    [0,     cz,     sz],
    [0,     -sz,    cz]]

R = Rz * Ry * Rx(这是gu工具库中的默认顺序)
=[[czcy,    szcx-czsysx,    szsx+czsysx],
  [-szcy,   czcx+szsysx,    czsx-szsycx],
  [-sy,     -cysx,          cycx]]

R' = Rx * Ry * Rz
=[[cycz,        cysz,       sy],
  [sxsycz-cxsz, sxsysz+cxcz,sxcy],
  [-cxsycz+sxsz,-cxsysz-sxcz,cxcy]]


The 3 Euler angles are

x = atan2(-r32,r33)

y = atan2(-r31,sqrt(r32*r32+r33*r33))

z = atan2(-r21,r11)

atan2 是求(x,y)到x轴正方向的夹角

Here atan2 is the same arc tangent function, with quadrant checking, you typically find in C or Matlab.

注意 当atan2函数的y参数为0的情况会无法计算
Note: Care must be taken if the angle around the y-axis is exactly +/-90°. In that case all elements in the first column and last row, except the one in the lower corner, which is either 1 or -1, will be 0 (cos(1)=0). One solution would be to fix the rotation around the x-axis at 180° and compute the angle around the z-axis from: atan2(r_12, -r_22).

from http://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-roational-matrix
'''
print 'ik[8]=' + str(ik[8])
ik[8]-=math.atan2(matest[2,1],matest[2,2])
print 'radx=' + str(math.atan2(matest[2,1],matest[2,2])) 
print 'ik[7]=' + str(ik[7])
ik[7]-=math.atan2(-matest[2,0],math.sqrt(matest[2,1] ** 2 + matest[2,2] ** 2))
ik[7] = -ik[7]
print 'rady=' + str(math.atan2(-matest[2,0],math.sqrt(matest[2,1] ** 2 + matest[2,2] ** 2)))
print 'ik[6]' + str(ik[6])
ik[6]-=math.atan2(matest[1,0],matest[0,0])
print 'radz=' + str(math.atan2(matest[1,0],matest[0,0]))

i = 0
while i < len(ik):
    #print i
    while (ik[i] > pi) or (ik[i] < -pi):
        #print ik[i]
        if ik[i] > pi:
            ik[i] -= pi * 2
            #print ik[i]
        elif ik[i] < -pi:
            ik[i] += pi * 2
            #print ik[i]
    msg.names.append(arm.links[i].name)
    msg.poses.append(ik[i])
    i+=1

#explorer_pub.publish(msg)

print ik
#print arm.links[3].name
#ik = [0.0] * len(arm.links)
rospy.spin()
'''
aa = [0.0] * len(ik)
aa[3] = 1.57
aa = ik
arm.plot(aa, ax)
plot_utils.show_figure()

#np.testing.assert_almost_equal(arm.forward_kinematics(ik)[:3, 3], target, decimal=3)


[[  9.99406967e-01,  -6.22922663e-16,   3.44341915e-02, 5.67740233e-01],
[  6.12995417e-16,   1.00000000e+00,   2.98853332e-16, 6.86500000e-02],
[ -3.44341915e-02,  -2.77568101e-16,   9.99406967e-01, 7.06416712e-01],
[  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]]
[[  9.99406967e-01,  -6.22922663e-16,   3.44341915e-02],[  6.12995417e-16,   1.00000000e+00,   2.98853332e-16],[ -3.44341915e-02,  -2.77568101e-16,   9.99406967e-01]
'''
