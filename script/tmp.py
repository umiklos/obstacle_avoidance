#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import rospy
from shapely.geometry import Point
import visualization_msgs.msg as vismsg
from geometry_msgs.msg import PoseStamped,TwistStamped
import math

params=rospy.get_param(rospy.get_param("car_name"))
car_width = params['car_width']
rear_axle_car_front_distance = params['rear_axle_car_front_distance']
car_length = params['car_length']

kiteres_iranya = rospy.get_param('/obstacle_avoidance_params/kiteres_iranya')
kiteres_hossza = rospy.get_param('/obstacle_avoidance_params/kiteres_hossza')
oldaliranyu_eltolas = rospy.get_param('/obstacle_avoidance_params/oldaliranyu_eltolas')
elkerules_hossza = rospy.get_param('/obstacle_avoidance_params/elkerules_hossza')
visszateres_hossza = rospy.get_param('/obstacle_avoidance_params/visszateres_hossza')
distance_delta = rospy.get_param('/obstacle_avoidance_params/distance_delta')
lookahead = rospy.get_param('/obstacle_avoidance_params/lookahead')
polygon_size_threshold = rospy.get_param('/obstacle_avoidance_params/polygon_size_threshold')
presence_threshold = rospy.get_param('/obstacle_avoidance_params/presence_threshold')
delete_threshold = rospy.get_param('/obstacle_avoidance_params/delete_threshold')

current_pose = None

def closest_point(data,x,y):
    min_distance, min_index = 10000000, 0
    for i,w in enumerate(data): 
        dx2 = w[0] - x                                  
        dy2 = w[1] - y                                      
        distance = math.sqrt(dx2**2+dy2**2)
        if distance < min_distance:
            min_distance, min_index = distance,i
    return min_index

def line_orientation(x1, x2, y1, y2):
    # https://en.wikipedia.org/wiki/Atan2
    return np.arctan2((y2-y1), (x2-x1)) 

def rotate( origin,point_x,point_y, radians):
    x,y = point_x,point_y
    offset_x, offset_y = origin
    adjusted_x = (x - offset_x)
    adjusted_y = (y - offset_y)
    cos_rad = np.cos(radians)
    sin_rad = np.sin(radians)
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
    return Point(qx, qy)

waypoint_list = []
with open(rospy.get_param("waypoint_file_name")) as f:
    header = f.readline()
    for i,line in enumerate(f):
        values = line.strip().split(',')
        x = float(values[0])
        y = float(values[1])
        z = float(values[2])
        yaw = float(values[3])
        lin_vel = float(values[4])
        waypoint_list.append([x,y,yaw,lin_vel])
elkerules=np.array(waypoint_list)

angles=np.zeros((len(elkerules),))
#car = np.zeros((len(elkerules),))
car=np.zeros((len(elkerules),5,2))

for ij in range(len(elkerules)-1):
    x1 = elkerules[ij][0]
    x2 = elkerules[ij+1][0]
    y1 = elkerules[ij][1]
    y2 = elkerules[ij+1][1]

    angles[ij] = line_orientation(x1, x2, y1, y2) 
angles[-1]= elkerules[-1][2]

#a=rotate((0,0),3,4,5)
#print(a.x)

for i in range(len(elkerules)):
    car[i,0,0:2] = rotate((elkerules[i][0],elkerules[i][1]),elkerules[i][0] + rear_axle_car_front_distance,elkerules[i][1] + (car_width/2),-angles[i])
    car[i,1,0:2] = rotate((elkerules[i][0],elkerules[i][1]),elkerules[i][0] + rear_axle_car_front_distance,elkerules[i][1] - (car_width/2),-angles[i])
    car[i,2,0:2] = rotate((elkerules[i][0],elkerules[i][1]),elkerules[i][0] - (car_length-rear_axle_car_front_distance) , elkerules[i][1] - (car_width/2),-angles[i])
    car[i,3,0:2] = rotate((elkerules[i][0],elkerules[i][1]),elkerules[i][0] - (car_length-rear_axle_car_front_distance) , elkerules[i][1] + (car_width/2),-angles[i])
    car[i,4,0:2] = car[i,0,0:2]

    # p1 = rotate((elkerules[i][0],elkerules[i][1]),elkerules[i][0] + rear_axle_car_front_distance,elkerules[i][1] + (car_width/2),-angles[i])
    # p2 = rotate((elkerules[i][0],elkerules[i][1]),elkerules[i][0] + rear_axle_car_front_distance,elkerules[i][1] - (car_width/2),-angles[i])
    # p3 = rotate((elkerules[i][0],elkerules[i][1]),elkerules[i][0] - (car_length-rear_axle_car_front_distance) , elkerules[i][1] - (car_width/2),-angles[i])
    # p4 = rotate((elkerules[i][0],elkerules[i][1]),elkerules[i][0] - (car_length-rear_axle_car_front_distance) , elkerules[i][1] + (car_width/2),-angles[i])

    #print('c')

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])


def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def callback_current_pose(pose):
    global current_pose
    current_pose = pose

def callback_detectedobjects(data):
    waypoints_size = len(waypoint_list)

    polygon_list=[]
    for i in range (len(data.markers)):
        #print(i)
        polygon_data=[]
        for j in range(len(data.markers[i].points)):
            polygon_data.append([data.markers[i].points[j].x,data.markers[i].points[j].y])    
        polygon_list.append(polygon_data)

    polygon_list=np.array(polygon_list)
    
    # print(polygon_list.shape)

    if current_pose is not None:

        
        closest_waypoint = closest_point(waypoint_list,current_pose.pose.position.x,current_pose.pose.position.y) 
        
        la = lookahead
        if waypoints_size - closest_waypoint < la:
            la = waypoints_size - closest_waypoint

        a=[]
        for k in range(closest_waypoint,waypoints_size):
            b=[]
            for i in range(len(polygon_list)):
                c=[]
                for j in range(len(polygon_list[i])-1):
                    d=[]
                    for l in range(4):
                        
                        d.append(intersect(car[k,l],car[k,l+1],polygon_list[i][j],polygon_list[i][j+1]))
                        
                    c.append(d)
                    print(c)
                b.append(c)
            a.append(b)
        

        #         polygon_data=[]
        #         for j in range(len(data.markers[i].points)):
        #             polygon_data.append
        #                 a.append(intersect(car[k,l],car[k,l+1],j,j+1))
                



#p11=-0.5,0
# p12=2,2
# q11=-1,0
# q12=1,2

# p1=np.array((p11,p12))
# q1=np.array((q11,q12))

# #print(p1[0][0])


# a=intersect(p1[0],p1[1],q1[0],q1[1])
# print(a)

# plt.plot(p1[:,0],p1[:,1],c='r')
# plt.plot(q1[:,0],q1[:,1],c='b')
# plt.axis('equal')
# plt.show()

def pub():
    global waypoint_list
    
    rospy.init_node('points')
    rospy.Subscriber("/converted_euclidean_objects", vismsg.MarkerArray, callback_detectedobjects)
    rospy.Subscriber("/current_pose", PoseStamped,callback_current_pose)
    rospy.spin()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass