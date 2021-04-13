#!/usr/bin/env python


import rospy

import numpy as np
import matplotlib.pyplot as plt

import autoware_msgs.msg as autoware

import geometry_msgs.msg as geomsg
import visualization_msgs.msg as vismsg
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import Lane
import math
from shapely.geometry import Polygon, LineString, Point
from shapely import affinity
from scipy.stats import linregress

closest_waypoint = None 
list= None
current_pose = None
waypoints_size = None
waypoint_list= []
polygons=[]
elkerules=[]
centroids=[]
midle_index_list=[]
collect_intersect_id=[]



#### params ###

params=rospy.get_param(rospy.get_param("car_name"))
car_width = params['car_width']
rear_axle_car_front_distance = params['rear_axle_car_front_distance']
car_length = params['car_length']

kiteres_iranya = "balra"
kiteres_hossza = 5.0
oldaliranyu_eltolas = 2.5
elkerules_hossza = 2.0
visszateres_hossza = 4.0
distance_delta = 1.2




def callback_closest_waypoints(data):
    global closest_waypoint
    closest_waypoint = data

def callback_current_pose(pose):
    global current_pose
    current_pose = pose

#def callback_base_waypoints(lane):
def callback_detectedobjects(data):
    global waypoints_size,car_width,rear_axle_car_front_distance,car_length,closest_waypoint,polygons,collect_intersect_id,elkerules,polygons,centroids,midle_index_list
    
    waypoints_size = len(waypoint_list)
    yaw=[]

    intersect_points=[]
    polygon_list=[]
    polygons=[]
    centroids = []
    merged_list=[]


    for i in range (len(data.markers)):
        polygon_data=[]
        

        centroids.append(Point(data.markers[i].pose.position.x,data.markers[i].pose.position.y))
        
        


        for j in range(len(data.markers[i].points)):
            polygon_data.append([data.markers[i].points[j].x,data.markers[i].points[j].y])    
        polygon_list.append(polygon_data)

    for k in range(len(polygon_list)):        
        polygons.append(Polygon(polygon_list[k]))

    polygons=np.array(polygons)

    #print(len(centroids))


    collision_examination(waypoint_list,waypoints_size,closest_waypoint)

    


    if len(collect_intersect_id) > 0 :
        # for i in collect_intersect_id:
        #     intersect_points.append([waypoint_list[i][0],waypoint_list[i][1]])
        # intersect_points=np.array(intersect_points)

        # szakasz_yaw = (waypoint_list[collect_intersect_id[-1]][1] - waypoint_list[collect_intersect_id[0]][1]) / (waypoint_list[collect_intersect_id[-1]][0]- waypoint_list[collect_intersect_id[0]][0])
        
        # if szakasz_yaw == -np.inf or szakasz_yaw == np.inf:
        #     szakasz_yaw = 0.0

        elkerules_start= waypoint_list[midle_index_list[0]][0] + (elkerules_hossza/2) * np.cos(waypoint_list[midle_index_list[0]][3]+ np.pi), waypoint_list[midle_index_list[0]][1] + (elkerules_hossza/2) * np.sin(waypoint_list[midle_index_list[0]][3]+ np.pi)
        elkerules_vege= waypoint_list[midle_index_list[0]][0] + (elkerules_hossza/2) * np.cos(waypoint_list[midle_index_list[0]][3]), waypoint_list[midle_index_list[0]][1] + (elkerules_hossza/2) * np.sin(waypoint_list[midle_index_list[0]][3])
        
        elkerules_kezdeti_index = closest_point(waypoint_list,elkerules_start[0],elkerules_start[1])
        elkerules_utolso_index = closest_point(waypoint_list,elkerules_vege[0],elkerules_vege[1])

        start_index = closest_point(waypoint_list,waypoint_list[elkerules_kezdeti_index][0] + kiteres_hossza * np.cos(waypoint_list[elkerules_kezdeti_index][3]+ np.pi),waypoint_list[elkerules_kezdeti_index][1] + kiteres_hossza * np.sin(waypoint_list[elkerules_kezdeti_index][3]+ np.pi))
        end_index = closest_point(waypoint_list,waypoint_list[elkerules_utolso_index][0] + visszateres_hossza * np.cos(waypoint_list[elkerules_utolso_index][3]),waypoint_list[elkerules_utolso_index][1] + visszateres_hossza * np.sin(waypoint_list[elkerules_utolso_index][3]))
        start_point = waypoint_list[start_index][0:2]
        end_point = waypoint_list[end_index][0:2]


        #print(start_index,end_index)

        
        elkerules_points=[]
        elkerules_points.append(start_point)


        for i in range(elkerules_kezdeti_index,elkerules_utolso_index+1):
            if kiteres_iranya == 'balra':
                elkerules_points.append((waypoint_list[i][0] + oldaliranyu_eltolas * np.cos(waypoint_list[i][3]+np.pi/2),waypoint_list[i][1] + oldaliranyu_eltolas * np.sin(waypoint_list[i][3]+np.pi/2)))
            elif kiteres_iranya == 'jobbra':
                elkerules_points.append((waypoint_list[i][0] + oldaliranyu_eltolas * np.cos(waypoint_list[i][3] - np.pi/2),waypoint_list[i][1] + oldaliranyu_eltolas * np.sin(waypoint_list[i][3]- np.pi/2)))

        

        elkerules_points.append(end_point)

        
        

        #print(start_index,end_index)
        
        # if kiteres_iranya == 'balra':
        #     transformed_line=affinity.rotate(LineString([(waypoint_list[midle_index_list[0]][0:2]),(waypoint_list[midle_index_list[0]][0]+oldaliranyu_eltolas,waypoint_list[midle_index_list[0]][1])]),szakasz_yaw + (math.pi/2),origin=(waypoint_list[midle_index_list[0]][0:2]),use_radians=True)
        # elif kiteres_iranya == 'jobbra':
        #     transformed_line=affinity.rotate(LineString([(waypoint_list[midle_index_list[0]][0:2]),(waypoint_list[midle_index_list[0]][0]+oldaliranyu_eltolas,waypoint_list[midle_index_list[0]][1])]),szakasz_yaw -(math.pi/2),origin=(waypoint_list[midle_index_list[0]][0:2]),use_radians=True)
        # transformed_point=transformed_line.coords.xy[0][1],transformed_line.coords.xy[1][1]

        # elkerules_visszateresi_ag = affinity.rotate((LineString([(transformed_point),(transformed_point[0]+elkerules_hossza/2,transformed_point[1])])),szakasz_yaw,origin=transformed_point,use_radians=True)
        # elkerules_kiteresi_ag = affinity.rotate((LineString([(transformed_point),(transformed_point[0]+elkerules_hossza/2,transformed_point[1])])),szakasz_yaw + math.pi,origin=transformed_point,use_radians=True)

        # elkerules_kezdeti_pont = elkerules_kiteresi_ag.coords.xy[0][1],elkerules_kiteresi_ag.coords.xy[1][1]
        # elkerules_utolso_pont = elkerules_visszateresi_ag.coords.xy[0][1],elkerules_visszateresi_ag.coords.xy[1][1]


        # kiteres_pont = kiteresi_ag.coords.xy[0][1],kiteresi_ag.coords.xy[1][1]
        # visszateresi_pont = visszateresi_ag.coords.xy[0][1],visszateresi_ag.coords.xy[1][1]

        # start_index = closest_point(waypoint_list,kiteres_pont[0],kiteres_pont[1])
        # end_index = closest_point(waypoint_list,visszateresi_pont[0],visszateresi_pont[1])


        #elkerules_ls = LineString([(start_point),elkerules_points,(end_point)]) 

        elkerules_ls = LineString(elkerules_points) 
        #print(elkerules_ls.coords.xy)

        # # #print(elkerules_ls.coords.xy)
        # plt.plot(elkerules_ls.coords.xy[0],elkerules_ls.coords.xy[1])
       
        # plt.axis('equal')
        # plt.show()

        distances = np.linspace(0,elkerules_ls.length,round(elkerules_ls.length/distance_delta))

        points = [elkerules_ls.interpolate(distance) for distance in distances]

        new_line = LineString(points)

        #print(len(points),len(new_line.coords),end_index-start_index)


        
        elkerules_data=np.zeros((len(new_line.coords),2))

        elkerules_data[:,0]=new_line.coords.xy[0]
        elkerules_data[:,1]=new_line.coords.xy[1]
        
        for i in range(1,len(new_line.coords)):
        
            if (new_line.coords.xy[1][i]-new_line.coords.xy[1][i-1])/(new_line.coords.xy[0][i]-new_line.coords.xy[0][i-1]) == -np.inf:
                yaw.append(0.0)
            elif (new_line.coords.xy[1][i]-new_line.coords.xy[1][i-1])/(new_line.coords.xy[0][i]-new_line.coords.xy[0][i-1]) == np.inf:
                yaw.append(0.0)
            else:
                yaw.append((new_line.coords.xy[1][i]-new_line.coords.xy[1][i-1])/(new_line.coords.xy[0][i]-new_line.coords.xy[0][i-1]))

        
    
        if len(yaw) < len(new_line.coords):
            yaw.append(waypoint_list[end_index][3])

        # #print elkerules
        yaw=np.array(yaw)

        elkerules=np.column_stack((elkerules_data,yaw))

        # #print(yaw)

    

                  

            



def collision_examination(data,waypoints_size,closest_waypoint):
    intersect_id=[]
    #midle_index_list=[]
    if closest_waypoint is not None:
        for i in range(closest_waypoint.data, 38):                    #### waypoint_size ig megy az iteracio
            
            p1=data[i][0] + rear_axle_car_front_distance,  data[i][1] + car_width/2
            p2=data[i][0] + rear_axle_car_front_distance, data[i][1] - car_width/2
            p3=data[i][0] - car_length, data[i][1] - car_width/2
            p4=data[i][0] - car_length, data[i][1] + car_width/2

            for k in range(len(centroids)):
                if Polygon([p1,p2,p3,p4]).intersects(centroids[k]) == True:
                    midle_index=closest_point(waypoint_list,centroids[k].x,centroids[k].y)
                    if len(midle_index_list)== 0:
                        midle_index_list.append(midle_index)
                    elif len(midle_index_list) != 0:
                        if midle_index not in midle_index_list:
                            midle_index_list.append(midle_index)                    

            
            
            for j in range(len(polygons)):
                # try:
                #     affinity.rotate(Polygon([p1,p2,p3,p4]),data[i][3],origin=(data[i][0],data[i][1]),use_radians=True).intersects(polygons[j]) == True
                    
                # except:
                #     print("hiba",j,len(polygons))
                
                if affinity.rotate(Polygon([p1,p2,p3,p4]),data[i][3],origin=(data[i][0],data[i][1]),use_radians=True).intersects(polygons[j]) == True:
                    intersect_id.append(i)
                    
                    print(len(intersect_id))
                    if len(collect_intersect_id) == 0:
                        collect_intersect_id.append(i)
                    elif len(collect_intersect_id) != 0:
                        if i not in collect_intersect_id:
                            collect_intersect_id.append(i)
                            collect_intersect_id.sort
            
            
        
            
                
            

# def local_directions(data):
#     dx,dy,dyaw=[],[],[]
#     for i in collect_intersect_id:
#         dx.append(data[i][0]-data[i+1][0])
#         dy.append(data[i][1]- data[i+1][1])
#         dyaw.append(data[i][3])
#         return np.average(dx),np.average(dy),np.average(dyaw)


def line_orientation(x1, x2, y1, y2):
    # https://en.wikipedia.org/wiki/Atan2
    return np.arctan2((y2-y1), (x2-x1)) 

def line_length(x1, x2, y1, y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5

def line_proportion(x1, x2, y1, y2, proportion):
    if not (0 < proportion < 1):
        print("proportion should be between 0 and 1")
    m = proportion
    n = 1 - proportion
    x_p = (float)((n * x1)+(m * x2))/(m + n)
    y_p = (float)((n * y1)+(m * y2))/(m + n)
    line = np.array([x_p, y_p])
    return line


    




def load_csv(path):
    waypoint_list = []
    with open(path) as f:
        header = f.readline()
        for i,line in enumerate(f):
            values = line.strip().split(',')
            x = float(values[0])
            y = float(values[1])
            z = float(values[2])
            yaw = float(values[3])
            lin_vel = float(values[4])
            waypoint_list.append([x,y,z,yaw,lin_vel])
    return waypoint_list


def closest_point(data,x,y):
    min_distance, min_index = 10000000, 0
    for i,w in enumerate(data): 
        dx2 = w[0] - x                                  
        dy2 = w[1] - y                                      
        distance = math.sqrt(dx2**2+dy2**2)
        if distance < min_distance:
            min_distance, min_index = distance,i
    return min_index


# def get_start_and_end_point(avg_dx,avg_dy,avg_yaw,slope,intercept):
#     if abs(avg_dx) > abs(avg_dy):
#         if avg_dx < 0 :      ##pozitiv irany x tengelyen globalissan
            
#             start_index = closest_point(waypoint_list,waypoint_list[midle_index_list[0]][0] - elkerules_hossza,slope*(waypoint_list[midle_index_list[0]][0] - elkerules_hossza)+intercept)
#             end_index = closest_point(waypoint_list,waypoint_list[midle_index_list[0]][0] + elkerules_hossza,slope*(waypoint_list[midle_index_list[0]][0] + elkerules_hossza)+intercept)


#         elif avg_dx > 0 :     ### negativ irany az x tengelyen   
            
#             start_index = closest_point(waypoint_list,waypoint_list[midle_index_list[0]][0] + elkerules_hossza,slope*(waypoint_list[midle_index_list[0]][0] + elkerules_hossza)+intercept)
#             end_index = closest_point(waypoint_list,waypoint_list[midle_index_list[0]][0] - elkerules_hossza,slope*(waypoint_list[midle_index_list[0]][0] - elkerules_hossza)+intercept)

#     elif abs(avg_dx) < abs(avg_dy):
#         if avg_yaw > 0 :
            
#             start_index = closest_point(waypoint_list,(waypoint_list[midle_index_list[0]][1] - elkerules_hossza -intercept)/slope ,waypoint_list[midle_index_list[0]][1] - elkerules_hossza)
#             end_index =  closest_point(waypoint_list,(waypoint_list[midle_index_list[0]][1] + elkerules_hossza -intercept)/slope ,waypoint_list[midle_index_list[0]][1] + elkerules_hossza)

#         elif avg_yaw < 0 :
            
#             start_index = closest_point(waypoint_list,(waypoint_list[midle_index_list[0]][1] + elkerules_hossza -intercept)/slope ,waypoint_list[midle_index_list[0]][1] + elkerules_hossza)
#             end_index =  closest_point(waypoint_list,(waypoint_list[midle_index_list[0]][1] - elkerules_hossza -intercept)/slope ,waypoint_list[midle_index_list[0]][1] - elkerules_hossza)
#     return start_index,end_index 


# def lateral_offset(data_new,merged_list,avg_dx,avg_dy):
    
#     for k in merged_list:
        
#         if abs(avg_dx) > abs(avg_dy):
#             data_new[k-merged_list[0],0] = waypoint_list[k][0]
#             data_new[k-merged_list[0],1] = waypoint_list[k][1] + oldaliranyu_eltolas
#             if kiteres_iranya =="jobbra":
#                 data_new[k-merged_list[0],1] = waypoint_list[k][1] - oldaliranyu_eltolas
#         elif abs(avg_dx) < abs(avg_dy):
#             data_new[k-merged_list[0],1] = waypoint_list[k][1]
#             data_new[k-merged_list[0],0] = waypoint_list[k][0] - oldaliranyu_eltolas
#             if kiteres_iranya =="jobbra":
#                 data_new[k-merged_list[0],0] = waypoint_list[k][0] + oldaliranyu_eltolas
#     return data_new

            

       
    

def pub():
    global waypoint_list
    rospy.init_node('transform_polygon', anonymous=True)
    
    rospy.Subscriber("/converted_euclidean_objects", vismsg.MarkerArray, callback_detectedobjects)
    rospy.Subscriber("/closest_waypoint", Int32, callback_closest_waypoints)
    rospy.Subscriber("/current_pose", PoseStamped,callback_current_pose)
    #rospy.Subscriber("/base_waypoints", Lane,callback_base_waypoints, queue_size=1)
    pub_new_data = rospy.Publisher("/new_waypoints",vismsg.MarkerArray, queue_size=1 )
    try:
        waypoint_list=load_csv(rospy.get_param("waypoint_file_name"))
    except:
        print('no waypoint')

    rate=rospy.Rate(10)

    ma = vismsg.MarkerArray()

    while not rospy.is_shutdown():
        if elkerules is not None :

            for i in range(len(elkerules)):
                ori_arrow = vismsg.Marker()
                ori_arrow.type = ori_arrow.ARROW
                ori_arrow.pose.position.x = elkerules[i][0]
                ori_arrow.pose.position.y = elkerules[i][1]
                ori_arrow.pose.position.z = -1.36
                ori_arrow.pose.orientation.z = math.sin(elkerules[i][2]/2.0)
                ori_arrow.pose.orientation.w = math.cos(elkerules[i][2]/2.0)
                ori_arrow.action = ori_arrow.ADD
                ori_arrow.color.r = 0.2
                ori_arrow.color.g = 1.0
                ori_arrow.color.b = 0.6
                ori_arrow.color.a = 1.0
                ori_arrow.scale.x = 1.0
                ori_arrow.scale.y = 0.1
                ori_arrow.id = i
                ori_arrow.ns = "orientation"
                ori_arrow.header.frame_id = "map"
                ma.markers.append(ori_arrow)
                # Velocity
                marker_lin_vel = vismsg.Marker()
                marker_lin_vel.type = marker_lin_vel.TEXT_VIEW_FACING
                marker_lin_vel.pose.position.x = elkerules[i][0]
                marker_lin_vel.pose.position.y = elkerules[i][1]
                marker_lin_vel.pose.position.z = 0.7
                marker_lin_vel.ns = "linvel"
                marker_lin_vel.header.frame_id = "map"
                marker_lin_vel.color.r = 1.0
                marker_lin_vel.color.g = 1.0
                marker_lin_vel.color.a = 1.0
                marker_lin_vel.scale.z = 0.5
                marker_lin_vel.id = i
                marker_lin_vel.text = str(round(waypoint_list[i][3],1))      #### nem jo ####
                ma.markers.append(marker_lin_vel)
                # Sphere
                marker_lane_points = vismsg.Marker()
                marker_lane_points.header.frame_id = "map"
                marker_lane_points.ns = "lane_points"
                marker_lane_points.type = marker_lane_points.SPHERE
                #sphere_color = self.gradient(lin_vel, 10) # max speed color (red) is 10
                marker_lane_points.color.r = 1.0
                marker_lane_points.color.g = 0.0
                marker_lane_points.color.b = 0.0
                marker_lane_points.color.a = 1.0
                marker_lane_points.scale.x = 0.4
                marker_lane_points.scale.y = 0.4
                marker_lane_points.scale.z = 0.4
                marker_lane_points.pose.position.x = elkerules[i][0]
                marker_lane_points.pose.position.y = elkerules[i][1]
                marker_lane_points.pose.position.z = -1.36
                marker_lane_points.pose.orientation.w = 1.0
                marker_lane_points.id = i 
                ma.markers.append(marker_lane_points)

            pub_new_data.publish(ma)
    
    #rospy.spin()
    rate.sleep()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass