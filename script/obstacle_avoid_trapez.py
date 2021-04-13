#!/usr/bin/env python


import rospy

import numpy as np
import matplotlib.pyplot as plt



import geometry_msgs.msg as geomsg
import visualization_msgs.msg as vismsg
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped,TwistStamped
from autoware_msgs.msg import Lane
import math
from shapely.geometry import Polygon, LineString, Point
from shapely import affinity
from scipy.stats import linregress
from autoware_msgs.msg import Lane, Waypoint

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
#path_replanned=False





#### params ###

params=rospy.get_param(rospy.get_param("car_name"))
car_width = params['car_width']
rear_axle_car_front_distance = params['rear_axle_car_front_distance']
car_length = params['car_length']

kiteres_iranya = "balra"
kiteres_hossza = 8.0
oldaliranyu_eltolas = 3.0
elkerules_hossza = 2.0
visszateres_hossza = 8.0
distance_delta = 0.8





def callback_closest_waypoints(data):
    global closest_waypoint
    closest_waypoint = data
    

def callback_current_pose(pose):
    global current_pose
    current_pose = pose

#def callback_base_waypoints(lane):
def callback_detectedobjects(data):
    global waypoints_size,car_width,rear_axle_car_front_distance,car_length,closest_waypoint,polygons,collect_intersect_id,elkerules,polygons,centroids,midle_index_list,first
    
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

    
    #print(len(id),collect_intersect_id,elkerules is None)

    #print(len(collect_intersect_id),len(id))
    

    # if len(collect_intersect_id) == 0:

    #     elkerules=np.zeros((len(waypoint_list),4))  
    #     for i in range(len(waypoint_list)):
    #         elkerules[i,0]= waypoint_list[i][0]
    #         elkerules[i,1]= waypoint_list[i][1]
    #         elkerules[i,2]= waypoint_list[i][3]
    #         elkerules[i,3]= waypoint_list[i][4] 

    #length_collisions=get_collect_intersect_id_len()

    #print(length_collisions)

    # if first==True and id != None:
    #     first=False

    #     print('aaa')
    # else:
    #     print('fff')


    if len(collect_intersect_id) > 0 :
        
        # for i in collect_intersect_id:
        #     intersect_points.append([waypoint_list[i][0],waypoint_list[i][1]])
        # intersect_points=np.array(intersect_points)

        szakasz_yaw = np.arctan2((waypoint_list[collect_intersect_id[-1]][1] - waypoint_list[collect_intersect_id[0]][1]), (waypoint_list[collect_intersect_id[-1]][0]- waypoint_list[collect_intersect_id[0]][0]))
        #print(szakasz_yaw)
        
        # if szakasz_yaw == -np.inf or szakasz_yaw == np.inf:
        #     szakasz_yaw = 0.0

        elkerules_start= waypoint_list[midle_index_list[0]][0] + (elkerules_hossza/2) * np.cos(szakasz_yaw + np.pi), waypoint_list[midle_index_list[0]][1] + (elkerules_hossza/2) * np.sin(szakasz_yaw + np.pi)
        elkerules_vege= waypoint_list[midle_index_list[0]][0] + (elkerules_hossza/2) * np.cos(szakasz_yaw), waypoint_list[midle_index_list[0]][1] + (elkerules_hossza/2) * np.sin(szakasz_yaw)
        
        elkerules_kezdeti_index = closest_point(waypoint_list,elkerules_start[0],elkerules_start[1])
        elkerules_utolso_index = closest_point(waypoint_list,elkerules_vege[0],elkerules_vege[1])

        start_index = closest_point(waypoint_list,waypoint_list[elkerules_kezdeti_index][0] + kiteres_hossza * np.cos(waypoint_list[elkerules_kezdeti_index][3]+ np.pi),waypoint_list[elkerules_kezdeti_index][1] + kiteres_hossza * np.sin(waypoint_list[elkerules_kezdeti_index][3]+ np.pi))
        end_index = closest_point(waypoint_list,waypoint_list[elkerules_utolso_index][0] + visszateres_hossza * np.cos(waypoint_list[elkerules_utolso_index][3]),waypoint_list[elkerules_utolso_index][1] + visszateres_hossza * np.sin(waypoint_list[elkerules_utolso_index][3]))
        start_point = waypoint_list[start_index][0:2]
        end_point = waypoint_list[end_index][0:2]

        

        elkerules_points=[]
        vx=[]
        original_distances=[]
        distances_between_points=0

        for i in range(len(waypoint_list)-1):
            x1 = waypoint_list[i][0]
            x2 = waypoint_list[i+1][0]
            y1 = waypoint_list[i][1]
            y2 = waypoint_list[i+1][1]

            distances_between_points += line_length(x1,x2,y1,y2)
            original_distances.append(distances_between_points)
            
            vx.append(waypoint_list[i][4])
            if i > start_index and i < end_index:
                if kiteres_iranya == 'balra':
                    elkerules_points.append((waypoint_list[i][0] + oldaliranyu_eltolas * np.cos(waypoint_list[i][3]+np.pi/2),waypoint_list[i][1] + oldaliranyu_eltolas * np.sin(waypoint_list[i][3]+np.pi/2)))
                elif kiteres_iranya == 'jobbra':
                    elkerules_points.append((waypoint_list[i][0] + oldaliranyu_eltolas * np.cos(waypoint_list[i][3] - np.pi/2),waypoint_list[i][1] + oldaliranyu_eltolas * np.sin(waypoint_list[i][3]- np.pi/2)))
            else:
                elkerules_points.append((waypoint_list[i][0],waypoint_list[j][i]))

        # 

        # actual_len_of_avoid = 0
        # distances_between_points=0
        # for k in range(len(waypoint_list)-1):
        #     x1 = waypoint_list[k][0]
        #     x2 = waypoint_list[k+1][0]
        #     y1 = waypoint_list[k][1]
        #     y2 = waypoint_list[k+1][1]
           
        #     angle = line_orientation(x1, x2, y1, y2)              # ori_wayp[i] is jo lehet, 
        #     #print(angle)

        #     vx.append(waypoint_list[k][4])

        #     #print(len(vx))
            
        #     if k > start_index:
        #         actual_len_of_avoid += line_length(x1, x2, y1, y2)
        #         if actual_len_of_avoid < kiteres_hossza:
        #             distance = oldaliranyu_eltolas * (actual_len_of_avoid / kiteres_hossza)
        #             #vx.append(waypoint_list[k][4] * (actual_len_of_avoid / kiteres_hossza))
        #         elif kiteres_hossza < actual_len_of_avoid < kiteres_hossza + elkerules_hossza:
        #             distance = oldaliranyu_eltolas
        #             #vx.append(waypoint_list[k][4])
        #         elif kiteres_hossza + elkerules_hossza < actual_len_of_avoid < kiteres_hossza + elkerules_hossza + visszateres_hossza:
        #             distance = oldaliranyu_eltolas * -1 * ((actual_len_of_avoid - elkerules_hossza - kiteres_hossza - visszateres_hossza)/ visszateres_hossza)
        #             #vx.append(waypoint_list[k][4] * -1 * ((actual_len_of_avoid - elkerules_hossza - kiteres_hossza - visszateres_hossza)/ visszateres_hossza) )
        #         else:
        #             distance = 0
        #             #vx.append(waypoint_list[k][4])

        #     else:
        #         distance = 0
        #         #vx.append(waypoint_list[k][4])

        #     #print(actual_len_of_avoid, distance)
        #     if kiteres_iranya == "balra":
        #         elkerules_points.append((x1 + distance * np.cos(angle + np.pi / 2),y1 + distance * np.sin(angle + np.pi / 2)))
        #     else:
        #         elkerules_points.append((x1 + distance * np.cos(angle - np.pi / 2),y1 + distance * np.sin(angle - np.pi / 2)))
        #     # x_left = x_middle + distance * np.cos(angle + np.pi / 2) 
        #     # y_left = y_middle + distance * np.sin(angle + np.pi / 2)


        #print(original_distances)

        
        # elkerules_points.append(start_point)


        # for i in range(elkerules_kezdeti_index,elkerules_utolso_index+1):
        #     if kiteres_iranya == 'balra':
        #         elkerules_points.append((waypoint_list[i][0] + oldaliranyu_eltolas * np.cos(waypoint_list[i][3]+np.pi/2),waypoint_list[i][1] + oldaliranyu_eltolas * np.sin(waypoint_list[i][3]+np.pi/2)))
        #     elif kiteres_iranya == 'jobbra':
        #         elkerules_points.append((waypoint_list[i][0] + oldaliranyu_eltolas * np.cos(waypoint_list[i][3] - np.pi/2),waypoint_list[i][1] + oldaliranyu_eltolas * np.sin(waypoint_list[i][3]- np.pi/2)))

        

        # elkerules_points.append(end_point)

        
        

       

        # #elkerules_ls = LineString([(start_point),elkerules_points,(end_point)]) 
        velocity_ls = LineString(np.column_stack((original_distances,vx))) 
        elkerules_ls = LineString(elkerules_points) 
        # #print(elkerules_ls.coords.xy)

        # # # #print(elkerules_ls.coords.xy)

        n=round(elkerules_ls.length/distance_delta)

        distances = np.linspace(0,elkerules_ls.length,n)

        distances_for_velocity = np.linspace(0,velocity_ls.length,n)

        new_velocities = [velocity_ls.interpolate(distance_v) for distance_v in distances_for_velocity]



        points = [elkerules_ls.interpolate(distance_ls) for distance_ls in distances]

        new_line = LineString(points)

        
        nw = LineString(new_velocities)

        new_velocities_data = np.zeros((len(nw.coords),1))  


        new_velocities_data[:,0] = nw.coords.xy[1] 

        # #print(len(points),len(new_line.coords),end_index-start_index)


        #print(elkerules_ls.length,velocity_ls.length)

        # plt.plot(velocity_ls.coords.xy[0],velocity_ls.coords.xy[1])
        # plt.plot(nw.coords.xy[0],nw.coords.xy[1])
       
        # plt.axis('equal')
        # plt.show()
        
        elkerules_data=np.zeros((len(new_line.coords),2))

        elkerules_data[:,0]=new_line.coords.xy[0]
        elkerules_data[:,1]=new_line.coords.xy[1]
        
        for i in range(1,len(new_line.coords)):
        
            if (new_line.coords.xy[1][i]-new_line.coords.xy[1][i-1])/(new_line.coords.xy[0][i]-new_line.coords.xy[0][i-1]) == -np.inf:
                yaw.append(0.0)
            elif (new_line.coords.xy[1][i]-new_line.coords.xy[1][i-1])/(new_line.coords.xy[0][i]-new_line.coords.xy[0][i-1]) == np.inf:
                yaw.append(0.0)
            else:
                #yaw.append((new_line.coords.xy[1][i]-new_line.coords.xy[1][i-1])/(new_line.coords.xy[0][i]-new_line.coords.xy[0][i-1]))
                yaw.append(np.arctan2((new_line.coords.xy[1][i]-new_line.coords.xy[1][i-1]),new_line.coords.xy[0][i]-new_line.coords.xy[0][i-1]))
        
    
        if len(yaw) < len(new_line.coords):
            yaw.append(waypoint_list[end_index][3])

        # #print elkerules
        yaw=np.array(yaw)

        elkerules_=np.column_stack((elkerules_data,yaw))


        elkerules = np.column_stack((elkerules_,new_velocities_data)) 

    else:
        elkerules=np.zeros((len(waypoint_list),4))  
        for i in range(len(waypoint_list)):
            elkerules[i,0]= waypoint_list[i][0]
            elkerules[i,1]= waypoint_list[i][1]
            elkerules[i,2]= waypoint_list[i][3]
            elkerules[i,3]= waypoint_list[i][4] 


            



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

def line_orientation(x1, x2, y1, y2):
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
    pub_new_data = rospy.Publisher("/global_waypoints/visualization",vismsg.MarkerArray, queue_size=1 ) 
    pub_based_waypoint_list = rospy.Publisher("/base_waypoints",Lane,queue_size=1)
    try:
        waypoint_list=load_csv(rospy.get_param("waypoint_file_name"))
    except:
        print('no waypoint')


    msg_pub_lane = Lane()
    w0 = Waypoint()
    pose = PoseStamped()
    w0.twist = TwistStamped()
    

    # if len(elkerules) > 0: #is not None:
    #     path_replanned=True
    # else:
    #     path_replanned=False

    rate=rospy.Rate(10)

    ma = vismsg.MarkerArray()

    while not rospy.is_shutdown():

    

        #if elkerules is not None or elkerules is not []:
        #if path_replanned==True:
        if len(collect_intersect_id) > 0:
            
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
                #Velocity
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
                marker_lin_vel.text = str(round(elkerules[i][3],1))      #### nem jo ####
                ma.markers.append(marker_lin_vel)
                #Sphere
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

                pose.pose.position.x = elkerules[i][0]
                pose.pose.position.y = elkerules[i][1]
                pose.pose.position.z = -1.36
                pose.pose.orientation.z = math.sin((elkerules[i][2])/2.0)
                pose.pose.orientation.w = math.cos((elkerules[i][2])/2.0)
                w0.pose = pose
                w0.twist.twist.linear.x = elkerules[i][3]
                msg_pub_lane.waypoints.append(w0)
            pub_based_waypoint_list.publish(msg_pub_lane)
            

            pub_new_data.publish(ma)
        else:
            for j in range(len(waypoint_list)):
                pose.pose.position.x = waypoint_list[j][0]
                pose.pose.position.y = waypoint_list[j][1]
                pose.pose.position.z = -1.36
                pose.pose.orientation.z = math.sin(waypoint_list[j][3]/2.0)
                pose.pose.orientation.w = math.cos(waypoint_list[j][3]/2.0)
                w0.pose = pose
                w0.twist.twist.linear.x = waypoint_list[j][4]
                msg_pub_lane.waypoints.append(w0)

                ori_arrow = vismsg.Marker()
                ori_arrow.type = ori_arrow.ARROW
                ori_arrow.pose.position.x = waypoint_list[j][0]
                ori_arrow.pose.position.y = waypoint_list[j][1]
                ori_arrow.pose.position.z = -1.36
                ori_arrow.pose.orientation.z = math.sin(waypoint_list[j][3]/2.0)
                ori_arrow.pose.orientation.w = math.cos(waypoint_list[j][3]/2.0)
                ori_arrow.action = ori_arrow.ADD
                ori_arrow.color.r = 0.2
                ori_arrow.color.g = 1.0
                ori_arrow.color.b = 0.6
                ori_arrow.color.a = 1.0
                ori_arrow.scale.x = 1.0
                ori_arrow.scale.y = 0.1
                ori_arrow.id = j
                ori_arrow.ns = "orientation"
                ori_arrow.header.frame_id = "map"
                ma.markers.append(ori_arrow)
                #Velocity
                marker_lin_vel = vismsg.Marker()
                marker_lin_vel.type = marker_lin_vel.TEXT_VIEW_FACING
                marker_lin_vel.pose.position.x = waypoint_list[j][0]
                marker_lin_vel.pose.position.y = waypoint_list[j][1]
                marker_lin_vel.pose.position.z = 0.7
                marker_lin_vel.ns = "linvel"
                marker_lin_vel.header.frame_id = "map"
                marker_lin_vel.color.r = 1.0
                marker_lin_vel.color.g = 1.0
                marker_lin_vel.color.a = 1.0
                marker_lin_vel.scale.z = 0.5
                marker_lin_vel.id = j
                marker_lin_vel.text = str(round(waypoint_list[j][4],1))      #### nem jo ####
                ma.markers.append(marker_lin_vel)
                #Sphere
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
                marker_lane_points.pose.position.x = waypoint_list[j][0]
                marker_lane_points.pose.position.y = waypoint_list[j][1]
                marker_lane_points.pose.position.z = -1.36
                marker_lane_points.pose.orientation.w = 1.0
                marker_lane_points.id = j 
                ma.markers.append(marker_lane_points)


            pub_new_data.publish(ma)
            pub_based_waypoint_list.publish(msg_pub_lane)
    
    
        rate.sleep()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass