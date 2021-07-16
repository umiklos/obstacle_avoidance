#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import rospy
from autoware_msgs.msg import Lane, Waypoint
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
collect_intersected_waypoints=[]
midle_index_list=[]
path_replanned=False
count=0

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
    return qx, qy

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
    global collect_intersected_waypoints,midle_index_list,path_replanned,elkerules,count
    waypoints_size = len(waypoint_list)
    centroids=np.empty((len(data.markers),2))
    polygon_list=[]
    valid_points=np.empty(0,)

    for i in range (len(data.markers)):
        centroids[i,0]=data.markers[i].pose.position.x
        centroids[i,1]=data.markers[i].pose.position.y
        polygon_data=[]
        for j in range(len(data.markers[i].points)):
            polygon_data.append([data.markers[i].points[j].x,data.markers[i].points[j].y])    
        polygon_list.append(polygon_data)

    
    #print(polygon_list[0][0][0])

    if current_pose is not None:
    #     if path_replanned==False:
        near_waypoint_polygon_indexes=[]
        detected_waypoints=[]

        closest_waypoint = closest_point(elkerules,current_pose.pose.position.x,current_pose.pose.position.y) 
        
        la = lookahead
        if waypoints_size - closest_waypoint < la:
            la = waypoints_size - closest_waypoint

        for i in range(closest_waypoint,closest_waypoint+la):
            for j in range(len(centroids)):
                #for k in range(len(polygon_list[j])):

                distance=math.sqrt(((centroids[j,0]-elkerules[i][0])**2)+(((centroids[j,1]-elkerules[i][1])**2)))

                if distance < 5:
                    near_waypoint_polygon_indexes.append(j)
                    detected_waypoints.append(i)
        near_waypoint_polygon_indexes=np.unique(near_waypoint_polygon_indexes)
        detected_waypoints=np.unique(detected_waypoints)
        
        intersected_waypoints=[]
        
        for i in detected_waypoints:
            f=[]
            for k in near_waypoint_polygon_indexes:
                midle_index=closest_point(elkerules,centroids[k][0],centroids[k][1])
                for l in range(1,len(polygon_list[k])):
                    for m in range(1,5):
                        f.append(intersect(car[i,m-1],car[i,m],polygon_list[k][l-1],polygon_list[k][l]))
            if (np.any(f))==True:
                intersected_waypoints.append(([i],[k]))

        print(intersected_waypoints)

        #         # if len(collect_intersected_waypoints)==0:
        #         #     collect_intersected_waypoints.append(i)                
        #         # elif len(collect_intersected_waypoints) != 0:
        #         #     if i not in collect_intersected_waypoints:
        #         #         collect_intersected_waypoints.append(i)
        #         #         collect_intersected_waypoints.sort
        #         if len(midle_index_list)== 0:
        #             midle_index_list.append(midle_index)
        #         elif len(midle_index_list) != 0:
        #             if midle_index not in midle_index_list:
        #                 midle_index_list.append(midle_index)
        #     inter_id=np.unique(a)
        # collect_intersected_waypoints.extend(inter_id)
            
        # if (inter_id.size)==0:
        #     count=count+1
        # elif (inter_id.size)!=0:
        #     count=0

        # if len(collect_intersected_waypoints) > 0 :
        #     counter = np.bincount(np.array(collect_intersected_waypoints)) 
        #     if count >= delete_threshold:
        #         counter=np.empty(0,)
        #     valid_points = np.asarray(np.where(counter > presence_threshold))

        #     print(valid_points)

        # if valid_points.size > 0:
        #     mask = np.isin(midle_index_list,valid_points)
        #     center= np.array(midle_index_list)
        #     center=center[mask]     
                            
        #     if center.size > 0:                    
        #         szakasz_yaw = np.arctan2((waypoint_list[valid_points[0][-1]][1] - waypoint_list[valid_points[0][0]][1]), (waypoint_list[valid_points[0][-1]][0]- waypoint_list[valid_points[0][0]][0]))
        #         start_index = closest_point(waypoint_list,waypoint_list[center[0]][0] + (kiteres_hossza  + elkerules_hossza) * np.cos(szakasz_yaw + np.pi),waypoint_list[center[0]][1] + (elkerules_hossza +kiteres_hossza) * np.sin(szakasz_yaw + np.pi))
        #         end_index = closest_point(waypoint_list,waypoint_list[center[0]][0] + (visszateres_hossza) * np.cos(szakasz_yaw),waypoint_list[center[0]][1] + (visszateres_hossza) * np.sin(szakasz_yaw))
        #         start_point = waypoint_list[start_index][0:2]
        #         end_point = waypoint_list[end_index][0:2]

        #         elkerules_points=[]
        #         original_distances=[]

        #         actual_len_of_avoid = 0
        #         distances_between_points=0
        #         for k in range(closest_waypoint,len(waypoint_list)-1):
        #             x1 = waypoint_list[k][0]
        #             x2 = waypoint_list[k+1][0]
        #             y1 = waypoint_list[k][1]
        #             y2 = waypoint_list[k+1][1]
                    
        #             distances_between_points += line_length(x1,x2,y1,y2)
        #             original_distances.append(distances_between_points)
                    
        #             if k > start_index:
        #                 actual_len_of_avoid += line_length(x1, x2, y1, y2)
        #                 if actual_len_of_avoid < kiteres_hossza:
        #                     distance = oldaliranyu_eltolas * (actual_len_of_avoid / kiteres_hossza)
        #                 elif kiteres_hossza < actual_len_of_avoid < kiteres_hossza + elkerules_hossza:
        #                     distance = oldaliranyu_eltolas
        #                 elif kiteres_hossza + elkerules_hossza < actual_len_of_avoid < kiteres_hossza + elkerules_hossza + visszateres_hossza:
        #                     distance = oldaliranyu_eltolas * -1 * ((actual_len_of_avoid - elkerules_hossza - kiteres_hossza - visszateres_hossza)/ visszateres_hossza)
        #                 else:
        #                     distance = 0
        #             else:
        #                 distance = 0
                
        #             if kiteres_iranya == "balra":
        #                 elkerules_points.append((x1 + distance * np.cos(angles[k] + np.pi / 2),y1 + distance * np.sin(angles[k] + np.pi / 2)))
        #             else:
        #                 elkerules_points.append((x1 + distance * np.cos(angles[k] - np.pi / 2),y1 + distance * np.sin(angles[k] - np.pi / 2)))
                
        #         velocity_ls = LineString(np.column_stack((original_distances,elkerules[closest_waypoint+1:len(waypoint_list),3]))) 
        #         elkerules_ls = LineString(elkerules_points) 
        #         n=round(elkerules_ls.length/distance_delta)
        #         distances = np.linspace(0,elkerules_ls.length,n)
        #         distances_for_velocity = np.linspace(0,velocity_ls.length,n)
        #         new_velocities = [velocity_ls.interpolate(distance_v) for distance_v in distances_for_velocity]
        #         points = [elkerules_ls.interpolate(distance_ls) for distance_ls in distances]
        #         new_line = LineString(points)
        #         nw = LineString(new_velocities)
        #         new_velocities_data = np.zeros((len(nw.coords),1))  
        #         new_velocities_data[:,0] = nw.coords.xy[1] 
        #         elkerules_data=np.zeros((len(new_line.coords),2))
        #         elkerules_data[:,0]=new_line.coords.xy[0]
        #         elkerules_data[:,1]=new_line.coords.xy[1]

        #         yaw = np.zeros((len(new_line.coords),))
        #         for i in range(len(elkerules_data)-1):
        #             yaw[i] = np.arctan2((elkerules_data[i+1,1]-elkerules_data[i,1]),(elkerules_data[i+1,0]- elkerules_data[i,0]))
        #         yaw[-1]= waypoint_list[-1][2]
                
        #         elkerules_= np.column_stack((elkerules_data,yaw))
        #         elkerules = np.column_stack((elkerules_,new_velocities_data)) 
        #         path_replanned=True
        #         collect_intersect_id=[]
        #     elif center.size==0:
        #         rospy.logwarn('no valid centerpoint')

            


def pub():
    global waypoint_list
    
    rospy.init_node('points')
    rospy.Subscriber("/converted_euclidean_objects", vismsg.MarkerArray, callback_detectedobjects)
    rospy.Subscriber("/current_pose", PoseStamped,callback_current_pose)
    pub_new_data = rospy.Publisher("/global_waypoints/visualization",vismsg.MarkerArray, queue_size=1 ) 
    pub_based_waypoint_list = rospy.Publisher("/base_waypoints",Lane,queue_size=1)

    msg_pub_lane = Lane()
    rate=rospy.Rate(10)
    ma = vismsg.MarkerArray()


    
    while not rospy.is_shutdown():
        
        rate.sleep()
        ma.markers = []
        msg_pub_lane.waypoints = []
        
        if elkerules is not None or elkerules is not []:
            i = 0
            for e in elkerules:
                i += 1
                ori_arrow = vismsg.Marker()
                ori_arrow.ns = "orientation"
                ori_arrow.header.frame_id = "map"
                ori_arrow.type = ori_arrow.ARROW
                ori_arrow.pose.position.x = e[0]
                ori_arrow.pose.position.y = e[1]
                ori_arrow.pose.position.z = -1.36
                ori_arrow.pose.orientation.z = math.sin(e[2]/2.0)
                ori_arrow.pose.orientation.w = math.cos(e[2]/2.0)
                ori_arrow.action = ori_arrow.ADD
                ori_arrow.color.r = 0.2
                ori_arrow.color.g = 1.0
                ori_arrow.color.b = 0.6
                ori_arrow.color.a = 1.0
                ori_arrow.scale.x = 1.0
                ori_arrow.scale.y = 0.1
                ori_arrow.id = i                
                ma.markers.append(ori_arrow)
                #Velocity
                
                marker_lin_vel = vismsg.Marker()
                marker_lin_vel.type = marker_lin_vel.TEXT_VIEW_FACING
                marker_lin_vel.pose.position.x = e[0]
                marker_lin_vel.pose.position.y = e[1]
                marker_lin_vel.pose.position.z = 0.7
                marker_lin_vel.ns = "linvel"
                marker_lin_vel.header.frame_id = "map"
                marker_lin_vel.color.r = 1.0
                marker_lin_vel.color.g = 1.0
                marker_lin_vel.color.a = 1.0
                marker_lin_vel.scale.z = 0.5
                marker_lin_vel.id = i
                marker_lin_vel.text = str(round(e[3],1))      #### nem jo ####
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
                if i in collect_intersected_waypoints:
                    marker_lane_points.color.r=0.0
                    marker_lane_points.color.g=1.0
                    marker_lane_points.color.b=0.0

                
                marker_lane_points.color.a = 1.0
                marker_lane_points.scale.x = 0.4
                marker_lane_points.scale.y = 0.4
                marker_lane_points.scale.z = 0.4
                marker_lane_points.pose.position.x = e[0]
                marker_lane_points.pose.position.y = e[1]
                marker_lane_points.pose.position.z = -1.36
                marker_lane_points.pose.orientation.w = 1.0
                marker_lane_points.id = i 
                ma.markers.append(marker_lane_points)

                w0 = Waypoint()
                pose = PoseStamped()
                w0.twist = TwistStamped()

                pose.pose.position.x = e[0]
                pose.pose.position.y = e[1]
                pose.pose.position.z = -1.36
                pose.pose.orientation.z = math.sin((e[2])/2.0)
                pose.pose.orientation.w = math.cos((e[2])/2.0)
                w0.pose = pose
                w0.twist.twist.linear.x = e[3]
                msg_pub_lane.waypoints.append(w0)
            
            pub_based_waypoint_list.publish(msg_pub_lane)
            pub_new_data.publish(ma)
        


    #rospy.spin()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass