# obstacle avoidance

This repository contains the algorithm which is capable of the avoidance of one static object at the moment with trapezoid shape.

## The parameters of the algorithm 

![trapez_parameterekkel](https://user-images.githubusercontent.com/51919446/116975967-c42e5080-acc0-11eb-9e3c-d99198bf09f8.png)

* **kiteres_iranya**: From which side should the car avoid the obstacle [string] *jobbra* or *balra*

* **kiteres_hossza**: How long should be the first stage of the avoidance (before the obstacle) [m]

* **oldaliranyu_eltolas**: How high should be the trapezoid from the original trajectory [m]

* **elkerules_hossza**: How long should be the second stage of the avoidance (alongside the obstacle)[m]

* **visszateres_hossza**: How long should be the return to the original trajectory (after the obstacle)[m]

* **distance_delta**: The distance between two points in the resampled trajectory [m]

* **lookahead**: In how many waypoints should the collision examination before the car happen. [int]

* **polygon_size_threshold**: The minimum length of a polygon what we get from euclidean cluster. Above this value we won't detect any objects.[m]

* **presence_threshold**: How many times should be the waypoint appear as occupaid before it becomes valid. The bigger value means that there is a lower chance of **false detect** but the path replannig will be **slower**. [int]

* **delete_threshold**: If no object in the waypoints the algorithm waits *delete_threshold* times before deleting from the valid points. [int] 

## Inputs of the algorihm

* /current_pose
* /points_no_ground (**Pointcloud2** from autoware ray ground filter)
* /detection/lidar_detector/objects  **DetectedObjectArray**  from autoware  euclidean_cluster_detect)
