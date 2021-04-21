#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <obstacle_avoidance/ParamsConfig.h>

void callback(obstacle_avoidance::ParamsConfig &config, uint32_t level){
    
}

int main(int argc, char **argv){
    ros::init(argc,argv,"obstacle_avoidance_params");

    dynamic_reconfigure::Server<obstacle_avoidance::ParamsConfig> server;
    dynamic_reconfigure::Server<obstacle_avoidance::ParamsConfig>::CallbackType f;

    f=boost::bind(&callback,_1,_2);
    server.setCallback(f);

    ros::spin();

    return 0;
}