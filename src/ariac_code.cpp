#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_srvs/Trigger.h"
#include "osrf_gear/msg/Order.msg"

std::vector<osrf_gear::Order> orders;
orders.clear();
void orderCallback(const osrf_gear::Order msg){
    orders.push_back(msg)
}

int main(int argc, char **argv) {

    orders.clear();
    // Initialize the node
    ros::init(argc, argv, "ariac_entry");
    
    // Create the nodehandle instance
    ros::NodeHandle n;

    // Call to subscribe to no topic for now
    ros::Subscriber sub = n.subscribe("/ariac/orders", 10, orderCallback);

    // Call to publish to no topic for now
    //ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // set the loop frequency
    ros::Rate loop_rate(10);

    // initialize the competition start variable
    std_srvs::Trigger begin_comp;

    // create the service client
    ros::ServiceClient begin_client = 
        n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    if(begin_client.call(begin_comp) == false){
        ROS_ERROR_STREAM("Competition not started");
    }
    else if(begin_comp.response.success == false){
        ROS_WARN("Competition service returned failure: %s",
            begin_comp.response.message.c_str());
    }
    else{
        ROS_INFO("Competition service called succesfully: %s", 
            begin_comp.response.message.c_str());
    }
    while (ros::ok())
    {
        

        // Necessary lines for smooth program flow
        ros::spinOnce();
        loop_rate.sleep();
    }


  return 0;
}