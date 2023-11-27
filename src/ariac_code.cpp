#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_srvs/Trigger.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "geometry_msgs/Pose.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

std::vector<osrf_gear::Order> orders;
int current_shipment, current_product = 0;
void orderCallback(const osrf_gear::Order msg){
    orders.push_back(msg);
}

void iterateOrder(){
    // Another product in this shipment?
    if(current_product + 1 < sizeof(orders[0].shipments[current_shipment].products)){
        current_product++; // If so, move to next
        ROS_INFO("Next Product, number %i", current_product);
    }
    else{
        ROS_INFO("End of shipment, on to next");
        current_product = 0; // If not, move to next shipment
        // Another shipment in this order?
        if(current_shipment + 1 < sizeof(orders[0].shipments)){
            current_shipment++; // If so, move to next
            ROS_INFO("Next Shipment, number %i", current_shipment);
        }
        else{
            ROS_INFO("End of order");
            // If not, order is complete. Remove the order
            orders.erase(orders.begin());
        }
    }
}
osrf_gear::LogicalCameraImage images [10];
void lc_bin1_cb(const osrf_gear::LogicalCameraImage msg){images[1] = msg;}
void lc_bin2_cb(const osrf_gear::LogicalCameraImage msg){images[2] = msg;}
void lc_bin3_cb(const osrf_gear::LogicalCameraImage msg){images[3] = msg;}
void lc_bin4_cb(const osrf_gear::LogicalCameraImage msg){images[4] = msg;}
void lc_bin5_cb(const osrf_gear::LogicalCameraImage msg){images[5] = msg;}
void lc_bin6_cb(const osrf_gear::LogicalCameraImage msg){images[6] = msg;}
void lc_agv1_cb(const osrf_gear::LogicalCameraImage msg){images[7] = msg;}
void lc_agv2_cb(const osrf_gear::LogicalCameraImage msg){images[8] = msg;}
void lc_qcs2_cb(const osrf_gear::LogicalCameraImage msg){images[9] = msg;}
void lc_qcs1_cb(const osrf_gear::LogicalCameraImage msg){images[0] = msg;}

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "ariac_entry");
    
    // Initiate the transfrom listener first so it has time to collect everything
    // Declare the transformation buffer to maintain a list of transformations
    tf2_ros::Buffer tfBuffer;
    // Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
    tf2_ros::TransformListener tfListener(tfBuffer);

    orders.clear();
    // Create the nodehandle instance
    ros::NodeHandle n;

    // Call to subscribe to the orders topic
    ros::Subscriber sub = n.subscribe("/ariac/orders", 10, orderCallback);

    // Call to subscribe to all of the logical_cameras
    ros::Subscriber lc_bin1 = n.subscribe("/ariac/logical_camera_bin1", 10, lc_bin1_cb);
    ros::Subscriber lc_bin2 = n.subscribe("/ariac/logical_camera_bin2", 10, lc_bin2_cb);
    ros::Subscriber lc_bin3 = n.subscribe("/ariac/logical_camera_bin3", 10, lc_bin3_cb);
    ros::Subscriber lc_bin4 = n.subscribe("/ariac/logical_camera_bin4", 10, lc_bin4_cb);
    ros::Subscriber lc_bin5 = n.subscribe("/ariac/logical_camera_bin5", 10, lc_bin5_cb);
    ros::Subscriber lc_bin6 = n.subscribe("/ariac/logical_camera_bin6", 10, lc_bin6_cb);
    ros::Subscriber lc_agv1 = n.subscribe("/ariac/logical_camera_agv1", 10, lc_agv1_cb);
    ros::Subscriber lc_agv2 = n.subscribe("/ariac/logical_camera_agv2", 10, lc_agv2_cb);
    ros::Subscriber lc_qcs1 = n.subscribe("/ariac/quality_control_sensor_1", 10, lc_qcs1_cb);
    ros::Subscriber lc_qcs2 = n.subscribe("/ariac/quality_control_sensor_2", 10, lc_qcs2_cb);
    // Call to publish to n2 topic for no2
    //ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    

    // set the loop frequency
    ros::Rate loop_rate(10);

    // initialize the competition start variable
    std_srvs::Trigger begin_comp;

    // create the competition start service client
    ros::ServiceClient begin_client = 
        n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    begin_client.waitForExistence();

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

    // initialize the competition start variable
    osrf_gear::GetMaterialLocations find_location;

    // create the product locations service client
    ros::ServiceClient location_client = 
        n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    if(location_client.call(find_location) == false){
        ROS_ERROR_STREAM("Material Locations Service not started");
    }

    bool orderFilled = true;

    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    while (ros::ok())
    {
        if(orders.size() > 0 && orderFilled){
            // Indicate that a new order is being processed
            orderFilled = false;

            // Request material location service to find the current material type
            std::string product_type = orders[0].shipments[current_shipment].products[current_product].type;
            ROS_INFO_STREAM("Product of type: " + product_type);

            // Use the response to determine where the part is located
            find_location.request.material_type = product_type;
            if(location_client.call(find_location)){
                std::string bin_s = find_location.response.storage_units[0].unit_id;
                ROS_INFO_STREAM("Located in: " + bin_s);

                // Determine which bin is being used
                int bin;
                std::string frame;
                if(bin_s == "bin1"){bin = 1;frame="logical_camera_bin1_frame";}
                else if(bin_s == "bin2"){bin = 2;frame="logical_camera_bin2_frame";}
                else if(bin_s == "bin3"){bin = 3;frame="logical_camera_bin3_frame";}
                else if(bin_s == "bin4"){bin = 4;frame="logical_camera_bin4_frame";}
                else if(bin_s == "bin5"){bin = 5;frame="logical_camera_bin5_frame";}
                else if(bin_s == "bin6"){bin = 6;frame="logical_camera_bin6_frame";}
                else{bin = 0;frame="logical_camera_bin4_frame";}

                osrf_gear::LogicalCameraImage image = images[bin];

                // Retrieve the transformation
                geometry_msgs::TransformStamped tfStamped;
                try {
                    tfStamped = tfBuffer.lookupTransform("arm1_base_link", frame, ros::Time(0.0), ros::Duration(1.0));
                    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
                } catch (tf2::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
                }
                // Create variables
                geometry_msgs::PoseStamped part_pose, goal_pose;
                // Copy pose from the logical camera.
                part_pose.pose = image.models[0].pose;
                tf2::doTransform(part_pose, goal_pose, tfStamped);

                // Display the pose and the transformed pose as well
                ROS_INFO_STREAM("Found " + product_type + " in " + bin_s + " at camera Pose " << image.models[0].pose << "\nand arm pose " << goal_pose.pose);
            
            }
        }



        // Necessary lines for smooth program flow
        ros::spinOnce();
        loop_rate.sleep();
    }


  return 0;
}