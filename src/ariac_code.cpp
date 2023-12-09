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
#include "sensor_msgs/JointState.h"
#include "ik_service/PoseIK.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "ur_kinematics/ur_kin.h"

std::vector<osrf_gear::Order> orders;
int current_shipment, current_product = 0;
void orderCallback(const osrf_gear::Order msg)
{
    orders.push_back(msg);
}

sensor_msgs::JointState joint_states;
void joint_states_cb(const sensor_msgs::JointState msg)
{
    joint_states = msg;
}

void iterateOrder()
{
    // Another product in this shipment?
    if (current_product + 1 < sizeof(orders[0].shipments[current_shipment].products))
    {
        current_product++; // If so, move to next
        ROS_INFO("Next Product, number %i", current_product);
    }
    else
    {
        ROS_INFO("End of shipment, on to next");
        current_product = 0; // If not, move to next shipment
        // Another shipment in this order?
        if (current_shipment + 1 < sizeof(orders[0].shipments))
        {
            current_shipment++; // If so, move to next
            ROS_INFO("Next Shipment, number %i", current_shipment);
        }
        else
        {
            ROS_INFO("End of order");
            // If not, order is complete. Remove the order
            orders.erase(orders.begin());
        }
    }
}
osrf_gear::LogicalCameraImage images[10];
void lc_bin1_cb(const osrf_gear::LogicalCameraImage msg) { images[1] = msg; }
void lc_bin2_cb(const osrf_gear::LogicalCameraImage msg) { images[2] = msg; }
void lc_bin3_cb(const osrf_gear::LogicalCameraImage msg) { images[3] = msg; }
void lc_bin4_cb(const osrf_gear::LogicalCameraImage msg) { images[4] = msg; }
void lc_bin5_cb(const osrf_gear::LogicalCameraImage msg) { images[5] = msg; }
void lc_bin6_cb(const osrf_gear::LogicalCameraImage msg) { images[6] = msg; }
void lc_agv1_cb(const osrf_gear::LogicalCameraImage msg) { images[7] = msg; }
void lc_agv2_cb(const osrf_gear::LogicalCameraImage msg) { images[8] = msg; }
void lc_qcs2_cb(const osrf_gear::LogicalCameraImage msg) { images[9] = msg; }
void lc_qcs1_cb(const osrf_gear::LogicalCameraImage msg) { images[0] = msg; }

void goalActiveCallback()
{
    ROS_INFO("New trajectory sent, goal is active");
}
void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &fb)
{
}
void resultCallback(const actionlib::SimpleClientGoalState &gs, const control_msgs::FollowJointTrajectoryResultConstPtr &res)
{
    ROS_INFO_STREAM("Process finished, goal state: " << gs.toString().c_str());
}
trajectory_msgs::JointTrajectory joint_trajectory;
void addPoint(const geometry_msgs::Pose input_pose, ros::ServiceClient ik_client, trajectory_msgs::JointTrajectory *joint_trajectory, int gripper)
{
    ik_service::PoseIK pose_ik;

    // Send the desired position to the joint_trajectory node
    pose_ik.request.part_pose = input_pose;

    // pose_ik.request.part_pose.position.x = -0;
    ROS_INFO_STREAM("inner loop reqyest: " << pose_ik.request);
    ROS_INFO_STREAM("Sending to service: " << pose_ik.request);
    if (ik_client.call(pose_ik))
    {
        ROS_INFO("Response returned %i solutions", pose_ik.response.num_sols);
    }
    else
    {
        ROS_ERROR("Failed to call service ik_service");
    }
    int current_point = (*joint_trajectory).points.size();
    // Set a start and end point.
    (*joint_trajectory).points.resize(current_point + 1);
    // Set the start point to the current position of the joints from joint_states.
    // When to start (immediately upon receipt).
    (*joint_trajectory).points[current_point].time_from_start = ros::Duration(2.0 * (current_point + 1));
    (*joint_trajectory).points[current_point].positions.resize((*joint_trajectory).joint_names.size() + 1);

    // Must select which of the num_sols solutions to use. Just start with the first.
    ROS_INFO_STREAM(pose_ik.response);
    int joint_sols_indx = 0;
    for (int i = 0; i < pose_ik.response.num_sols; i++)
    {
        ROS_INFO("Checking response [%i]", i);
        if ((pose_ik.response.joint_solutions[i].joint_angles[0] >=  0) && (pose_ik.response.joint_solutions[i].joint_angles[0] <=  3.1415 / 2) &&
        (pose_ik.response.joint_solutions[i].joint_angles[1] >= (3.14)) && (pose_ik.response.joint_solutions[i].joint_angles[1] <= (3.14 * 2)) &&
        (pose_ik.response.joint_solutions[i].joint_angles[4] >= 4.6) && (pose_ik.response.joint_solutions[i].joint_angles[4] <= 4.9))
        {
            joint_sols_indx = i;
            ROS_INFO("Selected response [%i]", joint_sols_indx);
            break;
        }
    }
    if (joint_sols_indx >= 8)
    {
        ROS_ERROR("No adequate solutions");
        joint_sols_indx = 0;
    }
    ROS_INFO("Using solution [%i]", joint_sols_indx);

    // The actuators are commanded in an odd order, enter the joint positions in the correct positions

    // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
    (*joint_trajectory).points[current_point].positions[0] = joint_states.position[1];
    for (int indy = 0; indy < 6; indy++)
    {
        (*joint_trajectory).points[current_point].positions[indy + 1] = pose_ik.response.joint_solutions[joint_sols_indx].joint_angles[indy];
    }
    (*joint_trajectory).points[current_point].positions[(*joint_trajectory).joint_names.size()] = gripper;
    ROS_INFO_STREAM("Current point (in loop): " << (*joint_trajectory));
}

int main(int argc, char **argv)
{
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
    ros::Publisher joint_trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command", 100);

    // Subscribe to the arm 1 joint states topic
    ros::Subscriber arm1_joint_states = n.subscribe("/ariac/arm1/joint_states", 10, joint_states_cb);

    // Instantiate variables for use with the kinematic system.
    double T_pose[4][4], T_des[4][4];
    double q_pose[6], q_des[8][6];

    // set the loop frequency
    ros::Rate loop_rate(10);

    // initialize the competition start variable
    std_srvs::Trigger begin_comp;

    // create the competition start service client
    ros::service::waitForService("/ariac/start_competition", 100000);
    ros::ServiceClient begin_client =
        n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    if (begin_client.call(begin_comp) == false)
    {
        ROS_ERROR_STREAM("Competition not started");
    }
    else if (begin_comp.response.success == false)
    {
        ROS_WARN("Competition service returned failure: %s",
                 begin_comp.response.message.c_str());
    }
    else
    {
        ROS_INFO("Competition service called succesfully: %s",
                 begin_comp.response.message.c_str());
    }

    // initialize the competition start variable
    osrf_gear::GetMaterialLocations find_location;

    // create the product locations service client
    ros::ServiceClient location_client =
        n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    ros::service::waitForService("/ariac/material_locations", 10000);
    if (location_client.call(find_location) == false)
    {
        ROS_ERROR_STREAM("Material Locations Service not started");
    }

    // Use the ik_pose service
    ros::ServiceClient ik_client = n.serviceClient<ik_service::PoseIK>("/pose_ik");
    ros::service::waitForService("/pose_ik", 10000);
    bool orderFilled = true;

    // Instantiate the Action Server Client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
        trajectory_ac("ariac/arm1/arm/follow_joint_trajectory", true);

    // Create the structure to populate for running the Action Server
    control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();

    int count = 0;
    int msgCnt = 0;
    while (ros::ok())
    {
        ROS_INFO_STREAM_THROTTLE(10, "Current joint states " << joint_states);
        if (orders.size() > 0 && orderFilled)
        {
            // Indicate that a new order is being processed
            orderFilled = false;

            // Request material location service to find the current material type
            std::string product_type = orders[0].shipments[current_shipment].products[current_product].type;
            ROS_INFO_STREAM("Product of type: " + product_type);

            // Use the response to determine where the part is located
            find_location.request.material_type = product_type;
            if (location_client.call(find_location))
            {
                ROS_INFO_STREAM("Current joint states " << joint_states);
                std::string bin_s = find_location.response.storage_units[0].unit_id;
                ROS_INFO_STREAM("Located in: " + bin_s);

                // Determine which bin is being used
                int bin;
                std::string frame;
                if (bin_s == "bin1")
                {
                    bin = 1;
                    frame = "logical_camera_bin1_frame";
                }
                else if (bin_s == "bin2")
                {
                    bin = 2;
                    frame = "logical_camera_bin2_frame";
                }
                else if (bin_s == "bin3")
                {
                    bin = 3;
                    frame = "logical_camera_bin3_frame";
                }
                else if (bin_s == "bin4")
                {
                    bin = 4;
                    frame = "logical_camera_bin4_frame";
                }
                else if (bin_s == "bin5")
                {
                    bin = 5;
                    frame = "logical_camera_bin5_frame";
                }
                else if (bin_s == "bin6")
                {
                    bin = 6;
                    frame = "logical_camera_bin6_frame";
                }
                else
                {
                    bin = 0;
                    frame = "logical_camera_bin4_frame";
                }

                osrf_gear::LogicalCameraImage image = images[bin];

                // Retrieve the transformation
                geometry_msgs::TransformStamped tfStamped;
                try
                {
                    tfStamped = tfBuffer.lookupTransform("arm1_base_link", frame, ros::Time(0.0), ros::Duration(1.0));
                    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_ERROR("%s", ex.what());
                }
                // Create variables
                geometry_msgs::PoseStamped part_pose, goal_pose;
                // Copy pose from the logical camera.
                part_pose.pose = image.models[0].pose;
                tf2::doTransform(part_pose, goal_pose, tfStamped);

                // Display the pose and the transformed pose as well
                ROS_INFO_STREAM("Found " + product_type + " in " + bin_s + " at camera Pose " << image.models[0].pose << "\nand arm pose " << goal_pose.pose);
                joint_trajectory.points.clear();
                // Fill out the joint trajectory header.
                // Each joint trajectory should have an non-monotonically increasing sequence number.
                joint_trajectory.header.seq = count++;
                joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
                joint_trajectory.header.frame_id = "/world";      // Frame in which this is specified.

                // Set the names of the joints being used. All must be present.
                joint_trajectory.joint_names.clear();
                joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
                joint_trajectory.joint_names.push_back("shoulder_pan_joint");
                joint_trajectory.joint_names.push_back("shoulder_lift_joint");
                joint_trajectory.joint_names.push_back("elbow_joint");
                joint_trajectory.joint_names.push_back("wrist_1_joint");
                joint_trajectory.joint_names.push_back("wrist_2_joint");
                joint_trajectory.joint_names.push_back("wrist_3_joint");

                // Set the start point to the current position of the joints from joint_states.
                joint_trajectory.points.resize(1);
                joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
                for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++)
                {
                    for (int indz = 0; indz < joint_states.name.size(); indz++)
                    {
                        if (joint_trajectory.joint_names[indy] == joint_states.name[indz])
                        {
                            joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                            break;
                        }
                    }
                }
                // When to start (immediately upon receipt).
                joint_trajectory.points[0].time_from_start = ros::Duration(0.2);
                geometry_msgs::Pose to_send;
                to_send.position.x = 0;
                to_send.position.y = goal_pose.pose.position.y;
                to_send.position.z = goal_pose.pose.position.z + 0.2;

                addPoint(to_send, ik_client, (trajectory_msgs::JointTrajectory *)&joint_trajectory, 0);
                // Next point will be just the y movement to the part
                to_send.position.x = goal_pose.pose.position.x;
                addPoint(to_send, ik_client, (trajectory_msgs::JointTrajectory *)&joint_trajectory, 0);
                
                // Next point will be to z + 0.1
                to_send.position.z = goal_pose.pose.position.z + .1;
                addPoint(to_send, ik_client, (trajectory_msgs::JointTrajectory *)&joint_trajectory, 0);

                // Next will be the proper z.
                to_send.position.z = goal_pose.pose.position.z + .01;
                addPoint(to_send, ik_client, (trajectory_msgs::JointTrajectory *)&joint_trajectory, 0);

                ROS_INFO_STREAM("Sending to /ariac/arm/arm1/command" << joint_trajectory);
                joint_trajectory_pub.publish(joint_trajectory);
                // joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
                // joint_trajectory_as.action_goal.header.seq = count++;
                // joint_trajectory_as.action_goal.header.stamp = ros::Time::now(); // When was this message created.
                // joint_trajectory_as.action_goal.header.frame_id = "/world";
                // joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
                // joint_trajectory_as.action_goal.goal_id.id = "Message" + msgCnt++;

                // trajectory_ac.sendGoal(joint_trajectory_as.action_goal.goal, &resultCallback, &goalActiveCallback, &feedbackCallback);
            }
        }

        loop_rate.sleep();
    }

    return 0;
}