#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

costmap_2d::Costmap2D my_costmap;

using namespace std;

void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    std::cout << "Pose received"  << std::endl; 

    geometry_msgs::Pose target_pose = msg->pose.pose;

    MoveBaseClient ac("/move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) {

        ROS_INFO("Waiting for move_base action server to come up");

    }

    move_base_msgs::MoveBaseGoal goal; 

    goal.target_pose.header.frame_id= "odom" ; 

    goal.target_pose.header.stamp= ros::Time::now();

    goal.target_pose.pose = target_pose; 
    goal.target_pose.pose.orientation.w = 1.0; 

    ROS_INFO("Sending goal");

    ac.sendGoal(goal); 

    ac.waitForResult(); 

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Successfully moved to goal");
    }

    else {
        ROS_INFO("Failed to move to the goal");
    }
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "explore_node");

    ros::NodeHandle nh("explore_node"); 

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    
    string global_frame, robot_base_frame;  

    nh.param("global_costmap/global_frame", global_frame, std::string("odom"));
    nh.param("global_costmap/robot_base_frame", robot_base_frame, std::string("base_link"));

    costmap_2d::Costmap2DROS *global_costmap = new costmap_2d::Costmap2DROS("global_costmap", buffer);    
    costmap_2d::Costmap2DROS *local_costmap = new costmap_2d::Costmap2DROS("local_costmap", buffer);


    cout << "global_frame: " << global_frame << endl; 
    cout << "robot_base_frame: " << robot_base_frame << endl;

/*    costmap_2d::Costmap2D *costmap_one = costmap_one_wrapper->getCostmap();

    unsigned char* costmap_one_array  = costmap_one->getCharMap();

    costmap_one->saveMap("map_one.pgm");

    int rows =  sizeof costmap_one_array /sizeof costmap_one_array[0];

    int cols = sizeof costmap_one_array[0]/ sizeof (unsigned char);

    cout << "rows: " << rows << " cols: " << cols << endl;
    
  */

    ros::spin();




}



