#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


#include <actionlib/client/simple_action_client.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <mutex>

#include <move_base_msgs/MoveBaseAction.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

using namespace std;
 
class Explore{


    public:

        Explore(ros::NodeHandle &nh, tf2_ros::Buffer &buffer);

    private: 

        
        bool is_frontier(size_t mx, size_t my);
        void explore_level_three();
        void go_to_cell(size_t mx, size_t my);
        void publish_markers_array();
        void publish_markers();
        
        
        costmap_2d::Costmap2DROS* global_costmap, *local_costmap;
        costmap_2d::Costmap2D* global_costmap_, *local_costmap_;
        
        ros::NodeHandle nh_;
        ros::Publisher frontier_array_pub, frontier_pub; 

        vector<pair<size_t, size_t> >frontiers;
        
        string global_frame, robot_base_frame;
        unsigned char *global_og;

        size_t size_x, size_y;  
        double init_wx, init_wy; 

        
        int vis[4001][4001], frontier_vis[4001][4001] ; 
        

};

Explore::Explore(ros::NodeHandle &nh, tf2_ros::Buffer &buffer):nh_{nh}{

    cout << "Inside the Explore class constructor!" << endl;
    
    global_costmap = new costmap_2d::Costmap2DROS("global_costmap", buffer);  
    local_costmap = new costmap_2d::Costmap2DROS("local_costmap", buffer);

    global_costmap_ = global_costmap->getCostmap();  

    global_og = global_costmap_->getCharMap();
    
    size_x = global_costmap_->getSizeInCellsX();
    size_y = global_costmap_->getSizeInCellsY(); 

    geometry_msgs::PoseStamped global_pose; 

    bool flag = global_costmap->getRobotPose(global_pose); 

    if(flag) {cout << "global pose found successfully!" << endl;}

    init_wx = global_pose.pose.position.x , init_wy = global_pose.pose.position.y;

    cout << "init_wx: " << init_wx << " init_wy: " << init_wy << endl;

    frontier_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_markers", 10 );
        
    frontier_array_pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_markers_array", 10 );
        
    explore_level_three();

}

bool Explore::is_frontier(size_t mx, size_t my) {

    bool is_frontier = false;

    for(int i = (int)mx - 1; i <= mx + 1; i++) {

        for(int j = (int)my - 1; j <= my + 1; j++) {
            
            if(is_frontier) {   return is_frontier;  }

            if(i == mx && j == my) {continue; }

            if(i < 0 || i >= size_x || j < 0 || j >= size_y) {continue ; }

            unsigned char cell_cost = global_costmap_->getCost(i, j);

            if(cell_cost == costmap_2d::FREE_SPACE) { is_frontier = true; }

        }
    }
}

void Explore::go_to_cell(size_t mx, size_t my) {

    double wx, wy; 

    global_costmap_->mapToWorld(mx, my, wx, wy);
    
    geometry_msgs::Pose target_pose; 

    target_pose.position.x = wx; 
    target_pose.position.y = wy; 
    target_pose.position.z= 0 ; 

    target_pose.orientation.w = 1; 
    target_pose.orientation.x = target_pose.orientation.y = target_pose.orientation.z = 0 ;


    cout << "Calling the MoveBaseClient now!" << endl;
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) {

        ROS_INFO("Waiting for move_base action server to come up");

    }

    move_base_msgs::MoveBaseGoal goal; 

    goal.target_pose.header.frame_id= "map" ; 

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

void Explore::publish_markers() {

    cout << "INSIDE THE PUBLISH_MARKERS FUNCTION! " << endl;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

   
    marker.ns = nh_.getNamespace();
    cout << "namespace: " << marker.ns << endl;

    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

 
    marker.pose.position.x = init_wx + 1;
    marker.pose.position.y = init_wy + 3;
    marker.pose.position.z = 1; 
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.lifetime = ros::Duration(100.0);

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1.0;
    
    marker.color.a = 1.0; // Don't forget to set the alpha!
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    ros::Time start = ros::Time::now(); 
    ros::Duration del(3000.0);
    ros::Time end = start + del ;

    cout << "Loop starts at: " << ros::Time::now().toSec() << endl;
    int id = 0;
    while(ros::Time::now() < start + del) {
        
        
        frontier_pub.publish(marker);
        //ros::Duration(0.5).sleep();
        //cout << "Current time: " << ros::Time::now().toSec() << endl;
        
        
    }

    cout << "Loop ends at: " << ros::Time::now().toSec() << endl;
   

}

void Explore::publish_markers_array() {

    cout << "INSIDE THE PUBLISH_MARKERS FUNCTION! " << endl;

    int sz = (int)frontiers.size();

    cout << "sz: " << sz << endl;

    visualization_msgs::MarkerArray frontier_marker_array;

    for(int i =0 ;i  < sz ; i++){

        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();

        marker.ns = nh_.getNamespace();
        cout << "namespace: " <<marker.ns<< endl;

        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        size_t mx = frontiers[i].first, my = frontiers[i].second;

        double wx, wy; 
        global_costmap_->mapToWorld(mx, my, wx, wy);

        marker.pose.position.x =  wx;
        marker.pose.position.y =   wy;
        marker.pose.position.z = 1; 
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = ros::Duration(100.0);

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 1.0;

        marker.color.a = 1.0; // Don't forget to set the alpha!

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;


        frontier_marker_array.markers.push_back(marker);

        cout << "frontier_marker_array.markers.size(): " << frontier_marker_array.markers.size() << endl; 

    }

    ros::Time start = ros::Time::now(); 
    ros::Duration del(10000.0);
    ros::Time end = start + del ;

    cout << "Loop starts at: " << ros::Time::now().toSec() << endl;

    while(ros::Time::now() < start + del) {

        frontier_array_pub.publish(frontier_marker_array);
    }

    cout << "Loop ends at: " << ros::Time::now().toSec() << endl;


}

void Explore::explore_level_three() {

    ros::Rate loop_rate(0.1);

    costmap_2d::Costmap2D* global_costmap_ = global_costmap->getCostmap();

    int free_cell_cnt =0 ; 

    memset(vis, -1, sizeof(vis));
    memset(frontier_vis, -1, sizeof(frontier_vis));

    while(ros::ok()) {

        cout << "------------- RESTARTING INSIDE THE MAIN WHILE LOOP! ------------------------" << endl;
        cout <<"Sleeping for 5 seconds!" << endl;

        ros::Duration(5.0).sleep();

       unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getMutex()));
        
        geometry_msgs::PoseStamped global_pose; 

        bool flag = global_costmap->getRobotPose(global_pose);

        unsigned int mx , my; 

        global_costmap_->worldToMap(global_pose.pose.position.x , global_pose.pose.position.y, mx, my);

        queue<pair<unsigned int, unsigned int> > q; 

        q.push({mx, my});

        cout << "global_pose mx: " << mx << " my: " << my << endl;

        while(!q.empty()) {

            
            pair<unsigned int, unsigned int> front = q.front(); 
            
            int cx =  front.first , cy = front.second; 

            cout << "cx: " << cx << " cy: " << cy << endl;

            vis[cx][cy] = 1;

            q.pop(); 

            for(int i = cx - 10; i <= cx+ 10; i++) {

                for(int j = cy - 10; j <= cy + 10 ; j++) {

                    if(i == cx && j == cy) {continue;}

                    if(i < 0 || i >= size_x || j < 0 || j >= size_y ) {continue;}

                    if(vis[i][j] > 0) {continue;}

                    unsigned char cell_cost = global_costmap_->getCost(i, j);

                    //cout << "(" << i << "," << j << "): " << (int)cell_cost << endl; 

                    if(cell_cost == costmap_2d::FREE_SPACE && (vis[i][j] == -1) ) {
                        
                        q.push({i, j});
                        free_cell_cnt++;
                        vis[i][j] = 1;
                    }

                    else if(cell_cost == costmap_2d::NO_INFORMATION && is_frontier(i, j)) {
                        
                       if(frontier_vis[i][j] > 0) {continue;}

                        frontiers.push_back({i, j});
                        frontier_vis[i][j] = 1;

                    }


                }
            }    

        }
        
        cout  << "free_cell_cnt: " << free_cell_cnt << endl;

        int sz = (int)frontiers.size(); 
        cout << "sz: " << sz << endl; 
        
        //
        
        if(sz > 0) {
            
            
            pair<unsigned int, unsigned int> target = frontiers[sz/2];

            
            //publish_marker_array();
            publish_markers_array();
            cout << "Out of the publish marker array function" << endl;
            cout << "current location: (" << mx << "," << my << ")" << endl;
            cout << "Attempting to move to a frontier point - (" << target.first <<"," << target.second << ")" << endl;
            
            go_to_cell(target.first, target.second);

        }

        lock.unlock();

        ros::spinOnce();
        
        //loop_rate.sleep();

    }


}



int main(int argc, char** argv) {

    ros::init(argc, argv, "explore_node");

    ros::NodeHandle nh("explore_node"); 

   
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    
    
    Explore* explore_one = new Explore(nh, buffer);

   
    return 0;
}   



