
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

    struct Point{

        __uint32_t x, y;

    };

    struct Frontier{

        __uint32_t num_points; 
        Point median;
        vector<Point> frontier_points;

    };


    public:

        Explore(ros::NodeHandle &nh, tf2_ros::Buffer &buffer);
        void explore_level_four();
        
        vector<Frontier> frontier_collection;

        ros::Publisher frontier_array_pub, frontier_pub, marker_pub; 

    private: 
        
        // Member functions
        bool is_frontier(size_t mx, size_t my);
        void go_to_cell(size_t mx, size_t my);
        bool has_free_cell_neighbour(int a, int b);
        geometry_msgs::Pose get_currrent_pose_map();    
        void go_to_frontier_median();
        //void publish_markers_array(vector<pair<size_t, size_t> > &frontiers_);
        
        void publish_markers_array(Frontier &frontier);
        void publish_point(__uint32_t median_x, __uint32_t median_y);
        
        costmap_2d::Costmap2DROS* global_costmap, *local_costmap;
        costmap_2d::Costmap2D* global_costmap_, *local_costmap_;
        
        ros::NodeHandle nh_;
      
        vector<pair<size_t, size_t> >frontiers;
        
        string global_frame, robot_base_frame;
        unsigned char *global_og;

        size_t size_x, size_y;  
        double init_wx, init_wy; 

        int vis[4000][4000], frontier_vis[4000][4000] ; 

        int map_open_list[4000][4000], map_close_list[4000][4000];
        int frontier_open_list[4000][4000], frontier_close_list[4000][4000];
        
};

Explore::Explore(ros::NodeHandle &nh, tf2_ros::Buffer &buffer):nh_{nh}{

    cout << "Inside the Explore class constructor!" << endl;
    
    global_costmap = new costmap_2d::Costmap2DROS("global_costmap", buffer);  
    local_costmap = new costmap_2d::Costmap2DROS("local_costmap", buffer);

    global_costmap_ = global_costmap->getCostmap();  

    global_og = global_costmap_->getCharMap();
    
    size_x = global_costmap_->getSizeInCellsX();
    size_y = global_costmap_->getSizeInCellsY(); 

    cout << "size_x: " << size_x << " size_y: " << size_y << endl;

    cout  << "Sleeping for 3 seconds inside the constructor!" << endl;
    ros::Duration(5.0).sleep();
    
    geometry_msgs::PoseStamped global_pose; 

    bool flag = global_costmap->getRobotPose(global_pose); 

    if(flag) {cout << "global pose found successfully!" << endl;}

    init_wx = global_pose.pose.position.x , init_wy = global_pose.pose.position.y;

    __uint32_t mx , my ; 

    global_costmap_->worldToMap(init_wx, init_wy, mx, my);

    cout << "costmap_2d::NO_INFORMATION: " << (__uint32_t)costmap_2d::NO_INFORMATION << endl;
    cout << "init_wx: " << init_wx << " init_wy: " << init_wy << " cell_cost: " << (__uint32_t)global_costmap_->getCost(mx, my) << endl;

    cout << "is_frontier(mx, my): " << (int)is_frontier(mx, my) << endl; 

    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        
    frontier_array_pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_markers_array", 10 );

}

bool Explore::is_frontier(size_t mx, size_t my) {

    bool is_frontier = false;

    unsigned char cell_cost = global_costmap_->getCost(mx, my);

    if(cell_cost != costmap_2d::NO_INFORMATION) {return false;}

    for(int i = (int)mx - 1; i <= mx + 1; i++) {

        for(int j = (int)my - 1; j <= my + 1; j++) {
   
            if(i == mx && j == my) {continue; }

            if(i < 0 || i >= size_x || j < 0 || j >= size_y) {continue ; }

            unsigned char cell_cost = global_costmap_->getCost(i, j);

            if(cell_cost == costmap_2d::FREE_SPACE) {
                
                cout << "(" << i << "," <<  j << ") is free space" << endl;
                return true; 
                
            }

        }
    }

    return false;
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

void Explore::publish_markers_array(Frontier &frontier) {

    //cout << "INSIDE THE PUBLISH_MARKERS FUNCTION! " << endl;


    int sz = (int)frontier.num_points;
    int cnt = 0 ;
        
    cout << "Size of the frontier array: " << sz << endl;

    visualization_msgs::MarkerArray frontier_marker_array;

    for(int i =0 ;i  <sz ; i++){

        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();

        marker.ns = nh_.getNamespace();
        //cout << "namespace: " <<marker.ns<< endl;

        marker.id = i + 50;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        size_t mx = frontier.frontier_points[i].x, my = frontier.frontier_points[i].y;

        double wx, wy, median_wx, median_wy; 
        
        global_costmap_->mapToWorld(mx, my, wx, wy);

        marker.pose.position.x =  wx;
        marker.pose.position.y =   wy;
        marker.pose.position.z = 1; 
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = ros::Duration();

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.a = 1.0; // Don't forget to set the alpha!

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;


        frontier_marker_array.markers.push_back(marker);

        //cout << "frontier_marker_array.markers.size(): " << frontier_marker_array.markers.size() << endl; 

    }

    cout << "cnt:  " << cnt << " sz " << sz <<  endl;


    ros::Time start = ros::Time::now(); 
    ros::Duration del(15.0);
    ros::Time end = start + del ;

    //cout << "Loop starts at: " << ros::Time::now().toSec() << endl;

    while(ros::Time::now() < start + del) {

        frontier_array_pub.publish(frontier_marker_array);
    }

    //cout << "Loop ends at: " << ros::Time::now().toSec() << endl;


}

bool Explore::has_free_cell_neighbour(int mx, int my) {

    
    for(int i = mx- 1; i <= mx + 1; i++) {

        for(int j = my - 1; j <= my + 1; j++) {

            if(i < 0 || j < 0 || i >= size_x || j >= size_y || (i ==mx  && j== my) ) {continue;}

            unsigned char cell_cost = global_costmap_->getCost(i, j);

            if(cell_cost == costmap_2d::FREE_SPACE) {

                return true;
            }
        }

    }

    return false;

}

geometry_msgs::Pose Explore::get_currrent_pose_map() {

    geometry_msgs::PoseStamped current_pose;

    __uint32_t mx, my; 
    double wx, wy; 

    //global_costmap->getRobotPose(current_pose); 

    global_costmap->getRobotPose(current_pose);
    
    global_costmap_->worldToMap(wx, wy, mx, my);

    geometry_msgs::Pose transformed_pose;
    transformed_pose.position.x = mx; 
    transformed_pose.position.y = my;
    
    return transformed_pose;

}

void Explore::go_to_frontier_median() {

    cout << "Inside the go_to_frontier_median funtion!" << endl << endl;
    cout << "frontier_collection.size(): " << frontier_collection.size() << endl;

    int  mx_index = -1, mx_val =-1; 

    for(int i = 0; i < frontier_collection.size(); i++) {
        
        //cout << "mx_val: " << mx_val << " frontier_collection[i].size(): " << frontier_collection[i].size() << endl;

        if((int)frontier_collection[i].num_points > mx_val) {
            
            cout << "Updating mx_index and mx_val" << endl;
            mx_val = frontier_collection[i].num_points; 
            mx_index = i;

            cout << "Updated mx_val: " << mx_val << " mx_index: " << mx_index << endl;

        }

    }
    
    cout << "mx_index: " << mx_index << " mx_val: " << mx_val << endl;

    if(mx_index == -1) {

        cout << "ALERT: mx_index is -1" << endl;
        ros::Duration(3.0).sleep();
        return;
    
    }

    Frontier frontier = frontier_collection[mx_index];

    Point median_frontier_point = frontier.median; 

    __uint32_t median_x = median_frontier_point.x , median_y = median_frontier_point.y;

    geometry_msgs::Pose current_pose = get_currrent_pose_map();
    
    cout << "current_pose.x: " << current_pose.position.x << " current_pose.y: " << current_pose.position.y << endl;

    cout << "median_x: " << median_x << " median_y: " << median_y << endl;
    
    cout << "Publishing the median marker" << endl; 
    cout << "Sleeping for 2 seconds!" << endl;
    
    ros::Duration(2.0).sleep();

    publish_point(median_x, median_y);

    cout << "Trying to publish the markers for the frontier - V" << endl; 
    
    publish_markers_array(frontier);

    ros::Duration(2.0).sleep();
    
    cout << "Going to the frontier median pos!" << endl;

    go_to_cell(median_x, median_y);

}

void Explore::publish_point(__uint32_t mx , __uint32_t my) {

    ros::Rate r(1);
    
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    ros::Time current_time = ros::Time().now(); 

    ros::Time final = current_time + ros::Duration(15.0); 

    double wx, wy; 
    global_costmap_->mapToWorld(mx, my, wx, wy);

    while (ros::Time().now() < final)
//    while(ros::ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.ns = nh_.getNamespace() + "_point";
        marker.id = 10;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = (float)wx;
        marker.pose.position.y = (float)wy;
        marker.pose.position.z = 2;
        
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
        }

        marker_pub.publish(marker);

        switch (shape)
        {
        case visualization_msgs::Marker::CUBE:
        shape = visualization_msgs::Marker::SPHERE;
        break;
        case visualization_msgs::Marker::SPHERE:
        shape = visualization_msgs::Marker::ARROW;
        break;
        case visualization_msgs::Marker::ARROW:
        shape = visualization_msgs::Marker::CYLINDER;
        break;
        case visualization_msgs::Marker::CYLINDER:
        shape = visualization_msgs::Marker::CUBE;
        break;
        }

        r.sleep();
    }



}

void Explore::explore_level_four() {
    
    cout << "size_x: " << size_x << " size_y: " << size_y << endl;

    while(ros::ok){

        memset(map_open_list, -1, sizeof(map_open_list));
        memset(map_close_list, -1, sizeof(map_close_list));

        memset(frontier_open_list, -1, sizeof(frontier_open_list));
        memset(frontier_close_list, -1, sizeof(frontier_close_list));
   
        cout << "Starting the ros::ok() loop -- sleeping for 1 second" << endl;
        ros::Duration(1.0).sleep();

        cout << "Clearing frontier_collection vector!" << endl; 
        frontier_collection.clear();
        cout << "new_size of frontier_collection: " << frontier_collection.size() << endl;

        unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getMutex()));
        
        cout << "Locking the costmap!" << endl;

        queue<pair<size_t, size_t> > q_m;

        geometry_msgs::PoseStamped global_pose;

        bool getRobotPose_flag = global_costmap->getRobotPose(global_pose);

        if(getRobotPose_flag) { cout << "global_pose updated successfully!" << endl; }

        else { cout << "Error - Unable to update global_pose for the bot.";}

        double wx = global_pose.pose.position.x, wy = global_pose.pose.position.y;

        unsigned int mx, my; 

        bool worldToMap_flag =  global_costmap_->worldToMap(wx, wy, mx, my);

        //if(worldToMap_flag) {cout << "Successfully transformed from world frame to map frame!" << endl;}
        
        //else {cout << "Could not transform from world frame to map frame!" << endl;}

        if(!worldToMap_flag) {

            cout << "ALERT- Could not transform from world frame to map frame!" << endl; 
            ros::Duration(5.0).sleep();

        }

        
        map_open_list[mx][my] = 1;
        
                
        q_m.push({mx, my});

        while(!q_m.empty()) {

            pair<unsigned int, unsigned int>  front = q_m.front();

            unsigned int cx = front.first , cy = front.second ; 

            //cout << "cx: " << cx  << " cy: " << cy << " q_m.size(): " << q_m.size() << endl;
            
            q_m.pop();

            if(map_close_list[cx][cy] != -1) {continue;}

            cout << "Calling is_frontier from the q_m loop!" << endl;
           
            if(is_frontier(cx, cy)) {

                //cout << "Frontier point detected!" << endl;

                queue<pair<size_t, size_t> >q_f;

                //vector<pair<size_t, size_t> > curr_frontier;

                vector<Point> curr_frontier;

                //curr_frontier.push_back({cx, cy});

                q_f.push({cx, cy});

                frontier_open_list[cx][cy] = 1;
                
                 while(!q_f.empty()) {
                    
                    pair<unsigned int, unsigned int> front_ = q_f.front();

                    unsigned int cx_ = front_.first, cy_ = front_.second;

                    //cout << "cx_: " << cx_ << " cy_: " << cy_ <<  " sz: " << q_f.size() << endl;

                    q_f.pop();

                    if(frontier_close_list[cx_][cy_] != -1 || map_close_list[cx_][cy_] != -1) {continue;}

                    cout << "Calling is_frontier from the q_f loop!" << endl;

                    if(is_frontier(cx_, cy_)) {
                        
                        Point frontier_point {cx_, cy_};

                        //cout << "Frontier found inside q_f!" << endl;

                        curr_frontier.push_back(frontier_point);
                        
                        for(int i = cx_ - 1; i <= cx_ + 1; i++) {
                            
                            for(int j = cy_ - 1; j <= cy_ + 1; j++) {
                                
                                if(i < 0 || j < 0 || i >= size_x || j >= size_y || (i == cx_ && j == cy_)) {continue;}

                                if(frontier_open_list[i][j] != -1 || frontier_close_list[i][j] != -1 || map_close_list[i][j] != -1 ) {continue;}

                                q_f.push({i,j});
                                frontier_open_list[i][j] =1;

                            }

                        }
                
                    }

                    frontier_close_list[cx_][cy_] = 1;

                }

                __uint32_t frontier_size = curr_frontier.size();

                if(frontier_size > 0) {

                    Point median_point = curr_frontier[(int)curr_frontier.size()/2];
                    Frontier frontier = {frontier_size, median_point, curr_frontier};
                    frontier_collection.push_back(frontier);

                }

                
            }

            int elems_added = 0 ;

            for(int i = (int)cx- 5 ; i < cx + 5; i++) {
                
                for(int j = (int)cy -5 ; j < cy + 5 ; j++) {
                    
                    if(i < 0 || j < 0 || i >= size_x || j >= size_y || (i == cx && j == cy)) {continue;}

                    if(map_open_list[i][j] == -1 && map_close_list[i][j] == -1 && has_free_cell_neighbour(i, j)) {
                        
                        elems_added++;
                        q_m.push({i,j}); 
                        map_open_list[i][j] = 1;

                    }

                }

            }
    
        
            cout << "elems_added: " << elems_added << endl;
            cout << "Finished the for loop! " << "q_m.size(): " << q_m.size()  << endl;

            cout << "Reached end of the q_m while loop" << endl;
            cout << "cx: " << cx << " cy: " << cy  << endl;

            map_close_list[cx][cy] = 1;
            
        }

        cout << "frontier_collection.size(): " << frontier_collection.size() << endl;
        
        cout << "END OF ROS::OK while loop! --- sleeping for 2 seconds!" << endl;
        
        ros::Duration(2.0).sleep(); 
        lock.unlock(); 

        cout << "Moving to the frontier medium!"  << endl;

        go_to_frontier_median();

        
        ros::spinOnce();
    
    }

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "explore_node");

    ros::NodeHandle nh("explore_node"); 
   
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    
    Explore* explore_one = new Explore(nh, buffer);

    explore_one->explore_level_four();

    return 0;
}   



