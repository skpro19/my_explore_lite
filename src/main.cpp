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

costmap_2d::Costmap2D my_costmap;

//define global variables

int size_x = 4000; 
int size_y = 4000;

using namespace std;

int v[256]; //Do not delete this

int outer_m[1010][1010] , inner_m[1010][1010];

int vis[4010][4010], frontier_vis[4010][4010] ; 

costmap_2d::Costmap2D* temp_global_costmap_;

pair<unsigned int, unsigned int>  bot_pose_in_world_frame(costmap_2d::Costmap2DROS *global_costmap) {

    geometry_msgs::PoseStamped global_pose; 

    bool pos = global_costmap->getRobotPose(global_pose) ;

    if(!pos) {

        cout << "Unable to detect global pose of the bot!" << endl;
        cout << "ABORTING"  << endl; 
    }

    double x = global_pose.pose.position.x; 
    double y = global_pose.pose.position.y; 

    cout << "x: " << x << " y:" << y << endl; 

    costmap_2d::Costmap2D* global_costmap_ = global_costmap->getCostmap();  

    unsigned int mx, my; 
    global_costmap_->worldToMap(x , y, mx , my);


    cout << "mx: " << mx << " my: " << my << endl;

    pair<unsigned int, unsigned int> p = {mx, my};

    return p; 

}

void go_to_free_cell(int wx, int wy, string frame_id = "odom") {

    cout << "Inside the go_to_free_cell function" << endl;

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

void go_to_cell(unsigned int mx, unsigned int my, costmap_2d::Costmap2D* &global_costmap_) {

    //cout << "mx: " << mx << " my: " << my << endl;

    double wx, wy; 

    global_costmap_->mapToWorld(mx, my, wx, wy);
    
    //cout << "Inside the go_to_cell function!" << endl; 


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


bool is_frontier(unsigned int mx, unsigned int my, costmap_2d::Costmap2D* &global_costmap_) {

    unsigned char* og = global_costmap_->getCharMap();

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

 
void publish_markers(ros::NodeHandle &nh, costmap_2d::Costmap2D* &global_costmap_, double wx, double wy) {

    cout << "INSIDE THE PUBLISH_MARKERS FUNCTION! " << endl;

    

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    cout << "namespace: " << nh.getNamespace() << endl;

    marker.ns = nh.getNamespace();

    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

 
    marker.pose.position.x = wx + 1;
    marker.pose.position.y = wy + 3;
    marker.pose.position.z = 1; 
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.lifetime = ros::Duration(1.0);

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
    ros::Duration del(10.0);
    ros::Time end = start + del ;

    cout << "Loop starts at: " << ros::Time::now().toSec() << endl;
    int id = 0;
    while(ros::Time::now() < start + del) {
        
        
        ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_markers", 10 );
        //marker.id = ++id;
        vis_pub.publish(marker);
        //ros::Duration(0.5).sleep();
        //cout << "Current time: " << ros::Time::now().toSec() << endl;
        
        
    }

    cout << "Loop ends at: " << ros::Time::now().toSec() << endl;
   

}


void publish_marker_array(ros::NodeHandle &nh, vector<pair<size_t, size_t> >&frontiers, costmap_2d::Costmap2D* &global_costmap_) {

    cout << "INSIDE THE PUBLISH_MARKERS FUNCTION! " << endl;

    size_t  sz = frontiers.size();

    visualization_msgs::MarkerArray frontier_marker_array;

    for(int i =0 ;i  < sz ; i++){

        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();

        cout << "namespace: " << nh.getNamespace() << endl;

        marker.ns = nh.getNamespace();

        marker.id = 1;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;


        marker.pose.position.x =  2000;
        marker.pose.position.y =   2000;
        marker.pose.position.z = 1; 
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = ros::Duration(1.0);

        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1.0;

        marker.color.a = 1.0; // Don't forget to set the alpha!

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;


        frontier_marker_array.markers.push_back(marker);

    }





    ros::Time start = ros::Time::now(); 
    ros::Duration del(20.0);
    ros::Time end = start + del ;

    cout << "Loop starts at: " << ros::Time::now().toSec() << endl;

    while(ros::Time::now() < start + del) {


        ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_markers", 10 );
        //marker.id = ++id;
        vis_pub.publish(frontier_marker_array);
        //ros::Duration(0.5).sleep();
        //cout << "Current time: " << ros::Time::now().toSec() << endl;


    }

    cout << "Loop ends at: " << ros::Time::now().toSec() << endl;


}

void explore_level_three(costmap_2d::Costmap2DROS* &global_costmap, ros::NodeHandle &nh) {

    ros::Rate loop_rate(0.1);

    costmap_2d::Costmap2D* global_costmap_ = global_costmap->getCostmap();

    int free_cell_cnt =0 ; 

    vector<pair<size_t, size_t> > frontiers;

    memset(vis, -1, sizeof(vis)) ;
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

                    else if(cell_cost == costmap_2d::NO_INFORMATION && is_frontier(i, j, global_costmap_)) {
                        
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

            cout << "current location: (" << mx << "," << ")" << endl;

            publish_marker_array(nh, frontiers,global_costmap_);

            cout << "Out of the publish marker array function" << endl;
            cout << "Attempting to move to a frontier point - (" << target.first <<"," << target.second << ")" << endl;
            
            go_to_cell(target.first, target.second, global_costmap_);

        }

        lock.unlock();

        ros::spinOnce();
        
        //loop_rate.sleep();

    }


}

void explore_level_two(costmap_2d::Costmap2DROS *global_costmap) {
    
    costmap_2d::Costmap2D* global_costmap_ = global_costmap->getCostmap();  
    
    ros::Rate loop_rate(0.1);

    while(ros::ok()){

        pair<unsigned int, unsigned int> p = bot_pose_in_world_frame(global_costmap); 
        
        unsigned int mx = p.first , my = p.second ; 

        unsigned char* global_og = global_costmap_->getCharMap();

        unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getMutex()));
        bool frontier_found =  0; 


        pair<int, int> last_found_free_cell = {-1, -1};

        for(int i = 0; i < size_x ; i++) {

            for(int j = 0 ; j < size_y; j++) {

                if (is_frontier(i, j, global_costmap_)) {
                    
                    frontier_found = 1;
                    go_to_cell(i, j, global_costmap_);
                    break;

                }

                else {

                    if(global_costmap_->getCost(i, j) == costmap_2d::FREE_SPACE) {

                        last_found_free_cell = {i ,j};

                    }
                }
            }
        }

        if(!frontier_found) {   go_to_cell(last_found_free_cell.first, last_found_free_cell.second, global_costmap_); }


        lock.unlock();

        loop_rate.sleep();
        
        ros::spinOnce();

    }
}


void explore_level_one(costmap_2d::Costmap2DROS *global_costmap) {

    
    costmap_2d::Costmap2D* global_costmap_ = global_costmap->getCostmap();  

    
    while(ros::ok()){

         //Time Calculations for the while loop 
        
        double begin = ros::Time::now().toSec();

        cout << "begin: " << begin << endl;

        
        cout << "Inside the while loop!" << endl;
        double now = ros::Time::now().toSec();
        cout << "now: " << now << endl;
        cout <<"diff: " << now- begin << endl; 

        begin = now;

        /// End of time calculations


        pair<unsigned int, unsigned int> p = bot_pose_in_world_frame(global_costmap); 
        
        unsigned int mx = p.first , my = p.second ; 

        int frontier_size = 25; 

       // ros::Rate r(0.1) ; 

        unsigned char* global_og = global_costmap_->getCharMap();

        unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getMutex()));


        bool free_cell_found =  0; 

        for(int i = (int)mx - 50; i < mx + 50; i++) {

            if (free_cell_found) {

                cout << "free cell found!" << endl; 
                break;
            }

            for(int j = (int)my - 50; j < my + 50; j++) {
                


                if(i < 0 || j < 0) {
                    
                    cout << "i: " << i << " j: " << j << endl;
                    cout << "Reached negative indices" << endl; 
                    continue;

                }
             
                int idx = global_costmap_->getIndex(i, j);
                
                unsigned char cell_cost = global_costmap_->getCost(i,j);

                if (cell_cost == costmap_2d::FREE_SPACE) {

                    free_cell_found = 1; 
                    global_og[idx] = costmap_2d::LETHAL_OBSTACLE;
                    
                    double wx, wy; 

                    global_costmap_->mapToWorld(i, j, wx, wy) ;

                    cout << "wx: " << wx << " wy: " << wy << endl;

                    go_to_free_cell(wx, wy);



                }


            }

        }

        lock.unlock();

        //r.sleep();
        
        //ros::Duration(5.0).sleep();
        ros::spinOnce();

    }




}



void detect_initial_frontiers(costmap_2d::Costmap2DROS* global_costmap) {

    geometry_msgs::PoseStamped global_pose;


    bool pos = global_costmap->getRobotPose(global_pose);


    if(!pos) {

        cout << "Unable to detect global pose of the bot!" << endl; 
        cout << "ABORTING!" << endl;

    }

    double x = global_pose.pose.position.x; 
    double y = global_pose.pose.position.y; 

    cout << "x: " << x << " y: " << y << endl; 

    costmap_2d::Costmap2D* global_costmap_ = global_costmap->getCostmap();  


    unsigned int mx, my; 
    global_costmap_->worldToMap(x , y, mx , my);


    cout << "mx: " << mx << " my: " << my << endl;

    ros::Rate r(5); 

    unsigned char* global_og = global_costmap_->getCharMap();

    while(ros::ok()){

        unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getMutex()));

        for(int i = mx - 50; i < mx + 50; i++) {

            for(int j = my - 50; j < my + 50; j++) {


                int idx = global_costmap_->getIndex(i, j);
                global_og[idx] = costmap_2d::LETHAL_OBSTACLE; 


            }

        }
        
        ros::Duration(5.0).sleep();
        
        lock.unlock();

        //r.sleep();
        ros::spinOnce();

    }


    


}


void manipulate_center_og(costmap_2d::Costmap2D* global_costmap_) {

    unsigned char* global_og = global_costmap_->getCharMap();

    size_t size_x = global_costmap_->getSizeInCellsX();
    size_t size_y = global_costmap_->getSizeInCellsY(); 

    ros::Rate r(5);



    while (ros::ok()) {

        unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getMutex()));

        memset(v, 0, sizeof(v));

        for(int i = size_x/2 - 100; i < size_x/2 + 100; i++) {

            for(int j= size_y/2 - 100; j < size_y/2 + 100; j++) {

                int idx = global_costmap_->getIndex(i, j); 

                //v[global_og[idx]]++;    

                global_og[idx] = costmap_2d::LETHAL_OBSTACLE;    

            }
        }

        lock.unlock();

        r.sleep();
        
        

        ros::spinOnce(); 


    }

}


void manipulate_og(costmap_2d::Costmap2D* global_costmap_) {

    unsigned char* global_og = global_costmap_->getCharMap();

    size_t size_x = global_costmap_->getSizeInCellsX();
    size_t size_y = global_costmap_->getSizeInCellsY(); 

    ros::Rate r(5);



    while (ros::ok()) {

        unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getMutex()));

        memset(v, 0, sizeof(v));

        for(int i = 0; i < size_x; i++) {

            for(int j= 0; j < size_y; j++) {

                int idx = global_costmap_->getIndex(i, j); 

                v[global_og[idx]]++;        

            }
        }

        for(int i = 0; i < 256; i++) {

            if(v[i] > 0) {
             
                cout << i << " " << v[i] << endl;
            
            }
        }

        lock.unlock();


        ros::spinOnce(); 

        r.sleep();

    }

}

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

void initialpose_callback2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    std::cout << "Pose received"  << std::endl; 

    geometry_msgs::Pose target_pose = msg->pose.pose;

    //costmap_2d::Costmap2D* global_costmap_ = temp_global_costmap_->getCostmap();

    double wx = target_pose.position.x , wy = target_pose.position.y; 

    unsigned int mx , my; 

    temp_global_costmap_->worldToMap(wx, wy, mx, my);

    unsigned char cell_cost = temp_global_costmap_->getCost(mx, my) ;


    cout << "cell_cost: " << cell_cost << " " << (int)cell_cost << endl;

}



int main(int argc, char** argv) {

    ros::init(argc, argv, "explore_node");

    ros::NodeHandle nh("explore_node"); 

    //ros::NodeHandle nh{};

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    
    string global_frame, robot_base_frame;  

    nh.param("global_costmap/global_frame", global_frame, std::string("map"));
    nh.param("global_costmap/robot_base_frame", robot_base_frame, std::string("base_link"));

    costmap_2d::Costmap2DROS *global_costmap = new costmap_2d::Costmap2DROS("global_costmap", buffer);    
    costmap_2d::Costmap2DROS *local_costmap = new costmap_2d::Costmap2DROS("local_costmap", buffer);


    cout << "global_frame: " << global_frame << endl; 
    cout << "robot_base_frame: " << robot_base_frame << endl;



    costmap_2d::Costmap2D* global_costmap_ = temp_global_costmap_ =  global_costmap->getCostmap();  


    unsigned char* global_og = global_costmap_->getCharMap();
    

    
    //unsigned char* global_og = global_costmap->getCostmap()->getCharMap();

    size_t size_x = global_costmap_->getSizeInCellsX();
    size_t size_y = global_costmap_->getSizeInCellsY(); 

    cout << "size_x: " << size_x << " size_y: " << size_y << endl;

    size_t idx = global_costmap_->getIndex(10, 10);

    cout << "idx: " << idx << endl;

    cout <<"global_og[idx]: " << (int)global_og[idx] << endl;
    
    //
    cout << "FREE_SPACE: " << costmap_2d::FREE_SPACE << " " << (int)costmap_2d::FREE_SPACE << endl; 

    cout << "NO_INFORMATION: " << costmap_2d::NO_INFORMATION << " " << (int)costmap_2d::NO_INFORMATION << endl;


    //ros::Subscriber sub = nh.subscribe("/initialpose", 1000, initialpose_callback2);

        geometry_msgs::PoseStamped global_pose; 

        bool flag = global_costmap->getRobotPose(global_pose); 

        if(flag) {cout << "global pose found successfully!" << endl;}

        double wx = global_pose.pose.position.x , wy = global_pose.pose.position.y;

    cout << "wx: " << wx << " wy: " << wy << endl;

    //publish_markers(nh, global_costmap_,  wx, wy);

    explore_level_three(global_costmap, nh);  


    //ros::spin();

    //cout << 1 << " " << 2;


    return 0;
}   



