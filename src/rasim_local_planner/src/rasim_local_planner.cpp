#include <pluginlib/class_list_macros.h>
#include <laser_geometry/laser_geometry.h>
#include "rasim_local_planner.h"
#include <base_local_planner/goal_functions.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//register this planner as a BaseLocalPlanner plugin


namespace rasim_local_planner {

    // GLOBAL VARIABLES
  ros::Publisher point_cloud_publisher_;
  double c_x , c_y;
	double obstacleDistance;
	double theta;
  ros::Publisher marker_pub1 ;
	bool goal_avail = false;
	sensor_msgs::PointCloud cloud;
	boost::shared_ptr<tf::TransformListener> listener_;
	geometry_msgs::PoseStamped goal_pose;
	geometry_msgs::PoseStamped base_goal_pose;
	ros::Publisher cmd_pub;
	ros::Subscriber cmd_sub;
	ros::Subscriber goal_sub;
	geometry_msgs::Twist msg1;
	struct F
	{
  		double x,y;
	};



  void DummyLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      
      prune_plan_ = false;
      xy_goal_tolerance_ = 0.1;
      yaw_goal_tolerance_ = 6.28;
      rot_stopped_vel_ = 1e-2;
      trans_stopped_vel_ = 1e-2;

      tf_ = tf;

      costmap_ros_ = costmap_ros;

      ros::NodeHandle pn("~/" + name);

      //ASK THE DIFFERENCE BETWEEN PN AND GN NODEHANDLERS
      //to get odometry information, we need to get a handle to the topic in the global namespace
      ros::NodeHandle gn;

      cmd_sub = gn.subscribe<sensor_msgs::LaserScan>("/scan", 1, boost::bind(&DummyLocalPlanner::scanCallback , this, _1));
 	    point_cloud_publisher_ = gn.advertise<sensor_msgs::PointCloud> ("/cloud", 1, false);
      cmd_pub = gn.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
      odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&DummyLocalPlanner::odomCallback, this, _1));
      //goal_sub = gn.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&DummyLocalPlanner::goalCallback , this, _1));
      marker_pub1 = gn.advertise<visualization_msgs::MarkerArray>("/visualization_markers", 1);

      initialized_ = true;
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }



	void DummyLocalPlanner::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		laser_geometry::LaserProjection projector_;
	    tf::TransformListener listener;	    
		

		if(!listener.waitForTransform(scan_in->header.frame_id,"/base_link",scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
          ros::Duration(1.0))) return;

   	    projector_.transformLaserScanToPointCloud("/base_link",*scan_in, cloud,listener);

	}




  void DummyLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG_NAMED("rasim_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
    /*ROS_INFO("rasim_local_planner In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);*/
  }



 bool DummyLocalPlanner::GoalAvail()
 {
    tf::Stamped<tf::Pose> goal_point;
    geometry_msgs::PoseStamped global_pose;

    if(!costmap_ros_->getRobotPose(global_pose))return false;

    costmap_2d::Costmap2D *costmap;
    costmap = costmap_ros_->getCostmap();
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
      //std::vector<geometry_msgs::PoseStamped> transformed_plan;
      //get the global plan in our frame
    if(!base_local_planner::transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap, costmap_ros_->getGlobalFrameID(), transformed_plan)){
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
      }

     std::string str = costmap_ros_->getGlobalFrameID();
     ROS_INFO("Frame ID: %s" , str);


    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    goal_x = goal_point.getOrigin().getX();
    goal_y = goal_point.getOrigin().getY();

    ROS_INFO("Goal: %f , %f" , goal_x,goal_y);

    return true;
 }

  bool DummyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    /*
    ROS_INFO("\n-------------------------------------------------------------------------------");
    ROS_INFO("Goal is available!!!!");
    ROS_INFO("\n-------------------------------------------------------------------------------");

  
    */


      GoalAvail();

      visualization_msgs::MarkerArray markers; //Marker array to store arrows representing our forces.
      uint32_t shape = visualization_msgs::Marker::ARROW; // shape of the posted markers.
      F F_attractive;
      double K_att = 600;
      double goalDistance = sqrt(pow(goal_x,2)+pow(goal_y,2));
      F_attractive.x = K_att * (goal_x / goalDistance);
      F_attractive.y = K_att * (goal_y / goalDistance);


      // ATTRACTIVE FORCE MARKERS
      //////////////////////////////////////////////////////////////////////////////////////////////
      visualization_msgs::Marker Attractive_Marker;
      geometry_msgs::Point p1;

      Attractive_Marker.header.frame_id = "/base_link"; //marker in robot frame
      Attractive_Marker.header.stamp = ros::Time::now();

      Attractive_Marker.ns = "attractive_force"; // unique namespace for the arrow showing attractive force vector
      Attractive_Marker.id = 0;

      Attractive_Marker.type = shape; //predefined arrow in "shape" variable is passed to the marker type
      Attractive_Marker.action = visualization_msgs::Marker::ADD;


      //Start point of arrow marker.
      p1.x = 0;
      p1.y = 0;
      p1.z = 0;
      Attractive_Marker.points.push_back(p1);
      //tip of arrow marker.
      p1.x = F_attractive.x/500;
      p1.y = F_attractive.y/500;
      p1.z = 0;

      Attractive_Marker.points.push_back(p1);
      Attractive_Marker.pose.orientation.x = 0.0;
      Attractive_Marker.pose.orientation.y = 0.0;
      Attractive_Marker.pose.orientation.z = 0.0;
      Attractive_Marker.pose.orientation.w = 1.0;

      Attractive_Marker.scale.x = 0.1;
      Attractive_Marker.scale.y = 0.1;
      Attractive_Marker.scale.z = 0.1;

      Attractive_Marker.color.r = 0.0f;
      Attractive_Marker.color.g = 1.0f;
      Attractive_Marker.color.b = 0.0f;
      Attractive_Marker.color.a = 1.0;

      Attractive_Marker.lifetime = ros::Duration();

      //////////////////////////////////////////////////////////////////////////////////////////////





      ROS_INFO("\n-------------------------------------------------------------------------------");
      ROS_INFO("ATTARCTIVE FORCE: %f %f" ,F_attractive.x,F_attractive.y);
      ROS_INFO("\n-------------------------------------------------------------------------------");

      ROS_INFO("\n-------------------------------------------------------------------------------");
      ROS_INFO("Goal distance: %f" , goalDistance);
      ROS_INFO("\n-------------------------------------------------------------------------------");


      F F_repulsive;
      double K_rep = 2.7;
      F_repulsive.x = 0;
      F_repulsive.y = 0;
      size_t total_points = cloud.points.size();
      int t_p = total_points;


      for(int i = 0; i < t_p ; i++)
  		{
  		    c_x = (-1)*cloud.points[i].x;// setting opposite of obstacle points, as they should attract, which in fact makes repulsion.
  		    c_y = (-1)*cloud.points[i].y;
  		    obstacleDistance = pow(sqrt(c_x*c_x+c_y*c_y),3);//osbtacle distance in the third power,to make it more sensitive to distance from osbtacle. 
  		    F_repulsive.x = F_repulsive.x + K_rep * (c_x / obstacleDistance);
  		    F_repulsive.y = F_repulsive.y + K_rep * (c_y / obstacleDistance); 
  		}

      ROS_INFO("\n-------------------------------------------------------------------------------");
      ROS_INFO("REPULSIVE: %f %f" ,F_repulsive.x,F_repulsive.y);
      ROS_INFO("\n-------------------------------------------------------------------------------");

  	  F F_resultant;
      F_resultant.x = F_attractive.x + F_repulsive.x;
      F_resultant.y = F_attractive.y + F_repulsive.y; 


      ROS_INFO("\n-------------------------------------------------------------------------------");
      ROS_INFO("RESULTANT: %f %f" ,F_resultant.x,F_resultant.y);
      ROS_INFO("\n-------------------------------------------------------------------------------");

      //obtaining angle between y and resultant force to find orientation.
      double theta = atan2(F_resultant.y,F_resultant.x);

      ROS_INFO("\n-------------------------------------------------------------------------------");
      ROS_INFO("obstacleDistance: %f" , obstacleDistance);
      ROS_INFO("\n-------------------------------------------------------------------------------");

    /*
      if(base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_){

        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        return true;
      }*/

      if (abs(obstacleDistance) <= 0.8 || abs(obstacleDistance) > 0.26)
      {
        cmd_vel.linear.x = 0.2;
        cmd_vel.angular.z = 0.7*theta;
      }
      else
      {
      cmd_vel.linear.x = 0.27;
      cmd_vel.angular.z = 1.5*theta;
      }
      /////////////////////////////////////////////////////////////////////////////////////////////////
      ROS_INFO("-------------------------ANGULAR VELOCITY--------------------------------------");
      ROS_INFO("Angular:%f Linear:%f" , cmd_vel.angular.z , cmd_vel.linear.x);
      ROS_INFO("-------------------------------------------------------------------------------");

      
      markers.markers.push_back(Attractive_Marker);
      marker_pub1.publish(markers);
      cmd_pub.publish(cmd_vel);
  
    return true;
  }



   bool DummyLocalPlanner::isGoalReached(){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    geometry_msgs::PoseStamped global_pose;
    if(!costmap_ros_->getRobotPose(global_pose)){
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_mutex_);
      base_odom = base_odom_;
    }

    return base_local_planner::isGoalReached(*tf_, global_plan_, *(costmap_ros_->getCostmap()), costmap_ros_->getGlobalFrameID(), global_pose, base_odom, 
        rot_stopped_vel_, trans_stopped_vel_, xy_goal_tolerance_, yaw_goal_tolerance_);
  }




  bool DummyLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;


    return true;
  }

};


PLUGINLIB_EXPORT_CLASS(rasim_local_planner::DummyLocalPlanner, nav_core::BaseLocalPlanner)