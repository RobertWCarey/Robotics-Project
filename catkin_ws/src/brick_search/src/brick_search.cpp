#include <atomic>
#include <cmath>
#include <cstdlib>
#include <vector>

#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "brick_search/occupancy_grid.h"

namespace
{
double wrapAngle(double angle)
{
  // Function to wrap an angle between 0 and 2*Pi
  while (angle < 0.)
  {
    angle += 2 * M_PI;
  }

  while (angle > (2 * M_PI))
  {
    angle -= 2 * M_PI;
  }

  return angle;
}

geometry_msgs::Pose pose2dToPose(const geometry_msgs::Pose2D& pose_2d)
{
  geometry_msgs::Pose pose{};

  pose.position.x = pose_2d.x;
  pose.position.y = pose_2d.y;

  pose.orientation.w = std::cos(pose_2d.theta);
  pose.orientation.z = std::sin(pose_2d.theta / 2.);

  return pose;
}
}  // namespace

namespace brick_search
{
class BrickSearch
{
public:
  // Constructor
  explicit BrickSearch(ros::NodeHandle& nh);

  // Publich methods
  void mainLoop();

private:
  // Variables
  OccupancyGrid occupancy_grid_;
  nav_msgs::OccupancyGrid map_{};
  cv::Mat map_image_{};
  std::atomic<bool> localised_{ false };
  std::atomic<bool> brick_found_{ false };
  int image_msg_count_ = 0;
  //Initialise map limit variables
  double map_x_min = 0., map_x_max = 0., map_y_min = 0., map_y_max = 0.;
  // Transform listener
  tf2_ros::Buffer transform_buffer_{};
  tf2_ros::TransformListener transform_listener_{ transform_buffer_ };
  double inflation_radius_ = 0.1;
  struct position
  {
    int x;
    int y;
  };

  // Subscribe to the AMCL pose to get covariance
  ros::Subscriber amcl_pose_sub_{};

  // Velocity command publisher
  ros::Publisher cmd_vel_pub_{};
  
  //Publish inflated map
  ros::Publisher inflated_map_pub_{};

  // Image transport and subscriber
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_{};

  // Action client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_{ "move_base", true };

  // Private methods
  geometry_msgs::Pose2D getPose2d();
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr);
};

// Constructor
BrickSearch::BrickSearch(ros::NodeHandle& nh) : it_{ nh }
{
  // Wait for "static_map" service to be available
  ROS_INFO("Waiting for \"static_map\" service...");
  ros::service::waitForService("static_map");

  // Get the map
  nav_msgs::GetMap get_map{};

  if (!ros::service::call("static_map", get_map))
  {
    ROS_ERROR("Unable to get map");
    ros::shutdown();
  }
  else
  {
    map_ = get_map.response.map;
    ROS_INFO("Map received");
  }

  // This allows you to access the map data as an OpenCV image
  // map_image_ = cv::Mat(map_.info.height, map_.info.width, CV_8U, &map_.data.front());
  
  // map_x_min = map_.info.origin.position.x;
  // map_x_max = map_.info.width * map_.info.resolution + map_x_min;

  // map_y_min = map_.info.origin.position.y;
  // map_y_max = map_.info.height * map_.info.resolution + map_y_min;
  // ROS_INFO("map x min, map x max, map y min, map y max");
  // ROS_INFO_STREAM(map_x_min);
  // ROS_INFO_STREAM(map_x_max);
  // ROS_INFO_STREAM(map_y_min);
  // ROS_INFO_STREAM(map_y_max);
  ROS_INFO("Print dat data bitch");
  std::vector<position> validpos;
  int y = 0;
  position currPos;
  for (int i = 0 ; i < map_.info.width*map_.info.height; i++)
  {
    int data = map_.data[i];
    if (data != -1 && data != 100)
    {
      currPos.x = i%map_.info.width;
      currPos.y = i/map_.info.width;
      validpos.push_back(currPos);
      ROS_INFO("Value %d", data);
    }
      
  }
  
  // Create an occupancy grid from the occupancy grid message
  occupancy_grid_ = OccupancyGrid(get_map.response.map, inflation_radius_);
  
  inflated_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map_inflated", 1, true);
  inflated_map_pub_.publish(occupancy_grid_.getOccupancyGridMsg());

  // Wait for the transform to be become available
  ROS_INFO("Waiting for transform from \"map\" to \"base_link\"");
  while (ros::ok() && !transform_buffer_.canTransform("map", "base_link", ros::Time(0.)))
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("Transform available");
  // Subscribe to "amcl_pose" to get pose covariance
  amcl_pose_sub_ = nh.subscribe("amcl_pose", 1, &BrickSearch::amclPoseCallback, this);

  // Subscribe to the camera
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &BrickSearch::imageCallback, this);

  // Advertise "cmd_vel" publisher to control TurtleBot manually
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);


  // Action client for "move_base"
  ROS_INFO("Waiting for \"move_base\" action...");
  move_base_action_client_.waitForServer();
  ROS_INFO("\"move_base\" action available");

  // Reinitialise AMCL
  ros::ServiceClient global_localization_service_client = nh.serviceClient<std_srvs::Empty>("global_localization");
  std_srvs::Empty srv{};
  global_localization_service_client.call(srv);
  std::cin.get();
}

geometry_msgs::Pose2D BrickSearch::getPose2d()
{
  // Lookup latest transform
  geometry_msgs::TransformStamped transform_stamped =
      transform_buffer_.lookupTransform("map", "base_link", ros::Time(0.), ros::Duration(0.2));

  // Return a Pose2D message
  geometry_msgs::Pose2D pose{};
  pose.x = transform_stamped.transform.translation.x;
  pose.y = transform_stamped.transform.translation.y;

  double qw = transform_stamped.transform.rotation.w;
  double qz = transform_stamped.transform.rotation.z;

  pose.theta = qz >= 0. ? wrapAngle(2. * std::acos(qw)) : wrapAngle(-2. * std::acos(qw));

  return pose;
}

void BrickSearch::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{
  // Check the covariance
  double frobenius_norm = 0.;

  for (const auto e : pose_msg.pose.covariance)
  {
    frobenius_norm += std::pow(e, 2.);
  }

  frobenius_norm = std::sqrt(frobenius_norm);

  if (frobenius_norm < 0.05)
  {
    localised_ = true;

    // Unsubscribe from "amcl_pose" because we should only need to localise once at start up
    amcl_pose_sub_.shutdown();
  }
}

void BrickSearch::imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr)
{
  // Use this method to identify when the brick is visible

  // The camera publishes at 30 fps, it's probably a good idea to analyse images at a lower rate than that
  if (image_msg_count_ < 15)
  {
    image_msg_count_++;
    return;
  }
  else
  {
    image_msg_count_ = 0;
  }

  // Copy the image message to a cv_bridge image pointer
  cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_msg_ptr);

  // This is the OpenCV image
  cv::Mat& image = image_ptr->image;

  // You can set "brick_found_" to true to signal to "mainLoop" that you have found a brick
  // You may want to communicate more information
  // Since the "imageCallback" and "mainLoop" methods can run at the same time you should protect any shared variables
  // with a mutex
  // "brick_found_" doesn't need a mutex because it's an atomic

  ROS_INFO("imageCallback");
  ROS_INFO_STREAM("brick_found_: " << brick_found_);
}

void BrickSearch::mainLoop()
{
  // Wait for the TurtleBot to localise
  ROS_INFO("PASE waiting for input");
  std::cin.get();

  ROS_INFO("Localising...");
  bool twisty = true;
  ROS_INFO_STREAM(localised_);

  while (ros::ok())
  {
    geometry_msgs::Pose2D localpose = getPose2d();
    geometry_msgs::Twist twist{};
    
    move_base_msgs::MoveBaseActionGoal local_goal{};
    local_goal.goal.target_pose.header.frame_id = "map";
    if(twisty)
    {
      ros::Time time = ros::Time::now();
      ros::Time newtime;
      ros::Duration d = ros::Duration(2, 0);
      ROS_INFO("Spinning bitch");
      while (ros::ok())
      {
        if ((newtime - time) > d)
        {
          break;
        }
        newtime = ros::Time::now();
        // Turn slowly
        twist.angular.z = 1.;
        cmd_vel_pub_.publish(twist);
      }
      ROS_INFO("Exit Spinning bitch");
    }
    else
    {
      ROS_INFO("Gonna move to a goal cunt");
      twist.angular.z = 0.;
      cmd_vel_pub_.publish(twist);
      localpose.x += 0.5 * std::cos(localpose.theta);
      localpose.y += 0.5 * std::sin(localpose.theta);      
      local_goal.goal.target_pose.pose = pose2dToPose(localpose);
      ros::Duration dExecute = ros::Duration(5, 0);
      ros::Duration dPreempt = ros::Duration(5, 0);
      actionlib::SimpleClientGoalState state = move_base_action_client_.sendGoalAndWait(local_goal.goal, dExecute, dPreempt);
      ROS_INFO_STREAM(state.getText());
    }
    //ros::shutdown();
    twisty = !twisty;
    if (localised_)
    {
      ROS_INFO("Localised");
      break;
    }

    ros::Duration(0.1).sleep();
  }

  // Stop turning
  geometry_msgs::Twist twist{};
  twist.angular.z = 0.;
  cmd_vel_pub_.publish(twist);

  // The map is stored in "map_"
  // You will probably need the data stored in "map_.info"
  // You can also access the map data as an OpenCV image with "map_image_"

  // Here's an example of getting the current pose and sending a goal to "move_base":
  geometry_msgs::Pose2D pose_2d = getPose2d();

  ROS_INFO_STREAM("Current pose: " << pose_2d);

  // Move forward 0.5 m
  pose_2d.x += 1. * std::cos(pose_2d.theta);
  pose_2d.y += 1. * std::sin(pose_2d.theta);

  ROS_INFO_STREAM("Target pose: " << pose_2d);

  // Send a goal to "move_base" with "move_base_action_client_"
  move_base_msgs::MoveBaseActionGoal action_goal{};

  action_goal.goal.target_pose.header.frame_id = "map";
  action_goal.goal.target_pose.pose = pose2dToPose(pose_2d);

  ROS_INFO("Sending goal...");
  move_base_action_client_.sendGoal(action_goal.goal);


  /* initialize random seed: */
  srand (time(NULL));
  
  double randomX, randomY;


  // This loop repeats until ROS shuts down, you probably want to put all your code in here
  while (ros::ok())
  {
    ROS_INFO("mainLoop");

    // Get the state of the goal
    actionlib::SimpleClientGoalState state = move_base_action_client_.getState();

    if ((state == actionlib::SimpleClientGoalState::PENDING) || (state == actionlib::SimpleClientGoalState::ACTIVE))
    {

    }
    else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      pose_2d = getPose2d();
      
      // Print the state of the goal
      ROS_INFO_STREAM(state.getText());

      ROS_INFO_STREAM("Current pose: " << pose_2d);

      /* generate secret number between 1 and 10: */
      randomX = (rand() % 20)/map_x_max;
      randomY = (rand() % 20)/map_y_max;
      // Move forward 0.5 m
      pose_2d.x = randomX;
      pose_2d.y = randomY;

      GridPosition posegrid = {(int)randomX, (int)randomY};
      ROS_INFO_STREAM(occupancy_grid_.isOccupied(posegrid));
      ROS_INFO_STREAM("Target pose: " << pose_2d);


      action_goal.goal.target_pose.pose = pose2dToPose(pose_2d);

      ROS_INFO("Sending goal...");
      move_base_action_client_.sendGoal(action_goal.goal);

      // Shutdown when done
      // ros::shutdown();
    }
    else
    {
      ROS_INFO("Error");
    }
    

    // Delay so the loop doesn't run too fast
    ros::Duration(0.2).sleep();
  }
}

}  // namespace brick_search

int main(int argc, char** argv)
{
  ros::init(argc, argv, "brick_search");

  ros::NodeHandle nh{};

  brick_search::BrickSearch bs(nh);

  // Asynchronous spinner doesn't block
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bs.mainLoop();

  return 0;
}
