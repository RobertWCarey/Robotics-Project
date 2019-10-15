#include <atomic>
#include <cmath>
#include <cstdlib>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>

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
  std::vector<GridPosition> validGridPos;
  std::vector<WorldPosition> validWorldPos;  
  double resolution = 0.05;
  WorldPosition findRandGoal(const std::vector<WorldPosition> worldPositions);
  void sendRandGoal(geometry_msgs::Pose2D pose2d, move_base_msgs::MoveBaseActionGoal actionGoal);
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
  /* initialize random seed: */
  srand (time(NULL));
  
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
  
  // load openCV image of map
  map_image_ = cv::Mat(map_.info.height,map_.info.width,CV_8U,&map_.data.front());
  cv::namedWindow( "Map", 0 );// Create a window for display.
  cv::imshow( "Map", map_image_ );
  // cv::waitKey(0);

  cv::Mat imageDilate , imageErosion;

  cv::dilate(map_image_,imageDilate,cv::Mat());
  cv::erode(map_image_,imageErosion,cv::Mat());

  cv::namedWindow( "Dilate", 0 );// Create a window for display.
  cv::imshow( "Dilate", imageDilate );

  cv::namedWindow( "Erode", 0 );// Create a window for display.
  cv::imshow( "Erode", imageErosion );

  

  occupancy_grid_ = OccupancyGrid(map_, inflation_radius_);
  
  ROS_INFO("Print dat data bitch");
  
  int y = 0;
  GridPosition currGridPos;
  WorldPosition currWorldPos;

  static cv::Vec3b tempVec;

  ROS_INFO_STREAM("X size of original map" << map_.info.width);
  ROS_INFO_STREAM("Y size of original map" << map_.info.height);
  ROS_INFO_STREAM("X size of map image" << map_image_.size().width);
  ROS_INFO_STREAM("Y size of map image" << map_image_.size().height);
  ROS_INFO_STREAM("X size of dilated map image" << imageDilate.size().width);
  ROS_INFO_STREAM("Y size of dilated map image" << imageDilate.size().height);
  cv::waitKey(0);
  for (int32_t y =0; y< imageDilate.size().height; y++)
  {
    for (int32_t x = 0 ; x< imageDilate.size().width; x++)
    {
      // ROS_INFO_STREAM("CurrentCount" << count);
      // ROS_INFO_STREAM("Value"<<redImage.at<cv::Vec3b>(y,x));
      tempVec = imageDilate.at<cv::Vec3b>(y,x);
      
      // ROS_INFO_STREAM("Y: "<<y);
      // ROS_INFO_STREAM("X: "<<x);
      // ROS_INFO_STREAM("Value of vector: " << tempVec);
      // ROS_INFO_STREAM(" ");
      int count = 0;
      // if (tempVec.val[1] == 0)
      for (int i = 0; i < 3; i++)
      {

        if (tempVec.val[i] == 0)
        {
          count++;
          if (count == 3)
          {
            ROS_INFO_STREAM("Y: "<<y);
            ROS_INFO_STREAM("X: "<<x);
            ROS_INFO_STREAM("Value of vector: " << tempVec);
          // ROS_INFO_STREAM("Value of vector: " << tempVec);
            currGridPos.x = x;
            currGridPos.y = y;
            validGridPos.push_back(currGridPos);

            // Store valid World Pos
            currWorldPos = occupancy_grid_.getWorldPosition(currGridPos);
            validWorldPos.push_back(currWorldPos); 
          } 
        }
      }
    }
  }
  // for (int i = 0 ; i < map_.info.width*map_.info.height; i++)
  // {
  //   int data = map_.data[i];
  //   if (data != -1 && data != 100)
  //   {
  //     // Store valid Grid Position
  //     currGridPos.x = i%map_.info.width;
  //     currGridPos.y = i/map_.info.width;
  //     validGridPos.push_back(currGridPos);

  //     // Store valid World Pos
  //     currWorldPos = occupancy_grid_.getWorldPosition(currGridPos);
  //     validWorldPos.push_back(currWorldPos);
  //   }
      
  // }
        
  // Print sizes
  ROS_INFO("Number of Valid Grid Positions %d", validGridPos.size());
  ROS_INFO("Number of Valid World Positions %d", validWorldPos.size());

  
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
  // std::cin.get();
}

WorldPosition BrickSearch::findRandGoal(const std::vector<WorldPosition> worldPositions)
{
  int numPos = worldPositions.size();
  WorldPosition randWorldPos;

  // generate random number between o and numPos-1
  int randArrPos = rand() % numPos;

  randWorldPos = worldPositions[randArrPos];

  ROS_INFO("X Pos %f", randWorldPos.x);
  ROS_INFO("Y Pos %f", randWorldPos.y);

  return randWorldPos;
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
  if (image_msg_count_ < 5)
  {
    image_msg_count_++;
    return;
  }
  else
  {
    image_msg_count_ = 0;
  }

  // Copy the image message to a cv_bridge image pointer
  cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_msg_ptr,"bgr8");

  // This is the OpenCV image
  cv::Mat& image = image_ptr->image;
  // cv::Mat hsvImage;
  // cv2::cvtColour(image,hsvImage,COLOR_BGR2HSV);
  // cv::namedWindow( "Standard Image", 0 );// Create a window for display.
  // cv::imshow( "Standard Image", image );
  // cv::waitKey(0); 

  // ROS_INFO(image);
  // ROS_INFO_STREAM(image.at<cv::Vec3b>(5,5));
  static cv::Scalar upperRed = cv::Scalar(1,1,255);
  static cv::Scalar lowerRed = cv::Scalar(0,0,100);

  static cv::Mat redImage;

  cv::inRange(image,lowerRed,upperRed,redImage);
  // cv::namedWindow( "Red Image", 0 );
  // cv::imshow("Red Image",redImage);
  // cv::waitKey(0);

  // Mask of image with only red pixels
  // std::cin.get();
  static cv::Size s = redImage.size();
  ROS_INFO_STREAM("RedImage Y: " << s.height);
  ROS_INFO_STREAM("RedImage X: " << s.width);
  static int32_t redImagePix = s.height*s.width;

  int32_t count = 0;

  // ROS_INFO_STREAM("Image" << tempVec);
  static cv::Vec3b tempVec;

  for (int32_t y =1; y< s.height; y++)
  {
    for (int32_t x = 1 ; x< s.width; x++)
    {
      // ROS_INFO_STREAM("CurrentCount" << count);
      // ROS_INFO_STREAM("Value"<<redImage.at<cv::Vec3b>(y,x));
      tempVec = redImage.at<cv::Vec3b>(y,x);
      
      // ROS_INFO_STREAM("Y: "<<y);
      // ROS_INFO_STREAM("X: "<<x);
      // ROS_INFO_STREAM("Value of vector: " << tempVec);
      // ROS_INFO_STREAM(" ");
      
      if (tempVec.val[1] > 1)
      {
        // ROS_INFO_STREAM("Value of vector: " << tempVec);
          count ++;        
      }
    }
  }

  ROS_INFO_STREAM("Final Count: " << count);
  ROS_INFO_STREAM("% Red Pixels: " << count/(double)redImagePix);
  if (count/(double)redImagePix > 0.2)
  {
    brick_found_ = true;
    ROS_INFO_STREAM("Brick Found");
    
  }

  ROS_INFO_STREAM("Number of Pixels: " << redImage.size());
  // cv::waitKey(0);
  // for (auto val : image)
  // {
  //   ROS_INFO_STREAM(val)
  // }

  // You can set "brick_found_" to true to signal to "mainLoop" that you have found a brick
  // You may want to communicate more information
  // Since the "imageCallback" and "mainLoop" methods can run at the same time you should protect any shared variables
  // with a mutex
  // "brick_found_" doesn't need a mutex because it's an atomic

  ROS_INFO("imageCallback");
  ROS_INFO_STREAM("brick_found_: " << brick_found_);
}

void BrickSearch::sendRandGoal(geometry_msgs::Pose2D pose2d, move_base_msgs::MoveBaseActionGoal actionGoal)
{
    ROS_INFO_STREAM("Current pose: " << pose2d);

  // Generate Random Goal
  WorldPosition worldPos = findRandGoal(validWorldPos);

  // Update pose with random valid goal
  pose2d.x = worldPos.x;
  pose2d.y = worldPos.y;

  ROS_INFO_STREAM("Target pose: " << pose2d);

  actionGoal.goal.target_pose.pose = pose2dToPose(pose2d);

  ROS_INFO("Sending goal...");
  move_base_action_client_.sendGoal(actionGoal.goal);
}

void BrickSearch::mainLoop()
{
  ROS_INFO("Localising...");
  // Variable to control localise method
  bool twisty = true;

  move_base_msgs::MoveBaseActionGoal action_goal{};
  action_goal.goal.target_pose.header.frame_id = "map";
  geometry_msgs::Pose2D pose_2d = getPose2d();

  while (ros::ok())
  {
    
    
    static geometry_msgs::Twist twist{};
    

    if(twisty)
    {
      ros::Time time = ros::Time::now();
      ros::Time newtime;
      ros::Duration d = ros::Duration(6, 0);
      ROS_INFO("Spin to Localise");
      
      // Remain in loop spinning until duration elasped
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
    }
    else
    {
      ROS_INFO("Move to a Goal to localise");

      // Stop spinning
      twist.angular.z = 0.;
      cmd_vel_pub_.publish(twist);

      // Send move goal to 0.5 m infront of current posistion
      pose_2d.x += 0.5 * std::cos(pose_2d.theta);
      pose_2d.y += 0.5 * std::sin(pose_2d.theta);      
      action_goal.goal.target_pose.pose = pose2dToPose(pose_2d);

      // time out after 10sec total with preempt and execute
      ros::Duration timeoutDur = ros::Duration(5, 0);
      // only exits after timeout or success
      actionlib::SimpleClientGoalState state = move_base_action_client_.sendGoalAndWait(action_goal.goal, timeoutDur, timeoutDur);
      ROS_INFO_STREAM(state.getText());
    }
    // toggle localise method
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

  // Cancel all goals
  move_base_action_client_.cancelAllGoals();


  // The map is stored in "map_"
  // You will probably need the data stored in "map_.info"
  // You can also access the map data as an OpenCV image with "map_image_"

  // Here's an example of getting the current pose and sending a goal to "move_base":
  pose_2d = getPose2d();

  // ROS_INFO_STREAM("Current pose: " << pose_2d);

  // Move forward 0.5 m
  // pose_2d.x += 1. * std::cos(pose_2d.theta);
  // pose_2d.y += 1. * std::sin(pose_2d.theta);

  // ROS_INFO_STREAM("Target pose: " << pose_2d);

  // Send a goal to "move_base" with "move_base_action_client_"
  // move_base_msgs::MoveBaseActionGoal action_goal{};

  // action_goal.goal.target_pose.header.frame_id = "map";
  // action_goal.goal.target_pose.pose = pose2dToPose(pose_2d);

  // ROS_INFO("Sending goal...");
  // move_base_action_client_.sendGoal(action_goal.goal);


  


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
      sendRandGoal(getPose2d(), action_goal);
    }
    else
    {
      ROS_INFO("Error");

      sendRandGoal(getPose2d(), action_goal);
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
