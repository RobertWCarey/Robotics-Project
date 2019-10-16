#include <atomic>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <limits>
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
  std::vector<WorldPosition> NEquad, NWquad, SEquad, SWquad;
  WorldPosition findRandGoal(const std::vector<WorldPosition> worldPositions);
  void divideValidPos(const std::vector<WorldPosition> worldPositions);
  int quadNum = 0;
  void sendRandGoal(geometry_msgs::Pose2D pose2d, move_base_msgs::MoveBaseActionGoal actionGoal);
  OccupancyGrid occupancy_grid_;
  nav_msgs::OccupancyGrid map_{};
  cv::Mat map_image_{};
  std::atomic<bool> localised_{ true };
  std::atomic<bool> brick_found_{ true };
  bool findBrick(const cv::Mat image);
  bool moveToBrick(const cv::Mat image);
  double getPixPercent(const cv::Mat image);
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
  
  occupancy_grid_ = OccupancyGrid(map_, inflation_radius_);
  
  ROS_INFO("Print dat data bitch");
  
  int y = 0;
  GridPosition currGridPos;
  WorldPosition currWorldPos;
  for (int i = 0 ; i < map_.info.width*map_.info.height; i++)
  {
    int data = map_.data[i];
    if (data != -1 && data != 100)
    {
      // Store valid Grid Position
      currGridPos.x = i%map_.info.width;
      currGridPos.y = i/map_.info.width;
      validGridPos.push_back(currGridPos);

      // Store valid World Pos
      currWorldPos = occupancy_grid_.getWorldPosition(currGridPos);
      validWorldPos.push_back(currWorldPos);
    }
      
  }
        
  // Print sizes
  ROS_INFO("Number of Valid Grid Positions %d", validGridPos.size());
  ROS_INFO("Number of Valid World Positions %d", validWorldPos.size());
  divideValidPos(validWorldPos);
  
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

double BrickSearch::getPixPercent(const cv::Mat image)
{
  cv::Size imageSize = image.size();
  int32_t whitePixCnt = 0;
  cv::Vec3b tempVec;
  double whitePixPerc = 0.;

  int32_t imagePix = imageSize.height*imageSize.width;
  ROS_INFO_STREAM("x: "<<imageSize.width);
  ROS_INFO_STREAM("y: "<<imageSize.height);

  
  for (int16_t y =1; y < imageSize.height; y++)
  {
    for (int32_t x = 1 ; x < imageSize.width; x++)
    {
      tempVec = image.at<cv::Vec3b>(y,x);
            
      if (tempVec.val[1] > 1)
      {
        // ROS_INFO_STREAM("x: "<<x);
        // ROS_INFO_STREAM("y: "<<y);
        // ROS_INFO_STREAM("Value: "<<tempVec);
        whitePixCnt ++;        
      }
    }
  }

  whitePixPerc = whitePixCnt/(double)imagePix;

  ROS_INFO_STREAM("White Pixels: " << whitePixCnt);
  ROS_INFO_STREAM("% White Pixels: " << whitePixPerc);
  return whitePixPerc;
}

bool BrickSearch::findBrick(const cv::Mat image)
{
  const double redPixThres = 0.1;
  double redPixPerc = 0;
  ROS_INFO_STREAM("Locate Brick");

  redPixPerc = getPixPercent(image);

  // ROS_INFO_STREAM("White Pixels: " << whitePixCnt);
  ROS_INFO_STREAM("% Red Pixels: " << redPixPerc);
  cv::namedWindow( "findBrick Image", 0 );// Create a window for display.
  cv::imshow( "findBrick Image", image );
  cv::waitKey(0);

  if (redPixPerc > redPixThres)
  {
    return true;
  }

  return false;
}

bool BrickSearch::moveToBrick(const cv::Mat image)
{
  ROS_INFO_STREAM("Move To Brick");
  static int16_t segWidth = 640;
  static int16_t segHeight = 1080;

  // cv::Mat image_Left, image_Mid, image_Right;

  // image.colRange(0,639).copyTo(image_Left.colRange(0,639));
  // cv::Mat image_Left = image(cv::Rect(640,1080,1920,1080));
  cv::Mat image_Left = image(cv::Range(0,segHeight-1),cv::Range(0 ,segWidth-1));
  cv::Mat image_Mid = image(cv::Range(0,segHeight-1),cv::Range(segWidth ,(segWidth*2)-1));
  cv::Mat image_Right = image(cv::Range(0,segHeight-1),cv::Range((segWidth*2) ,(segWidth*3)-1));


  ROS_INFO_STREAM("Image Left %");
  getPixPercent(image_Left);
  ROS_INFO_STREAM("Image Mid %");
  getPixPercent(image_Mid);
  ROS_INFO_STREAM("Image Right %");
  getPixPercent(image_Right);

  cv::namedWindow( "Standard Image", 0 );// Create a window for display.
  cv::imshow( "Standard Image", image_Left );
  cv::namedWindow( "Standard Image1", 0 );// Create a window for display.
  cv::imshow( "Standard Image1", image_Mid );
  cv::namedWindow( "Standard Image2", 0 );// Create a window for display.
  cv::imshow( "Standard Image2", image_Right );
  cv::waitKey(0); 
  

  return false;
}

void BrickSearch::imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr)
{
  // Use this method to identify when the brick is visible
  static bool searchForBrick = true;

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
  cv::namedWindow( "Image", 0 );// Create a window for display.
  cv::imshow( "Image", image );
  cv::waitKey(0); 

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
  
  cv::namedWindow( "Standard Image", 0 );// Create a window for display.
  cv::imshow( "Standard Image", redImage );
  cv::waitKey(0);
  searchForBrick = findBrick(redImage);
  // !searchForBrick for FindBrick
  // if (!searchForBrick)
  // {
  //   brick_found_ = false;
  //   searchForBrick = findBrick(redImage);
  // }
  // else
  // {
  //   brick_found_ = true;
  //   move_base_action_client_.cancelAllGoals();
  //   searchForBrick = moveToBrick(redImage);
  // }
  
  
  // if (count/(double)redImagePix > 0.2)
  // {
  //   brick_found_ = true;
  //   ROS_INFO_STREAM("Brick Found");
    
  // }

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
  WorldPosition worldPos;
    ROS_INFO_STREAM("Current pose: " << pose2d);
  if (quadNum == 0)
  {
    // Generate Random Goal
    worldPos = findRandGoal(NEquad);
    ROS_INFO_STREAM("Selected from NE quad");
    quadNum++;
  }
  else if (quadNum == 1)
  {
    // Generate Random Goal
    worldPos = findRandGoal(SEquad);
    ROS_INFO_STREAM("Selected from SE quad");
    quadNum++;
  }
  else if (quadNum == 2)
  {
    // Generate Random Goal
    worldPos = findRandGoal(SWquad);
    ROS_INFO_STREAM("Selected from SW quad");
    quadNum++;
  }
  else
  {
    // Generate Random Goal
    worldPos = findRandGoal(NWquad);
    ROS_INFO_STREAM("Selected from NW quad");
    quadNum = 0;
  }

  // Update pose with random valid goal
  pose2d.x = worldPos.x;
  pose2d.y = worldPos.y;

  ROS_INFO_STREAM("Target pose: " << pose2d);

  actionGoal.goal.target_pose.pose = pose2dToPose(pose2d);

  ROS_INFO("Sending goal...");
  move_base_action_client_.sendGoal(actionGoal.goal);
}

void BrickSearch::divideValidPos(const std::vector<WorldPosition> worldPositions)
{
  WorldPosition maxPos, minPos, avePos;
  // Initialise max and min positions
  maxPos.x = 0.;
  maxPos.y = 0.;
  minPos.x = std::numeric_limits<double>::max();
  minPos.y = std::numeric_limits<double>::max();
  
  // Get max and min positions from valid positions
  for (auto pos : worldPositions)
  {
    if (pos.x > maxPos.x)
    {
      maxPos.x = pos.x;
    }
    if (pos.y > maxPos.y)
    {
      maxPos.y = pos.y;
    }
    if (pos.x < minPos.x)
    {
      minPos.x = pos.x;
    }
    if (pos.y < minPos.y)
    {
      minPos.y = pos.y;
    }
  }
  // Calculate average x and y positions
  avePos.x = (minPos.x + maxPos.x) / 2.;
  avePos.y = (minPos.y + maxPos.y) / 2.;


  for (auto pos : worldPositions)
  {
    if (pos.x >= avePos.x && pos.y >= avePos.y)
    {
      NEquad.push_back(pos);
    }
    else if (pos.x > avePos.x && pos.y < avePos.y)
    {
      SEquad.push_back(pos);
    }
    else if (pos.x <= avePos.x && pos.y <= avePos.y)
    {
      SWquad.push_back(pos);
    }
    else
    {
      NWquad.push_back(pos);
    }
    
  }

}

void BrickSearch::mainLoop()
{
  ROS_INFO("Localising...");
  // Variable to control localise method
  bool twisty = true;

  move_base_msgs::MoveBaseActionGoal action_goal{};
  action_goal.goal.target_pose.header.frame_id = "map";
  geometry_msgs::Pose2D pose_2d = getPose2d();

  // Localisation stage
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

    if (!brick_found_)
    {  
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
