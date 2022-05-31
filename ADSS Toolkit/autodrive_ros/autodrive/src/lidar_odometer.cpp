#include "lidar_odometry.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace lidar_odometry {

class LIDAR_Odometer : LIDAR_Odometry
{
public:

  LIDAR_Odometer();
  ~LIDAR_Odometer() = default;

  void process(const ros::TimerEvent &);
  void publish();

  bool setLaserPoseFromTf();

public:

  bool publish_tf, new_scan_available;

  double freq;

  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;

  ros::NodeHandle             n;
  sensor_msgs::LaserScan      last_scan;
  bool                        GT_pose_initialized;
  tf::TransformListener       tf_listener; // Do not put inside callback
  tf::TransformBroadcaster    odom_broadcaster;
  nav_msgs::Odometry          initial_robot_pose;

  // Subscriptions & publishers
  ros::Subscriber laser_sub, initPose_sub;
  ros::Publisher odom_pub;

  bool scan_available();

  // Callbacks
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& new_scan);
  void initPoseCallback(const nav_msgs::Odometry::ConstPtr& new_initPose);
};

LIDAR_Odometer::LIDAR_Odometer() :
  LIDAR_Odometry()
{
  //ROS_INFO("Initializing node...");

  // Parameters
  //----------------------------------------------------------------------------
  ros::NodeHandle pn("~");
  pn.param<std::string>("laser_scan_topic",laser_scan_topic,"/scan");
  pn.param<std::string>("odom_topic", odom_topic, "/odom");
  pn.param<std::string>("base_frame_id", base_frame_id, "/base");
  pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
  pn.param<bool>("publish_tf", publish_tf, true);
  pn.param<std::string>("init_pose_from_topic", init_pose_from_topic, "/base_pose_ground_truth");
  pn.param<double>("freq", freq, 10.0);
  pn.param<bool>("verbose", verbose, true);

  // Publishers and subscribers
  //----------------------------------------------------------------------------
  odom_pub  = pn.advertise<nav_msgs::Odometry>(odom_topic, 5);
  laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic,1,&LIDAR_Odometer::laserCallback,this);

  // Init pose
  if (init_pose_from_topic != "")
  {
    initPose_sub = n.subscribe<nav_msgs::Odometry>(init_pose_from_topic,1,&LIDAR_Odometer::initPoseCallback,this);
    GT_pose_initialized  = false;
  }
  else
  {
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 0;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  // Init variables
  module_initialized = false;
  first_laser_scan   = true;

  //ROS_INFO_STREAM("Listening to laserscan from topic: " << laser_sub.getTopic());
}

bool LIDAR_Odometer::setLaserPoseFromTf()
{
  bool retrieved = false;

  // Set laser pose on the robot (through tf)
  // This allows estimation of the odometry with respect to robot base
  tf::StampedTransform transform;
  transform.setIdentity();
  try
  {
    tf_listener.lookupTransform(base_frame_id, last_scan.header.frame_id, ros::Time(0), transform);
    retrieved = true;
  }
  catch (tf::TransformException &ex)
  {
    //ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    retrieved = false;
  }

  //TF:transform -> Eigen::Isometry3d

  const tf::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  setLaserPose(laser_tf);

  return retrieved;
}

bool LIDAR_Odometer::scan_available()
{
  return new_scan_available;
}

void LIDAR_Odometer::process(const ros::TimerEvent&)
{
  if( is_initialized() && scan_available() )
  {
    // Process odometry estimation
    odometryCalculation(last_scan);
    publish();
    // Avoid possibility of running algorithm twice on the same laser scan
    new_scan_available = false;
  }
  else
  {
    //ROS_WARN("Waiting for laser scan...") ;
  }
}

//------------------------------------------------------------------------------
//                           SUBSCRIBER CALLBACKS
//------------------------------------------------------------------------------

void LIDAR_Odometer::laserCallback(const sensor_msgs::LaserScan::ConstPtr& new_scan)
{
  if (GT_pose_initialized)
  {
    // Keep the last received laserscan in memory
    last_scan = *new_scan;
    current_scan_time = last_scan.header.stamp;

    // Initialize module on first scan
    if (!first_laser_scan)
    {
      // Copy laser scan to internal variable
      for (unsigned int i = 0; i<width; i++)
        range_wf(i) = new_scan->ranges[i];
      new_scan_available = true;
    }
    else
    {
      setLaserPoseFromTf();
      init(last_scan, initial_robot_pose.pose.pose);
      first_laser_scan = false;
    }
  }
}

void LIDAR_Odometer::initPoseCallback(const nav_msgs::Odometry::ConstPtr& new_initPose)
{
  // Initialize module on first GT pose.
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}

//------------------------------------------------------------------------------
//                           PUBLISHER FUNCTIONS
//------------------------------------------------------------------------------

void LIDAR_Odometer::publish()
{
  // Publish the odometry over tf
  //----------------------------------------------------------------------------
  if (publish_tf)
  {
    //ROS_DEBUG("Publishing transform: [base] to [odom]");
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = last_odom_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = robot_pose_.translation()(0);
    odom_trans.transform.translation.y = robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(lidar_odometry::getYaw(robot_pose_.rotation()));
    // Broadcast the transform
    odom_broadcaster.sendTransform(odom_trans);
  }

  // Publish the odometry message
  //----------------------------------------------------------------------------
  //ROS_DEBUG ("Publishing odometry: [odom topic]");
  nav_msgs::Odometry odom;
  odom.header.stamp = last_odom_time;
  odom.header.frame_id = odom_frame_id;
  // Set pose
  odom.pose.pose.position.x = robot_pose_.translation()(0);
  odom.pose.pose.position.y = robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(lidar_odometry::getYaw(robot_pose_.rotation()));
  // Set twist
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = lin_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = ang_speed;
  // Publish odometry message
  odom_pub.publish(odom);
}

} /* namespace lidar_odometry */

//------------------------------------------------------------------------------
//                                   MAIN
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_odometer");

  lidar_odometry::LIDAR_Odometer my_lidar_odometer;

  ros::TimerOptions timer_opt;
  timer_opt.oneshot   = false;
  timer_opt.autostart = true;
  timer_opt.callback_queue = ros::getGlobalCallbackQueue();
  timer_opt.tracked_object = ros::VoidConstPtr();
  timer_opt.callback = boost::bind(&lidar_odometry::LIDAR_Odometer::process, &my_lidar_odometer, _1);
  timer_opt.period   = ros::Rate(my_lidar_odometer.freq).expectedCycleTime();

  ros::Timer lidar_odometer_timer = ros::NodeHandle("~").createTimer(timer_opt);

  ros::spin();

  return EXIT_SUCCESS;
}
