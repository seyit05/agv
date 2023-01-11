/*
 *
 *
 */

#include "sicks300B.h"

#include "termios.h"
#include <chrono>
#include <thread>

SickS300B::SickS300B() : param_node_("~"), nodeHandle_("/") {

  double x, y, z;

  // reading transformation parameters from parameter server
  param_node_.param(std::string("frame"), frame_id_, std::string("base_laser_link_B"));
  param_node_.param<bool>(std::string("send_transform"), send_transform_, 1);

  param_node_.param(std::string("tf_x"), x, -0.505);
  param_node_.param(std::string("tf_y"), y, -0.4);
  param_node_.param(std::string("tf_z"), z, 0.0);

  connect_cmd_ = param_node_.param<std::string>("connect_cmd", std::string());

  transform_vector_ = tf::Vector3(x, y, z);

  // Reduce field of view to this number of degrees
  double fov;
  param_node_.param(std::string("field_of_view"), fov, 265.0);
  if ((fov > 270) || (fov < 0)) {
    ROS_WARN("S300 field of view parameter set out of range (0-270). Assuming 270.");
    fov = 270.0;
  }
  field_of_view_ = (unsigned int) (fov * 2.0); // angle increment is .5 degrees
  field_of_view_ <<= 1;
  field_of_view_ >>= 1; // round to a multiple of two
  start_scan_ = 270 - field_of_view_ / 2;
  end_scan_ = 270 + field_of_view_ / 2;

  scan_data_.header.frame_id = frame_id_;
  scan_data_.angle_min = (float) (-(field_of_view_ / 4.0) / 180. * M_PI);
  scan_data_.angle_max = (float) ((field_of_view_ / 4.0) / 180. * M_PI);
  scan_data_.angle_increment = (float) (0.5 / 180. * M_PI);
  scan_data_.time_increment = 0;
  scan_data_.scan_time = 0.042;
  scan_data_.range_min = 0.1;
  scan_data_.range_max = 29.0;
  scan_data_.ranges.resize(field_of_view_);
  scan_data_.intensities.resize(field_of_view_);

  connected_ = -1;

  // Reading device parameters
  param_node_.param(std::string("devicename"), device_name_, std::string("/dev/arka_lidar"));
  baud_rate_ = (unsigned int) param_node_.param(std::string("baudrate"), 500000);

  scan_data_publisher_ = nodeHandle_.advertise<sensor_msgs::LaserScan>("laserscanB", 10);
}

SickS300B::~SickS300B() {
}

void SickS300B::update() {

  if (connected_ != 0) {
    if (!connect_cmd_.empty()) {
      ROS_INFO_STREAM("Executing connect cmd " << connect_cmd_ << "...");
      system(connect_cmd_.c_str());
    }
    ROS_INFO("Opening connection to Sick300-laser...");
    connected_ = serial_comm_.connect(device_name_, baud_rate_);
    if (connected_ == 0) {
      ROS_INFO("Sick300 connected.");
    } else {
      ROS_ERROR("Sick300 not connected, will try connecting again in 500 msec...");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  if (connected_ == 0) {
    int status = serial_comm_.readData();
    if (status == -2) {
      ROS_ERROR("Error in communication, closing connection and waiting 500 msec...");
      serial_comm_.disconnect();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      connected_ = -1;
    } else if (status == 0) { // packet successfully read
      float *ranges = serial_comm_.getRanges();
      unsigned int numRanges = serial_comm_.getNumRanges();

      for (unsigned int i = start_scan_, j = 0; i < end_scan_; i++, j++) {
        scan_data_.ranges[j] = ranges[i];
      }

      scan_data_.header.stamp = serial_comm_.getReceivedTime();
      unsigned int scanNum = serial_comm_.getScanNumber();
      ROS_DEBUG("ScanNum: %u", scanNum);

      scan_data_publisher_.publish(scan_data_);
    }
  }
}

void SickS300B::broadcast_transform() {
  if (send_transform_) {
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.383, -0.924, 0, 0), transform_vector_),
                                                       ros::Time::now(), "base_link", frame_id_));
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "SickS300B");
  ros::Time::init();
  //ros::Rate loop_rate(20); // not needed since blocking read is used

  SickS300B SickS300B;

  while (ros::ok()) {
    SickS300B.update();
    SickS300B.broadcast_transform();
    ros::spinOnce();
    //loop_rate.sleep();
  }

  ROS_INFO("Laser shut down.");

  return 0;
}

