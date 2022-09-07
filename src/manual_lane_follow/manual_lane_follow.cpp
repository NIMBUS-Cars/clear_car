#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <manual_lane_follow/steeringAngleFromPhoto.h>
using namespace std;
using namespace cv;

class LaneFollower{
    private:
    ros::NodeHandle n;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_sub_;
    ros::Publisher drive_pub;
    public:
    LaneFollower():image_transport(n) {
        n = ros::NodeHandle("~");
        std::string path_to_package = ros::package::getPath("car");
        string model_folder = path_to_package + "/models/end_to_end_lane_follow/";
        std::string drive_topic, camera_topic, lane_follow_file;
        n.getParam("/camera_topic",camera_topic);
        image_sub_ = image_transport.subscribe(camera_topic, 1, &LaneFollower::image_callback, this);
        n.getParam("/nav_drive_topic", drive_topic);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        n.getParam("/lane_follow_file",lane_follow_file);
    }
    ~LaneFollower()
  {
  }
    
  void image_callback(const sensor_msgs::ImageConstPtr& msg){
    double steeringAngle = 0;
    double carSpeed = 0;
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, "");
        Mat imageInRGB = cv_ptr->image;
        steeringAngle = get_steering_angle_from_mat(imageInRGB);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.steering_angle = steeringAngle*-1;
    drive_msg.speed = 0.8;
    ROS_INFO("Steering Angle %s",std::to_string(steeringAngle).c_str());
    ROS_INFO("Speed %s",std::to_string(carSpeed).c_str());
    drive_st_msg.drive = drive_msg;
    drive_pub.publish(drive_st_msg);
  }
};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "lane_follower");
    LaneFollower rw;
    ros::spin();
    return 0;
}