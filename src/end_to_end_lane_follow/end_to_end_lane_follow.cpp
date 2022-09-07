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
#include "torch/torch.h"
#include "torch/script.h"

using namespace std;
using namespace cv;

torch::Device device(torch::kCUDA);
class LaneFollower{
    private:
    ros::NodeHandle n;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_sub_;
    ros::Publisher drive_pub;
    torch::jit::script::Module laneFollowModule;
    int camera_height, camera_width;
    public:
    LaneFollower():image_transport(n) {
        n = ros::NodeHandle("~");
        std::string path_to_package = ros::package::getPath("car");
        string model_folder = path_to_package + "/models/end_to_end_lane_follow/";
        n.getParam("/camera_height",camera_height);
        n.getParam("/camera_width",camera_width);
        std::string drive_topic, camera_topic, lane_follow_file;
        n.getParam("/camera_topic",camera_topic);
        image_sub_ = image_transport.subscribe(camera_topic, 1, &LaneFollower::image_callback, this);
        n.getParam("/nav_drive_topic", drive_topic);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        n.getParam("/lane_follow_file",lane_follow_file);
        try {
          laneFollowModule = torch::jit::load(model_folder + lane_follow_file);
          laneFollowModule.to(device);
          laneFollowModule.eval();
          }
          catch (const c10::Error& e) {
            ROS_ERROR("Error Loading Model: %s", e.what());
          }

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

        //PREPROCESSING STEP
        Mat resolutionReduced;
        cv::resize(imageInRGB,resolutionReduced,Size(camera_width,camera_height));
        //DATA FORMAT FOR INPUT TO NETWORK
        std::vector<torch::jit::IValue> inputs;
        torch::Tensor tensor_image = torch::from_blob(resolutionReduced.data, {1,resolutionReduced.rows, resolutionReduced.cols, resolutionReduced.channels() }, at::kByte).to(device).toType(c10::kFloat);
        tensor_image = tensor_image.permute({ 0,3,1,2 });
        inputs.push_back(tensor_image);
        //PREDICTION
        at::Tensor output = laneFollowModule.forward(inputs).toTensor();
        double steering_output = output[0].item().toDouble();
        steeringAngle = steering_output;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.steering_angle = steeringAngle;
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