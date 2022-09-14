#include <ros/ros.h>
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

// MODULARIZE THIS LATER
using namespace std;
using namespace cv;
torch::Device device(torch::kCUDA);

class LaneFollower{
    private:
    ros::NodeHandle n;

    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    ros::Publisher drive_pub;

    torch::jit::script::Module semanticSegmentationModule;
    torch::jit::script::Module laneFollowModule;
    torch::jit::script::Module speedControllerModule;

    int camera_height,camera_width;
    int desiredX = 256,desiredY=256;
    int outputX = 1280, outputY = 720;

    public:
    LaneFollower():image_transport(n) {
        n = ros::NodeHandle("~");
        n.getParam("/camera_height",camera_height);
        n.getParam("/camera_width",camera_width);
        std::string drive_topic, camera_topic, semanticSegmentationPath, laneFollowPath, imageOutput, object_detection_topic,speedControllerPath;
        n.getParam("/camera_topic",camera_topic);
        image_sub_ = image_transport.subscribe(camera_topic, 1, &LaneFollower::image_callback, this);
        n.getParam("/nav_drive_topic", drive_topic);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        n.getParam("/semantic_segmentation_path",semanticSegmentationPath);
        n.getParam("/lane_follow_path",laneFollowPath);
        n.getParam("/speed_controller_path",speedControllerPath);
        n.getParam("/image_output",imageOutput);
        image_pub_ = image_transport.advertise(imageOutput, 1);
        n.getParam("/object_detection_topic",object_detection_topic);

        // Load pytorch model
        try {
            semanticSegmentationModule = torch::jit::load(semanticSegmentationPath);
            semanticSegmentationModule.to(device);
            semanticSegmentationModule.eval();
            laneFollowModule = torch::jit::load(laneFollowPath);
            laneFollowModule.to(device);
            laneFollowModule.eval();
            speedControllerModule = torch::jit::load(speedControllerPath);
            speedControllerModule.to(device);
            speedControllerModule.eval();
            }
            catch (const c10::Error& e) {
            ROS_ERROR("Error Loading Model: %s", e.what());
            }

    }
    ~LaneFollower()
  {
  }
    
  void image_callback(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
      Mat outputMat;
      double steeringAngle = 0;
      double carSpeed = 0;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg,"");
      Mat imageInRGB = cv_ptr->image;

      //PREPROCESSING STEP
      Mat resolutionReduced;
      cv::resize(imageInRGB,resolutionReduced,Size(desiredX,desiredY));
      //DATA FORMAT FOR INPUT TO NETWORK
      std::vector<torch::jit::IValue> inputs;
      torch::Tensor tensor_image = torch::from_blob(resolutionReduced.data, {1,resolutionReduced.rows, resolutionReduced.cols, resolutionReduced.channels() }, at::kByte).to(device).toType(c10::kFloat);
      tensor_image = tensor_image.permute({ 0,3,1,2 });
      inputs.push_back(tensor_image);

      //PREDICTION FROM SEMANTIC SEGMENTATION NETWORK
      at::Tensor output = semanticSegmentationModule.forward(inputs).toTensor().argmax(1);
      at::Tensor cpuTensor = output.to("cpu");

      //Steering Angle
      std::vector<torch::jit::IValue> inputsToLaneFollow;
      inputsToLaneFollow.push_back(output.toType(c10::kFloat).unsqueeze(0));
      at::Tensor outputSteeringAngle = laneFollowModule.forward(inputsToLaneFollow).toTensor();
      steeringAngle = outputSteeringAngle[0].item().toDouble();
      //Speed
      std::vector<torch::jit::IValue> inputsToSpeedControl;
      inputsToSpeedControl.push_back(output.toType(c10::kFloat).unsqueeze(0));
      at::Tensor outputSpeed = speedControllerModule.forward(inputsToSpeedControl).toTensor();
      carSpeed = outputSpeed[0].item().toDouble();
      //PREPARE FOR CAMERA OUTPUT
      //DISABLE WHEN RUNNING EXPERIMENTS
      at::Tensor prediction = output.permute({1,2,0});
      prediction=prediction.mul(80).clamp(0,255).to(torch::kU8).to(torch::kCPU); // add .mul(80) for visualization
      Mat predictionMat(cv::Size(desiredX, desiredY), CV_8UC1, prediction.data_ptr<uchar>());
      cv::resize(predictionMat,outputMat,Size(outputX,outputY));
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.steering_angle = steeringAngle;
    drive_msg.speed = carSpeed;
    ROS_INFO("Steering Angle %s",std::to_string(steeringAngle).c_str());
    ROS_INFO("Speed %s",std::to_string(carSpeed).c_str());
    drive_st_msg.drive = drive_msg;
    drive_pub.publish(drive_st_msg);

    //Upload Mat Image as ROS TOPIC 
    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header; 
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; 
    out_msg.image    = outputMat;

    // Output modified video stream
    image_pub_.publish(out_msg.toImageMsg());
  }
};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "lane_follower");
    LaneFollower rw;
    ros::spin();
    return 0;
}