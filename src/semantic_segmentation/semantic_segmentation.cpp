#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include "torch/torch.h"
#include "torch/script.h"

using namespace std;
using namespace cv;

torch::Device device(torch::kCUDA);

class SemanticSegmentation{
  private:
    ros::NodeHandle n;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    torch::jit::script::Module semantic_segmentation_module;

    int camera_height,camera_width;
    int network_width = 0, network_height = 0;
    int viewing_width = 0, viewing_height = 0;
    bool view_image = false;

  public:
    SemanticSegmentation():image_transport(n) {
      n = ros::NodeHandle("~");

      string custom_prefix = "/semantic_segmentation_node";
      std::string path_to_package = ros::package::getPath("car");
      string model_folder = path_to_package + "/models/semantic_segmentation/";

      n.getParam(custom_prefix + "/network_width", network_width);
      n.getParam(custom_prefix + "/network_height", network_height);
      n.getParam(custom_prefix + "/viewing_width", viewing_width);
      n.getParam(custom_prefix + "/viewing_height", viewing_height);
      n.getParam(custom_prefix + "/view_image",view_image);
      std::string camera_topic, semantic_segmentation_output_topic,semantic_segmentation_module_path;
      n.getParam(custom_prefix + "/semantic_segmentation_output_topic",semantic_segmentation_output_topic);
      n.getParam(custom_prefix + "/camera_topic",camera_topic);
      n.getParam(custom_prefix + "/semantic_segmentation_module_path",semantic_segmentation_module_path);
      
      image_sub = image_transport.subscribe(camera_topic, 1, &SemanticSegmentation::image_callback, this);
      image_pub = image_transport.advertise(semantic_segmentation_output_topic, 1);
      
      // Load pytorch model
      try {
        semantic_segmentation_module = torch::jit::load(model_folder + semantic_segmentation_module_path);
        semantic_segmentation_module.to(device);
        semantic_segmentation_module.eval();
        }
        catch (const c10::Error& e) {
          ROS_ERROR("Error Loading Model: %s", e.what());
        }

    }
    ~SemanticSegmentation()
    {
    }
    
  void image_callback(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
      cv_ptr = cv_bridge::toCvCopy(msg, "");
      Mat imageInRGB = cv_ptr->image;
      // cvtColor(cv_ptr->image,imageInRGB,COLOR_RGBA2RGB);

      //PREPROCESSING STEP
      Mat resolutionReduced;
      cv::resize(imageInRGB,resolutionReduced,Size(network_width,network_height));

      //DATA FORMAT FOR INPUT TO NETWORK

      std::vector<torch::jit::IValue> inputs;
      torch::Tensor tensor_image = torch::from_blob(resolutionReduced.data, {1,resolutionReduced.rows, resolutionReduced.cols, resolutionReduced.channels() }, at::kByte).to(device).toType(c10::kFloat);
      tensor_image = tensor_image.permute({ 0,3,1,2 });
      inputs.push_back(tensor_image);

      //PREDICTION FROM SEMANTIC SEGMENTATION NETWORK

      at::Tensor output = semantic_segmentation_module.forward(inputs).toTensor().argmax(1);
      
      //PUBLISH PREDICTION
      Mat outputMat;
      if(view_image){
        at::Tensor prediction = output.mul(80).clamp(0,255).permute({1,2,0}).to(torch::kU8).to("cpu");
        Mat predictionMat(cv::Size(network_width, network_height), CV_8UC1, prediction.data_ptr<uchar>());
        cv::resize(predictionMat,outputMat,Size(viewing_width,viewing_height));
      }else{
        at::Tensor prediction = output.permute({1,2,0}).to(torch::kU8).to("cpu");
        Mat predictionMat(cv::Size(network_width, network_height), CV_8UC1, prediction.data_ptr<uchar>());
        outputMat = predictionMat;
      }
      //Upload Mat Image as ROS TOPIC 
      cv_bridge::CvImage out_msg;
      out_msg.header   = msg->header; 
      out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; 
      out_msg.image    = outputMat;

      image_pub.publish(out_msg.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "semantic_segmentation");
    SemanticSegmentation rw;
    ros::spin();
    return 0;
}
