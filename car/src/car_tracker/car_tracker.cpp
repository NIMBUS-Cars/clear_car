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
#include "car/semantic_segmentation.h"

using namespace std;
using namespace cv;

torch::Device device(torch::kCUDA);

class SemanticSegmentation{
  private:
    ros::NodeHandle n;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber rgb_image_sub;
    image_transport::Subscriber depth_image_sub;
    image_transport::Publisher image_pub;
    SemanticSegmentationHandler semanitcSegmnationHandler;
    int semantic_segmentation_network_width = 0, semantic_segmentation_network_height = 0;


  public:
    SemanticSegmentation():image_transport(n){
      n = ros::NodeHandle("~");

      string custom_prefix = "/semantic_segmentation_node";
      std::string path_to_package = ros::package::getPath("car");
      string model_folder = path_to_package + "/models/semantic_segmentation/";

      std::string camera_topic, semantic_segmentation_output_topic,semantic_segmentation_module_path;
      n.getParam(custom_prefix + "/semantic_segmentation_output_topic",semantic_segmentation_output_topic);
      n.getParam(custom_prefix + "/camera_topic",camera_topic);
      n.getParam(custom_prefix + "/semantic_segmentation_module_path",semantic_segmentation_module_path);
      
      rgb_image_sub = image_transport.subscribe(camera_topic, 1, &SemanticSegmentation::rgb_callback, this);
      image_pub = image_transport.advertise(semantic_segmentation_output_topic, 1);
      this->semanitcSegmnationHandler.load_model(model_folder + semantic_segmentation_module_path);

    }
    ~SemanticSegmentation()
    {
    }
    
  void rgb_callback(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "");
      Mat imageInRGB = cv_ptr->image;
      at::Tensor semantic_segmentation_tensor = this->semanitcSegmnationHandler.get_semantic_segmentation_image(imageInRGB);
      

      //PUBLISH PREDICTION
      Mat outputMat;
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
  void depth_callback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "");
            Mat depthImage = cv_ptr->image;
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
