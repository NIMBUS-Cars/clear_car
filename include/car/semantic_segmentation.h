#include "torch/torch.h"
#include "torch/script.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class SemanticSegmentationHandler{
    public:
    int network_width = 256;
    int network_height = 256;
    torch::jit::script::Module semantic_segmentation_module;

    SemanticSegmentationHandler(){
    }
    void load_model(string module_path){
      try {
        semantic_segmentation_module = torch::jit::load(module_path);
        semantic_segmentation_module.to(torch::kCUDA);
        semantic_segmentation_module.eval();
        }
        catch (const c10::Error& e) {
        }
    }
    at::Tensor get_semantic_segmentation_image(cv::Mat input){
      cv::Mat resolutionReduced;
      cv::resize(input,resolutionReduced,cv::Size(network_width,network_height));
      std::vector<torch::jit::IValue> inputs;
      torch::Tensor tensor_image = torch::from_blob(resolutionReduced.data, {1,resolutionReduced.rows, resolutionReduced.cols, resolutionReduced.channels() }, at::kByte).to(torch::kCUDA).toType(c10::kFloat);
      tensor_image = tensor_image.permute({ 0,3,1,2 });
      inputs.push_back(tensor_image);
      at::Tensor output = semantic_segmentation_module.forward(inputs).toTensor().argmax(1);
      return output;
    }
};