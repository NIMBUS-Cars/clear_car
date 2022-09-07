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
#include <manual_lane_follow/imageProcessor.h>
#include <manual_lane_follow/controlConst.h>
using namespace std;
using namespace cv;

torch::Device device(torch::kCUDA);

double slope(Point first,Point second);
int laneFinder(vector<vector<double> > wLines, vector<vector<double> > yLines, int lane);
int laneNumber = -1;

//should be 720 x 1280 rbg image
//do not read implementation. This was mixed from mutiple files and will not make sense.
double get_steering_angle_from_mat(cv::Mat imageinRGB){
    double steeringAngle = 0;
    double carSpeed = 0.5;
    try
    {
        Rect crop(250, 450, 960, 270);
            Mat croppedImage;
            croppedImage = imageinRGB(crop);

            //White mins and maxes for Hue-Saturation-Value model
            int whmin = 34, whmax = 179, wsmin = 0, wsmax = 255, wvmin = 252, wvmax = 255;

            Scalar wLower(whmin, wsmin, wvmin);
            Scalar wUpper(whmax, wsmax, wvmax);
            //Yellow mins and maxes for Hue-Saturation-Value model
            int yhmin = 19, yhmax = 49, ysmin = 34, ysmax = 126, yvmin = 175, yvmax = 255;

            Scalar yLower(yhmin, ysmin, yvmin);
            Scalar yUpper(yhmax, ysmax, yvmax);

            //Create ImageProcessor (From Header file)
            ImageProcessor i;

            //Gets the blurred images for they white and yellow lines
            Mat yBlur = i.getBlur(yLower, yUpper, croppedImage);
            Mat wBlur = i.getBlur(wLower, wUpper, croppedImage);
        vector<vector<double> > yellowLaneLines;
        vector<vector<double> > whiteLaneLines;       

            Mat yErodeMat = i.getErode(yBlur);
            Mat wErodeMat = i.getErode(wBlur);

            //Processes images using Hough Transform and adds all slopes and intercepts along the bottom of image to the vector
            yellowLaneLines = i.processImage(yErodeMat);
            whiteLaneLines = i.processImage(wErodeMat);





            //Finds total number of lanes found
            int numLanes = static_cast<int>(whiteLaneLines.size()) + static_cast<int>(yellowLaneLines.size());
            

            /** -------------------------------------------------**\
            * ----------------Steering Calculation----------------- *
            \**--------------------------------------------------**/

            
            //IF BOTH COLORS OF LANES ARE FOUND, REDETERMINE LANE (USING FUNCTION DEFINED BELOW)
            if (static_cast<int>(whiteLaneLines.size()) >= 1 && static_cast<int>(yellowLaneLines.size()) >= 1)
            {
                laneNumber = laneFinder(whiteLaneLines, yellowLaneLines, laneNumber);
            }

            //CREATE STEERING CONTROLER FOR CONSTANT SPEED
            ControlConst c;

            //Function returns steering angle and speed (always 0.75 in this case) based on line lines and lane number
            vector<double> speedSteer = c.steerSpeed(yellowLaneLines, whiteLaneLines, laneNumber);
            carSpeed = speedSteer.at(0);
            steeringAngle = speedSteer.at(1);      

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return 0;
    }
    return steeringAngle;
    
}
int laneFinder(vector<vector<double> > wLines, vector<vector<double> > yLines, int lane)
{
    //USES THE X COORDINATE OF EACH OF THE LINES TO DETERMINE LANE
    int laneFind = lane;
    if (yLines.size() > 0)
    {
        if (yLines.at(0).at(0) > 480)
        {
            laneFind = 0;
        }
        else
        {
            laneFind = 1;
        }
    }
    else if (wLines.size() > 0)
    {
        if (wLines.at(0).at(0) < 480)
        {
            laneFind = 0;
        }
        else
        {
            laneFind = 1;
        }
    }
    return laneFind;
}

double slope(Point first,Point second){
  // slope is taken such that horizontal side of camera is y axis where right side is positive and left side is negative
  // vertical side of camera is x axis where top side would be positve and bottom side is negative
  // so 0,0 to 1,1 should output -1
  // any line leaning left in the camera will have a negative slope
  // any line leaning right in the camera should have a positive slope
  // slopes close to zero are straight lines
  double secondX = second.x;
  double secondY = second.y;
  double firstX  = first.x;
  double firstY = first.y;
    if(second.y - first.y == 0 ){
      return 0;
    }
    return (-1.0 * (second.x - first.x )) / (second.y - first.y);
}
