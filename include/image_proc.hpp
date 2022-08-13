#ifndef IMAGE_PROC_HPP
#define IMAGE_PROC_HPP

// OpenCV
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc/structured_edge_detection.hpp>
#include <opencv2/highgui/highgui_c.h>
using namespace cv;
using namespace std;

#include <ament_index_cpp/get_package_share_directory.hpp>
// PI Constant
#define PI 3.1415926

class image_proc {
    public:
        image_proc(); // Constructor
        Mat IPM(Mat& image); // Inverse perspective mapping
        // Based on Carson Loyal's mono_image_pub.cpp
        static std::string mat_type2encoding(int mat_type)
        {
            switch (mat_type) {
                case CV_8UC1:
                    return "mono8";
                case CV_8UC3:
                    return "bgr8";
                case CV_16SC1:
                    return "mono16";
                case CV_8UC4:
                    return "rgba8";
                default:
                    throw std::runtime_error("Unsupported encoding type");
            }
        }
        Mat convertSat(Mat& image); // Convert image based on saturation
        Mat SobelGrad(Mat& Input); // Apply Sobel gradiant to image
        Mat HSVFilt(Mat& Input); // Apply HSV Filter4
        Mat RGBSplit(Mat& Input, int channel); // Split RGB Channels
        Mat CannyGrad(Mat& Input); // Apply Cann gradient to image
        Mat StructuredForest(Mat& Input); // Structured Forest Edge Detection
        Mat Laplace(Mat& Input); // Apply Laplacian gradiant to image
        vector<vector<Point>> getContours(Mat& image); // Get all contours within image
        int maxContourIndex(vector<vector<Point>> contours); // Get the index of the max contour
    private:
        const Ptr<ximgproc::StructuredEdgeDetection> edge_detector = 
            ximgproc::createStructuredEdgeDetection(ament_index_cpp::get_package_share_directory("cam_control") + "/config/model.yml"); // location of edge detection model
};

#endif