/**
 * @file image_proc.cpp
 *
 * @brief Image processing for line following
 * 
 * Based on code from Jetracer Starter Code and
 * https://stackoverflow.com/questions/46187563/finding-largest-contours-c
 *
 * @author Walter Livingston
 */

#include "image_proc.hpp"

image_proc::image_proc() {}

// From Jetracer Starter Code
Mat image_proc::convertSat(Mat& image){
    // Intialization.
    cv::Mat hls;
    // Convert Color to HLS Colorspace
    cv::cvtColor(image, hls, CV_BGR2HLS);
    // Initializing 3 individual single channel images.
    Mat hlsSplit[3];
    // Splitting
    cv::split(hls, hlsSplit);
    // Getting the Saturation Channel
    Mat out = hlsSplit[2];
    // Thresholding to remove lower saturations.
    threshold(out, out, 100, 255, THRESH_BINARY);//$$ Min and Max Threshold (80 and 255 by default) $$
    return out;
}

Mat image_proc::SobelGrad(Mat& Input){ 
    // Initializing Variables.
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    Mat grad;

    cv::Mat Input_Gray;
    cvtColor(Input, Input_Gray, COLOR_BGR2GRAY);
    // Applying Gaussian Blur to deal with smaller contours
    GaussianBlur(Input_Gray, Input_Gray, Size(3,3), 0, 0); //$$ Kernal Size (Default is 3,3) Increasing Makes Blurier. Note: Must be odd number $$
    // Computing Sobel 
    // Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT); [opencv Function Definitions]
        Sobel(Input_Gray, grad_x, CV_16S, 1, 0, 3, 1, 0);  //$$ Ksize (3 By Default) $$
        Sobel(Input_Gray, grad_y, CV_16S, 0, 1, 3, 1, 0);
        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);
        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    // Thresholding Edges (Weaker Edges Removed)
    threshold(grad, grad, 70, 255, THRESH_BINARY); //$$ Min and Max Threshold (90 and 255 by default) $$
    // Dialating to Fill in a bit: 
    // dilate(grad, grad, Mat(), Point(-1, -1), 2, 1, 1); //$$ This defaults to 3x3 Kernal for the convolution. Will Need to do more reading if you want more/less $$
    return grad;
}

Mat image_proc::Laplace(Mat& Input){
    cv::Mat Input_Gray;
    cvtColor(Input, Input_Gray, COLOR_BGR2GRAY);
    // Applying Gaussian Blur to deal with smaller contours
    GaussianBlur(Input_Gray, Input_Gray, Size(5,5), 0); //$$ Kernal Size (Default is 3,3) Increasing Makes Blurier. Note: Must be odd number $$
    Mat lap;
    // Laplacian(src_gray, laplacian, ddepth, ksize); [opencv Function Definitions]
    Laplacian(Input_Gray, lap, CV_64F, 3);
    // Convert back to CV_8U for finding contours
    Mat dist_8u;
    lap.convertTo(dist_8u, CV_8U);
    
    return dist_8u;
}

Mat image_proc::HSVFilt(Mat& Input){
    Mat hsv;
    // cvtColor(Input, hsv, COLOR_HSV);
    Mat hsv_filt;
    inRange(Input, Scalar(100, 140, 100), Scalar(110, 150, 190), hsv_filt);
    return hsv_filt;
}

Mat image_proc::RGBSplit(Mat& Input, int channel){
    Mat different_Channels[3];
    split(Input, different_Channels);
    return different_Channels[channel];
}

Mat image_proc::CannyGrad(Mat& Input){
    Mat Input_Gray;
    cvtColor(Input, Input_Gray, COLOR_BGR2GRAY);
    // Applying Gaussian Blur to deal with smaller contours
    GaussianBlur(Input_Gray, Input_Gray, Size(5,5), 0); //$$ Kernal Size (Default is 3,3) Increasing Makes Blurier. Note: Must be odd number $$
    Mat can;
    Canny(Input_Gray, can, 20, 80, 5);
    return can;
}

Mat image_proc::StructuredForest(Mat& Input){
    Mat work_image;
    //cvtColor(Input, rgb, COLOR_BGR2RGB);
    Input.convertTo(work_image, DataType<float>::type, 1/255.0);
    Mat edges;
    edge_detector->detectEdges(work_image, edges);
    
    return edges;
}

// From Jetracer Starter Code
Mat image_proc::IPM(Mat& image){
    int alpha_ = 287, beta_ = 90, gamma_ = 90;
    int f_ = 331, dist_ = 320;
    Mat dst;
    cv::Mat og = image.clone();
    
    // Converting Parameters to Degrees
    double focalLength, dist, alpha, beta, gamma; 

    alpha = alpha_ * PI/180;
    beta =((double)beta_ -90) * PI/180;
    gamma =((double)gamma_ -90) * PI/180;
    focalLength = (double)f_;
    dist = (double)dist_;

    // Resizing
    Size image_size = image.size();
    double w = (double)image_size.width, h = (double)image_size.height;

    // Projecion matrix 2D -> 3D
    Mat A1 = (Mat_<float>(4, 3)<< 
        1, 0, -w/2,
        0, 1, -h/2,
        0, 0, 0,
        0, 0, 1);

    // Rotation matrices Rx, Ry, Rz
    Mat RX = (Mat_<float>(4, 4) << 
        1, 0, 0, 0,
        0, cos(alpha), -sin(alpha), 0,
        0, sin(alpha), -cos(alpha), 0,
        0, 0, 0, 1);

    Mat RY = (Mat_<float>(4, 4) << 
        cos(beta), 0, -sin(beta), 0,
        0, 1, 0, 0,
        sin(beta), 0, cos(beta), 0,
        0, 0, 0, 1);
    Mat RZ = (Mat_<float>(4, 4) << 
        cos(gamma), -sin(gamma), 0, 0,
        sin(gamma), cos(gamma), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // R - rotation matrix
    Mat R = RX * RY * RZ;

    // T - translation matrix
    Mat T = (Mat_<float>(4, 4) << 
        1, 0, 0, 0,  
        0, 1, 0, 0,  
        0, 0, 1, dist,  
        0, 0, 0, 1); 

    // K - intrinsic matrix 
    Mat K = (Mat_<float>(3, 4) << 
        focalLength, 0, w/2, 0,
        0, focalLength, h/2, 0,
        0, 0, 1, 0); 

    Mat transformationMat = K * (T * (R * A1));
    warpPerspective(image, dst, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
    return dst;
}

vector<vector<Point>> image_proc::getContours(Mat& image){
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    return contours;
}

int image_proc::maxContourIndex(vector<vector<Point>> contours) {
    double c_max = 0;
    int max_index = -1;
    for (int i = 0; i < (int)contours.size(); i++) {
        if (contourArea(contours.at(i)) > c_max) {
            c_max = contourArea(contours.at(i));
            max_index = i;
        }
    }
    return max_index;
}