/**
 * @file path_finder.cpp
 *
 * @brief Path finding using edge detection
 * 
 * Based on code from Jetracer Starter Code and
 *
 * @author Walter Livingston
 */

#include "path_finder.hpp"
#include "cmath"

path_finder::path_finder(){
    lastAngle = 0;
    lastWaypoint = point(0.0,0.0);
    pathImage = Mat::zeros(Size(640,480),CV_8UC1);
}

double path_finder::getPathAngle(Mat& image, double look_ahead_dist, double weight){
    // Intialization of the Path Image for republishing
    pathImage = image.clone();

    // Look ahead
    double look_ahead_height = image.size().height/4;
    Rect ROI(0, image.size().height - look_ahead_height - look_ahead_dist, image.size().width, look_ahead_height); // Rectangle of Interest
    Mat look_ahead_rect = image(ROI); // Crop image

    // HSV Filter
    Mat hsv = IPROC->HSVFilt(look_ahead_rect);

    // Sobel Filter
    // Mat grad = IPROC->SobelGrad(look_ahead_rect);

    // Getting Saturation
    Mat sat = IPROC->convertSat(look_ahead_rect);

    // // Split RGB Channels
    // // (TODO) change 1 to 0 for blue channels
    // Mat b = IPROC->RGBSplit(look_ahead_rect, 2);

    // // Canny Filter
    // Mat can = IPROC->CannyGrad(image);

    // Combine HSV & Sat Filtered Images
    Mat comb;
    bitwise_or(sat, hsv, comb);
    // addWeighted(comb, 0.8, grad, 0.2, 0, comb);

    // Contour & Center Point Calculations
    vector<vector<Point>> contours = IPROC->getContours(comb);
    if(!contours.empty()){
        int max_index = IPROC->maxContourIndex(contours);
        if(max_index >= 0){

            Point origin(image.size().width/2, image.size().height);    // lastWaypoint = Point(0,0);
            // vector<Point> angleLine = findClosest(contours.at(max_index));
            
            // double lineAngle = newCalcYaw(angleLine.at(0), angleLine.at(1));
            
            // Find Bounding Rect
            RotatedRect rect = minAreaRect(contours.at(max_index));
            Point2f corners[4];
            rect.points(corners);

            vector<Point2f> clp = centerLinePoints(corners);
            double lineAngle = calcYaw(clp.at(0), clp.at(1));

            // Calculate Center Point
            double cx = rect.center.x;
            double cy = ROI.y + ROI.height/2;

            // Center Point Smoothing
            double cmx = (cx + lastWaypoint.x)/2 + 15;
            double cmy = (cy + lastWaypoint.y)/2;

            Point center(cmx,cmy);

            double centerAngle = calcYaw(center, origin);

            double retAngle = centerAngle + lineAngle*(weight);
            lastAngle = retAngle;

            // Draw Center Point Crosshairs
            line(pathImage, Point(cmx,0), Point(cmx,image.size().height), Scalar(255,0,0), 3);
            line(pathImage, Point(0,cmy), Point(image.size().width,cmy), Scalar(255,0,0), 3);
            line(pathImage(ROI), clp.at(0), clp.at(1), Scalar(0,0,255), 3);

            // Draw Contour
            drawContours(pathImage(ROI), contours, max_index, Scalar(120,190,33), 3);
            // line(pathImage(ROI), angleLine.at(0), angleLine.at(1), Scalar(255,0,0), 3);
            for(int i = 0; i < 4; i++){
                line(pathImage(ROI), corners[i], corners[(i+1)%4], Scalar(0,0,255), 3);
            }

            // Set Last Waypoint
            lastWaypoint = point(cmx, cmy);
            return retAngle;
        }
    }
    return lastAngle;
}

double path_finder::calcYaw(Point2f a, Point2f b){
    double theta = atan((a.x - b.x) / (a.y - b.y));

    return theta;
}

vector<Point2f> path_finder::centerLinePoints(Point2f* corners){
    Point2f bl = corners[0];
    Point2f tl = corners[1];
    Point2f tr = corners[2];
    Point2f br = corners[3];
    double bcx = (bl.x + br.x) / 2;
    double bcy = (bl.y + br.y) / 2;
    double tcx = (tl.x + tr.x) / 2;
    double tcy = (tl.y + tr.y) / 2;

    double dist1 = sqrt(pow(bcx - tcx, 2) + pow(bcy - tcy, 2));

    double lmx = (tl.x + bl.x) / 2; // middle of left side x
    double lmy = (tl.y + bl.y) / 2; // middle of left side y
    double rmx = (tr.x + br.x) / 2; // middle of right side x
    double rmy = (tr.y + br.y) / 2; // middle of right side y

    double dist2 = sqrt(pow(lmx - rmx, 2) + pow(lmy - rmy, 2));

    vector<Point2f> ret;
    if(abs(dist1) > abs(dist2)){
        ret.push_back(Point2f(bcx, bcy));
        ret.push_back(Point2f(tcx, tcy));
    }else{
        ret.push_back(Point2f(rmx, rmy));
        ret.push_back(Point2f(lmx, lmy));
    }

    return ret;
}

Mat path_finder::getPathImage(){
    return this->pathImage;
}