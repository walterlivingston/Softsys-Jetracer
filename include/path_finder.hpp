#ifndef PATH_FINDER_HPP
#define PATH_FINDER_HPP

#include "image_proc.hpp"

class path_finder {
    private:
        image_proc* IPROC = new image_proc(); // Image processing object
        Mat pathImage; // Current image of path with contour
    public:
        path_finder(); // Constructor
        struct point { double x; double y; }; // point struct
        typedef struct point Waypoint; // Waypoint Structure definition
        Waypoint point(double xx, double yy){
            Waypoint w;
            w.x = xx;
            w.y = yy;
            return w;
        };
        double lastAngle;
        Waypoint lastWaypoint;
        double calcYaw(Point2f a, Point2f b);
        vector<Point2f> centerLinePoints(Point2f* corners);
        double getPathAngle(Mat& image, double look_ahead_dist, double weight); // Get the center point of the largest contour
        Mat getPathImage(); // Getter for path image Mat
        vector<Point> findClosest(vector<Point> contour);
};

#endif