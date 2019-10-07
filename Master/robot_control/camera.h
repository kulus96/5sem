#ifndef CAMERA_H
#define CAMERA_H
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <fl/Headers.h>
#include <iostream>
#include <vector>
#include "define.h"

using namespace cv;

class camera
{
public:
    camera();
    void cameraCallback(ConstImageStampedPtr &msg,bool showCircles);
    void showHistogram(std::string const& name, cv::Mat1b const& image);
    void showCircles();
    vector<Vec3f> getLocations();
private:
    vector<Vec3f> circles;
};

#endif // CAMERA_H
