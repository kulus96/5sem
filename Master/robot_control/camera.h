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
    void cameraCallback(ConstImageStampedPtr &msg);
    void showHistogram(std::string const& name, cv::Mat1b const& image);
    void showCircles();
    float posMarbel();
    bool marbelLocated();

    bool messageRecievedCamera;
private:
    vector<Vec3f> circles;
    float width;
    float height;
    bool SC; //show camera define i define.h
};

#endif // CAMERA_H
