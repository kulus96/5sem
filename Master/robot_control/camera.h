#ifndef CAMERA_H
#define CAMERA_H
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <fl/Headers.h>
#include <iostream>
#include <vector>

static boost::mutex cameramutex;


class camera
{
public:
    camera();
    void cameraCallback(ConstLaserScanStampedPtr &msg);
};

#endif // CAMERA_H
