#ifndef LIDAR_H
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <fl/Headers.h>
#define LIDAR_H


class LIDAR
{
public:
    LIDAR();
   static void lidarCallback(ConstLaserScanStampedPtr &msg);
private:
    float distance;
    float angle;
};

#endif // LIDAR_H
