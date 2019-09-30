
#ifndef lidar_H
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <fl/Headers.h>
#include <iostream>
#include <vector>

#define lidar_H

struct laser
{
    float distance = 0;
    float angle= 0;
};
static boost::mutex lidarmutex;

class lidar
{
    public:

         lidar();
        void lidarCallback(ConstLaserScanStampedPtr &msg);
        void coutlidar();
        float getShortestDistance();
        float getAngleShortestDistance();

    private:
        std::vector <laser> Laser;
        int runs =0;
        float shortestDistance;
        float angleShortestDistance;

};

#endif // lidar_H
