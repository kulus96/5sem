
#ifndef lidar_H
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <fl/Headers.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "define.h"
#include <fstream>


#define lidar_H

struct laser
{
    float distance = 0;
    float angle= 0;
};

class lidar
{

public:

         lidar();
        void lidarCallback(ConstLaserScanStampedPtr &msg);
        float getShortestDistance();
        float getAngleShortestDistance();
        float objInFront(float range);
        void coutlidar();
        void printtoCSV(float);
        float getLidarReading(int index);
        int getNumberofLidarReadings();
        float getLidarRangeMax();
        float getLidarIncrement();
        bool  lidarReady();

        bool messageRecievedLidar;


private:
        void showLidar();
        std::vector <laser> Laser;
        int runs =0;
        float shortestDistance;
        float angleShortestDistance;
        float angle_min;
        float angle_increment;
        float range_max;
        float range_min;
        int nintensities;
        int nranges;
        int sec;
        int nsec;
        bool SL;  //show Lidar define i define.h
};

#endif // lidar_H
