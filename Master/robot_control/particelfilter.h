#ifndef PARTICELFILTER_H
#define PARTICELFILTER_H
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <vector>
#include "define.h"
#include <iostream>
#include "lidar.h"

struct particelData
{
    float posx;
    float posy;
    float ori;
    float weight;
};

class particelFilter
{
public:
    particelFilter();
    particelFilter(std::string path,lidar& laser);
    cv::Mat enlargeImage(cv::Mat img,int size);
    void findPos(float dir,float velo);
    float likelihood(int posx,int posy, float ori);

    void generateDistanceFromEachPosistion();
    float normPDF(int x,float mean,float std);
    float normalize(float x,float min,float max,float rangemin,float rangemax);


private:
    int sizeofMapComparedtoOriginal;
    cv::Mat map;
    int numberOfLaserReadings;
    int numberOfGeneratedLaserReadings;
    int STD;
    int NP;
    std::vector<std::vector<std::vector<float>>> realMapDistance;
    std::vector<particelData> M;
    std::vector<std::vector<particelData>> N;

    //Used in the function likelihood
    float readingsIntervall;
    float lidarIncrement;
    float sensorRange;
    int indexIntervall;
    lidar* sensor;
};

#endif // PARTICELFILTER_H
