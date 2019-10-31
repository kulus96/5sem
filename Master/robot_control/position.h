#ifndef POSITION_H
#define POSITION_H

#include <iostream>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "define.h"
#include <opencv2/opencv.hpp>
#include <vector>

class position
{
public:
    position();
    void                        poseCallback(ConstPosesStampedPtr &_msg);
    void                        ToEulerAngles();
    std::vector<float>          disNAngleToXY(float X,float Y);
    void setPath(std::string    pathToImage);
    std::vector<float>          getPath();
    float posX;
    float posY;
    float posZ;
    bool messageRecievedPos;

private:

    float lengthVector(std::vector<float>);
    cv::Mat enlargeImage(cv::Mat img,int size);
    std::vector<std::vector<float>>path;
    static void mouse_callback(int  event, int  x, int  y, int flags, void* userdata);
    void mouse_callback(int  event, int  x, int  y);
    void drawRobot();
    float oriW;
    float oriX;
    float oriY;
    float oriZ;

    double roll;
    double pitch;
    double yaw;

    bool CP; //cout position of robot, is set in define
    float intervall;
    int counter;
    int size;
    float scaleMapToTrack;
    float scaleTrackToMap;

    cv::Mat map;
};



#endif // POSITION_H
