#ifndef POSITION_H
#define POSITION_H

#include <iostream>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "define.h"

class position
{
public:
    position();
    void poseCallback(ConstPosesStampedPtr &_msg);
private:
    bool CP; //cout position of robot, is set in define
};

#endif // POSITION_H
