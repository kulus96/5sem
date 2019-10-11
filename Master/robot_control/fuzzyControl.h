#ifndef fuzzyControl_H
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include "fl/Headers.h"
#include <iostream>
#include "define.h"
#include <vector>
using namespace fl;


#define fuzzyControl_H


class fuzzyControl
{
	public:
                fuzzyControl();
                void init(std::string);
                std::vector<double> fuzzyController(float,float);
private:
                Engine* engine;
                InputVariable* obstacle;
                InputVariable* distance;
                OutputVariable* mSteer;
                OutputVariable* mSpeed;
                std::vector<double> output;
};

#endif 
