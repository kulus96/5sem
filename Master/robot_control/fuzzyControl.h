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
                void init(std::string,std::string);
                std::vector<double> fuzzyController(float,float,float);
private:
                Engine* obsEngine1;
                InputVariable* obsAngle1;
                InputVariable* obsDistance1;
                OutputVariable* obsSteer1;
                OutputVariable* obsSpeed1;

                Engine* dirEngine2;
                InputVariable* dirAngle2;
                OutputVariable* dirSteer2;
                OutputVariable* dirSpeed2;
                std::vector<double> output;
};

#endif 
