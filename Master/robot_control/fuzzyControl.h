#ifndef fuzzyControl_H

#include "fl/Headers.h"
#include <iostream>

#define fuzzyControl_H

class fuzzyControl
{
	public:
                fuzzyControl(std::string);
                float fuzzyController(float,float);

	private:
                Engine* engine;
                InputVariable* obstacle;
                InputVariable* distance;
                OutputVariable* steer;
};

#endif 
