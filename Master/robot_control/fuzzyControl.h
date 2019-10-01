#ifndef fuzzyControl_H

#include "fl/Headers.h"
#include <iostream>
using namespace fl;
#define fuzzyControl_H

class fuzzyControl
{
	public:
                fuzzyControl();
                void init();
                float fuzzyController(float,float);

                Engine* engine;
                InputVariable* obstacle;
                InputVariable* distance;
                OutputVariable* steer;
};

#endif 
