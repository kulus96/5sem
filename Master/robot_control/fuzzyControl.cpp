#include "fuzzyControl.h"



fuzzyControl::fuzzyControl()
{
}


void fuzzyControl::init()
{

    engine= FllImporter().fromFile("fuzzyControl.fll");
    obstacle = engine->getInputVariable("obstacle");
    distance = engine->getInputVariable("distance");
    mSteer = engine->getOutputVariable("steer");
    mSpeed = engine->getOutputVariable("speed");
    output.push_back(0);
    output.push_back(0);
}

std::vector<double> fuzzyControl::fuzzyController(float dist, float angle)
{
    if(abs(angle) < 3.14/2)
    {
    mutex.lock();
    std::string status;
    if (not engine->isReady(&status))
    {
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
    }
    obstacle->setValue(angle);
    distance->setValue(dist);
    engine->process();

    mutex.unlock();
    output[0] = double(mSteer->getValue());
    output[1] = double(mSpeed->getValue());
    }
    return output;
}
