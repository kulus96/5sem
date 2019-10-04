#include "fuzzyControl.h"



fuzzyControl::fuzzyControl()
{
}


void fuzzyControl::init()
{
    engine= FllImporter().fromFile("fuzzyControl.fll");
    obstacle = engine->getInputVariable("obstacle");
    distance = engine->getInputVariable("distance");
    steer = engine->getOutputVariable("mSteer");
}

float fuzzyControl::fuzzyController(float dist, float angle)
{
    std::string status;
    if (not engine->isReady(&status))
    {
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
    }
    obstacle->setValue(angle);
    distance->setValue(dist);
    engine->process();
    std:: cout << Op::str(steer->getValue()) <<std::endl;

    return float(steer->getValue());

}
