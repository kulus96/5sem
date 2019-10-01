#include "fuzzyControl.h"

fuzzyControl(string f)
{
    engine = FllImporter().fromFile(f);
}

float fuzzyController(float dist, float angle)
{
    using namespace fl;


    std::string status;
    if (not engine->isReady(&status))
    {
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
    }
    obstacle = engine->getInputVariable("obstacle");
    distance = engine->getInputVariable("distance");
    steer = engine->getOutputVariable("mSteer");


    obstacle->setValue(angle);
    distance->setValue(dist);
    engine->process();
    return steer;

}
