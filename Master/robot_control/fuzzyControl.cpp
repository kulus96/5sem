#include "fuzzyControl.h"



fuzzyControl::fuzzyControl()
{
}


void fuzzyControl::init(std::string path)
{
    std::cout << path << std::endl;

    engine= FllImporter().fromFile(path);
    obstacle = engine->getInputVariable("obstacle");
    distance = engine->getInputVariable("distance");
    mSteer = engine->getOutputVariable("steer");
    mSpeed = engine->getOutputVariable("speed");
    output.push_back(0);
    output.push_back(0);
    std::string status;
    if (not engine->isReady(&status))
    {
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
    }
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

        //std::cout << output[0] << " " << output[1] << std::endl;
    }
    else
    {
        output[0] = 0;
        output[1] = 1;
    }

    return output;
}
