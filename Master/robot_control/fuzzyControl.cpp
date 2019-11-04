#include "fuzzyControl.h"



fuzzyControl::fuzzyControl()
{

}


void fuzzyControl::init(std::string path1,std::string path2)
{
    std::cout << path1 << std::endl;
    std::cout << path2 << std::endl;

    obsEngine1= FllImporter().fromFile(path1);
    obsAngle1       = obsEngine1->getInputVariable("obstacle");
    obsDistance1    = obsEngine1->getInputVariable("distance");
    obsSteer1       = obsEngine1->getOutputVariable("steer");
    obsSpeed1       = obsEngine1->getOutputVariable("speed");

    dirEngine2= FllImporter().fromFile(path2);
    dirAngle2 = dirEngine2->getInputVariable("angle");
    dirSteer2 = dirEngine2->getOutputVariable("steer");
    dirSpeed2 = dirEngine2->getOutputVariable("speed");

    output.push_back(0);
    output.push_back(0);
    std::string status;
    if (not obsEngine1->isReady(&status)&& not dirEngine2->isReady(&status))
    {
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
        while(1)
        {
            std::cout << "inf loop" << std::endl;
        }
    }
}

std::vector<double> fuzzyControl::fuzzyController(float obsDist, float obsAngle,float dirAngle)
{
    mutex.lock();

    dirAngle2  -> setValue(dirAngle);
    dirEngine2 -> process();

    output[0] = double(dirSteer2->getValue());
    output[1] = double(dirSpeed2->getValue());

    if(abs(obsAngle) < 3.14/2)
    {

        obsAngle1->setValue(obsAngle);
        obsDistance1->setValue(obsDist);
        obsEngine1->process();
        output[0] += double(obsSteer1->getValue());
        output[1] += double(obsSpeed1->getValue());

    }
   mutex.unlock();
   return output;
}
