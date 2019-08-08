#include "navtime.h"
#include "navbase.hpp"
#include <iostream>
using namespace utiltool;

int main(int argc, char const *argv[])
{
    std::string dat;
    while (true)
    {
        NavTime::Ptr time;
        std::cout
            << " please input time\n G for GPSTIME or C for COMMONTIME or  D for DOYTIME or N for NOW" << std::endl;
        std::cin >> dat;
        auto data = TextSplit(dat, ",");
        if (data.size() < 0)
        {
            continue;
        }
        if (data.at(0) == "G")
        {
            time = std::make_shared<NavTime>(std::stoi(data.at(1)), std::stod(data.at(2)));
        }
        else if (data.at(0) == "C")
        {
            time = std::make_shared<NavTime>(std::stoi(data.at(1)),
                                             std::stoi(data.at(2)),
                                             std::stoi(data.at(3)),
                                             std::stoi(data.at(4)),
                                             std::stoi(data.at(5)),
                                             std::stod(data.at(6)));
        }
        else if (data.at(0) == "D")
        {
            time = std::make_shared<NavTime>(std::stoi(data.at(1)), std::stoi(data.at(2)), std::stod(data.at(3)));
        }
        else if (data.at(0) == "N")
        {
            time = std::make_shared<NavTime>(NavTime::NowTime());
        }
        else
        {
            continue;
        }
        std::cout << *time << std::endl
                  << " doy " << time->Doy() << std::endl
                  << " Second Of day " << time->SecondOfDay() << std::endl;
    }
    return 0;
}
