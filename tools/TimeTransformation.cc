#include "navtime.h"
#include "navbase.hpp"
#include "gflags/gflags.h"
#include <iostream>
using namespace utiltool;

DEFINE_string(time_type, "gpstime", "时间类型");
DEFINE_string(gpstime, "2000,0", "gps时间");
DEFINE_string(commontime, "2019,01,01,0,0,0.0", "一般时间");
DEFINE_string(doytime, "2019, 01,0.0", "年积日时间");

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << "/*--------------------------- input ---------------------------*/ " << std::endl;
    std::cout << "input time type: " << FLAGS_time_type << std::endl
              << std::endl;
    std::cout << "/*--------------------------- output ---------------------------*/ " << std::endl;
    NavTime::Ptr time;
    if(FLAGS_time_type == "gpstime")
    {
        auto data = TextSplit(FLAGS_gpstime, ",");
        time = std::make_shared<NavTime>(std::stoi(data.at(1 - 1)), std::stod(data.at(2 - 1)));
    }
    else if(FLAGS_time_type=="commontime")
    {
        auto data = TextSplit(FLAGS_commontime, ",");
        time = std::make_shared<NavTime>(std::stoi(data.at(1 - 1)),
                                         std::stoi(data.at(2 - 1)),
                                         std::stoi(data.at(3 - 1)),
                                         std::stoi(data.at(4 - 1)),
                                         std::stoi(data.at(5 - 1)),
                                         std::stod(data.at(6 - 1)));
    }
    else if(FLAGS_time_type=="doytime")
    {
        auto data = TextSplit(FLAGS_doytime, ",");
        time = std::make_shared<NavTime>(std::stoi(data.at(1 - 1)), std::stoi(data.at(2 - 1)), std::stod(data.at(3 - 1)));
    }
    else
    {
        std::cout << "time type error" << std::endl;
    }
    std::cout << *time << std::endl
              << " doy " << time->Doy() << std::endl
              << " Second Of day " << time->SecondOfDay() << std::endl;
    return 0;
}
