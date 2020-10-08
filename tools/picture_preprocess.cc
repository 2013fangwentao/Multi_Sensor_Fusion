/*
** picture_preprocess.cc for mscnav in /media/fwt/Data/program/mscnav/tools
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Fri Dec 27 下午3:51:45 2019 little fang
** Last update Tue Jan 13 上午9:31:42 2020 little fang
*/

#include "stdio.h"
#include "navbase.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

struct time_info
{
    int index;
    int week;
    double millsecond;
};

std::vector<time_info> TimeInfoVec;

int main(int argc, const char **argv)
{
    if (argc != 4)
    {
        std::cout << ("usage: picture_preprocess pps_event.txt picture_name timetamp_file(outfile)") << std::endl;
        return 0;
    }
    std::ifstream pps_event(argv[1]);
    if (!pps_event.good())
    {
        std::cout << "pps_event.txt read failed" << std::endl;
        return 0;
    }
    time_info ti_bak;
    ti_bak.index = -999;
    std::string line;
    while (!pps_event.eof())
    {
        getline(pps_event, line);
        auto data = utiltool::TextSplit(line, "\\s+");
        if (data.size() <= 2)
            continue;
        time_info ti;
        ti.index = std::stoi(data[2]);
        ti.week = std::stoi(data[3]);
        ti.millsecond = std::stoi(data[4]);
        if (ti_bak.index == -999)
        {
            TimeInfoVec.emplace_back(ti);
            ti_bak = ti;
        }
        else
        {
            int index_diff = ti.index - ti_bak.index;
            if (index_diff < 0)
            {
                index_diff += 65536;
            }
            double millsecond_diff = ti.millsecond - ti_bak.millsecond;
            double diff = millsecond_diff / (double)index_diff;
            while (ti_bak.index < ti.index)
            {
                ti_bak.index += 1;
                ti_bak.millsecond += diff;
                TimeInfoVec.emplace_back(ti_bak);
            }
        }
    }
    pps_event.close();
    std::ifstream picture_file(argv[2]);
    if (!picture_file.good())
    {
        std::cout << "pps_event.txt read failed" << std::endl;
        return 0;
    }

    std::vector<std::string> picture_files;

    while (!picture_file.eof())
    {
        getline(picture_file, line);
        line.append(".png");
        picture_files.emplace_back(line);
    }
    picture_file.close();

    if (abs(static_cast<int>(TimeInfoVec.size() - picture_files.size()) > 2))
    {
        std::cout << "time stamp error, please check" << std::endl;
        std::cout << "press any key for break" << std::endl;
        std::getchar();
        return 0;
    }

    std::ofstream timestamp_file(argv[3]);
    int size = TimeInfoVec.size() - picture_files.size() > 0 ? picture_files.size() : TimeInfoVec.size();
    for (size_t i = 0; i < size; i++)
    {
        auto tmp = TimeInfoVec.at(i);
        timestamp_file << tmp.week << "\t" << std::fixed << std::setprecision(5) << (tmp.millsecond / 1000) << "\t" << picture_files.at(i) << std::endl;
    }
    timestamp_file.close();
    return 0;
}