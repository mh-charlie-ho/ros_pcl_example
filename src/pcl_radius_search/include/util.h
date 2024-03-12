#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <chrono>
#include <string>

namespace util
{
    class Timer
    {
    public:
        Timer(std::string source)
        {
            start_timepoint_ = std::chrono::high_resolution_clock::now();
            data_source = source;
        }

        ~Timer()
        {
            // stop();
        }

        void stop()
        {
            auto end_timepoint = std::chrono::high_resolution_clock::now();
            auto start = std::chrono::time_point_cast<std::chrono::microseconds>(start_timepoint_).time_since_epoch().count();
            auto end = std::chrono::time_point_cast<std::chrono::microseconds>(end_timepoint).time_since_epoch().count();

            auto duration = end - start;
            double ms = duration * 0.001;

            std::cout << data_source << "Timer:Elapsed time: " << ms << " ms." << std::endl;
        }

    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> start_timepoint_;
        std::string data_source;
    };

}

#endif