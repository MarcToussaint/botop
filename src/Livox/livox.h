#pragma once

#include <Kin/kin.h>
#include <Core/thread.h>


struct Point {
    float x;
    float y;
    float z;
};

struct PointCloudData {
    std::vector<Point> points;
    int max_points;
};

namespace rai
{
    struct Livox : Thread
    {    
        RAI_PARAM("livox/", double, filter, .9)
        
        Livox();
        ~Livox();
        
        void pull(rai::Configuration& C);
        
        void step();
        
        PointCloudData points_message;
        private:
            std::mutex mux;
            int max_points;
            arr points;
    };

} //namespace
