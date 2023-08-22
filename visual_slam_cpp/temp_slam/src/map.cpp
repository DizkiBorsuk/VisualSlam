#include "../include/map.hpp"

namespace mrVSLAM
{
    MapPoint::MapPoint(unsigned int id, std::array<float,3> point_position) noexcept
        : id(id), position(point_position)
    {

    }
}