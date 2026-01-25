
#include "abv_guidance/FromFileGenerator.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include "plog/Log.h"

FromFileGenerator::FromFileGenerator() : mIndex(0)
{

}

FromFileGenerator::~FromFileGenerator()
{

}

bool FromFileGenerator::init()
{
    std::string filePath =
        ament_index_cpp::get_package_share_directory("abv_gnc") +
        "/configuration/path.csv";

    std::ifstream file(filePath);

    if (!file.is_open())
    {
        LOGW << "Failed to open path file from " + filePath;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string field;
        double x, y, yaw;

        std::getline(ss, field, ','); x = std::stod(field);
        std::getline(ss, field, ','); y = std::stod(field);
        std::getline(ss, field, ','); yaw = std::stod(field);

        LOGV << "Wp: " << x << "," << y << "," << yaw; 
        mPath.emplace_back(x, y, yaw, "pose");
    }

    mIndex = 0;
    return true;
}

bool FromFileGenerator::hasNext()
{
    return mIndex < mPath.size();
}

Waypoint FromFileGenerator::getNext()
{
    LOGV << "Sending waypoint " << mIndex;
    return mPath.at(mIndex++);
}
