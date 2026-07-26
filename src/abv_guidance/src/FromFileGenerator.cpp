
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
        ament_index_cpp::get_package_share_directory("abv_bringup") +
            "/config/path.csv";

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
        double timeout = -1.0; // -1 means "use the configured default"
        Eigen::Vector3d arrivalTol(-1.0, -1.0, -1.0); // per-axis <= 0 means "no override"

        std::getline(ss, field, ','); x = std::stod(field);
        std::getline(ss, field, ','); y = std::stod(field);
        std::getline(ss, field, ','); yaw = std::stod(field);

        // optional 4th column overrides the default waypoint timeout
        if (std::getline(ss, field, ','))
        {
            timeout = std::stod(field);
        }

        // optional 5th-7th columns override the arrival tolerance [m, m, rad]
        // for this waypoint; all three must be present or none are applied
        std::string xTolField, yTolField, yawTolField;
        if (std::getline(ss, xTolField, ',') &&
            std::getline(ss, yTolField, ',') &&
            std::getline(ss, yawTolField, ','))
        {
            arrivalTol << std::stod(xTolField), std::stod(yTolField), std::stod(yawTolField);
        }

        LOGV << "Wp: " << x << "," << y << "," << yaw << " (timeout: " << timeout
             << ", tol: " << arrivalTol.transpose() << ")";
        mPath.emplace_back(x, y, yaw, "pose", timeout, arrivalTol);
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
