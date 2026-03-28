#ifndef I2CTHRUSTERDRIVER_H
#define I2CTHRUSTERDRIVER_H

#include <vector> 
#include "IThrusterDriver.hpp"
#include "pca9685/PCA9685.h"
 
class I2cThrusterDriver : public IThrusterDriver
{ 
public:
    I2cThrusterDriver(const YAML::Node& aConfig);
    ~I2cThrusterDriver() override; 

    bool init() override; 
    bool fini() override; 

    bool send(const std::string& aThrusterCommand) override; 

private:
    std::string mDevice; 
    std::unique_ptr<PCA9685::PCA9685> mDriver;
};
#endif //I2CTHRUSTERDRIVER_H    