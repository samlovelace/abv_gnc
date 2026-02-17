#ifndef ICONTROLDEVICE_H
#define ICONTROLDEVICE_H
 
#include <eigen3/Eigen/Dense>
#include <mutex> 
 
class IControlDevice 
{ 
public:
    virtual ~IControlDevice() = default; 
    virtual bool handleInput() = 0; 
    
    void setCommand(const Eigen::VectorXd& aCmd) {std::lock_guard<std::mutex> lock(mCommandMutex); mCommand = aCmd; }
    Eigen::VectorXd getCommand() {std::lock_guard<std::mutex> lock(mCommandMutex); return mCommand; }


protected:
    Eigen::VectorXd mCommand; 
    std::mutex mCommandMutex; 
};
#endif //ICONTROLDEVICE_H   