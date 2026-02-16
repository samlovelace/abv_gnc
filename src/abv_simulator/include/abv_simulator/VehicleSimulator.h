#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include "UdpServer.h"
#include <mutex> 
#include <eigen3/Eigen/Dense>
#include "VehicleState.h"
#include "RosTopicManager.h"
#include <deque>
#include <random>

class VehicleSimulator
{
public:
    VehicleSimulator(/* args */);
    ~VehicleSimulator();

    void onRecieved(const std::string& message);
    void update(const double dt); 
    void listen(); 

    void setThrusterCommand(const std::string& aMsg) {std::lock_guard<std::mutex> lock(mThrusterCommandMutex); mThrusterCommand = aMsg;}
    std::string getThrusterCommand() {std::lock_guard<std::mutex> lock(mThrusterCommandMutex); return mThrusterCommand;}


private:
    std::unique_ptr<UdpServer> mUdpServer; 
    std::string mThrusterCommand; 
    std::mutex mThrusterCommandMutex; 

    double mMass; 
    double mIzz; // moment of inertia of vehicle around vertical axis 
    double mThrusterForce; 
    double mMomentArm; 
    double mTimestep;
    double mDamping;

    Eigen::Vector3d mThrustForceCmd;   // instantaneous from convertThrusterCommandToForce()
    Eigen::Vector3d mThrustForceReal;  // actually applied (with lag)
    double mThrusterTau = 0.015;        // 50 ms time constant
    int mDelaySteps = 2;               // 100 ms dead-time (optional)
    std::deque<Eigen::Vector3d> mCmdBuffer; // for dead-time


    Eigen::Vector2d mVelocity; // vx, vy
    Eigen::Vector3d mThrustForce; // fx, fy, tz

    VehicleState mVehicleState; 

    std::default_random_engine mRng;

    void convertThrusterCommandToForce(const std::string& aCommand);
    abv_msgs::msg::AbvState convertToIdl(VehicleState aState);
    Eigen::Vector3d convertBodyForceToGlobal(const Eigen::Vector3d& aThrustForce); 
    inline double wrapPi(double a);

    void addSensorNoise(); 
    void addProcessNoise(); 
    void makeMeasurement(VehicleState& aStateToAlter);

    std::unique_ptr<RosTopicManager> mTopicManager; 
};
#endif //VEHICLESIMULATOR_H





