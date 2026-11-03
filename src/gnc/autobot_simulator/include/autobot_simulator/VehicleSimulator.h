#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include <memory>
#include <mutex>
#include <deque>
#include <random>

#include <eigen3/Eigen/Dense>

#include "autobot_msgs/msg/state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "OptitrackEmulator.h"
#include "UdpServer.h"
#include "VehicleState.h"

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
    std::unique_ptr<OptitrackEmulator> mOptitrackEmulator;
    bool mUseOptitrackEmulator;
    std::string mThrusterCommand;
    std::mutex mThrusterCommandMutex;

    double mMass;
    double mIzz; // moment of inertia of vehicle around vertical axis
    double mThrusterForce;
    double mMomentArm;
    double mTimestep;
    double mDamping;

    // External (Gazebo-sourced) state propagation
    bool mUseExternalPropagation;
    std::string mExternalStateTopic;
    std::string mWrenchCommandTopic;
    std::string mWrenchTargetLink;
    bool mExternalStateRecvd;

    void externalStateCallback(autobot_msgs::msg::State::SharedPtr aMsg);
    void setLatestExternalState(autobot_msgs::msg::State::SharedPtr aState) {std::lock_guard<std::mutex> lock(mExternalStateMutex); mExternalState = aState;}
    autobot_msgs::msg::State::SharedPtr getLatestExternalState() {std::lock_guard<std::mutex> lock(mExternalStateMutex); return mExternalState;}

    std::mutex mExternalStateMutex;
    autobot_msgs::msg::State::SharedPtr mExternalState;

    geometry_msgs::msg::WrenchStamped buildWrenchMsg(const Eigen::Vector2d& aForceWorld, double aTorqueZWorld);

    // control allocation matrix: row = contribution of each of the 8
    // thrusters to (fx, fy, tz) when that thruster fires alone. Sourced from
    // the same ThrusterConfig.mAllocationX/Y/Yaw fields autobot_thrusters'
    // MatrixThrusterMapper uses, so sim and hardware can't drift apart.
    // Only used when mUseMatrixAllocation is true - see convertThrusterCommandToForce.
    Eigen::Matrix<double, 3, 8> mB;

    // mirrors ThrusterCommander's strategy selection: true iff
    // Control.Thrusters.AllocationStrategy is "Matrix". The lookup-table
    // path's hand-tuned applied-thrust values for combined translation+yaw
    // commands are NOT pure single-thruster superposition (e.g. two
    // same-sign-yaw thrusters firing together is declared as 1x yaw torque,
    // not the 2x that summing their individual contributions would give) -
    // so the sim must mirror whichever mapper is actually choosing thruster
    // commands, not always use the matrix multiply.
    bool mUseMatrixAllocation;

    Eigen::Vector3d mThrustForceCmd;   // instantaneous from convertThrusterCommandToForce()
    Eigen::Vector3d mThrustForceReal;  // actually applied (with lag)
    double mThrusterTau = 0.015;        // 50 ms time constant
    int mDelaySteps = 2;               // 100 ms dead-time (optional)
    std::deque<Eigen::Vector3d> mCmdBuffer; // for dead-time


    Eigen::Vector2d mVelocity; // vx, vy
    Eigen::Vector3d mThrustForce; // fx, fy, tz

    VehicleState mVehicleState; 

    // Dropout configuration
    double mDropoutStartProbability{0.02};  // 2% chance per tick to START a burst
    int mMinDropoutSteps{10};               // minimum burst length
    int mMaxDropoutSteps{50};               // maximum burst length
    double mSimTime{0.0};

    // Dropout state
    bool mDropoutActive{false};
    int mDropoutRemainingSteps{0};
    bool mSimulateDropoutEnabled;

    // Random generators
    std::mt19937 mRng{std::random_device{}()};
    std::uniform_real_distribution<double> mUniformDist{0.0, 1.0};

    void convertThrusterCommandToForce(const std::string& aCommand);
    autobot_msgs::msg::State convertToIdl(VehicleState aState);
    Eigen::Vector3d convertBodyForceToGlobal(const Eigen::Vector3d& aThrustForce, double aYaw);
    inline double wrapPi(double a);

    void addSensorNoise(); 
    void addProcessNoise(); 
    void makeMeasurement(VehicleState& aStateToAlter);
};
#endif //VEHICLESIMULATOR_H





