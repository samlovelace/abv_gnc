
#include "abv_simulator/VehicleSimulator.h"
#include "abv_simulator/RosTopicManager.h"
#include "abv_msgs/msg/abv_state.hpp"
#include "abv_msgs/msg/vec3.hpp"
#include <string> 
#include <random>

VehicleSimulator::VehicleSimulator(/* args */) : 
    mUdpServer(std::make_unique<UdpServer>(6969, std::bind(&VehicleSimulator::onRecieved, this, std::placeholders::_1))), 
    mMass(12.7), mIzz(0.35), mThrusterForce(0.15), mMomentArm(0.1235), mTimestep(0.01), mDamping(0.0005),
    mVelocity(Eigen::Vector2d::Zero()), mThrustForce(Eigen::Vector3d::Zero()), 
    mTopicManager(std::make_unique<RosTopicManager>())
{ 

}

VehicleSimulator::~VehicleSimulator()
{
}

void VehicleSimulator::listen()
{
    mUdpServer->start(); 
    mTopicManager->createPublisher<abv_msgs::msg::AbvState>("abv/sim/state"); 
    mTopicManager->spinNode(); 
}

void VehicleSimulator::onRecieved(const std::string& message)
{
    setThrusterCommand(message);  
}

void VehicleSimulator::update(const double dt)
{
    mTimestep = dt; 

    convertThrusterCommandToForce(getThrusterCommand());
    mThrustForceCmd = mThrustForce; // rename for clarity

    mCmdBuffer.push_front(mThrustForceCmd);
    if (mCmdBuffer.size() > mDelaySteps)
        mCmdBuffer.pop_back();
    Eigen::Vector3d delayedCmd = mCmdBuffer.back();

    mThrustForceReal += (delayedCmd - mThrustForceReal) * (mTimestep / mThrusterTau);

    Eigen::Vector3d thrustForceVec_Gl = convertBodyForceToGlobal(mThrustForceReal);
    Eigen::Vector2d Fg(thrustForceVec_Gl.x(), thrustForceVec_Gl.y());
    double tau = thrustForceVec_Gl.z();

    Eigen::Vector2d a = (1.0 / mMass) * Fg - mDamping * mVelocity;
    double alpha = (tau / mIzz) - mDamping * mVehicleState.omega;

    mVelocity += a * mTimestep;
    mVehicleState.omega += alpha * mTimestep;
    mVehicleState.vx = mVelocity.x(); 
    mVehicleState.vy = mVelocity.y(); 
    mVehicleState.x += mVehicleState.vx * mTimestep;
    mVehicleState.y += mVehicleState.vy * mTimestep;
    mVehicleState.yaw = wrapPi(mVehicleState.yaw + mVehicleState.omega * mTimestep);

    //addProcessNoise();
    
    VehicleState mNoisyState = mVehicleState;
    mSimTime += dt;
    bool addNoise = true; 
    if(addNoise)
    {
        makeMeasurement(mNoisyState);
    } 

    // -------------------------
    // Burst Dropout Logic
    // -------------------------

    // If currently in dropout, decrement counter
    if (mDropoutActive)
    {
        mDropoutRemainingSteps--;

        if (mDropoutRemainingSteps <= 0)
        {
            mDropoutActive = false;
        }
    }
    else
    {
        // Possibly START a new dropout burst
        if (mUniformDist(mRng) < mDropoutStartProbability)
        {
            std::uniform_int_distribution<int> burstDist(mMinDropoutSteps,
                                                        mMaxDropoutSteps);

            mDropoutRemainingSteps = burstDist(mRng);
            mDropoutActive = true;
        }
    }

    // override
    mDropoutActive = false; 

    // Publish only if not in dropout
    if (!mDropoutActive)
    {
        mTopicManager->publishMessage<abv_msgs::msg::AbvState>(
            "abv/sim/state",
            convertToIdl(mNoisyState));
    }
}

void VehicleSimulator::makeMeasurement(VehicleState& s)
{
    std::normal_distribution<double> posNoise(0.0, 0.02);
    std::normal_distribution<double> yawNoise(0.0, 0.003);

    s.x += posNoise(mRng);
    s.y += posNoise(mRng);
    s.yaw += yawNoise(mRng);

    // Add small high frequency vibration
    double freq = 60.0; // Hz
    s.x += 0.005 * sin(2.0 * M_PI * freq * mSimTime);
    s.y += 0.005 * cos(2.0 * M_PI * freq * mSimTime);
}

void VehicleSimulator::addSensorNoise()
{
    const double stddev = 0.2; 
    std::default_random_engine gen; 
    std::normal_distribution<double> dist(0.0, stddev); 

    double noise = dist(gen); 
    mVehicleState.x += dist(gen); 
    mVehicleState.y += dist(gen); 
    mVehicleState.yaw += dist(gen); 
}

void VehicleSimulator::addProcessNoise()
{
    const double linearNoiseStdDev = 0.001;
    const double angularNoiseStdDev = 0.005;

    static std::default_random_engine gen(std::random_device{}());
    std::normal_distribution<double> linearNoise(0.0, linearNoiseStdDev);
    std::normal_distribution<double> angularNoise(0.0, angularNoiseStdDev);

    // Add noise to linear velocity
    mVelocity.x() += linearNoise(gen);
    mVelocity.y() += linearNoise(gen);

    // Add noise to angular velocity
    mVehicleState.omega += angularNoise(gen);
}

abv_msgs::msg::AbvState VehicleSimulator::convertToIdl(VehicleState aState)
{
    abv_msgs::msg::AbvVec3 position; 
    abv_msgs::msg::AbvVec3 velocity;

    position.x = aState.x; 
    position.y = aState.y; 
    position.yaw = aState.yaw; 

    velocity.x = aState.vx; 
    velocity.y = aState.vy; 
    velocity.yaw = aState.omega; 

    abv_msgs::msg::AbvState state; 
    state.set__position(position); 
    state.set__velocity(velocity); 

    return state;  
}

Eigen::Vector3d VehicleSimulator::convertBodyForceToGlobal(const Eigen::Vector3d& aThrustForce)
{
    double yaw = mVehicleState.yaw; 

    // rotation of abv relative to global 
    Eigen::Matrix3d Rz;
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw),  0,
             0,        0,      1;

    // Transform the vector into the new frame
    return Rz * aThrustForce;
}

inline double VehicleSimulator::wrapPi(double a)
{
    while(a <= -M_PI) a += 2.0*M_PI;
    while(a >   M_PI) a -= 2.0*M_PI;
    return a;
}

void VehicleSimulator::convertThrusterCommandToForce(const std::string& thrusterCommand)
{   
    // assume the thruster command comes in as a string of 0's and 1's
    // 0 = off, 1 = on
    // thrusterCommand = "00000000" means all thrusters are off
    // thrusterCommand = "10000000" means thruster 1 is on, all others are off

    mThrustForce = Eigen::Vector3d::Zero(); // Default case

    if (thrusterCommand == "00000011") {
        mThrustForce = Eigen::Vector3d(2*mThrusterForce, 0, 0); // +x
    } 
    else if (thrusterCommand == "00110000") 
    {
        mThrustForce = Eigen::Vector3d(-2*mThrusterForce, 0, 0); // -x
    } 
    else if (thrusterCommand == "11000000") // +y 
    {
        mThrustForce = Eigen::Vector3d(0, 2*mThrusterForce, 0);
    } 
    else if (thrusterCommand == "00001100") // -y  
    {
        mThrustForce = Eigen::Vector3d(0, -2*mThrusterForce, 0);
    } 
    else if (thrusterCommand == "01000100") // +phi  
    {
        mThrustForce = Eigen::Vector3d(0, 0, 2*mThrusterForce*mMomentArm); 
    } 
    else if (thrusterCommand == "10001000") // -phi
    {
        mThrustForce = Eigen::Vector3d(0, 0, -2*mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "01000010") // +x, +y
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, mThrusterForce, 0);
    } 
    else if (thrusterCommand == "00001001") // +x, -y
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, -mThrusterForce, 0);
    } 
    else if (thrusterCommand == "10010000") // -x, +y
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, mThrusterForce, 0); 
    } 
    else if (thrusterCommand == "00100100") // -x, -y
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, 0);
    } 
    else if (thrusterCommand == "00000001") // +x, +phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, 0, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00000010") // +x, -phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, 0, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00010000") // -x, +phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, 0, mThrusterForce*mMomentArm);
    }
    else if (thrusterCommand == "00100000") // -x, -phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, 0, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "01000000") // +y, +phi
    {
        mThrustForce = Eigen::Vector3d(0, mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "10000000") // +y, -phi
    {
        mThrustForce = Eigen::Vector3d(0, mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00000100") //-y, +phi
    {
        mThrustForce = Eigen::Vector3d(0, -mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00001000") //-y, -phi
    {
        mThrustForce = Eigen::Vector3d(0, -mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "01000001") // +x, +y, +phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00000101") // +x, -y, +phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, -mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "01010000") // -x, +y, +phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00010100") // -x, -y, +phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "10000010") // +x, +y, -phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00001010") // +x, -y, -phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, -mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "10100000") // -x, +y, -phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00101000") // -x, -y, -phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00000000") // all off
    {
        mThrustForce = Eigen::Vector3d::Zero();
    }
    else 
    {
        mThrustForce = Eigen::Vector3d::Zero();
    }

}