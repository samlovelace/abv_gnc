
#include "abv_controller/ThrusterCommander.h"
#include "plog/Log.h"

#if defined(ARCH_X86)
    // not really arch specific but im too lazy to make configurable rn
    #include "abv_controller/UdpThrusterDriver.h"
    using ThrusterDriverImpl = UdpThrusterDriver;
#elif defined(ARCH_ARM)
    #include "abv_controller/GpioThrusterDriver.h"
    using ThrusterDriverImpl = GpioThrusterDriver;
#else
    #error "Unsupported architecture. Define ARCH_X86 or ARCH_ARM."
#endif


ThrusterCommander::ThrusterCommander() : 
    mThrusterCommand("00000000"),
    mConfig(ConfigurationManager::getInstance()->getControlConfig()),
    mThrusterDriver(std::make_unique<ThrusterDriverImpl>(mConfig.mGpioPins))
{
    mThrusterDriver->init(); 

    mThrusterForce = mConfig.mForce; 
    mMomentArm = mConfig.mMomentArm; 
    mAppliedThrustVector = Eigen::Vector3d::Zero(); 

    mMatrixOfThrustDirCombinations << 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, -1, 0, 0, 0, 0, 1, 1, 1, -1, 1, -1, -1, -1, 0,
			                          0, 0, 1, -1, 0, 0, 1, -1, 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 0,
			                          0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 0;

    Eigen::Matrix<double, 3, 8> b; 
    b << 0, 0, -1, -1, 0, 0, 1, 1,
         1, 1, 0, 0, -1, -1, 0, 0,
        -1, 1, -1, 1, -1, 1, -1, 1;

    // fault a thruster
    // b.col(7) = Eigen::Vector3d::Zero(); 
    // b.col(6) = Eigen::Vector3d::Zero(); 
    //b.col(0) = Eigen::Vector3d::Zero();    // +y;

    mB = b; 
    mBpinv = mB.completeOrthogonalDecomposition().pseudoInverse(); 
}

ThrusterCommander::~ThrusterCommander()
{

}

void ThrusterCommander::commandThrusters(Eigen::Vector3d aControlInput)
{
    // convert control input to thruster dir vector
    Eigen::Vector3i thrustDirVector = convertToThrustVector(aControlInput); 

    // convert thrust dir vector into thruster combination 
    allocate(thrustDirVector); 

    mThrusterDriver->send(mThrusterCommand); 
}

Eigen::Vector3i ThrusterCommander::convertToThrustVector(Eigen::Vector3d aControlInput)
{
    Eigen::Vector3i thrustDir;
    double uOn = mConfig.mSchmittTriggerOn;
    double uOff = mConfig.mSchmittTriggerOff;
        
    for(int i = 0; i < 3; i++)
    {
        // determine the thrust direction based on the control input
        if (aControlInput[i] >= uOn)
		{
			thrustDir[i] = 1;
		}
		else if (aControlInput[i] < -uOn)
		{
			thrustDir[i] = -1;
		}
		else if (aControlInput[i] <= uOff && aControlInput[i] >= -uOff)
		{
			thrustDir[i] = 0;
		}
		else
		{
			thrustDir[i] = 0;
		}

    }

    //LOGW << "ThrustDir" << thrustDir; 
    return thrustDir;
}

void ThrusterCommander::allocate(const Eigen::Vector3i& aThrustDir)
{
    Eigen::Vector3d wrench = aThrustDir.cast<double>();
    Eigen::VectorXd u = mBpinv * wrench;
    Eigen::Vector3d input = Eigen::Vector3d::Zero(); 

    std::stringstream ss; 
    for (int i = 0; i < 8; i++)
    {
        bool fires = u[i] > THRESHOLD;
        ss << (fires ? "1" : "0");

        if (fires)
            input += mB.col(i);  // only accumulate fired thrusters
    }

    mThrusterCommand = ss.str(); 
    
    mAppliedThrustVector[0] = input[0] * mThrusterForce; 
    mAppliedThrustVector[1] = input[1] * mThrusterForce; 
    mAppliedThrustVector[2] = input[2] * (mMomentArm * mThrusterForce);   

    LOGV << "T: " << aThrustDir.transpose() 
         //<< " u: " << u.transpose() 
         << " cmd: " + ss.str() 
         << " input: " << input.transpose() 
         << " wrench: " << mAppliedThrustVector.transpose(); 
}

Eigen::Vector3d ThrusterCommander::getAppliedThrustVector()
{
    std::lock_guard<std::mutex> lock(mThrusterCommandMutex); 
    return mAppliedThrustVector; 
}