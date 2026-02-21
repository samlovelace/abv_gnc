
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

    mMatrixOfThrustDirCombinations << 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, -1, 0, 0, 0, 0, 1, 1, 1, -1, 1, -1, -1, -1, 0,
			                          0, 0, 1, -1, 0, 0, 1, -1, 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 0,
			                          0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 0;
}

ThrusterCommander::~ThrusterCommander()
{

}

void ThrusterCommander::commandThrusters(Eigen::Vector3d aControlInput)
{
    // convert control input to thruster dir vector
    Eigen::Vector3i thrustDirVector = convertToThrustVector(aControlInput); 

    // convert thrust dir vector into thruster combination 
    determineThrusterCommand(thrustDirVector); 

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

Eigen::Vector3d ThrusterCommander::getAppliedThrustVector()
{
    std::lock_guard<std::mutex> lock(mThrusterCommandMutex); 
    return mAppliedThrustVector; 
}

void ThrusterCommander::determineThrusterCommand(Eigen::Vector3i aThrustDir)
{
    std::lock_guard<std::mutex> lock(mThrusterCommandMutex);

    // calculate the thruster command sequence based on the control input
    if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 0))) // +x
    {
        mThrusterCommand = "00000011";
        mAppliedThrustVector << 2 * mThrusterForce, 0, 0; 
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 1))) // -x
    {
        mThrusterCommand = "00110000";
        mAppliedThrustVector << -2 * mThrusterForce, 0, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 2))) // +y
    {
        mThrusterCommand = "11000000";
        mAppliedThrustVector << 0, 2 * mThrusterForce, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 3))) // -y
    {
        mThrusterCommand = "00001100";
        mAppliedThrustVector << 0, -2 * mThrusterForce, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 4))) // +phi
    {					
        mThrusterCommand = "01000100";
        mAppliedThrustVector << 0, 0, 2*mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 5))) // -phi
    {
        mThrusterCommand = "10001000";
        mAppliedThrustVector << 0, 0, -2*mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 6))) // +x +y
    {
        mThrusterCommand = "01000010";
        mAppliedThrustVector << mThrusterForce, mThrusterForce, 0;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 7))) // +x -y
    {
        mThrusterCommand = "00001001";
        mAppliedThrustVector << mThrusterForce, -mThrusterForce, 0; 
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 8))) // -x +y
    {
        mThrusterCommand = "10010000";
        mAppliedThrustVector << -mThrusterForce, mThrusterForce, 0; 
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 9))) // -x -y
    {
        mThrusterCommand = "00100100";
        mAppliedThrustVector << -mThrusterForce, -mThrusterForce, 0; 
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 10))) // +x +phi
    {
        mThrusterCommand = "00000001";
        mAppliedThrustVector << mThrusterForce, 0, mThrusterForce * mMomentArm; 
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 11))) // +x -phi
    {
        mThrusterCommand = "00000010";
        mAppliedThrustVector << mThrusterForce, 0, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 12))) // -x +phi
    {
        mThrusterCommand = "00010000";
        mAppliedThrustVector << -mThrusterForce, 0, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 13))) // -x -phi
    {
        mThrusterCommand = "00100000";
        mAppliedThrustVector << -mThrusterForce, 0, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 14))) // +y +phi
    {
        mThrusterCommand = "01000000";
        mAppliedThrustVector << 0, mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 15))) // +y -phi
    {
        mThrusterCommand = "10000000";
        mAppliedThrustVector << 0, mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 16))) // -y +phi
    {
        mThrusterCommand = "00000100";
        mAppliedThrustVector << 0, -mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 17))) // -y -phi
    {
        mThrusterCommand = "00001000";
        mAppliedThrustVector << 0, -mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 18))) // +x +y +phi
    {
        mThrusterCommand = "01000001";
        mAppliedThrustVector << mThrusterForce, mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 19))) // +x +y -phi
    {
        mThrusterCommand = "10000010";
        mAppliedThrustVector << mThrusterForce, mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 20))) // +x -y +phi
    {
        mThrusterCommand = "00000101";
        mAppliedThrustVector << mThrusterForce, -mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 21))) // -x +y +phi
    {
        mThrusterCommand = "01010000";
        mAppliedThrustVector << -mThrusterForce, mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 22))) // +x -y -phi
    {
        mThrusterCommand = "00001010";
        mAppliedThrustVector << mThrusterForce, mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 23))) // -x +y -phi
    {
        mThrusterCommand = "10100000";
        mAppliedThrustVector << -mThrusterForce, mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 24))) // -x -y +phi
    {
        mThrusterCommand = "00010100";
        mAppliedThrustVector << -mThrusterForce, -mThrusterForce, mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 25))) // -x -y -phi
    {
        mThrusterCommand = "00101000";
        mAppliedThrustVector << -mThrusterForce, -mThrusterForce, -mThrusterForce * mMomentArm;
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 26))) // nothing
    {
        mThrusterCommand = "00000000";
        mAppliedThrustVector << 0, 0, 0; 
    }
}
