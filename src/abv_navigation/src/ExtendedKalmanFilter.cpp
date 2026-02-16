
#include "abv_navigation/ExtendedKalmanFilter.h"
#include "common/RateController.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
    // TODO: init other variables 
    Eigen::VectorXd cov(6); 
    cov << 0, 0, 0, 0.01, 0.01, 0.01; 
    mProcessNoiseCovariance = cov.asDiagonal(); 

    Eigen::VectorXd mcov(3); 
    mcov << 0.01, 0.01, 0.01;
    mMeasurementNoiseCovariance = mcov.asDiagonal(); 

    // TODO: get these from config 
    mMass = 12.7; 
    mIz = 0.3; 

    mPredictionThread = std::thread(&ExtendedKalmanFilter::predictionLoop, this); 
}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{
    mRunning.store(false); 
    if(mPredictionThread.joinable())
    {
        mPredictionThread.join(); 
    }
}

void ExtendedKalmanFilter::predict(const double aDt)
{   
    Eigen::Vector3d latestInput = getLatestInput(); 
    
    propagate(aDt, latestInput); 
    linearize(aDt, latestInput);  
    
    mCovarianceEst = mStateTransitionMatrix * 
                     mPrevCovariance * 
                     mStateTransitionMatrix.transpose() + 
                     mProcessNoiseCovariance; 
}

void ExtendedKalmanFilter::propagate(const double aDt, const Eigen::Vector3d& aControlInput)
{
    AbvState statePred;
    AbvState prevStateEst = getLatestStateEstimate(); 

    statePred.x = prevStateEst.x + prevStateEst.vx * aDt; 
    statePred.y = prevStateEst.y + prevStateEst.vy * aDt; 
    statePred.theta = prevStateEst.theta + prevStateEst.omega * aDt; 

    Eigen::Rotation2D<double> Rz(prevStateEst.theta); 

    Eigen::Vector2d prevVel = {prevStateEst.vx, prevStateEst.vy}; 
    Eigen::Vector2d velPred = prevVel + (aDt / mMass) * (Rz * aControlInput.head<2>());
    
    statePred.vx = velPred.x(); 
    statePred.vy = velPred.y(); 

    double Tz = aControlInput[2]; 
    statePred.omega = prevStateEst.omega + (aDt / mIz) * Tz; 

    setLatestStatePrediction(statePred); 
}

void ExtendedKalmanFilter::linearize(const double aDt, const Eigen::Vector3d& aControlInput)
{
    updateStateTransitionMatrix(aDt, aControlInput); 
    updateControlMappingMatrix(aDt, aControlInput); 
}

void ExtendedKalmanFilter::updateStateTransitionMatrix(const double aDt, 
                                                       const Eigen::Vector3d& aControlInput)
{
    mStateTransitionMatrix.setIdentity();

    // x,y,θ rows = 0,1,2
    // vx,vy,ω cols = 3,4,5
    mStateTransitionMatrix.block<3,3>(0,3) =
        (Eigen::Vector3d(aDt, aDt, aDt)).asDiagonal();
 
    double Fx = aControlInput[0]; 
    double Fy = aControlInput[1]; 

    AbvState state = getLatestStateEstimate(); 
    double th = state.theta; 

    double a = aDt / mMass * (-Fx * sin(th) - Fy * cos(th)); 
    double b = aDt / mMass *  (Fx * cos(th) - Fy * sin(th)); 

    mStateTransitionMatrix(3, 2) = a;   // ∂vx / ∂θ
    mStateTransitionMatrix(4, 2) = b;   // ∂vy / ∂θ
}

void ExtendedKalmanFilter::updateControlMappingMatrix(const double aDt,
                                                      const Eigen::Vector3d& aControlInput)
{
    mControlMappingMatrix.setZero();

    AbvState state = getLatestStateEstimate();
    double th = state.theta;
    double dm = aDt / mMass;

    // vx, vy rows = 3,4
    // Fx, Fy cols = 0,1
    mControlMappingMatrix.block<2,2>(3,0) <<
        dm * cos(th), -dm * sin(th),
        dm * sin(th),  dm * cos(th);

    // ω row = 5, Tz col = 2
    mControlMappingMatrix(5,2) = aDt / mIz;
}


void ExtendedKalmanFilter::update(const AbvState& aStateMeasurement, AbvState& aStateEstimateOut)
{
    auto I6x6 = Eigen::Matrix<double, 6, 6>::Identity(); 
    Eigen::Matrix<double,3,6> H;
    H.setZero();
    H(0,0) = 1;
    H(1,1) = 1;
    H(2,2) = 1;

    AbvState statePred = getLatestStatePrediction(); 

    Eigen::Vector3d measurement; 
    measurement << aStateMeasurement.x, aStateMeasurement.y, aStateMeasurement.theta; 

    Eigen::Vector3d prediction; 
    prediction << statePred.x, statePred.y, statePred.theta; 
 
    // Measurement matrix is just identity on x,y so innovation is difference of state
    Eigen::Vector3d innovation = measurement - prediction;  

    // Innovation covariance 
    auto S = H * mCovarianceEst * H.transpose() + mMeasurementNoiseCovariance; 
    
    // Kalman Gain
    auto K = mCovarianceEst * H.transpose() * S.inverse(); 

    // state update 
    auto stateEst = toEigen(statePred) + K * innovation;
    
    // convert to internal state representation 
    aStateEstimateOut = fromEigen(stateEst);  
    setLatestStateEstimate(aStateEstimateOut); 

    // covariance update 
    mPrevCovariance = mCovarianceEst; 
    mCovarianceEst = (I6x6 - K * H) * mCovarianceEst; 
}

void ExtendedKalmanFilter::predictionLoop()
{
    // TODO: get from config 
    RateController rate(50); 

    mRunning.store(true);
    rate.start(); 
    rate.block(); // warmup to get actual dt in loop

    while(mRunning.load())
    {
        rate.start(); 

        // EKF prediction step 
        predict(rate.getDeltaTime());  
        
        rate.block(); 
    }

}

void ExtendedKalmanFilter::setLatestStatePrediction(const AbvState& aState)
{
    std::lock_guard<std::mutex> lock(mStatePredMutex); 
    mStatePred = aState; 
}

AbvState ExtendedKalmanFilter::getLatestStatePrediction()
{
    std::lock_guard<std::mutex> lock(mStatePredMutex); 
    return mStatePred;
}


void ExtendedKalmanFilter::setLatestStateEstimate(const AbvState& aState)
{
    std::lock_guard<std::mutex> lock(mStateEstMutex); 
    mStateEst = aState; 
}

AbvState ExtendedKalmanFilter::getLatestStateEstimate()
{
    std::lock_guard<std::mutex> lock(mStateEstMutex); 
    return mStateEst;
}

Eigen::Vector3d ExtendedKalmanFilter::getLatestInput()
{
    std::lock_guard<std::mutex> lock(mLatestInputMutex); 
    return mLatestInput; 
}

void ExtendedKalmanFilter::setLatestInput(const Eigen::Vector3d& anInput)
{
    std::lock_guard<std::mutex> lock(mLatestInputMutex); 
    mLatestInput = anInput;
}
