
#include "abv_navigation/ExtendedKalmanFilter.h"
#include "common/RateController.hpp"
#include "plog/Log.h"

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
    // process noise covariance matrix 
    Eigen::VectorXd cov(6); 
    cov << 1e-5, 1e-5, 1e-6,
           1e-2, 1e-2, 1e-3; 
    mProcessNoiseCovariance = cov.asDiagonal(); 

    // measurement noise covariance matrix 
    Eigen::VectorXd mcov(3); 
    mcov << 0.05, 0.05, 0.05;
    mMeasurementNoiseCovariance = mcov.asDiagonal();

    mMeasurementMatrix = Eigen::Matrix<double,3,6>::Zero();
    mMeasurementMatrix(0,0) = 1;
    mMeasurementMatrix(1,1) = 1;
    mMeasurementMatrix(2,2) = 1;

    mI6 = Eigen::Matrix<double,6,6>::Identity();

    // TODO: get these from config 
    mMass = 12.7; 
    mIz = 0.3; 

    mStateEst = {0,0,0,0,0,0};
    mStatePred = mStateEst;
    mCovarianceEst = Eigen::Matrix<double,6,6>::Identity() * 1e-3;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{
    mRunning.store(false); 
    if(mPredictionThread.joinable())
    {
        mPredictionThread.join(); 
    }
}

void ExtendedKalmanFilter::predict(const double dt, AbvState& out)
{
    Eigen::Vector3d u = getLatestInput();
    setLatestInput(Eigen::Vector3d::Zero());

    propagate(dt, u);
    linearize(dt, u);

    mCovarianceEst =
        mStateTransitionMatrix *
        mCovarianceEst *
        mStateTransitionMatrix.transpose()
        + mProcessNoiseCovariance;

    out = getLatestStatePrediction();
    setLatestStateEstimate(out);
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

    AbvState state = getLatestStatePrediction(); 
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


void ExtendedKalmanFilter::update(const AbvState& meas, AbvState& out)
{
    Eigen::Vector3d z;
    z << meas.x, meas.y, meas.theta;

    AbvState pred = getLatestStatePrediction();
    Eigen::Vector3d h;
    h << pred.x, pred.y, pred.theta;

    Eigen::Vector3d innovation = z - h;

    Eigen::Matrix<double,3,3> S =
        mMeasurementMatrix * mCovarianceEst * mMeasurementMatrix.transpose()
        + mMeasurementNoiseCovariance;

    Eigen::Matrix<double,6,3> K =
        mCovarianceEst * mMeasurementMatrix.transpose() * S.inverse();

    Eigen::Matrix<double,6,1> x =
        toEigen(pred) + K * innovation;

    out = fromEigen(x);
    setLatestStateEstimate(out);

    // Joseph form covariance update (numerically safer)
    mCovarianceEst =
        (mI6 - K * mMeasurementMatrix) * mCovarianceEst *
        (mI6 - K * mMeasurementMatrix).transpose()
        + K * mMeasurementNoiseCovariance * K.transpose();
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
