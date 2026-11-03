#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H
 
#include <thread>
#include <mutex>
#include <atomic>

#include <eigen3/Eigen/Dense>
#include "autobot_common/State.hpp"
 
class ExtendedKalmanFilter 
{ 
public:
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter();

    void predict(const double aDt, State& aStatePredictionOut);
    void update(const State& aStateMeasurement, State& aStateEstimateOut);
  
    void setLatestInput(const Eigen::Vector3d& anInput);

private:
 
    std::mutex mStateMutex; 
    std::thread mPredictionThread; 
    std::atomic<bool> mRunning; 

    Eigen::Vector3d mLatestInput; 
    std::mutex mLatestInputMutex; 
    Eigen::Vector3d getLatestInput(); 

    // system dynamics variables 
    double mMass, mIz; 
   
    // KalmanFilter variables 
    Eigen::Matrix<double, 6, 6> mCovarianceEst; 
    Eigen::Matrix<double, 6, 6> mPrevCovariance; 
    Eigen::Matrix<double, 6, 6> mProcessNoiseCovariance; 
    Eigen::Matrix<double, 3, 3> mMeasurementNoiseCovariance;

    Eigen::Matrix<double,3,6> mMeasurementMatrix;

    Eigen::Matrix<double, 6, 6> mStateTransitionMatrix; 
    Eigen::Matrix<double, 6, 3> mControlMappingMatrix; 

    Eigen::Matrix<double,6,6> mI6;

    State mStatePred; 
    State mStateEst; 
    State mPrevStateEst; 
    std::mutex mStatePredMutex; 
    std::mutex mStateEstMutex; 

    void setLatestStatePrediction(const State& aState); 
    State getLatestStatePrediction(); 

    void setLatestStateEstimate(const State& aState); 
    State getLatestStateEstimate(); 

    void propagate(const double aDt, const Eigen::Vector3d& aControlInput); 
    void linearize(const double aDt, const Eigen::Vector3d& aControlInput); 

    void updateStateTransitionMatrix(const double aDt, 
                                     const Eigen::Vector3d& aControlInput);
                                     
    void updateControlMappingMatrix(const double aDt, 
                                    const Eigen::Vector3d& aControlInput);


};
#endif //EXTENDEDKALMANFILTER_H