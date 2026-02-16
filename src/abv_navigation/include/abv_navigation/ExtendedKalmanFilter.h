#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H
 
#include <thread>
#include <mutex>
#include <atomic>

#include <eigen3/Eigen/Dense>
#include "common/AbvState.hpp"
 
class ExtendedKalmanFilter 
{ 
public:
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter();

    void update(const AbvState& aStateMeasurement, AbvState& aStateEstimateOut); 

    void setLatestInput(const Eigen::Vector3d& anInput);

private:
 
    std::mutex mStateMutex; 
    std::thread mPredictionThread; 
    std::atomic<bool> mRunning; 

    void predictionLoop(); 

    void predict(const double aDt);

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

    Eigen::Matrix<double, 6, 6> mStateTransitionMatrix; 
    Eigen::Matrix<double, 6, 3> mControlMappingMatrix; 

    AbvState mStatePred; 
    AbvState mStateEst; 
    AbvState mPrevStateEst; 
    std::mutex mStatePredMutex; 
    std::mutex mStateEstMutex; 

    void setLatestStatePrediction(const AbvState& aState); 
    AbvState getLatestStatePrediction(); 

    void setLatestStateEstimate(const AbvState& aState); 
    AbvState getLatestStateEstimate(); 

    void propagate(const double aDt, const Eigen::Vector3d& aControlInput); 
    void linearize(const double aDt, const Eigen::Vector3d& aControlInput); 

    void updateStateTransitionMatrix(const double aDt, 
                                     const Eigen::Vector3d& aControlInput);
                                     
    void updateControlMappingMatrix(const double aDt, 
                                    const Eigen::Vector3d& aControlInput);


};
#endif //EXTENDEDKALMANFILTER_H