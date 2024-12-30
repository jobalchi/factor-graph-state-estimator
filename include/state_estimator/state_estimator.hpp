#ifndef STATE_ESTIMATOR_HPP_
#define STATE_ESTIMATOR_HPP_

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/base/timing.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/GPSFactor.h>

#include <list>
#include <iostream>
#include <fstream>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "state_estimator/BlockingQueue.h"

#define PI 3.14159265358979323846264338

using namespace gtsam;

struct InitialPose {
    geometry_msgs::msg::Quaternion orientation;
    geometry_msgs::msg::Vector3 bias;
};

class StateEstimator : public rclcpp::Node {
    public:
        StateEstimator();
        ~StateEstimator();

        // Callback Function
        void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // General Function
        void initialPose();
        void GpsHelper();
        void GetAccGyro(const sensor_msgs::msg::Imu::SharedPtr& imu, gtsam::Vector3 &acc, gtsam::Vector3 &gyro);
        gtsam::BetweenFactor<gtsam::Pose3> integrateWheelOdom(double prevTime, double stopTime, int curFactor);

    private:
        // Declare Subscriber
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        // Declare Publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr est_odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gpsPosPub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasAccPub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasGyroPub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr posePub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr timePub_;

        // Declare Timer
        rclcpp::TimerBase::SharedPtr initialPose_timer_;
        rclcpp::TimerBase::SharedPtr gps_helper_timer_;

        geometry_msgs::msg::PoseStamped pose_msg_;
        sensor_msgs::msg::Imu imu_msg_;
        nav_msgs::msg::Odometry odom_msg_;

        InitialPose initialPose_;

        bool odom_received_;
        double lastImuT_;
        double lastImuTgps_;
        int maxQSize_;
        // BlockingQueue<std::shared_ptr<sensor_msgs::msg::NavSatFix>> gpsOptQ_;
        BlockingQueue<std::shared_ptr<geometry_msgs::msg::PoseStamped>> gpsOptQ_;
        BlockingQueue<std::shared_ptr<sensor_msgs::msg::Imu>> imuOptQ_;
        BlockingQueue<std::shared_ptr<nav_msgs::msg::Odometry>> odomOptQ_;
        bool gotFirstFix_;
        int odomKey;
        int imuKey;
        int latestGPSKey;
        bool initialPoseInitialized_;

        double startTime;
        imuBias::ConstantBias prevBias;
        Vector3 prevVel;
        Pose3 prevPose;
        boost::mutex optimizedStateMutex_;
        gtsam::NavState optimizedState_;

        double initialRotationNoise_, initialVelNoise_, initialBiasNoiseAcc_, initialBiasNoiseGyro_;
        double accSigma_, gyroSigma_, accelBiasSigma_, gyroBiasSigma_, gpsSigma_;
        double sensorX_, sensorY_, sensorZ_;
        double sensorXAngle_, sensorYAngle_, sensorZAngle_;
        double carXAngle_, carYAngle_, carZAngle_;
        double gravityMagnitude_;
        bool invertx_, inverty_, invertz_;
        double imuDt_;
        double gpsx_, gpsy_, gpsz_;
        bool fixedInitialPose_;
        double initialRoll_, initialPitch_, initialYaw_;
        bool fixedOrigin_;
        double latOrigin_, lonOrigin_, altOrigin_;
        bool usingOdom_;
        double maxGPSError_;

        double optimizedTime_;

        std::list<std::shared_ptr<sensor_msgs::msg::Imu>> imuMeasurements_, imuGrav_;
        std::shared_ptr<nav_msgs::msg::Odometry> lastOdom_;
        std::shared_ptr<sensor_msgs::msg::Imu> lastIMU_;

        gtsam::Pose3 imuPgps_;
        GeographicLib::LocalCartesian enu_;   /// Object to put lat/lon coordinates into local cartesian
        boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams_;
        gtsam::imuBias::ConstantBias optimizedBias_, previousBias_;
        boost::shared_ptr<gtsam::PreintegratedImuMeasurements> imuPredictor_;
        gtsam::Pose3 bodyPSensor_, carENUPcarNED_;
        gtsam::ISAM2 *isam_;
        gtsam::SharedDiagonal priorNoisePose_;
        gtsam::SharedDiagonal priorNoiseVel_;
        gtsam::SharedDiagonal priorNoiseBias_;
        gtsam::Vector noiseModelBetweenBias_sigma_;
};

#endif // STATE_ESTIMATOR_HPP_
