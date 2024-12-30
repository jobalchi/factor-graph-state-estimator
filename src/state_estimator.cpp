#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <vector>

#include "state_estimator/state_estimator.hpp"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace gtsam;
// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::G; // GPS pose

// Macro for getting the time stamp of a ros message
#define TIME(msg) ( (msg)->header.stamp.sec + (msg)->header.stamp.nanosec * 1e-9 )

StateEstimator::StateEstimator() : Node("state_estimator_node"),
    odom_received_(false),
    lastImuT_(0.0),
    lastImuTgps_(0.0),
    maxQSize_(0),
    gpsOptQ_(40),
    imuOptQ_(400),
    odomOptQ_(100),
    gotFirstFix_(false),
    odomKey(1),
    imuKey(1),
    latestGPSKey(0),
    initialPoseInitialized_(false) {

    // Declare and get parameters
    this->declare_parameter<double>("InitialRotationNoise", 1.0);
    this->declare_parameter<double>("InitialVelocityNoise", 0.1);
    this->declare_parameter<double>("InitialBiasNoiseAcc", 1e-1);
    this->declare_parameter<double>("InitialBiasNoiseGyro", 1e-2);
    
    this->declare_parameter<double>("AccelerometerSigma", 6.0e-2);
    this->declare_parameter<double>("GyroSigma", 2.0e-2);
    this->declare_parameter<double>("AccelBiasSigma", 2.0e-4);
    this->declare_parameter<double>("GyroBiasSigma", 3.0e-5);
    this->declare_parameter<double>("GPSSigma", 0.07);

    this->declare_parameter<double>("SensorTransformX", 0.0);
    this->declare_parameter<double>("SensorTransformY", 0.0);
    this->declare_parameter<double>("SensorTransformZ", 0.0);

    this->declare_parameter<double>("SensorXAngle", 0.0);
    this->declare_parameter<double>("SensorYAngle", 0.0);
    this->declare_parameter<double>("SensorZAngle", 0.0);

    this->declare_parameter<double>("CarXAngle", 0.0);
    this->declare_parameter<double>("CarYAngle", 0.0);
    this->declare_parameter<double>("CarZAngle", 0.0);

    this->declare_parameter<double>("Gravity", 9.8);

    this->declare_parameter<bool>("InvertX", false);
    this->declare_parameter<bool>("InvertY", false);
    this->declare_parameter<bool>("InvertZ", false);
    
    this->declare_parameter<double>("Imudt", 1.0/200.0);

    this->declare_parameter<double>("GPSX", 0.0);
    this->declare_parameter<double>("GPSY", 0.0);
    this->declare_parameter<double>("GPSZ", 0.0);

    this->declare_parameter<bool>("FixedInitialPose", false);

    this->declare_parameter<double>("initialRoll", 0.0);
    this->declare_parameter<double>("initialPitch", 0.0);
    this->declare_parameter<double>("initialYaw", 0.0);

    this->declare_parameter<bool>("FixedOrigin", false);

    this->declare_parameter<double>("latOrigin", 0.0);
    this->declare_parameter<double>("lonOrigin", 0.0);
    this->declare_parameter<double>("altOrigin", 0.0);

    this->declare_parameter<bool>("UseOdom", false);
    this->declare_parameter<double>("MaxGPSError", 10.0);

    // Retrieve parameters
    this->get_parameter("InitialRotationNoise", initialRotationNoise_);
    this->get_parameter("InitialVelocityNoise", initialVelNoise_);
    this->get_parameter("InitialBiasNoiseAcc", initialBiasNoiseAcc_);
    this->get_parameter("InitialBiasNoiseGyro", initialBiasNoiseGyro_);
    
    this->get_parameter("AccelerometerSigma", accSigma_);
    this->get_parameter("GyroSigma", gyroSigma_);
    this->get_parameter("AccelBiasSigma", accelBiasSigma_);
    this->get_parameter("GyroBiasSigma", gyroBiasSigma_);
    this->get_parameter("GPSSigma", gpsSigma_);

    this->get_parameter("SensorTransformX", sensorX_);
    this->get_parameter("SensorTransformY", sensorY_);
    this->get_parameter("SensorTransformZ", sensorZ_);

    this->get_parameter("SensorXAngle", sensorXAngle_);
    this->get_parameter("SensorYAngle", sensorYAngle_);
    this->get_parameter("SensorZAngle", sensorZAngle_);

    this->get_parameter("CarXAngle", carXAngle_);
    this->get_parameter("CarYAngle", carYAngle_);
    this->get_parameter("CarZAngle", carZAngle_);

    this->get_parameter("Gravity", gravityMagnitude_);

    this->get_parameter("InvertX", invertx_);
    this->get_parameter("InvertY", inverty_);
    this->get_parameter("InvertZ", invertz_);

    this->get_parameter("Imudt", imuDt_);

    this->get_parameter("GPSX", gpsx_);
    this->get_parameter("GPSY", gpsy_);
    this->get_parameter("GPSZ", gpsz_);
    imuPgps_ = Pose3(Rot3(), Point3(gpsx_, gpsy_, gpsz_));
    imuPgps_.print("IMU->GPS");

    this->get_parameter("FixedInitialPose", fixedInitialPose_);

    this->get_parameter("initialRoll", initialRoll_);
    this->get_parameter("initialPitch", initialPitch_);
    this->get_parameter("initialYaw", initialYaw_);

    this->get_parameter("FixedOrigin", fixedOrigin_);

    this->get_parameter("latOrigin", latOrigin_);
    this->get_parameter("lonOrigin", lonOrigin_);
    this->get_parameter("altOrigin", altOrigin_);

    this->get_parameter("UseOdom", usingOdom_);
    this->get_parameter("MaxGPSError", maxGPSError_);

    if (fixedOrigin_) {
        enu_.Reset(latOrigin_, lonOrigin_, altOrigin_);
    }

    std::cout << "InitialRotationNoise: " << initialRotationNoise_ << std::endl
    << "InitialVelocityNoise: " << initialVelNoise_ << std::endl
    << "InitialBiasNoiseAcc: " << initialBiasNoiseAcc_ << std::endl
    << "InitialBiasNoiseGyro: " << initialBiasNoiseGyro_ << std::endl
    << "AccelerometerSigma: " << accSigma_ << std::endl
    << "GyroSigma: " << gyroSigma_ << std::endl
    << "AccelBiasSigma: " << accelBiasSigma_ << std::endl
    << "GyroBiasSigma: " << gyroBiasSigma_ << std::endl
    << "GPSSigma: " << gpsSigma_ << std::endl
    << "SensorTransformX: " << sensorX_ << std::endl
    << "SensorTransformY: " << sensorY_ << std::endl
    << "SensorTransformZ: " << sensorZ_ << std::endl
    << "SensorXAngle: " <<  sensorXAngle_ << std::endl
    << "SensorYAngle: " << sensorYAngle_ << std::endl
    << "SensorZAngle: " <<   sensorZAngle_ << std::endl
    << "CarXAngle: " <<  carXAngle_ << std::endl
    << "CarYAngle: " <<  carYAngle_ << std::endl
    << "CarZAngle: " <<  carZAngle_ << std::endl
    << "Gravity: " <<   gravityMagnitude_ << std::endl;

    // Use an ENU frame
    preintegrationParams_ = PreintegrationParams::MakeSharedU(gravityMagnitude_);
    preintegrationParams_->accelerometerCovariance = accSigma_ * I_3x3;
    preintegrationParams_->gyroscopeCovariance = gyroSigma_ * I_3x3;
    preintegrationParams_->integrationCovariance = 1e-5 * I_3x3;
    
    Vector biases((Vector(6) << 0, 0, 0, 0, 0, 0).finished());
    optimizedBias_ = imuBias::ConstantBias(biases);
    previousBias_ = imuBias::ConstantBias(biases);
    imuPredictor_ = boost::make_shared<PreintegratedImuMeasurements>(preintegrationParams_, optimizedBias_);
    
    optimizedTime_ = 0;
    
    // Subscriber
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tracked_pose", 10, std::bind(&StateEstimator::PoseCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&StateEstimator::ImuCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&StateEstimator::OdomCallback, this, std::placeholders::_1));

    // Publisher
    est_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/estimated_state", 10);
    gpsPosPub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gps_pos", 10);
    biasAccPub_ = this->create_publisher<geometry_msgs::msg::Point>("/bias_acc", 10);
    biasGyroPub_ = this->create_publisher<geometry_msgs::msg::Point>("/bias_gyro", 10);
    posePub_ = this->create_publisher<nav_msgs::msg::Odometry>("/pose", 10);
    timePub_ = this->create_publisher<geometry_msgs::msg::Point>("/time_delays", 10);

    // initialPose_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateEstimator::initialPose, this));

    // initialPose_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
    //     if (!initialPoseInitialized_ && !odom_received_) {
    //         initialPose();
    //     }
    //     else if (!initialPoseInitialized_ && odom_received_) {
    //         initialPose();
    //         initialPoseInitialized_ = true;
    //     }
    // });

    // gps_helper_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateEstimator::GpsHelper, this));

    initialPose_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (!initialPoseInitialized_ && !odom_received_) {
            initialPose();
        } else if (!initialPoseInitialized_ && odom_received_) {
            initialPose();
            initialPoseInitialized_ = true;

            gps_helper_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() {
                    GpsHelper();
                }
            );

            RCLCPP_INFO(this->get_logger(), "GPS Helper Timer started after Initial Pose");
        }
    });
}

StateEstimator::~StateEstimator() {
    // None
}

void StateEstimator::PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_msg_ = *msg;
    
    if (!gpsOptQ_.pushNonBlocking(msg)) {
        RCLCPP_WARN(this->get_logger(), "Dropping a Pose measurement due to full queue!!");
    }
}

void StateEstimator::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_msg_ = *msg;

    double dt;
    if (lastImuT_ == 0) {
        dt = 0.005;
    }
    else {
        dt = TIME(msg) - lastImuT_;
    }

    lastImuT_ = TIME(msg);

    // Push the IMU measurement to the optimization thread
    int qSize = imuOptQ_.size();

    if (qSize > maxQSize_) {
        maxQSize_ = qSize;
    }
    if (!imuOptQ_.pushNonBlocking(msg)) {
        RCLCPP_WARN(this->get_logger(), "Dropping an IMU measurement due to full queue!!");
    }

    // Each time we get an imu measurement, calculate the incremental pose from the last GTSAM pose
    imuMeasurements_.push_back(msg);

    //Grab the most current optimized state
    double optimizedTime;
    NavState optimizedState;
    imuBias::ConstantBias optimizedBias;

    {
      boost::mutex::scoped_lock guard(optimizedStateMutex_);
      optimizedState = optimizedState_;
      optimizedBias = optimizedBias_;
      optimizedTime = optimizedTime_;
    }

    if (optimizedTime == 0) {
        return; // haven't optimized first state yet
    }

    bool newMeasurements = false;
    int numImuDiscarded = 0;
    double imuQPrevTime;

    Vector3 acc, gyro;

    while (!imuMeasurements_.empty() && (TIME(imuMeasurements_.front()) < optimizedTime)) {
        imuQPrevTime = TIME(imuMeasurements_.front());
        imuMeasurements_.pop_front();
        newMeasurements = true;
        numImuDiscarded++;
    }

    if(newMeasurements) {
        // We need to reset integration and iterate through all our IMU measurements
        imuPredictor_->resetIntegration();

        int numMeasurements = 0;

        for (auto it=imuMeasurements_.begin(); it!=imuMeasurements_.end(); ++it) {
            double dt_temp =  TIME(*it) - imuQPrevTime;
            imuQPrevTime = TIME(*it);
            GetAccGyro(*it, acc, gyro);
            imuPredictor_->integrateMeasurement(acc, gyro, dt_temp);
            numMeasurements++;
            // RCLCPP_INFO(this->get_logger(), "IMU time %f, dt %f", rclcpp::Time((*it)->header.stamp).seconds(), dt_temp);
        }

        // RCLCPP_INFO(this->get_logger(), "Resetting Integration, %d measurements integrated, %d discarded", numMeasurements, numImuDiscarded);
    }
    else {
        // Just need to add the newest measurement, no new optimized pose
        GetAccGyro(msg, acc, gyro);
        imuPredictor_->integrateMeasurement(acc, gyro, dt);
        // RCLCPP_INFO(this->get_logger(), "Integrating %f, dt %f", m_lastImuT, dt);
    }

    // predict next state given the imu measurements
    NavState currentPose = imuPredictor_->predict(optimizedState, optimizedBias);
    
    nav_msgs::msg::Odometry poseNew;
    poseNew.header.stamp = msg->header.stamp;

    Vector4 q = currentPose.quaternion().coeffs();
    poseNew.pose.pose.orientation.x = q[0];
    poseNew.pose.pose.orientation.y = q[1];
    poseNew.pose.pose.orientation.z = q[2];
    poseNew.pose.pose.orientation.w = q[3];

    poseNew.pose.pose.position.x = currentPose.position().x();
    poseNew.pose.pose.position.y = currentPose.position().y();
    poseNew.pose.pose.position.z = currentPose.position().z();

    poseNew.twist.twist.linear.x = currentPose.velocity().x();
    poseNew.twist.twist.linear.y = currentPose.velocity().y();
    poseNew.twist.twist.linear.z = currentPose.velocity().z();
    
    poseNew.twist.twist.angular.x = gyro.x() + optimizedBias.gyroscope().x();
    poseNew.twist.twist.angular.y = gyro.y() + optimizedBias.gyroscope().y();
    poseNew.twist.twist.angular.z = gyro.z() + optimizedBias.gyroscope().z();

    poseNew.child_frame_id = "base_link";
    poseNew.header.frame_id = "odom";

    posePub_->publish(poseNew);

    geometry_msgs::msg::Point delays;
    delays.x = TIME(msg);
    delays.y = (this->get_clock()->now() - rclcpp::Time(msg->header.stamp)).seconds();
    delays.z = TIME(msg) - optimizedTime;

    timePub_->publish(delays);

    return;
}

void StateEstimator::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_msg_ = *msg;
    odom_received_ = true;
    
    if (!odomOptQ_.pushNonBlocking(msg) && usingOdom_) {
        RCLCPP_WARN(this->get_logger(), "Dropping an wheel odometry measurement due to full queue!!");
    }
}

void StateEstimator::initialPose() {
    if (!fixedInitialPose_) {
        if (!odom_received_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for valid initial orientation");
            return;
        }

        initialPose_.orientation.w = pose_msg_.pose.orientation.w;
        initialPose_.orientation.x = pose_msg_.pose.orientation.x;
        initialPose_.orientation.y = pose_msg_.pose.orientation.y;
        initialPose_.orientation.z = pose_msg_.pose.orientation.z;
        initialPose_.bias.x = 0.0;
        initialPose_.bias.y = 0.0;
        initialPose_.bias.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Initial pose set from odometry.");
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Using fixed initial orientation");

        Rot3 initialRotation = Rot3::Ypr(initialYaw_, initialPitch_, initialRoll_);

        initialPose_.orientation.w = initialRotation.toQuaternion().w();
        initialPose_.orientation.x = initialRotation.toQuaternion().x();
        initialPose_.orientation.y = initialRotation.toQuaternion().y();
        initialPose_.orientation.z = initialRotation.toQuaternion().z();
        initialPose_.bias.x = 0.0;
        initialPose_.bias.y = 0.0;
        initialPose_.bias.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Fixed initial orientation applied.");
    }

    Rot3 initRot(Quaternion(initialPose_.orientation.w, initialPose_.orientation.x, initialPose_.orientation.y, initialPose_.orientation.z));

    bodyPSensor_ = Pose3(Rot3::RzRyRx(sensorXAngle_, sensorYAngle_, sensorZAngle_), Point3(sensorX_, sensorY_, sensorZ_));
    carENUPcarNED_ = Pose3(Rot3::RzRyRx(carXAngle_, carYAngle_, carZAngle_), Point3());

    bodyPSensor_.print("Body pose\n");
    carENUPcarNED_.print("CarBodyPose\n");

    ISAM2Params params;
    params.factorization = ISAM2Params::QR;
    isam_ = new ISAM2(params);

    // prior on the first pose
    priorNoisePose_ = noiseModel::Diagonal::Sigmas(
        (Vector(6) << initialRotationNoise_, initialRotationNoise_, 3*initialRotationNoise_,
            gpsSigma_, gpsSigma_, gpsSigma_).finished());

    // Add velocity prior
    priorNoiseVel_ = noiseModel::Diagonal::Sigmas(
        (Vector(3) << initialVelNoise_, initialVelNoise_, initialVelNoise_).finished());

    // Add bias prior
    priorNoiseBias_ = noiseModel::Diagonal::Sigmas(
        (Vector(6) << initialBiasNoiseAcc_,
            initialBiasNoiseAcc_,
            initialBiasNoiseAcc_,
            initialBiasNoiseGyro_,
            initialBiasNoiseGyro_,
            initialBiasNoiseGyro_).finished());

    std::cout<<"checkpoint"<<std::endl;

    Vector sigma_acc_bias_c(3), sigma_gyro_bias_c(3);
    sigma_acc_bias_c << accelBiasSigma_,  accelBiasSigma_,  accelBiasSigma_;
    sigma_gyro_bias_c << gyroBiasSigma_, gyroBiasSigma_, gyroBiasSigma_;
    noiseModelBetweenBias_sigma_ = (Vector(6) << sigma_acc_bias_c, sigma_gyro_bias_c).finished();

    std::cout << "Initialization complete." << std::endl;
}

void StateEstimator::GpsHelper() {
    bool optimize = false;

    if (!gotFirstFix_) {
        std::shared_ptr<geometry_msgs::msg::PoseStamped> fix = gpsOptQ_.popBlocking();

        startTime = TIME(fix);

        if(imuOptQ_.size() <= 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,  // 1000ms
                "no IMU messages before first fix, continuing until there is");
            return;
        }

        // errors out if the IMU and GPS are not close in timestamps
        double most_recent_imu_time = imuOptQ_.back()->header.stamp.sec + imuOptQ_.back()->header.stamp.nanosec * 1e-9;
        
        if(std::abs(most_recent_imu_time - startTime) > 0.1) {
            RCLCPP_ERROR(
                this->get_logger(),
                "There is a very large difference in the GPS and IMU timestamps: %f", most_recent_imu_time - startTime);
            exit(-1);
        }

        if(usingOdom_) {
            lastOdom_ = odomOptQ_.popBlocking();
        }

        NonlinearFactorGraph newFactors;
        Values newVariables;

        gotFirstFix_ = true;

        double E, N, U;
        
        // if (!fixedOrigin_)
        // {
        //     enu_.Reset(fix->latitude, fix->longitude, fix->altitude);
        //     E = 0; N = 0; U = 0; // we're choosing this as the origin
        // }
        // else
        // {
        //     // we are given an origin
        //     enu_.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);
        // }

        E = fix->pose.position.x;
        N = fix->pose.position.y;
        U = fix->pose.position.z;

        // Add prior factors on pose, vel and bias
        Rot3 initialOrientation = Rot3::Quaternion(initialPose_.orientation.w,
            initialPose_.orientation.x,
            initialPose_.orientation.y,
            initialPose_.orientation.z);

        std::cout << "Initial orientation" << std::endl;
        std::cout << bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation() << std::endl;

        Pose3 x0(bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation(), Point3(E, N, U));
        prevPose = x0;
        PriorFactor<Pose3> priorPose(X(0), x0, priorNoisePose_);
        newFactors.add(priorPose);
        PriorFactor<Vector3> priorVel(V(0), Vector3(0, 0, 0), priorNoiseVel_);
        newFactors.add(priorVel);
        Vector biases((Vector(6) << 0, 0, 0, initialPose_.bias.x, -initialPose_.bias.y, -initialPose_.bias.z).finished());
        prevBias = imuBias::ConstantBias(biases);
        PriorFactor<imuBias::ConstantBias> priorBias(B(0), imuBias::ConstantBias(biases), priorNoiseBias_);
        newFactors.add(priorBias);

        //Factor for imu->gps translation
        BetweenFactor<Pose3> imuPgpsFactor(X(0), G(0), imuPgps_, noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
        newFactors.add(imuPgpsFactor);

        // add prior values on pose, vel and bias
        newVariables.insert(X(0), x0);
        newVariables.insert(V(0), Vector3(0, 0, 0));
        newVariables.insert(B(0), imuBias::ConstantBias(biases));
        newVariables.insert(G(0), x0.compose(imuPgps_));

        isam_->update(newFactors, newVariables);
        //Read IMU measurements up to the first GPS measurement
        lastIMU_ = imuOptQ_.popBlocking();
        //If we only pop one, we need some dt
        lastImuTgps_ = TIME(lastIMU_) - 0.005;

        while(TIME(lastIMU_) < TIME(fix)) {
            lastImuTgps_ = TIME(lastIMU_);
            lastIMU_ = imuOptQ_.popBlocking();
        }
    }
    else {
        NonlinearFactorGraph newFactors;
        Values newVariables;

        // add IMU measurements
        while (imuOptQ_.size() > 0 && (TIME(imuOptQ_.back()) > (startTime + imuKey * 0.1))) {
            double curTime = startTime + imuKey * 0.1;

            PreintegratedImuMeasurements pre_int_data(preintegrationParams_, previousBias_);

            while(TIME(lastIMU_) < curTime) {
                Vector3 acc, gyro;

                GetAccGyro(lastIMU_, acc, gyro);

                double imuDT = TIME(lastIMU_) - lastImuTgps_;
                lastImuTgps_ = TIME(lastIMU_);

                pre_int_data.integrateMeasurement(acc, gyro, imuDT);
                lastIMU_ = imuOptQ_.popBlocking();
            }

            // adding the integrated IMU measurements to the factor graph
            ImuFactor imuFactor(X(imuKey-1), V(imuKey-1), X(imuKey), V(imuKey), B(imuKey-1), pre_int_data);
            newFactors.add(imuFactor);
            newFactors.add(BetweenFactor<imuBias::ConstantBias>(B(imuKey-1), B(imuKey), imuBias::ConstantBias(), noiseModel::Diagonal::Sigmas( sqrt(pre_int_data.deltaTij()) * noiseModelBetweenBias_sigma_)));

            // Predict forward to get an initial estimate for the pose and velocity
            NavState curNavState(prevPose, prevVel);
            NavState nextNavState = pre_int_data.predict(curNavState, prevBias);
            newVariables.insert(X(imuKey), nextNavState.pose());
            newVariables.insert(V(imuKey), nextNavState.v());
            newVariables.insert(B(imuKey), previousBias_);
            newVariables.insert(G(imuKey), nextNavState.pose().compose(imuPgps_));
            prevPose = nextNavState.pose();
            prevVel = nextNavState.v();
            ++imuKey;
            optimize = true;
        }

        // add GPS measurements that are not ahead of the imu messages
        while (optimize && gpsOptQ_.size() > 0 && TIME(gpsOptQ_.front()) < (startTime + (imuKey-1)*0.1 + 1e-2)) {
            std::shared_ptr<geometry_msgs::msg::PoseStamped> fix = gpsOptQ_.popBlocking();

            double timeDiff = (TIME(fix) - startTime) / 0.1;
            int key = round(timeDiff);

            if (std::abs(timeDiff - key) < 1e-4) {
                // this is a gps message for a factor
                latestGPSKey = key;

                double E,N,U;

                E = fix->pose.position.x;
                N = fix->pose.position.y;
                U = fix->pose.position.z;

                // check if the GPS message is close to our expected position
                Pose3 expectedState;

                if (newVariables.exists(X(key))) {
                    expectedState = (Pose3) newVariables.at<Pose3>(X(key));
                }
                else {
                    expectedState = isam_->calculateEstimate<Pose3>(X(key));
                }

                double dist = std::sqrt( std::pow(expectedState.x() - E, 2) + std::pow(expectedState.y() - N, 2) );

                if (dist < maxGPSError_ || latestGPSKey < imuKey-2) {
                    geometry_msgs::msg::PoseWithCovarianceStamped point;
                    point.header.stamp = this->get_clock()->now();
                    point.header.frame_id = "odom";
                    point.pose.pose.position.x = E;
                    point.pose.pose.position.y = N;
                    // point.pose.pose.position.z = U;
                    // point.pose.covariance[0] = fix->position_covariance[0];
                    // point.pose.covariance[7] = fix->position_covariance[4];
                    point.pose.covariance[0] = 0.0;
                    point.pose.covariance[7] = 0.0;
                    gpsPosPub_->publish(point);

                    SharedDiagonal gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(gpsSigma_, gpsSigma_, 3.0 * gpsSigma_));
                    GPSFactor gpsFactor(G(key), Point3(E, N, U), gpsNoise);
                    newFactors.add(gpsFactor);
                    BetweenFactor<Pose3> imuPgpsFactor(X(key), G(key), imuPgps_,
                        noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
                    newFactors.add(imuPgpsFactor);

                    if (!usingOdom_) {
                        odomKey = key+1;
                    }
                }
                else {
                    RCLCPP_WARN(this->get_logger(), "Received bad GPS message");
                    // diag_warn("Received bad GPS message");
                }
            }
        }

        // if only using odom with no GPS, then remove old messages from queue
        while (!usingOdom_ && odomOptQ_.size() > 0 && TIME(odomOptQ_.front()) < (odomKey*0.1 + startTime)) {
            lastOdom_ = odomOptQ_.popBlocking();
        }

        // if available, add any odom factors that are not ahead of the imu messages
        while ((usingOdom_ || latestGPSKey < imuKey-2) && optimize && odomKey < imuKey && odomOptQ_.size() > 0 && (TIME(odomOptQ_.back()) > (startTime + odomKey * 0.1))) {
            double prevTime = startTime + (odomKey-1) * 0.1;
            newFactors.add(integrateWheelOdom(prevTime, prevTime+0.1, odomKey++));
        }

        // if we processed imu - then we can optimize the state
        if (optimize) {
            try {
                isam_->update(newFactors, newVariables);
                Pose3 nextState = isam_->calculateEstimate<Pose3>(X(imuKey-1));

                prevPose = nextState;
                prevVel = isam_->calculateEstimate<Vector3>(V(imuKey-1));
                prevBias = isam_->calculateEstimate<imuBias::ConstantBias>(B(imuKey-1));

                // if we haven't added gps data for 2 message (0.2s) then change status
                if (latestGPSKey + 3 < imuKey) {
                    RCLCPP_WARN(this->get_logger(), "No gps");
                    // diag_warn("No gps");
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "Still ok!");
                    // diag_ok("Still ok!");
                }

                double curTime = startTime + (imuKey-1) * 0.1;

                {
                    boost::mutex::scoped_lock guard(optimizedStateMutex_);
                    optimizedState_ = NavState(prevPose, prevVel);
                    optimizedBias_ = prevBias;
                    optimizedTime_ = curTime;
                }

                nav_msgs::msg::Odometry poseNew;
                poseNew.header.stamp.sec = static_cast<int32_t>(curTime);
                poseNew.header.stamp.nanosec = static_cast<uint32_t>((curTime - static_cast<int32_t>(curTime)) * 1e9);

                geometry_msgs::msg::Point ptAcc;
                ptAcc.x = prevBias.vector()[0];
                ptAcc.y = prevBias.vector()[1];
                ptAcc.z = prevBias.vector()[2];

                geometry_msgs::msg::Point ptGyro;
                ptGyro.x = prevBias.vector()[3];
                ptGyro.y = prevBias.vector()[4];
                ptGyro.z = prevBias.vector()[5];

                biasAccPub_->publish(ptAcc);
                biasGyroPub_->publish(ptGyro);
            }
            catch(gtsam::IndeterminantLinearSystemException ex) {
                RCLCPP_ERROR(this->get_logger(), "Encountered Indeterminant System Error!");
                // diag_error("State estimator has encountered indeterminant system error");
                {
                    boost::mutex::scoped_lock guard(optimizedStateMutex_);
                }
            }
        }
    }
}

void StateEstimator::GetAccGyro(const sensor_msgs::msg::Imu::SharedPtr& imu, gtsam::Vector3 &acc, gtsam::Vector3 &gyro) {
    double accx, accy, accz;

    if (invertx_) accx = -imu->linear_acceleration.x;
    else accx = imu->linear_acceleration.x;
    if (inverty_) accy = -imu->linear_acceleration.y;
    else accy = imu->linear_acceleration.y;
    if (invertz_) accz = -imu->linear_acceleration.z;
    else accz = imu->linear_acceleration.z;

    acc = Vector3(accx, accy, accz);

    double gx, gy, gz;

    if (invertx_) gx = -imu->angular_velocity.x;
    else gx = imu->angular_velocity.x;
    if (inverty_) gy = -imu->angular_velocity.y;
    else gy = imu->angular_velocity.y;
    if (invertz_) gz = -imu->angular_velocity.z;
    else gz = imu->angular_velocity.z;

    gyro = Vector3(gx, gy, gz);
}

BetweenFactor<Pose3> StateEstimator::integrateWheelOdom(double prevTime, double stopTime, int curKey) {
    double x=0, y=0, theta=0, xVar=0, yVar=0, zVar=0, thetaVariance=0, dt=0, lastTimeUsed=prevTime;

    while (lastTimeUsed != stopTime) {
        if (odomOptQ_.size() != 0 && TIME(odomOptQ_.front()) < stopTime) {
            lastOdom_ = odomOptQ_.popBlocking();
            dt = TIME(lastOdom_) - lastTimeUsed;
            lastTimeUsed = TIME(lastOdom_);
        }
        else {
            dt = stopTime - lastTimeUsed;
            lastTimeUsed = stopTime;
        }

        // the local frame velocities
        double vx = lastOdom_->twist.twist.linear.x;
        double vy = lastOdom_->twist.twist.linear.y;

        // update the relative position from the initial
        x += vx*dt*cos(theta) - vy*dt*sin(theta);
        y += vx*dt*sin(theta) + vy*dt*cos(theta);
        theta += dt*lastOdom_->twist.twist.angular.z;
        xVar += dt * lastOdom_->twist.covariance[0];
        yVar += dt * lastOdom_->twist.covariance[7];
        zVar += dt * lastOdom_->twist.covariance[14];
        thetaVariance += dt*lastOdom_->twist.covariance[35];
    }

    Pose3 betweenPose = Pose3(Rot3::Rz(theta), Point3(x, y, 0.0));

    return BetweenFactor<Pose3>(X(curKey-1), X(curKey), betweenPose, noiseModel::Diagonal::Sigmas(
            (Vector(6) << thetaVariance*2,thetaVariance*2,thetaVariance,xVar,yVar,zVar).finished()));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimator>());
    rclcpp::shutdown();
    return 0;
}
