// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_TaiYiDroneController_hpp
#define msr_airlib_TaiYiDroneController_hpp

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

// Sensors
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/distance/DistanceSimple.hpp"
#include "sensors/lidar/LidarSimple.hpp"

#include "UdpSocket.hpp"

#include <sstream>

namespace msr
{
namespace airlib
{

    class SkymagicVehicleControllerApi : public MultirotorApiBase
    {

    public:
        SkymagicVehicleControllerApi(const MultiRotorParams* vehicle_params, const AirSimSettings::VehicleSetting& vehicle_setting,
                                     const AirSimSettings::SkyMagicVehiclConnectInfo& connection_info)
            : vehicle_params_(vehicle_params), vehicle_setting_(vehicle_setting), connection_info_(connection_info)
        {
            sensors_ = &getSensors();
            if (connection_info_.model == "SKMX300C8") {
                 actuator_upper_limit_ = 2047.0F;
                 actuator_lower_limit_ = 0.0f;
            }
            else {
                // TYMini
                actuator_upper_limit_ = 300.0F;
                actuator_lower_limit_ = 0.0f;
            }
        }

        ~SkymagicVehicleControllerApi()
        {
            closeConnections();
        }

    public:
        void initialize(void)
        {
            try {
                createConnection();
                is_ready_ = true;
            }
            catch (std::exception& ex) {
                std::cout << "Failed to connect: " << ex.what() << std::endl;
            }
        }

        virtual void resetImplementation() override
        {
            MultirotorApiBase::resetImplementation();

            // Reset state
        }

        // Update sensor data & send to flight controller
        virtual void update() override
        {
            MultirotorApiBase::update();

            //sendSensors();
            //recvRotorControl();
        }

        // TODO:VehicleApiBase implementation
        virtual bool isApiControlEnabled() const override
        {
            Utils::log("Not Implemented: isApiControlEnabled", Utils::kLogLevelInfo);
            return false;
        }
        virtual void enableApiControl(bool is_enabled) override
        {
            Utils::log("Not Implemented: enableApiControl", Utils::kLogLevelInfo);
            unused(is_enabled);
        }
        virtual bool armDisarm(bool arm) override
        {
            Utils::log("Not Implemented: armDisarm", Utils::kLogLevelInfo);
            unused(arm);
            return false;
        }
        virtual GeoPoint getHomeGeoPoint() const override
        {
            //Utils::log("Not Implemented: getHomeGeoPoint", Utils::kLogLevelInfo);
            //return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
            return environment_->getHomeGeoPoint();
        }
        virtual void getStatusMessages(std::vector<std::string>& messages) override
        {
            unused(messages);
        }

        virtual const SensorCollection& getSensors() const override
        {
            return vehicle_params_->getSensors();
        }

    public: //TODO:MultirotorApiBase implementation
        virtual real_T getActuation(unsigned int rotor_index) const override
        {
            return rotor_controls_[rotor_index];
        }

        virtual size_t getActuatorCount() const override
        {
            return vehicle_params_->getParams().rotor_count;
        }

        virtual void moveByRC(const RCData& rc_data) override
        {
            setRCData(rc_data);
        }

        virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment) override
        {
            Utils::log("Not Implemented: setSimulatedGroundTruth", Utils::kLogLevelInfo);
            //unused(kinematics);
            //unused(environment);
            kinematics_ = kinematics;
            environment_ = environment;
        }

        virtual bool setRCData(const RCData& rc_data) override
        {
            last_rcData_ = rc_data;
            is_rc_connected_ = false;//true;

            return true;
        }

    protected:
        virtual Kinematics::State getKinematicsEstimated() const override
        {
            Kinematics::State state;
            state = *kinematics_;
            return state;
        }

        virtual Vector3r getPosition() const override
        {
            return kinematics_->pose.position;
        }

        virtual Vector3r getVelocity() const override
        {
            return kinematics_->twist.linear;
        }

        virtual Quaternionr getOrientation() const override
        {
            return kinematics_->pose.orientation;
        }

        virtual LandedState getLandedState() const override
        {
            Utils::log("Not Implemented: getLandedState", Utils::kLogLevelInfo);
            return LandedState::Landed;
        }

        virtual RCData getRCData() const override
        {
            //return what we received last time through setRCData
            return last_rcData_;
        }

        virtual GeoPoint getGpsLocation() const override
        {
            return environment_->getState().geo_point;
        }

        virtual float getCommandPeriod() const override
        {
            return 1.0f / 50; //50hz
        }

        virtual float getTakeoffZ() const override
        {
            // pick a number, 3 meters is probably safe
            // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
            // return params_.takeoff.takeoff_z;
            return -5.0;
        }

        virtual float getDistanceAccuracy() const override
        {
            return 0.5f; //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
        }

        virtual void setControllerGains(uint8_t controllerType, const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) override
        {
            unused(controllerType);
            unused(kp);
            unused(ki);
            unused(kd);
            Utils::log("Not Implemented: setControllerGains", Utils::kLogLevelInfo);
        }

        virtual void commandMotorPWMs(float front_right_pwm, float front_left_pwm, float rear_right_pwm, float rear_left_pwm) override
        {
            unused(front_right_pwm);
            unused(front_left_pwm);
            unused(rear_right_pwm);
            unused(rear_left_pwm);
            Utils::log("Not Implemented: commandMotorPWMs", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw_rate);
            unused(throttle);
            Utils::log("Not Implemented: commandRollPitchYawrateThrottle", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawZ(float roll, float pitch, float yaw, float z) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw);
            unused(z);
            Utils::log("Not Implemented: commandRollPitchYawZ", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw);
            unused(throttle);
            Utils::log("Not Implemented: commandRollPitchYawThrottle", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw_rate);
            unused(z);
            Utils::log("Not Implemented: commandRollPitchYawrateZ", Utils::kLogLevelInfo);
        }

        virtual void commandAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z) override
        {
            unused(roll_rate);
            unused(pitch_rate);
            unused(yaw_rate);
            unused(z);
            Utils::log("Not Implemented: commandAngleRatesZ", Utils::kLogLevelInfo);
        }

        virtual void commandAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle) override
        {
            unused(roll_rate);
            unused(pitch_rate);
            unused(yaw_rate);
            unused(throttle);
            Utils::log("Not Implemented: commandAngleRatesZ", Utils::kLogLevelInfo);
        }

        virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
        {
            unused(vx);
            unused(vy);
            unused(vz);
            unused(yaw_mode);
            Utils::log("Not Implemented: commandVelocity", Utils::kLogLevelInfo);
        }

        virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
        {
            unused(vx);
            unused(vy);
            unused(z);
            unused(yaw_mode);
            Utils::log("Not Implemented: commandVelocityZ", Utils::kLogLevelInfo);
        }

        virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
        {
            unused(x);
            unused(y);
            unused(z);
            unused(yaw_mode);
            Utils::log("Not Implemented: commandPosition", Utils::kLogLevelInfo);
        }

        virtual const MultirotorApiParams& getMultirotorApiParams() const override
        {
            return safety_params_;
        }

        //*** End: MultirotorApiBase implementation ***//

    protected:
        void createConnection()
        {
            connect();
            is_close_ = false;
            this->tx_thread_ = std::thread(&SkymagicVehicleControllerApi::tx_thread, this);
            this->rx_thread_ = std::thread(&SkymagicVehicleControllerApi::rx_thread, this);
        }

        void tx_thread()
        {
            while (!is_close_) {
                sendKinematics();
            }
        }
        void rx_thread()
        {
            while (!is_close_) {
                recvRotorControl();
            }
        }

        void closeConnections()
        {
            if (udp_socket_ != nullptr) {
                udp_socket_->close();
                is_close_ = true;

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                if (tx_thread_.joinable()) {
                    tx_thread_.join();
                }

                if (rx_thread_.joinable()) {
                    rx_thread_.join();
                }
            }
        }

        void connect()
        {
            port_ = static_cast<uint16_t>(connection_info_.udp_port);
            ip_ = connection_info_.udp_address;

            closeConnections();

            if (ip_ == "") {
                throw std::invalid_argument("UdpIp setting is invalid.");
            }

            if (port_ == 0) {
                throw std::invalid_argument("UdpPort setting has an invalid value.");
            }

            Utils::log(Utils::stringf("Using UDP port %d, local IP %s, remote IP %s for sending sensor data", port_, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);
            Utils::log(Utils::stringf("Using UDP port %d for receiving rotor power", connection_info_.control_port_local, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);

            udp_socket_ = std::make_unique<mavlinkcom::UdpSocket>();
            udp_socket_->bind(connection_info_.local_host_ip, connection_info_.control_port_local);
        }

    private:
        void recvRotorControl()
        {
            SITLRotorControlMessage srcm;
            int recv_ret = udp_socket_->recv(&srcm, sizeof(srcm), 1);
            if (recv_ret == sizeof(srcm)) {
                for (auto i = 0; i < kTyFlightControllerRotorControlCount; ++i) {
                    motor_ctrl_.pwm[i] = srcm.pwm[i];
                }
                normalizeRotorControls();
            }
            else {
                if (recv_ret <= 0) {
                    Utils::log(Utils::stringf("Error while receiving rotor control data - ErrorNo: %d", recv_ret), Utils::kLogLevelInfo);
                }
                else {
                    Utils::log(Utils::stringf("Received %d bytes instead of %zu bytes", recv_ret, sizeof(srcm)), Utils::kLogLevelInfo);
                    
                }
            }
        }

        virtual void normalizeRotorControls()
        {
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = (motor_ctrl_.pwm[i]) / (actuator_upper_limit_ - actuator_lower_limit_);
            }
        }

        void sendKinematics()
        {
            SITLKinematicsMessage skm;
            
            auto now = clock()->nowNanos() / 1000;
            if (last_sendKinematics_time_ + 2000 < now) {
                last_sendKinematics_time_ = now;
                skm.timestamp = (uint32_t) (last_sendKinematics_time_);
                skm.ned_pos[0] = kinematics_->pose.position.x();
                skm.ned_pos[1] = kinematics_->pose.position.y();
                skm.ned_pos[2] = kinematics_->pose.position.z();
                skm.ned_vel[0] = kinematics_->twist.linear.x();
                skm.ned_vel[1] = kinematics_->twist.linear.y();
                skm.ned_vel[2] = kinematics_->twist.linear.z();
                skm.ned_acc[0] = kinematics_->accelerations.linear.x();
                skm.ned_acc[1] = kinematics_->accelerations.linear.y();
                skm.ned_acc[2] = kinematics_->accelerations.linear.z();
                VectorMath::toEulerianAngle(kinematics_->pose.orientation, skm.rpy[1], skm.rpy[0], skm.rpy[2]);
                skm.pqr[0] = kinematics_->twist.angular.x();
                skm.pqr[1] = kinematics_->twist.angular.y();
                skm.pqr[2] = kinematics_->twist.angular.z();
                skm.serial = ++send_serial_;
                udp_socket_->sendto(&skm, sizeof(skm), ip_, port_);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            
        }

    private:
#ifdef __linux__
        struct __attribute__((__packed__)) SITLKinematicsMessage
        {
#else
#pragma pack(push, 1)
        struct SITLKinematicsMessage
        {
#endif
            // this is the packet sent by the simulator
            // to the flight controller to update the simulator state
            // All values are little-endian
            uint32_t timestamp; //Timestamp (ms)
            float ned_pos[3]; //position (m)£¬>=100Hz
            float ned_vel[3]; //NED velocity (m/s)£¬>=100Hz
            float ned_acc[3]; //NED acceleration (m*m/s)£¬>=100Hz
            float rpy[3]; //Angles, attitude (rad)£¬>=500Hz
            float pqr[3]; //Angular velocity (rad/s)£¬>=500Hz
            float quat[4]; //Quaternion£¬>=500Hz
            uint32_t serial;
        };
#ifndef __linux__
#pragma pack(pop)
#endif
        static const int kTyFlightControllerRotorControlCount = 8;

        struct SITLRotorControlMessage
        {
            uint16_t pwm[kTyFlightControllerRotorControlCount];
        };


        std::unique_ptr<mavlinkcom::UdpSocket> udp_socket_;

        AirSimSettings::SkyMagicVehiclConnectInfo connection_info_;
        uint16_t port_;
        std::string ip_;
        const AirSimSettings::VehicleSetting vehicle_setting_;
        const SensorCollection* sensors_;
        const MultiRotorParams* vehicle_params_;

        MultirotorApiParams safety_params_;

        RCData last_rcData_;

        std::mutex contorl_mutex_;
        bool is_rc_connected_;
        bool is_ready_ = false;
        bool is_close_ = true;
        uint64_t last_sendKinematics_time_ = 0;

        std::thread tx_thread_;
        std::thread rx_thread_;

        float actuator_upper_limit_ = 300.0f;
        float actuator_lower_limit_ = 25.0f;
        SITLRotorControlMessage motor_ctrl_ = { 0 };
        float rotor_controls_[kTyFlightControllerRotorControlCount];

        const Kinematics::State* kinematics_;
        
        const Environment* environment_;

        uint32_t send_serial_;
    };
}
} //namespace
#endif
