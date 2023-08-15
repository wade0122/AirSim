// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_TYFlightController_hpp
#define msr_airlib_vehicles_TYFlightController_hpp

#include "vehicles/multirotor/firmwares/skymagic_flight_controller/SkyMagicFlightControllerApi.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr
{
namespace airlib
{

    class TyFlightControllerParams : public MultiRotorParams
    {
    public:
        TyFlightControllerParams(const AirSimSettings::SkyMagicVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
            : vehicle_setting_(vehicle_setting), sensor_factory_(sensor_factory)
        {
            connection_info_ = getConnectionInfo(vehicle_setting);
        }

        virtual ~TyFlightControllerParams() = default;

        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
        {
            //return std::unique_ptr<MultirotorApiBase>(new TyFlightControllerApi(this, connection_info_));
            unique_ptr<MultirotorApiBase> api(new SkymagicVehicleControllerApi(this, vehicle_setting_, connection_info_));
            auto api_ptr = static_cast<SkymagicVehicleControllerApi*>(api.get());
            api_ptr->initialize();
            return api;
        }

    protected:
        virtual void setupParams() override
        {
            auto& params = getParams();
            if (connection_info_.model == "SKMX300C8")
            {
                setupFrameGenericSKMX300C8(params);
            }
            else
                setupFrameGenericSKMMini(params);
            
        }

        virtual const SensorFactory* getSensorFactory() const override
        {
            return sensor_factory_.get();
        }

        static const AirSimSettings::SkyMagicVehiclConnectInfo& getConnectionInfo(const AirSimSettings::SkyMagicVehicleSetting& vehicle_setting)
        {
            return vehicle_setting.connection_info;
        }

    private:
        AirSimSettings::SkyMagicVehiclConnectInfo connection_info_;
        const AirSimSettings::VehicleSetting vehicle_setting_;
        std::shared_ptr<const SensorFactory> sensor_factory_;
    };
}
} //namespace
#endif
