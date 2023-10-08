/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Floater3V.h"

#if 1
#if AP_FLOATER3V_ENABLED

#include <utility>
#include <stdio.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>



extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Floater3V::var_info[] = {
    // NOTE: Index numbers 0 and 1 were for the old integer
    // ground temperature and pressure

    // @Param: _FLT_ID
    // @DisplayName: Floater ID to receive can messages
    // @Description: This selects which barometer will be measuring internal pressure
    // @Values: 0:First,1:2nd,2:3rd
    // @User: Advanced
    AP_GROUPINFO("_FLT_ID", 1, AP_Floater3V, instance, 0),

    
    // @Param: _SNS_PIN
    // @DisplayName: Level sensor pin
    // @Description: This selects pin which measures signal
    // @Values: 0:gpio0,1:gpio1,2:gpio2
    // @User: Advanced
    AP_GROUPINFO("_SRV1", 2, AP_Floater3V, srv1Channel, 0),

    // @Param: _SNS_PIN
    // @DisplayName: Level sensor pin
    // @Description: This selects pin which measures signal
    // @Values: 0:gpio0,1:gpio1,2:gpio2
    // @User: Advanced
    AP_GROUPINFO("_SRV2", 3, AP_Floater3V, srv2Channel, 1),

    // @Param: _INT
    // @DisplayName: Internal pressure sensor instance
    // @Description: This selects which barometer will be measuring internal pressure
    // @Values: 0:FirstBaro,1:2ndBaro,2:3rdBaro
    // @User: Advanced
    AP_GROUPINFO("_INT", 4, AP_Floater3V, baro_internal, 0),

    // @Param: _EXT
    // @DisplayName: External pressure sensor instance
    // @Description: This selects which barometer will be measuring external pressure
    // @Values: 0:FirstBaro,1:2ndBaro,2:3rdBaro
    // @User: Advanced
    AP_GROUPINFO("_EXT", 5, AP_Floater3V, baro_external, 1),

    // @Param: _VDS_PIN
    // @DisplayName: VSD pin
    // @Description: This selects pin which controls VDS
    // @Values: 0:gpio0,1:gpio1,2:gpio2
    // @User: Advanced
    AP_GROUPINFO("_VDS_PIN", 6, AP_Floater3V, vds_pin, -1),

    // @Param: _SNS_PIN
    // @DisplayName: Level sensor pin
    // @Description: This selects pin which measures signal
    // @Values: 0:gpio0,1:gpio1,2:gpio2
    // @User: Advanced
    AP_GROUPINFO("_SNS_PIN", 7, AP_Floater3V, level_sensor_pin, -1),

    AP_GROUPEND
};

// singleton instance
AP_Floater3V *AP_Floater3V::_singleton;
HAL_Semaphore AP_Floater3V::_sem_registry;

#if HAL_GCS_ENABLED
#define FLOATER_SEND_TEXT(severity, format, args...) gcs().send_text(severity, format, ##args)
#else
#define FLOATER_SEND_TEXT(severity, format, args...)
#endif



Valve::Valve(valve_type_t _type, uint8_t _instance) :
type(_type),
instance(_instance){
}
void Valve::open(uint32_t now, uint32_t time){
    closeTime = now + time;
    open();
}
void Valve::open(){
    if (type == valve_type_t::HIWONDER){
        SRV_Channels::set_output_pwm_chan(instance, 2000);
    }
    if (type == valve_type_t::ANALOG){
        hal.gpio->pinMode(instance, HAL_GPIO_OUTPUT);
        hal.gpio->write(instance, 1);
    }
}

void Valve::close(){
    if (type == valve_type_t::HIWONDER){
        SRV_Channels::set_output_pwm_chan(instance, 1000);
    }
    if (type == valve_type_t::ANALOG){
        hal.gpio->pinMode(instance, HAL_GPIO_OUTPUT);
        hal.gpio->write(instance, 0);
    }
}

void Valve::update(uint32_t now){
    if (now > closeTime)
        close();
}




/*
  AP_Floater3V constructor
 */
AP_Floater3V::AP_Floater3V()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_Floater3V::detectLevelSensor()
{
    if (level_sensor_pin != -1){
        floaterSensor = hal.analogin->channel(level_sensor_pin);
        if (floaterSensor == nullptr){
            return false;
        }
        return true;
    }
    return false;
}



void AP_Floater3V::updateLevelReading()
{
    if (detectLevelSensor()){
        levelReading = floaterSensor->voltage_average();
    }
}
/*
  initialise the barometer object, loading backend drivers
 */
void AP_Floater3V::init(void)
{
    valves[TOP] = new Valve(valve_type_t::HIWONDER, srv1Channel);
    valves[BOTTOM] = new Valve(valve_type_t::HIWONDER, srv1Channel);
    valves[AIR] = new Valve(valve_type_t::ANALOG, vds_pin);
}


/*
  call update on all drivers
 */
void AP_Floater3V::update(void)
{
    WITH_SEMAPHORE(_rsem);
    if (!initialised) {
        initialised = true;
        init();
        return;
    }
    
    uint32_t now = AP_HAL::millis();
    updateLevelReading();

    for (uint8_t i = 0; i < VALVE_NUMBER; i++)
    {
        valves[i]->update(now);
    }

}

#if AP_DRONECAN_SNOWSTORM_SUPPORT
void AP_Floater3V::handle_floater(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    com_snowstorm_Pressure req;
    if (com_snowstorm_Pressure_decode(transfer, &req)) {
        return;
    }
    AP_Floater3V* driver;
    {
        WITH_SEMAPHORE(_sem_registry);
        driver = AP_Floater3V::get_singleton();
        if (driver == nullptr) {
            return;
        }
    }
    {
        WITH_SEMAPHORE(driver->get_semaphore());
        driver->levelReading = req.pressures.data[0].pressure;
        
        /*  _update_and_wrap_accumulator(&driver->_pressure, msg.pressures.data[0].pressure, &driver->_pressure_count, 32);
        driver->new_pressure = true;
        driver->_frontend.sensors[driver->_instance].type = static_cast<AP_Baro::baro_type_t>(msg.pressures.data[0].baro_type);  */
    }
}
#endif











namespace AP {

AP_Floater3V &floater()
{
    return *AP_Floater3V::get_singleton();
}

};

#endif
#endif