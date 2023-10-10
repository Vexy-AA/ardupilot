#pragma once



#include <AP_HAL/AP_HAL.h>


#ifndef AP_FLOATER3V_ENABLED
#define AP_FLOATER3V_ENABLED 0
#endif


#if AP_FLOATER3V_ENABLED


#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel.h>
#include <dronecan_msgs.h>



#define VALVE_NUMBER 3

typedef enum{
        NONE,
        HIWONDER,
        ANALOG,
    } valve_type_t;

    typedef enum{
        TOP,
        BOTTOM,
        AIR,
    } valve_place_t;
class Valve
{
    
public:
    Valve(valve_type_t _type, SRV_Channel::Aux_servo_function_t _function);
    void open(uint32_t now, uint32_t time);
    void open();

    void close();

    void update(uint32_t now);
    void updateFunction(SRV_Channel::Aux_servo_function_t _function);
    uint32_t closeTime;
    valve_type_t type;
    SRV_Channel::Aux_servo_function_t function;

    
private:
};

class AP_Floater3V
{
    

public:
    AP_Floater3V();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Floater3V);

    // get singleton
    static AP_Floater3V *get_singleton(void) {
        return _singleton;
    }
    
    void update(void);

    void init(void);
    
    // allow threads to lock against baro update
    HAL_Semaphore &get_semaphore(void) {
        return _rsem;
    }
    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];

#if AP_DRONECAN_SNOWSTORM_SUPPORT
    static void handle_floater(CanardInstance* canard_instance, CanardRxTransfer* transfer);
#endif
float levelReading = -1;
private:
    // singleton
    static AP_Floater3V *_singleton;

    bool detectLevelSensor();
    void updateLevelReading();

    void increaseLevel();
    void decreaseLevel();
    void closeAll();
    
    Valve* valves[VALVE_NUMBER] = {nullptr};
    
    bool valveSet(uint8_t ch, uint8_t enable);
    
    AP_HAL::AnalogSource *floaterSensor = nullptr;
    AP_Int8                             instance;
    AP_Enum16<SRV_Channel::Aux_servo_function_t>     srv1Func;
    AP_Enum16<SRV_Channel::Aux_servo_function_t>     srv2Func;
    AP_Int8                             baro_internal;
    AP_Int8                             baro_external;
    AP_Int8                             vds_pin;
    AP_Int8                             level_sensor_pin;
    uint8_t                             current_pin = 0;
    bool initialised = false;
     // semaphore for API access from threads
    HAL_Semaphore                      _rsem;
    static HAL_Semaphore _sem_registry;
    
};

namespace AP {
    AP_Floater3V &floater();
};

#endif