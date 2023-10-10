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
/*
  implementation of Hiwonder Dynamixel 2.0 protocol for controlling servos
 */

#pragma once

#include "AP_HiwonderServo_config.h"

#if AP_HIWONDERSERVO_ENABLED

#include <AP_Param/AP_Param.h>


#define AP_HIWONDER_SERVO_NUM 2

#define TIMEOUT_READ 100000
#define ADDRESS_MAX 20

class AP_HiwonderServo_Device;

class AP_HiwonderServo {
public:
    AP_HiwonderServo();

    CLASS_NO_COPY(AP_HiwonderServo);

    static AP_HiwonderServo *get_singleton();
    static AP_HiwonderServo *_singleton;

    static const struct AP_Param::GroupInfo var_info[];
    
    void update();
    
    uint8_t addressInvalid(uint8_t _instance);
    bool addTxItem(servoMessageItem& txItem);
private:
    AP_HiwonderServo_Device* servo[AP_HIWONDER_SERVO_NUM];
    AP_HiwonderServo_Device* currentServo;
    AP_HAL::UARTDriver *port;
    uint32_t baudrate;
    uint32_t us_per_byte;
    uint32_t us_gap;
    uint8_t activeServo = 0;
    void init(void);

    bool sendMessage(servoMessageItem& txItem);
    bool read_bytes();
    
    bool waitingReply();
    
    
    uint8_t decode(char c);

    bool initialised;
    
    uint8_t rxReceived = 0;
    uint8_t addressToCheck = 0;
    int16_t replyPending = -1;
    
    ObjectBuffer<servoMessageItem> rxQueue;
    ObjectBuffer<servoMessageItem> txQueue;
    uint8_t txBuf[16];
    uint8_t rxBuf[16];
    
    uint8_t crc = 0;
    // servo position limits
    AP_Int8  srv1Func;
    AP_Int8  srv2Func;
    AP_Int8  srv1Id;
    AP_Int8  srv2Id;
    uint32_t last_send_us;
    uint64_t timeout;
    uint8_t readTries = 0;
    bool waitForReply = false;
};

namespace AP {
    AP_HiwonderServo *hiwonder();
};

#endif  // AP_HIWONDERSERVO_ENABLED
