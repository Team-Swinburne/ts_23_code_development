#ifndef __PDM_INFO_H__
#define __PDM_INFO_H__
//This header contains all the structs and definitions for the brake module.

#define	CAN_HEARTBEAT_PERIOD 1 // Heartbeat message transmit period
#define CAN_ERROR_PERIOD 0.5 // Error message transmit period
#define CAN_DIGITAL_1_PERIOD 1 // Digital 1 message transmit period
#define CAN_ANALOG_1_PERIOD 0.02 //Analog 1 message transmit period


//All the variables required for the heartbeat
typedef struct HeartBeat_s{
  uint8_t Counter = 0;
  uint8_t State = 1;
}HeartBeat_struct;

//Enums for all the bytes in the heartbeat CAN message
typedef enum CAN_HEARTBEAT_SIGNALS{
  CAN_HEARTBEAT_STATE,
  CAN_HEARTBEAT_COUNTER,
  CAN_HEARTBEAT_PCB_TEMP,
  CAN_HEARTBEAT_HARDWARE_REVISION,
  CAN_HEARTBEAT_COMPILE_DATE,
  CAN_HEARTBEAT_COMPILE_TIME
} can_HEARTBEAT_signals_t;

//Enums for all the bytes in the error CAN message
typedef enum CAN_ERROR_SIGNALS{
    CAN_ERROR_OVER_VOLTAGE,
    CAN_ERROR_OVER_CURRENT,
    CAN_ERROR_SHORT_CIRCUIT,
    CAN_ERROR_OVER_TEMP
} can_ERROR_signals_t;



//Enums for all the bytes in the digital 1 CAN message
typedef enum CAN_DIGITAL_1_SIGNALS{
  CAN_DIGITAL_1_CIRCUIT_1_STATE,
  CAN_DIGITAL_1_CIRCUIT_2_STATE,
  CAN_DIGITAL_1_CIRCUIT_3_STATE,
  CAN_DIGITAL_1_CIRCUIT_4_STATE,
  CAN_DIGITAL_1_CIRCUIT_5_STATE,
  CAN_DIGITAL_1_CIRCUIT_6_STATE,
  CAN_DIGITAL_1_CIRCUIT_7_STATE,
  CAN_DIGITAL_1_CIRCUIT_8_STATE
} can_DIGITAL_1_signals_t;

//Enums for all the bytes in the H-Bridge 1 CAN Message
typedef enum H_BRIDGE_1_SIGNALS{
  CAN_H_BRIDGE_1_PWN_VALUE,
  CAN_H_BRIDGE_2_PWN_VALUE,
  CAN_H_BRIDGE_3_PWN_VALUE,
  CAN_H_BRIDGE_4_PWN_VALUE,
  CAN_H_BRIDGE_5_PWN_VALUE,
  CAN_H_BRIDGE_6_PWN_VALUE,
  CAN_H_BRIDGE_7_PWN_VALUE,
  CAN_H_BRIDGE_8_PWN_VALUE
}

//Enums for all the bytes in the analog 1 CAN message
typedef enum CAN_ANALOG_1_SIGNALS{
  CAN_ANALOG_1_CIRCUIT_1_CURRENT,
  CAN_ANALOG_1_CIRCUIT_2_CURRENT,
  CAN_ANALOG_1_CIRCUIT_3_CURRENT,
  CAN_ANALOG_1_CIRCUIT_4_CURRENT,
  CAN_ANALOG_1_CIRCUIT_5_CURRENT,
  CAN_ANALOG_1_CIRCUIT_6_CURRENT,
  CAN_ANALOG_1_CIRCUIT_7_CURRENT,
  CAN_ANALOG_1_CIRCUIT_8_CURRENT
} can_ANALOG_1_signals_t;

#endif //__PDM_INFO_H__