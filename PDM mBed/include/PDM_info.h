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

// typedef struct driverControlSignals{
//   u_int8_t CH_1 = 0;
//   u_int8_t CH_2 = 0;
//   u_int8_t CH_3 = 0;
//   u_int8_t CH_4 = 0;
//   u_int8_t CH_5 = 0;
//   u_int8_t CH_6 = 0;
//   u_int8_t CH_7 = 0;
//   u_int8_t CH_8 = 0;
// } driverControlSignals_struct;

typedef struct PWMControlSignals{
  float CH_1 = 0;
  float CH_2 = 0;
  float CH_3 = 0;
  float CH_4 = 0;
} PWMControlSignals_struct;

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
    CAN_ERROR_OVER_CURRENT
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

//Enums for types of ADC Configurations
typedef enum ADC_CONFIG{
  GPIO_OUT,
  ADC
} ADC_CONFIG_t;

#endif //__PDM_INFO_H__