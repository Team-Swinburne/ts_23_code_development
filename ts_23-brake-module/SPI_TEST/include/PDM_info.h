#ifndef __PDM_INFO_H__
#define __PDM_INFO_H__
//This header contains all the structs and definitions for the brake module.

#define	CAN_HEARTBEAT_PERIOD 1 // Heartbeat message transmit period
#define CAN_ERROR_PERIOD 0.5 // Error message transmit period
#define CAN_DIGITAL_1_PERIOD 1 // Digital 1 message transmit period
#define CAN_ANALOG_1_PERIOD 0.02 //Analog 1 message transmit period

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

//Enums for types of ADC Configurations
typedef enum ADC_CONFIG{
  GPIO_OUT,
  ADC
} ADC_CONFIG_t;

#endif //__PDM_INFO_H__