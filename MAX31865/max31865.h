/**
 * @file    max31865.h
 * @brief   MAX31865 RTD-to-Digital Converter Generic Library
 * @author  Pham Nam Hien
 * @date    2024
 * @version 3.0 - Generic version
 */

#ifndef MAX31865_H
#define MAX31865_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Debug Configuration */
#ifndef MAX31865_DEBUG
#define MAX31865_DEBUG 0
#endif

#if MAX31865_DEBUG
#define MAX31865_DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define MAX31865_DEBUG_PRINT(fmt, ...) ((void)0)
#endif

/* MAX31865 Register Map */
#define MAX31865_CONFIG_REG           0x00
#define MAX31865_RTD_MSB_REG          0x01
#define MAX31865_RTD_LSB_REG          0x02
#define MAX31865_HFAULT_MSB_REG       0x03
#define MAX31865_HFAULT_LSB_REG       0x04
#define MAX31865_LFAULT_MSB_REG       0x05
#define MAX31865_LFAULT_LSB_REG       0x06
#define MAX31865_FAULT_STATUS_REG     0x07

/* Configuration Register Bits */
#define MAX31865_CONFIG_BIAS          0x80  // V_BIAS enable
#define MAX31865_CONFIG_MODEAUTO      0x40  // Auto conversion mode
#define MAX31865_CONFIG_1SHOT         0x20  // One-shot conversion
#define MAX31865_CONFIG_3WIRE         0x10  // 3-wire RTD
#define MAX31865_CONFIG_FAULTCYCLE    0x0C  // Fault detection cycle control
#define MAX31865_CONFIG_FAULTSTAT     0x02  // Fault status clear
#define MAX31865_CONFIG_FILT50HZ      0x01  // 50Hz filter enable

/* Fault Status Register Bits */
#define MAX31865_FAULT_HIGHTHRESH     0x80
#define MAX31865_FAULT_LOWTHRESH      0x40
#define MAX31865_FAULT_REFINLOW       0x20
#define MAX31865_FAULT_REFINHIGH      0x10
#define MAX31865_FAULT_RTDINLOW       0x08
#define MAX31865_FAULT_OVUV           0x04

/* RTD Constants (IEC 751) */
#define RTD_A                         3.9083e-3f
#define RTD_B                         -5.775e-7f
#define RTD_C                         -4.183e-12f  // For T < 0°C

/* Default Values */
#define MAX31865_ADC_MAX              32768.0f  // 2^15
#define MAX31865_SPI_TIMEOUT          100       // SPI timeout in ms

/* Wire Configuration */
typedef enum {
    MAX31865_2WIRE = 0,
    MAX31865_3WIRE = 1,
    MAX31865_4WIRE = 0
} max31865_numwires_t;

/* Filter Configuration */
typedef enum {
    MAX31865_FILTER_60HZ = 0,
    MAX31865_FILTER_50HZ = 1
} max31865_filter_t;

/* Fault Detection Mode */
typedef enum {
    MAX31865_FAULT_NONE = 0,
    MAX31865_FAULT_AUTO = 1,
    MAX31865_FAULT_MANUAL_RUN = 2,
    MAX31865_FAULT_MANUAL_FINISH = 3
} max31865_fault_mode_t;

/* Return Status */
typedef enum {
    MAX31865_OK = 0,
    MAX31865_ERROR = 1,
    MAX31865_TIMEOUT = 2,
    MAX31865_FAULT = 3
} max31865_status_t;

/* Forward declaration */
struct MAX31865_Handle;

/* Platform Interface Function Pointers */
typedef struct {
    /* SPI Communication Functions */
    max31865_status_t (*spi_write_read)(struct MAX31865_Handle *hmax, uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

    /* GPIO Control Functions */
    void (*cs_low)(void);
    void (*cs_high)(void);

    /* Delay Function */
    void (*delay_ms)(uint32_t ms);

    /* Optional: Custom data pointer for platform-specific data */
    void *platform_data;
} max31865_platform_t;

/* MAX31865 Device Structure */
typedef struct MAX31865_Handle {
    /* Platform Interface */
    max31865_platform_t platform;

    /* RTD Configuration */
    float rref;                 // Reference resistor value (Ohms)
    float rnominal;            // Nominal RTD resistance at 0°C (Ohms)
    max31865_numwires_t wires; // Wire configuration
    max31865_filter_t filter;  // Noise filter selection

    /* Status */
    bool initialized;
    uint8_t last_fault;
    float last_temperature;
    uint16_t last_rtd_raw;
} MAX31865_Handle_t;

/* Function Prototypes */

/* Initialization and Configuration */
max31865_status_t MAX31865_Init(MAX31865_Handle_t *hmax,
                                const max31865_platform_t *platform,
                                max31865_numwires_t wires,
                                float rref, float rnominal);

max31865_status_t MAX31865_DeInit(MAX31865_Handle_t *hmax);

max31865_status_t MAX31865_SetWires(MAX31865_Handle_t *hmax, max31865_numwires_t wires);
max31865_status_t MAX31865_SetFilter(MAX31865_Handle_t *hmax, max31865_filter_t filter);
max31865_status_t MAX31865_SetThresholds(MAX31865_Handle_t *hmax, uint16_t lower, uint16_t upper);

/* Bias and Conversion Control */
max31865_status_t MAX31865_EnableBias(MAX31865_Handle_t *hmax, bool enable);
max31865_status_t MAX31865_EnableAutoConvert(MAX31865_Handle_t *hmax, bool enable);

/* Reading Functions */
max31865_status_t MAX31865_ReadRTD(MAX31865_Handle_t *hmax, uint16_t *rtd_raw);
max31865_status_t MAX31865_ReadTemperature(MAX31865_Handle_t *hmax, float *temperature);
max31865_status_t MAX31865_ReadResistance(MAX31865_Handle_t *hmax, float *resistance);

/* Fault Detection */
max31865_status_t MAX31865_ReadFault(MAX31865_Handle_t *hmax, uint8_t *fault_status);
max31865_status_t MAX31865_ClearFault(MAX31865_Handle_t *hmax);

/* Utility Functions */
float MAX31865_RTDtoResistance(uint16_t rtd_raw, float rref);
float MAX31865_ResistanceToTemperature(float resistance, float rnominal);
const char* MAX31865_GetFaultString(uint8_t fault_bits);

/* Low-Level Register Access */
max31865_status_t MAX31865_ReadRegister8(MAX31865_Handle_t *hmax, uint8_t reg, uint8_t *data);
max31865_status_t MAX31865_ReadRegister16(MAX31865_Handle_t *hmax, uint8_t reg, uint16_t *data);
max31865_status_t MAX31865_WriteRegister8(MAX31865_Handle_t *hmax, uint8_t reg, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* MAX31865_H */

