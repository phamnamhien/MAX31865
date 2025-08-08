[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/Q5Q1JW4XS)

# MAX31865 Generic Library

A platform-independent C library for MAX31865 RTD-to-Digital Converter supporting PT100/PT1000 sensors with 2-wire, 3-wire, and 4-wire configurations.

## Features

- ✅ **Platform Independent** - Works with STM32, Arduino, ESP32, etc.
- ✅ **Multiple RTD Types** - PT100, PT1000 support
- ✅ **Wire Configurations** - 2-wire, 3-wire, 4-wire
- ✅ **Fault Detection** - Automatic RTD and cable fault detection
- ✅ **Noise Filtering** - 50Hz/60Hz selectable filters
- ✅ **High Accuracy** - 15-bit ADC with 0.03°C resolution
- ✅ **Easy Integration** - Simple callback-based interface

## Quick Start

### 1. Implement Platform Functions

```c
// SPI communication
max31865_status_t platform_spi_write_read(MAX31865_Handle_t *hmax, 
                                          uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    // Your platform SPI implementation
    return MAX31865_OK;
}

// GPIO control
void platform_cs_low(MAX31865_Handle_t *hmax) {
    // Set CS pin LOW
}

void platform_cs_high(MAX31865_Handle_t *hmax) {
    // Set CS pin HIGH  
}

// Delay function
void platform_delay_ms(uint32_t ms) {
    // Your platform delay implementation
}
```

### 2. Initialize Library

```c
#include "max31865.h"

MAX31865_Handle_t rtd_sensor;

// Create platform interface
max31865_platform_t platform = {
    .spi_write_read = platform_spi_write_read,
    .cs_low = platform_cs_low,
    .cs_high = platform_cs_high,
    .delay_ms = platform_delay_ms,
    .platform_data = NULL
};

// Initialize sensor (PT100, 3-wire, RREF=430Ω)
MAX31865_Init(&rtd_sensor, &platform, cs_port, cs_pin,
              MAX31865_3WIRE, 430.0f, 100.0f);
```

### 3. Read Temperature

```c
float temperature;
max31865_status_t status = MAX31865_ReadTemperature(&rtd_sensor, &temperature);

if (status == MAX31865_OK) {
    printf("Temperature: %.2f°C\n", temperature);
} else {
    printf("Error: %d\n", status);
}
```

## Hardware Setup

### SPI Configuration
- **Mode**: SPI Mode 1 or 3 (CPOL=1, CPHA=1)
- **Speed**: 1-5 MHz (typically 2 MHz)
- **Bit Order**: MSB First

### RTD Connection (3-wire PT100)
```
MAX31865    RTD Sensor
--------    ----------
FORCE+   -> RTD Lead 1
FORCE2   -> RTD Lead 1 (same as FORCE+)
RTDIN+   -> RTD Lead 2  
RTDIN-   -> RTD Lead 3
FORCE-   -> RTD Lead 3 (same as RTDIN-)

REFIN+   -> BIAS
REFIN-   -> ISENSOR
RREF (430Ω) between REFIN+ and REFIN-
```

### Power Supply
- **VDD/DVDD**: 3.3V
- **GND1/GND2/DGND**: Ground

## STM32 Example

```c
// STM32 HAL implementation
max31865_status_t stm32_spi_write_read(MAX31865_Handle_t *hmax, 
                                       uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    if (HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size, HAL_MAX_DELAY) == HAL_OK) {
        return MAX31865_OK;
    }
    return MAX31865_TIMEOUT;
}

void stm32_cs_low(MAX31865_Handle_t *hmax) {
    HAL_GPIO_WritePin((GPIO_TypeDef*)hmax->cs_port, hmax->cs_pin, GPIO_PIN_RESET);
}

void stm32_cs_high(MAX31865_Handle_t *hmax) {
    HAL_GPIO_WritePin((GPIO_TypeDef*)hmax->cs_port, hmax->cs_pin, GPIO_PIN_SET);
}

void stm32_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

## Arduino Example

```c
// Arduino implementation
max31865_status_t arduino_spi_write_read(MAX31865_Handle_t *hmax, 
                                         uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    for (int i = 0; i < size; i++) {
        rx_data[i] = SPI.transfer(tx_data[i]);
    }
    return MAX31865_OK;
}

void arduino_cs_low(MAX31865_Handle_t *hmax) {
    digitalWrite((int)(uintptr_t)hmax->cs_port, LOW);
}

void arduino_cs_high(MAX31865_Handle_t *hmax) {
    digitalWrite((int)(uintptr_t)hmax->cs_port, HIGH);
}

void arduino_delay_ms(uint32_t ms) {
    delay(ms);
}
```

## API Reference

### Initialization
- `MAX31865_Init()` - Initialize sensor
- `MAX31865_DeInit()` - Deinitialize sensor

### Reading Functions
- `MAX31865_ReadTemperature()` - Read temperature in °C
- `MAX31865_ReadResistance()` - Read resistance in Ω
- `MAX31865_ReadRTD()` - Read raw ADC value

### Configuration
- `MAX31865_SetWires()` - Configure wire mode (2/3/4-wire)
- `MAX31865_SetFilter()` - Set noise filter (50Hz/60Hz)
- `MAX31865_SetThresholds()` - Set fault thresholds

### Fault Detection
- `MAX31865_ReadFault()` - Read fault status
- `MAX31865_ClearFault()` - Clear fault flags
- `MAX31865_GetFaultString()` - Get fault description

### Status Codes
- `MAX31865_OK` - Success
- `MAX31865_ERROR` - General error
- `MAX31865_TIMEOUT` - Communication timeout
- `MAX31865_FAULT` - RTD fault detected

## Common RTD Values

| RTD Type | RNOMINAL | RREF | Wire Config |
|----------|----------|------|-------------|
| PT100    | 100.0Ω   | 430Ω | 3-wire      |
| PT1000   | 1000.0Ω  | 4300Ω| 3-wire      |

## Error Handling

```c
uint8_t fault_status;
if (MAX31865_ReadFault(&rtd_sensor, &fault_status) == MAX31865_OK) {
    if (fault_status != 0) {
        printf("Fault: %s\n", MAX31865_GetFaultString(fault_status));
        MAX31865_ClearFault(&rtd_sensor);
    }
}
```

## Multiple Sensors

```c
MAX31865_Handle_t sensors[4];

// Initialize multiple sensors with different CS pins
for (int i = 0; i < 4; i++) {
    MAX31865_Init(&sensors[i], &platform, cs_port, cs_pin_array[i],
                  MAX31865_3WIRE, 430.0f, 100.0f);
}

// Read all sensors
for (int i = 0; i < 4; i++) {
    float temp;
    if (MAX31865_ReadTemperature(&sensors[i], &temp) == MAX31865_OK) {
        printf("Sensor %d: %.2f°C\n", i+1, temp);
    }
}
```

## Performance

- **Conversion Time**: ~65ms (60Hz filter), ~52ms (50Hz filter)
- **SPI Speed**: Up to 5MHz
- **Resolution**: 15-bit ADC (~0.03°C for PT100)
- **Accuracy**: ±0.5°C over full range
- **Temperature Range**: -200°C to +850°C

## Files

- `max31865.h` - Library header
- `max31865.c` - Library implementation
- `main.c` - STM32 example
- `main.h` - STM32 configuration

## Tested Platforms
- ✅ STM32F103C8T6 (Blue Pill)

## License

MIT License - Free for commercial and personal use.

## Support

For bugs and feature requests, please create an issue on GitHub.
