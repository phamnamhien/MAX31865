/**
 * @file    max31865_generic.c
 * @brief   MAX31865 RTD-to-Digital Converter Generic Library Implementation
 * @author  Pham Nam Hien
 * @date    2024
 * @version 3.0 - Generic version
 */

#include "max31865.h"
#include <string.h>

/* Private Functions */
static max31865_status_t MAX31865_WaitForConversion(MAX31865_Handle_t *hmax);

/* Wait for conversion completion */
static max31865_status_t MAX31865_WaitForConversion(MAX31865_Handle_t *hmax) {
    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(65); // 65ms for 60Hz filter, 52ms for 50Hz
    }
    return MAX31865_OK;
}

/**
 * @brief Initialize MAX31865 device
 */
max31865_status_t MAX31865_Init(MAX31865_Handle_t *hmax,
                                const max31865_platform_t *platform,
                                max31865_numwires_t wires,
                                float rref, float rnominal) {

    if (hmax == NULL || platform == NULL) {
        return MAX31865_ERROR;
    }

    /* Copy Platform Interface functions */
    hmax->platform = *platform;

    /* Initialize structure */
    hmax->rref = rref;
    hmax->rnominal = rnominal;
    hmax->wires = wires;
    hmax->filter = MAX31865_FILTER_60HZ;
    hmax->initialized = false;
    hmax->last_fault = 0;
    hmax->last_temperature = 0.0f;
    hmax->last_rtd_raw = 0;

    /* Set CS high initially */
    if (hmax->platform.cs_high) {
        hmax->platform.cs_high();
    }

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(10);
    }

    /* Test communication */
    uint8_t config_reg = 0;
    if (MAX31865_ReadRegister8(hmax, MAX31865_CONFIG_REG, &config_reg) != MAX31865_OK) {
        MAX31865_DEBUG_PRINT("MAX31865: Communication test failed\r\n");
        return MAX31865_ERROR;
    }

    /* Clear any existing faults */
    MAX31865_ClearFault(hmax);

    /* Configure wire mode */
    if (MAX31865_SetWires(hmax, wires) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    /* Disable bias initially */
    if (MAX31865_EnableBias(hmax, false) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    /* Disable auto conversion */
    if (MAX31865_EnableAutoConvert(hmax, false) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    /* Set default thresholds */
    if (MAX31865_SetThresholds(hmax, 0x0000, 0x7FFF) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    hmax->initialized = true;
    MAX31865_DEBUG_PRINT("MAX31865: Initialized successfully\r\n");

    return MAX31865_OK;
}

/**
 * @brief Deinitialize MAX31865 device
 */
max31865_status_t MAX31865_DeInit(MAX31865_Handle_t *hmax) {
    if (hmax == NULL) return MAX31865_ERROR;

    /* Disable bias */
    MAX31865_EnableBias(hmax, false);

    /* Disable auto conversion */
    MAX31865_EnableAutoConvert(hmax, false);

    /* Clear faults */
    MAX31865_ClearFault(hmax);

    hmax->initialized = false;

    return MAX31865_OK;
}

/**
 * @brief Read 8-bit register
 */
max31865_status_t MAX31865_ReadRegister8(MAX31865_Handle_t *hmax, uint8_t reg, uint8_t *data) {
    if (hmax == NULL || data == NULL || hmax->platform.spi_write_read == NULL) {
        return MAX31865_ERROR;
    }

    uint8_t tx_data = reg & 0x7F; // Clear MSB for read
    uint8_t rx_data[2] = {0, 0};
    max31865_status_t status;

    if (hmax->platform.cs_low) {
    	hmax->platform.cs_low();
    }

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    /* Send register address and read data */
    status = hmax->platform.spi_write_read(hmax, &tx_data, rx_data, 2);

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    if (hmax->platform.cs_high) {
    	hmax->platform.cs_high();
    }

    if (status != MAX31865_OK) {
        return status;
    }

    *data = rx_data[1]; // Data comes in second byte
    return MAX31865_OK;
}

/**
 * @brief Read 16-bit register
 */
max31865_status_t MAX31865_ReadRegister16(MAX31865_Handle_t *hmax, uint8_t reg, uint16_t *data) {
    if (hmax == NULL || data == NULL || hmax->platform.spi_write_read == NULL) {
        return MAX31865_ERROR;
    }

    uint8_t tx_data = reg & 0x7F; // Clear MSB for read
    uint8_t rx_data[3] = {0, 0, 0};
    max31865_status_t status;

    if (hmax->platform.cs_low) {
    	hmax->platform.cs_low();
    }

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    /* Send register address and read 2 bytes */
    status = hmax->platform.spi_write_read(hmax, &tx_data, rx_data, 3);

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    if (hmax->platform.cs_high) {
        hmax->platform.cs_high();
    }

    if (status != MAX31865_OK) {
        return status;
    }

    *data = ((uint16_t)rx_data[1] << 8) | rx_data[2];
    return MAX31865_OK;
}

/**
 * @brief Write 8-bit register
 */
max31865_status_t MAX31865_WriteRegister8(MAX31865_Handle_t *hmax, uint8_t reg, uint8_t data) {
    if (hmax == NULL || hmax->platform.spi_write_read == NULL) {
        return MAX31865_ERROR;
    }

    uint8_t tx_data[2];
    uint8_t rx_data[2];
    max31865_status_t status;

    tx_data[0] = reg | 0x80; // Set MSB for write
    tx_data[1] = data;

    if (hmax->platform.cs_low) {
        hmax->platform.cs_low();
    }

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    status = hmax->platform.spi_write_read(hmax, tx_data, rx_data, 2);

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    if (hmax->platform.cs_high) {
        hmax->platform.cs_high();
    }

    return status;
}

/**
 * @brief Set wire configuration
 */
max31865_status_t MAX31865_SetWires(MAX31865_Handle_t *hmax, max31865_numwires_t wires) {
    if (hmax == NULL) return MAX31865_ERROR;

    uint8_t config = 0;
    if (MAX31865_ReadRegister8(hmax, MAX31865_CONFIG_REG, &config) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    if (wires == MAX31865_3WIRE) {
        config |= MAX31865_CONFIG_3WIRE;
    } else {
        config &= ~MAX31865_CONFIG_3WIRE;
    }

    if (MAX31865_WriteRegister8(hmax, MAX31865_CONFIG_REG, config) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    hmax->wires = wires;
    return MAX31865_OK;
}

/**
 * @brief Set noise filter
 */
max31865_status_t MAX31865_SetFilter(MAX31865_Handle_t *hmax, max31865_filter_t filter) {
    if (hmax == NULL) return MAX31865_ERROR;

    uint8_t config = 0;
    if (MAX31865_ReadRegister8(hmax, MAX31865_CONFIG_REG, &config) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    if (filter == MAX31865_FILTER_50HZ) {
        config |= MAX31865_CONFIG_FILT50HZ;
    } else {
        config &= ~MAX31865_CONFIG_FILT50HZ;
    }

    if (MAX31865_WriteRegister8(hmax, MAX31865_CONFIG_REG, config) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    hmax->filter = filter;
    return MAX31865_OK;
}

/**
 * @brief Enable/disable bias voltage
 */
max31865_status_t MAX31865_EnableBias(MAX31865_Handle_t *hmax, bool enable) {
    if (hmax == NULL) return MAX31865_ERROR;

    uint8_t config = 0;
    if (MAX31865_ReadRegister8(hmax, MAX31865_CONFIG_REG, &config) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    if (enable) {
        config |= MAX31865_CONFIG_BIAS;
    } else {
        config &= ~MAX31865_CONFIG_BIAS;
    }

    return MAX31865_WriteRegister8(hmax, MAX31865_CONFIG_REG, config);
}

/**
 * @brief Enable/disable auto conversion mode
 */
max31865_status_t MAX31865_EnableAutoConvert(MAX31865_Handle_t *hmax, bool enable) {
    if (hmax == NULL) return MAX31865_ERROR;

    uint8_t config = 0;
    if (MAX31865_ReadRegister8(hmax, MAX31865_CONFIG_REG, &config) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    if (enable) {
        config |= MAX31865_CONFIG_MODEAUTO;
    } else {
        config &= ~MAX31865_CONFIG_MODEAUTO;
    }

    return MAX31865_WriteRegister8(hmax, MAX31865_CONFIG_REG, config);
}

/**
 * @brief Clear fault status
 */
max31865_status_t MAX31865_ClearFault(MAX31865_Handle_t *hmax) {
    if (hmax == NULL) return MAX31865_ERROR;

    uint8_t config = 0;
    if (MAX31865_ReadRegister8(hmax, MAX31865_CONFIG_REG, &config) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    config &= ~0x2C; // Clear fault detection bits
    config |= MAX31865_CONFIG_FAULTSTAT;

    return MAX31865_WriteRegister8(hmax, MAX31865_CONFIG_REG, config);
}

/**
 * @brief Set fault thresholds
 */
max31865_status_t MAX31865_SetThresholds(MAX31865_Handle_t *hmax, uint16_t lower, uint16_t upper) {
    if (hmax == NULL) return MAX31865_ERROR;

    /* Set lower threshold */
    if (MAX31865_WriteRegister8(hmax, MAX31865_LFAULT_MSB_REG, (lower >> 8) & 0xFF) != MAX31865_OK) {
        return MAX31865_ERROR;
    }
    if (MAX31865_WriteRegister8(hmax, MAX31865_LFAULT_LSB_REG, lower & 0xFF) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    /* Set upper threshold */
    if (MAX31865_WriteRegister8(hmax, MAX31865_HFAULT_MSB_REG, (upper >> 8) & 0xFF) != MAX31865_OK) {
        return MAX31865_ERROR;
    }
    if (MAX31865_WriteRegister8(hmax, MAX31865_HFAULT_LSB_REG, upper & 0xFF) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    return MAX31865_OK;
}

/**
 * @brief Read RTD raw value (one-shot mode)
 */
max31865_status_t MAX31865_ReadRTD(MAX31865_Handle_t *hmax, uint16_t *rtd_raw) {
    if (hmax == NULL || rtd_raw == NULL) return MAX31865_ERROR;

    /* Clear faults */
    MAX31865_ClearFault(hmax);

    /* Enable bias */
    if (MAX31865_EnableBias(hmax, true) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(10); // Wait for bias to stabilize
    }

    /* Start one-shot conversion */
    uint8_t config = 0;
    if (MAX31865_ReadRegister8(hmax, MAX31865_CONFIG_REG, &config) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    config |= MAX31865_CONFIG_1SHOT;
    if (MAX31865_WriteRegister8(hmax, MAX31865_CONFIG_REG, config) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    /* Wait for conversion */
    if (MAX31865_WaitForConversion(hmax) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    /* Read RTD register */
    uint16_t rtd_reg = 0;
    if (MAX31865_ReadRegister16(hmax, MAX31865_RTD_MSB_REG, &rtd_reg) != MAX31865_OK) {
        return MAX31865_ERROR;
    }

    /* Disable bias to reduce self-heating */
    MAX31865_EnableBias(hmax, false);

    /* Remove fault bit (LSB) and store result */
    *rtd_raw = rtd_reg >> 1;
    hmax->last_rtd_raw = *rtd_raw;

    /* Check for fault in LSB */
    if (rtd_reg & 0x01) {
        uint8_t fault_status = 0;
        MAX31865_ReadFault(hmax, &fault_status);
        hmax->last_fault = fault_status;
        return MAX31865_FAULT;
    }

    return MAX31865_OK;
}

/**
 * @brief Read fault status
 */
max31865_status_t MAX31865_ReadFault(MAX31865_Handle_t *hmax, uint8_t *fault_status) {
    if (hmax == NULL || fault_status == NULL) return MAX31865_ERROR;

    return MAX31865_ReadRegister8(hmax, MAX31865_FAULT_STATUS_REG, fault_status);
}

/**
 * @brief Read temperature
 */
max31865_status_t MAX31865_ReadTemperature(MAX31865_Handle_t *hmax, float *temperature) {
    if (hmax == NULL || temperature == NULL) return MAX31865_ERROR;

    uint16_t rtd_raw = 0;
    max31865_status_t status = MAX31865_ReadRTD(hmax, &rtd_raw);

    if (status != MAX31865_OK) {
        return status;
    }

    /* Convert to resistance */
    float resistance = MAX31865_RTDtoResistance(rtd_raw, hmax->rref);

    /* Convert to temperature */
    *temperature = MAX31865_ResistanceToTemperature(resistance, hmax->rnominal);
    hmax->last_temperature = *temperature;

    return MAX31865_OK;
}

/**
 * @brief Read resistance
 */
max31865_status_t MAX31865_ReadResistance(MAX31865_Handle_t *hmax, float *resistance) {
    if (hmax == NULL || resistance == NULL) return MAX31865_ERROR;

    uint16_t rtd_raw = 0;
    max31865_status_t status = MAX31865_ReadRTD(hmax, &rtd_raw);

    if (status != MAX31865_OK) {
        return status;
    }

    *resistance = MAX31865_RTDtoResistance(rtd_raw, hmax->rref);

    return MAX31865_OK;
}

/**
 * @brief Convert RTD raw value to resistance
 */
float MAX31865_RTDtoResistance(uint16_t rtd_raw, float rref) {
    return ((float)rtd_raw * rref) / MAX31865_ADC_MAX;
}

/**
 * @brief Convert resistance to temperature using Callendar-Van Dusen equation
 */
float MAX31865_ResistanceToTemperature(float resistance, float rnominal) {
    float Z1, Z2, Z3, Z4, temp;

    /* Callendar-Van Dusen equation for positive temperatures */
    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4.0f * RTD_B);
    Z3 = (4.0f * RTD_B) / rnominal;
    Z4 = 2.0f * RTD_B;

    temp = Z2 + (Z3 * resistance);
    temp = (sqrtf(temp) + Z1) / Z4;

    if (temp >= 0.0f) {
        return temp;
    }

    /* For negative temperatures */
    float rpoly = resistance / rnominal * 100.0f; // Normalize to PT100

    temp = -242.02f;
    temp += 2.2228f * rpoly;
    rpoly *= (resistance / rnominal * 100.0f);
    temp += 2.5859e-3f * rpoly;
    rpoly *= (resistance / rnominal * 100.0f);
    temp -= 4.8260e-6f * rpoly;
    rpoly *= (resistance / rnominal * 100.0f);
    temp -= 2.8183e-8f * rpoly;
    rpoly *= (resistance / rnominal * 100.0f);
    temp += 1.5243e-10f * rpoly;

    return temp;
}

/**
 * @brief Get fault status string
 */
const char* MAX31865_GetFaultString(uint8_t fault_bits) {
    static char fault_str[256];
    fault_str[0] = '\0';

    if (fault_bits == 0) {
        return "No faults";
    }

    if (fault_bits & MAX31865_FAULT_HIGHTHRESH) {
        strcat(fault_str, "RTD High Threshold; ");
    }
    if (fault_bits & MAX31865_FAULT_LOWTHRESH) {
        strcat(fault_str, "RTD Low Threshold; ");
    }
    if (fault_bits & MAX31865_FAULT_REFINLOW) {
        strcat(fault_str, "REFIN- > 0.85 x BIAS; ");
    }
    if (fault_bits & MAX31865_FAULT_REFINHIGH) {
        strcat(fault_str, "REFIN- < 0.85 x BIAS; ");
    }
    if (fault_bits & MAX31865_FAULT_RTDINLOW) {
        strcat(fault_str, "RTDIN- < 0.85 x BIAS; ");
    }
    if (fault_bits & MAX31865_FAULT_OVUV) {
        strcat(fault_str, "Over/Under Voltage; ");
    }

    return fault_str;
}
