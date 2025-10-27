# Readme for ADC for ESP-IDF

## Overview

The salinity probe uses 4 ADC channels:

- **Channel 0 (UNB)**: Unbuffered input
- **Channel 1 (DAC)**: DAC output after buffer op-amp
- **Channel 2 (AMP)**: Amplifier input (before R2)
- **Channel 3 (SGN)**: Signal from x11 amplifier (drop over R2)

---
## Macros (`#define`)
```c
// --- ADC Input Pins ---       
//#define ADC_UNB_PIN  1                // ADC input from DAC output
#define ADC_UNB_CHN  ADC1_CHANNEL_0
//#define ADC_DAC_PIN  2                // ADC input from DAC output after Buffer Op-Amp
#define ADC_DAC_CHN  ADC1_CHANNEL_1
//#define ADC_AMP_PIN  3                // ADC input before R2
#define ADC_AMP_CHN  ADC1_CHANNEL_2
//#define ADC_SGN_PIN  4                // ADC input from x11 ampl (drop over R2)
#define ADC_SGN_CHN  ADC1_CHANNEL_3
// ADC Parameters
#define ADC_ATTEN    ADC_ATTEN_DB_0    // maps ~0-3.3 V to full scale (attenuate for higher voltages, 0:k=100%, 2.5:k=75%, 6:k=50%, 12:k=25%)
#define ADC_WIDTH    ADC_BITWIDTH_13    // 13-bit resolution
```

## Global Variables

Add these to the global variables section: 

```c

// ADC calibration structure
static esp_adc_cal_characteristics_t *adc_chars;

// The existing calibration factors
float calibrationFactor1 = 0.875;  // For unbuffered channel
float calibrationFactor4 = 0.81;   // For signal channel

```

---
## Core Functions
### ADC Initialization

```c

/**
 * @brief Initialize ADC with proper configuration and calibration
 * @return esp_err_t Error code
 */

static esp_err_t adc_init(void)
{
    // Configure ADC1 for specified width and attenuation
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_UNB_CHN, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_DAC_CHN, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_AMP_CHN, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_SGN_CHN, ADC_ATTEN));

    // Characterize ADC for voltage calculation
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 1100, adc_chars);

    // Print calibration information
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else {
        printf("Characterized using Default Vref\n");
    }

    return ESP_OK;
}

```

**What it does:**
- Configures all 4 ADC channels with the defined attenuation and width
- Sets up ADC calibration using ESP32's built-in characterization
- Reports which calibration method is being used

**When to call:** Once during initialization, after I2C init.

### Generic Voltage Reading

```c

/**
 * @brief Read ADC value from specified channel and return calibrated voltage
 * @param channel ADC channel to read from (ADC_UNB_CHN, ADC_DAC_CHN, etc.)
 * @param samples Number of samples to average (default: 64)
 * @return float Voltage reading in volts
 */

static float adc_read_voltage(adc1_channel_t channel, int samples)

{

    if (samples <= 0) samples = 64; // Default sampling
    uint32_t adc_reading = 0;

    // Take multiple samples for better accuracy
    for (int i = 0; i < samples; i++) {
        adc_reading += adc1_get_raw(channel);
    }

    adc_reading /= samples;

    // Convert adc_reading to voltage in mV then to volts
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    return voltage_mv / 1000.0; // Convert mV to V
}

```

**What it does:**
- Takes multiple samples and averages them for noise reduction
- Converts raw ADC reading to actual voltage using calibration
- Returns voltage in volts (not millivolts)

**Parameters:**
- `channel`: Use the defined channel constants (ADC_UNB_CHN, ADC_DAC_CHN, etc.)
- `samples`: Higher = more accurate but slower (typical: 32-128)

### Raw ADC Reading

```c

/**
 * @brief Read raw ADC value from specified channel
 * @param channel ADC channel to read from
 * @param samples Number of samples to average (default: 64)
 * @return int Raw ADC reading (0-4095 for 12-bit)
 */
static int adc_read_raw(adc1_channel_t channel, int samples)
{
    if (samples <= 0) samples = 64; // Default sampling
    uint32_t adc_reading = 0;
    
    // Take multiple samples for better accuracy
    for (int i = 0; i < samples; i++) {
        adc_reading += adc1_get_raw(channel);
    }

    return adc_reading / samples;
}

```

**What it does:**
- Returns the raw ADC value (0-4095 for 12-bit resolution)
- Useful for debugging or custom voltage calculations