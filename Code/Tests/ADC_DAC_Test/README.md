# ESP32 ADC + DAC LED Blink Project

## Overview

- The **ADC** reads the position of a potentiometer (0–3.3V) and converts it into a digital value.  
- The **DAC** outputs a voltage proportional to the potentiometer reading.  
- An **LED** blinks at a speed determined by the potentiometer position — higher pot value = faster blink.

---

## Hardware Requirements
- **ESP32-WROOM** development board  
- **10 kΩ potentiometer**  
- **LED** (any color)  
- **220 Ω resistor** (for LED current limiting)  
- Breadboard and jumper wires  

---

## Pin Connections

| Component     | Pin on ESP32  | Notes                                    |
|---------------|--------------|------------------------------------------|
| Potentiometer | GPIO34 (ADC) | Middle pin of pot to GPIO34, sides to 3.3V and GND |
| LED           | GPIO2        | Anode (+) to GPIO2 via 220Ω resistor, cathode (-) to GND |
| DAC Output    | GPIO25 (DAC) | Optional: measure with multimeter or connect to other circuit |

---

## How It Works
1. **Analog Reading (ADC)**  
   - `analogRead(potPin)` reads the potentiometer position.  
   - The ESP32 ADC is 12-bit by default, giving values from 0 to 4095.

2. **Mapping to Blink Speed**  
   - The potentiometer reading is mapped to a delay time between LED state changes.  
   - Low pot value → long delay (slow blink).  
   - High pot value → short delay (fast blink).  
   - This is done with:  
     ```cpp
     int blinkDelay = map(potRaw, 0, 4095, 1000, 30);
     ```

3. **Mapping to DAC Output**  
   - The same ADC reading is mapped to an 8-bit DAC range (0–255).  
   - The DAC outputs a voltage roughly between 0 and 3.3V based on this value.  
   - This is done with:
     ```cpp
     int dacVal = map(potRaw, 0, 4095, 0, 255);
     dacWrite(dacPin, dacVal);
     ```

4. **LED Control**  
   - The LED is turned ON, a delay is applied, then turned OFF, with the same delay again.  
   - The delay is set by the mapped value from the potentiometer.

5. **Serial Output**  
   - The program prints:  
     - ADC input (raw value)  
     - Blink delay (ms)  
     - DAC output (0–255 raw)  

---

## Example Serial Monitor Output
ADC Input (raw): 2100 | Blink Delay: 520 ms | DAC Output: 130
ADC Input (raw): 3900 | Blink Delay: 50 ms | DAC Output: 242
ADC Input (raw): 400 | Blink Delay: 920 ms | DAC Output: 25

---

## Extensions
- Print DAC output voltage in volts instead of raw `0–255`.  
- Add filtering (moving average) to smooth noisy ADC readings.  
