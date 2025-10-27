#include <Wire.h>

#define MCP4725_ADDR 0x60  // MCP4725A0 default address
#define DAC_MAX_5V   4095  // 12-bit range
#define DAC_MAX_3V3  2000  // Limit for 3V out when powered at 5V

#define ADC_PIN      1     // ADC input
#define SWITCH_PIN   14    // Controls switch

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);  // SDA=GPIO8, SCL=GPIO9
  //pinMode(SWITCH_PIN, OUTPUT);
  //digitalWrite(SWITCH_PIN, HIGH); // turn on switch
}

void loop() {
  // Example: sweep 0 → 3V
  for (int val = 0; val <= DAC_MAX_3V3; val += 200) {
    writeDAC(val);

    // Read ADC
    int adcRaw = analogRead(ADC_PIN);
    float adcVolt = (adcRaw / 8191.0) * 3.3;

    // DAC voltage (scaled to 5V ref, but capped at 3)
    float dacVolt = (val / (float)DAC_MAX_5V) * 5.0;

    Serial.print("DAC Code: ");
    Serial.print(val);
    Serial.print("  DAC V: ");
    Serial.print(dacVolt, 3);
    Serial.print(" V | ADC Raw: ");
    Serial.print(adcRaw);
    Serial.print("  ADC V: ");
    Serial.print(adcVolt, 3);
    Serial.print(" V");
    Serial.print(" | Diff: ");
    Serial.println(adcVolt-dacVolt);

    delay(500);
  }
}

void writeDAC(uint16_t value) {
  if (value > DAC_MAX_3V3) value = DAC_MAX_3V3; // safety clamp
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(0x40);              // Fast mode command
  Wire.write(value >> 4);        // Upper 8 bits
  Wire.write((value & 0xF) << 4); // Lower 4 bits
  Wire.endTransmission();
}
