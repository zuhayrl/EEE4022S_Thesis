#include <Wire.h>
#include <HardwareSerial.h>
#include <MS5837.h>
#include "salinity.h"

#define MCP4725_ADDR 0x60 // MCP4725A0 default address
#define MS5837_ADDR 0x76  // MS5837 address

#define DAC_MAX_5V 4095   // 12-bit range
#define DAC_MAX_3V3 2200  // Limit for 3V out when powered at 5V

// ADC Inputs
#define ADC_UNB 1         // ADC input from unbiased DAC (before op-amp)
#define ADC_DAC 2         // ADC input from op amp after DAC
#define ADC_AMP 3         // ADC input over R2 before 11x op-amp
#define ADC_SGN 4         // ADC input from ampl (drop of R2) after 11x op amp

// Switch for Caps by ADCs
#define SW_UNB 14         // ADC IO 1 Switch UNB
#define SW_DAC 15         // ADC IO 2 Switch DAC
#define SW_AMP 16         // ADC IO 3 Switch AMP
#define SW_SGN 17         // ADC IO 4 Switch SGN

// Switch for R1 Cap
#define SW_R1_100 5       // R1 - 100 Ohm switch
#define SW_R1_1K  6       // R1 - 1K Ohm Switch
#define SW_R1_10K 7       // R1 - 10K ohm Switch
#define SW_Calib 21       // Calibration Resistor switch

// Switch for choosing probe
#define SW_AuP     10     // SW_Au+
#define SW_AuN     11     // SW_Au-
#define SW_AuP_GND 12     // SW_Au+_GND
#define SW_AuN_GND 13     // SW_Au-_GND  

// Switch for choosing guard
#define SW_AuP_ShP 33     // SW_Au+_SH+
#define SW_AuN_ShP 34     // SW_Au-_SH+
#define SW_AuP_ShN 35     // SW_Au+_SH-
#define SW_AuN_ShN 36     // SW_Au-_SH-

// RS485
#define RE_DE 40          // Controls transmit/receive
#define RO 39             //RX
#define DI 41             //TX
HardwareSerial RS485(1);  // Use UART1 for RS485

// Sensors
MS5837 sensor;

// Calibration factor for ADC
float calibrationFactor1 = 0.875; 
float calibrationFactor2 = 1;
float calibrationFactor3 = 1;
float calibrationFactor4 = 0.805;

// ===== SETUP ===============================================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);  // SDA=GPIO8, SCL=GPIO9 for I2C
  Wire.setClock(400000); //set dac to fast mode

  // Initialize MS5837 Pressure Sensor
  if (!sensor.begin(0)) {  // 0 = float math, 1 = integer math
    printBoth("MS5837 not found! Check wiring.");
  } else {
    printBoth("MS5837 detected.");
    sensor.setDensity(997); // fresh water density (kg/m³)
  }


  // RS485 Startup
  RS485.begin(9600, SERIAL_8N1, RO, DI); // RX=RO, TX=DI
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW); // Default to receive mode

  // --- Switches ---
  // R1 & Calib
  pinMode(SW_R1_100, OUTPUT);
  pinMode(SW_R1_1K, OUTPUT);
  pinMode(SW_R1_10K, OUTPUT);
  pinMode(SW_Calib, OUTPUT);
  // ADC
  pinMode(SW_UNB, OUTPUT);
  pinMode(SW_DAC, OUTPUT);
  pinMode(SW_AMP, OUTPUT);
  pinMode(SW_SGN, OUTPUT);
  //digitalWrite(SW_UNB, HIGH);
  //digitalWrite(SW_DAC, HIGH);
  //digitalWrite(SW_AMP, HIGH);
  //digitalWrite(SW_SGN, HIGH);
  // Probes
  pinMode(SW_AuP, OUTPUT);
  pinMode(SW_AuN, OUTPUT);
  pinMode(SW_AuP_GND, OUTPUT);
  pinMode(SW_AuN_GND, OUTPUT);
  // Shields
  pinMode(SW_AuP_ShP, OUTPUT);
  pinMode(SW_AuN_ShP, OUTPUT);
  pinMode(SW_AuP_ShN, OUTPUT);
  pinMode(SW_AuN_ShN, OUTPUT);
  
  delay(3000);  // Wait for serial connection
  Serial.println("=== Salinity Test ===");
  //Serial.println("Calibration Factor ADC 1: " + String(calibrationFactor1, 3));
  printBoth("Waiting for RS485 commands...");
  writeDAC(0);
}

// ===== LOOP =================================================================================================
void loop() {
  // Check RS485 for incoming commands
  if (RS485.available()) {
    String cmd = RS485.readStringUntil('\n');
    cmd.trim(); cmd.toLowerCase(); // Make case-insensitive
    handleCommand(cmd);
  }

  // Handle Serial Input
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toLowerCase();
    handleCommand(cmd);
  }
}

// ===== Sending Messages =====================================================================================
//--- Send String over RS485 ---
void sendRS485(String msg) {
  digitalWrite(RE_DE, HIGH);
  RS485.println(msg);
  RS485.flush();
  digitalWrite(RE_DE, LOW);
}

// --- Send over RS485 and Serial
void printBoth(String msg) {
  msg = "msg: " + msg;
  Serial.println(msg);
  sendRS485(msg);
}

//  ===== DAC =================================================================================================
// --- Function to write a value to the DAC (unclamped) ---
void writeDAC_Unclamped(uint16_t value) {
  if (value > DAC_MAX_5V) value = DAC_MAX_5V; // safety clamp
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(0x40);              // Fast mode command
  Wire.write(value >> 4);        // Upper 8 bits
  Wire.write((value & 0xF) << 4); // Lower 4 bits
  Wire.endTransmission();
}

// --- Function to write a value to the DAC ---
void writeDAC(uint16_t value) {
  if (value > DAC_MAX_3V3) value = DAC_MAX_3V3; // max output of DAC is 5V
  writeDAC_Unclamped(value);
}

// ===== Probes ===============================================================================================
// --- Function to put positive probe on/off ---
void posProbe(int mode) {
  digitalWrite(SW_AuP, mode);
  digitalWrite(SW_AuN_GND, mode);
  digitalWrite(SW_AuP_ShP, mode);
  digitalWrite(SW_AuN_ShN, mode);
}
// --- Function to put negative probe on/off ---
void negProbe(int mode) {
  digitalWrite(SW_AuN, mode);
  digitalWrite(SW_AuP_GND, mode);
  digitalWrite(SW_AuN_ShP, mode);
  digitalWrite(SW_AuP_ShN, mode);
}

// ===== ADC Read =============================================================================================
String readADCs() {
  int adc[4] = {analogRead(ADC_UNB), analogRead(ADC_DAC), analogRead(ADC_AMP), analogRead(ADC_SGN)};
  float volt[4];
  String csvRaw="", csvVolt="";
  for(int i=0;i<4;i++){
    volt[i]=(adc[i]/8191.0)*3.3; // covert from d to a
    //csvRaw += String(adc[i]) + (i<3?",":"");
    csvVolt += String(volt[i],3) + (i<3?",":"");
  }
  //return csvRaw+"\n"+csvVolt+"\n";
  return csvVolt;
}

String stringADC(int adcPin){
  float volt = (analogRead(adcPin)/8191.0)*3.3; // covert from d to a
  return String(volt);
}

// ===== Pressure Sensor ======================================================================================
// --- Get Pressure Sensor data ---
String readPressure() {
  if (sensor.read() == MS5837_OK) {
    float pressure = sensor.getPressure();     // mbar
    float temperature = sensor.getTemperature(); // °C
    float depth = sensor.getDepth();             // meters

    // Construct CSV string
    String csv = String(pressure, 2) + "," + String(temperature, 2) + "," + String(depth, 3);
    return csv;
  } else {
    return "ERROR";
  }
}

// ===== Handle Received RS485 Command ========================================================================
void handleCommand(String cmd) {
  printBoth("Received Command: " + cmd);

  cmd.trim();
  cmd.toLowerCase(); // make it case-insensitive
  if (cmd=="dac sweep"){
    testDAC_sweep();
  }
  else if (cmd.startsWith("dac ")) {  // command has a value
    float voltage = cmd.substring(3).toFloat();
    testDAC(voltage);
  }
  else if (cmd.startsWith("pv ")){
    //defaults
    double voltage = 1.5; int r1 = 2;
    //parse
    sscanf(cmd.c_str(), "pv %lf %d", &voltage, &r1);
    //run
    pointVoltage(voltage, r1);
  }
  else if (cmd.startsWith("calib ")){
    //defaults
    double voltage = 1.65; int r1 = 0;
    //parse
    sscanf(cmd.c_str(), "calib %lf %d", &voltage, &r1);
    //run
    calibFactor(voltage, r1);
  }
  else if (cmd.startsWith("dvsweep ")){
    int probe = cmd.substring(7).toInt();
    sweepVoltage(2,probe);
  }
  else if (cmd.startsWith("ac ")){
    //defaults 
    float freq = 100; float amp = 1.0;
    //parse
    sscanf(cmd.c_str(), "ac %f %f", &freq, &amp);
    generateSineWaveEIS(freq, amp,4.5);
  }

}


// ===== DAC Test (Output a dac volatge and test with multimeter and ADC) =====================================
void testDAC(float voltage){
  digitalWrite(SW_Calib, HIGH);
  digitalWrite(SW_R1_1K,HIGH);

  writeDAC(0);
  delay(2500);
  printBoth(readADCs());
  delay(2500);
  int dacVal = (int)((voltage / 5) * 4095); // scale 0–5V to 0–4095
  dacVal = constrain(dacVal, 0, 4095);
  writeDAC_Unclamped(dacVal);
  delay(5000);
  printBoth(readADCs());
  delay(5000);
  writeDAC(0);

  digitalWrite(SW_Calib, LOW);
  digitalWrite(SW_R1_1K,LOW);
}

// ===== DAC Test (Sweep DAC voltages) ==========================================================
void testDAC_sweep(){
  for (int i=0; i<=50; i++){
    float voltage = ((float)(i))/10;
    Serial.printf("Voltage: %f :", voltage);
    int dacVal = (int)((voltage / 5) * 4095);
    writeDAC_Unclamped(dacVal);
    delay(5000);
    printBoth(readADCs());
    delay(5000);
    //writeDAC(0);
    //delay(1000);
  }
  writeDAC(0);
}

// ===== get voltage over probes at a point =====================================================
void pointVoltage(float voltage, int r1){
  printBoth("PV: FWD, BCK, CLF:");
  int r1pins[] = {SW_R1_100, SW_R1_1K, SW_R1_10K};
  float r1Resistances[] = {100, 1000, 10000};
  int dacVal = (int)((voltage / 5) * 4095); // scale 0–5V to 0–4095

  float calibFact = calibFactor(voltage, 0);

  writeDAC(0); // set DAC to 0
  digitalWrite(r1pins[r1],HIGH); // turn on chosen R1 resistor
  posProbe(1); // turn on positive probe
  writeDAC(dacVal); // turn on DAC output
  delay(500);String forward = readADCs();Serial.println(stringADC(ADC_AMP));delay(500); // wait 0.5 secs read adcs and wait 0.5 secs
  writeDAC(0); //dac = 0
  posProbe(0);delay(3000); // turn of posprobe and leave it like that for three secs
  negProbe(1); //turn on negprobe
  writeDAC(dacVal); // turn on DAC output
  delay(500);String backward = readADCs();Serial.println(stringADC(ADC_AMP));delay(500); // wait 0.5 secs read adcs and wait 0.5 secs
  writeDAC(0); //dac = 0
  negProbe(0); digitalWrite(r1pins[r1],LOW);

  //print output
  printBoth(forward);
  printBoth(backward);
  printBoth(String(calibFact,4));

}

// ===== get calib factor =======================================================================
float calibFactor(float voltage, int r1){
  printBoth("Calib Factor:");
  int r1pins[] = {SW_R1_100, SW_R1_1K, SW_R1_10K};
  float r1Resistances[] = {100, 1000, 10000};
  float res = r1Resistances[r1];
  int dacVal = (int)((voltage / 5) * 4095); // scale 0–5V to 0–4095

  writeDAC(0);
  digitalWrite(SW_Calib, HIGH);
  digitalWrite(r1pins[r1],HIGH);
  writeDAC(dacVal);
  delay(500);
  int adcVal = analogRead(ADC_SGN);
  digitalWrite(SW_Calib, LOW);
  digitalWrite(r1pins[r1],LOW);
  float measured = (adcVal/8191.0)*3.3;
  float expected = (5/(res+5))*voltage*11;

  printBoth(String(measured,4));
  printBoth(String(expected,4));

  return expected/measured;
}

// ===== sweep voltage ==========================================================================
void sweepVoltage(int r1, int probe){
  printBoth("Voltage Sweep");
  printBoth(String(calibFactor(1.5, 0)));
  int r1pins[] = {SW_R1_100, SW_R1_1K, SW_R1_10K};
  digitalWrite(r1pins[r1],HIGH); // turn on chosen R1 resistor
  if (probe==0){posProbe(1);}else{negProbe(1);}
  for (int i=0; i<=20; i++){ // 0-2.5V
    float voltage = ((float)(i))/10;
    Serial.printf("Voltage: %f :", voltage);
    int dacVal = (int)((voltage / 5) * 4095);
    writeDAC_Unclamped(dacVal);
    delay(500);
    printBoth(readADCs());
    delay(500);
    writeDAC(0);
    delay(2000); //wait 2 secs betwee each reading
  }
  writeDAC(0);
  digitalWrite(r1pins[r1],LOW); // turn off chosen R1 resistor
  posProbe(0);negProbe(0);
}

// ===== ac generation ==========================================================================
void generateSineWaveEIS(float frequency, float amplitude_volts, float vref) {
  // Turn ON R pins
  //posProbe(1);
  digitalWrite(SW_Calib,HIGH);
  digitalWrite(SW_R1_100,HIGH);
  writeDAC(0);


  const int adc_pin = ADC_SGN; //ChECK
  const uint16_t SAMPLES_PER_CYCLE = 50;  // Fixed for consistent waveform shape
  const uint16_t NUM_CYCLES = 10;
  const uint16_t MAX_SAMPLES = 512;
  
  uint16_t samples_per_cycle = SAMPLES_PER_CYCLE;
  uint16_t total_samples = samples_per_cycle * NUM_CYCLES;

  
  // Safety check for array bounds
  if (total_samples > MAX_SAMPLES) {
    total_samples = MAX_SAMPLES;
  }
  
  // Arrays to store measurements
  uint16_t dac_values[MAX_SAMPLES];
  uint16_t adc_values[MAX_SAMPLES];
  unsigned long timestamps[MAX_SAMPLES];  // Store timestamps in microseconds
  
  // Convert voltage amplitude to DAC units
  uint16_t amplitude_dac = (uint16_t)((amplitude_volts / vref) * 4095.0);
  
  // Calculate timing based on frequency and samples per cycle
  float period = 1.0 / frequency;  // seconds
  float sample_interval = (period / samples_per_cycle) * 1000000.0;  // microseconds
  
  // Calculate amplitude midpoint
  uint16_t amp_midpoint = amplitude_dac / 2;
  
  // Pre-calculate sine lookup table
  uint16_t sine_lut[SAMPLES_PER_CYCLE];
  for (uint16_t i = 0; i < samples_per_cycle; i++) {
    float angle = (2.0 * PI * i) / samples_per_cycle;
    float sine_val = sin(angle);
    
    // Scale to amplitude range
    int32_t dac_value = amp_midpoint + (int32_t)(sine_val * (amplitude_dac / 2));
    
    // Clamp to valid 12 bit DAC range
    if (dac_value < 0) {
      sine_lut[i] = 0;
    } else if (dac_value > 4095) {
      sine_lut[i] = 4095;
    } else {
      sine_lut[i] = (uint16_t)dac_value;
    }
  }
  
  // Generate sine wave and capture synchronized measurements
  unsigned long start_time = micros();
  unsigned long next_sample_time = start_time;
  
  for (uint16_t i = 0; i < total_samples; i++) {
    // Write DAC value
    uint16_t dac_val = sine_lut[i % samples_per_cycle];
    writeDAC(dac_val);
    
    // Wait for DAC settling
    delayMicroseconds(2);
    
    // Read ADC synchronized with DAC output
    uint16_t adc_val = analogRead(adc_pin);
    
    // Store measurements
    dac_values[i] = dac_val;
    adc_values[i] = adc_val;
    timestamps[i] = micros() - start_time;  // Time since start in microseconds
    
    // Precise timing control
    next_sample_time += (unsigned long)sample_interval;
    while (micros() < next_sample_time) {
      // Tight loop for accurate timing
    }
  }

  // Turn OFF R pins
  //posProbe(0);
  digitalWrite(SW_Calib,LOW);
  digitalWrite(SW_R1_100,LOW);
  writeDAC(0);
  
  // Print results
  Serial.println("=== EIS Measurement Results ===");
  Serial.print("Frequency: ");
  Serial.print(frequency, 2);
  Serial.println(" Hz");
  Serial.print("Samples per cycle: ");
  Serial.println(samples_per_cycle);
  Serial.print("Total cycles: ");
  Serial.println(NUM_CYCLES);
  Serial.print("Amplitude: ");
  Serial.print(amplitude_volts, 3);
  Serial.println(" V peak-to-peak");
  Serial.print("Total Samples: ");
  Serial.println(total_samples);
  Serial.println();
  
  // Print DAC voltages as csv string
  Serial.println("DAC_V: ");
  for (uint16_t i = 0; i < total_samples; i++) {
    float voltage = (dac_values[i] / 4095.0) * vref;
    Serial.print(voltage, 4);
    if (i < total_samples - 1) {
      Serial.print(',');
    }
  }
  Serial.println();
  
  // Print ADC voltages as comma-separated string
  Serial.println("ADC_V: ");
  for (uint16_t i = 0; i < total_samples; i++) {
    float voltage = (adc_values[i] / 8191.0) * 3.3;
    Serial.print(voltage, 4);
    if (i < total_samples - 1) {
      Serial.print(',');
    }
  }
  Serial.println();
  
  // Print timestamps as comma-separated string (microseconds)
  Serial.println("Time_us: ");
  for (uint16_t i = 0; i < total_samples; i++) {
    Serial.print(timestamps[i]);
    if (i < total_samples - 1) {
      Serial.print(',');
    }
  }
  Serial.println();
  
  Serial.println("=== End ===");
  Serial.println();
}
