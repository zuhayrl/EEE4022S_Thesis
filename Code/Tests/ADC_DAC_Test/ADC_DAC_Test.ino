// Pot-controlled LED blink speed + DAC output with Serial printing
// Pot on GPIO34 (ADC1), LED on GPIO2, DAC on GPIO25

const int potPin = 34;   // ADC input (ADC1)
const int ledPin = 2;    // LED digital output
const int dacPin = 25;   // DAC output (0-255)

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // Read potentiometer
  int potRaw = analogRead(potPin); // 0–4095
  // Map pot to blink delay: 1000 ms (slow) to 30 ms (fast)
  int blinkDelay = map(potRaw, 0, 4095, 1000, 30);
  // Map pot to DAC value 0–255
  int dacVal = map(potRaw, 0, 4095, 0, 255);

  // Output DAC voltage proportional to pot
  dacWrite(dacPin, dacVal);

  // Blink LED
  digitalWrite(ledPin, HIGH);
  delay(blinkDelay);
  digitalWrite(ledPin, LOW);
  delay(blinkDelay);

  // Print input and output values
  Serial.print("ADC Input (raw): ");
  Serial.print(potRaw);
  Serial.print(" | Blink Delay: ");
  Serial.print(blinkDelay);
  Serial.print(" ms | DAC Output: ");
  Serial.println(dacVal);
}
