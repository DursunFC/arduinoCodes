#include <Wire.h>

// Define PWM output pins
const int axPin = 2;  // PWM pin for AX
const int ayPin = 4; // PWM pin for AY

const int threshold = 1000;
const int zeroThreshold = 3 * 182; // Approximate value for ~3 degrees (32767 / 180 * 3)

void setup() {
  Serial.begin(115200);
  // Initialize I2C communication
  Wire.begin();

  // Set up PWM pins as outputs
  pinMode(axPin, OUTPUT);
  pinMode(ayPin, OUTPUT);

  // Start communication with MPU-6050
  Wire.beginTransmission(0x68); // MPU-6050 I2C address
  Wire.write(0x6B);             // Access power management register
  Wire.write(0);                // Wake up the sensor
  Wire.endTransmission(true);
}

void loop() {
  // Request accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true); // Request 6 bytes

  // Read accelerometer data
  int16_t ax = (Wire.read() << 8 | Wire.read());
  int16_t ay = (Wire.read() << 8 | Wire.read());

  // Map raw accelerometer data (-32768 to 32767) to PWM range (0 to 255)
  int pwmAX = (abs(ax) < zeroThreshold) ? 0 : map(ax, -32768, 32767, 0, 255);
  int pwmAY = (abs(ay) < zeroThreshold) ? 0 : map(ay, -32768, 32767, 0, 255);

  // Ensure values near zero are set to zero to avoid mid-scale output
  if (pwmAX > 124 && pwmAX < 131) pwmAX = 0;
  if (pwmAY > 124 && pwmAY < 131) pwmAY = 0;

  // Print values for debugging
  Serial.print("AX: ");
  Serial.print(ax);
  Serial.print(" PWM_AX: ");
  Serial.println(pwmAX);

  Serial.print("AY: ");
  Serial.print(ay);
  Serial.print(" PWM_AY: ");
  Serial.println(pwmAY);

  // Output PWM signal to the corresponding pins
  analogWrite(axPin, pwmAX);
  analogWrite(ayPin, pwmAY);

  // Check if AX or AY exceeds the threshold
  if (abs(ax) > threshold || abs(ay) > threshold) {
    digitalWrite(axPin, HIGH); // Turn on the buzzer
  } else {
    digitalWrite(axPin, LOW);  // Turn off the buzzer
  }

  delay(50); // Small delay for stable output
}