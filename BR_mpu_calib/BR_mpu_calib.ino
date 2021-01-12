#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "LCD_I2C.h"

Adafruit_MPU6050 mpu;
LCD_I2C lcd(0x27); // Default address of most PCF8574 modules is 0x27, change according

// Calibration variables
float gyro_x_offset, gyro_y_offset, gyro_z_offset;

// Kalman Filter sample rate
#define DT 10000 // 10 msec time step for main loop -> 100 Hz
// lcd fps
#define DT_lcd 250 // 4 fps for LCD screen

const float dt = 0.01; // Kalman Filter time step in seconds

// Sensor measurements placeholders
sensors_event_t a, g, temp;

float pitch_est; // variable for pitch estimation
float pitch_meas, rate_meas; // computed pitch & rate from sensor measurements

float pitch_est_k, bias_est_k; // Kalman filter states
uint32_t last_time = 0;
uint32_t last_time_lcd = 0;

void setup() {
  // Serial
  Serial.begin(115200);
  
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);

  // If you are using more I2C devices using the Wire library use lcd.begin(false)
  lcd.begin(false); // lcd.begin(false) stop the library(LCD_I2C) from calling Wire.begin()
  lcd.backlight();
  delay(100);

  Serial.println("Calibrating gyro...");
  
  lcd.print("Calibrating gyro");
  lcd.setCursor(0, 6);
  lcd.print("...");
  while (calibrateGyro());
  Serial.println("Gyro calibrated.");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Gyro calibrated!");
  delay(1000);
  lcd.clear();
  Serial.println("Measurements started:");

  // initialize pitchEst, rateEst
  pitch_est = 0;

  pitch_est_k = 0;
  bias_est_k = 0;
  Serial.println("Complementary_filter, Kalman_filter");
}

void loop() {
  
  // Get new sensor events with the readings
  mpu.getEvent(&a, &g, &temp);
  pitch_meas = estimatePitchFromAcc(a); // "measured pitch"
  pitch_est = complementaryFilter(pitch_meas, (g.gyro.x - gyro_x_offset)*RAD_TO_DEG, dt);
  kalmanFilter(pitch_meas, (g.gyro.x - gyro_x_offset)*RAD_TO_DEG, dt);

  // Print out the values
  //Serial.print(a.acceleration.x);
  //Serial.print(",");
  //Serial.print(a.acceleration.y);
  //Serial.print(",");
  //Serial.print(a.acceleration.z);
  //erial.println("");
  //Serial.print((g.gyro.x - gyro_x_offset)*RAD_TO_DEG);
  //Serial.print(",");
  //Serial.print((g.gyro.y - gyro_y_offset)*RAD_TO_DEG);
  //Serial.print(",");
  //Serial.print((g.gyro.z - gyro_z_offset)*RAD_TO_DEG);
  Serial.print(pitch_est);
  Serial.print(",");
  Serial.print(pitch_est_k);
  Serial.println("");

  // update lcd with 1/DT_lcd Hz
  if (millis() - last_time_lcd > DT_lcd) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("pitch: ");
    lcd.print(pitch_est_k,1);
    lcd.print("deg");
    last_time_lcd = millis();
  }

  while (micros() - last_time < DT);
  last_time = micros();
}



int countSampleRate() {
  float t_0 = millis();
  int counter = 0;

  while (millis()-t_0 < 30000 ) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    counter = counter+1;
    delay(8);
  }
  Serial.println(counter/30);
  return counter/30;
}

bool calibrateGyro() {
  int16_t gyro_x_buffer[100], gyro_y_buffer[100], gyro_z_buffer[100];
  gyro_x_offset = 0;
  gyro_y_offset = 0;
  gyro_z_offset = 0;

  for (uint8_t i = 0; i < 100; i++) {
    mpu.getEvent(&a, &g, &temp);
    gyro_x_buffer[i] = g.gyro.x;
    gyro_y_buffer[i] = g.gyro.y;
    gyro_z_buffer[i] = g.gyro.z;
    gyro_x_offset += g.gyro.x;
    gyro_y_offset += g.gyro.y;
    gyro_z_offset += g.gyro.z;
    delay(10);
  }

  if (!checkMinMaxGyro(gyro_x_buffer, 100, 15) || !checkMinMaxGyro(gyro_y_buffer, 100, 15) || !checkMinMaxGyro(gyro_z_buffer, 100, 15)) { // check min and max differ by no more than 15 deg/s
    return 1;
  }
  else {
    gyro_x_offset /= 100.0f;
    gyro_y_offset /= 100.0f;
    gyro_z_offset /= 100.0f;
    return 0;
  }
}

bool checkMinMaxGyro(int16_t *array, uint8_t length, int16_t maxDifference) {
  int16_t min = array[0], max = array[0];
  for (uint8_t i = 1; i< length; i++) {
    if (array[i] < min){
      min = array[i];
      }
    else if (array[i] > max) {
      max = array[i];
      }
  }
  return (max-min)*57.3f < maxDifference; // conversion to deg/s
}


float estimatePitchFromAcc(sensors_event_t a) {
  return -atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
}

float complementaryFilter(float pitch_meas, float rate_meas, float dt) {
  return 0.95f*(pitch_est + dt*rate_meas) + 0.05f*pitch_meas;
}

void kalmanFilter(float pitch_meas, float rate_meas, float dt) { // meas = measurement
  const float K[2] = {0.08, -0.060}; // {0.033810, -0.025380}
 
  float y_e = (pitch_meas - pitch_est_k);
  pitch_est_k += dt*(rate_meas-bias_est_k); // - bias_est_k
  pitch_est_k += K[0] * y_e;
  bias_est_k  += K[1] * y_e;
  
}
