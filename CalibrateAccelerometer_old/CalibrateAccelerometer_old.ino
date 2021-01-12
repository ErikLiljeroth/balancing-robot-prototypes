#include <Adafruit_MPU6050.h> // Adafruit mpu-6050 library
#include <Adafruit_Sensor.h> // Adafruit Sensor library
#include <Wire.h>
#include "LCD_I2C.h" // LCD library

Adafruit_MPU6050 mpu;
LCD_I2C lcd(0x27); // Default address of most PCF8574 modules is 0x27, change according

// Sample rate
#define DT 10000 // 10 msec time step for main loop, 10000 mu-sec
#define fHz 100 // 100Hz sample rate for main loop
// lcd fps
#define DT_lcd 250 // 4 fps LCD screen

sensors_event_t a, g, temp;

// Calibration values
float acc_z_neg, acc_z_pos, acc_x_neg, acc_x_pos, acc_y_neg, acc_y_pos;
float last_time;
uint32_t last_time_lcd = 0;

void setup() {
    // Serial
    /*
    Serial.begin(115200);
  
    while (!Serial) {
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    }
    */

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

    // Start LCD
    lcd.begin(false); // If you are using more I2C devices using the Wire library use lcd.begin(false)
                    // this stop the library(LCD_I2C) from calling Wire.begin()
    lcd.backlight();
    delay(100);


    // intro to calibration
    countDown("Calibrate az+", 5);
    while(calibrateAcc("z", "+", 0.5));

    lcd.setCursor(0,0);
    lcd.print("Az+ calibrated");
    delay(1000);

    countDown("Calibrate az-", 5);
    while(calibrateAcc("z", "-", 0.5));

    lcd.clear();
    lcd.print("Az- calibrated  ");
    delay(1000);

    lcd.setCursor(0, 0);
    lcd.print("Start display   ");
    lcd.setCursor(0,1);
    lcd.print("calib values    ");
    delay(2000);

}

void loop() {
    // print the resulting calibration values on LCD to note for future use

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("z_neg: ");
    lcd.print(acc_z_neg);
    lcd.setCursor(0, 1);
    lcd.print("z_pos: ");
    lcd.print(acc_z_pos);

    while(1){delay(10);};
}

void countDown(String message, uint8_t length) {

  // 5 sec countDown
  lcd.clear();
  delay(5);
  lcd.setCursor(0,0);
  lcd.print(message);
  if (!(length == 3)) {
  // 5
  lcd.setCursor(0,1);lcd.print("5  ");delay(333);lcd.setCursor(2,1);lcd.print(".");delay(333);lcd.print(".");delay(333);
  // 4
  lcd.setCursor(0,1);lcd.print("4  ");delay(333);lcd.setCursor(2,1);lcd.print(".");delay(333);lcd.print(".");delay(333);
  }
  // 3
  lcd.setCursor(0,1);lcd.print("3  ");delay(333);lcd.setCursor(2,1);lcd.print(".");delay(333);lcd.print(".");delay(333);
  // 2
  lcd.setCursor(0,1);lcd.print("2  ");delay(333);lcd.setCursor(2,1);lcd.print(".");delay(333);lcd.print(".");delay(333);
  // 1
  lcd.setCursor(0,1);lcd.print("1  ");delay(333);lcd.setCursor(2,1);lcd.print(".");delay(333);lcd.print(".");delay(333);
  // Go!
  lcd.setCursor(0,1);lcd.print("Go!");delay(500);lcd.clear();
}

bool calibrateAcc(String axis, String sign, float accTrshhld) {
  int16_t acc_x_buffer[200], acc_y_buffer[200], acc_z_buffer[200];
  float acc_x_mean = 0;
  float acc_y_mean = 0;
  float acc_z_mean = 0;

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("gathering data..");
  // Gather data while moving the accelerometer around "z in g direction"
  last_time = micros();
  for (uint8_t i = 0; i< 200; i++) {
    mpu.getEvent(&a, &g, &temp);
    acc_x_buffer[i] = a.acceleration.x*1000;
    acc_y_buffer[i] = a.acceleration.y*1000;
    acc_z_buffer[i] = a.acceleration.z*1000;
    // while (micros() - last_time < DT); // constant sample rate 100Hz
    // last_time = micros();
    delay(20);
  }
  lcd.setCursor(0, 0);
  lcd.print("done!      ");
  delay(1000);
  lcd.clear();

  // count nbr samples where axis is sufficiently aligned with g
  float counter = 0;

  // Go through data and only keep samples where axis is approximately aligned with g
  for (uint8_t i = 0; i< 200; i++) {
    if (axis == "z") {
      if (abs(acc_y_buffer[i]/1000.f) < accTrshhld && abs(acc_x_buffer[i]/1000.f) < accTrshhld) {
        acc_z_mean += acc_z_buffer[i];
        counter = counter + 1;
      }
    }
    else if (axis == "x") {
      countDown("x not implemented...", 5);
    }
    else if (axis == "y") {
      countDown("y not implemented...", 5);
    }
    else {
      // wrong input - check this
      countDown("Error", 5);
    }
  }

  if (axis == "z") {  
    if (counter < 10) {
      // display msg: not properly aligned, align closer to g
      countDown("align better "+ axis + sign, 3);
      return 1;
    }
    acc_z_mean = acc_z_mean/1000.0;
    acc_z_mean = acc_z_mean/counter;
    if (sign == "+" && acc_z_mean > 0) {
      acc_z_pos = acc_z_mean;
      return 0;
    }
    else if (sign == "-" && acc_z_mean < 0) {
      acc_z_neg = acc_z_mean;
      return 0;
    }
    else {
      // display msg rotate 180 deg
      countDown("turn upside down", 3);
      return 1;
    }
  }

  else if (axis == "x") {
    // TODO
  }
  else if (axis == "y") {
    // TODO
  }
  
  // errr... check this
  countDown("End of method", 5);
  return 0;
}

/*
bool calibrateAccZ() {
  int16_t acc_x_buffer[100], acc_y_buffer[100], acc_z_buffer[100];
  float acc_x_mean = 0;
  float acc_y_mean = 0;
  float acc_z_mean = 0;

  for (uint8_t i = 0; i < 100; i++) {
    mpu.getEvent(&a, &g, &temp);
    acc_x_buffer[i] = a.acceleration.x;
    acc_y_buffer[i] = a.acceleration.y;
    acc_z_buffer[i] = a.acceleration.z;
    acc_x_mean += a.acceleration.x;
    acc_y_mean += a.acceleration.y;
    acc_z_mean += a.acceleration.z;
    delay(10);
  }

  acc_x_mean /= 100.0f;
  acc_y_mean /= 100.0f;
  acc_z_mean /= 100.0f;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("1-sec acc means");
  lcd.setCursor(1,0);
  lcd.print("x: ");
  lcd.print(acc_x_mean,1);
  lcd.print("|y: ");
  lcd.print(acc_y_mean,1);
  lcd.print("|z: ");
  lcd.print(acc_z_mean,1);
  delay(1000);

  if (!checkMinMaxAcc(acc_x_buffer, 100, 0.1) || !checkMinMaxAcc(acc_y_buffer, 100, 0.1)) {
    return 1;
  }
  else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("calib_vals:");
    lcd.setCursor(1,0);
    lcd.print("x: ");
    lcd.print(acc_x_mean,1);
    lcd.print("|y: ");
    lcd.print(acc_y_mean,1);
    lcd.print("|z: ");
    lcd.print(acc_z_mean,1);
    delay(10000);
    return 0;
  }
}
*/

/*
bool checkMinMaxAcc(int16_t *array, uint8_t length, int16_t maxDifference) {
  int16_t min = array[0], max = array[0];
  for (uint8_t i = 1; i< length; i++) {
    if (array[i] < min){
      min = array[i];
      }
    else if (array[i] > max) {
      max = array[i];
      }
  }
  return min < 0 && max > 0 && max-min < maxDifference;
}
*/
