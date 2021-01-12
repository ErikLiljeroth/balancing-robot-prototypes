#include <Adafruit_MPU6050.h> // Adafruit mpu-6050 library
#include <Adafruit_Sensor.h> // Adafruit Sensor library
#include <Wire.h> // I2C communication
#include "LCD_I2C.h" // LCD library

Adafruit_MPU6050 mpu;
LCD_I2C lcd(0x27); // Default address of most PCF8574 modules is 0x27, change according

// placeholders for sensor values
sensors_event_t a, g, temp;

// Possibility to save and test calibration values. To estimate them, put all of them = 9.8.
float z_neg_c = 9.8;// -> 10.77 // mpu-robot: 8.98 8.94 8.98
float z_pos_c = 9.8;// -> 9.44 // mpu-robot: 10.05 10.04 10.04
float y_neg_c = 9.8;// -> 9.83 // mpu-robot: 9.87 9.85 9.84
float y_pos_c = 9.8;// -> 9.70 // mpu-robot: 9.85 9.85 9.85
float x_neg_c = 9.8;// -> 9.63 // mpu-robot: 9.91 9.94 9.94
float x_pos_c = 9.8;// -> 10.14 // mpu-robot: 9.52 9.52 9.52

// calibration constants to be displayed on LCD:
float acc_z_neg, acc_z_pos, acc_x_neg, acc_x_pos, acc_y_neg, acc_y_pos;
float last_time;
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

    // If you are using more I2C devices using the Wire library use lcd.begin(false), 
    // this stops the library(LCD_I2C) from calling Wire.begin()
    lcd.begin(false);
    lcd.backlight();
    delay(100);

    // intro to calibration
    countDown("Calibrate az+", 5);
    while(calibrateAcc("z", "+", 1.0));

    lcd.setCursor(0,0);
    lcd.print("Az+ calibrated");
    delay(1000);

    countDown("Calibrate az-", 5);
    while(calibrateAcc("z", "-", 1.0));

    lcd.clear();
    lcd.print("Az- calibrated  ");
    delay(1000);

    // ----------------------------------------
    countDown("Calibrate ax+", 5);
    while(calibrateAcc("x", "+", 1.0));

    lcd.setCursor(0,0);
    lcd.print("ax+ calibrated");
    delay(1000);

    countDown("Calibrate ax-", 5);
    while(calibrateAcc("x", "-", 1.0));

    lcd.clear();
    lcd.print("ax- calibrated  ");
    delay(1000);

    // -----------------------------------------
    countDown("Calibrate ay+", 5);
    while(calibrateAcc("y", "+", 1.0));

    lcd.setCursor(0,0);
    lcd.print("Ay+ calibrated");
    delay(1000);

    countDown("Calibrate ay-", 5);
    while(calibrateAcc("y", "-", 1.0));

    lcd.clear();
    lcd.print("Ay- calibrated  ");
    delay(1000);

    // ------------------------------------------

    lcd.clear();
    lcd.print("all calibrated  ");
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
    lcd.print(acc_z_neg * 9.8/z_neg_c);
    lcd.setCursor(0, 1);
    lcd.print("z_pos: ");
    lcd.print(acc_z_pos * 9.8/z_pos_c);

    delay(5000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("x_neg: ");
    lcd.print(acc_x_neg * 9.8/x_neg_c);
    lcd.setCursor(0, 1);
    lcd.print("x_pos: ");
    lcd.print(acc_x_pos * 9.8/x_pos_c);

    delay(5000);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("y_neg: ");
    lcd.print(acc_y_neg * 9.8/y_neg_c);
    lcd.setCursor(0, 1);
    lcd.print("y_pos: ");
    lcd.print(acc_y_pos * 9.8/y_pos_c);

    delay(5000);

}

bool calibrateAcc(String axis, String sign, float accTrshhld) {
  
  float acc_mean = 0;
  int sample_counter = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("gathering data..");

  float acc_x, acc_y, acc_z;

  //Gather data
  for (uint16_t i = 0; i< 500; i++) {
    mpu.getEvent(&a, &g, &temp);
    delay(5);
    acc_x = a.acceleration.x;
    acc_y = a.acceleration.y;
    acc_z = a.acceleration.z;

    if (axis == "z" && abs(acc_x) < accTrshhld && abs(acc_y) < accTrshhld) {
      acc_mean += acc_z;
      sample_counter += 1;
    }
    else if (axis == "x" && abs(acc_y) < accTrshhld && abs(acc_z) < accTrshhld) {
      acc_mean += acc_x;
      sample_counter += 1;
    }
    else if (axis == "y" && abs(acc_x) < accTrshhld && abs(acc_z) < accTrshhld) {
      acc_mean += acc_y;
      sample_counter += 1;
    }
    else if (axis == "z" || axis == "x" || axis == "y") {
      Serial.println("Align better!");
      Serial.print(acc_x);
      Serial.print(",");
      Serial.print(acc_y);
      Serial.print(",");
      Serial.print(acc_z);
      Serial.println("");
      delay(100);
    }
    else {
      countDown("undef axis!", 3);
      while (1){delay(100);};
    }

    delay(5);
  }

  lcd.setCursor(0, 0);
  lcd.print("done!             ");
  delay(1000);
  lcd.clear();

  // check that data was gathered
  if (sample_counter <= 100) {
    countDown("align better " + axis + sign, 5);
    return 1;
  }

  // Compute the sample mean
  acc_mean /= sample_counter;

  if (sign == "+" && acc_mean < 0 || sign == "-" && acc_mean > 0) {
    countDown("turn upside down", 3);
    return 1;
  }

  if (axis == "x") {
    if (sign == "+") {
      acc_x_pos = acc_mean;
      return 0;
    }
    else {
      acc_x_neg = acc_mean;
      return 0;
    }
  }
  else if (axis == "y") {
    if (sign == "+") {
      acc_y_pos = acc_mean;
      return 0;
    }
    else {
      acc_y_neg = acc_mean;
      return 0;
    }
  }
  else if (axis == "z")
    if (sign == "+") {
      acc_z_pos = acc_mean;
      return 0;
    }
    else {
      acc_z_neg = acc_mean;
      return 0;
  }
  else {
    countDown("wrong input..", 5);
    return 1;
  }
}

// Helper functions

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
