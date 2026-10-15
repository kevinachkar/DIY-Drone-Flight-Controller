#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include "MPU6050_Mahony_DynamicNotch.h"
#include "TiltCompensatedMag.h"
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "NonBlockingBaro.h"
#include "KalmanVertVel.h"

// ================= CALIBRATION STORAGE =================
int16_t xMin = 32767, xMax = -32768;
int16_t yMin = 32767, yMax = -32768;
int16_t zMin = 32767, zMax = -32768;

float xOffset, yOffset, zOffset;

NonBlockingBaro baro;


#define GPS_RX_PIN 7  // Connected to GPS TX
#define GPS_TX_PIN 8  // Connected to GPS RX


SFE_UBLOX_GNSS myGNSS;

// Home position
double homeLat = 0.0;
double homeLon = 0.0;
double homeAlt = 0.0;
bool homeSet = false;

bool gpsStarted = false;
unsigned long gpsStartTime = 0;

// Earth radius (meters)
const double EARTH_RADIUS = 6378137.0;

// ESP32 HardwareSerial port 1
HardwareSerial gpsSerial(1);

//////////LOOP LIMITER/////////////
const unsigned long LOOP_PERIOD_US = 2380;  // ~420Hz
unsigned long loopStartMicros = 0;
/////////////////////

////////LOOP SPEED MONITOR////////
unsigned long loopMicros_prev = 0;
float loopHz_now = 0;
float loopHz_avg = 0;
unsigned long printTimer = 0;

////////IMU////////
#define SDA_PIN 10
#define SCL_PIN 9

MPU6050_Mahony imu(SDA_PIN, SCL_PIN);

TiltCompensatedMag compass(10, 9);

KalmanVertVel velocityEstimator(imu, baro);

///////LOW PASS FILTER GYRO////////
double gyroX_f = 0, gyroY_f = 0, gyroZ_f = 0;
double roll_f = 0, pitch_f = 0;

// filter strength (0.0 → very smooth, 1.0 → no filtering)
double alpha = 0.6;  //0.65

///////LOW PASS FILTER MAG////////
double MagHeading = 0;

double alpha_mag = 0.3;


////gps+baro complementary filter/////////////

float alphaAlt = 0.95;

/////////////////

bool dnf = false; //dynamic notch filter

/////////////////

#define ESC1_PIN 0   //front left
#define ESC2_PIN 21  //front right
#define ESC3_PIN 1   //rear left
#define ESC4_PIN 20  //rear right

#define CE_PIN 3
#define CSN_PIN 2
#define SCK_PIN 4
#define MOSI_PIN 6
#define MISO_PIN 5

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int motorFL, motorFR, motorRL, motorRR;

int throttle = 0;

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

//----RECEIVER NRF---//
struct JoystickData {
  int16_t joy1x;  //roll
  int16_t joy1y;  //throttle
  int16_t joy2x;  //pitch
  int16_t joy2y;  //yaw
  int16_t btn1;   //se
  int16_t btn2;   //sa
  int16_t btn3;   //sb
  int16_t btn4;   //sd
  int16_t btn5;   //sc
};

JoystickData joyData;
///////////////////////////////////////////

////////////////FAIL SAFE///////////////

unsigned long lastRadioTime = 0;
bool failsafe = false;

////////////////////////////////////////

unsigned long lastTime;
double dt = 0.01;  // time step in seconds

// ----- Outer PID Roll(Angle) -----
double roll_setpoint = 0;
double roll_input;
double roll_error, roll_previousError = 0;
double roll_integral = 0, roll_derivative = 0;
double roll_Kp = 10.0, roll_Ki = 0.05, roll_Kd = 0.0;  // 3d prop: 14 / 0 / 0.02    DJI prop: 8 / 0 / 0.01
double roll_output1;                                   //                           DJI prop: 7 / 0 / 0.012
                                                       //                           DJI prop: 13.0 / 0.05 / 0.0  no vobration
// ----- Inner PID Roll (Rate) -----
double roll_input2;
double roll_error2, roll_previousError2 = 0;
double roll_integral2 = 0, roll_derivative2 = 0;
double roll_Kp2 = 1.0, roll_Ki2 = 0.0, roll_Kd2 = 0.035;  // 3d prop: 2 / 0 / 0.1    DJI prop: 1.5 / 0 / 0.05
double roll_output2;                                      //                          DJI prop: 1.4 / 0 / 0.05
                                                          //                          DJI prop: 1.0 / 0 / 0.035

// ----- Outer PID Ptich(Angle) -----
double pitch_setpoint = 0;
double pitch_input;
double pitch_error, pitch_previousError = 0;
double pitch_integral = 0, pitch_derivative = 0;
double pitch_Kp = 10.0, pitch_Ki = 0.05, pitch_Kd = 0.0;  // 3d prop: 14 / 0 / 0.02   DJI prop: 8 / 0 / 0.01
double pitch_output1;                                     //                          DJI prop: 7 / 0 / 0.012

// ----- Inner PID Pitch (Rate) -----
double pitch_input2;
double pitch_error2, pitch_previousError2 = 0;
double pitch_integral2 = 0, pitch_derivative2 = 0;
double pitch_Kp2 = 1.0, pitch_Ki2 = 0.0, pitch_Kd2 = 0.035;  // 3d prop: 2 / 0 / 0.1    DJI prop: 1.5 / 0 / 0.05
double pitch_output2;                                        //                         DJI prop: 1.4 / 0 / 0.05

// ----- Outer PID yaw(Angle) -----
double yaw_setpoint1 = 0;
double yaw_input;
double yaw_error, yaw_previousError = 0;
double yaw_integral = 0, yaw_derivative = 0;
double yaw_Kp = 2.0, yaw_Ki = 0.0, yaw_Kd = 0.0;  // 3d prop: 14 / 0 / 0.02   DJI prop: 8 / 0 / 0.01
double yaw_output1;

// ----- Inner PID Yaw (Rate) -----
double yaw_setpoint2 = 0;
double yaw_input2;
double yaw_error2, yaw_previousError2 = 0;
double yaw_integral2 = 0, yaw_derivative2 = 0;
double yaw_Kp2 = 30.0, yaw_Ki2 = 0.0, yaw_Kd2 = 0.0;  //best values : 30 / 0 / 0.0
double yaw_output2;
/////////////////////////////////////////////////////

// ----- Outer PID PosN -----
double N_setpoint = 0;
double N_input;
double N_error, N_previousError = 0;
double N_integral = 0, N_derivative = 0;
double N_Kp = 0.35, N_Ki = 0.02, N_Kd = 0.05;  //best 0.3/0.02/0.1
double N_output1;

// ----- Inner PID velN -----
double N_input2;
double N_error2, N_previousError2 = 0;
double N_integral2 = 0, N_derivative2 = 0;
double N_Kp2 = 3.0, N_Ki2 = 0.1, N_Kd2 = 0.0;  // best 4.0/0.0/0.0
double N_output2;
////////////////////////////

// ----- Outer PID posE -----
double E_setpoint = 0;
double E_input;
double E_error, E_previousError = 0;
double E_integral = 0, E_derivative = 0;
double E_Kp = 0.35, E_Ki = 0.02, E_Kd = 0.05;
double E_output1;

// ----- Inner PID velE -----
double E_input2;
double E_error2, E_previousError2 = 0;
double E_integral2 = 0, E_derivative2 = 0;
double E_Kp2 = 3.0, E_Ki2 = 0.1, E_Kd2 = 0.0;
double E_output2;
////////////////////////////

// ----- Outer PID posD -----
double D_setpoint = 0;
double D_input;
double D_error, D_previousError = 0;
double D_integral = 0, D_derivative = 0;
double D_Kp = 1.1, D_Ki = 0.0, D_Kd = 0.02;  // best 1.1/0.0/0.02
double D_output1;

// ----- Inner PID velD -----
double D_input2;
double D_error2, D_previousError2 = 0;
double D_integral2 = 0, D_derivative2 = 0;
double D_Kp2 = 65.0, D_Ki2 = 0.0, D_Kd2 = 0.0;  // best 65/0/0
double D_output2;
////////////////////////////

double posNorth = 0.0;
double posEast = 0.0;
double relativeAltitude = 0.0;

float velNorth = 0.0f;
float velEast = 0.0f;
float velDown = 0.0f;

float FusedAlt;

int failsafe_step = 0;
int failsafe_save = 0;
float rth_target_alt = 10.0;

bool posHoldSaved = false;

int alt_save = 0;

int hover_throttle = 1400;

int hover_pressed = 0;
int hover_time = 0;

int calibration_pressed = 0;
int calibration_time = 0;

double h = 0;
double right = 0;
double forward = 0;



void setup() {

  Serial.begin(115200);
  lastTime = micros();

  imu.begin();
  imu.calibrate();
  baro.begin(SDA_PIN, SCL_PIN);
  velocityEstimator.begin();
  velocityEstimator.setLPF(0.30f);

  //////////////////// GPS //////////////////
  Serial.println("Initializing ESP32-C3 Supermini GPS (NEO-M8N)...");

  bool gpsConnected = false;

  // --------------------------------------------------
  // STEP 1: Try 115200 first
  // --------------------------------------------------
  Serial.println("Trying GPS at 115200 baud...");

  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);

  if (myGNSS.begin(gpsSerial)) {
    // GPS is already at 115200
    Serial.println("GPS detected at 115200 baud.");
    gpsConnected = true;
  }

  // --------------------------------------------------
  // STEP 2: If 115200 failed, try 9600
  // --------------------------------------------------
  if (!gpsConnected) {

    Serial.println("GPS not detected at 115200.");
    Serial.println("Trying GPS at 9600 baud...");

    gpsSerial.end();
    delay(100);

    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(100);

    if (myGNSS.begin(gpsSerial)) {

      Serial.println("GPS detected at 9600 baud.");
      Serial.println("Changing GPS baud rate to 115200...");

      // Tell GPS to change its UART1 baud rate
      myGNSS.setSerialRate(115200);

      // Give GPS time to change its baud rate
      delay(200);

      // Restart ESP32 UART at 115200
      gpsSerial.end();
      delay(100);

      gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
      delay(100);

      // Reconnect library at new baud rate
      if (myGNSS.begin(gpsSerial)) {
        Serial.println("GPS successfully switched to 115200 baud.");
        gpsConnected = true;
      } else {
        Serial.println("ERROR: GPS failed to reconnect at 115200 baud.");
      }
    }
  }

  // --------------------------------------------------
  // STEP 3: If neither baud rate worked
  // --------------------------------------------------
  if (!gpsConnected) {
    Serial.println("ERROR: Could not communicate with GPS at 9600 or 115200!");
    while (1)
      ;
  }

  // --------------------------------------------------
  // GPS configuration
  // --------------------------------------------------

  // Turn off NMEA output and use UBX only
  myGNSS.setUART1Output(COM_TYPE_UBX);

  // Set navigation rate to 10 Hz
  myGNSS.setNavigationFrequency(10);

  // Enable Auto-PVT
  myGNSS.setAutoPVT(true);

  // Save configuration
  myGNSS.saveConfiguration();

  Serial.println("GPS successfully configured at 115200 baud and 10Hz!");

  ///////////Magnometer//////////
  if (!compass.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      ;
  }

  compass.setMagOffsets(-460, -51, 119);
  compass.setHeadingOffset(0);
  ///////////////////////////////

  imu.enableDynamicNotch = dnf;

  esc1.setPeriodHertz(300);
  esc2.setPeriodHertz(300);
  esc3.setPeriodHertz(300);
  esc4.setPeriodHertz(300);

  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  esc3.attach(ESC3_PIN, 1000, 2000);
  esc4.attach(ESC4_PIN, 1000, 2000);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  Serial.println("Receiver ready");

  N_setpoint = 0;
  E_setpoint = 0;
  D_setpoint = 0;
}

void loop() {

  loopStartMicros = micros();

  ////// LOOP SPEED MEASURE ///////
  unsigned long loopMicros_now = micros();
  unsigned long loopDelta = loopMicros_now - loopMicros_prev;
  loopMicros_prev = loopMicros_now;

  if (loopDelta > 0) {
    loopHz_now = 1000000.0 / loopDelta;
  }

  loopHz_avg = 0.9 * loopHz_avg + 0.1 * loopHz_now;

  imu.update();
  compass.update();
  baro.update();
  velocityEstimator.update();

  float vert_vel = velocityEstimator.getVelocity();

  float relativeAltBaro = baro.getRelativeAltitude();

  /////CALIBRATION MODE/////

  calibration_pressed = millis();

  if (joyData.btn3 == -1) {

    if (calibration_pressed - calibration_time >= 2000) {

      int16_t x, y, z;

      compass.getRawMagData(&x, &y, &z);

      if (x < xMin) xMin = x;
      if (x > xMax) xMax = x;
      if (y < yMin) yMin = y;
      if (y > yMax) yMax = y;
      if (z < zMin) zMin = z;
      if (z > zMax) zMax = z;

      xOffset = (xMax + xMin) / 2.0f;
      yOffset = (yMax + yMin) / 2.0f;
      zOffset = (zMax + zMin) / 2.0f;

      compass.setMagOffsets(xOffset, yOffset, zOffset);

      Serial.print("Calibrating... Offsets -> X: ");
      Serial.print(xOffset);
      Serial.print(" Y: ");
      Serial.print(yOffset);
      Serial.print(" Z: ");
      Serial.println(zOffset);
    }
  }

  else {

    xMin = 32767;
    xMax = -32768;
    yMin = 32767;
    yMax = -32768;
    zMin = 32767;
    zMax = -32768;

    calibration_time = calibration_pressed;
  }


  ///////////gps///////////
  // getPVT() is now strictly non-blocking. It instantly returns true if the ESP32-C3's serial
  // buffer has a new 10Hz payload waiting. If not, it safely passes in microseconds.
  if (myGNSS.getPVT() == true) {

    // Check if we have a valid fix before using the data
    byte fixType = myGNSS.getFixType();

    if (fixType == 0) {
      //Serial.println("Waiting for satellites...");
    } else {
      // --- POSITION ---
      // Coordinates are returned in 10^-7 degrees. Divide by 10,000,000 to get standard degrees.
      double latitude = myGNSS.getLatitude() / 10000000.0;
      double longitude = myGNSS.getLongitude() / 10000000.0;

      // Altitude is returned in millimeters. Divide by 1000 for meters.
      float altitude = myGNSS.getAltitude() / 1000.0;

      // Save the first valid position as home
      if (!gpsStarted) {
        gpsStartTime = millis();
        gpsStarted = true;
        Serial.println("GPS fix acquired. Stabilizing...");
      }

      if (!homeSet && (millis() - gpsStartTime >= 6000)) {
        homeLat = latitude;
        homeLon = longitude;
        homeAlt = altitude;
        homeSet = true;
        Serial.println("Home position set.");
      }

      // Only perform relative coordinate calculations and prints if home is established
      if (homeSet) {
        relativeAltitude = altitude - homeAlt;

        // Convert latitude/longitude to local North/East coordinates (meters)
        double dLat = radians(latitude - homeLat);
        double dLon = radians(longitude - homeLon);
        double avgLat = radians((latitude + homeLat) * 0.5);

        posNorth = dLat * EARTH_RADIUS;
        posEast = dLon * EARTH_RADIUS * cos(avgLat);

        // --- DOPPLER VELOCITY ---
        // The u-blox calculates highly precise velocity internally using carrier-phase Doppler shift.
        // Values are returned in mm/s. Divide by 1000 to get m/s.
        velEast = myGNSS.getNedEastVel() / 1000.0;    // East velocity (Linear)
        velNorth = myGNSS.getNedNorthVel() / 1000.0;  // North velocity (Linear)
        velDown = myGNSS.getNedDownVel() / 1000.0;    // Vertical velocity (Positive = downwards)

        // --- TOTAL SPEED ---
        // Total 3D speed magnitude calculated from NED vectors using 3D Pythagorean theorem
        float speed3D = sqrt((velEast * velEast) + (velNorth * velNorth) + (velDown * velDown));

        // --- PRINT DATA ---
        /*
        Serial.print("East: ");
        Serial.print(posEast, 2);
        Serial.print(" m | North: ");
        Serial.print(posNorth, 2);
        Serial.print(" m | Alt: "); 
        Serial.print(relativeAltitude, 2);
        Serial.print(" m || ");

        Serial.print("Vel East: "); 
        Serial.print(velEast, 3);
        Serial.print(" m/s | Vel North: "); 
        Serial.print(velNorth, 3);
        Serial.print(" m/s | Vel Down: "); 
        Serial.println(velDown, 3);
        */
      }
    }
  }

  FusedAlt = alphaAlt * relativeAltBaro + (1 - alphaAlt) * relativeAltitude;

  ///////////LOW PASS FILTER IMU////////////

  // raw values
  double gyroX = imu.getGyroX();
  double gyroY = imu.getGyroY();
  double gyroZ = imu.getGyroZ();

  double roll_raw = imu.getRoll();
  double pitch_raw = imu.getPitch();

  // apply low-pass filter
  gyroX_f += alpha * (gyroX - gyroX_f);
  gyroY_f += alpha * (gyroY - gyroY_f);
  gyroZ_f += alpha * (gyroZ - gyroZ_f);

  roll_f += alpha * (roll_raw - roll_f);
  pitch_f += alpha * (pitch_raw - pitch_f);

  double rawHeading = compass.getHeading();
  double headingDiff = rawHeading - MagHeading;

  while (headingDiff <= -180.0) headingDiff += 360.0;
  while (headingDiff > 180.0) headingDiff -= 360.0;

  MagHeading += alpha_mag * headingDiff;

  while (MagHeading < 0.0) MagHeading += 360.0;
  while (MagHeading >= 360.0) MagHeading -= 360.0;

  ///////////////////////////NRF24/////////////////////////
  if (radio.available()) {
    radio.read(&joyData, sizeof(joyData));
    //fail safe
    lastRadioTime = millis();
    failsafe = false;

    failsafe_step = 0;
    failsafe_save = 0;
    alt_save = 0;

    // Print received data (debug)
    /*
    Serial.print("J1X: "); Serial.print(joyData.joy1x);
    Serial.print(" | J1Y: "); Serial.print(joyData.joy1y);
    Serial.print(" | J2X: "); Serial.print(joyData.joy2x);
    Serial.print(" | J2Y: "); Serial.print(joyData.joy2y);

    Serial.print(" | B1: "); Serial.print(joyData.btn1);
    Serial.print(" | B2: "); Serial.print(joyData.btn2);
    Serial.print(" | B3: "); Serial.print(joyData.btn3);
    Serial.print(" | B4: "); Serial.print(joyData.btn4);
    Serial.print(" | B5: "); Serial.println(joyData.btn5);
    */
  }

  if (millis() - lastRadioTime > 700) {
    failsafe = true;
  }

  /////////////////////////////////////////////////////////

  if (joyData.btn2 == -1) {

    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
  }

  /////////////////////MAPING////////////////////

  throttle = map(joyData.joy1y, 0, 1023, 1200, 1700);
  yaw_setpoint2 = map(joyData.joy2y, 0, 1023, 100, -100);
  roll_setpoint = map(joyData.joy1x, 0, 1023, 15, -15);
  pitch_setpoint = map(joyData.joy2x, 0, 1023, -15, 15);



  if (joyData.btn1 == 1) {

    N_setpoint = posNorth;
    E_setpoint = posEast;
    D_setpoint = FusedAlt;
    //yaw_setpoint1 = MagHeading;
  }

  hover_pressed = millis();

  if (joyData.btn4 == 1) {

    if (hover_pressed - hover_time >= 1000) {

      hover_throttle = throttle;
    }
  }

  else {

    hover_time = hover_pressed;
  }


  ///////////////////FailSafe////////////////

  if (failsafe && failsafe_step == 0) {

    failsafe_step = 1;
  }

  if (failsafe_step == 1) {

    if (failsafe_save == 0) {
      N_setpoint = posNorth;
      E_setpoint = posEast;
      D_setpoint = FusedAlt;

      if (FusedAlt > 10) {
        rth_target_alt = FusedAlt;
      }

      else {
        rth_target_alt = 10;
      }

      failsafe_save = 1;
    }

    if (D_setpoint < rth_target_alt) {
      D_setpoint += 0.7 * dt;
    }
  }

  if (FusedAlt >= (rth_target_alt - 2) && failsafe_step == 1) {

    D_setpoint = rth_target_alt;
    failsafe_step = 2;
  }

  if (failsafe_step == 2) {

    N_setpoint = 0;
    E_setpoint = 0;
    D_setpoint = rth_target_alt;
  }

  if ((posNorth - 0) * (posNorth - 0) + (posEast - 0) * (posEast - 0) <= 8 && failsafe_step == 2) {  // circle radius 2

    failsafe_step = 3;
  }

  if (failsafe_step == 3) {

    N_setpoint = 0;
    E_setpoint = 0;
    D_setpoint -= 0.5 * dt;
  }

  if (FusedAlt <= 4 && failsafe_step == 3) {
    failsafe_step = 4;
  }

  if (failsafe_step == 4) {
    N_setpoint = 0;
    E_setpoint = 0;
    D_setpoint = 3;
  }

  /*  //in case gps is not active//
  if (current_fix == 0 && failsafe) {

    roll_setpoint = 0;
    pitch_setpoint = 0;
    yaw_setpoint2 = 0;
    if (alt_save == 0) {

      D_setpoint = FusedAlt;
      alt_save = 1;
    }

    if (FusedAlt >= 1)
      D_setpoint -= 0.5 * dt;
  }

  */


  ///////////////////// TRIPLE CASCADED PID + GPS CASCADED PID ///////////////////////

  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;  // convert us to seconds
  lastTime = currentTime;
  if (dt <= 0) return;


  // ----- GPS NORTH -----
  N_input = posNorth;   // measured angle
  N_input2 = velNorth;  // measured angular velocity / gyro

  // ----- Outer PID -----
  N_error = N_setpoint - N_input;
  N_integral += N_error * dt;
  N_integral = constrain(N_integral, -5, 5);
  N_derivative = (N_error - N_previousError) / dt;

  N_output1 = (N_Kp * N_error) + (N_Ki * N_integral) + (N_Kd * N_derivative);  // desired speed

  N_previousError = N_error;

  // ----- Inner PID (Speed -> Motor Output) -----
  N_error2 = N_output1 - N_input2;  // input2 = measured speed, output1 = setpoint for speed
  N_integral2 += N_error2 * dt;
  N_integral2 = constrain(N_integral2, -5.0, 5.0);
  N_derivative2 = (N_error2 - N_previousError2) / dt;

  N_output2 = (N_Kp2 * N_error2) + (N_Ki2 * N_integral2) + (N_Kd2 * N_derivative2);  // motor command

  N_previousError2 = N_error2;
  /////////////////////////////////

  // ----- GPS EAST -----
  E_input = posEast;   // measured angle
  E_input2 = velEast;  // measured angular velocity / gyro

  // ----- Outer PID -----
  E_error = E_setpoint - E_input;
  E_integral += E_error * dt;
  E_integral = constrain(E_integral, -5, 5);
  E_derivative = (E_error - E_previousError) / dt;

  E_output1 = (E_Kp * E_error) + (E_Ki * E_integral) + (E_Kd * E_derivative);  // desired speed

  E_previousError = E_error;

  // ----- Inner PID (Speed -> Motor Output) -----
  E_error2 = E_output1 - E_input2;  // input2 = measured speed, output1 = setpoint for speed
  E_integral2 += E_error2 * dt;
  E_integral2 = constrain(E_integral2, -5.0, 5.0);
  E_derivative2 = (E_error2 - E_previousError2) / dt;

  E_output2 = (E_Kp2 * E_error2) + (E_Ki2 * E_integral2) + (E_Kd2 * E_derivative2);  // motor command

  E_previousError2 = E_error2;

  /////////////////////

  // ----- GPS DOWN -----
  D_input = FusedAlt;   // measured angle
  D_input2 = vert_vel;  // measured angular velocity / gyro

  // ----- Outer PID -----
  D_error = D_setpoint - D_input;
  D_integral += D_error * dt;
  D_integral = constrain(D_integral, -30, 30);
  D_derivative = (D_error - D_previousError) / dt;

  D_output1 = (D_Kp * D_error) + (D_Ki * D_integral) + (D_Kd * D_derivative);  // desired speed

  D_previousError = D_error;


  // ----- Inner PID (Speed -> Motor Output) -----
  D_error2 = D_output1 - D_input2;  // input2 = measured speed, output1 = setpoint for speed
  D_integral2 += D_error2 * dt;
  D_derivative2 = (D_error2 - D_previousError2) / dt;

  D_output2 = (D_Kp2 * D_error2) + (D_Ki2 * D_integral2) + (D_Kd2 * D_derivative2);  // motor command

  D_previousError2 = D_error2;

  //Serial.println(D_output2);


  //////////////Altitude hold activation///////////////////
  /*
  if (joyData.btn5 != -1 || failsafe) {
    throttle = hover_throttle + D_output2;
    throttle = constrain(throttle, 1200, 1700);
  }
  */

  ////tilt compensated throttle////
  if (joyData.btn5 != -1 || failsafe) {
    // 1. Convert your cleanly filtered angles to radians
    // (Using your existing roll_f and pitch_f variables)
    float roll_rad = radians(roll_f);
    float pitch_rad = radians(pitch_f);

    // 2. Calculate the total geometric tilt factor
    float cos_tilt = cos(roll_rad) * cos(pitch_rad);

    // 3. SAFETY LIMIT: Cap compensation at ~45 degrees (cos(45) ≈ 0.707)
    // This prevents massive throttle spikes if the drone banks aggressively or tumbles
    if (cos_tilt < 0.707f) {
      cos_tilt = 0.707f;
    }

    // 4. Apply Feed-Forward Compensation
    // We divide the requested vertical thrust by the tilt factor to boost the motors
    float raw_vertical_throttle = hover_throttle + D_output2;
    throttle = raw_vertical_throttle / cos_tilt;

    // 5. Constrain to your defined ESC/throttle limits
    throttle = constrain(throttle, 1200, 1700);
  }
  ///////////////////////////

  N_output2 = constrain(N_output2, -15, 15);
  E_output2 = constrain(E_output2, -15, 15);

  //Serial.print(N_output2);
  //Serial.print(":__:");
  //Serial.println(-E_output2);

  /////////////yaw gps conversion////////////

  h = radians(MagHeading);

  forward = N_output2 * cos(h) + E_output2 * sin(h);
  right = -N_output2 * sin(h) + E_output2 * cos(h);

  //Serial.print(forward);
  //Serial.print(":__:");
  //Serial.println(-right);


  // ----- ROLL -----
  roll_input = pitch_f;   // measured angle
  roll_input2 = gyroY_f;  // measured angular velocity / gyro

  // ----- Outer PID -----
  if (joyData.btn5 != 1 && !failsafe) {

    roll_error = roll_setpoint - roll_input;
  } else {

    roll_error = -E_output2 - roll_input;
  }

  roll_integral += roll_error * dt;
  roll_integral = constrain(roll_integral, -50, 50);
  roll_derivative = (roll_error - roll_previousError) / dt;

  roll_output1 = (roll_Kp * roll_error) + (roll_Ki * roll_integral) + (roll_Kd * roll_derivative);  // desired speed

  roll_previousError = roll_error;

  // ----- Inner PID (Speed -> Motor Output) -----
  roll_error2 = roll_output1 - roll_input2;  // input2 = measured speed, output1 = setpoint for speed
  roll_integral2 += roll_error2 * dt;
  roll_derivative2 = (roll_error2 - roll_previousError2) / dt;

  roll_output2 = (roll_Kp2 * roll_error2) + (roll_Ki2 * roll_integral2) + (roll_Kd2 * roll_derivative2);  // motor command

  roll_previousError2 = roll_error2;


  // ----- PITCH -----
  pitch_input = roll_f;    // measured angle
  pitch_input2 = gyroX_f;  // measured angular velocity / gyro

  // ----- Outer PID -----
  if (joyData.btn5 != 1 && !failsafe) {

    pitch_error = pitch_setpoint - pitch_input;
  } else {

    pitch_error = N_output2 - pitch_input;
  }

  pitch_integral += pitch_error * dt;
  pitch_integral = constrain(pitch_integral, -50, 50);
  pitch_derivative = (pitch_error - pitch_previousError) / dt;

  pitch_output1 = (pitch_Kp * pitch_error) + (pitch_Ki * pitch_integral) + (pitch_Kd * pitch_derivative);  // desired speed

  pitch_previousError = pitch_error;

  // ----- Inner PID (Speed -> Motor Output) -----
  pitch_error2 = pitch_output1 - pitch_input2;  // input2 = measured speed, output1 = setpoint for speed
  pitch_integral2 += pitch_error2 * dt;
  pitch_derivative2 = (pitch_error2 - pitch_previousError2) / dt;

  pitch_output2 = (pitch_Kp2 * pitch_error2) + (pitch_Ki2 * pitch_integral2) + (pitch_Kd2 * pitch_derivative2);  // motor command

  pitch_previousError2 = pitch_error2;


  // ----- YAW -----
  yaw_input = MagHeading;
  yaw_input2 = gyroZ_f;  // measured angular velocity / gyro

  // ----- Outer PID -----
  yaw_error = yaw_setpoint1 - yaw_input;

  if (yaw_error > 180) {
    yaw_error -= 360;
  }
  if (yaw_error < -180) {
    yaw_error += 360;
  }

  yaw_integral += yaw_error * dt;
  yaw_integral = constrain(yaw_integral, -50, 50);
  yaw_derivative = (yaw_error - yaw_previousError) / dt;

  yaw_output1 = (yaw_Kp * yaw_error) + (yaw_Ki * yaw_integral) + (yaw_Kd * yaw_derivative);  // desired speed

  yaw_previousError = yaw_error;

  // ----- Inner PID (Speed -> Motor Output) -----
  if (joyData.btn5 != 1 && !failsafe) {

    yaw_error2 = yaw_setpoint2 - yaw_input2;
  } else {

    yaw_error2 = -yaw_output1 - yaw_input2;
  }

  yaw_integral2 += yaw_error2 * dt;
  yaw_derivative2 = (yaw_error2 - yaw_previousError2) / dt;

  yaw_output2 = (yaw_Kp2 * yaw_error2) + (yaw_Ki2 * yaw_integral2) + (yaw_Kd2 * yaw_derivative2);  // motor command

  yaw_previousError2 = yaw_error2;

  roll_output2 = constrain(roll_output2, -350, 350);
  pitch_output2 = constrain(pitch_output2, -350, 350);
  yaw_output2 = constrain(yaw_output2, -200, 200);

  ////////////////MOTOR MIXING///////////////
  double roll = -roll_output2;
  double pitch = -pitch_output2;
  double yaw = -yaw_output2;

  // Mixing (X configuration)
  motorFL = throttle + roll + pitch - yaw;  // CW
  motorFR = throttle - roll + pitch + yaw;  // CCW
  motorRL = throttle + roll - pitch + yaw;  // CCW
  motorRR = throttle - roll - pitch - yaw;  // CW

  // constrain to ESC range
  motorFL = constrain(motorFL, 1200, 2000);
  motorFR = constrain(motorFR, 1200, 2000);
  motorRL = constrain(motorRL, 1200, 2000);
  motorRR = constrain(motorRR, 1200, 2000);

  if (joyData.btn2 == 1) {

    esc1.writeMicroseconds(motorFL);
    esc2.writeMicroseconds(motorFR);
    esc3.writeMicroseconds(motorRL);
    esc4.writeMicroseconds(motorRR);
  }

  /*
   Serial.print(roll_f);
   Serial.print(" ");

   Serial.print(pitch_f);
   Serial.print(" ");

   Serial.println(gyroZ_f);
   Serial.println(compass.getHeading());
   */

  /*
     if (millis() - printTimer > 100) {
    printTimer = millis();
    Serial.print("Loop Hz: ");
    Serial.println(loopHz_avg);
  }
  */

  //Serial.println(compass.getHeading());
  //Serial.println(FusedAlt);

  //Serial.println(vert_vel);

  //Serial.print(N_setpoint);
  //Serial.print("||");
  //Serial.println(E_setpoint);

  //Serial.println(hover_throttle);

  //Serial.println(D_output1);


  //////////////LIMITER////////////////

  unsigned long elapsed = micros() - loopStartMicros;

  if (elapsed < LOOP_PERIOD_US) {
    delayMicroseconds(LOOP_PERIOD_US - elapsed);
  }
}
