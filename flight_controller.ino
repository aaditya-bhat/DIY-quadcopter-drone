#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>

// --- PID Tuning ---
// Kp seems okay at 1.5, might need to bump Kd if it gets wobbly outdoors
float Kp = 1.5;  
float Ki = 0.04; 
float Kd = 0.05; 

// hardware setup
Servo m1, m2, m3, m4;
VL53L0X laser;

#define LED_PIN 4
#define NUM_LEDS 12   
Adafruit_NeoPixel ledRing(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// IMU variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ, gyroXrate, gyroYrate;
float roll, pitch;
float gyroXoffset = 0, gyroYoffset = 0; 
float accelRollOffset = 0, accelPitchOffset = 0; 
bool firstImuRun = true; // stops the drone from drifting immediately on startup

unsigned long prevTime;
unsigned long lastLaserTime = 0; 

int groundLevel_mm = 0;
int currentHeight_mm = 0;
bool laserOK = false;

// flight states
enum FlightState { DISARMED, FLYING, AUTO_LANDING, FAULT };
FlightState currentState = DISARMED;

const int TEST_MAX_THROTTLE = 1800; // DO NOT SET ABOVE 1800 YET

int baseThrottle = 1000;
float targetRoll = 0, targetPitch = 0;
float errorRoll, errorPitch, prevErrorRoll = 0, prevErrorPitch = 0;
float integralRoll = 0, integralPitch = 0;
float pidRoll, pidPitch;

unsigned long lastPacketTime = 0;
unsigned long lastLandRampTime = 0;
unsigned long lastLedUpdate = 0;
bool ledBlinkOn = false;
bool hasLifted = false; 
bool readyToArm = true; 
const int ARM_LOW_THRESH = 100;     
const int ARM_RELEASE_THRESH = 500; 
char packetBuffer[32];
int bufIndex = 0;
bool recording = false;
float lastDt = 0.005;

void setup() {
  // Gotta claim motor pins IMMEDIATELY and send 1000us 
  // so the ESCs don't freak out and spin up during power-on.
  m1.attach(3); m2.attach(9); m3.attach(10); m4.attach(11);
  setMotors(1000, 1000, 1000, 1000);

  // switched to hardware serial, pins D0 & D1
  Serial.begin(38400);
  Wire.begin();

  ledRing.begin();
  ledRing.setBrightness(60);
  setAllLeds(0, 0, 0);
  ledRing.show();

  // wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00); 
  Wire.endTransmission(true);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x08); 
  Wire.endTransmission(true);

  // turn on digital low pass filter (20Hz) - smooths out motor vibrations
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); 
  Wire.write(0x05); 
  Wire.endTransmission(true);

  // IMU Calib - DO NOT BUMP THE DRONE HERE
  setAllLeds(60, 60, 0); // yellow = calibrating
  ledRing.show();
  
  long gyroXsum = 0, gyroYsum = 0;
  float accelRollSum = 0, accelPitchSum = 0;
  
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    float aX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float aY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float aZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    
    // finally fixed the damn pitch inversion... 
    accelRollSum += (atan(aY / sqrt(pow(aX, 2) + pow(aZ, 2))) * 180 / PI);
    accelPitchSum += (atan(aX / sqrt(pow(aY, 2) + pow(aZ, 2))) * 180 / PI); 

    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 4, true);
    gyroXsum += (Wire.read() << 8 | Wire.read());
    gyroYsum += (Wire.read() << 8 | Wire.read());
    delay(3);
  }
  
  gyroXoffset = (gyroXsum / 500.0) / 65.5;
  gyroYoffset = (gyroYsum / 500.0) / 65.5;
  accelRollOffset = accelRollSum / 500.0;
  accelPitchOffset = accelPitchSum / 500.0;

  setAllLeds(0, 0, 0); 
  ledRing.show();

  // fire up the laser sensor
  laser.setTimeout(500);
  laserOK = laser.init();
  if (laserOK) {
    laser.startContinuous();
  } else {
    // red flash of death if laser fails
    currentState = FAULT;
    while (true) {
      setAllLeds(255, 0, 0); ledRing.show(); delay(200);
      setAllLeds(0, 0, 0); ledRing.show(); delay(200);
    }
  }

  // grab a baseline for ground level
  // (motors are already holding at 1000us from earlier)
  long heightSum = 0;
  int validReads = 0;
  for (int i = 0; i < 50; i++) {
    int r = laser.readRangeContinuousMillimeters();
    if (!laser.timeoutOccurred() && r > 0 && r < 8000) {
      heightSum += r;
      validReads++;
    }
    delay(60);
  }

  if (validReads < 25) {
    currentState = FAULT;
    while (true) {
      setAllLeds(255, 0, 0); ledRing.show(); delay(100);
      setAllLeds(0, 0, 0); ledRing.show(); delay(100);
    }
  }
  groundLevel_mm = heightSum / validReads;

  prevTime = micros();
  currentState = DISARMED;
}

void loop() {
  readIMU();

  // don't poll laser too fast, 50ms seems to be the sweet spot
  if (millis() - lastLaserTime > 50) {
    int laserReading = laser.readRangeContinuousMillimeters();
    if (!laser.timeoutOccurred() && laserReading > 0 && laserReading < 8000) {
      currentHeight_mm = laserReading;
    }
    lastLaserTime = millis();
  }

  readRadio();
  updateStatusLed();

  if (currentState == FLYING) {
    if (baseThrottle > 1200) {
      hasLifted = true; // we are officially airborne
    }

    // shut down if throttle drops too low after takeoff
    if (hasLifted && baseThrottle <= 1010) {
      currentState = DISARMED;
      readyToArm = false; 
    }

    // failsafe: trigger auto land if we lose radio for 1 sec
    if (millis() - lastPacketTime > 1000) {
      if (baseThrottle < 1200) {
        currentState = DISARMED;
        readyToArm = false; 
      } else {
        currentState = AUTO_LANDING;
      }
    }
  }
  else if (currentState == AUTO_LANDING) {
    targetRoll = 0;
    targetPitch = 0;

    // slowly back off throttle
    if (millis() - lastLandRampTime > 40) { 
      baseThrottle--;
      lastLandRampTime = millis();
    }

    // cut motors if we hit the ground
    if (currentHeight_mm < (groundLevel_mm + 30) && currentHeight_mm > 5 && baseThrottle < 1400) {
      currentState = DISARMED;
      readyToArm = false; 
    }

    // absolute floor for throttle during landing
    if (baseThrottle <= 1050) {
      currentState = DISARMED;
      readyToArm = false; 
    }
  }

  if (currentState == DISARMED) {
    baseThrottle = 1000;
    setMotors(1000, 1000, 1000, 1000);
    integralRoll = 0;
    integralPitch = 0; 
    hasLifted = false;
  }
  else if (currentState == FLYING || currentState == AUTO_LANDING) {
    calculatePID();

    // mixing logic - mapping pitch/roll to the 4 corners
    // took a while to get the signs right here, don't change this
    int speedM1 = baseThrottle + pidPitch + pidRoll; // front left
    int speedM2 = baseThrottle + pidPitch - pidRoll; // front right
    int speedM3 = baseThrottle - pidPitch + pidRoll; // back left
    int speedM4 = baseThrottle - pidPitch - pidRoll; // back right

    setMotors(
      constrain(speedM1, 1000, 2000),
      constrain(speedM2, 1000, 2000),
      constrain(speedM3, 1000, 2000),
      constrain(speedM4, 1000, 2000)
    );
  }
}

// --- Radio Stuff ---
void readRadio() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '<') { recording = true; bufIndex = 0; }
    else if (c == '>') {
      recording = false;
      packetBuffer[bufIndex] = '\0';

      int mode, joyX, joyY;
      if (sscanf(packetBuffer, "%d,%d,%d", &mode, &joyX, &joyY) == 3) {
        lastPacketTime = millis();

        // arming logic
        if ((currentState == DISARMED || currentState == AUTO_LANDING) && mode == 1 && joyY > ARM_RELEASE_THRESH) {
          readyToArm = true;
        }

        if ((currentState == AUTO_LANDING || currentState == DISARMED) && mode == 1 && joyY < ARM_LOW_THRESH && readyToArm) {
          currentState = FLYING;
          baseThrottle = 1000;
          hasLifted = false;
          readyToArm = false; 
        }

        static unsigned long lastThrottleRampTime = 0;

        if (currentState == FLYING) {
          if (mode == 1) {
            if (millis() - lastThrottleRampTime > 20) { 
              if (joyY > 2150) {
                // smooth throttle up
                int speedUp = map(constrain(joyY, 2150, 3800), 2150, 3800, 1, 12); 
                baseThrottle = constrain(baseThrottle + speedUp, 1000, TEST_MAX_THROTTLE);
              }
              else if (joyY < 1900) {
                // smooth throttle down
                int speedDown = map(constrain(joyY, 200, 1900), 200, 1900, 12, 1);
                baseThrottle = constrain(baseThrottle - speedDown, 1000, TEST_MAX_THROTTLE);
              }
              lastThrottleRampTime = millis();
            }
          } else {
            targetRoll = map(joyX, 0, 4095, -20, 20);
            targetPitch = map(joyY, 0, 4095, -20, 20);
          }
        }
      }
    } else if (recording && bufIndex < 31) {
      packetBuffer[bufIndex++] = c;
    }
  }
}

// --- IMU / PID calculations ---
void readIMU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  Wire.read(); Wire.read(); // toss temp data, don't need it

  gyroXrate = ((Wire.read() << 8 | Wire.read()) / 65.5) - gyroXoffset;
  
  // hacked gyro pitch inversion here too
  gyroYrate = -1.0 * (((Wire.read() << 8 | Wire.read()) / 65.5) - gyroYoffset);
  
  gyroZ = (Wire.read() << 8 | Wire.read()) / 65.5;

  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;

  // protect against dt glitches jumping to weird values
  if (dt <= 0 || dt > 0.1) dt = 0.005;

  float accelAngleRoll = (atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI) - accelRollOffset;
  
  // inverted accel pitch
  float accelAnglePitch = (atan(accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI) - accelPitchOffset;

  // complementary filter
  if (firstImuRun) {
    roll = accelAngleRoll;
    pitch = accelAnglePitch;
    firstImuRun = false;
  } else {
    roll = 0.98 * (roll + gyroXrate * dt) + 0.02 * accelAngleRoll;
    pitch = 0.98 * (pitch + gyroYrate * dt) + 0.02 * accelAnglePitch;
  }

  lastDt = dt; 
}

void calculatePID() {
  float dt = lastDt;

  errorRoll = targetRoll - roll;
  errorPitch = targetPitch - pitch;

  // only build up integral when we are actually flying, otherwise it winds up on the ground
  if (baseThrottle > 1050) {
    integralRoll += errorRoll * dt;
    integralPitch += errorPitch * dt;
  } else {
    integralRoll = 0;
    integralPitch = 0;
  }

  // hard cap on integral windup
  integralRoll = constrain(integralRoll, -50, 50);
  integralPitch = constrain(integralPitch, -50, 50);

  float derivativeRoll = (errorRoll - prevErrorRoll) / dt;
  float derivativePitch = (errorPitch - prevErrorPitch) / dt;

  pidRoll = (Kp * errorRoll) + (Ki * integralRoll) + (Kd * derivativeRoll);
  pidPitch = (Kp * errorPitch) + (Ki * integralPitch) + (Kd * derivativePitch);

  prevErrorRoll = errorRoll;
  prevErrorPitch = errorPitch;
}

void setMotors(int v1, int v2, int v3, int v4) {
  m1.writeMicroseconds(v1);
  m2.writeMicroseconds(v2);
  m3.writeMicroseconds(v3);
  m4.writeMicroseconds(v4);
}

// --- LEDs ---
void setAllLeds(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    ledRing.setPixelColor(i, ledRing.Color(r, g, b));
  }
}

void updateStatusLed() {
  // simple blink timer
  if (millis() - lastLedUpdate > 300) {
    lastLedUpdate = millis();
    ledBlinkOn = !ledBlinkOn;
  }

  switch (currentState) {
    case DISARMED: setAllLeds(0, 0, 40); break; // faint blue
    case FLYING: setAllLeds(0, 60, 0); break; // green
    case AUTO_LANDING: (ledBlinkOn) ? setAllLeds(60, 60, 0) : setAllLeds(0, 0, 0); break; // flashing yellow
    case FAULT: setAllLeds(80, 0, 0); break; // red
  }
  ledRing.show();
}