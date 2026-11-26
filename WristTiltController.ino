
/*
  SPIDER PONG - WRIST TILT CONTROLLER

  Board: Adafruit ESP32 Feather V2
  Sensor: LSM6DS3TR-C

  Hold the palm button to move the paddle.
  Release the button to hold its position.

  MOVEMENT:
  - Original 30-degree range
  - Original default paddle speed
  - No extra software smoothing
  - Relative movement with position locking
*/

#include <Wire.h>
#include <DFPongController.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <MadgwickAHRS.h>

DFPongController controller;
Adafruit_LSM6DS3TRC imu;
Madgwick filter;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;


// -------------------- SETTINGS --------------------

const bool USE_ROLL_AXIS = false;

const float ANGLE_RANGE = 30.0;

const int CONTROLLER_NUMBER = 3;
const int BUTTON_PIN = 32;

const unsigned long BUTTON_DEBOUNCE_TIME = 30;
const unsigned long SENSOR_INTERVAL = 10;
const unsigned long PRINT_INTERVAL = 100;


// -------------------- TIMING VARIABLES --------------------

unsigned long lastSensorUpdate = 0;
unsigned long lastPrint = 0;
unsigned long lastDebounceTime = 0;


// -------------------- ANGLE VARIABLES --------------------

float rollAngle = 0;
float pitchAngle = 0;
float neutralAngle = 0;

float controlAngle = 0;

float paddlePositionAngle = 0;

float angleWhenButtonPressed = 0;
float positionWhenButtonPressed = 0;


// -------------------- BUTTON VARIABLES --------------------

int lastButtonReading = HIGH;
int stableButtonState = HIGH;


// -------------------- UPDATE IMU --------------------

void updateIMU()
{
  imu.getEvent(&accel, &gyro, &temp);

  filter.updateIMU(
    degrees(gyro.gyro.x),
    degrees(gyro.gyro.y),
    degrees(gyro.gyro.z),
    accel.acceleration.x,
    accel.acceleration.y,
    accel.acceleration.z
  );

  rollAngle = filter.getRoll();
  pitchAngle = filter.getPitch();
}


// -------------------- CONTROL ANGLE --------------------

float getControlAngle()
{
  if (USE_ROLL_AXIS)
  {
    return rollAngle;
  }

  return pitchAngle;
}


// -------------------- CALIBRATION --------------------

void calibrateNeutralPosition()
{
  const int CALIBRATION_READINGS = 150;
  float angleTotal = 0;

  Serial.println("Keep your wrist still in its neutral position.");

  for (int i = 0; i < CALIBRATION_READINGS; i++)
  {
    updateIMU();
    angleTotal += getControlAngle();
    delay(10);
  }

  neutralAngle = angleTotal / CALIBRATION_READINGS;

  Serial.print("Neutral angle: ");
  Serial.println(neutralAngle);

  Serial.println("Calibration complete.");
}


// -------------------- BUTTON CONTROL --------------------

void updateButton()
{
  int currentButtonReading = digitalRead(BUTTON_PIN);

  if (currentButtonReading != lastButtonReading)
  {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime >= BUTTON_DEBOUNCE_TIME)
  {
    if (currentButtonReading != stableButtonState)
    {
      stableButtonState = currentButtonReading;

      if (stableButtonState == LOW)
      {
        // Preserve the paddle's last position.
        positionWhenButtonPressed = paddlePositionAngle;

        // Use the current wrist angle as the new reference.
        angleWhenButtonPressed = controlAngle;

        Serial.println("Button pressed - movement enabled.");
      }
      else
      {
        Serial.println("Button released - paddle position held.");
      }
    }
  }

  lastButtonReading = currentButtonReading;
}


// -------------------- PADDLE MOVEMENT --------------------

void updatePaddlePosition()
{
  if (stableButtonState == LOW)
  {
    // Calculate wrist movement relative to the new reference.
    float wristMovement =
      controlAngle - angleWhenButtonPressed;

    // Add relative wrist movement to the last paddle position.
    paddlePositionAngle =
      positionWhenButtonPressed + wristMovement;

    paddlePositionAngle = constrain(
      paddlePositionAngle,
      -ANGLE_RANGE,
      ANGLE_RANGE
    );

    controller.sendPosition(
      (long)paddlePositionAngle,
      (long)-ANGLE_RANGE,
      (long)ANGLE_RANGE
    );
  }
}


// -------------------- SETUP --------------------

void setup()
{
  Serial.begin(9600);
  delay(500);

  Wire.begin();

  Serial.println("Spider Pong Controller");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!imu.begin_I2C())
  {
    Serial.println("ERROR: IMU not found. Check the STEMMA QT cable.");

    while (true)
    {
      delay(100);
    }
  }

  filter.begin(100);

  controller.setControllerNumber(CONTROLLER_NUMBER);
  controller.setStatusLED(LED_BUILTIN);
  controller.setControllerName("Spider Pong");

  // Spider-Man customizations.
  controller.setPlayerColor("#E23636");
  controller.setHitSound("laser");
  controller.setScoreSound("gong");
  controller.setWinSound("flourish");

  // Use the game's normal default movement speed.

  controller.begin();

  delay(1500);

  calibrateNeutralPosition();

  Serial.println("Spider Pong is ready.");
  Serial.println("Hold the button to move the paddle.");
}


// -------------------- MAIN LOOP --------------------

void loop()
{
  controller.update();

  if (millis() - lastSensorUpdate >= SENSOR_INTERVAL)
  {
    lastSensorUpdate = millis();

    updateIMU();

    controlAngle = getControlAngle() - neutralAngle;
  }

  updateButton();

  updatePaddlePosition();

  if (millis() - lastPrint >= PRINT_INTERVAL)
  {
    lastPrint = millis();

    Serial.print("Roll: ");
    Serial.print(rollAngle);

    Serial.print(" | Pitch: ");
    Serial.print(pitchAngle);

    Serial.print(" | Control: ");
    Serial.print(controlAngle);

    Serial.print(" | Paddle: ");
    Serial.println(paddlePositionAngle);
  }
}


/*
  EXTERNAL REFERENCES

  DF Pong:
  https://github.com/DigitalFuturesOCADU/df-pong

  DF Pong Controller:
  https://github.com/DigitalFuturesOCADU/df-pong-controller

  Adafruit LSM6DS:
  https://github.com/adafruit/Adafruit_LSM6DS

  Madgwick AHRS:
  https://github.com/arduino-libraries/MadgwickAHRS
*/
