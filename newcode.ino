#include <DHT.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <Servo.h>
#include <LiquidCrystal.h>

#define BUZZER_PIN A0  //Define buzzer pin
#define FAN_PIN 5      // Define Fan pin

const int servoPin = 13;  // Pin connected to the servo

// Define the data pins for the LCD
#define LCD_RS 8
#define LCD_EN 9
#define LCD_D4 10
#define LCD_D5 11
#define LCD_D6 12
#define LCD_D7 7

// Initialize the LCD display
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
unsigned long lcdUpdateTime = 0;
const unsigned long lcdUpdateInterval = 250;  // Update every 0.25 seconds




//DHT22 variables
#define DHTPIN 4      // Digital pin connected to the DHT11
#define DHTTYPE DHT11  // DHT11 sensor type

// Define the pins for the heating wire controlled by IRFZ44N
#define HEATER_PIN A1

// Define the stepper motor connections
#define motorPin1 0
#define motorPin2 1
#define motorPin3 2
#define motorPin4 3

const int stepsPerRevolution = 2048;                                                        // Motor steps per revolution (adjust this based on your motor)
const int stepsPerHour = 360;                                                               // Number of steps to rotate the motor every hour
AccelStepper stepper(AccelStepper::FULL4WIRE, motorPin1, motorPin3, motorPin2, motorPin4);  // Create a new instance of AccelStepper
unsigned long lastRotationTime = 0;                                                         // Variable to store the last time the rotation occurred

// Define the setpoints for temperature and humidity
const float TEMP_SETPOINT = 37.8;      // Adjust this to your desired temperature in Celsius
const float HUMIDITY_SETPOINT = 52.5;  // Adjust this to your desired humidity level in percentage

// Initialize the DHT sensor
DHT dht(DHTPIN, DHTTYPE);

Servo ventServo;  // Create a servo object

// Define variables for days, temperature setpoint, humidity setpoint, and current mode
unsigned long initialTimestamp = 0;  // Variable to store the initial timestamp when counting starts
int days = 21;
int daysCounted = 1;  // Initialize daysCounted to 1
float tempSetpoint = TEMP_SETPOINT;
float humiditySetpoint = HUMIDITY_SETPOINT;
String currentMode = "Day";

// Define button pins
#define modeButtonPin A2
#define addButtonPin A3
#define subButtonPin A4


// Define button states
bool lastModeButtonState = HIGH;
bool lastAddButtonState = HIGH;
bool lastSubButtonState = HIGH;

// Define button timers
unsigned long modeButtonPressTime = 0;
unsigned long addButtonPressTime = 0;
unsigned long subButtonPressTime = 0;

// Define debounce delay
const int debounceDelay = 50;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize DHT sensor
  dht.begin();

  EEPROM.begin();


  // Set the maximum speed and acceleration of the stepper
  stepper.setMaxSpeed(3000);      // Adjust the speed as needed
  stepper.setAcceleration(4000);  // Adjust the acceleration as needed
  stepper.setCurrentPosition(0);  // Set the initial position of the stepper motor

  // Initialize the LCD display
  lcd.begin(16, 2);


  // Set the pin mode for the heating wire
  pinMode(HEATER_PIN, OUTPUT);

  // Set the FAN_PIN as an output
  pinMode(FAN_PIN, OUTPUT);

  // Attach the servo to the specified pin
  ventServo.attach(servoPin);

  // Set the pin mode for the buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Set button pins as inputs
  pinMode(modeButtonPin, INPUT_PULLUP);
  pinMode(addButtonPin, INPUT_PULLUP);
  pinMode(subButtonPin, INPUT_PULLUP);
}

void rotateStepperFor3Revolutions() {
  // Get the current time
  unsigned long currentTime = millis();

  // Calculate the time elapsed since the last rotation
  unsigned long timeElapsed = currentTime - lastRotationTime;

  // Check if an hour has passed
  if (timeElapsed >= 3600000) {  // 3600000 milliseconds = 1 hour
    // Rotate the motor by the specified number of steps (360 degrees)
    stepper.move(stepsPerHour);
    stepper.runToPosition();

    // Update the last rotation time to the current time
    lastRotationTime = currentTime;
  }
}


// Function to open the vent
void openVent() {
  ventServo.write(0);  // Set the servo angle to open position (0 degrees)
}

// Function to close the vent
void closeVent() {
  ventServo.write(90);  // Set the servo angle to close position (90 degrees)
}

void tempbuzz() {
  // Generate the sound pattern for temperature below set point
  tone(BUZZER_PIN, 1000, 10000);  // Buzz at 1000 Hz for 10000ms
}

void humiditybuzz() {
  // Generate the sound pattern for humidity below set point
  tone(BUZZER_PIN, 2000, 10000);  // Buzz at 2000 Hz for 10000ms
}

// Function to update the LCD
void updateLCD() {

  // Check if it's time to update the LCD
  if ((millis() - lcdUpdateTime) >= lcdUpdateInterval) {


    lcdUpdateTime = millis();  // Reset the update time
    lcd.clear();

    // Display the data on the LCD
    lcd.setCursor(0, 0);
    lcd.print("Total Days:");
    lcd.print(days);
    lcd.setCursor(0, 1);
    lcd.print("TP:");
    lcd.print(tempSetpoint, 1);
    lcd.print(" HP:");
    lcd.print(humiditySetpoint, 1);
    delay(500);
    lcd.clear();
  }
}


void loop() {
  rotateStepperFor3Revolutions();

  // Save the current value of millis() to the EEPROM.
  uint32_t millisValue = millis();
  EEPROM.write(0, millisValue >> 24);
  EEPROM.write(1, millisValue >> 16);
  EEPROM.write(2, millisValue >> 8);
  EEPROM.write(3, millisValue);

  // Read the saved value of millis() from the EEPROM.
  millisValue = (EEPROM.read(0) << 24) | (EEPROM.read(1) << 16) | (EEPROM.read(2) << 8) | EEPROM.read(3);

  // If the saved value is not 0, then use it as the starting value for millis().
  if (millisValue != 0) {
    initialTimestamp = millisValue;
  }

  // Calculate the number of days counted since the initial timestamp
  unsigned long elapsedDays = (millisValue - initialTimestamp) / (24 * 60 * 60 * 1000);
  int daysCounted = elapsedDays + 1;  // Add 1 to start counting from day 1

  // Read temperature and humidity from the DHT sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.clear();
    lcd.print("Sensor Error!");
    return;
  }

  // Output temperature and humidity values (for debugging)
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  // Display temperature and humidity on the LCD
  lcd.clear();
  lcd.print("Days:");
  lcd.print(daysCounted);
  lcd.print("/");
  lcd.print(days);
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C ");
  lcd.print("H:");
  lcd.print(humidity, 1);
  lcd.print("%");



  // Update heating wire based on temperature setpoint
  if (temperature <= tempSetpoint) {  // OutputTemp will be positive when heating is needed
    digitalWrite(HEATER_PIN, HIGH);   // Turn ON the heating wire
  } else {
    digitalWrite(HEATER_PIN, LOW);  // Turn OFF the heating wire
  }

  if (humidity <= humiditySetpoint) {  // OutputHumidity will be positive when humidity is low
    int fanSpeed = 200;                // Adjust the fan speed value as needed (0 to 255)
    int fanSpeedLow = 50;
    analogWrite(FAN_PIN, fanSpeed);
    closeVent();
  } else {
    analogWrite(FAN_PIN, 50);
    openVent();
  }

  if (temperature >= 40) {
    tempbuzz();
  }
  if (humidity >= 75) {
    humiditybuzz();
  }

  // Reset the timer when the day count reaches the total number of days
  if (daysCounted >= days) {
    initialTimestamp = millis();
    tempbuzz();
    digitalWrite(HEATER_PIN, LOW);  // Turn OFF the heating wire
    digitalWrite(FAN_PIN, LOW);     // Turn OFF the Fan wire
    lcd.clear();
    lcd.print("days completed");
    return;
  }

  // Update the setpoints based on the day count
  if (days == 21 && daysCounted >= 18) {
    tempSetpoint = 37.5;
    humiditySetpoint = 57.5;
  }

  if (days == 21 && daysCounted >= 20) {
    tempSetpoint = 37.5;
    humiditySetpoint = 70;
  }

  bool currentModeButtonState = digitalRead(modeButtonPin);
  if (currentModeButtonState != lastModeButtonState) {
    modeButtonPressTime = millis();
  }
  Serial.println(currentModeButtonState);
  if ((millis() - modeButtonPressTime) >= debounceDelay) {
    if (currentModeButtonState == LOW) {

      // Change mode
      if (currentMode == "Day") {
        currentMode = "Temp";
      } else if (currentMode == "Temp") {
        currentMode = "Humid";
      } else if (currentMode == "Humid") {
        currentMode = "Day";
      }
      updateLCD();
      noTone(BUZZER_PIN);  // Stop the buzzer
    }
  }

  lastModeButtonState = currentModeButtonState;


  // Check for add button press
  bool currentAddButtonState = digitalRead(addButtonPin);
  if (currentAddButtonState != lastAddButtonState) {
    addButtonPressTime = millis();
  }

  Serial.println(currentAddButtonState);
  if ((millis() - addButtonPressTime) >= debounceDelay) {
    if (currentAddButtonState == LOW) {
      // Increment values or perform actions
      // For example: days++;
      if (currentMode == "Day") {
        days++;
      } else if (currentMode == "Temp") {
        tempSetpoint += 0.1;
      } else if (currentMode == "Humid") {
        humiditySetpoint++;
      }
      updateLCD();         // Update the LCD with the new value
      noTone(BUZZER_PIN);  // Stop the buzzer
    }
  }

  lastAddButtonState = currentAddButtonState;

  // Check for subtract button press
  bool currentSubButtonState = digitalRead(subButtonPin);
  if (currentSubButtonState != lastSubButtonState) {
    subButtonPressTime = millis();
  }

  if ((millis() - subButtonPressTime) >= debounceDelay) {
    if (currentSubButtonState == LOW) {
      // Decrement values or perform actions
      // For example: days--;
      if (currentMode == "Day") {
        days--;
      } else if (currentMode == "Temp") {
        tempSetpoint -= 0.1;
      } else if (currentMode == "Humid") {
        humiditySetpoint--;
      }
      updateLCD();         // Update the LCD with the new value
      noTone(BUZZER_PIN);  // Stop the buzzer
    }
  }

  lastSubButtonState = currentSubButtonState;
}
