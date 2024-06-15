// Include the necessary libraries for operation
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include "max6675.h"
#include <EEPROM.h>

// Constants definition for the OLED display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

// Create an OLED display instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Temperature sensor calibration parameters (y = mx + b)
float m = 1.0; // Slope
float q = 0.0; // Y-intercept

// Reference temperatures for control
float stepTemp = 5;
float minTemp = 20;
float maxTemp = 310;

// Initial PID setup
double Setpoint, Input, Output; // PID variables
double Kp=4.5, Ki=0.0045, Kd=45; // PID parameters (tailored for each heating system)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // PID instance

// Setup for averaging temperature readings
const int numReadings = 5; // Number of readings to average
float readings[numReadings]; // Array for storing readings
int readIndex = 0; // Current index in the array
float total = 0; // Total sum for average calculation
int actualReadings = 0; // Actual number of readings taken

// Temperature sensor MAX6675 connection pins
int thermoCLK = 7;
int thermoCS = 6;
int thermoDO = 5;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Relay control pin
const int relayPin = 11;

// Encoder pin configuration
const int pinCLK = 2; 
const int pinDT = 3; 
const int pinButton = 4; 

// Variables for encoder management
bool buttonPressed = false;
volatile int encoderPos = 0; 
volatile int lastEncoded = 0; 

// Timer for event management
unsigned long previousMillis = 0; 
const long interval = 1000;

// Variables for settings storage
unsigned long lastEncoderActivity = 0;
const long saveDelay = 3000; 
bool readyToSave = false; 

// Initial setup of the program
void setup() {
  Serial.begin(9600); // Begin serial communication
  pinMode(relayPin, OUTPUT); // Set the relay pin as output
  pinMode(pinCLK, INPUT_PULLUP); // Configure encoder pins
  pinMode(pinDT, INPUT_PULLUP);
  pinMode(pinButton, INPUT_PULLUP);
  
  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;); // Halt execution if the display does not initialize
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Configure the PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PWM range

  // Configure the encoder to trigger an interrupt on state change
  attachInterrupt(digitalPinToInterrupt(pinCLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinDT), updateEncoder, CHANGE);

  // Initialize the temperature readings array
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // Read the Setpoint from EEPROM and validate it
  EEPROM.get(0, Setpoint);
  if (Setpoint < minTemp || Setpoint > maxTemp) {
    Setpoint = minTemp;
  } else {
    encoderPos = (Setpoint - minTemp) * 4;
  }

  delay(500); // Delay for temperature sensor initialization

  // Check the initialization of the temperature sensor
  float initialReading = thermocouple.readCelsius();
  if(isnan(initialReading)) {
    Serial.println("ERROR: temperature reading failed.");
  } 
}

// Function to read temperature
float readTemperature() {
  float temp;
  for (int attempts = 0; attempts < 3; attempts++) {
    temp = m * thermocouple.readCelsius() + q;
    if (!isnan(temp)) {
      return temp;
    }
    delay(50);
  }
  return temp;
}

// Main loop function
void loop() {
  // Read the current temperature and update the average
  total = total - readings[readIndex];
  readings[readIndex] = readTemperature();
  delay(200);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  if (actualReadings < numReadings) {
    actualReadings++;
  }

  Input = total / actualReadings; // Update PID input with temperature average

  // Handle periodic data transmission on the serial port
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.println(Input);
  }

  // Perform PID calculation and update relay output
  myPID.SetSampleTime(1000);
  myPID.Compute();
  if (Output > 0) {
    analogWrite(relayPin, Output);
  } else {
    digitalWrite(relayPin, LOW);
  }

  // Update display data
  display.clearDisplay();
  displayData();

  // Read serial commands and update Setpoint if needed
  readSerialCommand();

  // Check if it is time to save the Setpoint value to EEPROM
  if (millis() - lastEncoderActivity >= saveDelay && readyToSave) {
    EEPROM.put(0, Setpoint);
    readyToSave = false; // Reset the flag after saving
  }
}

// Function to display data on the OLED display
void displayData() {
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("PV: "); // Display the current value (Process Value)
  display.print(Input,0);

  // Display the "°C" symbol next to the current value
  int x = display.getCursorX();
  int y = display.getCursorY();
  for (int i = 0; i < 2; i++) {
    display.drawCircle(x + 10, y + 3, 2 + i, WHITE); // Draw a circle for the "°" symbol
  }
  display.setCursor(x + 5, y); // Position the cursor for the "C" text
  display.print(" C");

  // Display the Setpoint (the desired temperature value)
  display.setCursor(0, 16);
  display.print("SV:");
  display.setTextSize(4.5);
  display.setCursor(0, 32);
  display.print(Setpoint,0);
  x = display.getCursorX();
  y = display.getCursorY();
  for (int i = 0; i < 3; i++) {
    display.drawCircle(x + 16, y + 6, 4 + i, WHITE); // Draw a circle for the "°" symbol
  }
  display.setCursor(x + 4, y); // Position the cursor for the "C" text
  display.print(" C");

  display.display(); // Update the display with the displayed information
}

// Function called by the interrupt to update the encoder position
void updateEncoder() {
  int MSB = digitalRead(pinCLK); // Most significant bit
  int LSB = digitalRead(pinDT); // Least significant bit
  int encoded = (MSB << 1) | LSB; // Combine the two values into a single number
  int sum = (lastEncoded << 2) | encoded; // Add the current value to the previous one
  int tempEncoderPos = encoderPos; // Temporary variable for encoder position

  double tempIncrement = digitalRead(pinButton) == HIGH ? 1 : 10; // Increment based on the button state

  // Calculate the new encoder position
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) tempEncoderPos += tempIncrement;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) tempEncoderPos -= tempIncrement;

  // Calculate the potential new Setpoint
  double newSetpoint = minTemp + (tempEncoderPos / 4);
  if (newSetpoint >= minTemp && newSetpoint <= maxTemp) {
    encoderPos = tempEncoderPos;
    Setpoint = newSetpoint;
  }

  lastEncoded = encoded; // Store the value for next time
  lastEncoderActivity = millis(); // Update the time of the last encoder activity
  readyToSave = true; // Set the flag to indicate that we are ready to save
}  

// Function to read and handle commands from the serial port
void readSerialCommand() {
  static unsigned long targetTime = millis(); // Waiting time for the next command
  static bool isWaiting = false; // Indicates whether the system is waiting before reading a new command

  if (!isWaiting || millis() >= targetTime) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      int commaIndex = command.indexOf(',');
      unsigned long duration = 0;
      float newSetpoint = 0.0;

      if (commaIndex != -1 && commaIndex + 1 < command.length()) {
        duration = command.substring(0, commaIndex).toInt() * 1000; // Convert to milliseconds
        newSetpoint = command.substring(commaIndex + 1).toFloat();
      }

      if (newSetpoint >= minTemp && newSetpoint <= maxTemp) {
        Setpoint = newSetpoint;
        if (duration > 0) {
          targetTime = millis() + duration;
          isWaiting = true;
        } else {
          isWaiting = false;
        }
      }
    }
  }

  if (isWaiting && millis() >= targetTime) {
    isWaiting = false; // Reset the waiting condition
  }
}
