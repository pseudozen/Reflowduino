/*
 * Title: ReflowPlate PID Test
 * Based on Reflowduino by Timothy Woo
 * 
 * -----------------------------------------------------------------------------------------------
 * This is an example sketch to test the PID control of the Reflowduino and facilitate choosing the
 * right PID constants. This example sets the desired temperature to a fixed value and controls the
 * oven or hot platefot, and steady-
 * state error which are explained in more detail on my Github tutorial.
 * 
 * Note: From testing it was concluded that a subtraction factor may be the easiest and most
 * effective solution for compensating for overshoot during the reflow phase.
 * 
 * -----------------------------------------------------------------------------------------------
 * Credits: Special thanks to Brett Beauregard, author of the Arduino PID Library!
 * 
 * -----------------------------------------------------------------------------------------------
 * License: This code is released under the GNU General Public License v3.0
 * https://choosealicense.com/licenses/gpl-3.0/ and appropriate attribution must be
 * included in all redistributions of this code.
 * 
 * -----------------------------------------------------------------------------------------------
 * This version of the code is set up for the Arduino Pro Micro
 */

#include <SoftwareSerial.h> // Library needed for Bluetooth communication
#include <Keyboard.h> // Only if you need the ATmega32u4 to act as a keyboard

// Libraries needed for using MAX31855 thermocouple interface
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// Library for PID control
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

// Define pins
#define relay 7
#define LED 13
#define MAX_CS 8 // MAX31855 chip select pin


// Initialize thermocouple with hardware SPI
// Reflowduino uses hardware SPI to save digital pins
//Adafruit_MAX31855 thermocouple(MAX_CS);

Adafruit_MLX90614 thermocouple = Adafruit_MLX90614();

// Define if you want to enable the keyboard feature to type data into Excel
#define enableKeyboard false

// Define a subtraction constant to compensate for overshoot:
#define T_const 5; // From testing, overshoot was about 5-6*C

// Define a desired temperature in deg C
#define desiredTemp 75 - T_const

// Define PID parameters. Tune them to your taste!
#define PID_sampleTime 1000
#define Kp 150
#define Ki 0
#define Kd 100

// Bluetooth app settings. Define which characters belong to which functions
#define dataChar '*' // App is receiving data from Reflowduino
#define stopChar '!' // App is receiving command to stop reflow process (process finished!)
#define startReflow 'A' // Command from app to "activate" reflow process
#define stopReflow 'S' // Command from app to "stop" reflow process at any time

double temperature, output, setPoint; // Input, output, set point
double t_start;
PID myPID(&temperature, &output, &setPoint, Kp, Ki, Kd, DIRECT);

// Logic flags
bool justStarted = true;
bool reflow = false; // Baking process is underway!

int windowSize = 2000;
unsigned long sendRate = 2000; // Send data to app every 2s
unsigned long previousMillis = 0;
unsigned long windowStartTime, timer;

void setup() {
  Serial.begin(9600); // This should be different from the Bluetooth baud rate

  pinMode(LED, OUTPUT);
  pinMode(relay, OUTPUT);

  digitalWrite(LED, LOW);
  digitalWrite(relay, LOW); // Set default relay state to OFF

  setPoint = desiredTemp;

  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTime(PID_sampleTime);
  myPID.SetMode(AUTOMATIC); // Turn on PID control

//   while (!Serial) delay(1); // OPTIONAL: Wait for serial to connect
  Serial.println("*****Reflowduino PID Test*****");

  if (enableKeyboard) Keyboard.begin(); // Only if you want to type data into Excel

  thermocouple.begin();  // start up the MXL temp sensor
}

void loop() {
  /***************************** MEASURE TEMPERATURE *****************************/
  temperature = thermocouple.readObjectTempC(); // Read temperature
//  temperature = thermocouple.readObjectTempF(); // Alternatively, read in deg F but will need to modify code
  
  /***************************** REFLOW PROCESS CODE *****************************/
  if (reflow) {
    digitalWrite(LED, HIGH); // Red LED indicates process is underway

    // This only runs when you first start the test
    if (justStarted) {
      justStarted = false;
      windowStartTime = millis();
      
      if (isnan(temperature)) {
       Serial.println("Invalid reading, check thermocouple!");
      }
      else {
       Serial.print("Starting temperature: ");
       Serial.print(temperature);
       Serial.println(" *C");
      }
    }
  
    // Compute PID output (from 0 to windowSize) and control relay accordingly
    myPID.Compute(); // This will only be evaluated at the PID sampling rate
    if (millis() - windowStartTime >= windowSize) windowStartTime += windowSize; // Shift the time window
    if (output > millis() - windowStartTime) digitalWrite(relay, HIGH);
    else digitalWrite(relay, LOW);
  }
  else digitalWrite(LED, LOW);

  /***************************** BLUETOOTH CODE *****************************/
  char request = ' ';

  // Send data to the app periodically
  if (millis() - previousMillis > sendRate) {
    previousMillis = millis();
    Serial.print("--> Temperature: "); // The right arrow means it's sending data out
    Serial.print(temperature);
    Serial.println(" *C");
    if (!isnan(temperature)) { // Only send the temperature values if they're legit
//      BT.print(dataChar); // This tells the app that it's data
//      BT.print(String(temperature)); // Need to cast to String for the app to receive it properly

      if (enableKeyboard && reflow) {
        // Type time and temperature data into Excel on separate columns!
        Keyboard.print((millis()-timer)/1000); // Convert elapsed time from ms to s
        Keyboard.print('\t'); // Tab to go to next column
        Keyboard.print(temperature);
        Keyboard.println('\n'); // Jump to new row
      }
    }
  }
  
  
//  Serial.print("REQUEST: "); Serial.println(request); // DEBUG

  if (request == startReflow) { // Command from app to start reflow process
    justStarted = true;
    reflow = true; // Reflow started!
    timer = millis(); // Timer for logging data points
    Serial.println("<-- ***Reflow process started!"); // Left arrow means it received a command
  }
  else if (request == stopReflow) { // Command to stop reflow process
    digitalWrite(relay, LOW); // Turn off appliance and set flag to stop PID control
    reflow = false;
    Serial.println("<-- ***Reflow process aborted!");
  }
  // Add you own functions here and have fun with it!
  

  // Alternatively, read commands from the serial monitor
  char serialByte;
  
  if (Serial.available() > 0) {
    serialByte = Serial.read();
  }
  if (serialByte == startReflow) {
    justStarted = true;
    reflow = true; // Reflow started!
    t_start = millis(); // Record the start time
    timer = millis(); // Timer for logging data points
    Serial.println("<-- ***Reflow process started!"); // Left arrow means it received a command
  }
  else if (serialByte == stopReflow) { // Command to stop reflow process
    digitalWrite(relay, LOW); // Turn off appliance and set flag to stop PID control
    reflow = false;
    Serial.println("<-- ***Reflow process aborted!");
  }

}
