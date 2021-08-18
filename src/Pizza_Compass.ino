/*
 * Project: Beer Crawl Compass
 * Author: Seamus de Cleir
 * Description: Adapted from Joe Grand's (@joegrand) Pizza Compass (https://github.com/joegrand/pizza-compass/)
 * This compass uses a ESP32, GY511, SIM800L, NEO 6M GPS and LED to lead the user to different pubs on a pub crawl
 * License: Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)
 */

#include <Adafruit_NeoPixel.h>
#include <NeoPatterns.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LSM303.h>
#include <Wire.h>

/* ************************* Variables ************************* */

// PINs for Neo Ring
#define PIXEL_PIN           12
#define PIXEL_COUNT         24

// Length of button press required to enter calibration mode on start-up (in ms)
#define COMPASS_CAL_DELAY   3000

// Time between GPS/compass updates
#define UI_INTERVAL_TIME    100

// Distance to target (in meters) before LED's blink green
#define UI_DISTANCE_ALERT   50.0

// Blink time (in ms) for visual alert when we're within range of target
#define UI_BLINK_TIME       250

// Test variables
#define GPS_TARGET_LAT      53.0978374      // Kinnitty
#define GPS_TARGET_LNG      -7.7192709

void Ring1Complete(NeoPatterns *aLedsPtr);

// Set colour changing variable
uint32_t LEDColour;

// Neopixel LEDs
NeoPatterns colourWipe = NeoPatterns(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800, &Ring1Complete);
NeoPatterns rainbow = NeoPatterns(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Pin definitions
const int buttonStart = 2;
static const uint32_t ConsoleBaud = 115200;
unsigned long time_now = 0;

//Baud rate for GY511
static const uint32_t GPSBaud = 9600;

// Serial Pins
static const int RXPin = 16, TXPin = 17;

//GPS
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
double local_lat, local_lng;
double target_lat, target_lng;
double courseToNextPub;
double distanceToNextPub;
int direction_index;

// LSM303 - GY511
LSM303 compass;

// If we haven't searched for pizza yet
bool first_time = true;

// Blink LEDs if we are within range of target
bool blinkFlag = false;

unsigned long lastUpdateTime = 0;
unsigned long lastPublishTime = 0;
unsigned long lastBlinkTime = 0;


/* ************************* Steup ************************* */

void setup() {

  Wire.begin();
  pinMode(buttonStart, INPUT);
  Serial.begin(ConsoleBaud);

  Serial.print("[*] Initializing Neopixels...");
  rainbow.begin();
  colourWipe.begin();
  colourWipe.clear();
  Serial.println("Done!");

  colourWipe.ColorSet(COLOR32_PURPLE_HALF);
  colourWipe.show();
  delay(1000);
  colourWipe.clear();

  // Wait for valid GPS fix
  Serial.print("[*] Waiting for GPS...");
  
  // GPS baud rate
  ss.begin(GPSBaud);

  // Read GPS buffer
  while (ss.available() > 0){
    gps.encode(ss.read());
  }

  // Set Green LEDs to circle while GPS is found
  LEDColour = COLOR32_GREEN_HALF;
  colourWipe.ColorWipe(LEDColour, 80, 0, DIRECTION_UP);

    // Wait until the GPS returns valid
  while (gps.location.isValid() == false){
    
    colourWipe.update();
    
    // Continue to read buffer
    while (ss.available() > 0){
      gps.encode(ss.read());
    }
  }

  // GPS Found
  colourWipe.updateAndWaitForPatternToStop();
  Serial.println("Done!");

  
  Serial.print("[*] Initializing magnetometer...");
  // Automatically detect type of LSM303
  bool error = compass.init();
  
  if (!error){
    // Cannot detect the LSM303
    Serial.println("Failed!");
    Compass_Error();
  }
  else{
    // Initilize accelerometer and magnetometer
    compass.enableDefault();
    Serial.println("Done!");
  }

  Serial.print("[*] Calibrating magnetometer...");
  
  LEDColour = COLOR32_BLUE_HALF;
  colourWipe.ColorWipe(LEDColour, 70, 0, DIRECTION_UP);

   // Wait for a button press
  while (digitalRead(buttonStart) == 0){
    colourWipe.update();
  }

  // Calibrate the compass
  Compass_Warmup();

  colourWipe.clear();

  unsigned long start_time = millis();
  
  // If button press is short, skip calibration and use default values
  bool new_cal = false;

  LEDColour = COLOR32_RED_HALF;
  colourWipe.ColorSet(LEDColour);
  
  while (digitalRead(buttonStart) == 1){
    colourWipe.show();
    if (millis() - start_time > COMPASS_CAL_DELAY){
      // Read new compass values to calculate minimum/maximum ranges
      new_cal = true;
    }
  }
  colourWipe.clear();
  Compass_Calibration(new_cal);   
  
  if (new_cal){
    Serial.println("Done!");
  }
  else{
    Serial.println("Skipped!");
  }
  
  colourWipe.clear();
  
  Compass_DisplayCalibration();

  compass.m_min = (LSM303::vector<int16_t>){-542,   -713,   -526};
  compass.m_max = (LSM303::vector<int16_t>){+330,   +282,   +369};

}

/* ************************* Main Loop ************************* */

void loop(){

  // Wait here for first search
  while (first_time == true){   
      rainbow.RainbowCycle(10);
      rainbow.updateAndWaitForPatternToStop();
      
      // If the button is pressed
      if (digitalRead(buttonStart) == 1)
      {
        Serial.println("Done!");
        first_time = false;
      }
  }

  // Check buffer for GPS location
  while (ss.available() > 0){
    gps.encode(ss.read());
  }

  if ( (millis() - lastUpdateTime >= UI_INTERVAL_TIME)){
      lastUpdateTime = millis();
      
      Serial.printf("[*] Let's go!");

      // Get current local GPS coordinates
      local_lat = gps.location.lat();
      local_lng = gps.location.lng();

      // Force hard-coded coordinates here for testing
      //local_lat = GPS_LOCAL_LAT;
      //local_lng = GPS_LOCAL_LNG;
      target_lat = GPS_TARGET_LAT;
      target_lng = GPS_TARGET_LNG;

      Serial.printf("-> Local: %.10g,%.10g", local_lat, local_lng);
      Serial.printf("-> Target: %.10g,%.10g", target_lat, target_lng);

      distanceToNextPub = 
      TinyGPSPlus::distanceBetween(
        local_lat,
        local_lng,
        target_lat, 
        target_lng);

      courseToNextPub = 
      TinyGPSPlus::courseTo(
        local_lat,
        local_lng,
        target_lat, 
        target_lng);

      Serial.printf("-> Distance: %.6g meters", distanceToNextPub);
      Serial.printf("-> Course: %.6g [", courseToNextPub);
      Serial.print(TinyGPSPlus::cardinal(courseToNextPub));
      Serial.printf("]");

      // Read new compass values
      compass.read();
      float heading = compass.heading((LSM303::vector<int>){1, 0, 0});
      Serial.printf("-> Current Heading: %d", (int)heading);

      int courseChangeNeeded = (int)(360 + courseToNextPub - heading) % 360;
      Serial.printf("-> Course Change: %d", courseChangeNeeded);

      // Divide to find which pie slice it's in (360 degrees / number of pixels)
      direction_index = courseChangeNeeded / (360 / PIXEL_COUNT);

      Serial.printf("-> LED: %d", direction_index);

      unsigned long currentBlinkTime = millis();
      
      // If we are within range of our target...
      if (distanceToNextPub <= UI_DISTANCE_ALERT)
      {
        if (currentBlinkTime - lastBlinkTime >= UI_BLINK_TIME)
        {
          lastBlinkTime = currentBlinkTime;
          if (blinkFlag == false)
          {
            blinkFlag = true;
          }
          else
          {
            blinkFlag = false;
          }
        }
      }
      else
      {
        blinkFlag = false;
      }
        
      // Light the pixel for the direction we need to go
      for (int i = 0; i < rainbow.numPixels(); i++) {
        
        if (i == direction_index && blinkFlag == false){

          // If we're facing in the direction of our target
          if (direction_index == 0){
            
            rainbow.setPixelColor(i, COLOR32_GREEN);
          }else{
            
            rainbow.setPixelColor(i, COLOR32_RED);
          }
        }
        else{
          // Turn off all others
          rainbow.setPixelColor(i, COLOR32_BLACK);
        }
      }

      rainbow.show();
    }
  
}

/* ************************* LED Functions ************************* */

// Ring1 Completion Callback
void Ring1Complete(NeoPatterns *aLedsPtr){
    
    if (colourWipe.Color1 != 0){
      colourWipe.ColorWipe(COLOR32_BLACK, 80, FLAG_DO_NOT_CLEAR, DIRECTION_UP);
    }else{
      colourWipe.ColorWipe(LEDColour, 80, 0, DIRECTION_UP); // light Blue
    }
}

/* ************************* Compass - LSM303 - GY511 ************************* */

// Code based on https://github.com/pololu/lsm303-arduino

// Calibrate sensor by finding the range of X, Y, and Z values 
// for the accelerometer and magnetometer. This is done by rotating 
// the device around each axis.
void Compass_Calibration(bool read_new_values)
{
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};

  // Set default values in case user skips calibration (not recommended)
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  if (read_new_values)
  {
    // Continually get readings until the button is pressed
    while (digitalRead(buttonStart) == 1)
    {
      // Read all 6 channels of the LSM303 and stores them in the object variables
      compass.read();
  
      // Set minimum and maximum values of magnetometer readings. Ignore 0 readings
      if (compass.m.x != 0.0 && compass.m.y != 0.0 && compass.m.z != 0)
      {
        running_min.x = min(running_min.x, compass.m.x);
        running_min.y = min(running_min.y, compass.m.y);
        running_min.z = min(running_min.z, compass.m.z);

        running_max.x = max(running_max.x, compass.m.x);
        running_max.y = max(running_max.y, compass.m.y);
        running_max.z = max(running_max.z, compass.m.z);
    
      }
      
      delay(5);
    }

    delay(100);
    // Wait until button is released
    while (digitalRead(buttonStart) == 0);
    delay(100); 

    // Update global variables with calibrated results
    compass.m_min = running_min;
    compass.m_max = running_max;
  }
}

/* ************************* Compass_DisplayCalibration ************************* */

void Compass_DisplayCalibration(void)
{
  Serial.printf("-> Max Value (X): %6d", compass.m_max.x);
  Serial.printf("-> Min Value (X): %6d", compass.m_min.x);

  Serial.printf("-> Max Value (Y): %6d", compass.m_max.y);
  Serial.printf("-> Min Value (Y): %6d", compass.m_min.y);

  Serial.printf("-> Max Value (Z): %6d", compass.m_max.z);
  Serial.printf("-> Min Value (Z): %6d", compass.m_min.z);
}

/* ************************* Compass_Warmup ************************* */

void Compass_Warmup(void) 
{
  // Discard samples on power up
  for (int ignore = 0; ignore < 100; ignore++) 
  {
    // Read all 6 channels of the LSM303
    compass.read();
    delay(10);
  }
}

/* ************************* Compass_Error ************************* */

void Compass_Error(void)
{
  while (1)
  {
    // Blink LEDs to indicate failure
    colourWipe.ColorSet(COLOR32_RED);
    colourWipe.show();
    delay(1000);
    colourWipe.ColorSet(COLOR32_BLACK);
    colourWipe.show();
    delay(1000);
  }
}
