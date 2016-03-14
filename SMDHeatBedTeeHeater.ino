#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#include "pid_control.h"

// TEMP_HIST_SIZE == display.width()
#define TEMP_HIST_SIZE 128
#define GRAPH_HEIGHT 104
#define TEMP_OVERSAMPLING 4
#define PIN_SSR 3
uint16_t temp_samples_[TEMP_OVERSAMPLING];
uint8_t temp_samples_idx_ = 0;
int8_t temp_history_[TEMP_HIST_SIZE];
uint8_t temp_history_idx_ = 0;

/*
// If using software SPI (the default case):
#define OLED_MOSI  11 
#define OLED_CLK   13
#define OLED_DC    8
#define OLED_CS    10
#define OLED_RESET 9
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
*/

// Uncomment this block to use hardware SPI
#define OLED_DC     8
#define OLED_CS     SS
#define OLED_RESET  9
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);


// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin1 = A1;  // Analog input pin that the potentiometer is attached to
const int analogInPin2 = A2;  // Analog input pin that the potentiometer is attached to

int16_t potiRawValue = 0;        // value read from the pot
int16_t tempRawValue = 0;        // value read from the pot
float aRef=3.3;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  //PID
  pid_setP(4096);
  pid_setI(717);
  pid_setD(12288);
  pid_setLimits(0,255);

  //DISPLAY
  for (uint8_t c=0; c<TEMP_HIST_SIZE; c++) {
    temp_history_[c] = -10; // outside display
  }
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);  
  display.display(); // show splashscreen
  delay(2000);
  display.clearDisplay();   // clears the screen and buffer

}

void displayTemp(float target, float current) {
  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(F("Set: ")); display.print(target); display.println(F("degC"));
  display.print(F("Temp: ")); display.print(current); display.println(F("degC"));

  uint8_t y = display.height() - 1 - (current * GRAPH_HEIGHT / 200);
  uint8_t setline = display.height()-1-(target * GRAPH_HEIGHT / 200);

  temp_history_[temp_history_idx_] = y;
  temp_history_idx_++;
  temp_history_idx_ %= TEMP_HIST_SIZE;

  display.drawLine(0, setline, display.width()-1, setline, WHITE);

  for (uint8_t c=0; c<TEMP_HIST_SIZE; c++) {
    display.drawPixel(c, temp_history_[(c + temp_history_idx_) % TEMP_HIST_SIZE], WHITE);
  }
  display.display();
}

void loop() {
  // read the analog in value:
  potiRawValue = analogRead(analogInPin1);            
  tempRawValue = analogRead(analogInPin2);            
  // map it to the range of the analog out:

  // print the results to the serial monitor:
  Serial.print(F("potiRawValue = "));
  Serial.println(potiRawValue);
  Serial.print(F("tempRawValue = "));
  Serial.println(tempRawValue);

 // now print out the temperature
  float target_temperatureC = ((float(potiRawValue)*aRef / 1024.0) - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
  // now print out the temperature
  float temperatureC = ((float(tempRawValue)*aRef / 1024.0) - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                               //to degrees ((volatge - 500mV) times 100)
  Serial.print(F("Target : ")); Serial.print(target_temperatureC); Serial.println(F(" degrees C"));
  Serial.print(F("Current: ")); Serial.print(temperatureC); Serial.println(F(" degrees C"));
 
  displayTemp(target_temperatureC, temperatureC);

  temp_samples_[temp_samples_idx_] = tempRawValue;
  temp_samples_idx_++;
  temp_samples_idx_%=TEMP_OVERSAMPLING;

  //PID every TEMP_OVERSAMPLING runs
  if (temp_samples_idx_ == 0)
  {
    //Calc Average
    tempRawValue=0;
    for (uint8_t c=0; c<TEMP_OVERSAMPLING; c++)
      tempRawValue += temp_samples_[c];
    tempRawValue /= TEMP_OVERSAMPLING;
    
    //PID
    pid_setTargetValue(potiRawValue);
    uint16_t stellwert = pid_calc(tempRawValue);
    Serial.print(F("Stellwert : ")); Serial.print(stellwert); Serial.println("");
    analogWriteFrequency(PIN_SSR, 200); // 200 Hz
    analogWrite(PIN_SSR, stellwert);
  }

  delay(500);

}
