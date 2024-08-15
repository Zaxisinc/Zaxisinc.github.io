#include <Wire.h>
#include <Arduino.h>
#include<ADS1115_WE.h>
#include "EEPROM.h"
#define DACIN_ADDRESS 0x48
#define PSENIN_ADDRESS 0x49
#include <FastLED.h>
#define NUM_LEDS 2
CRGBArray<NUM_LEDS> leds;

#define MINILCD

#ifdef MINILCD

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C//0x78// ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#endif

TaskHandle_t PressureCore;

#define AdjMul .0002//.0003 200psi
#define ADS1114
#define REV 5
float pressure, temperature;
int pSense = 14;
int zSense = 15;
int AorD = 0;
bool adjust = false;
bool shimAdjust = true;
int retest = 0;
int retestNum = 0;
int ee = 0;
long dacNum = 0;
long dacNum2 = 0;
float pNum = 0;
float pNum2 = 0;
int address = 0;

// Duty Cycles for the proportional valves. 
int duty = 1769;//1990;// 10 volt749 for 500 psi;
int dutyLow = 1100;
int dutyMax = 2400;
TaskHandle_t Task1;

// Caibration Data {Pressure,Counts}
double Cal[2][2] = {{ 0, 0 }, { 400, 16000 } };
double DacCal[2][2] = {{ 0, 0 }, { 400, 16000 } };
int pCounts = 0;
int dCounts = 0;

//Timing variables
#define shimTimer 200000
unsigned long msTime = 0;
unsigned long msTime1 = 0;
unsigned long msTime2 = 0;
unsigned long shimmy = 0;
unsigned long runningTime = 0;
unsigned long timeoutTime = 0;
bool timeout = false;


unsigned long pressTime = 100;
unsigned long lastPressTime = 0;

//Valve variables
int freq = 10000;

int valve1 = 13;
int valve2 = 12;
int resolution = 12;
int steps;
float setPress;
int VENTVALVE = 12;//27;
long shimmyCount = 0;

//Input command variables
char tempChar;
String NUMS;
String NUMS2;
String in232;

//Setup ads1115 A/D
ADS1115_WE DAC = ADS1115_WE(DACIN_ADDRESS);
ADS1115_WE PSEN = ADS1115_WE(PSENIN_ADDRESS);


void setup() {
  //ESP.restart();

  EEPROM.begin(1000);
  Serial.begin(115200);
  Serial.println(EEPROM.readDouble(address), 8);
  Cal[0][0] = EEPROM.readDouble(address);
  address += sizeof(double);
  Serial.println(EEPROM.readDouble(address), 8);
  Cal[0][1] = EEPROM.readDouble(address);
  address += sizeof(double);
  Serial.println(EEPROM.readDouble(address), 8);
  DacCal[0][0] = EEPROM.readDouble(address);
  address += sizeof(double);
  Serial.println(EEPROM.readDouble(address), 8);
  DacCal[0][1] = EEPROM.readDouble(address);
  address += sizeof(double);
  Serial.println(EEPROM.readDouble(address), 8);
  Cal[1][0] = EEPROM.readDouble(address);
  address += sizeof(double);
  Serial.println(EEPROM.readDouble(address), 8);
  Cal[1][1] = EEPROM.readDouble(address);
  address += sizeof(double);
  Serial.println(EEPROM.readDouble(address), 8);
  DacCal[1][0] = EEPROM.readDouble(address);
  address += sizeof(double);
  Serial.println(EEPROM.readDouble(address), 8);
  DacCal[1][1] = EEPROM.readDouble(address);
  address += sizeof(double);
  Serial.print("THIS HSIT ");
  Serial.println(Cal[1][0]);
  delay(2000);
  /* DacCal[1][1] = 23235;
    DacCal[1][0] = 400;
     DacCal[0][1] = 0;
    DacCal[0][0] = 0;

     Cal[1][1] = 23235;
    Cal[1][0] = 400;
     Cal[0][1] = 0;
    Cal[0][0] = 0;
  */

  //dac.begin(0x60);

  //PWM Setup
  ledcSetup(valve1, freq, resolution);
  ledcAttachPin(13, valve1);
  ledcSetup(valve2, freq, resolution);
  ledcAttachPin(12, valve2);
  //Set PWM to 0
  ledcWrite(valve1, 0);
  ledcWrite(valve2, 0);

  Serial1.begin(230400, SERIAL_8N1, 16, 17);
  Serial.println("HEY WILL");
  Serial.println(AdjMul * Cal[1][0]);
  Serial.print("REV ");
  Serial.println(REV);
  xTaskCreatePinnedToCore(
    CheckPress, // Function to implement the task
    "PressureCore", // Name of the task
    10000,  // Stack size in words
    NULL,  // Task input parameter
    0,   //Priority of the task
    &PressureCore, //  Task handle.
    0); // Core where the task should run


  msTime = millis();
  setPress = 1;
  shimmy  = millis();
  FastLED.addLeds<NEOPIXEL, 17>(leds, NUM_LEDS);

}


float readChannel(ADS1115_WE AD) {
  float voltage = 0.0;
  // adc.setCompareChannels(channel);
  voltage = AD.getResult_mV(); // alternative: getResult_mV for Millivolt
  return voltage;
}


void loop() {
 
  msTime = millis();
  do{
for(int x = 0; x < 4000; x+=10)
 {
  Serial.print("X: ");
  Serial.println(x);
    ledcWrite(valve1, x);
    delay(50);
 }
  }while(true);  
  if ((msTime - shimmy) >= shimTimer)
  {
    shimmy  = millis();
    shimmyCount = 0;
  }
  if (msTime - lastPressTime > pressTime)
  {

    if (msTime - timeoutTime > 10000)
    {
      timeout = false;
      // Serial.println("Loop TIMEOUT = FALSE");
    }
    if (retest > retestNum && !timeout)
    {

      if ((pressure + (AdjMul * Cal[1][0])) < setPress)
        Increase1();
      else if (pressure - (AdjMul * 20 * (Cal[1][0] * 3) * 2) > setPress)
        Increase2(duty, setPress);
      else if ((pressure - (AdjMul * 2 * (Cal[1][0] * 3) * 2) > setPress) && (setPress > 65))
        shimmyCount = 0;
      retest = 0;
    }
    retest++;
    lastPressTime = millis();
  }
}

void Increase1()
{
  int x = 0;
  int  correct = 350;
  uint16_t i;

  float lastPress = pressure;
  float timeOutPressure = pressure;
  float tempPress = 0;
  float absPress = 0;
  bool retryTimeout = true;

  if ((pressure < setPress - 10) || shimmyCount > 400)
    i = duty;
  else
  {
    i = dutyLow;
    correct = 250;
  }
  runningTime = millis();
  msTime1 = millis();
  do
  {
    if (pressure < setPress - 20)
      i = dutyMax;
if(x%50000 == 0)
{
Serial.print("Duty: ");
Serial.println(i);
    ledcWrite(valve1, i);
}

    if (abs(pressure - lastPress) < .05 && i < dutyMax && (x % correct == 0) )
    {
      shimmyCount ++;
      if (shimmyCount > 200)
        i += 1;
      else
        i += 1;
      correct += 10;

      Serial.print(" Press ");
                Serial.print(pressure);
                Serial.print("  Set  ");
                Serial.print(setPress);
                Serial.print("  I    ");
                Serial.print(i);
                Serial.print("Shimmy");
                Serial.println(shimmyCount);
      

    }
   

    x++;
    if (x > 800000 )
    {

      tempPress = timeOutPressure - pressure;
      Serial.println(timeOutPressure);
      absPress = abs(tempPress);
      if ((absPress <= (.001 * Cal[1][0])))
      {
        if (!retryTimeout)
        {
          timeout = true;
          Serial.print(absPress);
          Serial.print("    ");
          Serial.print(.001 * Cal[1][0]);
          Serial.print("    ");
          Serial.println("Increase1 Timeout = True");
          timeoutTime = millis();
        }
        retryTimeout = false;
      }
      timeOutPressure = pressure;
      x = 0;
    }

    lastPress = pressure;
  } while ((pressure < (setPress - (AdjMul * (Cal[1][0] * 1)*.5))) && !timeout);
  //  delay(100);((pressure < (setPress )) && !timeout);
  ledcWrite(valve1, 0);
}

void Increase2(int num, float set)
{
  int x = 0;
  uint16_t i;
  float lastPress = setPress;
  float timeOutPressure = pressure;
  float tempPress = 0;
  float absPress = 0;
  bool retryTimeout = true;
  i = duty;
  if (setPress > 65)
    shimmyCount = 0;
  //Serial.println("Decrease");
  
  while ((pressure > setPress + AdjMul * Cal[1][0]) && !timeout)
  {

    if (pressure > setPress + 70)
      i = dutyMax;
    ledcWrite(valve2, i);

    if ((pressure - lastPress) < .5 && i < dutyMax)
      i += 1;
    x++;
    //Serial.println(x);
    if (x > 400000 )
    {
      tempPress = timeOutPressure - pressure;
      Serial.println(timeOutPressure);
      absPress = abs(tempPress);
      if (absPress <= (.001 * Cal[1][0]))
      {
        if (!retryTimeout)
        {
          timeout = true;

          Serial.println("Increase2 Timeout = True");
          timeoutTime = millis();
        }
        retryTimeout = false;
      }
      timeOutPressure = pressure;
      x = 0;
    }
    lastPress = pressure;

  }
  ledcWrite(valve2, 0);

}


void CheckPress(void * pvParameters)
{
  int avg, samples, output, z, readDac;
  float temp;
  output = 0;
  samples = 0;
  z = 0;
  readDac = 0;
  if (!AorD)
  {
    Wire.begin(); // wake up I2C bus
    delay (500);
    if (!DAC.init()) {
      Serial.println("ADS1115 not connected!");
    }
    if (!PSEN.init()) {
      Serial.println("ADS1115 not connected!");
    }
    //  Serial.println("Good");
    DAC.setVoltageRange_mV(ADS1115_RANGE_6144);
    //adc.setCompareChannels(ADS1115_COMP_1_GND);
    DAC.setCompareChannels(ADS1115_COMP_0_GND);
    DAC.setConvRate(ADS1115_860_SPS);
    DAC.setMeasureMode(ADS1115_CONTINUOUS);

    PSEN.setVoltageRange_mV(ADS1115_RANGE_6144);
    PSEN.setCompareChannels(ADS1115_COMP_0_GND);
    PSEN.setConvRate(ADS1115_860_SPS);
    PSEN.setMeasureMode(ADS1115_CONTINUOUS);

#ifdef MINILCD
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for (;;); // Don't proceed, loop forever
    }

    // Clear the buffer
    display.clearDisplay();
#endif
  }
  for (;;) {



    if (!AorD)
    {

      while (Serial.available() > 0) {
        tempChar = Serial.read();
        //Serial.println("1");
        in232.concat(tempChar);
        delayMicroseconds(500);
      }
      // while (Serial1.available() > 0) {
      //   tempChar = Serial1.read();
      //  in232.concat(tempChar);
      //Serial.println(tempChar);
      // delayMicroseconds(50);
      // }
      // Serial.println("1");
      if (in232.length() > 0)
      {

        if (in232.indexOf("C0") > -1)
        {

          NUMS = in232.substring(in232.indexOf("C0") + 2);

          Cal[0][0] = NUMS.toDouble();
          Cal[0][1] = pCounts;
          DacCal[0][0] = NUMS.toDouble();
          DacCal[0][1] = dCounts;
          Serial.println(NUMS.toDouble());
          address = 0;
          Serial.println(Cal[0][0]);
          Serial.println(Cal[0][1]);

          Serial.println(DacCal[0][1]);
          EEPROM.writeDouble(address, Cal[0][0]);
          address += sizeof(double);
          EEPROM.writeDouble(address, Cal[0][1]);
          address += sizeof(double);
          EEPROM.writeDouble(address, DacCal[0][0]);
          address += sizeof(double);
          EEPROM.writeDouble(address, DacCal[0][1]);
          address += sizeof(double);
          EEPROM.commit();

          in232 = "";
          vTaskDelay(300);
        }

        if (in232.indexOf("C1") > -1)
        {
          NUMS = in232.substring(in232.indexOf("C1") + 2);
          Cal[1][0] = NUMS.toDouble();
          Cal[1][1] = pCounts;
          address = 32;
          EEPROM.writeDouble(address, Cal[1][0]);
          address += sizeof(double);
          EEPROM.writeDouble(address, Cal[1][1]);
          address += sizeof(double);
          EEPROM.commit();
          in232 = "";
        }

        if (in232.indexOf("C2") > -1)
        {
          NUMS = in232.substring(in232.indexOf("C2") + 2);
          DacCal[1][0] = NUMS.toDouble();
          DacCal[1][1] = dCounts;
          address = 48;
          EEPROM.writeDouble(address, DacCal[1][0]);
          address += sizeof(double);
          EEPROM.writeDouble(address, DacCal[1][1]);
          address += sizeof(double);
          EEPROM.commit();
          in232 = "";
        }

        if (in232.indexOf("CALRESET") > -1)
        {

          Cal[0][0] = 0;
          Cal[0][1] = -300;
          DacCal[0][0] = 0;
          DacCal[0][1] = -3200;
          Cal[1][0] = 400;
          Cal[1][1] = 16000;
          DacCal[1][0] = 400;
          DacCal[1][1] = 16000;
          address = 0;
          EEPROM.writeDouble(address, Cal[0][0]);
          address += sizeof(double);
          EEPROM.writeDouble(address, Cal[0][1]);
          address += sizeof(double);
          EEPROM.writeDouble(address, DacCal[0][0]);
          address += sizeof(double);
          EEPROM.writeDouble(address, DacCal[0][1]);
          address += sizeof(double);
          EEPROM.writeDouble(address, Cal[1][0]);
          address += sizeof(double);
          EEPROM.writeDouble(address, Cal[1][1]);
          address += sizeof(double);
          EEPROM.writeDouble(address, DacCal[1][0]);
          address += sizeof(double);
          EEPROM.writeDouble(address, DacCal[1][1]);
          address += sizeof(double);
          EEPROM.commit();
          in232 = "";
        }

        if (in232.indexOf("S") > -1)
        {
          NUMS = in232.substring(in232.indexOf("S") + 1);

          in232 = "";
        }
        if (in232.indexOf("F") > -1)
        {
          NUMS = in232.substring(in232.indexOf("F") + 1);
          duty = NUMS.toInt();
          in232 = "";
        }

        if (in232.indexOf("V") > -1)
        {
          NUMS = in232.substring(in232.indexOf("V") + 1);
          steps = NUMS.toInt();
          if (in232.indexOf(",") > -1)
          {
            NUMS = in232.substring(in232.indexOf(",") + 1);
            setPress = NUMS.toInt();
          }
          else
          {
            setPress = -1;
          }
          Increase2(steps, setPress);

          in232 = "";
        }

        if (in232.indexOf("P") > -1)
        {
          // Serial.println(in232);
          NUMS = in232.substring(in232.indexOf("P") + 1);
          NUMS2 = in232.substring(in232.indexOf("P", in232.indexOf("P") + 1) + 1);
          adjust = true;
          dacNum = NUMS.toInt();
          dacNum2 = NUMS2.toInt();

          if (dacNum == -1)
            setPress = 0;
          else if (dacNum == 0)
            setPress;
          else
            setPress = dacNum / 131.07;
          //retest = 0;
          Serial.print("NUMS In ");
          Serial.println(setPress);

          in232 = "";
          adjust = false;
        }


        if (in232.indexOf("D") > -1)
        {
          NUMS = in232.substring(in232.indexOf("D") + 1);
          AorD = NUMS.toInt();
          setPress = 1;
          in232 = "";
        }

        if (in232.indexOf("Z") > -1)
        {
          //Serial.println(in232);
          NUMS = in232.substring(in232.indexOf("Z") + 1);
          NUMS2 = in232.substring(in232.indexOf("Z", in232.indexOf("Z") + 1) + 1);
          adjust = true;
          pNum = NUMS.toFloat();
          pNum2 = NUMS2.toFloat();

          if (pNum == -1)
            pressure = 0;
          else if (pNum == 0)
            pressure;
          else
            pressure = pNum;
          //retest = 0;
          //  Serial.print("pNUMS In ");
          // Serial.println(pressure);

          in232 = "";
          adjust = false;
        }


        in232 = "";
      }
      //   Serial.println("11");
      //  }
      //  else
      // {
      // msTime1 = millis();
#ifdef ADS1115
      DAC.setCompareChannels(ADS1115_COMP_1_GND);

      pressure = (readChannel(DAC) - 3376) / 72;
      DAC.setCompareChannels(ADS1115_COMP_0_GND);
      setPress = (readChannel(DAC) - 16) / 89.33;
#else

      //dCounts = readChannel(DAC);
      if (readDac >= 30)
      {
        samples += readChannel(DAC);
        readDac = 0;
        z++;
      }
      readDac++;
      // Serial.println(dCounts);
      // dCounts -= DacCal[0][1];
      // setPress=((DacCal[1][0])/(DacCal[1][1] - DacCal[0][1]))*abs(dCounts);
      // setPress= map(dCounts, DacCal[0][1],DacCal[1][1],0,DacCal[1][0]);
      if (z >= 10)
      {

        dCounts = samples / z;
        setPress = (dCounts - DacCal[0][1]) * (DacCal[1][0]) / (DacCal[1][1] - DacCal[0][1]);
        z = 0;
        samples = 0;
         UpdateLED();
      }

      pCounts = readChannel(PSEN);
      // pCounts -= Cal[0][1];
      //  pressure= map(pCounts, Cal[0][1],Cal[1][1],0,Cal[1][0]);
      pressure = (pCounts - Cal[0][1]) * (Cal[1][0]) / (Cal[1][1] - Cal[0][1]);
      // pressure = ((Cal[1][0])/(Cal[1][1] - Cal[0][1]))*abs(pCounts);
if(output % 500 ==0)
{
          Serial.print("Set:");
        Serial.println(setPress);
        Serial.print("PSI:");
        Serial.print(pressure);
}

#endif

      //  msTime2 = millis();
      output++;
#ifdef MINILCD
      if (output % 500 == 0)
      {



        //  UpdateDisplay();

        //   display.clearDisplay();

        // display.setTextSize(3);             // Normal 1:1 pixel scale
        // display.setTextColor(SSD1306_WHITE);        // Draw white text
        //display.setCursor(20,5);             // Start at top-left corner

        display.clearDisplay();

        display.setTextSize(2);             // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE);        // Draw white text
        display.setCursor(0, 0);
        display.print("Set:");
        display.println(setPress);
        display.print("PSI:");
        display.print(pressure);
        //delay(10);
        display.display();
      }
#endif
      if (output > 100000)
      {
        vTaskDelay(1);
#ifdef MIsNILCD
        UpdateDisplay();
#endif
        /*
                Serial.print(dCounts);
                Serial.print("  D  ");
                Serial.print(DacCal[0][1]);
                Serial.print("  0  ");
                Serial.print(DacCal[1][1]);
                Serial.print("  1  ");
                Serial.print(DacCal[1][0]);
                Serial.print("  1  ");

                Serial.print(pCounts);
                Serial.print(" Press ");
                Serial.print(pressure);
                Serial.print("  Set  ");
                Serial.println(setPress);
        */
        output = 0;
      }
    }
  }

}

#ifdef MINIL1CD
void UpdateDisplay(void) {

  display.clearDisplay();

  display.setTextSize(3);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(20, 5);            // Start at top-left corner

  display.clearDisplay();

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);
  display.print("Set:");
  display.println(setPress);
  display.print("PSI:");
  display.print(pressure);
  delay(10);
  display.display();

}
#endif
void UpdateLED()
{
  int bright = map(setPress,0,300,0,255);
  leds[1] = CHSV(180, 255, bright);
        FastLED.delay(1);
     leds[0] = CHSV(100, 255, 200);
           FastLED.delay(1);
}