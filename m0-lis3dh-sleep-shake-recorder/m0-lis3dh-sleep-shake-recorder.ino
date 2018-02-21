// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>


// power management
#include "LowPower.h"

// SD card
#include <SPI.h>
#include <SD.h>
#define cardSelect 4

// buttons and LEDs
#define BLUE_LED 10
#define BUTTON_6 6
#define BUTTON_5 5
#define RED_LED 13

// button long click time
#define LONG_CLICK_MS 2000

// how many samples to capture
// 100 Hz * 60 seconds
#define SAMPLES_TO_CAPTURE 6000

// pin to connect to INT pin on LIS3DH
#define INT_PIN A0

// RTC
#include "RTClib.h"
// clock object
RTC_DS3231 rtc;


// battery voltage pin
// this is button "A" on the feather OLED, so do not use it for interrupts
#define VBATPIN A7


int intPin = A0;
int LIS3DH_ADDR = 0x18; // change this to 0x19 for alternative i2c address
int reading = 0; //counter for number of readings taken - for diagnostic use
volatile int interruptFlag = 0;

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// file for the SD card
File logfile;

int chx;
int chy;
int chz;
int n;

char filename[13];

/*-----------------------------------------------------------------------------*/
unsigned int readRegister(byte reg) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(LIS3DH_ADDR, 1);
  return Wire.read();
}

/*-----------------------------------------------------------------------------*/
void writeRegister(byte reg, byte data) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

/*-----------------------------------------------------------------------------*/
/* Initialize accelerometer in the host processor. ----------------------------------*/
//It only needs to be executed one time after power up in initialization routine.
void init_ACC(void)
{
    // configurations for control registers
    writeRegister(0x20, 0x57); //Write A5h into CTRL_REG1;      // Turn on the sensor, enable X, Y, Z axes with ODR = 100Hz normal mode.
    writeRegister(0x21, 0x09); //Write 09h into CTRL_REG2;      // High-pass filter (HPF) enabled
    writeRegister(0x22, 0x40); //Write 40h into CTRL_REG3;      // ACC AOI1 interrupt signal is routed to INT1 pin.
    writeRegister(0x23, 0x00); //Write 00h into CTRL_REG4;      // Full Scale = +/-2 g
    writeRegister(0x24, 0x08); //Write 08h into CTRL_REG5;      // Default value is 00 for no latching. Interrupt signals on INT1 pin is not latched. 
                                                                //Users don’t need to read the INT1_SRC register to clear the interrupt signal.
    // configurations for wakeup and motionless detection
    writeRegister(0x32, 0x10); //Write 10h into INT1_THS;          // Threshold (THS) = 16LSBs * 15.625mg/LSB = 250mg.
    writeRegister(0x33, 0x00); //Write 00h into INT1_DURATION;     // Duration = 1LSBs * (1/10Hz) = 0.1s.
    //readRegister();  //Dummy read to force the HP filter to set reference acceleration/tilt value
    writeRegister(0x30, 0x2A); //Write 2Ah into INT1_CFG;          // Enable XLIE, YLIE, ZLIE interrupt generation, OR logic.
 
}

/*-----------------------------------------------------------------------------*/
void setup(void) {
#ifndef ESP8266
  //while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  // button pins
  pinMode(BUTTON_5, INPUT_PULLUP);
  pinMode(BUTTON_6, INPUT_PULLUP);

  // LED pins
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);

  // LIS3DH int pin
  pinMode(INT_PIN, INPUT);
  // Serial.begin(115200);
  // Serial.println("LIS3DH test!");

  // get the current voltage of the battery from
  float battery = getBatteryVoltage();

  // Serial.print("Battery voltage: ");
  // Serial.println(battery);

  // check if SD card is ready
  if (!SD.begin(cardSelect))
  {
    // Serial.println("SD card failed to initialize");
    while (1)
    {
      blinkRed(1, 50);
    }
  }
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    // Serial.println("Couldnt start");
    while (1)
    {
      blinkBlue(1, 50);
    }
  }
  // Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  
  // Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  // Serial.println("G");
  
  pinMode(intPin, INPUT);

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // confirm the time
  // now object
  DateTime now = rtc.now();
//  Serial.print(now.year(), DEC);
//  Serial.print('/');
//  Serial.print(now.month(), DEC);
//  Serial.print('/');
//  Serial.print(now.day(), DEC);
//  Serial.print(" ");
//  Serial.print(now.hour(), DEC);
//  Serial.print(':');
//  Serial.print(now.minute(), DEC);
//  Serial.print(':');
//  Serial.print(now.second(), DEC);
//  Serial.println();
  delay(3000);

  // blinkRed(2, 50);

  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
  

  readRegister(0x21); //read register to reset high-pass filter 
  readRegister(0x26); //read register to set reference acceleration
  readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;
  attachInterrupt(digitalPinToInterrupt(6), wakeUpPin6, LOW);
  LowPower.standby();
  // Serial.println("I am awake!");
  detachInterrupt(6);
  // Serial.println("Detached interrupt from pin 6");
 

  init_ACC();
  attachInterrupt(digitalPinToInterrupt(INT_PIN), LISinterrupt, HIGH);
  readRegister(0x21); //read register to reset high-pass filter 
  readRegister(0x26); //read register to set reference acceleration
  readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;
  LowPower.idle(IDLE_2);
  // Serial.println("Woke up");
  // Serial.println("Attached interrupt to LIS3DH INT pin");
}



void LISinterrupt()
{
  detachInterrupt(INT_PIN);
  interruptFlag = 1;
}

/*-----------------------------------------------------------------------------*/
float getBatteryVoltage() {
  float measuredvbat = analogRead(VBATPIN);

  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  return measuredvbat;

}

/*-----------------------------------------------------------------------------*/
// pin 6 interrupt handler
void wakeUpPin6(void)
{
  // doesn't do anything
}

/*-----------------------------------------------------------------------------*/
// blinks the red LED connected to pin 13 x times at y interval
void blinkRed(unsigned int x, unsigned int speed_ms)
{
  for (unsigned int i = 0; i < x; i++)
  {
    digitalWrite(RED_LED, HIGH);
    delay(speed_ms);
    digitalWrite(RED_LED, LOW);
    delay(speed_ms);
  }
  return;
}

/*-----------------------------------------------------------------------------*/
// blinks the blue LED connected to pin 13 x times at y interval
void blinkBlue(unsigned int x, unsigned int speed_ms)
{
  for (unsigned int i = 0; i < x; i++)
  {
    digitalWrite(BLUE_LED, HIGH);
    delay(speed_ms);
    digitalWrite(BLUE_LED, LOW);
    delay(speed_ms);
  }
  return;
}

/*-----------------------------------------------------------------------------*/
void loop() {

  if(interruptFlag == 1)
  {
    digitalWrite(RED_LED, HIGH);

    if (digitalRead(BUTTON_5) == LOW)
    {
      delay(20);
      // start button timer
      volatile unsigned long buttonTime = millis();
      // spin around here until button is released
      while (digitalRead(BUTTON_5) == LOW);
      if ((millis() - buttonTime) > LONG_CLICK_MS)
      {
        // turn off red LED
        digitalWrite(RED_LED, LOW); 
        // blink the blue LED to indicate going to off mode
        blinkBlue(5, 50);
        // unset flag
        interruptFlag = 0;
        // un-latch LIS3DH
        init_ACC();
        attachInterrupt(digitalPinToInterrupt(INT_PIN), LISinterrupt, HIGH);
        readRegister(0x21); //read register to reset high-pass filter 
        readRegister(0x26); //read register to set reference acceleration
        readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;
        // go to sleep
        attachInterrupt(digitalPinToInterrupt(6), wakeUpPin6, LOW);
        LowPower.standby();
        // Serial.println("I am awake!");
        detachInterrupt(6);
      }
    }

    DateTime now = rtc.now();
    
    filename[0] = '0' + now.day() / 10;
    filename[1] = '0' + now.day() % 10;
    filename[2] = '0' + now.hour() / 10;
    filename[3] = '0' + now.hour() % 10;
    filename[4] = '0' + now.minute() / 10;
    filename[5] = '0' + now.minute() % 10;
    filename[6] = '0' + now.second() / 10;
    filename[7] = '0' + now.second() % 10;
    filename[8] = '.';
    filename[9] = 'c';
    filename[10] = 's';
    filename[11] = 'v';
    filename[12] = '\0';
    
    // open the file
    logfile = SD.open(filename, FILE_WRITE);
    if ( ! logfile )
    {
      // Serial.print("Could not create ");
      // Serial.println(filename);
      while(1)
      {
        blinkRed(1, 100);
        blinkBlue(1, 100);
      }
    }
//    Serial.print("Writing to: ");
//    Serial.println(filename);
//    
//    Serial.print("  \tinterrupt: ");
//    Serial.print(reading++); Serial.print(",  ");
//    Serial.print(readRegister(0x21)); Serial.print(",  "); //read register to reset high-pass filter 
//    Serial.print(readRegister(0x26)); Serial.print(",  "); //read register to set reference acceleration
//    Serial.print(readRegister(LIS3DH_REG_INT1SRC)); Serial.print(",  "); //Read INT1_SRC to de-latch;
//    Serial.println();

    readRegister(0x21); //read register to reset high-pass filter 
    readRegister(0x26); //read register to set reference acceleration
    readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;
    
    // get some data
    for (int n = 0; n < SAMPLES_TO_CAPTURE; n++)
    {
      lis.read();
      chx = lis.x;
      chy = lis.y;
      chz = lis.z;
      // Serial.print(chx); Serial.print(','); Serial.print(chy); Serial.print(','); Serial.println(chz);
      logfile.print(chx); logfile.print(','); logfile.print(chy); logfile.print(','); logfile.println(chz);
    }
    logfile.flush();
    logfile.close();

    digitalWrite(RED_LED, LOW); 
    
    interruptFlag = 0;
    init_ACC();
    attachInterrupt(digitalPinToInterrupt(INT_PIN), LISinterrupt, HIGH);
    readRegister(0x21); //read register to reset high-pass filter 
    readRegister(0x26); //read register to set reference acceleration
    readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;
    LowPower.idle(IDLE_2);
  }
 
  delay(50); 
}
