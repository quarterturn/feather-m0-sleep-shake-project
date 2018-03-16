/***************************************************
  shake logger
  records movements over a certain threshold
  for the purpose of assisting with the diagnosis of periodic limb movement disorder

  designed for Adafruit Feather 32u4 Adalogger
  also depends on:
  ds3231 featherwing
  LIS3DH accelerometer mounted on a featherwing protoboard; int tied to pin 11
  2 buittons on pins 5 & 6
  an extra LED on pin 9

 ****************************************************/

// accel sensor stuff
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// SD card
#include <SPI.h>
#include <SD.h>
#define cardSelect 4

// power management
#include <avr/sleep.h>


// RTC
#include "RTClib.h"
// clock object
RTC_DS3231 rtc;


// Bounce button library
// from here git clone https://github.com/thomasfredericks/Bounce-Arduino-Wiring.git
#include <Bounce2.h>

// buttons and LEDs
#define BLUE_LED 10
#define BUTTON_5 5
#define BUTTON_0 0
#define RED_LED 13

// button debounce time
#define DEBOUNCE_PIN_MS 20
// button long click time
#define LONG_CLICK_MS 2000
// button retrigger time
#define BUTTON_RETRIGGER_MS 200

// how long to wait before going back to sleep
// one minute
#define SLEEP_TIMEOUT 30000

// how long to record data
// one minute
#define RECORD_TIMEOUT 60000

// how many samples to capture
// 100 Hz * 60 seconds
#define SAMPLES_TO_CAPTURE 6000

// battery voltage pin
// this is button "A" on the feather OLED, so do not use it for interrupts
#define VBATPIN A7

// pin to connect to INT pin on LIS3DH
#define INT_PIN 1

int LIS3DH_ADDR = 0x18; // change this to 0x19 for alternative i2c address

// track back to sleep timeout
unsigned long sleepTimeout = 0;
unsigned long buttonTime = 0;

// state names
#define STATE_OFF 0
#define STATE_INIT 1
#define STATE_ON_WAIT 2
#define STATE_ON_READY 3
#define STATE_RECORD_SLEEP 4
#define STATE_RECORD 5

// track the state we are in
// states
// 0 off
// 1 init
// 2 on-wait
// 3 on-ready
// 4 record-sleep
// 5 record
volatile int runState = 0;

// track time set
int isTimeSet = 0;

// I2C LIS3DH
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// file for the SD card
File logfile;


/*-----------------------------------------------------------------------------*/
void setup()
{

  // button pins
  pinMode(BUTTON_0, INPUT_PULLUP);
  pinMode(BUTTON_5, INPUT_PULLUP);

  // LED pins
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);

  // LIS3DH int pin
  pinMode(INT_PIN, INPUT);
  
  //Serial.begin(115200);
  //while(! //Serial);

  //Serial.println("\r\nMovement Logger Test");

  // get the current voltage of the battery from
  float battery = getBatteryVoltage();

  //Serial.print("Battery voltage: ");
  //Serial.println(battery);


  //Serial.println("LIS3DH configure... ");
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    //Serial.println("Couldnt start");
    while (1)
    {
      blinkRed(1, 50);
      blinkBlue(1, 50);
    }
  }
  //Serial.println("LIS3DH found!");

  // check if SD card is ready
  if (!SD.begin(cardSelect))
  {
    while (1)
    {
      blinkRed(1, 50);
      blinkBlue(1, 50);
    }
  }
  
  // set range to lowest (2G)
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G

  readRegister(0x21); //read register to reset high-pass filter 
  readRegister(0x26); //read register to set reference acceleration
  readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;

  //Serial.print("Range = "); //Serial.print(2 << lis.getRange());  
  //Serial.println("G");

  pinMode(INT_PIN, INPUT);
  
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // confirm the time
  // now object
  DateTime now = rtc.now();
  //Serial.print(now.year(), DEC);
  //Serial.print('/');
  //Serial.print(now.month(), DEC);
  //Serial.print('/');
  //Serial.print(now.day(), DEC);
  //Serial.print(" ");
  //Serial.print(now.hour(), DEC);
  //Serial.print(':');
  //Serial.print(now.minute(), DEC);
  //Serial.print(':');
  //Serial.print(now.second(), DEC);
  //Serial.println();
  delay(5000);

  //Serial.println("Going to STATE_OFF");
  
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  runState = STATE_OFF;
}

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
float getBatteryVoltage() {
  float measuredvbat = analogRead(VBATPIN);

  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  return measuredvbat;

}


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
// pin 0 interrupt handler
void wakeUpPin0(void)
{
  // doesn't do anything
}

/*-----------------------------------------------------------------------------*/
// pin 1 interrupt handler
void LISinterrupt()
{
  // doesn't do anything
}

/*-----------------------------------------------------------------------------*/
// go to sleep and wake on pin 0
// this happens after power up
void doPowerSleep(void)
{
  //Serial.println("doPowerSleep");
 
  readRegister(0x21); //read register to reset high-pass filter 
  readRegister(0x26); //read register to set reference acceleration
  readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  attachInterrupt(digitalPinToInterrupt(BUTTON_0), wakeUpPin0, LOW);
  sleep_mode();
  detachInterrupt(BUTTON_0);

  //Serial.println("I am awake!");
  //Serial.println("Detached interrupt from pin 0");
  
  runState = STATE_ON_WAIT;
}

/*-----------------------------------------------------------------------------*/
// wake up from pin 0
// test for long press on button 0
// if not long press, go back to sleep
// otherwise start timer waiting for button 0
// runs in STATE_ON_WAIT
void doOnWait(void)
{


  //digitalWrite(RED_LED, HIGH);

  delay(20);
  //digitalWrite(RED_LED, HIGH);
  runState = STATE_ON_READY;
  return;
  
//  buttonTime = (unsigned long)millis();
//  
//  // turn on the blue LED
//  digitalWrite(BLUE_LED, HIGH);
//  
//  while (((unsigned long)millis() - buttonTime) < LONG_CLICK_MS);
//  digitalWrite(RED_LED, HIGH);
//  
//  
//  //Serial.println("doOnWait");
//  delay(20);
//  // start button timer
//  buttonTime = (unsigned long)millis();  
//  // see if the button is held down long enough
//  while (digitalRead(BUTTON_0) == LOW)
//  {
//    // long enough click
//    if (((unsigned long)millis() - buttonTime) > LONG_CLICK_MS)
//    {
//      // go to sleep and wait for movement trigger
//      digitalWrite(BLUE_LED, LOW);
//      blinkRed(5, 50);
//      runState = STATE_RECORD_SLEEP;
//      return;
//    }
//  }
//  // otherwise go back to sleep
//  if (runState == STATE_ON_WAIT)
//  {
//    digitalWrite(RED_LED, LOW);
//    blinkBlue(5, 50);
//    // go to off state
//    runState = STATE_OFF;
//    return;
//  } 
//  return;
}

/*-----------------------------------------------------------------------------*/
// wait for button 5 to start recording
// otherwise go back to sleep
void doOnReady(void)
{
  //Serial.println("doOnReady");
  // turn on the blue LED
  digitalWrite(BLUE_LED, HIGH);
  // start sleep timer
  sleepTimeout = (unsigned long)millis();
  // wait for input until timeout
  while ((unsigned long)(millis() - sleepTimeout) < SLEEP_TIMEOUT)
  {
    if (digitalRead(BUTTON_5) == LOW)
    {
      // wait for bouncing to stop
      delay(20);
      // start button timer
      unsigned long buttonTime = millis();
      // spin around here until button is released
      while (digitalRead(BUTTON_5) == LOW)
      {
        if ((unsigned long)(millis() - buttonTime) > LONG_CLICK_MS)
        {
          // blink the blue LED to signal getting ready to sleep
          blinkBlue(5, 50);
          digitalWrite(RED_LED, LOW);
          // go to state RECOED_SLEEP
          runState = STATE_RECORD_SLEEP;
          return;
        }
      }
    }
  }
  // go back to sleep if nothing was pressed
  // fade the blue LED to show going back to sleep
  // fadeBlueLED();
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
  runState = STATE_OFF;
  return;
}

/*-----------------------------------------------------------------------------*/
// go to sleep
// wait for LIS3DH on 1
void doRecordSleep(void)
{
  //Serial.println("doRecordSleep");

  init_ACC();
  readRegister(0x21); //read register to reset high-pass filter 
  readRegister(0x26); //read register to set reference acceleration
  readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;
  readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  attachInterrupt(digitalPinToInterrupt(INT_PIN), LISinterrupt, HIGH);
  sleep_mode();
  detachInterrupt(INT_PIN);
  delay(50);
  //init_ACC();
  readRegister(0x21); //read register to reset high-pass filter 
  readRegister(0x21); //read register to set reference acceleration
  readRegister(LIS3DH_REG_INT1SRC); //Read INT1_SRC to de-latch;
  //Serial.println("Woke up to take reading");
  //Serial.println("Detatched interrupt from pin 1");
  
  runState = STATE_RECORD;
  return;
}

/*-----------------------------------------------------------------------------*/
// record
void doRecord(void)
{

  volatile int recordResult = 0;

  while (runState == STATE_RECORD)
  {

    //Serial.print("doRecord");
  
    // see if button 0 is being held down
    // this means the user wants to stop recording and shut down
    if (digitalRead(BUTTON_5) == LOW)
    {
      // wait for bouncing to stop
      delay(20);
      // start button timer
      buttonTime = millis();
      // spin around here until button is released
      while (digitalRead(BUTTON_5) == LOW);
      if ((unsigned long)(millis() - buttonTime) > LONG_CLICK_MS)
      {
        // blink the blue LED to indicate going to off mode
        blinkBlue(5, 50);
        // go to sleep
        runState = STATE_OFF;
      }
    }
    else
    {
      digitalWrite(RED_LED, HIGH);
      // record data to SD card
      recordResult = writeDataToSDCard();
      // test if successful
      if (recordResult == 1)
      {
        // go back to sleep and wait for next event
        runState = STATE_RECORD_SLEEP;

        digitalWrite(RED_LED, LOW);
      }
      // otherwise shut down
      else
      {
        runState = STATE_OFF;
        digitalWrite(RED_LED, LOW);
        // blink blue LED to indicate going to off mode due to problem
        // or the user held down button 5 to turn off
        blinkBlue(5, 50);
      }
    }
  }
}

/*-----------------------------------------------------------------------------*/
// write data to SD card
int writeDataToSDCard(void)
{

  int chx;
  int chy;
  int chz;
  int n;

  char filename[13];

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
    //Serial.print("Could not create ");
    //Serial.println(filename);
    blinkRed(20, 100);
    return 0;
  }
  //Serial.print("Writing to: ");
  //Serial.println(filename);

  // get the data
  for (n = 0; n < SAMPLES_TO_CAPTURE; n++)
  {
    lis.read();
    chx = lis.x;
    chy = lis.y;
    chz = lis.z;
    logfile.print(chx); logfile.print(','); logfile.print(chy); logfile.print(','); logfile.println(chz);

    if (digitalRead(BUTTON_5) == LOW)
    {
      // wait for bouncing to stop
      delay(20);
      // start button timer
      buttonTime = millis();
      // spin around here until button is released
      while (digitalRead(BUTTON_5) == LOW);
      if ((unsigned long)(millis() - buttonTime) > LONG_CLICK_MS)
      {
        // clean up and close the file
        logfile.flush();
        logfile.close();
        // return 2 so next state will be STATE_POWER_OFF
        return 2;
      }
    }
  }
  logfile.flush();
  logfile.close();
  return 1;

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
// fades out the blue LED
// fade out from max to min in increments of 1 points:
void fadeBlueLED(void)
{
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 1) {
    // sets the value (range from 0 to 255):
    analogWrite(BLUE_LED, fadeValue);
    // wait for 5 milliseconds to see the dimming effect
    delay(5);
  }
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
void loop()
{
  // nothing much happens here
  // baed on the machine state we call different functions
  switch (runState)
  {
    case STATE_OFF:
      doPowerSleep();
      break;
    case STATE_ON_WAIT:
      doOnWait();
      break;
    case STATE_ON_READY:
      doOnReady();
      break;
    case STATE_RECORD_SLEEP:
      doRecordSleep();
      break;
    case STATE_RECORD:
      doRecord();
      break;
    default:
      break;
  }
}
