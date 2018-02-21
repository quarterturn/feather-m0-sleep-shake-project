# feather-m0-sleep-shake-project
An attempt to make a working, low-power shake recorder.

Hardware:
Adafruit Feather M0 Adalogger
Adafruit DS3231 Featherwing
Adafruit Featherwing protoboard
Adafruit LIS3DH breakout
small buttons
blue LED and 220R resistor

The idea is to make a movement logger that does the following:
- uses sleep mode as "off"
- uses the LIS3DH assignable interrupt to wake up the MCU when an event occurs
- record data to the SD card using the date/time as a filename

I present here the non-working and the working versions. The non-working version is a state machine where main() is nothing more than a switch/case handing differnt states. This is nice and neat and DOES NOT WORK with lowPower.h - it seems to get condused about where it is in the stack if it is inside a function. The working version is a mess which throws all the code into main().

What I see going wrong is the in the state machine version is the 'record' wake interrupt fires over and over, despite the LIS3DH INT pin going low and despite detatching the interrupt while recording takes place. Who knows? Do you? Life it too short.
