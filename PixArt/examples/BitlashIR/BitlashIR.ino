/*
  Wii Remote IR Sensor Test Sketch with Bitlash
  2011 Andrew Beckett - drasnor@tamu.edu

  Assists manual calibration of PixArt IR sensor using Bitlash shell for user interaction. You
  need to already have Bitlash installed in your Arduino libraries directory to compile this sketch.
  Requires PixArt sensor on I2C bus.

  Implemented:
  pcs <cr>: print current settings, lists current settings in hexadecimal format
  p0(arg) <cr>: set sensitivity parameter p0 to arg, arg can be decimal or hexadecimal in 0xhh format.
  p1(arg) <cr>: set sensitivity parameter p1 to arg, arg can be decimal or hexadecimal in 0xhh format.
  p2(arg) <cr>: set sensitivity parameter p2 to arg, arg can be decimal or hexadecimal in 0xhh format.
  p3(arg) <cr>: set sensitivity parameter p3 to arg, arg can be decimal or hexadecimal in 0xhh format.

  Any tracked blob will generate a serial message of "BLOB# Detected: X:xxxx Y:yyyy" at roughly 4Hz.
  With the plastic filter off, the detector can track a lit candle or match at arm's length.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/ 

#include <PixArt.h>
#include <Wire.h>
#include <bitlash.h>

#define baud_rate 57600 // serial port baud rate
/* Sensitivity settings guide:
p0: MAXSIZE: Maximum blob size. Wii uses values from 0x62 to 0xc8
p1: GAIN: Sensor Gain. Smaller values = higher gain
p2: GAINLIMIT: Sensor Gain Limit. Must be less than GAIN for camera to function. No other effect?
p3: MINSIZE: Minimum blob size. Wii uses values from 3 to 5
*/

// Default settings:
int p0=0x72; int p1=0x20; int p2=0x1F; int p3=0x03; // Kako 1
//int p0=0xC8; int p1=0x36; int p2=0x35; int p3=0x03; // Kako 2
//int p0=0xAA; int p1=0x64; int p2=0x63; int p3=0x03; // Kako 3
//int p0=0x96; int p1=0xB4; int p2=0xB3; int p3=0x04; // Kako 4
//int p0=0x96; int p1=0xFE; int p2=0xFD; int p3=0x05; // Kako 5

const int ledPin = 13; // standard pin for Arduino built-in LED
//const int ledPin = 37; // pc0 APM1 Oilpan green LED 'A'
boolean ledState = false;
long previousMillis = 0;
long interval = 250;
int i;
int s;
byte mask; // active blobs bitmask. See PixArt driver library for details.

PixArt IRcam; // IR sensor object.

// Setup runs once at power-on
void setup() {
  Wire.begin();
  IRcam.init();
  initBitlash(baud_rate);
  pinMode(ledPin, OUTPUT); // Set the LED pin as output
  
  addBitlashFunction("p0", (bitlash_function) set_p0);
  addBitlashFunction("p1", (bitlash_function) set_p1);
  addBitlashFunction("p2", (bitlash_function) set_p2);
  addBitlashFunction("p3", (bitlash_function) set_p3);
  addBitlashFunction("pcs", (bitlash_function) printCurrentSensitivity);
  
  IRcam.setSensitivity(p0,p1,p2,p3);
}

// Loop is called in a while(1) loop
void loop() {
  runBitlash();
  
  // Toggle LED, sample
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    if (ledState) { digitalWrite(ledPin,HIGH); } else { digitalWrite(ledPin,LOW); }
    mask = IRcam.read();

	// Output
    if (mask & 0x01) {
      Serial.print("BLOB0 detected. X:");
      Serial.print(IRcam.getBlobX(0));
      Serial.print(" Y:");
      Serial.println(IRcam.getBlobY(0));
    }
    if (mask & 0x02) {
      Serial.print("BLOB1 detected. X:");
      Serial.print(IRcam.getBlobX(1));
      Serial.print(" Y:");
      Serial.println(IRcam.getBlobY(1));
    }
    if (mask & 0x04) {
      Serial.print("BLOB2 detected. X:");
      Serial.print(IRcam.getBlobX(2));
      Serial.print(" Y:");
      Serial.println(IRcam.getBlobY(2));
    }
    if (mask & 0x08) {
      Serial.print("BLOB3 detected. X:");
      Serial.print(IRcam.getBlobX(3));
      Serial.print(" Y:");
      Serial.println(IRcam.getBlobY(3));
    }
  }
}
