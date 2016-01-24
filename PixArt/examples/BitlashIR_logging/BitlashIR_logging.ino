/*
  Wii Remote IR Sensor Test Sketch with Bitlash and Logging
  2011 Andrew Beckett - drasnor@tamu.edu

  Assists manual calibration of PixArt IR sensor using Bitlash shell for user interaction. Synchronizes
  with external video using audible chirps.You
  need to already have Bitlash installed in your Arduino libraries directory to compile this sketch.
  Requires PixArt sensor on I2C bus, SD card adapter on SPI, and mid-high impedance speaker.
  This sketch is too large to fit on the ATMega328 found on standard Arduino hardware; you will
  need hardware based on the ATMega1280 or larger.

  Implemented:
  pcs <cr>: print current settings, lists current settings in hexadecimal format
  p0(arg) <cr>: set sensitivity parameter p0 to arg, arg can be decimal or hexadecimal in 0xhh format.
  p1(arg) <cr>: set sensitivity parameter p1 to arg, arg can be decimal or hexadecimal in 0xhh format.
  p2(arg) <cr>: set sensitivity parameter p2 to arg, arg can be decimal or hexadecimal in 0xhh format.
  p3(arg) <cr>: set sensitivity parameter p3 to arg, arg can be decimal or hexadecimal in 0xhh format.
  sample <cr>: take 15 samples at 33 ms intervals and log the results to SD card in binary format.

  Any tracked blob will generate a serial message of "BLOB# Detected: X:xxxx Y:yyyy" at roughly 2 Hz.
  With the plastic filter off, the detector can track a lit candle or match at arm's length. When the
  sample command is given, this program takes 15 samples with a 33 ms interval between samples and logs
  the result to the SD card. As each sample is taken, the speaker chirps a 15 ms note which is audible
  in the camera audio/visual record. 15 samples are taken so that one of the samples falls on an I-frame
  in the h.264 video encoding used on the camera.

  This library is for use with the Wii PixArt sensor connected to the Arduino
  I2C/TWI port. The PixArt sensor is discussed in detail on the Wiibrew wiki:
  http://wiibrew.org/wiki/Wiimote#IR_Camera. This library should only be used
  when the camera has been removed from the Wiimote and provided with support
  circuitry. See links above for more information.
  
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
#include <SD.h>
#include <Wire.h>
#include <bitlash.h>
#include "pitches.h"

#define baud_rate 57600 // serial port baud rate

//const int ledPin = 13; // standard pin for Arduino built-in LED
const int ledPin = 37; // pc0 APM1 Oilpan green LED 'A'
const int spkrPin = 8; // pin for 32-ohm speaker
const int sdcsPin = 4; // pin for SD card CS signal
const int numSamples = 15; // take this many samples in a set
const int timeSamples = 33; // take a sample every this many ms
const byte sessionMarker[3] = {0x7B, 0x98, 0xA0}; // beginning of session marker
const byte runMarker[3] = {0xA4, 0xCB, 0x80}; // beginning of sample run marker
const byte endMarker[3] = {0xDB, 0xBA, 0xCD}; // end of sample run marker

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

boolean ledState = false;
long previousMillis = 0;
long interval = 500; // poll interval in ms.
byte mask; // active blobs bitmask. See PixArt driver library for details.

// Boot-up melody:
int melody[] = {
  NOTE_E2,NOTE_G3,NOTE_C4,NOTE_D4,NOTE_E4,NOTE_D4,NOTE_C4,NOTE_D4,NOTE_E4,NOTE_C4,NOTE_C4};

int noteDurations[] = {
  8,8,8,8,8,8,8,8,4,4,2 };

PixArt IRcam; // IR sensor object.
File data; // Data log file.

// Setup runs once at power-on
void setup() {
  Wire.begin();
  IRcam.init();
  initBitlash(baud_rate);
  pinMode(ledPin, OUTPUT); // Set the LED pin as output
  pinMode(sdcsPin, OUTPUT); // Set the SD CS pin as output
  
  addBitlashFunction("p0", (bitlash_function) set_p0);
  addBitlashFunction("p1", (bitlash_function) set_p1);
  addBitlashFunction("p2", (bitlash_function) set_p2);
  addBitlashFunction("p3", (bitlash_function) set_p3);
  addBitlashFunction("pcs", (bitlash_function) printCurrentSensitivity);
  addBitlashFunction("sample", (bitlash_function) Sample);
  
  IRcam.setSensitivity(p0,p1,p2,p3);
  
  for (int thisNote = 0; thisNote < 11; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/noteDurations[thisNote];
    tone(spkrPin, melody[thisNote],noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(spkrPin);
  }
  runBitlash();
  if(!SD.begin(sdcsPin)) {
    Serial.println("SD driver initialization failed!");
    return;
  }
  // Start sample session
  data.write((const uint8_t*)(&sessionMarker),sizeof(sessionMarker));
  data.flush();
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
