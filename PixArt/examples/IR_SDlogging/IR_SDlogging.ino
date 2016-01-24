/*
  Wii Remote IR Sensor Flight Test Sketch
  2012 Andrew Beckett - drasnor@tamu.edu

  Logs Wii IR sensor data upon object detection. Synchronizes with external video using audible chirps.
  Uses a servo to tilt camera to correct pitch. Requires PixArt sensor on I2C bus, SD card adapter on
  SPI, a mid-high impedance speaker on pin 6, and a servo on pin 7.

  Any tracked blob will generate a serial message of "BLOB# Detected: X:xxxx Y:yyyy" at roughly 2 Hz.
  With the plastic filter off, the detector can track a lit candle or match at arm's length. Upon blob
  detection, this program takes 15 samples with a 33 ms interval between samples and log the result to
  the SD card. As each sample is taken, the speaker chirps a 15 ms note which is audible in the camera
  audio/visual record. 15 samples are taken so that one of the samples falls on an I-frame in the h.264
  video encoding used on the camera. The servo points the camera at a desired tilt angle.

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
#include <Servo.h>
#include <Wire.h>
#include "pitches.h"

#define baud_rate 57600 // serial port baud rate
#define servoPin 7	// servo pin
#define servoMax 2100	// servo max command in microseconds
#define servoMin 900	// servo min command in microseconds

const int spkrPin = 6; // pin for 32-ohm speaker
const int sdcsPin = 10; // pin for SD card CS signal
const int numSamples = 15; // take this many samples in a set
const int timeSamples = 33; // take a sample every this many ms
const unsigned long interval = 500; // poll interval in ms
const int servoMicroseconds = 900; // servo command
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

unsigned long previousMillis = 0; // last time switch triggered
unsigned long timeNow = 0; // current time event
byte mask; // active blobs bitmask. See PixArt driver library for details.

// Boot-up melody:
int melody[] = {
  NOTE_E2,NOTE_G3,NOTE_C4,NOTE_D4,NOTE_E4,NOTE_D4,NOTE_C4,NOTE_D4,NOTE_E4,NOTE_C4,NOTE_C4};

int noteDurations[] = {
  8,8,8,8,8,8,8,8,4,4,2 };

PixArt IRcam; // IR sensor object.
Servo myServo; // Servo object
File data; // Data log file
void sample(); // sample function prototype

// Setup runs once at power-on
void setup() {
  Serial.begin(baud_rate); // Open serial port
  Serial.println("1: Serial port open");
  Wire.begin(); // Start I2C bus
  Serial.println("2: I2C bus initialized");
  IRcam.init(); // Initialize IR sensor
  Serial.println("3: IR sensor initialized");
  IRcam.setSensitivity(p0,p1,p2,p3); // Set IR sensor sensitivity settings
  Serial.println("4: IR sensor sensitivity set");
  myServo.attach(servoPin,servoMin,servoMax); // Attach servo
  Serial.println("5: Servo object attached");
  myServo.writeMicroseconds(servoMicroseconds); // Set servo command
  Serial.println("6: Camera tilt set");
  pinMode(sdcsPin, OUTPUT); // Set the SD CS pin as output
  Serial.println("7: SD SPI chip select set");
  if(!SD.begin(sdcsPin)) {
    Serial.println("SD oject attach failed!");
    return;
  }
  else {
    Serial.println("8: SD object attached");
  }
  data = SD.open("samples.bin",FILE_WRITE); // open the data file for writing  
  Serial.println("9: SD card file open");
  data.write((const uint8_t*)(&sessionMarker),sizeof(sessionMarker));
  data.flush();
  Serial.println("10: Sample session started");
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
  Serial.println("Initialization complete");
}

// Loop is called in a while(1) loop
void loop() {
  timeNow = millis();
  // Poll
  if(timeNow - previousMillis > interval) {
    previousMillis = timeNow;
    
    mask = IRcam.read();

    // Debugging Output
    if(mask & 0x01) {
      Serial.print("BLOB0 detected. X:");
      Serial.print(IRcam.getBlobX(0));
      Serial.print(" Y:");
      Serial.println(IRcam.getBlobY(0));
    }
    if(mask & 0x02) {
      Serial.print("BLOB1 detected. X:");
      Serial.print(IRcam.getBlobX(1));
      Serial.print(" Y:");
      Serial.println(IRcam.getBlobY(1));
    }
    if(mask & 0x04) {
      Serial.print("BLOB2 detected. X:");
      Serial.print(IRcam.getBlobX(2));
      Serial.print(" Y:");
      Serial.println(IRcam.getBlobY(2));
    }
    if(mask & 0x08) {
      Serial.print("BLOB3 detected. X:");
      Serial.print(IRcam.getBlobX(3));
      Serial.print(" Y:");
      Serial.println(IRcam.getBlobY(3));
    }
  }
  
  // Sample if blob detected
  if(mask) {
    sample();
  }
}

void sample() {
  unsigned long now; // current millis()
  unsigned long next; // timing for next sample
  unsigned short pause; // time to wait between samples loop
  int temp; // temporary variable to hold XY data
  
  // beginning of sample set marker
  data.write((const uint8_t*)(&runMarker),sizeof(runMarker));
  // sample take & log loop
  for(int i = 0; i < numSamples; i++){
    mask = IRcam.read(); // refresh the sensor data
    now = millis(); // timestamp the sensor data
    next = now + timeSamples;
    tone(spkrPin,NOTE_E4,15); // 15 ms sound sync chirp
    
    // data writes
    data.write((const uint8_t*)(&now),sizeof(now)); // typecast long to byte array, write timestamp
    data.write(mask); // write bitmask
    if (mask & 0x01) {
      temp = IRcam.getBlobX(0);
      data.write((const uint8_t*)(&temp),sizeof(temp));
      temp = IRcam.getBlobY(0);
      data.write((const uint8_t*)(&temp),sizeof(temp));
    }
    if (mask & 0x02) {
      temp = IRcam.getBlobX(1);
      data.write((const uint8_t*)(&temp),sizeof(temp));
      temp = IRcam.getBlobY(1);
      data.write((const uint8_t*)(&temp),sizeof(temp));
    }
    if (mask & 0x04) {
      temp = IRcam.getBlobX(2);
      data.write((const uint8_t*)(&temp),sizeof(temp));
      temp = IRcam.getBlobY(2);
      data.write((const uint8_t*)(&temp),sizeof(temp));
    }
    if (mask & 0x08) {
      temp = IRcam.getBlobX(3);
      data.write((const uint8_t*)(&temp),sizeof(temp));
      temp = IRcam.getBlobY(3);
      data.write((const uint8_t*)(&temp),sizeof(temp));
    }
    
    // figure out timing for next sample
    now = millis();
    pause = next - now;
    delay(pause);
  }
  // end of sample set marker
  data.write((const uint8_t*)(&endMarker),sizeof(endMarker));
  data.flush(); // flush the write buffer
}
