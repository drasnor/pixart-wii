/*
  PixArt.h - - Vision library for Arduino using the Wii PixArt IR camera
  - Version 0.2
  
  Original work by Kako:  http://www.kako.com/neta/2007-001/2007-001.html
                          http://www.kako.com/neta/2008-009/2008-009.html
  Derived from PVision library by Stephen Hobley:
    http://www.stephenhobley.com/blog/2009/03/01/pixartwiimote-sensor-library-for-arduino/
  PixArt library (0.1) by Drew Beckett
  v0.2 includes sensitivity setting delay optimizations by Tim Woodbury.

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

  Last tested with Arduino 1.0.1
*/

#ifndef PixArt_h
#define PixArt_h

#include <Arduino.h>
#include <Wire.h>

// I2C address of PixArt Wii sensor
#define IRsensorAddress 0xB0

// Structure to hold blob data
struct Blob {
  int X; // X location, max val = 1023
  int Y; // Y location, max val = 767
};

class PixArt {

public:
  PixArt(); // default constructor

  void init();  // initializes with default (Kako's) sensitivity settings
  void setSensitivity(int p0, int p1, int p2, int p3); // initializes with specified sensitivity settings
  void setSensitivity(int px[]); // initializes with specified sensitivity array
  int* getSensitivity(); // returns sensitivity array
  int getSensitivity(int px); // returns specified sensitivity
  int getBlobX(int blobID); // returns blob x location
  int getBlobY(int blobID); // returns blob y location
  
  byte read();  // update the blobs and return active blob bitmask
  byte getActiveBlobs(); // returns the active blob bitmask
  
private:
  // per object data
  Blob IRblob[4]; // Array of Blob structs to hold blob data
  int sensitivity[4]; // Array of sensitivity parameters
  int IRslaveAddress; // TWI-compatible sensor address
  byte data_buf[16]; // buffer to hold I2C/TWI packets
  byte activeBlobs; // active blob list
  int i; // counter var
  int s; // temp var

  void unpack(); // unpack data_buf packets and return active blob bitmask
  void write(byte d1); // Complete transaction for single-byte write
  void write2(byte d1, byte d2); // Complete transaction for two-byte write
  void setSensitivity(); // 
};

#endif
