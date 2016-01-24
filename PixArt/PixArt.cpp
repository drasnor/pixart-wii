/*
  PixArt.cpp - - Vision library for Arduino using the Wii PixArt IR camera
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

*/

/******************************************************************************
* Includes
******************************************************************************/
#include "PixArt.h"
#include <Wire.h>
#include <Arduino.h>

/******************************************************************************
* Private methods
******************************************************************************/
void PixArt::unpack() {
  activeBlobs = 0; // flush active blob list
  
  // Unpack the packet
  IRblob[0].X = data_buf[1];
  IRblob[0].Y = data_buf[2];
  s = data_buf[3];
  IRblob[0].X += (s & 0x30) <<4;
  IRblob[0].Y += (s & 0xC0) <<2;
  activeBlobs |= (IRblob[0].Y < 768)? 0x01 : 0; // mask 0b0001
	
  IRblob[1].X = data_buf[4];
  IRblob[1].Y = data_buf[5];
  s   = data_buf[6];
  IRblob[1].X += (s & 0x30) <<4;
  IRblob[1].Y += (s & 0xC0) <<2;
  activeBlobs |= (IRblob[1].Y < 768)? 0x02 : 0; // mask 0b0010
  
  IRblob[2].X = data_buf[7];
  IRblob[2].Y = data_buf[8];
  s   = data_buf[9];
  IRblob[2].X += (s & 0x30) <<4;
  IRblob[2].Y += (s & 0xC0) <<2;
  activeBlobs |= (IRblob[2].Y < 768)? 0x04 : 0; // mask 0b0100

  IRblob[3].X = data_buf[10];
  IRblob[3].Y = data_buf[11];
  s   = data_buf[12];
  IRblob[3].X += (s & 0x30) <<4;
  IRblob[3].Y += (s & 0xC0) <<2;
  activeBlobs |= (IRblob[3].Y < 768)? 0x08 : 0; // mask 0b1000
}

void PixArt::setSensitivity() {
  write2(0x30,0x01); delay(1);
  
  Wire.beginTransmission(IRslaveAddress);
  Wire.write(0x00);
  Wire.write(0x02); Wire.write(0x00); Wire.write(0x00); Wire.write(0x71);
  Wire.write(0x01); Wire.write(0x00); Wire.write(sensitivity[0]);
  Wire.endTransmission(); delay(1);
  
  Wire.beginTransmission(IRslaveAddress);
  Wire.write(0x07); Wire.write(0x00); Wire.write(sensitivity[1]);
  Wire.endTransmission(); delay(1);
  
  Wire.beginTransmission(IRslaveAddress);
  Wire.write(0x1A); Wire.write(sensitivity[2]); Wire.write(sensitivity[3]);
  Wire.endTransmission(); delay(1);
  
  write2(0x33,0x03); delay(1); // Mode set (0x01 = Extended, 0x03 = Extended, 0x05 = Full)
  
  write2(0x30,0x08); delay(6);
}

// Function to send a single byte in a single transaction
void PixArt::write(byte d1) {
  Wire.beginTransmission(IRslaveAddress);
  Wire.write(d1);
  Wire.endTransmission();
}

// Function to send a pair of bytes in a single transaction
void PixArt::write2(byte d1, byte d2) {
  Wire.beginTransmission(IRslaveAddress);
  Wire.write(d1); Wire.write(d2);
  Wire.endTransmission();
}


/******************************************************************************
* Constructor
******************************************************************************/
PixArt::PixArt() {
  IRslaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI

  for(i=0;i<4;i++) {
    IRblob[i].X = 1023;
    IRblob[i].Y = 1023;
  }
}

/******************************************************************************
* Public methods
******************************************************************************/
// init the PixArt sensor with default (Kako's) sensitivity settings in Extended mode
void PixArt::init() {
  write2(0x30,0x01); //delay(1);
  write2(0x30,0x08); //delay(1);
  write2(0x06,0x90); //delay(1);
  write2(0x08,0xC0); //delay(1);
  write2(0x1A,0x40); //delay(1);
  write2(0x33,0x33); //delay(1); 
  write2(0x33,0x03); //delay(1);
}

// init the PixArt sensor with specified sensitivity settings in Extended mode
void PixArt::setSensitivity(int p0, int p1, int p2, int p3) {
  sensitivity[0] = p0;
  sensitivity[1] = p1;
  sensitivity[2] = p2;
  sensitivity[3] = p3;
  setSensitivity();
}

// init the PixArt sensor with specified sensitivity settings in Extended mode
void PixArt::setSensitivity(int px[]) {
  for(int i=0;i<4;i++) {sensitivity[i] = px[i]; }
  setSensitivity();
}

 // returns sensitivity array
int* PixArt::getSensitivity() {
  static int px[4];
  for(int i=0;i<4;i++) {px[i] = sensitivity[i]; }
  return px;
}

// returns specified sensitivity
int PixArt::getSensitivity(int px) {
  return sensitivity[px];
}

// returns blob x location
int PixArt::getBlobX(int blobID) {
  return IRblob[blobID].X;
}

// returns blob y location
int PixArt::getBlobY(int blobID) {
    return IRblob[blobID].Y;
}

byte PixArt::read() {
  //IR sensor read
  write(0x36);
  Wire.requestFrom(IRslaveAddress, 16); // Request the 2 byte heading (MSB comes first)

  // Null the buffer
  for(i=0;i<16;i++) {
    data_buf[i]=0;
  }
  i=0;

  // Receive the data
  while(Wire.available() && i < 16) { 
    data_buf[i] = Wire.read();
    i++;
  }

  unpack();
  return activeBlobs;
}

byte PixArt::getActiveBlobs() {
  // returns the active blob list
  return activeBlobs;
}
