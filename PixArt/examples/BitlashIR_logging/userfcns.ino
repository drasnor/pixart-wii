// Bitlash user functions. Read the bitlash docs to make sense of these.
// I used the standard Arduino output functions instead of the bitlash ones. -drasnor

numvar set_p0() {
  p0 = getarg(1);
  IRcam.setSensitivity(p0,p1,p2,p3);
  if(p0 == IRcam.getSensitivity(0)) {
    Serial.print("Sensitivity p0 set: ");
    Serial.println(p0,HEX);
  }
  return true;
}

numvar set_p1() {
  p1 = getarg(1);
  IRcam.setSensitivity(p0,p1,p2,p3);
  if(p1 == IRcam.getSensitivity(1)) {
    Serial.print("Sensitivity p1 set: ");
    Serial.println(p1,HEX);
  }
  return true;
}

numvar set_p2() {
  p2 = getarg(1);
  IRcam.setSensitivity(p0,p1,p2,p3);
  if(p2 == IRcam.getSensitivity(2)) {
    Serial.print("Sensitivity p2 set: ");
    Serial.println(p2,HEX);
  }
  return true;
}

numvar set_p3() {
  p3 = getarg(1);
  IRcam.setSensitivity(p0,p1,p2,p3);
  if(p3 == IRcam.getSensitivity(3)) {
    Serial.print("Sensitivity p3 set: ");
    Serial.println(p3,HEX);
  }
  return true;
}

numvar printCurrentSensitivity() {
  int* px = IRcam.getSensitivity();
  Serial.println("Current Settings:");
  Serial.println("-----------------");
  Serial.print("p0: 0x"); Serial.println(px[0],HEX);
  Serial.print("p1: 0x"); Serial.println(px[1],HEX);
  Serial.print("p2: 0x"); Serial.println(px[2],HEX);
  Serial.print("p3: 0x"); Serial.println(px[3],HEX);
  return true;
}

void Sample() {
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
