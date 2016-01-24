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
