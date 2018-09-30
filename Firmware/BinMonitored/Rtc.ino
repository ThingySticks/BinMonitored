// BinMonitored doesn't have a real RTC
// it wakes approximatly every minute 
// so we're faking a RTC by incrementing
// time by a minute.

// Date/time offset into the EEPROM
int rtcEepromOffset = 100;

// Determine if the bin status should be checked.
// i.e. orientation, fullness, temperature etc.
bool shouldCheckBin() {
  // Debug
  return true;

  // Every ...
  return minute % 10 == 0;
}

// TODO: Determine publish interval.
bool shouldUploadStatus() {
  // Upload every time.
  //return true;

  // Upload every time the bin is checked.
  return shouldCheckBin();
  
  // publish the bin measurements at 9pm daily.
  //return hour == 21 && minute == 0;

  // publish every hour.
  //return minute == 30;
}

void readRtc() {

  day =  readEEPROM(disk1, rtcEepromOffset);
 
  if (day == 255 || FORCE_EEPROM_RESET) {
    // Not set, or faulty EEPROM so fake it...
    Serial.println("*** RTC not set in EEPROM - Using default date ***");
    day = BIN_DAY_OFFSET;
    hour = 21; // 9pm to aligh with default daily transmit time.
    minute = 00;
    second = 12;
    printDate();
    return;
  }
  
  // Hour is stored in +1 +(0..31) offset from 
  // the day.
  hour =  readEEPROM(disk1, rtcEepromOffset+1 + day);
  if (hour > 23) {
    hour = 0;
  }

  // minute and second stored side by side.
  // +32 +hour
  minute =  readEEPROM(disk1, rtcEepromOffset + 32 + (hour * 2));
  if (minute > 59) {
    minute = 0;
  }

  // Second doesn't ever change....... but.
  second =  readEEPROM(disk1, rtcEepromOffset + 32 + (hour * 2) + 1);
  if (second > 59) {
    second = 0;
  }

  printDate();
}

void printDate() {
  Serial.print("Date:");
  Serial.print(day);
  Serial.print(".");
  Serial.print(hour);
  Serial.print(":");
  Serial.print(minute);
  Serial.print(":");
  Serial.print(second);
  Serial.println();
}

// 
// Assumes a wake interval of 1 minute
void updateRtc() {
  
  minute++;
  second = 0;

  if (minute>59) {
    minute = 0;
    hour++;

    if (hour > 23) {
      hour = 0;
      day++;

      // Somewhat of a dirty hack to save make
      // day month cycles easy. day is reset to 0
      // on bin collection day.
      // So if day == binCollectionInterval it's the day
      // before bin collection and hence "bin day" should be lit to 
      // indicate the bin needs to be put out today/tomorrow.
      // on bin day, day is reset to 0.
      // hopefully this will make strange (but regular) bin collection 
      // intervals easy to implement...
      // ............. (tumble weed) .........
      if (day > binCollectionInterval) {
        day = 0;
      }
    }
  }
}

bool isBinDay() {
  // actual bin day.
  if (day == 0) {
    return true;
  }

  // It's the day before bin day, 
  // so time to show the bin day indicator to put the bin out 
  // for the next day.
  if (day >= binCollectionInterval) {
    return true;
  }

  return false;
}

void writeRtc() {
  // Store RTC value in EEPROM.
  // To save writing to the same locations
  writeEEPROM(disk1, rtcEepromOffset, day);
  
  // Write the hour into +1 +(0..31) offset.
  // offset 1..32
  writeEEPROM(disk1, rtcEepromOffset +1 + day, hour);

  // 32 bytes offset +hour
  // minute and second stored side by side.
  writeEEPROM(disk1, rtcEepromOffset +32 + (hour * 2), minute);
  writeEEPROM(disk1, rtcEepromOffset +32 + (hour * 2) + 1, second);
  
  // Memory Map:
  // Offset :: Function
  // 0 :: Day
  // 1 :: Hour
  // 2 :: Minute
  // 3 :: Second (not used).

  // Using static location with 1 minute updates
  // and no checking for same value 
  // writing to EEPROM
  // update every minute.
  // 60 * 24 = 1440 writes per day.
  // = 525,600 per year.
  // approx life = 100,000
  // 70 days!
  // Out of spec.
}


// Original from http://www.hobbytronics.co.uk/arduino-external-eeprom

void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{ 
  // Don't write if the location already holds the correct value.
  byte existingValue = readEEPROM(deviceaddress, eeaddress);
  if (existingValue == data) {
    return;
  }
  
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}
 
byte readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress, 1);
 
  if (Wire.available()) {
    rdata = Wire.read();
  } else {
    Serial.println("EEPROM FAULT!!!");
  }
 
  return rdata;
}



