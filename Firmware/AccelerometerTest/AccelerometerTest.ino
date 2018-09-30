#include <SparkFun_MMA8452Q.h>

#define BIN_DAY_LED 14
#define STATUS_LED 12

// Accelerometer INT1 is connected to GPIO13.
// Note: INT2 is connected to reset on V1.0 boards
// Do not enable INT2!!!!
#define ACCEL_INT 13

MMA8452Q accel;

// 0: Vertical
// 1: On Side
// 2: On front
// 3: On back
int orientation = 0;
int lastOrientation = 0;
float xAcceleration;
float yAcceleration;
float zAcceleration;
bool hasAccelerationMeasurement;
bool lidOpened = false;
bool lastLidOpened = false;
byte portraitLandscape;

void setup() {
  pinMode(ACCEL_INT, INPUT_PULLUP);

  pinMode(BIN_DAY_LED, OUTPUT);
  digitalWrite(BIN_DAY_LED, HIGH);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  // INT2 is routed to reset.
  // Reset is active low.
  
  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup Accelerometer");
  if (!accel.init(SCALE_8G, ODR_800)) {
    Serial.println("Accelerometer failed");
  }

  // Accelerometer needs to be in standby mode
  // to be configured.
  accel.standby();

  // 0.063 g/LSB for threshold
  // 30 is enough for detecting strong movement
  // or the tap as the bin lid is closed.
  // Perhaps we could use a Z tap detect....
  byte threshold = 10;
  Serial.print("Motion Threshold: ");
  Serial.println(threshold * 0.063, DEC);

  // Z Axis only for lid opening.
  // X & Y useful for falling over/movement detection
  // however this may be noisy (i.e. in the wind)
  // and our periodic wait / RTC update 
  // should check this.
  // No latching.
  accel.setupMotion(false, true, false, false, threshold);

  // X/Y disabled
  // Tap detect in Z axis only
  //accel.setupTap(0x80, 0x80, 4);

  // Setup interrpt so that...
  // Source = freefall/motion (0b 0000 0100)
  //byte source = 0x04; // Motion
  //byte source = 0x08; // Pulse
  //byte source = 0x10; // Landscape/Portrate
  byte source = 0x14; // Orientation, pulse 
  
  // Pin for any interrupt is INT1 (0b 1011 1101)
  // DO NOT USE INT2 as it is wired to reset
  // and interrupts are not cleared until read!
  byte pin = 0xBD;
  
  // Interrupt doesn't wake the system.
  bool wake = false;
  
  // interrupt is active low
  bool activeHigh = false;
  
  // open drain to allow multiple sources
  // because reset pin is INT2 we need to use open 
  // drain rather than push/pull.
  bool openDrain = true;
  
  accel.setupInterrupts(source, pin, wake, activeHigh, openDrain);

  delay(10);

  // Now the accelerometer has been configured
  // we can run it.
  accel.active();

  delay(100);

  digitalWrite(BIN_DAY_LED, LOW);
  digitalWrite(STATUS_LED, LOW);

  Serial.println("Setup complete..");
}

void loop() {
  if (measureAcceleration()) {
    printCalculatedAccels(); 
    printInterrupt();
    printMotion();
    printTap();
    Serial.println();
  } else {
    Serial.println("Failed to get accelerometer readings");
  }
 
  delay(500);
}

void printInterrupt() {
  
  bool interrupt1Input = !digitalRead(ACCEL_INT);
  digitalWrite(STATUS_LED, interrupt1Input);
  
  if (interrupt1Input) {
    Serial.print("\t INT1");
  }

  byte interruptSource = accel.readInterruptSource();
  if (interruptSource > 0) {
    Serial.print("\t Interrupt Source: 0x");
    Serial.print(interruptSource , HEX);

    if (interruptSource  & 0x04 == 0x04) {
      Serial.print("t\ Motion");
    }

    if (interruptSource  & 0x08 == 0x08) {
      Serial.print("t\ Tap");
    }

    if (interruptSource  & 0x10 == 0x10) {
      Serial.print("t\ Orient");
    }
  }
}

void printMotion() {
  byte motion = accel.readMotion();
  if (motion > 0) {
    digitalWrite(BIN_DAY_LED, HIGH);
    
    Serial.print("\tMotion: 0x");
    Serial.print(motion, HEX);

    // Z 0b 00 11 0000
    if (motion & 0x20) {
      if (motion & 0x10) {
        Serial.print("-");
      }
      Serial.print("Z ");
    }

    // Y 0b 0000 11 00
    if (motion & 0x08) {
      if (motion & 0x04) {
        Serial.print("-");
      }
      Serial.print("Y ");
    }

    // X 0b 0000 00 11
    if (motion & 0x02) {
      if (motion & 0x01) {
        Serial.print("-");
      }
      Serial.print("X ");
    }
    
  } else {
    digitalWrite(BIN_DAY_LED, LOW);
  }
}

void printTap() {
  byte tap = accel.readTap();
  if (tap > 0) {
    Serial.print("\t Taps: 0x");
    Serial.print(tap, HEX);
    Serial.print("\t");

    if (tap & 0x08) {
      Serial.print("\t Double ");
    }

    if (tap & 0x40) {
      if (tap & 0x04) {
        Serial.print("-");
      }
      Serial.print("Z ");
    }

    if (tap & 0x20) {
      if (tap & 0x02) {
        Serial.print("-");
      }
      Serial.print("Y ");
    }

    if (tap & 0x10) {
      if (tap & 0x01) {
        Serial.print("-");
      }
      Serial.print("X ");
    }    
  }
}

bool measureAcceleration() {

  hasAccelerationMeasurement = accel.available();
  
  if (hasAccelerationMeasurement) { 
    accel.read(); 

    // floats cx, cy, and cz will store the calculated  
    //   acceleration from those 12-bit values. These variables  
    //   are in units of g's. 
    xAcceleration = accel.cx;
    yAcceleration = accel.cy;
    zAcceleration = accel.cz;

    portraitLandscape = accel.readRawPL();
    
  } 

  return hasAccelerationMeasurement;
}

// Compute the orientation of the bin using the accelerometer
// 0: Vertical
// 1: On Side
// 2: On front
// 3: On back
int computeBinOrientation() {
  
  // Y acceleration may be because the lid is opened, or because the
  // bin has falled on its front with the lid closed/partially open
  // expect gravity to mainly close the lid, however the wind may have 
  // blown it back...
  // lid is open (vertical).
  // Lid vertical (open) ~ 1G - ignore here.
  // Lid all the way over ~ -1G
  // Z mat still be registering > 0.5 at this time.
  // 
  if (yAcceleration < -0.8) {
    return 2;
  }

  // If X axis > 0.5G then it's probably 45° on the X axis so 
  // assume it's on its side.
  if (xAcceleration > 0.8 || xAcceleration < -0.8) {
    return 1;
  }
    
  // If the bin is at-least 80% of gravity in the Z axis assume it's vertical.  
  if (zAcceleration < -0.8) {
    // Vertical
    return 0;
  }

  // If the zAcceleration is +ve then the lid is either open 
  // or the bin has fallen on its back and the lid is flat on the floow.
  if (zAcceleration > 0.5 ) {
    return 3;
  }

  if (yAcceleration > 0.5) {
    lidOpened = true;
    // Return bin OK, but assume the lid is open.
    return 0;
  }

  // Error, unknown condition
  return 9;
}

// This function demonstrates how to use the accel.cx, accel.cy, 
// and accel.cz variables. 
void printCalculatedAccels() 
{  
    Serial.print("\t X: "); 
    Serial.print(xAcceleration, 3); 
    Serial.print("\t Y: "); 
    Serial.print(yAcceleration, 3); 
    Serial.print("\t Z: "); 
    Serial.print(zAcceleration, 3); 
    Serial.print("\t Orientation: ");
    Serial.print(portraitLandscape, HEX);    
} 

