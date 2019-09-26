#include "quaternionFilters.h"
#include "MPU9250.h"
#include "Wire.h"
#include "SPI.h"
#include "MIDIUSB.h"
#include "limits.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define NOTE_C5   48
#define NOTE_Db5  49
#define NOTE_D5   50
#define NOTE_Eb5  51
#define NOTE_E5   52
#define NOTE_F5   53
#define NOTE_Gb5  54
#define NOTE_G5   55
#define NOTE_Ab5  56
#define NOTE_A5   57
#define NOTE_Bb5  58
#define NOTE_B5   59

#define NOTE_C6   60
#define NOTE_Db6  61
#define NOTE_D6   62
#define NOTE_Eb6  63
#define NOTE_E6   64
#define NOTE_F6   65
#define NOTE_Gb6  66
#define NOTE_G6   67
#define NOTE_Ab6  68
#define NOTE_A6   69
#define NOTE_Bb6  70
#define NOTE_B6   71

#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using

#define fsrpin A1 //drucksensor

#define PIXEL_PIN    6  // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 10  // Number of NeoPixels

#define buttonPin A3

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
int notes[] = {NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6, NOTE_D6, NOTE_E6, NOTE_F6, NOTE_G6, NOTE_A6, NOTE_B6};
boolean calibrated = false;
int currentNote = 0;
int arraySize;
int fsrreading;
int prevIndex = INT_MIN;
int prevPrs = 0;
int prsTolerance = 15;
int prsStart = 120;
long lightTimer=0;
boolean pressed= false;
boolean enteredPressed = false;


MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

void setup() {
  delay(5000);
  arraySize = sizeof(notes) / sizeof(int);
  pinMode(buttonPin, INPUT_PULLUP);
  Wire.begin();
  Serial.begin(38400);
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);


    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");


    if (SerialDebug)
    {
      myIMU.getAres();
      myIMU.getGres();
      myIMU.getMres();

      if (SerialDebug)
      {
        Serial.println("Magnetometer:");
        Serial.print("X-Axis sensitivity adjustment value ");
        Serial.println(myIMU.factoryMagCalibration[0], 2);
        Serial.print("Y-Axis sensitivity adjustment value ");
        Serial.println(myIMU.factoryMagCalibration[1], 2);
        Serial.print("Z-Axis sensitivity adjustment value ");
        Serial.println(myIMU.factoryMagCalibration[2], 2);
      }
      else
      {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);

        // Communication failed, stop here
        Serial.println(F("Communication failed, abort!"));
        Serial.flush();
        abort();
      }
    }
  }
  
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
}

void colorWipe(uint32_t color, int amount) {
  int halfArray = arraySize/2-1;
  if(amount<=6){
    amount=map(amount, 0, halfArray, 0, 10);
  }
  else{
    amount = map(amount, halfArray+1, arraySize, 0, 10); 
  }
  //amount=map(amount, 0, arraySize, 0, strip.numPixels());
  for(int i=0; i<amount; i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)  
  }
  if(amount<strip.numPixels()){
   for(int k=amount+1; k<strip.numPixels(); k++){
    strip.setPixelColor(k, (0, 0, 0));
  }
  }
    strip.show();                          //  Update strip to match
}

void rainbow(float angle, int amount){
  int red = 255;
  int green=0;
  int blue = 0;
  if(angle<=90){
  green = map(angle, 0, 90, 0, 255);
  red = 255-map(angle, 0, 180, 0, 255);
  }
  else{
  green=255;
  }
  if(angle>90){
  blue = map(angle, 90, 180, 0, 255);
  green= 255-map(angle, 90, 180, 0, 255);
  }
  colorWipe(strip.Color(red, green, blue), amount);
}

void loop() {
  // put your main code here, to run repeatedly:
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
    
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 50)
    {
        myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
                                    * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                            * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3)
                            * *(getQ() + 3));
        myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
                                    * *(getQ() + 2)));
        myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2)
                                    * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                            * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3)
                            * *(getQ() + 3));
        myIMU.pitch *= RAD_TO_DEG;
        myIMU.yaw   *= RAD_TO_DEG;

        // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
        //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
        // - http://www.ngdc.noaa.gov/geomag-web/#declination
        myIMU.yaw  -= 8.5;
        myIMU.roll *= RAD_TO_DEG;



        //Serial.print("BUTTON IST AN?: ");
        //Serial.println(digitalRead(buttonPin));

        
        fsrreading = analogRead(fsrpin);
        int index = map(abs(myIMU.roll), 0, 180, 0, arraySize);//calculateRollIndex(abs(myIMU.roll));
       
        //Serial.print("Analog reading = ");
        //Serial.println(fsrreading);
        if(fsrreading>prsStart){
           pressed=true;
           fsrreading = map(fsrreading, 0, 1000, 0, 127);
           }
        else{
          pressed=false;
          enteredPressed=false;
          noteOff(0, notes[index], 0);
          fsrreading=0;
          }


        
        rainbow(abs(myIMU.roll), index);
        
        if((pressed&&!enteredPressed)||prevIndex!=index){
          //Serial.println("ENTERED PRESSED IF");
          enteredPressed=true;
        //Serial.print("INDEX: ");
        //Serial.println(index);
          noteOff(0, notes[prevIndex], 0);
          prevIndex = index;
          noteOn(0, notes[index], fsrreading);//fsrreading für druck, 127 für konstant  
        }

        
        /*if(prevPrs>fsrreading+prsTolerance || prevPrs<fsrreading-prsTolerance){
          noteOff(0, notes[index], 0);
          noteOn(0, notes[index], fsrreading);
          prevPrs=fsrreading;
        }*/

          //positives Z runter zur Erde
          //yaw= winkel zwischen x achse und magnetischem norden
          //pitch = Winkel zwischen X und Erdboden, zur erde positiv
          //roll = winkel zwischen Y achse und erdboden
          /*
           * Serial.print("yaw: ");
          Serial.println(myIMU.yaw);
          Serial.print("pitch: ");
          Serial.println(myIMU.pitch);
          Serial.print("roll: ");
          Serial.println(myIMU.roll);
          
          */

        
      
      myIMU.count = millis();
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  delay(20);
}

// Beginnt, eine Note zu spielen.
// - channel: MIDI Kanal, auf dem die Note gespielt wird. Es können z.B. mehrere Instrumente über unterschiedliche Kanäle gespielt werden.
// - pitch:   Tonhöhe des gespielten Tons (Umrechnung, siehe: https://newt.phys.unsw.edu.au/jw/notes.html)
// - velocity: Anschlagstärke des gespielten Tons (leise oder laut)
void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
  MidiUSB.flush();
}

// Beendet die gespielte Note.
// - channel: MIDI Kanal, auf dem die Note gestartet wurde.
// - pitch: Tonhöhe der Note, die gestartet wurde.
// - velocity: Normalerweise egal, standardmäßig 0 verwenden.
void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
  MidiUSB.flush();
}
