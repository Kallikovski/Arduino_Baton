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

int fsrreading;
int prevNote = INT_MIN;
int prevPrs = 0;
int prsTolerance = 15;
int prsStart = 120;
long lightTimer=0;
boolean pressed= false;
boolean enteredPressed = false;

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

class Notes {
  public:
    Notes():notes({ //Note array
        NOTE_C5,
        NOTE_D5,
        NOTE_E5,
        NOTE_F5,
        NOTE_G5,
        NOTE_A5,
        NOTE_B5,
        NOTE_C6,
        NOTE_D6,
        NOTE_E6,
        NOTE_F6,
        NOTE_G6,
        NOTE_A6,
        NOTE_B6
    }){}
    
    int get_Note(int roll, int angleLow, int angleHigh){
      unsigned int index = map(abs(roll), angleLow, angleHigh, 0, (sizeof(notes)/sizeof(int)));
      unsigned int note = notes[index];
      return note;
    }
    int get_NoteIndex(unsigned int note){
      for(int i=0; i<sizeof(notes); ++i){
        if(notes[i] ==note){
          return i;
        }
      }
    }
    int getSizeOfNotes(){
      return (sizeof(notes)/sizeof(int));
    }
  
  private:
    unsigned int notes[14]; // C-Array that defines amount of Notes
};

Notes* notesObj;

void setup() {
  delay(5000);
  notesObj = new Notes();
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

    myIMU.initMPU9250();

    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
      myIMU.getAres();
      myIMU.getGres();
      myIMU.getMres();
  }
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
}

void colorWipe(uint32_t color,unsigned int currentNote) {
  int amount=notesObj->get_NoteIndex(currentNote);
  int arraySize=notesObj->getSizeOfNotes();
  int halfArray=arraySize/2;
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

void rainbow(float angle,unsigned int currentNote){
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
  colorWipe(strip.Color(red, green, blue), currentNote);
}

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
  MidiUSB.flush();
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
  MidiUSB.flush();
}

void loop() {    
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();
  
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 20)
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
        myIMU.yaw  -= 8.5;
        myIMU.roll *= RAD_TO_DEG;

        fsrreading = analogRead(fsrpin);
        currentNote= notesObj->get_Note(myIMU.roll,0,180);
       
        if(fsrreading>prsStart){
           pressed=true;
           fsrreading = map(fsrreading, 0, 1000, 0, 127);
           }
        else{
          pressed=false;
          enteredPressed=false;
          noteOff(0, currentNote, 0);
          fsrreading=0;
          }

        rainbow(abs(myIMU.roll), currentNote);
        
        if((pressed&&!enteredPressed)||prevNote!=currentNote){
          enteredPressed=true;
          noteOff(0, prevNote, 0);
          prevNote = currentNote;          
          noteOn(0, currentNote, fsrreading);//fsrreading für druck, 127 für konstant  
        }      
      myIMU.count = millis();
    } // if (myIMU.delt_t > 50)
  delay(10);
}
