// HPR FLIGHT SOFTWARE V01
// FOR SPIKE V002 FLIGHT COMPUTER

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LIS3MDL.h>
#include <kalmanFilter.h>
#include <flightFunctions.h>
#include "sensorfunctions.h"
#include <Streaming.h>

#include <Adafruit_GPS.h>


// precompiler definitions
#define GPSSerial Serial1

// Hardware Classes
Adafruit_GPS GPS(&GPSSerial);     // GPS
Adafruit_BMP3XX bmp;              // Barometer/Altimeter
Adafruit_ISM330DHCX ism330dhcx;   // IMU
Adafruit_LIS3MDL mag;             // Magnetometer
SDClass builtInSD;                // Built In SD Card
SDClass flashSD;                  // Flash SD Card

// State Tracking Classes
using namespace Eigen;
Sensor sensor;
StateVector currentState;
constants::Noise noise;
Ricotti ricotti{buildRicotti(noise)};
Phase phase{phase_standby}; // set phase to start on standby

// Tracking Variables
double initLatitude{};
double initLongitude{};


// GPS pre-setup
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
#define GPSECHO false


IntervalTimer GPSTimer;

void readGPS() // GPS read function to be used with interval timer
{
  GPS.read();
}

// Barometer
#define SEALEVELPRESSURE_HPA (1013.25)
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 9

// Overall Timer
elapsedMicros looptimer;

// holds current magDeclination angle in radians
double magDeclination {0.0};

// -------------SETUP LOOP------------------
void setup()
{
  bool noFault{true}; // fault flag

  //while (!Serial);  

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  //Serial.begin(115200);
  
  // set pyros low
  pinMode(pin::drogue, OUTPUT);
  digitalWrite(pin::drogue, LOW);
  pinMode(pin::main, OUTPUT);
  digitalWrite(pin::main, LOW);

  // set up leds
  pinMode(pin::red, OUTPUT);
  pinMode(pin::blue, OUTPUT);
  pinMode(pin::green, OUTPUT);

  // buzzer
  pinMode(pin::buzzer, OUTPUT);

  pinMode(10,OUTPUT); // set default spi CS to output

  // light up blue for setup
  digitalWrite(pin::blue, HIGH);
  // for setup failures turn blue off, red on

  // ----------Barometer set up----------
  if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    digitalWrite(pin::blue, LOW);
    digitalWrite(pin::red, HIGH);
    noFault = false;
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  // ----------IMU set up----------
  if (!ism330dhcx.begin_I2C()) {
    Serial.println("Failed to find ISM330DHCX chip");
      digitalWrite(pin::blue, LOW);
      digitalWrite(pin::red, HIGH);
      noFault = false;
  }
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
  ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2
  
  // ----------Magnetometer set up----------
  if (! mag.begin_I2C()) 
  {
    Serial.println("Failed to find LIS3MDL chip");
      digitalWrite(pin::blue, LOW);
      digitalWrite(pin::red, HIGH);
      noFault = false;
  }
  mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
  mag.setRange(LIS3MDL_RANGE_4_GAUSS);
  mag.setIntThreshold(500);
  mag.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled

  // ----------SD Card set up----------

  pinMode(BUILTIN_SDCARD,OUTPUT);
  if(!builtInSD.begin(BUILTIN_SDCARD))
  {
    Serial.println("Built in Card failed, or not present."); // will want to replace with lights/buzzer
      digitalWrite(pin::blue, LOW);
      digitalWrite(pin::red, HIGH);
      noFault = false;
  }

  const int flashCS{10};
  pinMode(flashCS,OUTPUT);
  digitalWrite(flashCS, HIGH);
  

  if(!flashSD.begin(flashCS))
  {
    Serial.println("FlashSD Card failed, or not present."); // will want to replace with lights/buzzer
      digitalWrite(pin::blue, LOW);
      digitalWrite(pin::red, HIGH);
      noFault = false;
  }

  // Grab any data left on flash chip:
        File dataFile = flashSD.open("datalog.csv");
        File dataCopy = builtInSD.open("olddata.csv", FILE_WRITE);

        if (dataFile) 
        {
            // read from the file until there's nothing else in it:
            while (dataFile.available()) 
            {
                dataCopy.write(dataFile.read());
            }
            // close the file:
            dataFile.close();
            dataCopy.close();
            
        } else 
        {
            // if the file didn't open, print an error:
            Serial.println("error opening datalog");
        }

  
  // keep flash SD clean and set up column names
  flashSD.remove("datalog.csv");
  dataFile = flashSD.open("datalog.csv", FILE_WRITE);

  // write column headers
  if(dataFile)
  {
    dataFile.println("phase, time(millis), lam0, lam1, lam2, lam3, X, Y, Z, velX, velY, velZ, senAccX, senAccY, senAccZ, senGyroX, senGryoY, senGryoZ, senMagX, senMagY, senMagZ, senPosX, senPosY, senPosZ, senVelX, senVelY, senVelZ");
    dataFile.close();
  }



  // ----------GPS Set Up----------
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  if(!GPS.begin(9600))
  {
      digitalWrite(pin::blue, LOW);
      digitalWrite(pin::red, HIGH);
      noFault = false;
  }
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); // position fix update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

   // read GPS data on time using this interrupt
  GPSTimer.begin(readGPS, 1000);

  // wait for fix, let GPS attune
  {
    int count{0};
    while(!GPS.fix || count < 10)
    {
      if (GPS.newNMEAreceived())
      {
        if(GPS.parse(GPS.lastNMEA()))
        {
          ++count;
        }
      }
    }
  }

  // get initial location data to normalize to zero. use sign in place of N/S and E/W
    if(GPS.lat == 'N')
    {
      initLatitude = GPS.latitudeDegrees;
    }
    else
    {
      initLatitude = -1 * GPS.latitudeDegrees;
    }
    
    if(GPS.lon == 'E')
    {
      initLongitude = GPS.longitudeDegrees;
    }
    else
    {
      initLongitude = -1 * GPS.longitudeDegrees;
    }

  
  if(noFault)
  {
    digitalWrite(pin::blue, LOW);
    digitalWrite(pin::green, HIGH);
    for (int i{0}; i < 3; ++i)
    {
      delay(100);
      tone(pin::buzzer, 524);
      delay(100);
      noTone(pin::buzzer);
    }
  }
  

 
}

// ----------------- LOOP SECTION----------------------

void loop() // run over and over again
{
  if(looptimer >= 20000) // runs exactly every 0.02 seconds or 50Hz
  {
    looptimer = 0;
    bool gpsAvail{false};
    
   

    // check if a new GPS sentence was recieved, and try to parse it, and check for a fix
    if (GPS.newNMEAreceived()) 
    {  
      if (GPS.parse(GPS.lastNMEA()))
      {
        if (GPS.fix)
        {
          gpsAvail = true;
        } 
      }
    }

    // Serial << "GPS parse time: " << looptimer << '\n';

    if(gpsAvail)
    {
      magDeclination = deg2rad(GPS.magvariation);
    }

    // read sensors
    // Barometer
    bmp.performReading();

    // Serial << "Barometer time: " << looptimer << '\n';

    // IMU
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp; // not interested in temp but the library requires the container to run .getEvent()
    ism330dhcx.getEvent(&accel, &gyro, &temp);

    // Serial << "IMU time: " << looptimer << '\n';

    // magnetometer
    mag.read();

    // Serial << "Mag time: " << looptimer << '\n';

    sensor = sensorBuild(gpsAvail, GPS, initLatitude, initLongitude, bmp, accel, gyro, mag);

    // Serial << "Sensor Build Time: " << looptimer << '\n';
    

    // ----- Begin Kalman Section -----
    static uint32_t loopcounter{0};
    if(loopcounter < 1)
    {
      initializeStateVector(currentState, sensor);
    }

  if(loopcounter > 5) // initial sensor readings can be wonky, so I want to log a few and then start the filter up
    {
    // runs the kalman filter
    kalmanFilter(currentState, ricotti, sensor, gpsAvail, magDeclination);
    }

    // Serial << "Kalman Filter time: " << looptimer << '\n';

    // ----------Phase Testing----------
    phaseCheck(phase, currentState, sensor);

    // Pyro Check
    if(phase == phase_apogee)
    {
      drogueFire(phase, currentState);
    }
    if(phase == phase_mainChute)
    {
      mainFire(phase, currentState);
    }

    // Serial << "Phase test time: " << looptimer << '\n';

    datalog(phase, currentState, sensor, builtInSD, flashSD);

    //check time
    //Serial << "Loop time: " << looptimer << '\n';

    ++loopcounter;
  }


}