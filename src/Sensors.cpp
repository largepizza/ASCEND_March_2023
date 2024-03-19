#include "Arduino.h"
#include "Sensors.h"
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define GPSECHO true


Adafruit_GPS GPS(&GPSSerial);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void SensorSystem::init() {
    if (!mpl.begin()) {
        Serial.println("Could not find sensor. Check wiring.");
        while(1);
    }

    mpl.setSeaPressure(1013.26);

      if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    bno.setExtCrystalUse(true);

    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); // 10 second update time
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    GPSSerial.println(PMTK_Q_RELEASE);


}

void SensorSystem::loop() {
    
    //Aquire from MPL Sensor
    if (millis() - t_mpl >= MPL_DELAY) {
        t_mpl = millis();
        if (!MPLBusy) {
            mpl.startOneShot();
            MPLBusy = true;
        }
    }

    if (mpl.conversionComplete()) {
        MPLBusy = false;
        mpl_pressure = mpl.getLastConversionResults(MPL3115A2_PRESSURE);
        mpl_temp = mpl.getLastConversionResults(MPL3115A2_TEMPERATURE);
        mpl_altitude = mpl.getLastConversionResults(MPL3115A2_ALTITUDE);

    }

    if (millis() - t_bno >= BNO_DELAY) {
        bno_temp = bno.getTemp();
        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);


        bno_mag_x = mag.x();
        bno_mag_y = mag.y();
        bno_mag_z = mag.z();
        bno_gyro_x = gyro.x();
        bno_gyro_y = gyro.y();
        bno_gyro_z = gyro.z();
        bno_euler_x = euler.x();
        bno_euler_y = euler.y();
        bno_euler_z = euler.z();
        bno_linear_x = linear.x();
        bno_linear_y = linear.y();
        bno_linear_z = linear.z();
        bno_gravity_x = gravity.x();
        bno_gravity_y = gravity.y();
        bno_gravity_z = gravity.z();



    }


    // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c)
      Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag
                                    // to false
      return; // we can fail to parse a sentence in which case we should just
              // wait for another
  }

  // approximately every 2 seconds or so, random intervals, print out the
  // current stats
  static unsigned nextInterval = 1000;
  if (millis() - t_gps > nextInterval) {
    t_gps = millis(); // reset the timer
    
    gps_seconds = GPS.seconds + GPS.milliseconds / 1000. + GPS.secondsSinceTime();
    gps_minute = GPS.minute;
    gps_hour = GPS.hour;
    gps_day = GPS.day;

    gps_year = GPS.year;
    gps_month = GPS.month;
    
    gps_fix = ((int)GPS.fix);
    gps_quality = ((int)GPS.fixquality);
    gps_timeSinceFix = GPS.secondsSinceFix();
    gps_timeSinceTime = GPS.secondsSinceTime();
    gps_timeSinceDate = GPS.secondsSinceDate();
    if (GPS.fix) {
      gps_latitude = GPS.latitude;
      gps_lat = GPS.lat;
      gps_longitude = GPS.longitude;
      gps_lon = GPS.lon;
      gps_speed = GPS.speed;
      gps_angle = GPS.angle;
      gps_alt = GPS.altitude;
      gps_sats = GPS.satellites;
    }
  }
}


