#ifndef SENSORS_H
#define SENSORS_H

#define MPL_DELAY 100
#define BNO_DELAY 10



#include "Arduino.h"
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define GPSSerial Serial1




class SensorSystem {
    public:
        void init();
        void loop();

        Adafruit_MPL3115A2 mpl;
        
        bool MPLBusy = false;
        double mpl_temp;
        double mpl_pressure;
        double mpl_altitude;

        float gps_seconds;
        int gps_minute;
        int gps_hour;
        int gps_day;
        int gps_month;
        int gps_year;
        int gps_fix;
        int gps_quality;
        float gps_timeSinceFix;
        float gps_timeSinceTime;
        float gps_timeSinceDate;
        float gps_latitude;
        char gps_lat;
        float gps_longitude;
        char gps_lon;
        float gps_speed;
        float gps_angle;
        float gps_alt;
        int gps_sats;

        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
        int8_t bno_temp;
        float bno_mag_x;
        float bno_mag_y;
        float bno_mag_z;
        float bno_gyro_x;
        float bno_gyro_y;
        float bno_gyro_z;
        float bno_euler_x;
        float bno_euler_y;
        float bno_euler_z;
        float bno_linear_x;
        float bno_linear_y;
        float bno_linear_z;
        float bno_gravity_x;
        float bno_gravity_y;
        float bno_gravity_z;
    
    private:
        uint32_t t_mpl = 0;
        uint32_t t_gps = 0;
        uint32_t t_bno = 0;


};



#endif