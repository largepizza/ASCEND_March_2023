#ifndef SDLOGGER_H
#define SDLOGGER_H


#include "Arduino.h"
#include <SD.h>
#include <EEPROM.h>
#include "Sensors.h"
#include <ArduinoJson.h>


#define STATS_NAME "stats.json"
#define LAST_RUN_NAME "lastRun.json"
#define GPS_LOG_NAME "GPS.csv"
#define IMU_LOG_NAME "IMU.csv"
#define CONSOLE_LOG_NAME "console.csv"
#define RTC_SYNC_NAME "rtc_sync.csv"
#define RUN_STATS_NAME "run_stats.json"

#define MAX_RUNS 9999

#define LOG_DELAY 10
#define STATS_DELAY 1000

//EEPROM ADDRESSES



enum MsgType {LOG, USERINPUT, WARNING, CRITICAL_ERROR};


/*
class ConsoleLog {
    public:
        uint32_t t;
        MsgType type;
        char msg[64];
};
*/


class RuntimeStats {
    public:
        uint32_t runIndex;
        uint32_t systemActive_sec;
        
};



class DataSystem {
    public:
        SensorSystem *Payload;
    
        RuntimeStats total_stats;
        RuntimeStats prev_stats;
        RuntimeStats current_stats;
        //ConsoleLog log;

        uint16_t log_delay = 100;

        uint32_t runNum;
        uint32_t epoch;
        uint32_t epoch_millis;
        

        //SD System Objects
        bool sdCardAvaliable = false;
        bool forceSerialMonitor = false;
        uint32_t runIndex;
        char runDir[64];
        char GPSLogDir[64];
        char IMULogDir[64];
        char ConsoleLogDir[64];
        char RTCSyncDir[64];
        


        //Initialization
        void init(SensorSystem* Payload_adr);
        void buildDirectory();
        void buildFirstTimeFiles();
        //Datalogging
        void writeGPSLog();
        void writeIMULog();
        void writeConsoleLog(const char* msg, MsgType type);

        void loop();

        //Total Stats
        void readStats(const char* name, RuntimeStats* stats);
        void writeStats(const char* name, RuntimeStats* stats);

        void updateCurrentStats();
        void updateTotalStats();
        
        //Config
        void readConfig();
        void readConfigFromEEPROM();
        void writeConfig();
        void writeConfigToEEPROM();

        //EEPROM
        uint8_t writeEEPROMSequential(void* var, uint8_t size, u_int16_t adr);
        uint8_t readEEPROMSequential(void* var, uint8_t size, u_int16_t adr);

        private:
            const uint8_t chipSelect = SS;
            uint32_t t_logDelay;
            uint32_t t_statsDelay;
            //uint16_t addressTest = 0;


};









#endif