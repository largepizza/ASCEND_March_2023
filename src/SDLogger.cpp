#include "Arduino.h"
#include "SDLogger.h"
#include <SD.h>
#include <ArduinoJson.h>

char const *msgTypeStr[] = {"LOG", "USERINPUT", "WARNING", "ERROR"};


void DataSystem::init(SensorSystem* Payload_adr) {
    Payload = Payload_adr;

    //writeConfigToEEPROM();
    //readConfigFromEEPROM();
        
    if (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("The SD card failed to initalize");
        sdCardAvaliable = false;
        return;
    }
    sdCardAvaliable = true;
    
    buildFirstTimeFiles();
    //readConfig();
    buildDirectory();

    char str[32];
    sprintf(str,"Writing to directory \"%s\"",runDir);
    writeConsoleLog(str,LOG);

    updateTotalStats();
    
    
    //Serial.println("Data Logging Enabled");
}

void DataSystem::writeStats(const char* name, RuntimeStats* stats) {
    // Delete existing file, otherwise the configuration is appended to the file
    SD.remove(name);

    // Open file for writing
    File file = SD.open(name, FILE_WRITE);
    if (!file){
        Serial.println(F("Failed to create stats file"));
        return;
    }

    // Allocate a temporary JsonDocument
    StaticJsonDocument<256> doc;

    // Set the values in the document
    doc["runIndex"] = stats->runIndex;
    doc["systemActive_sec"] = stats->systemActive_sec;
    

    // Serialize JSON to file
    if (serializeJsonPretty(doc, file) == 0){
        Serial.println(F("Failed to write to stats file"));
    }

    // Close the file
    file.close();
}

void DataSystem::updateTotalStats() {
    readStats(LAST_RUN_NAME,&prev_stats);
    readStats(STATS_NAME, &total_stats);

    if (runIndex > 0) {
        char lastRunDir[64];
        sprintf(lastRunDir, "runs/run%d/%s",(int)(runIndex-1), RUN_STATS_NAME);
        writeStats(lastRunDir,&prev_stats);
    }
    

    total_stats.runIndex = runIndex;
    total_stats.systemActive_sec += prev_stats.systemActive_sec;


    writeStats(STATS_NAME, &total_stats);
}

void DataSystem::buildFirstTimeFiles() {
    
    if (sdCardAvaliable) {
        if (SD.mkdir("runs")){
            Serial.println("Created runs directory");
        }
        else {
            Serial.println("run Directory Exists");
        }    

        if (!SD.exists(STATS_NAME)) {
            //Build stats file with default settings
            Serial.println("Default stats written!");
            writeStats(STATS_NAME, &total_stats);
        } 
        if (!SD.exists(LAST_RUN_NAME)) {
            //Build stats file with default settings
            Serial.println("Default current stats written!");
            writeStats(LAST_RUN_NAME, &current_stats);
        } 

        
        

    }
}

void DataSystem::buildDirectory() {
    if (sdCardAvaliable) {
        //Checks to see the next avaliable runs/runXXX directory
        readStats(LAST_RUN_NAME,&prev_stats);

        runIndex = prev_stats.runIndex+1;
        
        char dirStr[64] = "runs/run0";
        sprintf(dirStr,"runs/run%d",(int)runIndex);
        strncpy(runDir,dirStr,64);

        //Creates runs/runXXX directory
        if (SD.mkdir(runDir)){
            Serial.print("Created: ");
            Serial.println(runDir);
        }
        else {
            Serial.println("run dir failed");
        }    

        //Builds directory names for all appropriate data
        sprintf(GPSLogDir,"%s/%s",runDir,GPS_LOG_NAME);
        sprintf(IMULogDir,"%s/%s",runDir,IMU_LOG_NAME);
        sprintf(ConsoleLogDir,"%s/%s",runDir,CONSOLE_LOG_NAME);
        sprintf(RTCSyncDir,"%s/%s",runDir,RTC_SYNC_NAME);

        //Creates .csv files with headers for all data
        //PMU
        File file = SD.open(GPSLogDir, FILE_WRITE);
        if (!file){
            Serial.println(F("Failed to create GPS file"));
        }
        else {
            file.print("systemTime(ms),");                     
            file.print("gps_minute,");                    
            file.print("gps_hour,");                      
            file.print("gps_day,");                       
            file.print("gps_month,");                      
            file.print("gps_year,");                  
            file.print("gps_fix,");                       
            file.print("gps_quality,");                  
            file.print("gps_timeSinceFix,");                 
            file.print("gps_timeSinceTime,");              
            file.print("gps_timeSinceDate,");            
            file.print("gps_latitude,");          
            file.print("gps_lat,");      
            file.print("gps_longitude,");      
            file.print("gps_lon,");                       
            file.print("gps_speed,");                
            file.print("gps_angle,");
            file.print("gps_alt,");       
            file.print("gps_sats");             
            file.println();

            /*float gps_seconds;
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
        int gps_sats;*/
        }
        file.close();

        //ECU
        file = SD.open(IMULogDir, FILE_WRITE);
        if (!file){
            Serial.println(F("Failed to create IMU file"));
        }
        else {
            file.print("systemTime(ms),");         //System millis()
            file.print("mpl_temp,");
            file.print("mpl_pressure,");
            file.print("mpl_altitude,");
            file.print("bno_temp,");
            file.print("bno_mag_x,");
            file.print("bno_mag_y,");
            file.print("bno_mag_z,");
            file.print("bno_gyro_x,");
            file.print("bno_gyro_y,");
            file.print("bno_gyro_z,");
            file.print("bno_euler_x,");
            file.print("bno_euler_y,");
            file.print("bno_euler_z,");
            file.print("bno_linear_x,");
            file.print("bno_linear_y,");
            file.print("bno_linear_z,");
            file.print("bno_gravity_x,");
            file.print("bno_gravity_y,");
            file.print("bno_gravity_z");
    
            file.println();
        }
        file.close();

        //Console
        file = SD.open(ConsoleLogDir, FILE_WRITE);
        if (!file){
            Serial.println(F("Failed to create Console Log file"));
        }
        else {
            file.print("systemTime(ms),");     //System millis()
            file.print("msgType,");            //Message type: :LOG, WARNING, ERROR, etc.
            file.print("msg");                 //Message
            file.println();
        }
        file.close();

        //RTC
        file = SD.open(RTCSyncDir, FILE_WRITE);
        if (!file){
            Serial.println(F("Failed to create RTCSync Dir  file"));
        }
        else {
            file.print("systemTime(ms),");     //System millis()
            file.print("UNIXEpoch(s),");       //EPOCH at given millis
            file.print("from");                //Where RTC was aquired: JSON/droneCAN
            file.println();
        }
        file.close();


    }
}


void DataSystem::writeGPSLog() {
    //PMU
    File file = SD.open(GPSLogDir, FILE_WRITE);
    if (!file){
        Serial.println(F("Failed to write to GPS file"));
    }
    else {
            file.print(millis());             
            file.print(",");        
            file.print(Payload->gps_minute); 
            file.print(",");                   
            file.print(Payload->gps_hour);    
            file.print(",");                  
            file.print(Payload->gps_day);     
            file.print(",");                  
            file.print(Payload->gps_month);   
            file.print(",");                   
            file.print(Payload->gps_year);    
            file.print(",");              
            file.print(Payload->gps_fix);     
            file.print(",");                  
            file.print(Payload->gps_quality); 
            file.print(",");                 
            file.print(Payload->gps_timeSinceFix);   
            file.print(",");              
            file.print(Payload->gps_timeSinceTime);  
            file.print(",");            
            file.print(Payload->gps_timeSinceDate);  
            file.print(",");          
            file.print(Payload->gps_latitude);       
            file.print(",");   
            file.print(Payload->gps_lat);      
            file.print(",");
            file.print(Payload->gps_longitude);      
            file.print(",");
            file.print(Payload->gps_lon);            
            file.print(",");           
            file.print(Payload->gps_speed);          
            file.print(",");      
            file.print(Payload->gps_angle);
            file.print(",");
            file.print(Payload->gps_alt);       
            file.print(",");
            file.print(Payload->gps_sats);             
            file.println();

        file.println();
        
    }
    file.close();
}

void DataSystem::writeIMULog() {
    //ECU
    File file = SD.open(IMULogDir, FILE_WRITE);
    if (!file){
        Serial.println(F("Failed to write to IMU file"));
    }
    else {
        file.print(millis());                       //System millis()
        file.print(",");
        file.print(Payload->mpl_temp);
        file.print(",");
        file.print(Payload->mpl_pressure);
        file.print(",");
        file.print(Payload->mpl_altitude);
        file.print(",");
        file.print(Payload->bno_temp);
        file.print(",");
        file.print(Payload->bno_mag_x);
        file.print(",");
        file.print(Payload->bno_mag_y);
        file.print(",");
        file.print(Payload->bno_mag_z);
        file.print(",");
        file.print(Payload->bno_gyro_x);
        file.print(",");
        file.print(Payload->bno_gyro_y);
        file.print(",");
        file.print(Payload->bno_gyro_z);
        file.print(",");
        file.print(Payload->bno_euler_x);
        file.print(",");
        file.print(Payload->bno_euler_y);
        file.print(",");
        file.print(Payload->bno_euler_z);
        file.print(",");
        file.print(Payload->bno_linear_x);
        file.print(",");
        file.print(Payload->bno_linear_y);
        file.print(",");
        file.print(Payload->bno_linear_z);
        file.print(",");
        file.print(Payload->bno_gravity_x);
        file.print(",");
        file.print(Payload->bno_gravity_y);
        file.print(",");
        file.print(Payload->bno_gravity_z);
    
        //file.print(engineStatusStr[ECU->status]);   //Engine status
        //file.print(",");

        file.println();
        
    }
    file.close();
}

void DataSystem::writeConsoleLog(const char* msg, MsgType type) {
    //Console Log
    if(sdCardAvaliable) {
        File file = SD.open(ConsoleLogDir, FILE_WRITE);
        if (!file){
            Serial.println(F("Failed to write to console file"));
        }
        else {
            file.print(millis());           //System millis()
            file.print(",");
            file.print(msgTypeStr[type]);   //Msg status
            file.print(",");
            file.print(msg);                //Msg
            file.println();
        }
        file.close();
    }
    /*
    if (!forceSerialMonitor) {
    StaticJsonDocument<256> doc;
    doc["systemTime"] = millis();     //System millis()
    doc["msgType"] = msgTypeStr[type];    //Message type: :LOG, WARNING, ERROR, etc.
    doc["msg"] = msg;                     //Message
    doc["id1"] = 0;                       //Unique Identifier

    Serial.print("[");
    serializeJson(doc,Serial);
    Serial.println("]");
    }*/
}

void DataSystem::readStats(const char* name, RuntimeStats* stats) {
// Open file for reading
    File file = SD.open(name);

    // Allocate a temporary JsonDocument
    StaticJsonDocument<512> doc;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
        Serial.println(F("Failed to read file, using default configuration"));
    }
    // Copy values from the JsonDocument to the Config
    stats->runIndex = doc["runIndex"];
    stats->systemActive_sec = doc["systemActive_sec"];
    // Close the file (Curiously, File's destructor doesn't close the file)
    file.close();
}


void DataSystem::updateCurrentStats() {
    current_stats.runIndex = runIndex;
    current_stats.systemActive_sec = millis()/1000.0;

    writeStats(LAST_RUN_NAME,&current_stats);

}







void DataSystem::loop(){
    if(sdCardAvaliable) {
        

        if (millis() - t_logDelay > 100) {
            t_logDelay = millis();
            writeGPSLog();
            writeIMULog();
        }

        if (millis() - t_statsDelay >  STATS_DELAY) {
            t_statsDelay = millis();
            updateCurrentStats();
            
            /*
            uint8_t int8e = 255;
            addressTest += writeEEPROMSequential(&int8e, sizeof(int8e), addressTest);
            uint32_t int32e = 3459082;
            addressTest += writeEEPROMSequential(&int32e, sizeof(int32e), addressTest);
            float floaty = 2.5453;
            addressTest += writeEEPROMSequential(&floaty, sizeof(floaty), addressTest);
            double doubley = 22.5453;
            addressTest += writeEEPROMSequential(&doubley, sizeof(doubley), addressTest);
            */

        }
    }





}
