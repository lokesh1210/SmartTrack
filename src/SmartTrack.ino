/*
 * Project SmartTrack
 * Description: Particle firmware to read Log vehicle data to cloud
 * Author: Lokesh Korapati (korapati.lokesh@gmail.com)
 * Date: 6 june 2017
 */

#include "AssetTracker.h"


#define ANALOG_PIN  A0

int transmittingData =0;
AssetTracker t = AssetTracker();
FuelGauge fuel;

int delaySeconds = 5;
long lastScanned = 0;
long startTime;

int analogValue=0;

#define BIAS_VOLTAGE    3300    //Voltage driving thermistor 
#define BIAS_RESISTOR   5000    //Resistance in series with thermister
#define ADC_REF         3300    //Electron ADC measurement REf
#define RES_AT_25       3000    //thermistor resistance at 25degC

typedef struct 
{
    float temperature;
    float rsFactor;
}RefPoint_t;

#define NO_OF_POINTS    13
RefPoint_t thLUT[] = {
    {00.0f, 2.8665f},
    {05.0f, 2.2907f},
    {10.0f, 1.8438f},
    {15.0f, 1.4920f},
    {20.0f, 1.2154f},    
    {25.0f, 1.0000f},    
    {30.0f, 0.8297f},    
    {35.0f, 0.6863f},    
    {40.0f, 0.5710f},    
    {45.0f, 0.4801f},    
    {50.0f, 0.4054f},    
    {55.0f, 0.3417f},    
    {60.0f, 0.2895f},    
    

};

float calculateR(int analogvalue){

    float v = (analogValue*ADC_REF)/4096; //voltage across thermistor
    float i = (BIAS_VOLTAGE-v)/BIAS_RESISTOR;   //current tru thermistor
    float r = v/i;  //resistance of thermistor
    return r;
}

float calculateT(float resistance){
    int stIndx=0, endIndx=1;
    if(resistance >= (thLUT[0].rsFactor*RES_AT_25)){
        //indexes at 0,1
        stIndx=0;
        endIndx=1;
    }
    else if(resistance <= (thLUT[NO_OF_POINTS-1].rsFactor*RES_AT_25)){
        stIndx=NO_OF_POINTS-2;
        endIndx=NO_OF_POINTS-1;
    }
    else{
        //search for the data reference points
        for(int i =1; i<NO_OF_POINTS; i++){
            stIndx = i-1;
            endIndx = i;
            if(resistance >= (thLUT[i].rsFactor*RES_AT_25))
            {               
                break;
            }
        }
    }
    //Serial.println(String::format("st=%d, end=%d, r=%.2f",stIndx, endIndx, resistance));
    //Interpolate the temperature 
    float r1 = thLUT[stIndx].rsFactor*RES_AT_25;
    float r2 = thLUT[endIndx].rsFactor*RES_AT_25;
    float t1 = thLUT[stIndx].temperature;
    float t2 = thLUT[endIndx].temperature;

    float t = t1 + (((resistance-r1) * (t2-t1))/(r2-r1));

    return t;
    
}

void setup() {

    //t.begin();

    //t.gpsOn();

    Serial.begin(9600);    

    Particle.function("tmode", transmitMode);
    Particle.function("batt", batteryStatus);
    startTime = millis();
    Serial.println("Initialization completed");
}


void loop() {

    //t.updateGPS();

    long current = millis();

    if((current-lastScanned) > delaySeconds*1000){
        lastScanned = current;

        Serial.print((int)(current-startTime)/1000);

        // String pubAccel = String::format("Accelerometer x:%d, y:%d, Z:%d", t.readX(), t.readY(), t.readZ());
        // Serial.println(pubAccel);

        analogValue = analogRead(ANALOG_PIN);
        float calculatedR = calculateR(analogValue);
        float temperatureVal = calculateT(calculatedR);

        String temperatureLog = String::format(" Battery V:%.2f",fuel.getVCell()) +
                         String::format(" C:%.2f",fuel.getSoC()) +
                         String::format(" Ta:%d",analogValue) + 
                         String::format(" T:%.2fC",temperatureVal);
        Serial.println(temperatureLog + String::format(" Rt:%.2f",calculatedR));

        if (transmittingData) {           
            Particle.publish("mst_data_log", temperatureLog, PRIVATE);
            //Serial.println("Transmitting data");
        }


        /*Serial.println(t.preNMEA());*/

        /*if (t.gpsFix()) {

            Serial.println(t.readLatLon());
            String gpsStr = String::format("GPS Lat:%.6f",t.readLat()) +
                             String::format("Lon:%.6f",t.readLon());m
            Serial.println(gpsStr);
        }
        else
        {
            Serial.println("No GPS Fix\n");
        }*/

    }

}

// Allows you to remotely change whether a device is publishing to the cloud
// or is only reporting data over Serial. Saves data when using only Serial!
// Change the default at the top of the code.
int transmitMode(String command) {
    transmittingData = atoi(command);
    if(transmittingData){
        Serial.println("Transmitting data");
    }
    else{
        Serial.println("Stop Transmitting data");
    }       
    return 1;
}

// Lets you remotely check the battery status by calling the function "batt"
// Triggers a publish with the info (so subscribe or watch the dashboard)
// and also returns a '1' if there's >10% battery left and a '0' if below
int batteryStatus(String command){
    // Publish the battery voltage and percentage of battery remaining
    // if you want to be really efficient, just report one of these
    // the String::format("%f.2") part gives us a string to publish,
    // but with only 2 decimal points to save space
    Particle.publish("B",
          "v:" + String::format("%.2f",fuel.getVCell()) +
          ",c:" + String::format("%.2f",fuel.getSoC()),
          60, PRIVATE
    );
    // if there's more than 10% of the battery left, then return 1
    if (fuel.getSoC()>10){ return 1;}
    // if you're running out of battery, return 0
    else { return 0;}
}
