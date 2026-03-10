<!-- 

Innovative Global Solutions - Infant Incubator Project
FILE: infantTempSensor.md
PURPOSE: 
AUTHOR: Fleser, Connor
CREATED: 2/16/26

 -->

<h1>Infant Temperature Sensor</h1>
This document outlines the basic functionality and implementation of the Infant Temperature Sensor.

<h2>Overview</h2>
The current implementation of the Infant Temperature Sensor utilizes the STS-35 sensor identified and initially developed by the Vitals subteam. This sensor connects over the I<sup>2</sup>C bus and uses a conversion formula calculated by the Vitals subteam. It is then read regularly and triggers the alarm when out of bounds.

<h2>Functionality</h2>

The functional implementation of the sensor works by:

    Program Sensor:
        readTemp := readSensorTemperature()
        calibratedTemp := convertTemp(readTemp)
        if calibratedTemp > infantTempMaxBound or calibratedTemp < infantTempMinBound:
            ActivateAlarm()
        else if alarmActivated:
            DeactivateAlarm()

    Function convertTemp(rawTemp):
        return rawTemp - 0.178(incubatorTemp - rawTemp)

    
    

Functional implementation of the sensor is primarily described by the vitals team under their investigation which may be found at: 
<a href="https://michigantech.sharepoint.com/:w:/s/IGSInfantIncubator/IQDBcJyeQeYXTLCEv6f7ScnDAdMtrMDROsgGFoQUf-eKk6k?e=6do62N">Calibrating Temperature</a>

<h2>Implementation</h2>

The infant temperature sensor is implemented in main.c via the function STS35_ReadTemperature() and utilizes STS35_CalcCRC to calculate its cyclic redundancy check. 

The STS35 sensor communicates over the HAL_I2C api.