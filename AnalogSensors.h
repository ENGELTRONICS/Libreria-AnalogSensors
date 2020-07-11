/******************************
    AnalogSensors

    Author: Ing.Felipe Andrés Ángel Rodríguez

      Date: 2020
 ******************************/


#ifndef ANALOGSENSOR_H
#define ANALOGSENSOR_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define RESOLUTION     4095       // Constante para definir la escala a la que se realizara la conversion del ADC.
#define VOLTAJEMAX      3.3       // Constante de voltaje para realizar los calculos del ADC.
#define TIMERADQ         20       // Tiempo de adquisición en milisegundos.


class AnalogSensor
{
  public:
    AnalogSensor(int pin);
    int rawADCReadings();
    float writeValueVoltaje();
    float writeValuePressure(float valPressureMin, float valPressureMax, float valRangeMin, float valRangeMax);

  private:
    int _pin;
    int   _valueADC      = 0;
    float _valVoltaje    = 0.0;
    float _valuePressure = 0.0;
};

#endif /* ANALOGSENSOR_H */
