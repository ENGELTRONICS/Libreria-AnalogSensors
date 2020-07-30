/******************************************** Project information *************************************************
  Name Proyect: Librería AnalogSensors
        Author: Ing.Felipe Andrés Ángel Rodríguez
          Data: 10/Julio/2020
           Rev: 1.0

  Description : Esta librería se encarga de leer los valores de un sensor analógico, también se implementan
                diferentes filtros para el procesamiento de datos.
******************************************************************************************************************/

#ifndef ANALOGSENSOR_H
#define ANALOGSENSOR_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//********************************************* Variables Macros **************************************************
#define TIMERADQ         20       // Tiempo de adquisición en milisegundos.

//********************************************* Variables Macros **************************************************
#define RANGE_MIN        0       // Rango mínimo de lectura del ADC.
#define RANGE_MAX     4095       // Rango máximo de lectura del ADC.
#define PWM_MIN          0       // Rango mínimo de escritura del PWM.
#define PWM_MAX        255       // Rango máximo de escritura del PWM. 
#define n               50       // Número de puntos de la média móvil. (250 para una respuesta más lenta y mejor filtrado)
//#define k             0.09       // Constante para cálculo de filtro exponencial (entre más pequeño es mejor filtra pero tarda más en estabilizarse)

//******************************************** Variables Globales *************************************************

//int dutyCycle    = 0;           // Variable global para leer los valores adquiridos por los canales ADC.
//float ValVoltaje = 0;           // Variable global para leer los valores adquiridos por los canales ADC.
//int adcRawAux    = 0;           // Variable global para leer los valores adquiridos por los canales ADC.
//int sensorValue = 0;         // Recibe el valor original filtrado.
const int _NUM_READ = 50;       // El número de promedios para el aritmo promedio. filtros

//********************************************** Inicio del Loop **************************************************
//adcRaw = analogRead(ADCPIN);                         // Lee el valor del potenciómetro (valor entre 0 y 4095)

//dutyCycle = expRunningAverage(median(adcRaw));              // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
//dutyCycle = expRunningAverageAdaptive(median(adcRaw));      // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.

//  serialPlotter();                        // Invoca la función que se encarga de organizar los datos para graficarlos en el serial plotter de Arduino.


class AnalogSensors
{
  public:
    AnalogSensors(int pin);
    int rawADCReadings();
    float rawVoltageValues( float resolution = 1024.0, float voltajeMax = 5.0 );                                     // Método que cálcula los valores de voltaje sin procesar de manera manual.
    float voltageCalculation( float newVal, float resolution = 1024.0, float voltajeMax = 5.0 );                     // Método que cálcula los valores de voltaje con o sin procesar de manera manual.
    long mapLong( long valuesX, long valInMin, long valInMax, long valOutMin, long valOutMax );                      // Método que cálcula los valores usando la función map (solo retorna enteros).
    float slopePoint( float valVoltaje = 0.0, float x1 = 0.0, float y1 = 0.0, float x2 = 0.0, float y2 = 0.0 );      // Método que cálcula el valor de 'Y' en la ecuación punto pendiente dados (X1,Y1) : (X2,Y2).

    float moving_average( float newVal );                // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.

    float midArifm2( float newVal );                     // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float runMiddleArifmBad( float newVal );             // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float runMiddleArifm( float newVal );                // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float runMiddleArifmOptim( float newVal );           // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float expRunningAverage( float newVal );             // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float expRunningAverageAdaptive( float newVal );     // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float median( float newVal );                        //
    float simpleKalman( float newVal, float _err_measure = 0.7, float _q = 0.09 );                            //
    float ABfilter( float newVal, float dt = 0.02, float sigma_process = 3.0, float sigma_noise = 0.7 );      // TIENE SOBRE IMPULSO HAY QUE AJUSTAR VALORES


  private:
    int _pin;                       // Pin GPIO como pin analógico utilizado para conectar un sensor analógico.
    int   _adcRawValues    = 0;     // Variable que almacena los valores sin procesar de ADC.
    float _rawValVoltaje   = 0.0;   //
    float _valVoltaje      = 0.0;   //
    long  _valueMapLong    = 0;     //
    float _valueSlopePoint = 0.0;   //

    int _numbers[n] = {0};          // Vector con los valores para el promedio móvil.

};

#endif /* ANALOGSENSOR_H */
