/******************************************** Project information *************************************************
  Name Proyect: Librería AnalogSensors
        Author: Ing.Felipe Andrés Ángel Rodríguez
          Data: 10/Julio/2020
           Rev: 1.0

  Description : Esta librería se encarga de leer los valores de un sensor analogico, tambien se implementan 
                diferentes filtros para el procesamiento de datos 
******************************************************************************************************************/

#ifndef ANALOGSENSOR_H
#define ANALOGSENSOR_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//********************************************* Variables Macros **************************************************
#define RESOLUTION     4095       // Constante para definir la escala a la que se realizará la conversión del ADC.
#define VOLTAJEMAX      3.3       // Constante de voltaje para realizar los cálculos del ADC.
#define TIMERADQ         20       // Tiempo de adquisición en milisegundos.

//********************************************* Variables Macros **************************************************
#define RANGE_MIN        0       // Rango mínimo de lectura del ADC.
#define RANGE_MAX     4095       // Rango máximo de lectura del ADC.
#define PWM_MIN          0       // Rango mínimo de escritura del PWM.
#define PWM_MAX        255       // Rango máximo de escritura del PWM. 
#define VOLTAJE_MAX    3.3       // 
#define TIMERADQ        20       // Tiempo de adquisición
#define n               50       // Número de puntos de la média móvil. (250 para una respuesta más lenta y mejor filtrado)
//#define k             0.09       // Constante para cálculo de filtro exponencial (entre más pequeño es mejor filtra pero tarda más en estabilizarse)

//******************************************** Variables Globales *************************************************

int adcRaw       = 0;           // Variable global para leer los valores adquiridos por los canales ADC.
int dutyCycle    = 0;           // Variable global para leer los valores adquiridos por los canales ADC.
float ValVoltaje = 0;           // Variable global para leer los valores adquiridos por los canales ADC.

int adcRawAux    = 0;           // Variable global para leer los valores adquiridos por los canales ADC.
//int sensorValue = 0;         // Recibe el valor original filtrado.
int numbers[n] = {0};           // Vector con los valores para el promedio móvil.

//********************************************** Inicio del Loop **************************************************
void loop() {
  adcRaw = analogRead(ADCPIN);                         // Lee el valor del potenciómetro (valor entre 0 y 4095)

  //dutyCycle = expRunningAverage(median(adcRaw));              // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
  //dutyCycle = expRunningAverageAdaptive(median(adcRaw));      // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.

  serialPlotter();                        // Invoca la función que se encarga de organizar los datos para graficarlos en el serial plotter de Arduino.
}




class AnalogSensor
{
  public:
    AnalogSensor(int pin);
    int rawADCReadings();
    float writeValueVoltaje();
    float writeValuePressure(float valPressureMin, float valPressureMax, float valRangeMin, float valRangeMax);
    
    float moving_average(float newVal);                                    // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.

    float midArifm2(float newVal);                     // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float runMiddleArifmBad(float newVal);             // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float runMiddleArifm(float newVal);                // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float runMiddleArifmOptim(float newVal);           // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float expRunningAverage(float newVal);             // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float expRunningAverageAdaptive(float newVal);     // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
    float median(float newVal);                        //
    float simpleKalman(float newVal);                  //
    float ABfilter(float newVal);                      // TIENE SOBRE IMPULSO HAY QUE AJUSTAR VALORES



  private:
    int _pin;                       // Pin GPIO como pin analógico utilizado para conectar un sensor analógico.
    int   _adcRawValues  = 0;       // Variable que almacena los valores sin procesar de ADC.
    float _valVoltaje    = 0.0;
    float _valuePressure = 0.0;


};

#endif /* ANALOGSENSOR_H */
