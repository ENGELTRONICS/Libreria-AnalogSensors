#include "AnalogSensors.h"

AnalogSensors::AnalogSensors(int pin) {   //
  pinMode(pin, INPUT);                    // Configura el pin analógico GPIO como entrada.
  _pin = pin;                             //
}

int AnalogSensors::rawADCReadings() {     // Método que adquiere los valores del ADC sin procesar.
  _adcRawValues = analogRead(_pin);       // Lee el valor del potenciómetro (valor entre 0 y 4095)
  delay(TIMERADQ);                        // Tiempo de adquisición en milisegundos.
  return _adcRawValues;                   // Retorna el valor que adquiere el ADC.
}

float AnalogSensors::voltageCalculation(float newVal, float resolution, float voltajeMax) {                      // Método que cálcula los valores de voltaje con o sin procesar de manera manual.
  if ( resolution < 200 ) {
    resolution = pow( 2 , resolution );    // Sintaxis pow(base, exponent)
  }

  _valVoltaje = (_adcRawValues * voltajeMax) / resolution;    // Convierte los valores del ADC a valores de voltaje.
  return _valVoltaje;                                         // Retorna el valor del voltaje cálculado sin filtros de forma manual.
}

float AnalogSensors::rawVoltageValues(float resolution, float voltajeMax) {                      // Método que cálcula los valores de voltaje sin procesar de manera manual.

  _rawValVoltaje = voltageCalculation(rawADCReadings(), resolution, voltajeMax);

  return _rawValVoltaje;                                         // Retorna el valor del voltaje cálculado sin filtros de forma manual.
}

long AnalogSensors::mapLong( long valuesX, long valInMin, long valInMax, long valOutMin, long valOutMax ) {     // Método que cálcula los valores usando la función map (solo retorna enteros).

  _valueMapLong = map( valuesX, valInMin, valInMax, valOutMin, valOutMax );      // ESP32 La función map cambia el rango de entrada a un valor mínimo y máximo deseado.
  return _valueMapLong;                                                          // Retorna el valor cálculado.
}

float AnalogSensors::slopePoint( float valVoltaje, float x1, float y1, float x2, float y2 ) {                // Método que cálcula el valor de 'Y' en la ecuación punto pendiente dados (X1,Y1) : (X2,Y2).

  _valueSlopePoint = (((y2 - y1) / (x2 - x1)) * (valVoltaje - x1)) + y1;    // Cálculo de la ecuación punto pendiente para recta de sensores líneales.
  return _valueSlopePoint;                                           // Retorna el valor cálculado.
}

//********************************************* Inicio Funciones **************************************************
float AnalogSensors::moving_average(float newVal) {      // Método que cálcula el filtro de media móvil para la lectura del ADC.
  for (int i = n - 1; i > 0; i--) {       //
    _numbers[i] = _numbers[i - 1];          // Desplaza los elementos del vector de media móvil.
  }

  _numbers[0] = newVal;                    // Posición inicial del vector recibe la lectura original.
  long acc = 0;                           // Acumulador para sumar los puntos de la media móvil.

  for (int i = 0; i < n; i++) {           //
    acc += _numbers[i];                    // Hace la suma del número de puntos.
  }

  return acc / n;                         // Retorna la média móvil
}

// LINK VIDEO EXPLICACION DE FILTROS: https://www.youtube.com/watch?v=R6yvl90TiI8
// LINK DEL REPOSITORIO: https://github.com/AlexGyver/tutorials

// media aritmética normal
//float midArifm() {
//  float sum = 0;                      // локальная переменная sum
//  for (int i = 0; i < _NUM_READ; i++)  // согласно количеству усреднений
//    sum += getSignal();               // суммируем значения с любого датчика в переменную sum
//  return (sum / _NUM_READ);
//}

// significado aritmetico
float AnalogSensors::midArifm2(float newVal) {   //
  static byte counter = 0;              //
  static float prevResult = 0;          //
  static float sum = 0;                 //
  sum += newVal;                        //
  counter++;                            //
  if (counter == _NUM_READ) {           //
    prevResult = sum / _NUM_READ;       //
    sum = 0;                            //
    counter = 0;                        //
  }
  return prevResult;                    //
}

// la media aritmética es generalmente no óptima
float AnalogSensors::runMiddleArifmBad(float newVal) {  // adquiere un nuevo significado
  static float valArray[_NUM_READ];         // una matriz

  for (int i = 0; i < _NUM_READ - 1; i++)   //
    valArray[i] = valArray[i + 1];          //

  valArray[_NUM_READ - 1] = newVal;         // escribe uno nuevo en la última celda
  float average = 0;                        // la media

  for (int i = 0; i < _NUM_READ; i++)       //
    average += valArray[i];                 // resumir

  return (float)average / _NUM_READ;        // regreso
}

// significado aritmetico
float AnalogSensors::runMiddleArifm(float newVal) {      // adquiere un nuevo significado
  static byte idx = 0;                     // índice
  static float valArray[_NUM_READ];        // Vector para almacenar los valores.
  valArray[idx] = newVal;                  // Escribir cada vez a una nueva celda.
  if (++idx >= _NUM_READ) idx = 0;         // Sobrescribir el valor más antiguo.
  float average = 0;                       // Inicializa el promedio en cero.
  for (int i = 0; i < _NUM_READ; i++) {    //
    average += valArray[i];                // Resumir
  }
  return (float)average / _NUM_READ;       // Regreso
}

// media aritmética de carrera óptima
float AnalogSensors::runMiddleArifmOptim(float newVal) {
  static int t = 0;                        //
  static int vals[_NUM_READ];              //
  static int average = 0;                  //
  if (++t >= _NUM_READ) t = 0;             // Rebobinar t
  average -= vals[t];                      // Restar lo viejo
  average += newVal;                       // Agregar nuevo
  vals[t] = newVal;                        // Recuerde ordenar
  return ((float)average / _NUM_READ);     // Retorna el valor dividido por el número de promedio.
}


float AnalogSensors::expRunningAverage(float newVal) {    // Función que cálcula el filtro promedio de funcionamiento, una versión aún más óptima del filtro anterior para la lectura del ADC.
  static long filVal = 0;                  //
  filVal += (newVal - filVal) * 0.09;      // Filtro exponencial
  return filVal;                           // Retorna el valor cálculado del filtro.
}

// coeficiente adaptativo corriente promedio
float AnalogSensors::expRunningAverageAdaptive(float newVal) {
  static float filVal = 0;
  float k;

  if (abs(newVal - filVal) > 1.5) k = 0.9;   // La nitidez del filtro depende del módulo de la diferencia de valores.
  else k = 0.03;

  filVal += (newVal - filVal) * k;
  return filVal;
}

float AnalogSensors::median(float newVal) {   //
  static float buf[3];                        //
  static byte count = 0;                      //
  buf[count] = newVal;                        //
  if (++count >= 3) count = 0;                //

  float a = buf[0];                           //
  float b = buf[1];                           //
  float c = buf[2];                           //

  float middle;                               //
  if ((a <= b) && (a <= c)) {                 //
    middle = (b <= c) ? b : c;                //
  } else {                                    //
    if ((b <= a) && (b <= c)) {               //
      middle = (a <= c) ? a : c;              //
    } else {                                  //
      middle = (a <= b) ? a : b;              //
    }                                         //
  }
  return middle;                              //
}


// Kalman simplificado más o menos como:
//float _err_measure = 0.7;  // Ruido de medición aproximado
//float _q = 0.09;           // tasa de cambio de valores 0.001-1, varíe usted mismo

float AnalogSensors::simpleKalman(float newVal, float _err_measure, float _q) {
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;

  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;

  return _current_estimate;
}


// período de muestreo (mediciones), variación del proceso, variación de ruido
/*
  float dt = 0.02;
  float sigma_process = 3.0;
  float sigma_noise = 0.7;
*/
float AnalogSensors::ABfilter(float newVal, float dt, float sigma_process, float sigma_noise) {
  static float xk_1, vk_1, a, b;
  static float xk, vk, rk;
  static float xm;

  float lambda = (float)sigma_process * dt * dt / sigma_noise;
  float r = (4 + lambda - (float)sqrt(8 * lambda + lambda * lambda)) / 4;
  a = (float)1 - r * r;
  b = (float)2 * (2 - a) - 4 * (float)sqrt(1 - a);

  xm = newVal;
  xk = xk_1 + ((float) vk_1 * dt );
  vk = vk_1;
  rk = xm - xk;
  xk += (float)a * rk;
  vk += (float)( b * rk ) / dt;
  xk_1 = xk;
  vk_1 = vk;
  return xk_1;
}
