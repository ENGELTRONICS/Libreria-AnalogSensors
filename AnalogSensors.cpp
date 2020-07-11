#include "AnalogSensor.h"

AnalogSensor::AnalogSensor(int pin)
{
  pinMode(pin, INPUT);
  _pin = pin;
}

int AnalogSensor::rawADCReadings() {      // Lecturas del ADC sin procesar.
  _valueADC = analogRead(_pin);           // Lee el valor del potenciómetro (valor entre 0 y 4095)
  delay(TIMERADQ);                        // Tiempo de adquisición en milisegundos.

  return _valueADC;
}

float AnalogSensor::writeValueVoltaje() {
  rawADCReadings();
  _valVoltaje = (_valueADC * VOLTAJEMAX) / RESOLUTION;      // Convierte los valores del ADC a valores de voltaje.

  return _valVoltaje;
}

float AnalogSensor::writeValuePressure(float valPressureMin, float valPressureMax, float valRangeMin, float valRangeMax) {
  writeValueVoltaje();
  _valuePressure = map( _valVoltaje, valPressureMin, valPressureMax,
                        valRangeMin, valRangeMax );                    // ESP32 La función map cambia el rango de entrada a un valor minimo y maximo deseado.

  return _valuePressure;
}






//***************************************** Variables Macros PINOUT ***********************************************
// Configuración pinout del ESP32 para conexiones con el proyecto:

#define LEDPIN          2       // Pin GPIO como pin digital que incorpora el led indicar
#define ADCPIN         33       // Pin GPIO como pin analógico utilizado para conectar el potenciómetro

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
String tab_space = "\t\t";      //

int adcRaw       = 0;           // Variable global para leer los valores adquiridos por los canales ADC.
int dutyCycle    = 0;           // Variable global para leer los valores adquiridos por los canales ADC.
float ValVoltaje = 0;           // Variable global para leer los valores adquiridos por los canales ADC.

int adcRawAux    = 0;           // Variable global para leer los valores adquiridos por los canales ADC.
//int sensorValue = 0;         // Recibe el valor original filtrado.
int numbers[n] = {0};           // Vector con los valores para el promedio móvil.

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

//***************************************** Declaración de Objetos ************************************************

//**************************************** Declaración de Funciones ***********************************************
long moving_average();          // Función que cálcula el filtro de media móvil para la lectura del ADC.

//**************************************** Inicio Programa Principal **********************************************
void setup() {
  Serial.begin(115200);         // Initializa la comunicación serial a 19200 bits por segundos
  pinMode(ADCPIN, INPUT);      // Initialize analog pin as an input.
  pinMode(LEDPIN, OUTPUT);     // Initialize digital pin digital as an output.

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(LEDPIN, ledChannel);
}

//********************************************** Inicio del Loop **************************************************
void loop() {
  adcRaw = analogRead(ADCPIN);                         // Lee el valor del potenciómetro (valor entre 0 y 4095)
  adcRawAux = adcRaw;

  //dutyCycle = expRunningAverage(median(adcRaw));              // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
  //dutyCycle = expRunningAverageAdaptive(median(adcRaw));      // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.

  //dutyCycle = midArifm2(adcRaw);                     // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
  //dutyCycle = runMiddleArifmBad(adcRaw);             // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
  //dutyCycle = runMiddleArifm(adcRaw);                // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
  //dutyCycle = runMiddleArifmOptim(adcRaw);           // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.

  //dutyCycle = expRunningAverage(adcRaw);             // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
  //dutyCycle = expRunningAverageAdaptive(adcRaw);     // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.
  //dutyCycle = median(adcRaw);                        //
  dutyCycle = simpleKalman(adcRaw);                  //
  //dutyCycle = ABfilter(adcRaw);                      // TIENE SOBRE IMPULSO HAY QUE AJUSTAR VALORES

  //dutyCycle = moving_average(adcRaw);                                    // Invoca la función que cálcula el filtro de media móvil para la lectura del ADC.

  ValVoltaje = (dutyCycle * VOLTAJE_MAX) / RANGE_MAX;                    // Convierte los valores del PWM a voltaje
  dutyCycle = map(dutyCycle, RANGE_MIN, RANGE_MAX, PWM_MIN, PWM_MAX) ;   // ESP32 La función map cambia el rango de entrada de 0-4095 a 0-255 (el rango que soporta analogWrite)

  ledcWrite(ledChannel, dutyCycle);       // changing the LED brightness with PWM
  delay(15);                              //

  serialPlotter();                        // Invoca la función que se encarga de organizar los datos para graficarlos en el serial plotter de Arduino.
}

//********************************************* Inicio Funciones **************************************************
long moving_average(long newVal) {                   // Función que cálcula el filtro de media móvil para la lectura del ADC.
  for (int i = n - 1; i > 0; i--) {
    numbers[i] = numbers[i - 1];          // Desplaza los elementos del vector de media móvil.
  }

  numbers[0] = newVal;                    // Posición inicial del vector recibe la lectura original.
  long acc = 0;                           // Acumulador para sumar los puntos de la media móvil.

  for (int i = 0; i < n; i++) {
    acc += numbers[i];                    // Hace la suma del número de puntos.
  }

  return acc / n;                         // Retorna la média móvil
}

// LINK VIDEO EXPLICACION DE FILTROS: https://www.youtube.com/watch?v=R6yvl90TiI8
// LINK DEL REPOSITORIO: https://github.com/AlexGyver/tutorials

const int NUM_READ = 50;  // El número de promedios para el aritmo promedio. filtros

// media aritmética normal
//float midArifm() {
//  float sum = 0;                      // локальная переменная sum
//  for (int i = 0; i < NUM_READ; i++)  // согласно количеству усреднений
//    sum += getSignal();               // суммируем значения с любого датчика в переменную sum
//  return (sum / NUM_READ);
//}

// significado aritmetico
float midArifm2(float newVal) {
  static byte counter = 0;
  static float prevResult = 0;
  static float sum = 0;
  sum += newVal;
  counter++;
  if (counter == NUM_READ) {
    prevResult = sum / NUM_READ;
    sum = 0;
    counter = 0;
  }
  return prevResult;
}

// la media aritmética es generalmente no óptima
float runMiddleArifmBad(float newVal) {  // adquiere un nuevo significado
  static float valArray[NUM_READ];       // una matriz

  for (int i = 0; i < NUM_READ - 1; i++)
    valArray[i] = valArray[i + 1];

  valArray[NUM_READ - 1] = newVal;    // escribe uno nuevo en la última celda
  float average = 0;                  // la media

  for (int i = 0; i < NUM_READ; i++)
    average += valArray[i];           // resumir

  return (float)average / NUM_READ;   // regreso
}

// significado aritmetico
float runMiddleArifm(float newVal) {  // принимает новое значение
  static byte idx = 0;                // индекс
  static float valArray[NUM_READ];    // массив
  valArray[idx] = newVal;             // пишем каждый раз в новую ячейку
  if (++idx >= NUM_READ) idx = 0;     // перезаписывая самое старое значение
  float average = 0;                  // обнуляем среднее
  for (int i = 0; i < NUM_READ; i++) {
    average += valArray[i];           // суммируем
  }
  return (float)average / NUM_READ;   // возвращаем
}

// media aritmética de carrera óptima
float runMiddleArifmOptim(float newVal) {
  static int t = 0;
  static int vals[NUM_READ];
  static int average = 0;
  if (++t >= NUM_READ) t = 0; // перемотка t
  average -= vals[t];         // вычитаем старое
  average += newVal;          // прибавляем новое
  vals[t] = newVal;           // запоминаем в массив
  return ((float)average / NUM_READ);
}


long expRunningAverage(long newVal) {                 // Función que cálcula el filtro promedio de funcionamiento, una versión aún más óptima del filtro anterior para la lectura del ADC.
  static long filVal = 0;
  filVal += (newVal - filVal) * 0.09;      // Filtro exponencial
  return filVal;                           // Retorna el valor cálculado del filtro.
}

// coeficiente adaptativo corriente promedio
long expRunningAverageAdaptive(long newVal) {
  static float filVal = 0;
  float k;
  // резкость фильтра зависит от модуля разности значений
  if (abs(newVal - filVal) > 1.5) k = 0.9;
  else k = 0.03;

  filVal += (newVal - filVal) * k;
  return filVal;
}

long median(long newVal) {
  static float buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3) count = 0;

  float a = buf[0];
  float b = buf[1];
  float c = buf[2];

  float middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  } else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    } else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}


// Kalman simplificado más o menos como:
float _err_measure = 0.7;  // Ruido de medición aproximado
float _q = 0.09;           // tasa de cambio de valores 0.001-1, varíe usted mismo

long simpleKalman(long newVal) {
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
float dt = 0.02;
float sigma_process = 3.0;
float sigma_noise = 0.7;

long ABfilter(long newVal) {
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


void serialPlotter() {        // Función que se encarga de organizar los datos para graficarlos en el serial plotter de Arduino.
  adcRawAux = map(adcRaw, RANGE_MIN, RANGE_MAX, PWM_MIN, PWM_MAX) ;   // ESP32 La función map cambia el rango de entrada de 0-4095 a 0-255 (el rango que soporta analogWrite)
  Serial.print("ADCRAW:");                // Imprime el palabra en el Monitor Serie
  Serial.print(adcRawAux);                // Imprime el valor leido por el Monitor Serie
  Serial.print(" ");                      // Imprime el palabra en el Monitor Serie
  Serial.print("ADCFILTER:");             // Imprime el palabra en el Monitor Serie
  Serial.println(dutyCycle);              // Imprime el valor leido por el Monitor Serie
  //Serial.print(" ");                      // Imprime el palabra en el Monitor Serie
  //Serial.print("VOLTAJE:");               // Imprime el palabra en el Monitor Serie
  //Serial.println(ValVoltaje);             // Imprime el valor leido por el Monitor Serie
}
