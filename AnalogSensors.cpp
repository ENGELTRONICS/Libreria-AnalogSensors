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
