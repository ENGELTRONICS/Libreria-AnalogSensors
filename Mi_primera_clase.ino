/******************************************** Project information *************************************************
  Name Proyect: Creación e implementación de la primera clase
        Author: Ing.Felipe Andrés Ángel Rodríguez
          Data: 07/Julio/2020
           Rev: 1.0

  Description : Se realizo una clase aprender como se hace, esta consiste en la adquisión de datos (ADC) de un sensor
                analógico con el objetivo de saber como implementarse dicha clase.

******************************************************************************************************************/
//**************************************** LIBRERIAS A IMPLEMENTADAS **********************************************
#include  <Wire.h>
#include  <LiquidCrystal_I2C.h>
#include  "AnalogSensors.h"

//***************************************** Variables Macros PINOUT ***********************************************
// Configuración pinout del ESP32 para las conexiones con el proyecto:
#define ADCSENSOR1       33//36       // Pin GPIO analógico utilizado para conectar el sensor de presión 1.
#define ADCSENSOR2       39       // Pin GPIO analógico utilizado para conectar el sensor de presión 2.
#define ADCSENSOR3       34       // Pin GPIO analógico utilizado para conectar el sensor de presión 3.

//********************************************* Variables Macros **************************************************
// Configure the baud rate:
#define BAUDRATE     115200       // 
// Parámetros de configuración de biblioteca para el módulo I2C LCD:
#define LCD_COLS         16       // Indica a la librería el número de Columnas de la pantalla que tenemos conectada.
#define LCD_ROWS          2       // Indica a la librería el número de Filas de la pantalla que tenemos conectada.

#define LCD_ADDR       0x27       // Dirección del módulo I2C basado en PCF8574T. ( Otras direcciones comunes: 0x27, 0x3F, 0x20, 0x38 )
#define EN                2       // Enable bit
#define RW                1       // Read/Write bit
#define RS                0       // Register select bit

// Asignación de línea de datos LCD, esta biblioteca solo admite control LCD de 4 bits.
#define D4                4       // Pin GPIO digital del controlador configurado para la línea de datos LCD.
#define D5                5       // Pin GPIO digital del controlador configurado para la línea de datos LCD.
#define D6                6       // Pin GPIO digital del controlador configurado para la línea de datos LCD.
#define D7                7       // Pin GPIO digital del controlador configurado para la línea de datos LCD.
#define LCD_BACKLINGHPIN  3       // Pin GPIO digital del controlador configurado para encender o apagar la luz de fondo de la LCD.
#define POLARITY      POSITIVE    // Indica el estado de salida del pin (LCD_BACKLINGHPIN) para encender o apagar de la luz de fondo de la LCD.
//--------------------------------
//******************************************** Variables Globales *************************************************



//***************************************** Declaración de Objetos ************************************************
// Aqui se configuran los pines asignados a la pantalla del PCF8574
LiquidCrystal_I2C lcd(LCD_ADDR, EN, RW, RS, D4, D5, D6, D7, LCD_BACKLINGHPIN, POLARITY);

AnalogSensors Sensor1(ADCSENSOR1);


//**************************************** Declaración de Funciones ***********************************************


//************************************* Declaración de Interrupciones *********************************************


//**************************************** Inicio Programa Principal **********************************************
void setup() {
  Serial.begin( BAUDRATE );               // Inicializa la comunicación serial.
  //lcd.begin( LCD_COLS, LCD_ROWS );        // Inicializa la pantalla LCD.
}
//********************************************** Inicio del Loop **************************************************
void loop() {
  serialValue();
}

//********************************************* Inicio Funciones **************************************************
void parallelADCpins_SerialMonitor() {
  int Sensor1 = 0;                // Entero de 8 bit: Para leer los valores adquiridos por los canales ADC
  float ValVoltaje1 = 0;          // Valor Flotante:
  float Presion1 = 0;             // Valor Flotante:

  Sensor1 = analogRead(36);              // Lee el valor del potenciómetro (valor entre 0 y 4095)

  ValVoltaje1 = (Sensor1 * 3.3) / 4095;  // Convierte los valores del PWM a voltaje

  Presion1 = map(ValVoltaje1, 0.86, 1.37, 10876, 15537) ;   // ESP32 La función map cambia el rango de entrada de 0-4095 a 0-255 (el rango que soporta analogWrite)

  Serial.print("FUN VALUE VOLTAJE 1: ");                    // Imprime el mensaje en el Monitor Serie.
  Serial.print(ValVoltaje1);                                // Imprime el valor leído por el Monitor Serie.
  Serial.print("\tFUN VALUE PRESION 1: ");                  // Imprime el mensaje en el Monitor Serie.
  Serial.print(Presion1);                                   // Imprime el valor leido por el Monitor Serie.
  Serial.print(" mmH2O");                                   // Imprime el mensaje en el Monitor Serie.
  Serial.print("\tADC1_CH0: ");                             // Imprime el mensaje en el Monitor Serie.
  Serial.println(Sensor1);                                  // Imprime el valor leído por el Monitor Serie
  Serial.print("\n");

  delay(20);                                      // Tiempo de espera
}

void serialValue() {
  Serial.print("OBJ VALOR VOLTAJE 1: ");
  Serial.print(Sensor1.writeValueVoltaje());
  Serial.print(" ");

  Serial.print("\tOBJ VALOR PRESION 1: ");
  Serial.print(Sensor1.writeValuePressure(0.86, 1.37, 10876, 15537));
  Serial.print(" ");

  Serial.print(" mmH2O");                               // Imprime el mensaje en el Monitor Serie.
  Serial.print("\tADC1_CH0: ");                            // Imprime el mensaje en el Monitor Serie.
  Serial.println(Sensor1.rawADCReadings());


  // COMPARACION ENTRE OBJETO Y FUNCION PARA VERIFICACION:
  //parallelADCpins_SerialMonitor();
  delay(1000);                              //
}

/*
  void serialPlotter(){
  Serial.print("VOLTAJE:");                             // Los nombres deben ser cortos y sin espacios.
  Serial.print(Sensor1.writeValueVoltaje());            // Variable a plotearse.
  Serial.print(" ");                                    // Para plotear diferentes variables se separa con espacios.

  Serial.print("PRESION:");                             // Los nombres deben ser cortos y sin espacios.
  Serial.print(Sensor1.writeValuePressure(0.86, 1.37, 10876, 15537));   // Variable a plotearse.
  Serial.print(" ");                                    // Para plotear diferentes variables se separa con espacios.

  Serial.print("ADC1_CH0:");                            // Los nombres deben ser cortos y sin espacios.
  Serial.println(Sensor1.adcRaw());               // La ultima variable lleva el println para que se efectue correctamente el ploteo de variables.
  delay(1000);                                          // Tiempo de ploteo puede variar o en su defecto omitirse segun se requiera.
  }
*/
