/*
Update 29/11/22  : añadido el acelerometro como 3 ultimos valores X Y Z

 *
 * Update 2/5/21 : suspendi el filtro delta en el sensor de distancia
 *                 anda bien pero pablo pidio datos 100% crudos
 * 
Timer: se setea una frecuencia de muestreo, y a cada interrupcion del timer se sube y baja el pin de clock
con ancho de pulso segun el delayMicroseconds

Sensores: confio un poquito mas en lo que reporta el sensor sin esmalte... chequear..
la calibracion se hara con el limpio como referencia (en principio el interno...)
*/

boolean DEBUG = false;     // imprime el booteo
#define ESPERA_BOOTEO 500   // en ms por cada inicializcion

// timer y control
#define SAMPLE_RATE 100   // en HZ
#define PRESCALER 8   // para 100 hz no daria el OCR de 16 bits sin preescalar
#define PULSO_LARGO 500   // en uS 
#define PULSO_CORTO 200   // en uS --- el tamaño del pulso determina el nivel que aparece de audio despues del capacitor...
const int pinDeSalida = 5;   // salida del pin de clock por pulsos de "audio"
//const int puertoSalida = DDC6;   // DDC6 es el pin 5, no es lo mismo el pin del arduino que el pin del chip
boolean hacer_pulso = false;
boolean primer_pulso = true;
boolean funcionando = false;
boolean ultimo_pulso = false;
boolean apagar = true;
float tiempoTranscurrido = 0.0;
float intervaloTiempo = 0.0;  // se calcula en setup!
const int CONTAR_HASTA = 3;
int contador_clock = 0;

// caracteres de control serial
const char SEP = 9;           // 9 es el tabulador, TAB    44 es la coma ,
const char INICIO = 33;       // 33 es el signo de exclamacion, !
const char LF = 10;           // ASCII salto de linea
const char REQ = 63;          // ? ASCII para requerir comunicacion
const char ACK = 64;          // ASCII que debiera mandar arduino al responder
const char PIDE = 49;         // 1
const char DETIENE = 48;      // 0
const char CALIBRA = 61;      // = para pedir re-calibracion e igualar sensores de presion
const char DESCALIBRA = 60;   // < para pedir anular la calibracion de sensores presion
const char OK_CALIB = 43;     // + para confirmar que se calibró
const char OK_DESCALIB = 45;  // - para confirmar que se DEScalibró
const char PEDIR_FILA = 35;   // # para solicitar fila de datos cuando NO esta andando

boolean confirmar_conexion = false;

// contador de vueltas ranurado
#define PIN_INTERRUPCION 7
#define PIN_DIRECCION 8
volatile int contadorRueda = 0;

// pulsador
#include <Bounce2.h>
#define PIN_BOTON 9
Bounce pulsador = Bounce(); 

// comandos serie
byte recibido = 0;

// sensores de presion
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
float offset_presion = 0.0; // diferencia calibrable entre sensores, no se a que se debe
Adafruit_BME280 bmeA; // I2C
Adafruit_BME280 bmeB; // I2C
float presionAdentro = 0;
float presionAtmosferica = 0;
boolean debo_calibrar = false;
boolean debo_descalibrar = false;
boolean usando_calibracion = false;

// acelerometro
//#include <basicMPU6050.h> 
//basicMPU6050<> mpu;

// ADAFRUIT (muy lenta)
#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
//#include <Wire.h>
Adafruit_MPU6050 mpu;

float aceleracionX = 0;
float aceleracionY = 0;
float aceleracionZ = 0;




// incluyo temperaturas referencia (enero 2021)
float tempAdentro = 0;
float tempAfuera = 0;

// sensor de distancia local
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X laser;
uint16_t distancia = 0;
float distancia_float_0 = 0.0;
float distancia_float_1 = 0.0;

void setup() {
  Serial.begin(115200);
  intervaloTiempo = 1.0 / SAMPLE_RATE;
  
  if (DEBUG) { 
    for (int i = 0; i < 10; i++){
      Serial.println("Arranca " + String(i));
      delay(ESPERA_BOOTEO); // margen de tiempo
    }
  }
  
  if (DEBUG) Serial.println("iniciar_timer");
  iniciar_timer();
  if (DEBUG) delay(ESPERA_BOOTEO); // margen de tiempo

  if (DEBUG) Serial.println("iniciar_BME");
  iniciar_BME();
  if (DEBUG) delay(ESPERA_BOOTEO); // margen de tiempo
  
  if (DEBUG) Serial.println("iniciar_pulsador");
  iniciar_pulsador();
  if (DEBUG) delay(ESPERA_BOOTEO); // margen de tiempo

  if (DEBUG) Serial.println("iniciar_distancia");
  iniciar_distancia();
  if (DEBUG) delay(ESPERA_BOOTEO); // margen de tiempo

  if (DEBUG) Serial.println("iniciar_rueda");
  iniciar_rueda();
  if (DEBUG) delay(ESPERA_BOOTEO); // margen de tiempo

  if (DEBUG) Serial.println("iniciar_acelerometro");
  iniciar_acelerometro();
  if (DEBUG) delay(ESPERA_BOOTEO); // margen de tiempo  

  Wire.setClock(400000);

  if (DEBUG) Serial.println("terminada inicializacion");
    
  if (DEBUG) delay(ESPERA_BOOTEO); // margen de tiempo
}

void loop() {

  // PRIMER PULSO --------------------------------
  if (hacer_pulso && primer_pulso && funcionando){
    contadorRueda = 0;
    tiempoTranscurrido = 0.0;
    pulsar(PULSO_LARGO);
      enviar_encabezados();
      leer_presiones();
      leer_temperaturas();
      leer_distancia();
      leer_acelerometro();
      enviar_valores();
    hacer_pulso = false;
    primer_pulso = false;
  }
  
  // DEMAS PULSOS ---------------------------------
  if (hacer_pulso && !primer_pulso && funcionando){
    pulsar(PULSO_CORTO);
      leer_presiones();
      leer_temperaturas();
      leer_distancia();
      leer_acelerometro();
      tiempoTranscurrido += intervaloTiempo;
      enviar_valores();
    hacer_pulso = false;
  }

  // RESPONDER CONFIRMACION DE CONEXION ----------
  if (confirmar_conexion) {
    Serial.print(ACK);
    confirmar_conexion = false;
  }

  if (debo_calibrar) calibrarPresion();
  if (debo_descalibrar) anularCalibracionPresion();

  leerPulsador();
  leer_serial();
}

void iniciar_timer() {
  pinMode(pinDeSalida, OUTPUT);

  // usar el Timer 1 que es de 16 bit
  TCCR1A = (TCCR1A | _BV(COM1A1)) & ~_BV(COM1A0); // CTC
  TCCR1A &= ~(_BV(WGM10) | _BV(WGM11)); // waveform ctc
  TCCR1B = (TCCR1B | _BV(WGM12)) & ~_BV(WGM13); // waveform ctc
  
  TCCR1B = (TCCR1B | _BV(CS10)) & ~(_BV(CS11) | _BV(CS12)); // sin prescaler

  //TCCR1B = (TCCR1B | _BV(CS31)) & ~(_BV(CS30) | _BV(CS32)); // prescaler /8
  //TCCR1B = (TCCR1B | _BV(CS32)) & (TCCR1B | _BV(CS31)) & ~(_BV(CS31)); // prescaler /1024
  
  cli();
  
  // la formula seria aproximadamente 16mhz / OCR1A / prescaler (1) / contar_hasta
  // como OCR1A no puede contar mas de 16 bit (65535), subdivido el numero real que necesito
  // con el contar_hasta, se puede aumentar un poco la resolucion bajando OCR1A
  OCR1A = 53330; // leonardo pro micro
  
  //19987; MEGA //(F_CPU / SAMPLE_RATE) / PRESCALER;  // ojo que no se pase de 32k
  
  sei();
  
  // Se activa la interrupcion segun el SAMPLE_RATE
  TIMSK1 |= _BV(OCIE1A);  // (esta andando)
}

void leer_serial() {
  if (Serial.available() > 0) {
    recibido = Serial.read();

    // caracter "0" para detener
    if (recibido == DETIENE) {
       funcionando = false;
       hacer_pulso = false; // no hay que hacer pulsos...
       primer_pulso = true; // la proxima iniciará con pulso largo
       //Serial.flush();  // para ver si limpia el buffer y no queda mugre pendiente
    }
    
    // caracter "1" para iniciar
    if (recibido == PIDE) {
       contadorRueda = 0;
       tiempoTranscurrido = 0.0;
       primer_pulso = true; // la proxima iniciará con pulso largo
       funcionando = true;
    }

    // caracter "@" para confirmar conexion
    if (recibido == REQ && funcionando == false) {
       confirmar_conexion = true;
       offset_presion = 0.0;
       usando_calibracion = false;
    }

    // caracter "=" para pedir calibracion
    if (recibido == CALIBRA && funcionando == false) {
       debo_calibrar = true;
    }

    // caracter ">" para anular calibracion
    if (recibido == DESCALIBRA && funcionando == false) {
       debo_descalibrar = true;
    }
    
    // caracter "#" para solicitar una fila de datos mientras NO esta corriendo
    if (recibido == PEDIR_FILA && funcionando == false) {
      //enviar_encabezados();
      leer_presiones();
      leer_temperaturas();
      leer_distancia();
      leer_acelerometro();
      enviar_valores();
    }
  }  
}

void leerPulsador(){
  pulsador.update();

  if (pulsador.fell()) {  // Al soltar? el pulsador
     funcionando = !funcionando; // Alternar
  }
  if (!funcionando) {
     hacer_pulso = false; // no hay que hacer pulsos...
     primer_pulso = true; // la proxima iniciará con pulso largo
  }

}

ISR(TIMER1_COMPA_vect) {
  // interrupcion por el timer
  //hacer_pulso = true; // no hay que hacer nada mas aca...

  contador_clock++;
  
  if (contador_clock >= CONTAR_HASTA){
    hacer_pulso = true;
    contador_clock = 0;
  }
  
}

void pulsar(int anchoMicrosegundos){
    // chequear el pin exacto...!
    // PORTC<<6 es el pin 5
    PORTC |= _BV(6);  // alto
    delayMicroseconds(anchoMicrosegundos);
    PORTC &= ~_BV(6); // bajo  
}

void iniciar_pulsador(){
  pinMode(PIN_BOTON, INPUT_PULLUP);   // pulsador de iniciar/detener
  pulsador.attach(PIN_BOTON);
  pulsador.interval(1); // interval in ms  
}

void iniciar_BME() {
  
    // Serial.println(F("BME280A test"));  // estorba al puerto serie

    unsigned status;
    
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bmeA.begin(0x76);    // el que tiene SDO a GND (pull down onboard)
    if (!status) {
        Serial.println("No funciona el sensor de presion de adentro!" + SEP);
        while (1);
    }

    Serial.println(F("BME280B test"));
    status = bmeB.begin(0x77);    // el que tiene SDO a vcc
    if (!status) {
        Serial.println("No funciona el sensor de presion de afuera!" + SEP);
        while (1);
    }
    
    /*  
    setSampling(sensor_mode mode = MODE_NORMAL,
                   sensor_sampling tempSampling = SAMPLING_X16,
                   sensor_sampling pressSampling = SAMPLING_X16,
                   sensor_sampling humSampling = SAMPLING_X16,
                   sensor_filter filter = FILTER_OFF,
                   standby_duration duration = STANDBY_MS_0_5);
    */

    // pareciera que si no esta activada la temperatura, la presion se mide mal, tal vez no la compensa
    // chequear en el datasheet
    //                modo              temperatura       presion           humedad             filtro          standby (ms)
    bmeA.setSampling(bmeA.MODE_NORMAL, bmeA.SAMPLING_X1, bmeA.SAMPLING_X1, bmeA.SAMPLING_NONE, bmeA.FILTER_X2, bmeA.STANDBY_MS_0_5);
    
    // prueba a ver si se empatan mejor con el mismo rate y filtros
    //bmeB.setSampling(bmeB.MODE_NORMAL, bmeB.SAMPLING_X1, bmeB.SAMPLING_X16, bmeB.SAMPLING_NONE, bmeB.FILTER_X16, bmeB.STANDBY_MS_0_5);
    bmeB.setSampling(bmeB.MODE_NORMAL, bmeB.SAMPLING_X1, bmeB.SAMPLING_X1, bmeB.SAMPLING_NONE, bmeB.FILTER_X2, bmeB.STANDBY_MS_0_5);
    
    // Serial.println("TODO OK");  // estorba al puerto serie
}

void iniciar_distancia(){
  Wire.begin();
  laser.init();
  //sensor.setTimeout(50);
  laser.startContinuous();
  laser.setMeasurementTimingBudget(30000);  // uS, el minimo es 20k, pero da demasiado ruido
}

void iniciar_rueda(){
  pinMode(PIN_INTERRUPCION, INPUT);
  pinMode(PIN_DIRECCION, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPCION), interrupcionRueda, RISING);
}


void iniciar_acelerometro(){

/*
  // Set registers - Always required
  mpu.setup();
  // Initial calibration of gyro
  //mpu.setBias();
*/
  
  // ADAFRUIT (es muy lenta)
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Fallo el inicio del acelerometro!");
    while (1) {
      delay(10);
    }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);      //(MPU6050_BAND_5_HZ);      //    //(MPU6050_BAND_21_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);  //(MPU6050_HIGHPASS_5_HZ);         //(MPU6050_HIGHPASS_DISABLE);
  //mpu.setCycleRate(MPU6050_CYCLE_40_HZ);            //(MPU6050_CYCLE_1_25_HZ);
  mpu.enableCycle(false);

  mpu.setGyroStandby(true, true, true);   // x y z
  mpu.setTemperatureStandby(true);
  
}

void leer_acelerometro() {

  //-- Scaled and calibrated output:
  // Accel
  //aceleracionX = mpu.ax();
  //aceleracionY = mpu.ay();
  //aceleracionZ = mpu.az();

  
  // ADAFRUIT (es muy lenta)
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  aceleracionX = a.acceleration.x;
  aceleracionY = a.acceleration.y;
  aceleracionZ = a.acceleration.z;
  
}


void interrupcionRueda() {
  // condensado en una sola linea, si hay 1 en el PIN_DIRECCION suma, si no, resta
  // PINB es el puerto B de pines digitales
  // PINB<<4 es el pin 8
  (PINB & (1<<4))? contadorRueda-- : contadorRueda++;
}

void leer_presiones() {
  presionAdentro = bmeA.readPressure()/100.0;
  presionAtmosferica = bmeB.readPressure()/100.0;
}


void leer_temperaturas() {
  tempAdentro = bmeA.readTemperature();
  tempAfuera = bmeB.readTemperature();
}

void leer_distancia() {
  // El sensor requiere una calibracion, hice los calculos (excel) y da aprox este mapeo
  // Utilizando melamina blanca como superficie reflectiva
  // Ecuacion de la recta estimada:
  //      Y = 1.05X + 4.5
  // Invirtiendo las variables y re-despejando, da aproximadamente:
  //      Y = 0.95238X - 4.28571
  // Asi y todo... en la realidad ajusta mejor manualmente, basado en esos numeros
  // La "longitud" interna del volumen del fuelle, sería aproximadamente
  //      650 - distancia 
  
  distancia = laser.readReg16Bit(laser.RESULT_RANGE_STATUS + 10);
  distancia_float_0 = 650 - ((0.95 * distancia) - 2);   // con calibracion de recta estimada
  
  // descomentar para activar el filtro !!!!!!!!!
  //distancia_float_0 = filtroDelta(distancia_float_0, distancia_float_1, 40.0, 0.05);
  //distancia_float_1 = distancia_float_0;
}

void enviar_valores() {
  // se envian en formato CSV: comas para separar columnas, saltos de linea para filas
  // cada columna contiene una medicion, cada fila un grupo de mediciones completo
  // TIEMPO,INTERNA,T_INTERNA,EXTERNA,T_EXTERNA,RUEDITA,DISTANCIA,ACEL_X, ACEL_Y, ACEL_Z
  Serial.print(tiempoTranscurrido);
  Serial.print(SEP);  
  Serial.print(presionAdentro);
  Serial.print(SEP);
  Serial.print(tempAdentro);
  Serial.print(SEP);  
  Serial.print((presionAtmosferica + offset_presion));
  Serial.print(SEP);
  Serial.print(tempAfuera);
  Serial.print(SEP);  
  Serial.print(contadorRueda);
  Serial.print(SEP);
  Serial.print(distancia_float_0);
  Serial.print(SEP);
  Serial.print(aceleracionX); 
  Serial.print(SEP);  
  Serial.print(aceleracionY);
  Serial.print(SEP);
  Serial.println(aceleracionZ); // termina con salto de linea
}

void enviar_encabezados(){

  Serial.print(INICIO); // para avisar al receptor que inicia la transmision
  Serial.print("Tiempo");
  Serial.print(SEP);
  Serial.print("PInterna");
  Serial.print(SEP);
  Serial.print("TInterna");
  Serial.print(SEP);  
  (usando_calibracion) ? Serial.print("PExterna_Calibrada") : Serial.print("PExterna_Cruda");
  Serial.print(SEP);
  Serial.print("TExterna");
  Serial.print(SEP);
  Serial.print("Contador");
  Serial.print(SEP);
  Serial.print("Longitud");
  Serial.print(SEP);
  Serial.print("AcelX");
  Serial.print(SEP);
  Serial.print("AcelY");
  Serial.print(SEP);
  Serial.println("AcelZ");
}

float filtroDelta (float valorNuevo, float valorViejo, float deltaTope, float pesoMinimo){
  // --------------------------------------------------------------------------------------
  // Filtro con peso adaptado a la diferencia (DELTA) entre valores
  // para que NO actue con cambios rapidos, pero SI suavice los lentos
  //    deltaTope   // delta para que el filtro no actue mas y deje la señal intacta
  //    pesoMinimo  // peso minimo del nuevo valor
    float resultado = 0;

    float pesoDelNuevo = pesoMinimo + 
                         (min(abs(valorNuevo - valorViejo), deltaTope) / deltaTope) *  // esto da entre 0 y 1
                         (1.0 - pesoMinimo);
  // 0.01  +   (5 / 50) * (1 - 0.01) = 0.109
  
    resultado = valorNuevo *  pesoDelNuevo  +
                valorViejo * (1.0-pesoDelNuevo);
  // --------------------------------------------------------------------------------------
  return resultado;
}

void calibrarPresion (){
  leer_presiones();
  offset_presion = presionAdentro - presionAtmosferica; // en principio la interior (sensor sin esmalte) es mayor...
  Serial.print(OK_CALIB);
  usando_calibracion = true;
  debo_calibrar = false;
}

void anularCalibracionPresion (){
  offset_presion = 0.0;
  Serial.print(OK_DESCALIB);
  usando_calibracion = false;
  debo_descalibrar = false;
}
