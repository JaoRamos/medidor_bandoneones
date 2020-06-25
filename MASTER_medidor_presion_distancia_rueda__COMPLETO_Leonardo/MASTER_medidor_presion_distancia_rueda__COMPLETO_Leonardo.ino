/*
Timer: se setea una frecuencia de muestreo, y a cada interrupcion del timer se sube y baja el pin de clock
con ancho de pulso segun el delayMicroseconds
*/

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

// debug de tiempo
//int tiempo = 0;
//int lapso = 0;

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
float offset = 0.8; // diferencia entre sensores, no se a que se debe
Adafruit_BME280 bmeA; // I2C
Adafruit_BME280 bmeB; // I2C
float presionAdentro = 0;
float presionAtmosferica = 0;

// sensor de distancia local
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X laser;
uint16_t distancia = 0; 

void setup() {
  Serial.begin(115200);
  intervaloTiempo = 1.0 / SAMPLE_RATE;
  
  iniciar_timer();
  iniciar_BME();
  iniciar_pulsador();
  iniciar_distancia();
  iniciar_rueda();
    
  delay(500); // margen de tiempo
}

void loop() {

  // PRIMER PULSO --------------------------------
  if (hacer_pulso && primer_pulso && funcionando){
    contadorRueda = 0;
    tiempoTranscurrido = 0.0;
    pulsar(PULSO_LARGO);
      enviar_encabezados();
      leer_presiones();
      leer_distancia();
      enviar_valores();
    hacer_pulso = false;
    primer_pulso = false;
  }
  
  // DEMAS PULSOS ---------------------------------
  if (hacer_pulso && !primer_pulso && funcionando){
    pulsar(PULSO_CORTO);
      leer_presiones();
      leer_distancia();
      tiempoTranscurrido += intervaloTiempo;
      enviar_valores();
    hacer_pulso = false;
  }

  // ULTIMO PULSO --------------------------------
/*
  if (ultimo_pulso && funcionando && hacer_pulso){
    pulsar(PULSO_LARGO);
      Serial.println("FIN");
    ultimo_pulso = false;
    hacer_pulso = false;
    funcionando = false;  // ACA SE APAGA!
    primer_pulso = true;
  }*/

  leerPulsador();
  leer_serial();
}

void iniciar_timer() {
  pinMode(pinDeSalida, OUTPUT);

  // usar el Timer 1 que es de 16 bit
  TCCR1A = (TCCR1A | _BV(COM1A1)) & ~_BV(COM1A0); // CTC
  TCCR1A &= ~(_BV(WGM10) | _BV(WGM11)); // waveform ctc
  TCCR1B = (TCCR1B | _BV(WGM12)) & ~_BV(WGM13); // waveform ctc
  
  //TCCR1B = (TCCR1B | _BV(CS10)) & ~(_BV(CS11) | _BV(CS12)); // sin prescaler

  TCCR1B = (TCCR1B | _BV(CS31)) & ~(_BV(CS30) | _BV(CS32)); // prescaler /8
  //TCCR1B = (TCCR1B | _BV(CS32)) & (TCCR1B | _BV(CS31)) & ~(_BV(CS31)); // prescaler /1024
  
  cli();
  
  // en teoria el numero con prescaler 8 es 20000, pero no debe ser perfecto el clock del Mega...
  // por lo que requiere un ajuste ligero, se puede testear grabando el audio y viendo lapsos de 0.1 1 y 10ms
  
  OCR1A = 19997; //(F_CPU / SAMPLE_RATE) / 8;
  
  //19987; MEGA //(F_CPU / SAMPLE_RATE) / PRESCALER;  // ojo que no se pase de 32k
  
  sei();
  
  // Se activa la interrupcion segun el SAMPLE_RATE
  TIMSK1 |= _BV(OCIE1A);  // (esta andando)
}

void leer_serial() {
  if (Serial.available() > 0) {
    recibido = Serial.read();

    // caracter "0" para detener
    if (recibido == 48) {
       funcionando = false;
       hacer_pulso = false; // no hay que hacer pulsos...
       primer_pulso = true; // la proxima iniciará con pulso largo
    }
    // caracter "1" para iniciar
    if (recibido == 49) {
       contadorRueda = 0;
       tiempoTranscurrido = 0.0;
       primer_pulso = true; // la proxima iniciará con pulso largo
       funcionando = true;
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

  /*
  if (pulsador.fell()) {  // Al apretar el pulsador
      funcionando = !funcionando; // Alternar
      if (!funcionando) apagar = true;
      if (funcionando) {
        apagar = false;
      }
  }
  
  if (!funcionando && apagar) {
      ultimo_pulso = true; // avisa que hay que cerrar
      apagar = false;
  }
*/
}

ISR(TIMER1_COMPA_vect) {
  // interrupcion por el timer
  hacer_pulso = true; // no hay que hacer nada mas aca...
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
    Serial.println(F("BME280A test"));

    unsigned status;
    
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bmeA.begin(0x76);    // el que tiene SDO a GND (pull down onboard)
    if (!status) {
        Serial.println("No funciona el sensor de presion de adentro!");
        while (1);
    }

    Serial.println(F("BME280B test"));
    status = bmeB.begin(0x77);    // el que tiene SDO a vcc
    if (!status) {
        Serial.println("No funciona el sensor de presion de afuera!");
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
    bmeB.setSampling(bmeB.MODE_NORMAL, bmeB.SAMPLING_X1, bmeB.SAMPLING_X16, bmeB.SAMPLING_NONE, bmeB.FILTER_X16, bmeB.STANDBY_MS_0_5);
        
    Serial.println("TODO OK");
}

void iniciar_distancia(){
  Wire.begin();
  laser.init();
  //sensor.setTimeout(50);
  laser.startContinuous();
  laser.setMeasurementTimingBudget(20000);
}

void iniciar_rueda(){
  pinMode(PIN_INTERRUPCION, INPUT);
  pinMode(PIN_DIRECCION, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPCION), interrupcionRueda, RISING);
}

void interrupcionRueda(){
  // condensado en una sola linea, si hay 1 en el PIN_DIRECCION suma, si no, resta
  // PIND es el puerto D que contiene los inputs de los pines 0-7
  // PINB<<4 es el pin 8
  (PINB & (1<<4))? contadorRueda++ : contadorRueda--;
}

void leer_presiones() {
  presionAtmosferica = offset + (bmeB.readPressure()/100.0);
  presionAdentro = bmeA.readPressure()/100.0;
}

void leer_distancia() {
  distancia = laser.readReg16Bit(laser.RESULT_RANGE_STATUS + 10);
}

void enviar_valores() {
  // se envian en formato CSV: comas para separar columnas, saltos de linea para filas
  // cada columna contiene una medicion, cada fila un grupo de mediciones completo
  // TIEMPO,INTERNA,EXTERNA,CONTADOR,DISTANCIA
  Serial.print(tiempoTranscurrido);
  Serial.print("\t");  
  Serial.print(presionAdentro);
  Serial.print("\t");
  Serial.print(presionAtmosferica);
  Serial.print("\t");
  Serial.print(contadorRueda);
  Serial.print("\t");
  Serial.println(min(distancia, 700)); // termina con salto de linea
}

void enviar_encabezados(){
  Serial.print("Tiempo");
  Serial.print("\t");
  Serial.print("PInterna");
  Serial.print("\t");
  Serial.print("PExterna");
  Serial.print("\t");
  Serial.print("Contador");
  Serial.print("\t");
  Serial.println("Distancia");
}
