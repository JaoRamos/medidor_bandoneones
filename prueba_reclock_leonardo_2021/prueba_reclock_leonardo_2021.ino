/*
Timer: se setea una frecuencia de muestreo, y a cada interrupcion del timer se sube y baja el pin de clock
con ancho de pulso segun el delayMicroseconds
*/

// timer y control
#define SAMPLE_RATE 100   // en HZ
#define PRESCALER 8   // para 100 hz no daria el OCR de 16 bits sin preescalar
#define PULSO_LARGO 100   // en uS 
#define PULSO_CORTO 30   // en uS --- el tamaño del pulso determina el nivel que aparece de audio despues del capacitor...
const int pinDeSalida = 5;   // salida del pin de clock por pulsos de "audio"
//const int puertoSalida = DDC6;   // DDC6 es el pin 5, no es lo mismo el pin del arduino que el pin del chip
boolean hacer_pulso = false;
boolean primer_pulso = true;
boolean funcionando = false;
boolean ultimo_pulso = false;
boolean apagar = true;
float tiempoTranscurrido = 0.0;
float intervaloTiempo = 0.0;  // se calcula en setup!
long DIVISOR_SIN_PS = 159988L;
const int CONTAR_HASTA = 3;
int contador_clock = 0;

// pulsador
#include <Bounce2.h>
#define PIN_BOTON 9
Bounce pulsador = Bounce(); 

void setup() {
  iniciar_pulsador();
  iniciar_timer();
}

void loop() {

  // PRIMER PULSO --------------------------------
  if (hacer_pulso && primer_pulso && funcionando){
    pulsar(PULSO_LARGO);

    hacer_pulso = false;
    primer_pulso = false;
  }
  
  // DEMAS PULSOS ---------------------------------
  if (hacer_pulso && !primer_pulso && funcionando){
    pulsar(PULSO_CORTO);

    hacer_pulso = false;
  }

  leerPulsador();
}

void iniciar_timer() {
  pinMode(pinDeSalida, OUTPUT);

  // usar el Timer 1 que es de 16 bit
  TCCR1A = (TCCR1A | _BV(COM1A1)) & ~_BV(COM1A0); // CTC
  TCCR1A &= ~(_BV(WGM10) | _BV(WGM11)); // waveform ctc
  TCCR1B = (TCCR1B | _BV(WGM12)) & ~_BV(WGM13); // waveform ctc
  
  TCCR1B = (TCCR1B | _BV(CS10)) & ~(_BV(CS11) | _BV(CS12)); // sin prescaler /1

  //TCCR1B = (TCCR1B | _BV(CS31)) & ~(_BV(CS30) | _BV(CS32)); // prescaler /8
  //TCCR1B = (TCCR1B | _BV(CS32)) & (TCCR1B | _BV(CS31)) & ~(_BV(CS31)); // prescaler /1024
  
  cli();

  // la formula seria aproximadamente 16mhz / OCR1A / prescaler (1) / contar_hasta
  // como OCR1A no puede contar mas de 16 bit (65535), subdivido el numero real que necesito
  // con el contar_hasta, se puede aumentar un poco la resolucion bajando OCR1A
  OCR1A = 53330; // leonardo pro micro
  
  sei();
  
  // Se activa la interrupcion segun el SAMPLE_RATE
  TIMSK1 |= _BV(OCIE1A);  // (esta andando)
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
  
  contador_clock++;
  
  if (contador_clock >= CONTAR_HASTA){
    hacer_pulso = true;
    contador_clock = 0;
  }
  
  //hacer_pulso = true;

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
