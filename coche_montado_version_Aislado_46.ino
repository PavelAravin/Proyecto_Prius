/*
  Este código es una implementación para Arduino Nano Every
  utilizando una pantalla SSD1309 y un módulo CAN
  (Controller Area Network). El código controla y muestra información sobre el
  estado de la batería de alto voltaje sustituida en un Toyota Prius Gen 2.
  Realiza comunicación a través del bus CAN. Consiguiendo su optimización de funcionamiento.
  https://attachments.priuschat.com/attachment-files/2021/09/211662_Prius22009_CAnCodes.pdf
  https://en.wikipedia.org/wiki/OBD-II_PIDs#Standard_PIDs
  https://es.wikipedia.org/wiki/OBD-II_PID
  https://www.eaa-phev.org/wiki/Prius_PHEV_TechInfo#4
  https://github.com/Lorevalles/Proyecto_Prius
*/
#include <U8g2lib.h>
#include <SPI.h>
#include <mcp2515.h>

const int SPI_CS_PIN = 10;
const int INT_PIN = 2;
const int SS_RX_PIN = 11;
const int SS_TX_PIN = 12;

U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/5, /* data=*/6, /* cs=*/7, /* dc=*/9, /* reset=*/8);
struct can_frame canMsg;
MCP2515 mcp2515(SPI_CS_PIN);


///unsigned long lastAvisoRvTime = 0;
const unsigned long avisoRvInterval = 1000;  // Intervalo de tiempo en milisegundos (1 segundo)
unsigned long lastAvisoRcTime = 0;
const unsigned long avisoRcInterval = 1000;  // Intervalo de tiempo en milisegundos (1 segundo)

unsigned long currentMillisEv = millis();  // Obtener el tiempo actual
unsigned long ultimaActualizacion = 0;     // Variable para almacenar el tiempo de la última actualización
int valorAnterior = -1;                    // Inicialmente, ningún valor anterior

// Declarar una variable para almacenar el tiempo del último cambio en el valor de lecturas por segundo (L:)
unsigned long lastReadingsUpdateMillis = 0;
// Declarar una variable para almacenar el tiempo del último cambio en el valor de mensajes enviados por segundo (E:)
unsigned long lastSentMessagesUpdateMillis = 0;

// Variables globales Cotrol activaEv
bool activacionPendiente = false;
unsigned long ultimaActivacion = 0;
unsigned long tiempoUltimaAplicacion = 0;
unsigned long tiempoSinDatos = 0;
const unsigned int evTiactivado = 300;     // 300 milisegundos Tiempo pulsado el boton
const unsigned int tiempoEsperar = 30000;  // 30 segundos en milisegundos Ciclo de verificación

// Inicio encendido control
int tiempoEspera = 10000;  // Espera para poder apagar el vehiculo al reiniciar

// Volores control A0 A1 y D3, D4
const byte socBajo = 70;         //75 Valor a mantener minimo de SOC
const byte socAlto = 75;         //77 Valor a mantener Maximo de SOC
const byte limiteCarga = 80;     //80 Comienza a descargar
const byte voltajeMinimo = 228;  // 215
const byte voltajeMaximo = 238;  // 230
const byte marcaBaja = 65;       // Este valor tambien condiciona a partir de carga voltajeMaximo
const byte marcaAlta = 80;       // Marca en icono SOC

// Variables para el control de la salida A0 (Rele Voltaje)
unsigned long lastOutputA0ChangeMillis = 0;   // Último cambio de estado
const unsigned int tiempoActivoA0 = 1000;     // 0.5 segundos en milisegundos
const unsigned int tiempoInactivoA0 = 30000;  // 30 segundos en milisegundos

// Refresco de pantalla
const unsigned int esperaRef = 500;  // 1000 cada segundo actualiza los datos

// Definición de pines para salidas
const byte outputPinD3 = 3;   // Carga
const byte outputPinD4 = 4;   // Descarga
const byte outputPinA0 = A0;  // Rele control voltaje simulador bateria
const byte outputPinA1 = A1;  // Rele activacion EV

const int botonPin = A2;        // Pin del botón en la entrada A2
const int interruptorPin = A3;  // Pin del interruptor en la entrada A3
// Declarar variables globales para mantener el estado anterior de A4 y A5
int lastA4State = HIGH;
int lastA5State = HIGH;
unsigned long lastAvisoRvTime = 0;  // Variable para realizar un seguimiento del tiempo del último avisoRv


const int debounceDelay = 50;  // Retardo de rebote en milisegundos

int lastButtonState = LOW;           // Estado anterior del botón
int lastSwitchState = LOW;           // Estado anterior del interruptor
unsigned long lastDebounceTime = 0;  // Último tiempo de rebote registrado

// Definir las coordenadas para ubicar el ícono de la batería en la pantalla OLED
const uint8_t batteryIconX = 0;
const uint8_t batteryIconY = 0;
const uint8_t batteryIconWidth = 128;
const uint8_t batteryIconHeight = 8;

// Variables para mostrar el estado en la pantalla
float lastHvSocValue = 79.0;
int lastByte3 = 225;
int PAValor = 0;

// Declara una variable global para almacenar el estado de EV
String EV = "SI";  // Inicialmente, asumimos que EV es "SI"
unsigned long tiempoInicio = 0;

// Otras variables
bool outputA0Activated = false;
bool d3Activated = false;
bool lastD3Activated = false;
bool lcdNeedsUpdate = false;
unsigned int readingsPerSecond = 0;
unsigned int readCountPerSecond = 0;
unsigned int sentMessageCount = 0;
unsigned int sentMessagesPerSecond = 0;
unsigned int totalSentMessages = 0;

int decimalValue = 0;  // Acelerador posicon

// Control de tiempo y recuento
unsigned int readCount = 0;

// Variables para el control de tiempo y ciclos por segundo
unsigned long previousSecondMillis = 0;

void setup() {
  u8g2.begin();
  ///u8g2.firstPage();
  do {
    //u8g2.setContrast(128);  // Contraste
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(5, 20, "Ciclo de espera");
    u8g2.setCursor(6, 40);
    u8g2.drawLine(6, 35, 120, 35);
    u8g2.drawStr(0, 55, "Se puede PARAR");
    u8g2.sendBuffer();
  } while (u8g2.nextPage());

  // Inicialización de las salidas digitales

  // Configura las entradas y salidas según la descripción proporcionada
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  pinMode(botonPin, INPUT_PULLUP);
  pinMode(interruptorPin, INPUT_PULLUP);
  pinMode(outputPinD3, OUTPUT);
  pinMode(outputPinD4, OUTPUT);
  pinMode(outputPinA0, OUTPUT);
  pinMode(outputPinA1, OUTPUT);

  // Inicializa las salidas según lo que mencionaste
  digitalWrite(outputPinD3, LOW);
  digitalWrite(outputPinD4, LOW);
  digitalWrite(outputPinA0, LOW);
  digitalWrite(outputPinA1, LOW);

  // Inicialización de la comunicación SPI para el módulo CAN
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // Borrado de errores
  borraError();

  // Esperar tiempoEspera antes de continuar
  delay(tiempoEspera);
  u8g2.clearBuffer();
  u8g2.firstPage();
  u8g2.clearDisplay();
  u8g2.setFont(u8g2_font_6x10_tf);
  updateu8g2();
}
bool debounce(int newButtonState, int newSwitchState, int &lastState, unsigned long &lastDebounceTimeButton) {
  unsigned long currentTime = millis();

  if (newButtonState != lastState) {
    // Actualizar el último tiempo estable
    lastDebounceTime = currentTime;
  }

  if ((currentTime - lastDebounceTime) > debounceDelay) {
    // Si ha pasado el tiempo de debounce
    lastState = newButtonState;  // Actualizar el último estado estable
    return true;
  }

  return false;
}

void borraError() {
  inicioObd();        //18DB33F1
  solicitudDatOBD();  //0x7DF
  errorMotor();       //0x7E0
  errorHybrid();      //0x7E2
  errorBattery();     //0x7E3
  erroresBat1();      //0x7E2
  erroresBat3();      //0x7E3
  erroresMot1();      //0x7E0
  erroresMot2();      //0x7E8
  // erroresBat2(); esto es una respuesta
  // erroresBat4(); esto es una respuesta
}

void avisoRv() {  // Aviso rele control voltaje bateria
  // Verifica si ha pasado al menos 1 segundo desde el último avisoRv
  unsigned long currentMillis = millis();
  if (currentMillis - lastAvisoRvTime >= avisoRvInterval) {
    // Realiza las acciones de avisoRv aquí
    u8g2.setCursor(63, 54);
    u8g2.print("V:M");
    u8g2.sendBuffer();

    // Actualiza el tiempo del último avisoRv
    lastAvisoRvTime = currentMillis;
  }
}
void avisoRc() {
  // Verifica si ha pasado al menos 1 segundo desde el último avisoRc
  unsigned long currentMillis = millis();
  if (currentMillis - lastAvisoRcTime >= avisoRcInterval) {
    // Realiza las acciones de avisoRc aquí
    u8g2.setCursor(88, 54);
    u8g2.print("C:M");
    u8g2.sendBuffer();

    // Actualiza el tiempo del último avisoRc
    lastAvisoRcTime = currentMillis;
  }
}
void controla() {  // Control por pantalla(BANDERA)
  u8g2.setCursor(63, 54);
  u8g2.print(lastHvSocValue);
  u8g2.sendBuffer();
  //delay(1000);
}

void control(int value) {  // Control por pantalla(BANDERA)
  u8g2.setCursor(63, 54);
  u8g2.print(decimalValue);
  u8g2.sendBuffer();
  //delay(1000);
}

void drawCurrentIcon(uint8_t y, int value) {
  const uint8_t maxCurrent = 100;  // Valor máximo de corriente
  const uint8_t barHeight = 9;     // Altura de la barra
  const uint8_t barSpacing = 2;    // Espacio entre barras

  // Calcular el ancho de la barra
  int barWidth = map(abs(value), 0, maxCurrent, 0, 128);

  // Dibujar la barra
  u8g2.drawFrame(0, 35, 128, barHeight);
  if (value >= 0) {
    u8g2.drawBox(0, 35, barWidth, barHeight - 1);
  } else {
    u8g2.drawBox(128 - barWidth, 35, barWidth, barHeight - 1);
  }
}

void drawThrottleBar(uint8_t decimalValue) {
  const int maxAcele = 200;   // Valor máximo de corriente
  const int baraHeight = 6;   // Altura de la barra
  const int baraSpacing = 1;  // Espacio entre barras
  const int baraX = 0;        // Posición X de la barra
  const int baraY = 19;       // Posición Y de la barra
  const int baraWidth = 128 - baraX * 2;

  // Mapea el valor de entrada al ancho de la barra
  int baraLength = map(abs(decimalValue), 0, maxAcele, 0, baraWidth);

  // Dibuja el marco de la barra
  u8g2.drawFrame(baraX, baraY, baraWidth, baraHeight);

  // Dibuja la barra de acuerdo al valor de aceleración
  if (decimalValue >= 0) {
    u8g2.drawBox(baraX, baraY, baraLength, baraHeight - 1);
  } else {
    u8g2.drawBox(baraX + baraWidth - baraLength, baraY, baraLength, baraHeight - 1);
  }
}

// Enviar solicitudes al sistema de datos de OBD.
void solicitudDatOBD() {
  canMsg.can_id = 0x7DF;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x02;  // Número de bytes de datos adicionales 02, 03
  canMsg.data[1] = 0x01;  // 01 = mostrar datos actuales 02 = congelar fotograma
  canMsg.data[2] = 0x00;  // si 03
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
  // https://en.wikipedia.org/wiki/OBD-II_PIDs#Standard_PIDs
  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

// Enviar solicitudes al sistema de diagnóstico del vehículo.
void finalObd() {
  canMsg.can_id = 0x7DF;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x02;
  canMsg.data[1] = 0x01;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

void inicioObd1() {
  canMsg.can_id = 0x7DF;  // ID del mensaje de solicitud de diagnóstico (Diagnostics Request)
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x02;  // Tamaño del mensaje (solicitud de PID)
  canMsg.data[1] = 0x09;  // Código del PID específico para obtener VIN
  canMsg.data[2] = 0x00;  // Datos adicionales (si es necesario)
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

// OBD-II inicio?para establecer comunicación
void inicioObd() {
  canMsg.can_id = 0x18DB33F1;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x02;
  canMsg.data[1] = 0x01;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

// Borra error de Unidad de control del motor #1
void errorMotor() {
  canMsg.can_id = 0x7E0;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x02;
  canMsg.data[1] = 0x3E;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}
// Borra error de Hybrid engine system
void errorHybrid() {
  canMsg.can_id = 0x7E2;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x02;
  canMsg.data[1] = 0x3E;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}
// Borra error de HV battery
void errorBattery() {
  canMsg.can_id = 0x7E3;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x02;
  canMsg.data[1] = 0x3E;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

void erroresBat1() {
  canMsg.can_id = 0x7E2;  // Crreccion errores bateria
  canMsg.can_dlc = 8;     // Send: 0x7E2 : 01 04 00 00 00 00 00 00
  canMsg.data[0] = 0x01;  // Resp: 0x7EA : 01 44 00 00 00 00 00 00
  canMsg.data[1] = 0x04;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

void erroresBat2() {
  canMsg.can_id = 0x7EA;  // Respuesta errores bateria a
  canMsg.can_dlc = 8;     // Send: 0x7E2 : 01 04 00 00 00 00 00 00
  canMsg.data[0] = 0x01;  // Resp: 0x7EA : 01 44 00 00 00 00 00 00
  canMsg.data[1] = 0x44;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

void erroresBat3() {      // Clearing HV Battery DTCs :
  canMsg.can_id = 0x7E3;  // Crreccion errores bateria se envia
  canMsg.can_dlc = 8;     // Send: 0x7E3 : 01 04 00 00 00 00 00 00
  canMsg.data[0] = 0x01;  // Resp: 0x7EB : 01 44 00 00 00 00 00 00
  canMsg.data[1] = 0x04;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}
void erroresBat4() {
  canMsg.can_id = 0x7EB;  // Respuesta Correccion errores bateria a la pregunta
  canMsg.can_dlc = 8;     // 0x7E3 : 01 04 00 00 00 00 00 00
  canMsg.data[0] = 0x01;
  canMsg.data[1] = 0x44;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

void erroresMot1() {
  canMsg.can_id = 0x7E0;  // Crreccion errores bateria
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x01;
  canMsg.data[1] = 0x04;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

void erroresMot2() {
  canMsg.can_id = 0x7E8;  // Crreccion errores bateria
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x01;
  canMsg.data[1] = 0x44;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

void solicitudDat1() {
  canMsg.can_id = 0x7EB;  // Solicitud datos
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x24;
  canMsg.data[1] = 0x13;
  canMsg.data[2] = 0x13;
  canMsg.data[3] = 0x13;
  canMsg.data[4] = 0x13;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

void activateEVMode() {
  canMsg.can_id = 0x529;
  canMsg.can_dlc = 7;
  canMsg.data[0] = 0xA8;
  canMsg.data[1] = 0x00;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x85;
  canMsg.data[4] = 0x40;  // Valor 40 para activar el modo EV
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}

void deactivateEVMode() {
  canMsg.can_id = 0x529;
  canMsg.can_dlc = 7;
  canMsg.data[0] = 0xA8;
  canMsg.data[1] = 0x00;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x85;
  canMsg.data[4] = 0x00;  // Valor 00 para desactivar el modo EV
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;

  mcp2515.sendMessage(&canMsg);
  sentMessageCount++;
}
// Función para dibujar el ícono SOC
void drawBatteryIcon(uint8_t percentage) {
  // Calcular el ancho total del ícono proporcional al valor del porcentaje
  uint8_t iconWidth = map(percentage, 0, 100, 0, batteryIconWidth);

  // Dibujar el rectángulo que representa la barra de SOC
  u8g2.drawFrame(batteryIconX, batteryIconY, batteryIconWidth, batteryIconHeight);

  // Dibujar las marcas en las posiciones 60% y 80% dentro del ícono de la batería
  uint8_t mark60 = map(marcaBaja, 0, 100, 0, batteryIconWidth);
  uint8_t mark80 = map(marcaAlta, 0, 100, 0, batteryIconWidth);
  u8g2.drawLine(batteryIconX + mark60, batteryIconY, batteryIconX + mark60, batteryIconY + batteryIconHeight);
  u8g2.drawLine(batteryIconX + mark80, batteryIconY, batteryIconX + mark80, batteryIconY + batteryIconHeight);

  // Rellenar el rectángulo que representa la barra de SOC hasta el valor correspondiente
  u8g2.drawBox(batteryIconX, batteryIconY, iconWidth, batteryIconHeight);
}

void updateu8g2() {
  u8g2.clearBuffer();
  //  Llamar a la función para dibujar el ícono del acelerador
  drawThrottleBar(decimalValue);
  // Llamar a la función para dibujar el ícono SOC de la batería con el valor actual de porcentaje
  drawBatteryIcon(lastHvSocValue);

  // Mostrar el porcentaje de carga de la batería en la posición (100, 10) de la pantalla LCD
  u8g2.setCursor(0, 18);
  u8g2.print("SOC:");
  u8g2.print(lastHvSocValue, 1);
  u8g2.print("%");

  // Mostrar la intensidad de corriente en la posición (0, 33) de la pantalla LCD
  u8g2.setCursor(0, 33);
  u8g2.print("A:");
  u8g2.print(PAValor);
  drawCurrentIcon(21, PAValor);

  // Mostrar el voltaje en la posición (92, 33) de la pantalla LCD
  u8g2.setCursor(92, 33);
  u8g2.print("V:");
  u8g2.print(lastByte3);

  // Mostrar el valor de EV en la posición (50, 33) de la pantalla LCD
  u8g2.setCursor(50, 33);
  u8g2.print("EV:");
  u8g2.print(EV);

  // Mostrar el estado de la salida A0 (R.Bat.H o R.Bat.L) en la posición (80, 54) de la pantalla LCD
  u8g2.setCursor(110, 54);
  if (outputA0Activated) {
    u8g2.print("R.H");
  } else {
    u8g2.print("R.L");
  }

  // Mostrar si el SOC Esta o no en rango
  u8g2.setCursor(85, 18);
  if (lastHvSocValue <= marcaBaja || lastHvSocValue >= marcaAlta) {
    u8g2.print("Fuera");
  } else {
    u8g2.print("Dentro");
  }

  // Mostrar si se puede parar o no en la posición (0, 64) de la pantalla LCD
  if (d3Activated) {
    //digitalWrite(outputPinD3, HIGH);
    u8g2.setCursor(0, 64);
    u8g2.print("NO PARAR");
  } else {
    //digitalWrite(outputPinD3, LOW);
    digitalWrite(outputPinD3, LOW);
    u8g2.setCursor(0, 64);
    u8g2.print("SI POWER");
  }

  //Mostrar los valores de lecturas y envíos por segundo en las posiciones (0, 54) y (30, 54) de la pantalla LCD
  u8g2.setCursor(0, 54);
  u8g2.print("L:");
  u8g2.print(readingsPerSecond);

  // Mostrar el valor de mensajes enviados por segundo (E:) en la posición (38, 54) de la pantalla LCD
  u8g2.setCursor(38, 54);
  u8g2.print("E:");
  u8g2.print(sentMessagesPerSecond);

  int switchState = digitalRead(interruptorPin);
  // Controlar el estado de la salida D4 y mostrarlo en la posición (60, 64) de la pantalla LCD
  if (lastHvSocValue >= limiteCarga && (!switchState == HIGH)) {
    digitalWrite(outputPinD4, HIGH);
    u8g2.setCursor(60, 64);
    u8g2.print("Ac.Descarga");
  } else {
    digitalWrite(outputPinD4, LOW);
    u8g2.setCursor(60, 64);
    u8g2.print("De.Inactiva");
  }

  // Intensidad barra -----
  u8g2.setCursor(0, 43);
  if (PAValor >= 0) {
    int lineLength = map(PAValor, 0, 100, 0, 20);
    for (int i = 0; i < 20; i++) {
      if (i >= 20 - lineLength) {
        u8g2.print("+");
      } else {
        u8g2.print(" ");
      }
    }
  } else {
    int lineLength = map(PAValor, 0, -100, 0, 20);
    for (int i = 0; i < 20; i++) {
      if (i < lineLength) {
        u8g2.print("-");
      } else {
        u8g2.print(" ");
      }
    }
  }
  u8g2.sendBuffer();  // Enviar los datos a la pantalla LCD
}

// Declaración de la función para actualizar el estado de la salida A0
void updateOutputA0(bool state) {
  digitalWrite(14, state ? HIGH : LOW);
}

void processCanMessage() {
  readCount++;
  readCountPerSecond++;

  // SOC
  if (canMsg.can_id == 0x3CB && canMsg.can_dlc == 7) {
    byte byte2 = canMsg.data[2];
    byte byte3 = canMsg.data[3];
    float hvSocValue = ((byte2 * 256 + byte3) / 2.0);

    if (hvSocValue != lastHvSocValue) {
      lastHvSocValue = hvSocValue;
    }
  }
  // Voltios
  if (canMsg.can_id == 0x03B && canMsg.can_dlc == 5) {
    byte byte1 = canMsg.data[1];
    byte byte3 = canMsg.data[3];
    int hvV = (canMsg.data[2] * 256) + byte3;
    hvV = (hvV & 0x07FF) - (hvV & 0x0800);
    hvV *= 1;
    // Amperios
    int aValue = ((canMsg.data[0]) * 256) + (canMsg.data[1]);
    if ((aValue & 0x800) != 0) {
      aValue = aValue - 0x1000;
    }
    PAValor = aValue / 10;

    lastByte3 = hvV;
  }

  // Acelerador posicion
  if (canMsg.can_id == 0x244 && canMsg.can_dlc == 8) {
    byte byte6 = canMsg.data[6];

    // Convertir valor hexadecimal a decimal
    decimalValue = byte6;
  }
}

void leerCan() {
  // Leer mensajes CAN y procesarlos
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    processCanMessage();
  }
}

void activaEv() {
  unsigned long currentMillisEv = millis();

  // Calcular el tiempo desde la última recepción de datos
  tiempoSinDatos = currentMillisEv - tiempoUltimaAplicacion;

  if (canMsg.can_id == 0x529) {
    if (canMsg.data[4] == 0x40) {
      // Si can_id es 0x529 y data[4] es 0x40, establece el mensaje en "SI"
      EV = "SI";
    } else {
      // Si can_id es 0x529 pero data[4] no es 0x40, establece el mensaje en "NO"
      EV = "NO";
    }
  }

  if (EV == "NO") {
    // Verificar la velocidad del vehículo
    if (canMsg.can_id == 0x3CA && canMsg.data[2] <= 0x31) {
      // Verificar si la velocidad es igual o inferior a 49 km/h en hexadecimal
      if (activacionPendiente || tiempoSinDatos >= tiempoEsperar) {
        // Si hay una activación pendiente o ha pasado el tiempo de espera,
        // inicia la activación y guarda el tiempo
        ultimaActivacion = currentMillisEv;
        activacionPendiente = true;
        digitalWrite(outputPinA1, HIGH);  // Activa el relé
      }
    }
  }

  if (activacionPendiente && currentMillisEv - ultimaActivacion >= evTiactivado) {
    // Si ha pasado el tiempo de activación, establece en LOW
    digitalWrite(outputPinA1, LOW);            // Establece en LOW
    activacionPendiente = false;               // Marca que no hay una activación pendiente
    tiempoUltimaAplicacion = currentMillisEv;  // Actualiza el tiempo de última aplicación
  }
}

void conCarga() {
  unsigned long currentMillis = millis();
  unsigned long elapsedTime = currentMillis - lastOutputA0ChangeMillis;

  // Variable para realizar un seguimiento del último estado de salida A0
  static bool lastOutputA0State = false;

  if (lastHvSocValue <= marcaBaja) {
    // Si lastHvSocValue es menor o igual a marcaBaja, activar la salida A0
    if (!lastOutputA0State && elapsedTime >= tiempoInactivoA0) {
      outputA0Activated = true;
      updateOutputA0(outputA0Activated);
      lastOutputA0ChangeMillis = currentMillis;
      lastOutputA0State = true;
    }
  } else if (lastByte3 <= voltajeMinimo) {
    // Si lastHvSocValue no cumple la primera condición pero lastByte3 es menor o igual a voltajeMinimo
    if (!lastOutputA0State && elapsedTime >= tiempoInactivoA0) {
      outputA0Activated = true;
      updateOutputA0(outputA0Activated);
      lastOutputA0ChangeMillis = currentMillis;
      lastOutputA0State = true;
    }
  } else if (lastByte3 >= voltajeMaximo) {
    // Si ninguna de las condiciones anteriores se cumple pero lastByte3 es mayor o igual a voltajeMaximo
    if (lastOutputA0State && elapsedTime >= tiempoActivoA0) {
      outputA0Activated = false;
      updateOutputA0(outputA0Activated);
      lastOutputA0ChangeMillis = currentMillis;
      lastOutputA0State = false;
    }
  } else {
    // Si ninguna de las condiciones anteriores se cumple, restablecer el estado de salida A0 a su valor anterior
    if (lastOutputA0State && elapsedTime >= tiempoActivoA0) {
      outputA0Activated = false;
      updateOutputA0(outputA0Activated);
      lastOutputA0ChangeMillis = currentMillis;
      lastOutputA0State = false;
    }
  }
}

void lecSegundo() {  // Actualizar el recuento de mensajes enviados por esperaRef

  unsigned long currentMillislec = millis();
  if (currentMillislec - previousSecondMillis >= esperaRef) {
    previousSecondMillis = currentMillislec;

    readingsPerSecond = readCountPerSecond;  // Guardar el valor actual de readCountPerSecond en readingsPerSecond
    readCountPerSecond = 0;                  // Reiniciar readCountPerSecond para contar el próximo segundo

    // Actualizar valores de lecturas y mensajes enviados en el LCD
    sentMessagesPerSecond = sentMessageCount;
    if (lcdNeedsUpdate) updateu8g2();

    sentMessageCount = 0;  // Reiniciar el contador de mensajes enviados por segundo
    lcdNeedsUpdate = true;
  }
}

void controlSoc() {
  // Control SOC
  if (lastHvSocValue <= marcaBaja) {
    d3Activated = true;
    digitalWrite(outputPinD3, HIGH);
  }
  // Agrega esta condición para que d3Activated no pueda ser false en otros casos
  else if (lastByte3 >= voltajeMaximo && lastHvSocValue >= marcaBaja) {
    d3Activated = false;
    digitalWrite(outputPinD3, LOW);
  }

  // Define una variable para indicar si la condición se cumple
  bool condicionCumplida = false;

  // Establece la duración máxima del ciclo en milisegundos
  unsigned long duracionMaxima = 1;  // Por ejemplo, 1 milisegundos afecta a CAN velocidad

  // Registra el tiempo de inicio
  unsigned long tiempoInicio = millis();

  while (!condicionCumplida) {
    // Verifica si la condición lastByte3 >= voltajeMaximo todavía se cumple

    if (lastByte3 >= voltajeMaximo) {
      // Si la condición aún se cumple, verifica si ha pasado el tiempo máximo
      if (millis() - tiempoInicio >= duracionMaxima) {
        condicionCumplida = true;  // Si ha pasado el tiempo máximo, sale del bucle
      }
    } else {
      // Si la condición ya no se cumple, detén el bucle
      condicionCumplida = true;
      cargaDes();
    }
  }
}
void cargaDes() {
  // Controlar la salida D3 dependiendo del valor de SOC y lastHvSocValue
  if (lastHvSocValue <= socBajo) {
    if (!d3Activated) {
      digitalWrite(outputPinD3, HIGH);  // Activar D3
      d3Activated = true;
      lcdNeedsUpdate = true;
    }
  } else if (lastHvSocValue >= socAlto) {
    if (d3Activated) {
      digitalWrite(outputPinD3, LOW);  // Desactivar D3
      d3Activated = false;
      lcdNeedsUpdate = true;
    }
  }
}
//////////////////////////////////////////////////////////////////////////

void loop() {
  // Lee el estado actual del botón, el interruptor y las entradas A4 y A5
  int buttonState = digitalRead(botonPin);
  int switchState = digitalRead(interruptorPin);
  int a4State = digitalRead(A4);
  int a5State = digitalRead(A5);

  // Comprueba si A4 o A5 han cambiado de estado
  if (a4State != lastA4State || a5State != lastA5State) {
    // Si alguno de ellos ha cambiado, ejecuta avisoRv()
  }

  // Actualiza el estado anterior de A4 y A5
  lastA4State = a4State;
  lastA5State = a5State;

  // Verifica si A4 o A5 están en estado de activación
  if (a4State == LOW || a5State == LOW) {
    // Si al menos uno de ellos está en estado de activación, establece el estado de pinA0
    digitalWrite(outputPinA0, (a4State == HIGH) ? HIGH : LOW);
    avisoRv();
  }

  /////////////////////////////////////////////////////////////////////
  // Comprueba si el botón se ha pulsado
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Borra el error
    borraError();
  }

  // Comprueba si el interruptor está en la posición correcta
  if (switchState == HIGH) {
    avisoRc();
    // Verifica que al menos uno de los pines D3 o D4 esté alto y cambia su estado a bajo
    if (digitalRead(outputPinD3) == HIGH || digitalRead(outputPinD4) == HIGH) {
      d3Activated = false;
      lastHvSocValue = socAlto + 1;
      // Cambia los pines D3 y D4 a bajo
      digitalWrite(outputPinD3, LOW);
      // digitalWrite(outputPinD4, LOW);
    }
  }

  // Actualiza el estado anterior del botón y el interruptor
  lastButtonState = buttonState;
  lastSwitchState = switchState;

  /////////////////////////////////////////////////////////////////////
  /* if (lastHvSocValue <= socBajo) {
    digitalWrite(outputPinD3, HIGH);
    d3Activated = true;
  }*/
  leerCan();
  activaEv();
  conCarga();
  lecSegundo();
  controlSoc();
}
