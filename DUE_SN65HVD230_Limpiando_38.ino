/*
Ésta versión, anula la necesidad de utilizar el Arduino de control del sensor de intensidad
(no he anulado su control, sigue siendo compatible con el)

  Este código es una implementación para Arduino Nano Due
  utilizando una pantalla SSD1309 y un módulo CAN
  (Controller Area Network SN65HVD230). El código controla y muestra información sobre el
  estado de la batería de alto voltaje sustituida en un Toyota Prius Gen 2.
  Realiza comunicación a través del bus CAN. Consiguiendo su optimización de funcionamiento.

    Al Prius NO LE GUSTA lo siguiente:

Voltaje real de la batería a 175 V o menos (independientemente de lo que le diga que es el voltaje). 
O bien, decirle al automóvil que el voltaje de la batería es mucho más alto de lo que realmente es
(por ejemplo, la batería es de 170 V, pero le dices que es de 200 V)
se genera un código de error, el coche se bloquea, en ese caso hay que 
borrar los fallos, decirle el voltaje correcto y todo estará bien.
Sí le dices al coche, que la batería está al 80 % del estado de carga o más
el motor funciona todo el tiempo, extrayendo corriente de la batería
(aproximadamente 9 A) para descargarla.
Resistencia de HV- o HV+ a la tierra del chasis inferior a unos 10 m

El Prius depende del motor en marcha para la distribución de aceite lubricante en los engranajes de la CVT 
Lo que implica que, una conversión enchufable debe garantizar que el Prius reinicie
el motor cada pocos kilómetros y luego vuelva a funcionar como vehículo eléctrico.

  https://www.eaa-phev.org/wiki/Prius_PHEV_TechInfo#4
  https://attachments.priuschat.com/attachment-files/2021/09/211662_Prius22009_CAnCodes.pdf
  https://en.wikipedia.org/wiki/OBD-II_PIDs#Standard_PIDs
  https://es.wikipedia.org/wiki/OBD-II_PID
  https://www.eaa-phev.org/wiki/Prius_PHEV_TechInfo#4
  https://github.com/Lorevalles/Proyecto_Prius
*/
// Mensajes que transmite el BMS (El BMS no recibe nada)
// 8 ms   03B      5 00 05 00 E2 27 (226V 0A) Itensidad y voltaje 03B  5 00 CB 00 E3 EE  (227V +20A) 5 0F 0F 00 E2 40 (226V -24A)
// 100 ms 3C9      8 03 FF 25 02 9A 03 22 BC 0x21 ( carga con potencia )
// 100 ms 3CB      7 67 64 00 99 15 13 61    SOC
// 100 ms 3CD      5 00 00 00 E1 B6          Voltaje del paquete: 16 bits, sin signo [V] SOC
// 100 ms 4D1      8 11 00 01 02 00 00 00 00 Datos desconocidos e inmutables. (A.V.: Batt -> HECU)


#include <U8g2lib.h>
#include <due_can.h>

U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/5, /* data=*/6, /* cs=*/7, /* dc=*/9, /* reset=*/8);

uint32_t receivedFrames;
CAN_FRAME incoming;

// Variables globales para acumular energía
float energiaAcumulada = 0.0;   // Energía consumida en vatios-hora
float energiaRegenerada = 0.0;  // Energía regenerada en vatios-hora

unsigned long tiempoAnterior = 0;          // Para calcular el intervalo de tiempo
const long intervaloActualizacion = 1000;  // Intervalo de actualización de 1000 ms (1 segundo)


int cargaModuloBat = 0;  // Declaración global

// Variable para histéresis enviando mensajes
bool permitirMensajeSOC = true;
// Define si el mensaje SOC está permitido basado en el voltajeV
bool permitirMensajePorVoltaje = true;  // o false, dependiendo del estado inicial deseado

// Variable para almacenar el último mensaje recibido

uint16_t ultimoMensajeRecibidoID = 0xFFFF;  // Inicializar con un valor que no sea un ID válido
uint16_t ultimo03BNeutro = 0xFFFF;          // Neutro
uint16_t ultimo3C9 = 0xFFFF;                // Carga con potencia
uint16_t ultimo3CD = 0xFFFF;                // Voltaje paquete
uint16_t ultimoInmutables = 0xFFFF;         // Inmutable

//revoluciones
int rpmValor = 0;
int temperatura = 0;

float hvSocValue = 0;  // Representado

// Variables globales para controlar el parpadeo
unsigned long currentMillisV = millis();
unsigned long previousMillisV = 0;  // Almacena la última vez que se actualizó el estado del parpadeo
const long interval = 300;          // Intervalo de parpadeo (300 ms = 0.3 segundos)
bool displayOn = true;              // Controla si el valor se muestra o no

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

// Variables globales Control activaEv
bool activacionPendiente = false;
unsigned long ultimaActivacion = 0;
unsigned long tiempoUltimaAplicacion = 0;
unsigned long tiempoSinDatos = 0;
const unsigned int evTiactivado = 300;     // 300 milisegundos Tiempo pulsado el boton
const unsigned int tiempoEsperar = 30000;  // 30 segundos en milisegundos Ciclo de verificación

// Inicio encendido control
int tiempoEspera = 300;  // Espera para poder apagar el vehiculo al reiniciar

// Valores control A0 A1 y D3, D4
const float factor = 1.089;        //
const byte socBajo = 71;           //70 Valor a mantener minimo de SOC
const byte socAlto = 79;           //79 Valor a mantener Maximo de SOC
const byte limiteBajoSOC = 20;     //20 Limite para el SOC más bajo
const byte histeresisAlto = 25;    //25 Histéresis SOC más bajo
const byte limiteCarga = 80;       //80 Comienza a descargar
const byte voltajeMinimo = 224;    // 224
const byte voltajeMaximo = 240;    // 240
const byte marcaBaja = 71;         // 70 Este valor tambien condiciona a partir de carga voltajeMaximo
const byte marcaAlta = 80;         // 80 Marca en icono SOC
const byte repMensaje = 2;         // 2 Número repeticiones enviado el mensaje.
const byte volLimBajoBat = 192;    // Limite de voltaje minimo de la batería para dejar de enviar mensaje SOC
const byte volLimHiteresis = 200;  // Hiteresis Limite de voltaje minimo

// Variables para el control de la salida A0 (Rele Voltaje)
unsigned long lastOutputA0ChangeMillis = 0;  // Último cambio de estado
const unsigned int tiempoActivoA0 = 10000;   // 15 segundos en milisegundos
const unsigned int tiempoInactivoA0 = 1000;  // 5 segundos en milisegundos

// Refresco de pantalla
const unsigned int esperaRef = 500;  // 1000 cada segundo actualiza los datos en la pantalla.

// Definición de pines para salidas
const byte outputPinD3 = 3;   // Carga
const byte outputPinD4 = 4;   // Descarga
const byte outputPinA0 = A0;  // Rele control voltaje simulador batería
const byte outputPinA1 = A1;  // Rele activación EV

const int botonPin = A2;        // Pin del botón en la entrada A2
const int interruptorPin = A3;  // Pin del interruptor en la entrada A3
// Declarar variables globales para mantener el estado anterior de A4 y A5
int lastA4State = HIGH;
int lastA5State = HIGH;
unsigned long lastAvisoRvTime = 0;  // Variable para realizar un seguimiento del tiempo del último avisoRv

// const int debounceDelay = 500;  // Retardo de rebote en milisegundos

int lastButtonState = LOW;           // Estado anterior del botón
int lastSwitchState = LOW;           // Estado anterior del interruptor
unsigned long lastDebounceTime = 0;  // Último tiempo de rebote registrado

// Definir las coordenadas para ubicar el ícono de la batería en la pantalla OLED
const uint8_t batteryIconX = 0;
const uint8_t batteryIconY = 0;
const uint8_t batteryIconWidth = 128;
const uint8_t batteryIconHeight = 8;

// Variables para mostrar el estado en la pantalla
float lastHvSocValue = 0;
int voltajeV = 228;
int voltajeTotal = 228;
float voltajeModulo = 3.72;
float PAValor = 0;

// Declara una variable global para almacenar el estado de EV
String EV = "NO";  // Inicialmente, asumimos que EV es "NO"
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
  //Serial.begin(115200);

  // Inicialización de componentes, como en tu código original
  tiempoAnterior = millis();  // Guarda el tiempo inicial

  if (Can0.begin(CAN_BPS_500K)) {
    //Serial.println("CAN0 inicializacion correcta");

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(5, 20, "CAN0 inicio correcto");
    u8g2.sendBuffer();
  } else {
    //Serial.println("CAN0 inicio ERROR");

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.drawStr(0, 55, "CAN0 inicializacion error");
    u8g2.sendBuffer();
  }

  delay(tiempoEspera);  // Esta línea se ejecutará después del if-else.

  error();  // llamada para borrar errores

  //Can0.watchFor();  // Anula filtro

  // Configura los filtros para los ID de interés (SOLO LEE LOS PRIMEROS 7)

  Can0.watchFor(0x3C8);  // ID 0x3C8, DLC 5  (256 * C + D)   ICE RPM Actua
  Can0.watchFor(0x3CB);  // ID 0x3CB, DLC 7, SOC Batería
  Can0.watchFor(0x03B);  // ID 0x03B, DLC 5, Voltaje batería EV
  Can0.watchFor(0x529);  // ID 0x529, DLC 7, Estado EV (botón)
  Can0.watchFor(0x3CA);  // ID 0x3CA, DLC 5, Velocidad
  Can0.watchFor(0x244);  // ID 0x244, DLC 8, Acelerarador
  Can0.watchFor(0x039);  // ID 0x039 , DLC 4, ICE Temperature (A) 4 30 02 0D 7C (de 0 a 255 °C)

  //Can0.watchFor(0x3CD);  // ID 0x3CD, DLC 5, Voltaje del paquete: 16 bits, sin signo [V] SOC

  // Can0.watchFor(0x3C9);  // ID 0x3C9, DLC 8, ( carga con potencia )
  // Can0.watchFor(0x4D1);  // ID 0x4D1, DLC 8, Datos desconocidos e inmutables. (A.V.: Batt -> HECU)

  // Can0.watchFor(0x07E221C4);  // ID 0x07E221C4, DLC 8, Ángulo acelerador
  // Can0.watchFor(0x18DB33F1);  // ID 0x18DB33F1, DLC 8, Pregunta conexion 7DF equivalente
  // Can0.watchFor(0x039);  // ID 0x039 , DLC 4, ICE Temperature (A) 4 30 02 0D 7C (de 0 a 255 °C)
  // Can0.watchFor(0x52C);  // ID 0x52C , DLC 2, ICE Temperature (B/2) 2 23 60 (de 0 a 127 °C)
  // Can0.watchFor(0x5A4);  // ID 0x5A4 , DLC 2, Fuel Tank Level Measured (B) 2 63 11 (full tank 0x2C/44)

  u8g2.begin();
  do {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(5, 20, "Ciclo de espera");
    u8g2.setCursor(6, 40);
    u8g2.drawLine(6, 35, 120, 35);
    u8g2.drawStr(0, 55, "CAN0 verificado");
    u8g2.sendBuffer();
  } while (u8g2.nextPage());

  // Configura las entradas y salidas
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  pinMode(botonPin, INPUT_PULLUP);
  pinMode(interruptorPin, INPUT_PULLUP);
  pinMode(outputPinD3, OUTPUT);
  pinMode(outputPinD4, OUTPUT);
  pinMode(outputPinA0, OUTPUT);
  pinMode(outputPinA1, OUTPUT);

  // Inicializa las salidas
  digitalWrite(outputPinD3, LOW);
  digitalWrite(outputPinD4, LOW);
  digitalWrite(outputPinA0, HIGH);
  digitalWrite(outputPinA1, LOW);

  // Esperar tiempoEspera antes de continuar
  delay(tiempoEspera);
  u8g2.clearBuffer();
  u8g2.firstPage();
  u8g2.clearDisplay();
  u8g2.setFont(u8g2_font_6x10_tf);
  updateu8g2();
}

void sendCANMessage(uint16_t id, uint8_t dlc, uint8_t data[8]) {
  CAN_FRAME frame;
  frame.id = id;  // Utiliza directamente el ID de 11 bits
  frame.length = dlc;
  frame.extended = false;  // Asegura que estás utilizando mensajes de 11 bits

  for (int i = 0; i < dlc; i++) {
    frame.data.byte[i] = data[i];
  }

  Can0.sendFrame(frame);
}

void error() {  // Borra los códigos de error del vehiculo (Toyota Prius)
  // Mensaje CAN ID 7DF (11 bits), DLC 8, Datos 02 01 00 00 00 00 00 00
  uint8_t data1[] = { 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendCANMessage(0x7DF, 8, data1);
  sentMessageCount++;
  // Mensaje CAN ID 7E0 (11 bits), DLC 8, Datos 02 3E 00 00 00 00 00 00
  uint8_t data2[] = { 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendCANMessage(0x7E0, 8, data2);
  sentMessageCount++;
  // Mensaje CAN ID 7E2 (11 bits), DLC 8, Datos 02 3E 00 00 00 00 00 00
  uint8_t data3[] = { 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendCANMessage(0x7E2, 8, data3);
  sentMessageCount++;
  // Mensaje CAN ID 7E3 (11 bits), DLC 8, Datos 02 3E 00 00 00 00 00 00
  uint8_t data4[] = { 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendCANMessage(0x7E3, 8, data4);
  sentMessageCount++;
  // Mensaje CAN ID 7E8 (11 bits), DLC 8, Datos 01 44 00 00 00 00 00 00
  uint8_t data5[] = { 0x01, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendCANMessage(0x7E8, 8, data5);
  sentMessageCount++;
}

void mensajeSOC() {
  // Mensaje CAN ID 3CB (11 bits), DLC 7, Datos 69(CDL) 7D(CCL) 00(DELTA SOC) 93(SOC) 21(TEM1) 20(TEM2) 8F (ChkSum)
  // uint8_t data6[] = { 0x69, 0x7D, 0x00, 0x92, 0x21, 0x20, 0x8F };//soc 73
  // uint8_t data6[] = { 0x67, 0x64, 0x00, 0x99, 0x15, 0x13, 0x61 }; //soc 76,5
  uint8_t data6[] = { 0x69, 0x7D, 0x00, 0x93, 0x21, 0x20, 0x8F };  //soc 73,5
  for (int i = 0; i < repMensaje; i++) {                           // Número repeticiones enviando el mensaje.
    // Envía el mensaje
    sendCANMessage(0x3CB, 7, data6);
    sentMessageCount++;
  }
}

void avisoRv() {  // Aviso rele control voltaje batería
  // Verifica si ha pasado al menos 1 segundo desde el último avisoRv
  unsigned long currentMillis = millis();
  if (currentMillis - lastAvisoRvTime >= avisoRvInterval) {
    u8g2.setCursor(83, 54);
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
    u8g2.setCursor(103, 54);
    u8g2.print("TODO");
    u8g2.sendBuffer();

    // Actualiza el tiempo del último avisoRc
    lastAvisoRcTime = currentMillis;
  }
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
  u8g2.drawBox(batteryIconX, batteryIconY, iconWidth, batteryIconHeight / 2.5);
}

// Asumiendo que voltajeModulo ya ha sido calculado anteriormente en tu código

// Inicializar cargaModuloBat
int calcularCargaModuloBat(float voltajeModulo) {
  if (voltajeModulo >= 4.2) return 100;
  if (voltajeModulo <= 3.61) return voltajeModulo < 3.5 ? 0 : 5;  // Asume 0% para voltajes < 3.5V

  // Puntos conocidos (voltaje, porcentaje)
  float puntos[][2] = {
    { 4.2, 100 }, { 4.15, 95 }, { 4.11, 90 }, { 4.08, 85 }, { 4.02, 80 }, { 3.98, 75 }, { 3.95, 70 }, { 3.91, 65 }, { 3.87, 60 }, { 3.85, 55 }, { 3.84, 50 }, { 3.82, 45 }, { 3.80, 40 }, { 3.79, 35 }, { 3.77, 30 }, { 3.75, 25 }, { 3.73, 20 }, { 3.71, 15 }, { 3.69, 10 }, { 3.61, 5 }
  };

  // Interpolación lineal entre puntos
  for (int i = 0; i < sizeof(puntos) / sizeof(puntos[0]) - 1; i++) {
    if (voltajeModulo >= puntos[i + 1][0] && voltajeModulo < puntos[i][0]) {
      // Interpolar entre puntos[i] y puntos[i+1]
      float m = (puntos[i][1] - puntos[i + 1][1]) / (puntos[i][0] - puntos[i + 1][0]);
      float b = puntos[i][1] - m * puntos[i][0];
      return static_cast<int>(m * voltajeModulo + b);
    }
  }

  return 0;  // Por defecto, si no se ajusta a ningún rango
}

void dibujarNivelCarga(int cargaModuloBat) {
  // Configuración de la barra de carga
  const int maxCarga = 100;                    // Valor máximo de carga en porcentaje
  const int alturaBarra = 4;                   // Altura de la barra de carga
  const int espaciadoBarra = 1;                // Espacio entre barras, si es necesario
  const int posXBarra = 0;                     // Posición X de la barra de carga
  const int posYBarra = 4;                     //u8g2.getDisplayHeight() - alturaBarra - espaciadoBarra;  // Posición Y de la barra de carga, en la parte inferior
  const int anchoBarra = 128 - posXBarra * 2;  // Ancho de la barra de carga

  // Mapea el porcentaje de carga al ancho de la barra
  int longitudBarra = map(cargaModuloBat, 0, maxCarga, 0, anchoBarra);

  // Dibuja el marco de la barra de carga
  // u8g2.drawFrame(posXBarra, posYBarra, anchoBarra, alturaBarra);

  // Dibuja la barra de carga según el porcentaje actual
  u8g2.drawBox(posXBarra, posYBarra, longitudBarra, alturaBarra - 1);
  /*
  // Opcional: Muestra el porcentaje de carga como texto en el centro de la barra
  u8g2.setCursor(anchoBarra / 2 - 10, posYBarra + alturaBarra / 2 + 1);  // Ajusta según la fuente
  u8g2.print(cargaModuloBat);
  u8g2.print("%");
  */
}


void updateu8g2() {
  u8g2.clearBuffer();
  //  Llamar a la función para dibujar el ícono del acelerador
  drawThrottleBar(decimalValue);
  // Llamar a la función para dibujar el ícono SOC de la batería con el valor actual de porcentaje
  drawBatteryIcon(hvSocValue);
  // Dibuja el nivel de carga de la batería
  dibujarNivelCarga(cargaModuloBat);

  // Mostrar la energía acumulada
  u8g2.setCursor(0, 54);
  u8g2.print("Gas:");
  u8g2.print(energiaAcumulada, 0);
  u8g2.print("Wh");

  // Mostrar la energía regenerada
  u8g2.setCursor(70, 54);
  u8g2.print("Reg:");
  u8g2.print(energiaRegenerada, 0);
  u8g2.print("Wh");

  //Temperatura 0x039
  u8g2.setCursor(31, 64);
  u8g2.print("Tem:");
  u8g2.print(temperatura, DEC);

  //Revoluciones Rpm.
  u8g2.setCursor(79, 64);
  u8g2.print("RM:");
  u8g2.print(rpmValor);

  // Mostrar el porcentaje de carga de la batería en la posición (100, 10) de la pantalla LCD
  u8g2.setCursor(0, 18);
  u8g2.print("SOC:");
  u8g2.print(hvSocValue, 1);
  u8g2.print("%");

  // Mostrar la intensidad de corriente en la posición (0, 33) de la pantalla LCD
  u8g2.setCursor(0, 33);
  u8g2.print("A:");
  u8g2.print(PAValor, 0);
  drawCurrentIcon(21, PAValor);

  // Mostrar el voltaje en la posición (92, 33) de la pantalla LCD
  u8g2.setCursor(72, 33);
  u8g2.print("V:");
  u8g2.print(voltajeTotal);

  // Mostrar el valor de EV en la posición (50, 33) de la pantalla LCD
  u8g2.setCursor(38, 33);
  u8g2.print("EV:");
  u8g2.print(EV);

  // Mostrar el estado de la salida A0 (R.Bat.H o R.Bat.L) en la posición (80, 54) de la pantalla LCD
  u8g2.setCursor(109, 33);
  if (outputA0Activated) {
    u8g2.print("Baj");
  } else {
    u8g2.print("Sub");
  }

  // Mostrar si el SOC Esta o no en rango
  u8g2.setCursor(90, 18);
  if (hvSocValue <= marcaBaja || hvSocValue >= marcaAlta) {
    u8g2.print("No");
  } else {
    u8g2.print("Si");
  }

  // Aplicar multiplicador si outputA0Activated está activo
  if (outputA0Activated) {
    voltajeModulo = (voltajeV * factor) / 60.0;
    voltajeTotal = (voltajeV * factor);
  } else {
    voltajeModulo = voltajeV / 60.0;
    voltajeTotal = voltajeV;
  }

  u8g2.setCursor(60, 18);

  // Obtén el tiempo actual
  unsigned long currentMillisV = millis();

  // Verifica si es momento de cambiar el estado del parpadeo
  if (currentMillisV - previousMillisV >= interval) {
    // Guarda el último tiempo de cambio
    previousMillisV = currentMillisV;

    // Cambia el estado de visualización
    displayOn = !displayOn;
  }

  // Si voltajeV es menor que volLimHiteresis y es momento de mostrar el valor
  if (voltajeV < volLimHiteresis && displayOn) {
    // Convertir voltajeModulo a String con 2 decimales para imprimir
    String voltajeModuloStr = String(voltajeModulo, 2);
    u8g2.print(voltajeModuloStr);
  } else if (voltajeV >= volLimHiteresis) {
    // Siempre muestra el valor si voltajeV es mayor o igual a volLimHiteresis
    // Convertir voltajeModulo a String con 2 decimales para imprimir
    String voltajeModuloStr = String(voltajeModulo, 2);
    u8g2.print(voltajeModuloStr);
  }


  // Ahora cargaModuloBat contiene el estado de carga estimado basado en voltajeModulo

  u8g2.setCursor(108, 18);
  u8g2.print(cargaModuloBat);
  u8g2.print("%");
  // Ahora cargaModuloBat contiene el estado de carga estimado

  //Mostrar los valores de lecturas y envíos por segundo en las posiciones (0, 54) y (30, 54) de la pantalla LCD
  //u8g2.setCursor(0, 64);
  //u8g2.print("L:");
  //u8g2.print(readingsPerSecond);

  // Mostrar el valor de mensajes enviados por segundo (E:) en la posición (38, 54) de la pantalla LCD
  u8g2.setCursor(0, 64);
  u8g2.print("E:");
  u8g2.print(sentMessagesPerSecond);
  u8g2.sendBuffer();  // Enviar los datos a la pantalla LCD
}

// Declaración de la función para actualizar el estado de la salida A0
void updateOutputA0(bool state) {
  digitalWrite(A0, state ? HIGH : LOW);
}
void processCanMessage(CAN_FRAME& incoming) {
  readCount++;
  readCountPerSecond++;

  // Temperatura

  if (incoming.id == 0x039 && incoming.length == 4) {  // Ajustado el número de bytes a 4
    byte byte0 = incoming.data.byte[0];

    temperatura = byte0;
  }

  // Revoluciones
  if (incoming.id == 0x3C8 && incoming.length == 5) {  // Ajustado el número de bytes a 5
    byte byte2 = incoming.data.byte[2];
    byte byte3 = incoming.data.byte[3];

    rpmValor = ((byte2 * 256) + byte3) / 8;
  }

  // Control condiciones para envío de mensaje SOC
  if (incoming.id == 0x3CB && incoming.length == 7) {
    byte byte2 = incoming.data.byte[2];
    byte byte3 = incoming.data.byte[3];
    hvSocValue = ((byte2 * 256 + byte3) / 2.0);
    mensajeSOC();

    // Reactivar mensajeSOC solo si hvSocValue alcanza o supera histeresisAlto
    if (!permitirMensajeSOC && hvSocValue >= histeresisAlto) {
      permitirMensajeSOC = true;
    }

    // Desactivar mensajeSOC una vez que hvSocValue cae por debajo de limiteBajoSOC
    if (hvSocValue < limiteBajoSOC) {
      permitirMensajeSOC = false;
    }

    // Implementación de la histéresis para voltajeV
    if (voltajeV < volLimBajoBat && permitirMensajePorVoltaje) {
      permitirMensajePorVoltaje = false;  // Desactiva debido al voltaje bajo
    } else if (voltajeV >= volLimHiteresis && !permitirMensajePorVoltaje) {
      permitirMensajePorVoltaje = true;  // Reactiva debido al voltaje alto
    }
    /*
    // Ejecutar mensajeSOC bajo las condiciones específicas si ambas permitirMensajeSOC y permitirMensajePorVoltaje son true
    if (permitirMensajeSOC && permitirMensajePorVoltaje && hvSocValue >= limiteBajoSOC) {
      // Primera condición específica
      if ((hvSocValue < marcaBaja || hvSocValue > marcaAlta) || hvSocValue > socAlto) {
        mensajeSOC();
      }
      // Segunda condición específica
      else if (digitalRead(interruptorPin) == HIGH) {
        mensajeSOC();
      }
    }
    */
    if (hvSocValue != lastHvSocValue) {
      lastHvSocValue = hvSocValue;
    }
  }


  // Voltios
  if (incoming.id == 0x03B && incoming.length == 5) {
    byte byte1 = incoming.data.byte[1];
    byte byte3 = incoming.data.byte[3];
    ultimoMensajeRecibidoID = 0x03B;
    int hvV = (incoming.data.byte[2] * 256) + byte3;
    hvV = (hvV & 0x07FF) - (hvV & 0x0800);
    hvV *= 1;
    // Amperios
    int aValue = ((incoming.data.byte[0]) * 256) + (incoming.data.byte[1]);
    if ((aValue & 0x800) != 0) {
      aValue = aValue - 0x1000;
    }
    PAValor = aValue / 10;

    voltajeV = hvV;
  }

  // Acelerador posicion
  if (incoming.id == 0x244 && incoming.length == 8) {
    byte byte6 = incoming.data.byte[6];

    // Convertir valor hexadecimal a decimal
    decimalValue = byte6;
  }
}

void leerCan() {
  // Variables locales para almacenar mensajes CAN recibidos
  CAN_FRAME incoming;

  // Comprueba si hay datos CAN recibidos
  if (Can0.available()) {
    Can0.read(incoming);

    // Llama a tu función de procesamiento para procesar el mensaje CAN
    activaEv(incoming);  // Pasa el objeto CAN_FRAME como argumento

    // Verifica si el mensaje cumple con alguna de las condiciones
    if ((incoming.id == 0x3CB && incoming.length == 7) || (incoming.id == 0x03B && incoming.length == 5) || (incoming.id == 0x529 && incoming.length == 7) || (incoming.id == 0x3CA && incoming.length == 5) || (incoming.id == 0x244 && incoming.length == 8) || (incoming.id == 0x3C9 && incoming.length == 8) || (incoming.id == 0x039 && incoming.length == 4) || (incoming.id == 0x4D1 && incoming.length == 8) || (incoming.id == 0x3C8 && incoming.length == 5)) {

      // Llama a tu función de procesamiento para procesar el mensaje CAN
      processCanMessage(incoming);
    }
  }
}

void activaEv(CAN_FRAME& canMsg) {
  unsigned long currentMillisEv = millis();

  // Calcular el tiempo desde la última recepción de datos
  tiempoSinDatos = currentMillisEv - tiempoUltimaAplicacion;

  if (canMsg.id == 0x529) {
    if (canMsg.data.byte[4] == 0x40) {
      // Si can_id es 0x529 y data[4] es 0x40, establece el mensaje en "SI"
      EV = "SI";
    } else {
      // Si can_id es 0x529 pero data[4] no es 0x40, establece el mensaje en "NO"
      EV = "NO";
    }
  }

  if (EV == "NO") {
    // Verificar la velocidad del vehículo
    if (canMsg.id == 0x3CA && activacionPendiente == false && canMsg.data.byte[2] <= 0x2D && hvSocValue >= 50) {
      // Verificar si la velocidad es igual o inferior a 45 km/h en hexadecimal
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
    // Si lastHvSocValue es menor o igual a marcaBaja, DESACTIVAR la salida A0

    if (!lastOutputA0State && elapsedTime >= tiempoInactivoA0) {
      outputA0Activated = false;  // Cambiado a false
      updateOutputA0(outputA0Activated);
      lastOutputA0ChangeMillis = currentMillis;
      lastOutputA0State = true;  // Mantener este true, ya que indica que se realizó un cambio
    }
  } else if ((voltajeV <= voltajeMinimo) && (hvSocValue <= socBajo)) {
    // Si lastHvSocValue no cumple la primera condición pero lastByte3 es menor o igual a voltajeMinimo
    if (!lastOutputA0State && elapsedTime >= tiempoInactivoA0) {
      outputA0Activated = false;  // Cambiado a false
      updateOutputA0(outputA0Activated);
      lastOutputA0ChangeMillis = currentMillis;
      lastOutputA0State = true;  // Mantener este true, ya que indica que se realizó un cambio
    }
  } else if (voltajeV >= voltajeMaximo) {
    // Si ninguna de las condiciones anteriores se cumple pero lastByte3 es mayor o igual a voltajeMaximo
    if (lastOutputA0State && elapsedTime >= tiempoActivoA0) {
      outputA0Activated = true;  // Cambiado a true
      updateOutputA0(outputA0Activated);
      lastOutputA0ChangeMillis = currentMillis;
      lastOutputA0State = false;  // Mantener este false, ya que indica que se realizó un cambio
    }
  } else {
    // Si ninguna de las condiciones anteriores se cumple, restablecer el estado de salida A0 a su valor anterior
    if (lastOutputA0State && elapsedTime >= tiempoActivoA0) {
      outputA0Activated = true;  // Cambiado a true
      updateOutputA0(outputA0Activated);
      lastOutputA0ChangeMillis = currentMillis;
      lastOutputA0State = false;  // Mantener este false, ya que indica que se realizó un cambio
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

  // Comprueba si el botón se ha pulsado
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Borra el error
    error();
  }

  // Parpadea C:M en la pantalla
  if (switchState == HIGH) {
    avisoRc();

    // Verifica que al menos uno de los pines D3 o D4 esté alto
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

  leerCan();           // Llama a leerCan() en el bucle principal
  activaEv(incoming);  // Pasa el objeto CAN_FRAME como argumento
  conCarga();
  lecSegundo();
  cargaModuloBat = calcularCargaModuloBat(voltajeModulo);  // Calcula el porcentaje de carga de la batería
  unsigned long tiempoActual = millis();

  // Verifica si ha pasado 1 segundo desde la última actualización
  if (tiempoActual - tiempoAnterior >= intervaloActualizacion) {
    // Calcula la potencia instantánea en vatios
    float potenciaInstantanea = voltajeTotal * PAValor;  // PAValor puede ser positivo o negativo

    // Actualiza los acumuladores de energía
    if (PAValor > 0) {
      // Energía consumida
      energiaAcumulada += (potenciaInstantanea / 3600.0);  // Convertir vatios a vatios-hora
    } else if (PAValor < 0) {
      // Energía regenerada
      energiaRegenerada += (-potenciaInstantanea / 3600.0);  // Convertir vatios a vatios-hora, PAValor es negativo
    }

    // Actualiza el tiempo anterior para la próxima iteración
    tiempoAnterior = tiempoActual;
  }
}
