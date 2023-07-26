/*
  Este código es una implementación para Arduino Nano Every
  utilizando una pantalla u8g2 y un módulo CAN
  (Controller Area Network). El código controla y muestra información sobre el
  estado de la batería de alto voltaje sustituida en un Toyota Prius Gen 2
  y realiza comunicación a través del bus CAN. Consiguiendo su optimización de funcionamiento.

  https://github.com/Lorevalles/Proyecto_Prius
*/
#include <U8g2lib.h>
#include <SPI.h>
#include <mcp2515.h>
#include <avr/wdt.h>
#include <Wire.h>

// Definiciones de pines
#define SCL 6
#define SI 5
#define CS 9
#define RS 7
#define RSE 8

// Variables globales
U8G2_ST7565_ERC12864_F_4W_SW_SPI u8g2(U8G2_R0, SCL, SI, CS, RS, RSE);
MCP2515 mcp2515(10);
struct can_frame canMsg;

// Variables para mostrar el estado en la pantalla
float lastHvSocValue = 0.0;
int lastByte3 = 0;
int PAValor = 0;
int EV = 0;

// Control de tiempo y recuento
unsigned int readCountPerSecond = 0;
unsigned int readCount = 0;

// Definición de pines para salidas
const int outputPinD3 = 3;
const int outputPinD4 = 4;
const int outputPinA0 = A0;

// Variables para el control de tiempo
unsigned long waitStartTime = 0;
const unsigned long waitDuration = 5000; // Duración de espera inicial en milisegundos (5 segundos)
bool waiting = true; // Variable para controlar si estamos en el tiempo de espera inicial
unsigned long lastMessageSentTime = 0;
unsigned long previousMillis = 0;
unsigned long previousMessageMillis = 0;
unsigned long previousLcdUpdateMillis = 0;
const unsigned long lcdUpdateInterval = 1000; // Intervalo de actualización de la pantalla LCD (500 milisegundos)
const unsigned long messageInterval = 108;    // Intervalo de envío de mensaje (108 milisegundos)

// Variables para el control de tiempo y ciclos por segundo
unsigned long previousSecondMillis = 0;
unsigned int readingsPerSecond = 0;
unsigned int sentMessageCount = 0;
// Contadores de mensajes enviados
unsigned int totalSentMessages = 0;

// Estado de las salidas D3 y D4
bool d3Activated = false;
bool lastD3Activated = false;
// Variable para almacenar el estado actual de la salida D14
bool outputD14Activated = false;
// Variable para indicar si la pantalla LCD necesita actualizarse
bool lcdNeedsUpdate = true;

// Indicador de mensaje válido recibido en el bus CAN
bool message_ok = false;

// Función para dibujar el ícono de la batería en la pantalla LCD
void drawBatteryIcon(uint8_t x, uint8_t y, uint8_t percentage) {
  u8g2.drawFrame(x, y, 32, 8); // Ajustar la altura a 8
  u8g2.drawFrame(x + 32, y + 2, 4, 4); // Ajustar la altura a 8 y desplazar verticalmente para centrar

  // Ajustar los valores de 'y' para centrar el icono verticalmente
  uint8_t lineY1 = y + 4;
  uint8_t lineY2 = y + 10;

  u8g2.drawLine(x + 36, lineY1, x + 36, lineY2);
  u8g2.drawLine(x + 37, lineY1 + 1, x + 37, lineY2 - 1);

  uint8_t width = map(percentage, 0, 100, 0, 30);
  u8g2.drawBox(x + 1, y + 1, width, 6); // Ajustar la altura a 6 para que se vea centrado en la caja de 8
}

void drawCurrentIcon(uint8_t y, int value) {
  const uint8_t maxCurrent = 100; // Valor máximo de intensidad (por ejemplo, 100 Amperios)
  const uint8_t barHeight = 10;
  const uint8_t screenWidth = 128;

  // Calcular el ancho de la barra proporcional al valor de la intensidad
  int barLength = map(abs(value), 0, maxCurrent, 0, screenWidth);

  u8g2.drawFrame(0, y, screenWidth, barHeight);

  if (value >= 0) {
    // Dibujar la barra que representa la intensidad en el sentido positivo (derecha)
    u8g2.drawBox(0, y + 1, barLength, barHeight - 2);
  } else {
    // Dibujar la barra que representa la intensidad en el sentido negativo (izquierda)
    int xStart = screenWidth - barLength;
    u8g2.drawBox(xStart, y + 1, barLength, barHeight - 2);
  }
}

void setup() {
  u8g2.begin();
  u8g2.setContrast(20); // Contraste
  //u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(5, 20, "Ciclo de espera");
  u8g2.setCursor(6, 40);
  u8g2.drawLine(6, 35, 120, 35);
  u8g2.drawStr(0, 55, "Se puede PARAR");
  // Almacenar el tiempo de inicio de espera
  waitStartTime = millis();
  u8g2.sendBuffer();

  // Inicialización de las salidas digitales
  pinMode(outputPinD3, OUTPUT);
  pinMode(outputPinD4, OUTPUT);
  pinMode(outputPinA0, OUTPUT);
  digitalWrite(outputPinD3, HIGH);
  digitalWrite(outputPinD4, HIGH);
  digitalWrite(outputPinA0, LOW);

  // Inicialización de la comunicación SPI para el módulo CAN
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}
// Función para actualizar la salida D14 y mostrar el estado en la pantalla LCD
void updateOutputD14(bool activated) {
  digitalWrite(outputPinA0, activated ? HIGH : LOW);
}

void updateu8g2() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Dibujar el ícono de la batería
  drawBatteryIcon(0, 0, static_cast<uint8_t>(lastHvSocValue));
  u8g2.setCursor(40, 8);
  u8g2.print("SOC:");
  u8g2.print(lastHvSocValue, 1);
  u8g2.print("%");

  u8g2.setCursor(0, 20);
  u8g2.print("A:");
  u8g2.print(PAValor);
  // Llamar a la función para dibujar el ícono de la intensidad
  drawCurrentIcon(21, PAValor);

  u8g2.setCursor(80, 20);
  u8g2.print("V:");
  u8g2.print(lastByte3);

  u8g2.setCursor(45, 20);
  u8g2.print("EV:");
  u8g2.print(EV);

  // Mostrar el estado de la salida D14 (R.Bat.H o R.Bat.L)
  u8g2.setCursor(80, 40);
  if (outputD14Activated) {
    u8g2.print("R.Bat.H");
  } else {
    u8g2.print("R.Bat.L");
  }

  // Mostrar si el mensaje CAN se envió correctamente (S) o no (N)
  u8g2.setCursor(60, 40);
  if (lastHvSocValue < 60 || lastHvSocValue > 79) {
    if (sentMessageCount > 0) {
      u8g2.print("S");
    } else {
      u8g2.print("N");
    }
  } else {
    u8g2.print(" ");
  }

  if (d3Activated) {
    u8g2.setCursor(0, 50);
    u8g2.print("NO PARAR");
  } else {
    u8g2.setCursor(0, 50);
    u8g2.print("SI POWER");
  }

  // Mostrar los valores de lecturas y envíos por segundo
  u8g2.setCursor(0, 40);
  u8g2.print("L:");
  u8g2.print(readingsPerSecond);

  u8g2.setCursor(30, 40); // Ajustamos la posición de la columna para imprimir sentMessageCount
  u8g2.print("E:");
  u8g2.print(sentMessageCount);

  if (lastHvSocValue > 76) {
    digitalWrite(outputPinD4, LOW);
    u8g2.setCursor(60, 50);
    u8g2.print("Ac.Descarga");
  } else {
    digitalWrite(outputPinD4, HIGH);
    u8g2.setCursor(60, 50);
    u8g2.print("De.Inactiva");
  }

  u8g2.setCursor(0, 30);
  if (PAValor >= 0) {
    int lineLength = map(PAValor, 0, 100, 0, 20);
    for (int i = 0; i < 20; i++) {
      if (i >= 20 - lineLength) {
        u8g2.print("-");
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
  u8g2.sendBuffer(); // Enviar los datos a la pantalla LCD
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
      updateu8g2();
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
    updateu8g2();
  }
  // EV
  if (canMsg.can_id == 0x529 && !message_ok) {
    byte byte4 = canMsg.data[4];
    EV = byte4;
    message_ok = true;
  }
}
void loop() {
  unsigned long currentMillis = millis();

  // Realizar la espera inicial de 5 segundos
  if (waiting) {
    if (currentMillis - waitStartTime < waitDuration) {
      return;
    } else {
      waiting = false;
    }
  }

  // Leer mensajes CAN y procesarlos
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    processCanMessage();
    readingsPerSecond++; // Incrementar el contador de lecturas por segundo
    lcdNeedsUpdate = true; // Indicar que la pantalla LCD necesita actualizarse
  }

  // Actualizar la pantalla LCD solo si es necesario
  if (lcdNeedsUpdate && currentMillis - previousLcdUpdateMillis >= lcdUpdateInterval) {
    previousLcdUpdateMillis = currentMillis;
    updateu8g2();
    lcdNeedsUpdate = false; // Restablecer el indicador de actualización de la pantalla LCD
  }

  // Enviar mensajes CAN a intervalos regulares
  if (currentMillis - previousMessageMillis >= messageInterval) {
    previousMessageMillis = currentMillis;

    // Envío del mensaje CAN dependiendo del valor de SOC
    if (lastHvSocValue < 60 || lastHvSocValue > 79) {
      canMsg.can_id = 0x3CB;
      canMsg.can_dlc = 7;
      canMsg.data[0] = 0x69;
      canMsg.data[1] = 0x7D;
      canMsg.data[2] = 0x00;
      canMsg.data[3] = 0x93;
      canMsg.data[4] = 0x21;
      canMsg.data[5] = 0x20;
      canMsg.data[6] = 0x8F;

      if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
        totalSentMessages++; // Incrementar el contador total de mensajes enviados
        sentMessageCount++; // Actualizar el contador de mensajes enviados
        u8g2.setCursor(60, 40);
        u8g2.print("S");
      } else {
        u8g2.setCursor(60, 40);
        u8g2.print("N");
      }
    }
  }

  // Controlar la salida D14 dependiendo del valor de voltaje
  if (lastByte3 <= 220) {
    // Si el valor es igual o menor a 220, activar la salida D14
    if (!outputD14Activated) {
      outputD14Activated = true;
      updateOutputD14(outputD14Activated);
    }
  } else if (lastByte3 >= 230) {
    // Si el valor es mayor o igual a 230, desactivar la salida D14
    if (outputD14Activated) {
      outputD14Activated = false;
      updateOutputD14(outputD14Activated);
    }
  }

  // Actualizar el recuento de mensajes enviados por segundo y lecturas por segundo
  if (currentMillis - previousSecondMillis >= 1000) {
    previousSecondMillis = currentMillis;

    readingsPerSecond = 0; // Reiniciar el contador de lecturas por segundo
    sentMessageCount = 0; // Reiniciar el contador de mensajes enviados por segundo

    // Actualizar valores de lecturas y mensajes enviados en el LCD
    lcdNeedsUpdate = true;
  }

  // Controlar la salida D3 dependiendo del valor de SOC
  if (lastHvSocValue <= 70 && !d3Activated) {
    digitalWrite(outputPinD3, LOW);
    d3Activated = true;
    lcdNeedsUpdate = true; // Indicar que la pantalla LCD necesita actualizarse
  } else if (lastHvSocValue >= 74 && d3Activated) {
    digitalWrite(outputPinD3, HIGH);
    d3Activated = false;
    lcdNeedsUpdate = true; // Indicar que la pantalla LCD necesita actualizarse
  } else if (lastHvSocValue > 70 && lastHvSocValue < 74) {
    if (d3Activated != lastD3Activated) {
      digitalWrite(outputPinD3, !d3Activated);
      lastD3Activated = d3Activated;
      lcdNeedsUpdate = true; // Indicar que la pantalla LCD necesita actualizarse
    }
  }
}
