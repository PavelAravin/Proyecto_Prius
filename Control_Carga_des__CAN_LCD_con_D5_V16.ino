#include <SPI.h>
#include <mcp2515.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

struct can_frame canMsg;
MCP2515 mcp2515(10);

unsigned int readCountPerSecond = 0;
unsigned int readCount = 0;
unsigned int startMessageCount = 0;
unsigned int sentMessageCount = 0;

const int outputPinD3 = 3;
const int outputPinD4 = 4;
const int outputPinD5 = 5; // Agregar la definición del pin D5

const unsigned int MASK_12BIT = 0x0FFF;

bool d3Activated = false;
bool lastD3Activated = false;

float lastHvSocValue = 0.0;
int lastByte3 = 0;
int PAValor = 0;
int EV = 0;

unsigned long previousMillis = 0;
const unsigned long interval = 1000;
const unsigned long messageInterval = 108; // Intervalo de envío de mensaje (108 milisegundos)

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Ciclo de espera");
  delay(5000);

  pinMode(outputPinD3, OUTPUT);
  pinMode(outputPinD4, OUTPUT);
  pinMode(outputPinD5, OUTPUT); // Establecer el pin D5 como salida

  digitalWrite(outputPinD3, HIGH);
  digitalWrite(outputPinD4, HIGH);
  digitalWrite(outputPinD5, LOW); // Establecer el pin D5 en estado bajo (desactivado)

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  lcd.clear();
}
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("SOC:");
  lcd.print(lastHvSocValue, 1);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("A:");
  lcd.setCursor(3, 1);
  lcd.print("     ");
  lcd.setCursor(3, 1);
  lcd.print(PAValor);

  lcd.setCursor(13, 1);
  lcd.print("V:");
  lcd.setCursor(15, 1);
  lcd.print(lastByte3);

  lcd.setCursor(8, 1);
  lcd.print("EV:");
  lcd.setCursor(10, 1);
  lcd.print(EV);

  if (d3Activated) {
    lcd.setCursor(0, 3);
    lcd.print("NO PARAR");
  } else {
    lcd.setCursor(0, 3);
    lcd.print("SI POWER");
  }

  if (lastHvSocValue > 76) {
    digitalWrite(outputPinD4, LOW);
    lcd.setCursor(9, 3);
    lcd.print("Ac.Descarga");
  } else {
    digitalWrite(outputPinD4, HIGH);
    lcd.setCursor(9, 3);
    lcd.print("De.Inactiva");
  }

  lcd.setCursor(9, 0);
  lcd.print("L:");
  lcd.setCursor(11, 0);
  lcd.print("   ");
  lcd.setCursor(12, 0);
  lcd.print(readCountPerSecond);

  lcd.setCursor(14, 0);
  lcd.print("M:");
  lcd.setCursor(16, 0);
  lcd.print("   ");
  lcd.setCursor(17, 0);
  lcd.print(sentMessageCount);

  lcd.setCursor(0, 2);
  if (PAValor >= 0) {
    int lineLength = map(PAValor, 0, 100, 0, 20);
    for (int i = 0; i < 20; i++) {
      if (i >= 20 - lineLength) {
        lcd.print("-");
      } else {
        lcd.print(" ");
      }
    }
  } else {
    int lineLength = map(PAValor, 0, -100, 0, 20);
    for (int i = 0; i < 20; i++) {
      if (i < lineLength) {
        lcd.print("-");
      } else {
        lcd.print(" ");
      }
    }
  }
}
bool message_ok = false; // Declarar e inicializar la variable message_ok

unsigned long lastMessageSentTime = 0; // Agregar una variable para almacenar el último momento en que se envió el mensaje

void processCanMessage() {

  // SOC
  if (canMsg.can_id == 0x3CB && canMsg.can_dlc == 7) {
    byte byte2 = canMsg.data[2];
    byte byte3 = canMsg.data[3];
    float hvSocValue = ((byte2 * 256 + byte3) / 2.0);

    if (hvSocValue != lastHvSocValue) {
      lastHvSocValue = hvSocValue;
      updateLCD();
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
    updateLCD();
  }
  // EV
  if (canMsg.can_id == 0x529 && !message_ok) {
    byte byte4 = canMsg.data[4];
    EV = byte4;
    message_ok = true;
    // Aquí puedes hacer otras operaciones si es necesario
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    processCanMessage();
    readCount++;
  }

  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    readCountPerSecond = readCount;
    readCount = 0;
    sentMessageCount = 0;
  }

  // Verificar si ha pasado suficiente tiempo para enviar el mensaje
  if (currentMillis - lastMessageSentTime >= messageInterval) {
    lastMessageSentTime = currentMillis;

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
        lcd.setCursor(19, 0);
        lcd.print("S");
        sentMessageCount++;
      } else {
        lcd.setCursor(19, 0);
        lcd.print("N");
      }
    }
  }

  // Verificar y controlar la salida digital D5
  if (lastByte3 < 222 && lastByte3 > 230) {
    digitalWrite(outputPinD5, HIGH); // Desactivar la salida D5
    lcd.setCursor(19, 1);
    lcd.print("H");
  } else {
    digitalWrite(outputPinD5, LOW); // Activar la salida D5
    lcd.setCursor(19, 1);
    lcd.print("L");
  }

  if (lastHvSocValue <= 70 && !d3Activated) {
    digitalWrite(outputPinD3, LOW);
    d3Activated = true;
    updateLCD();
  } else if (lastHvSocValue >= 74 && d3Activated) {
    digitalWrite(outputPinD3, HIGH);
    d3Activated = false;
    updateLCD();
  } else if (lastHvSocValue > 70 && lastHvSocValue < 74) {
    if (d3Activated != lastD3Activated) {
      digitalWrite(outputPinD3, !d3Activated);
      lastD3Activated = d3Activated;
      updateLCD();
    }
  }
}
