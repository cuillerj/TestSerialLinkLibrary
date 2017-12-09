/*
    this a arduino code that works with the jaba ESP8266SerialUdpGatewayExample using a ESP8266 as a serial IP gateway
    It has been tested with arduino UNO and arduino Mega
    It is easiest to test with a Mega that have more than one UART
    When using arduino UNO USB connexion is no longer available so debuging is much more difficult
    It sends alternativly 2 different frames
    The first one contains 5 digital pin values
    The second one contains one incremental count and one random value
    It listens on a serial link
    If frame type 1 received it set the GPIO to the value sent by the server (GPIO 13 by default that will set LED onoff)
    If frame type 2 received it prints the received count and received random value
*/
#include <SerialLink.h>
#define gatewayLinkSpeed 38400
SerialLink GatewayLink(gatewayLinkSpeed);   // define the object link to the gateway
#define debugConnection true     // can only be set with ATmega2560 or ATmega1280

uint8_t PendingReqRefSerial = 0x01; // 0x01 request to the gateway means route (ready to eventualy add some more action in the gateway)
uint8_t PendingSecReqRefSerial = 0x01; // 0x01 request to the gateway means route
byte cycleRetrySendUnit = 0; // cycle retry check unitary command - used in case of acknowledgement needed from the Linux server
uint8_t trameNumber = 0;     // frame number to send
uint8_t lastAckTrameNumber = 0;  // last frame number acknowledged by the server
uint8_t pendingAckSerial = 0;    // flag waiting for acknowledged
int retryCount = 0;            // number of retry for sending
unsigned long timeSendSecSerial;  // used to check for acknowledgment of secured frames
unsigned long timeReceiveSerial;  // used to regularly check for received message
unsigned long timeSendInfo;     // to regurarly send information to th server
unsigned long durationSincegatewayReady;  //  duration since GPIO ready goes on
uint8_t gatewayStatus = 0x00;    // 0x00 not ready  0x01 ready GPIO on wait boot completed 0x02 ready
#define delayBetweenInfo 5000   // delay before sending new data to the server  
uint8_t sendInfoSwitch = 0x00;  // flag to switch between different data to send
unsigned int count = 0;

#define pin1 2
#define pin2 3
#define pin3 4
#define pin4 5
#define pin5 6
#define readyPin 7
#define pinLED1 12
#define pinLED2 13
#define analog1 0
#define receiveFlagPosition 0
#define countPosition 1
#define randomPosition 3
#define gpioPosition 1
#define gpioValue 2
void setup() {
  // put your setup code here, to run once:
  GatewayLink.SerialBegin();
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  Serial.begin(38400);
#endif
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);
  pinMode(pin5, INPUT);
  pinMode(readyPin, INPUT);
  pinMode(pinLED1, OUTPUT);
  pinMode(pinLED2, OUTPUT);
}

void loop() {

  if (digitalRead(readyPin) == 1 && gatewayStatus == 0x02) {
    // ***  keep in touch with the server
    int getSerial = GatewayLink.Serial_have_message();  // check if we have received a message
    if (getSerial > 0)                                  // we got a message
    {
#if defined(debugConnection)
      Serial.println("receive something");
#endif
      TraitInput(GatewayLink.DataInSerial[receiveFlagPosition]);          // analyze the message
    }
    if (GatewayLink.PendingReqSerial != 0x00)           // check if we have a message to send (or a message currently sending)
    {
      GatewayLink.DataToSendSerial();                    // send message on the serial link
      timeSendSecSerial = millis();                      // reset timer
    }

    // put your main code here, to run repeatedly:
    if (millis() - timeSendInfo >= delayBetweenInfo )  // alternatively send status and power to the server
    {
      if (sendInfoSwitch % 2 == 0)
      {
#if defined(debugConnection)
        Serial.println("frame 1 to be sent");
#endif
        Send1();                 //
      }
      if (sendInfoSwitch % 2 == 1)
      {
#if defined(debugConnection)
        Serial.println("frame 2 to be sent");
#endif
        Send2();             // 
        count++;
      }
      sendInfoSwitch = sendInfoSwitch + 1;
      timeSendInfo = millis();
    }
  }
  else {
    Serial.println("wait for esp8226 to be ready");
    delay(2000);
    if (digitalRead(readyPin) == 0)
    {
      gatewayStatus = 0x00;
    }
    if (digitalRead(readyPin) == 1 && gatewayStatus == 0x00)
    {
      gatewayStatus = 0x01;
      durationSincegatewayReady = millis();
    }
    if (millis() - durationSincegatewayReady > 15000 && gatewayStatus == 0x01)
    {
      gatewayStatus = 0x02;
    }
  }
}
void TraitInput(uint8_t cmdInput) {
  int len = GatewayLink.DataInSerial[3];

#if defined(debugConnection)
  Serial.print(".");
  for (int i = 0; i < len; i++) {
    Serial.print(GatewayLink.DataInSerial[4 + i], HEX);
  }
  Serial.print("-");
#endif
  switch (cmdInput) {                   // first byte of input is the command type
    case 0x01:
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      Serial.print("set GPIO ");
      Serial.print(GatewayLink.DataInSerial[gpioPosition]);
      Serial.print(" to:");
      Serial.println(GatewayLink.DataInSerial[gpioValue]);
#endif
      digitalWrite(GatewayLink.DataInSerial[gpioPosition], GatewayLink.DataInSerial[gpioValue]);
      break;
    case 0x02:
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      Serial.print("count recevied:");
      Serial.println(GatewayLink.DataInSerial[countPosition] * 256 + GatewayLink.DataInSerial[countPosition + 1]);
      Serial.print("random received:");
      Serial.println(GatewayLink.DataInSerial[randomPosition] * 256 + GatewayLink.DataInSerial[randomPosition + 1]);
#endif
      break;
    default:
      break;
  }
#if defined(debugConnection)
  Serial.println();
#endif
}
void Send1()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;         // this byte is used by SerialLink code
  GatewayLink.PendingDataReqSerial[0] = 0x01; //       flag for server traitment
  GatewayLink.PendingDataReqSerial[1] = uint8_t(digitalRead(pin1)); //
  GatewayLink.PendingDataReqSerial[2] = uint8_t(digitalRead(pin2));
  GatewayLink.PendingDataReqSerial[3] = uint8_t(digitalRead(pin3));
  GatewayLink.PendingDataReqSerial[4] = uint8_t(digitalRead(pin4));
  GatewayLink.PendingDataReqSerial[5] = uint8_t(digitalRead(pin5));
  GatewayLink.PendingDataLenSerial = 0x06; // 6 longueur mini max 30 pour la gateway
}
void Send2()
{
  GatewayLink.PendingReqSerial = PendingReqRefSerial;     // this byte is used by SerialLink code
  GatewayLink.PendingDataReqSerial[0] = 0x02;             // flag for server traitment
  GatewayLink.PendingDataReqSerial[1] = uint8_t(random(0, 256));
  GatewayLink.PendingDataReqSerial[2] = uint8_t(random(0, 256));
  GatewayLink.PendingDataReqSerial[3] = 0x00;  // 0x00 or ascii byte mandatory to avoid the forbiden sequence to be sent
  GatewayLink.PendingDataReqSerial[4] = uint8_t(random(0, 256));
  GatewayLink.PendingDataReqSerial[5] = uint8_t(random(0, 256));
  int A0Level = map(analogRead(analog1), 0, 1023, 0, 5000);
  GatewayLink.PendingDataReqSerial[6] = uint8_t(A0Level / 256);
  GatewayLink.PendingDataReqSerial[7] = uint8_t(A0Level );
  GatewayLink.PendingDataReqSerial[8] = uint8_t(count / 256);
  GatewayLink.PendingDataReqSerial[9] = uint8_t(count);
  GatewayLink.PendingDataReqSerial[29] = 0xff;
  GatewayLink.PendingDataLenSerial = 0x1e; // 6 longueur mini max 30 pour la gateway
}

