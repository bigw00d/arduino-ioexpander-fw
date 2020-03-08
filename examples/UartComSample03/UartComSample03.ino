// UART Command Sample (Digital Write)

// D10(RX)  <-  TX
// D11(TX)  ->  RX

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX = D10, D11

#define RW_DATA_MAX_SIZE 50
int readCnt;
char readData[RW_DATA_MAX_SIZE];
int sendCnt;
char sendData[RW_DATA_MAX_SIZE];

#define UART_CMD_PREAMBLE 0x5A

enum UART_CMD_PHASE {
  UART_CMD_PHASE_SET_DIR_W = 0,
  UART_CMD_PHASE_OUT_H,
  UART_CMD_PHASE_OUT_L,
  UART_REG_TYPE2_NUM,
};
int uartCmdPhase = UART_CMD_PHASE_SET_DIR_W;

#define LED_CMD_WRITE_IO_DIR_SIZE 4
#define LED_CMD_WRITE_GPIO_SIZE 4

// Write : IO_DIR(ALL OUT)
const uint8_t writeIO_DIR[LED_CMD_WRITE_IO_DIR_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 0),  // PREAMBLE+W
  0x02,               // Function :IO_DIR
  1,                  // Size:1
  0xFF,     // DATA0(DIR:ALL OUT)
};

// Write : GPIO(ALL HIGH)
const uint8_t writeGpioAllHigh[LED_CMD_WRITE_GPIO_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 0),  // PREAMBLE+W
  0x01,               // Function :GPIO
  1,                  // Size:1
  0xFF,     // DATA0(DIR:ALL HIGH)
};

// Write : GPIO(ALL LOW)
const uint8_t writeGpioAllLow[LED_CMD_WRITE_GPIO_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 0),  // PREAMBLE+W
  0x01,               // Function :GPIO
  1,                  // Size:1
  0x00,     // DATA0(DIR:ALL LOW)
};

void setSendData(char *pSendData, uint8_t *src, uint8_t sendSize) {
    for(int i=0; i<sendSize; ++i) {
      pSendData[i] = src[i];
    }  
}

void setup() {

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial.println("Start Uart Command Sample(Digital Write)");

  mySerial.begin(115200);

  readCnt = 0;
  sendCnt = 0;
}

/**
 * Serial Message(for debug)
 */
void SerialHexPrint(char n) {
  Serial.print("0x");
  if ( n >= 0 )Serial.print(n < 16 ? "0" : "");
  Serial.print((n & 0x000000FF), HEX);
}

void loop() {

  while (mySerial.available()) {
    // Serial.write(mySerial.read());
    int n = mySerial.read();
    readData[readCnt] = n;
    readCnt++;
  }
  if(readCnt > 0) {
    // print received data
    for(int i=0; i<readCnt; ++i) {
      int n = readData[i];
      SerialHexPrint(n);
      Serial.print(" ");
    }
    Serial.println(" :receive");
    readCnt = 0;  
  }

  switch(uartCmdPhase){
    case UART_CMD_PHASE_SET_DIR_W:
      // set send data
      setSendData(&sendData[0], writeIO_DIR, LED_CMD_WRITE_IO_DIR_SIZE);
      sendCnt = LED_CMD_WRITE_IO_DIR_SIZE;
      uartCmdPhase = UART_CMD_PHASE_OUT_H;
      break;
    case UART_CMD_PHASE_OUT_H:
      // set send data
      setSendData(&sendData[0], writeGpioAllHigh, LED_CMD_WRITE_GPIO_SIZE);
      sendCnt = LED_CMD_WRITE_GPIO_SIZE;
      uartCmdPhase = UART_CMD_PHASE_OUT_L;
      break;
    case UART_CMD_PHASE_OUT_L:
      // set send data
      setSendData(&sendData[0], writeGpioAllLow, LED_CMD_WRITE_GPIO_SIZE);
      sendCnt = LED_CMD_WRITE_GPIO_SIZE;
      uartCmdPhase = UART_CMD_PHASE_OUT_H;
      break;
    default:
      // no impl
      break;
  }

  // send
  if(sendCnt > 0) {
    for(int i=0; i<sendCnt; ++i) {
      mySerial.write(sendData[i]);
    }
    // print sent data
    for(int i=0; i<sendCnt; ++i) {
      int n = sendData[i];
      SerialHexPrint(n);
      Serial.print(" ");
    }
    Serial.println(" :send");
    sendCnt = 0;
  }

  delay(3000);

}

