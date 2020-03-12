// UART Command Sample (Digital Read)

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
  UART_CMD_PHASE_SET_DIR_R = 0,
  UART_CMD_PHASE_READ_IODIR,
  UART_CMD_PHASE_READ_GPIO,
  UART_REG_TYPE2_NUM,
};
int uartCmdPhase = UART_CMD_PHASE_SET_DIR_R;

#define LED_CMD_WRITE_IO_DIR_SIZE 4
#define LED_CMD_READ_IO_DIR_SIZE 3
#define LED_CMD_READ_GPIO_SIZE 3

// Write : IO_DIR(ALL INPUT)
const uint8_t writeIO_DIR[LED_CMD_WRITE_IO_DIR_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 0),  // PREAMBLE+W
  0x02,               // Function :IO_DIR
  1,                  // Size:1
  0x00,     // DATA0(DIR:ALL INPUT)
};

// Read : IO_DIR
const uint8_t readIO_DIR[LED_CMD_READ_IO_DIR_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 1),  // PREAMBLE+R
  0x02,               // Function :IO_DIR
  1,                  // Size:1
};

// Read : GPIO
const uint8_t readGpio[LED_CMD_READ_GPIO_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 1),  // PREAMBLE+R
  0x01,               // Function :GPIO
  1,                  // Size:1
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

  Serial.println("Start Uart Command Sample(Digital Read)");

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

bool waitRdData = false;

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
  
    if(waitRdData) {
      waitRdData = false;
      int posi = 0;
      if( (readData[posi] == 0x06) || (readData[posi] == 0x15) ) {
        posi++;
      }
      Serial.print("Read Data(raw): ");
      SerialHexPrint(readData[posi]);
      Serial.println("");
    }

  }

  switch(uartCmdPhase){
    case UART_CMD_PHASE_SET_DIR_R:
      // set send data
      setSendData(&sendData[0], writeIO_DIR, LED_CMD_WRITE_IO_DIR_SIZE);
      sendCnt = LED_CMD_WRITE_IO_DIR_SIZE;
      uartCmdPhase = UART_CMD_PHASE_READ_IODIR;
      break;
    case UART_CMD_PHASE_READ_IODIR:
      // set send data
      Serial.println("READ_IODIR");
      setSendData(&sendData[0], readIO_DIR, LED_CMD_READ_IO_DIR_SIZE);
      sendCnt = LED_CMD_READ_IO_DIR_SIZE;
      uartCmdPhase = UART_CMD_PHASE_READ_GPIO;
      waitRdData = true;
      break;
    case UART_CMD_PHASE_READ_GPIO:
      // set send data
      Serial.println("READ_GPIO");
      setSendData(&sendData[0], readGpio, LED_CMD_READ_GPIO_SIZE);
      sendCnt = LED_CMD_READ_GPIO_SIZE;
      uartCmdPhase = UART_CMD_PHASE_SET_DIR_R;
      waitRdData = true;
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

