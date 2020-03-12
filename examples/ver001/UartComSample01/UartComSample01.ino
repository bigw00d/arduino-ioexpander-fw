// UART Command Sample (I2C RW)

// D10(RX)  <-  TX
// D11(TX)  ->  RX

#include <SoftwareSerial.h>

// SoftwareSerial mySerial(2, 3); // RX, TX = D2, D3
SoftwareSerial mySerial(10, 11); // RX, TX = D10, D11

#define RW_DATA_MAX_SIZE 50
int readCnt;
char readData[RW_DATA_MAX_SIZE];
int sendCnt;
char sendData[RW_DATA_MAX_SIZE];

#define UART_CMD_PREAMBLE 0x5A

#define I2C_SLAVE_ADDR 0x40
#define CMD_READ_TEMP_HOLD 0xe3
#define CMD_SOFT_RESET 0xfe

enum UART_CMD_PHASE {
  UART_CMD_PHASE_SW_RST = 0,    // i2c write: software reset
  UART_CMD_PHASE_RD_TMP,        // i2c write: read temperature
  UART_CMD_PHASE_I2C_RD_READY,  // i2c read ready
  UART_CMD_PHASE_I2C_RD,        // i2c read
  UART_REG_TYPE2_NUM,
};
int uartCmdPhase = UART_CMD_PHASE_SW_RST;

#define LED_CMD_I2C_WRITE_SAMPLE_SIZE 5
#define LED_CMD_I2C_READ_SAMPLE_SIZE 4
#define LED_CMD_I2C_READ_SEQ_SIZE 3

// Write I2C: CMD_SOFT_RESET
const uint8_t i2CWriteSoftReset[LED_CMD_I2C_WRITE_SAMPLE_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 0),  // PREAMBLE+W
  0x12,               // Function :I2C1_WRITE
  2,                  // Size:2
  I2C_SLAVE_ADDR,     // DATA0(I2C Slave Address)
  CMD_SOFT_RESET,     // DATA1(I2C Write Data0)
};

// Write I2C: CMD_READ_TEMP_HOLD
const uint8_t i2CWriteReadTemp[LED_CMD_I2C_WRITE_SAMPLE_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 0),  // PREAMBLE+W
  0x12,               // Function :I2C1_WRITE
  2,                  // Size:2
  I2C_SLAVE_ADDR,     // DATA0(I2C Slave Address)
  CMD_READ_TEMP_HOLD,     // DATA1(I2C Write Data0)
};

// Read I2C(Write Sequence)
const uint8_t i2CReadReady[LED_CMD_I2C_READ_SAMPLE_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 0),  // PREAMBLE+W
  0x13,               // Function :I2C1_READ
  1,                  // Size:1
  I2C_SLAVE_ADDR,     // DATA0(I2C Slave Address)
};

// Read I2C(Read Sequence): 3 byte
const uint8_t i2CReadData[LED_CMD_I2C_READ_SEQ_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 1),  // PREAMBLE+R
  0x13,               // Function :I2C1_READ
  3,                  // Size:3
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

  Serial.println("Start Uart Command Sample");

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

bool waitReadI2CData = false;

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

    if(waitReadI2CData) {
      waitReadI2CData = false;
      int posi = 0;
      if( (readData[posi] == 0x06) || (readData[posi] == 0x15) ) {
        posi++;
      }
      uint16_t tmp = readData[posi];
      tmp = tmp << 8;
      tmp = tmp | readData[posi+1];
      tmp = tmp & 0xFFFC;
    
      float temp = -46.85 + (175.72 * tmp / 65536);
    
      Serial.print("Temperature: ");
      Serial.print(temp, 1);
      Serial.println(" C");
    }
  
  }

  switch(uartCmdPhase){
    case UART_CMD_PHASE_SW_RST:
      // set send data
      setSendData(&sendData[0], i2CWriteSoftReset, LED_CMD_I2C_WRITE_SAMPLE_SIZE);
      sendCnt = LED_CMD_I2C_WRITE_SAMPLE_SIZE;
      uartCmdPhase = UART_CMD_PHASE_RD_TMP;
      break;
    case UART_CMD_PHASE_RD_TMP:
      // set send data
      setSendData(&sendData[0], i2CWriteReadTemp, LED_CMD_I2C_WRITE_SAMPLE_SIZE);
      sendCnt = LED_CMD_I2C_WRITE_SAMPLE_SIZE;
      uartCmdPhase = UART_CMD_PHASE_I2C_RD_READY;
      break;
    case UART_CMD_PHASE_I2C_RD_READY:
      // set send data
      setSendData(&sendData[0], i2CReadReady, LED_CMD_I2C_READ_SAMPLE_SIZE);
      sendCnt = LED_CMD_I2C_READ_SAMPLE_SIZE;
      uartCmdPhase = UART_CMD_PHASE_I2C_RD;
      break;
    case UART_CMD_PHASE_I2C_RD:
      // set send data
      setSendData(&sendData[0], i2CReadData, LED_CMD_I2C_READ_SEQ_SIZE);
      sendCnt = LED_CMD_I2C_READ_SEQ_SIZE;
      uartCmdPhase = UART_CMD_PHASE_SW_RST;
      waitReadI2CData = true;
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

