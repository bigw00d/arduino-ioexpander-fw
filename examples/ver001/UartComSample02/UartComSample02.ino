// UART Command Sample (Analog Read)

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


#define LED_CMD_AD_READ_SEQ_SIZE 3

// Read Analog(Read Sequence): 2 byte
const uint8_t adReadData[LED_CMD_AD_READ_SEQ_SIZE] = {
  ((UART_CMD_PREAMBLE << 1) + 1),  // PREAMBLE+R
  0x11,               // Function :AD
  2,                  // Size:2
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

bool waitAdData = false;

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

    if(waitAdData) {
      waitAdData = false;
      int posi = 0;
      if( (readData[posi] == 0x06) || (readData[posi] == 0x15) ) {
        posi++;
      }
      Serial.print("AD Data(raw): ");
      SerialHexPrint(readData[posi]);
      Serial.print(", ");
      SerialHexPrint(readData[posi+1]);
      Serial.println("");
      uint16_t tmp = ((uint16_t)readData[posi] << 8) | ((uint16_t)readData[posi+1] & 0x00FF);
      Serial.print("AD Data: ");
      //Serial.print((tmp & 0x0000FFFF), HEX);
      //Serial.println("");
      Serial.println(tmp);
    }
  
  }

  // set send data
  setSendData(&sendData[0], adReadData, LED_CMD_AD_READ_SEQ_SIZE);
  sendCnt = LED_CMD_AD_READ_SEQ_SIZE;
  waitAdData = true;
  delay(3000);

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

}

