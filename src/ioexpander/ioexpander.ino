// Sequence Ver002

// Pin connections:
// A5  ->  SCL
// A4  <-> SDA
// D10(RX)  <-  TX
// D11(TX)  ->  RX
// GND      <-> GND

#include <SoftwareSerial.h>
#include <Wire.h>

#define ENABLE_DEBUG_OUT // compile switch to show debug information(UART command)

// main loop
#define DELAY_MS 10

#define WAIT_BODY_MS 30

// target device:HTU21D(i2c temperature sensor)
#define ADDR 0x40
#define CMD_READ_TEMP_HOLD 0xe3
#define CMD_READ_HUM_HOLD 0xe5
#define CMD_READ_TEMP_NOHOLD 0xf3
#define CMD_READ_HUM_NOHOLD 0xf5
#define CMD_WRITE_USER_REG 0xe6
#define CMD_READ_USER_REG 0xe7
#define CMD_SOFT_RESET 0xfe

/**
 * I2C
 */
void write_i2c_byte(uint8_t adr, uint8_t val)
{
  Wire.beginTransmission(adr);
  Wire.write(val);
  Wire.endTransmission();  
}

#define READ_MAX_LEN 16
uint8_t readData[READ_MAX_LEN];
uint8_t cmdCnt;

uint8_t read_i2c_block(uint8_t adr, uint8_t len) {
  delay(100);

  cmdCnt = 0;
  Wire.requestFrom(adr, len);
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    readData[cmdCnt] = c;
    cmdCnt++;
  }

  return cmdCnt;  
}

uint8_t read_i2c_block_data(uint8_t adr, uint8_t reg, uint8_t len) {
  write_i2c_byte(adr, reg);
  delay(100);

  cmdCnt = 0;
  Wire.requestFrom(adr, len);
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    readData[cmdCnt] = c;
    cmdCnt++;
  }

  return cmdCnt;  
}


/**
 * Serial Message(for debug)
 */
void SerialHexPrint(char n) {
  Serial.print("0x");
  if ( n >= 0 )Serial.print(n < 16 ? "0" : "");
  Serial.print((n & 0x000000FF), HEX);
}

#define UART_CMD_PREAMBLE 0x5A

SoftwareSerial mySerial(10, 11); // RX, TX = D10, D11

#define RW_DATA_MAX_SIZE 50
int readCnt;
char readUartData[RW_DATA_MAX_SIZE];
#define RW_DATA_MAX_SIZE 50
int sendCnt;
char sendData[RW_DATA_MAX_SIZE];

enum UART_FUNC_TYPE1 {
  UART_FUNC_GPIO = 0,
  UART_FUNC_IO_DIR,
  UART_FUNC_INTEN,
  UART_FUNC_INTF,
  UART_FUNC_DEFAULT,
  UART_FUNC_MODE,
  UART_FUNC_TYPE1_NUM,
};

enum UART_FUNC_TYPE2 {
  UART_FUNC_ADD1 = 0,
  UART_FUNC_IO_I2C1_W,
  UART_FUNC_IO_I2C1_R,
  UART_FUNC_TYPE2_NUM,
};

/**
 * UART Command Register
 */
uint8_t uartCommRegs1[UART_FUNC_TYPE1_NUM] = {
  0x00,               // UART_FUNC_GPIO
  0x00,               // UART_FUNC_IO_DIR
  0x00,               // UART_FUNC_INTEN
  0x00,               // UART_FUNC_INTF
  0x00,               // UART_FUNC_DEFAULT
  0x00,               // UART_FUNC_MODE
};
uint8_t uartCommRegs2[UART_FUNC_TYPE2_NUM] = {
  0x00,               // UART_FUNC_ADD1
  0x00,               // UART_FUNC_IO_I2C1_W
};

// UART Command Header
#define UART_CMD_W (0)  // uart write command
#define UART_CMD_R (1)  // uart read command

// UART Command Register Address
#define UART_CMD_FUNC_TYPE1        (0x00)
#define UART_CMD_FUNC_TYPE2        (0x10)
#define UART_CMD_FUNC_GPIO            (UART_CMD_FUNC_TYPE1 | (UART_FUNC_GPIO+1))
#define UART_CMD_FUNC_IODIR           (UART_CMD_FUNC_TYPE1 | (UART_FUNC_IO_DIR+1))
#define UART_CMD_FUNC_I2C1_WRITE      (UART_CMD_FUNC_TYPE2 | (UART_FUNC_IO_I2C1_W+1))
#define UART_CMD_FUNC_I2C1_READ       (UART_CMD_FUNC_TYPE2 | (UART_FUNC_IO_I2C1_R+1))
#define UART_CMD_FUNC_ADC_READ        (UART_CMD_FUNC_TYPE2 | (UART_FUNC_ADD1+1))

#define DECODE_CMD_FUNCNUM_OFST 1
#define UART_CMD_SIZE_OFST 2
#define GPIO_WRITE_OFST 3
#define I2C_ADR_OFST 3
#define I2C_DATA_OFST 4
#define I2C_READ_LEN_OFST 3

#define DECODE_CMD_HEAD_SIZE 3 //STX+FuncNo+Size

/**
 * GPIO Handler
 */
void handleWriteGpio(uint8_t *readUartData, int len) {
  if (len == (DECODE_CMD_HEAD_SIZE + 1)) { //GPIO(1byte)
    Serial.print("handleWriteGpio :");
    char PtD = ((readUartData[GPIO_WRITE_OFST] << 3) & 0b11111000); //Digital0..4 -> PTD7...D3
    char PtB = ((readUartData[GPIO_WRITE_OFST] >> 5) & 0b00000011); //Digital5,6 -> PTB0, B1
    PtB |= ((readUartData[GPIO_WRITE_OFST] >> 3) & 0b00010000); //Digital7 -> PTB4
    PORTD = (PORTD & ~(0b11111000)) | PtD;
    PORTB = (PORTB & ~(0b00010011)) | PtB;

    Serial.print("PORTD :");
    SerialHexPrint(PORTD);
    Serial.print(", PORTB : ");
    SerialHexPrint(PORTB);
    Serial.println("");
  }
}

void handleWriteIoDir(uint8_t *readUartData, int len) {
  if (len == (DECODE_CMD_HEAD_SIZE + 1)) { //IO DIR(1byte)
    Serial.print("handleWriteIoDir :");
    char DdrD = ((readUartData[GPIO_WRITE_OFST] << 3) & 0b11111000); //Digital0..4 -> PTD7...D3
    char DdrB = ((readUartData[GPIO_WRITE_OFST] >> 5) & 0b00000011); //Digital5,6 -> PTB0, B1
    DdrB |= ((readUartData[GPIO_WRITE_OFST] >> 3) & 0b00010000); //Digital7 -> PTB4
    DDRD = (DDRD & ~(0b11111000)) | DdrD;
    DDRB = (DDRB & ~(0b00010011)) | DdrB;

    Serial.print("DDRD :");
    SerialHexPrint(DDRD);
    Serial.print(", DDRB : ");
    SerialHexPrint(DDRB);
    Serial.println("");
  }
}

void handleReadGpio(uint8_t *readUartData, int len) {
  if (len == DECODE_CMD_HEAD_SIZE) { //header only
    Serial.print("handleReadGpio :");
    char DigitalPt = (PIND >> 3) & 0b00011111;  //PTD7...D3 -> Digital4..0
    DigitalPt |= (PINB << 5) & 0b01100000; //PTB0, B1 -> Digital5,6
    DigitalPt |= (PINB << 3) & 0b10000000; //PTB4 -> Digital7

    //reply
    sendData[sendCnt++] = DigitalPt;
    sendUartData();

    Serial.print("PIND :");
    SerialHexPrint(PIND);
    Serial.print(", PINB : ");
    SerialHexPrint(PINB);
    Serial.println("");
  }
}

void handleReadIoDir(uint8_t *readUartData, int len) {
  if (len == DECODE_CMD_HEAD_SIZE) { //header only
    Serial.print("handleReadIoDir :");
    char DigitalPt = (DDRD >> 3) & 0b00011111;  //PTD7...D3 -> Digital4..0
    DigitalPt |= (DDRB << 5) & 0b01100000; //PTB0, B1 -> Digital5,6
    DigitalPt |= (DDRB << 3) & 0b10000000; //PTB4 -> Digital7

    //reply
    sendData[sendCnt++] = DigitalPt;
    sendUartData();

    Serial.print("DDRD :");
    SerialHexPrint(DDRD);
    Serial.print(", DDRB : ");
    SerialHexPrint(DDRB);
    Serial.println("");
  }
}

/**
 * ADC Handler
 */
int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

void handleReadADC(uint8_t *readUartData, int len) {
  if ( (len == DECODE_CMD_HEAD_SIZE) && (readUartData[UART_CMD_SIZE_OFST] == 2) ) {
    Serial.println("analogRead");
    sensorValue = analogRead(sensorPin);

    //reply
    uint16_t u16SensorVal = (uint16_t)sensorValue;
    sendData[sendCnt++] = (uint8_t)((u16SensorVal >> 8) & 0x00FF); // send D17...D8
    sendData[sendCnt++] = (uint8_t)(u16SensorVal & 0x00FF); // send D7...D0
    sendUartData();

    // //test
    // Serial.println(sensorValue);
  }
  else {
    // no impl
  }
}

/**
 * I2C Handler
 */
uint8_t i2cReadSlaveAddr;

void handleReadI2C(uint8_t *readUartData, int len) {
  if (len == DECODE_CMD_HEAD_SIZE) {
    Serial.print("read_i2c_block :");
    SerialHexPrint(i2cReadSlaveAddr);
    Serial.print(", ");
    SerialHexPrint(readUartData[UART_CMD_SIZE_OFST]);
    Serial.println("");
    uint8_t readSize = read_i2c_block(i2cReadSlaveAddr, readUartData[UART_CMD_SIZE_OFST]);

    if (readSize > 0) {
      //reply
      for (int i=0; i<readUartData[UART_CMD_SIZE_OFST]; ++i) {
        sendData[sendCnt++] = readData[i];
      }
      sendUartData();
    }

    //test
    if(readSize > 0) {
      Serial.print("readData[] :");
      SerialHexPrint(readData[0]);
      Serial.print(", ");
      SerialHexPrint(readData[1]);
      Serial.println("");
      uint16_t tmp = readData[0]; //msb
      tmp = tmp << 8;
      tmp = tmp | readData[1]; //lsb
      tmp = tmp & 0xFFFC;
    
      float temp = -46.85 + (175.72 * tmp / 65536);

      Serial.print("Temperature: ");
      Serial.print(temp, 1);
      Serial.println(" C");
    }

  }
  else {
    // no impl
  }
}

void handleReadSequence(uint8_t *readUartData, int len) {
  uint8_t funcNo = (readUartData[DECODE_CMD_FUNCNUM_OFST] >> 1) & 0x7F;
  Serial.print("handleReadSequence  funcNo:");
  SerialHexPrint(funcNo);
  Serial.println("");
  switch(funcNo){ // Function No.
    case UART_CMD_FUNC_GPIO:
      handleReadGpio(readUartData, len);;
      break;
    case UART_CMD_FUNC_IODIR:
      handleReadIoDir(readUartData, len);;
      break;
    case UART_CMD_FUNC_I2C1_READ:
      handleReadI2C(readUartData, len);;
      break;
    case UART_CMD_FUNC_ADC_READ:
      handleReadADC(readUartData, len);;
      break;
    default:
      // handleHciEvent no impl
      break;
  }
}

void handleWriteReadI2CAddless(uint8_t *readUartData, int len) {
  if (len >= (DECODE_CMD_HEAD_SIZE + 1)) { // I2C_ADR
    if (readUartData[UART_CMD_SIZE_OFST] == 1) {
      i2cReadSlaveAddr = readUartData[I2C_ADR_OFST];
      Serial.print("i2cReadSlaveAddr :");
      Serial.println(readUartData[I2C_ADR_OFST]);
    }
    else {
      // no impl
    }
  }
}

void handleWriteI2C(uint8_t *readUartData, int len) {
  if (len >= (DECODE_CMD_HEAD_SIZE + 2)) { // I2C_ADR + I2C_SDA(1byte only)
    if (readUartData[UART_CMD_SIZE_OFST] == 2) { // I2C_ADR + I2C_SDA(1byte only)
      Serial.print("write_i2c_byte :");
      SerialHexPrint(readUartData[I2C_ADR_OFST]);
      Serial.print(", ");
      SerialHexPrint(readUartData[I2C_DATA_OFST]);
      Serial.println("");
      write_i2c_byte(readUartData[I2C_ADR_OFST], readUartData[I2C_DATA_OFST]);
    }
    else {
      // no impl
    }
  }
}

void handleWriteSequence(uint8_t *readUartData, int len) {
  uint8_t funcNo = (readUartData[DECODE_CMD_FUNCNUM_OFST] >> 1) & 0x7F;
  Serial.print("handleWriteSequence  funcNo:");
  SerialHexPrint(funcNo);
  Serial.println("");
  switch(funcNo){ // Function No.
    case UART_CMD_FUNC_GPIO:
      handleWriteGpio(readUartData, len);;
      break;
    case UART_CMD_FUNC_IODIR:
      handleWriteIoDir(readUartData, len);;
      break;
    case UART_CMD_FUNC_I2C1_WRITE:
      handleWriteI2C(readUartData, len);;
      break;
    case UART_CMD_FUNC_I2C1_READ:
      handleWriteReadI2CAddless(readUartData, len);;
      break;
    default:
      // handleHciEvent no impl
      break;
  }
}

int decodeUartReceiveData(uint8_t *readUartData, int len) {
  int decodedReadCnt = 0;

  // seek start code and remove DLE
  for(int i=0; i<len; i++) {
    if( readUartData[i] == 0x02 ) { // STX
      decodedReadCnt = (len - i);
      break;
    }
  }

  Serial.print("decodeUartReceiveData  decodedReadCnt:");
  SerialHexPrint(decodedReadCnt);
  Serial.println("");

  return decodedReadCnt;
}

void handleUartReceiveData(uint8_t *readUartData, int len) {
  Serial.print("handleUartReceiveData  len:");
  SerialHexPrint(len);
  Serial.println("");
  uint8_t typeRW = readUartData[DECODE_CMD_FUNCNUM_OFST] & 0x01;
  switch(typeRW){ 
    case UART_CMD_W:
      handleWriteSequence(readUartData, len);;
      break;
    case UART_CMD_R:
      handleReadSequence(readUartData, len);;
      break;
    default:
      // handleHciEvent no impl
      break;
  }
}

void sendUartData() {
  if(sendCnt > 0) {
      for(int i=0; i<sendCnt; ++i) {
        mySerial.write(sendData[i]);
      }
      #ifdef ENABLE_DEBUG_OUT
      // print received data
      for(int i=0; i<sendCnt; ++i) {
        int n = sendData[i];
        SerialHexPrint(n);
        Serial.print(" ");
      }
      Serial.println(" :send");
      #endif
      sendCnt = 0;
  }
}

int readSTX(char *readData) {
  int cnt = 0;
  while (mySerial.available()) {
    int n = mySerial.read();
    if (n == 0x02) { //STX;
      readData[cnt] = n;
      cnt++;
      break;
    }
  }
  return cnt;
}

int readHeader(char *readData) {
  int cnt = 0;
  while (mySerial.available()) {
    int n = mySerial.read();
    readData[cnt] = n;
    cnt++;
  }
  return cnt;
}

int readBody(char *readData) {
  int cnt = 0;
  while (mySerial.available()) {
    int n = mySerial.read();
    readData[cnt] = n;
    cnt++;
  }
  return cnt;
}

/**
 * Task handling UART Command 
 */
void uartTask() {
  readCnt = readSTX(&readUartData[0]);
  if (readCnt > 0) {
    readCnt += readHeader(&readUartData[readCnt]);
  }
  if (readCnt == DECODE_CMD_HEAD_SIZE) {
    //reply
    sendData[sendCnt++] = 0x06; // send ACK
    sendUartData();
  }

  if(readCnt > 0) {

    delay(WAIT_BODY_MS);
    readCnt += readBody(&readUartData[readCnt]);
    // while (mySerial.available()) {
    //   int n = mySerial.read();
    //   readUartData[readCnt] = n;
    //   readCnt++;
    // }

    #ifdef ENABLE_DEBUG_OUT
    // print received data
    for(int i=0; i<readCnt; ++i) {
      int n = readUartData[i];
      SerialHexPrint(n);
      Serial.print(" ");
    }
    Serial.println(" :receive");
    #endif // ENABLE_DEBUG_OUT

    //reply
    sendData[sendCnt++] = 0x06; // send ACK
    sendUartData();

    //decode received data
    readCnt = decodeUartReceiveData((uint8_t *)readUartData, readCnt);
    // print decorded data
    for(int i=0; i<readCnt; ++i) {
      int n = readUartData[i];
      SerialHexPrint(n);
      Serial.print(" ");
    }
    Serial.println(" :decorded");
    if (readCnt > 0) {
      handleUartReceiveData((uint8_t *)readUartData, readCnt);
    }

    readCnt = 0;

  }

}


/**
 * Start Interrupt(~START)
 */
#define START_PIN 0        //interrupt 0 at arduino nano pin D2
volatile int startState;    // current state of the START
void startNow(){           // Interrupt service routine or ISR  
  startState = 1;
}


void setup()
{
  Serial.begin(115200); 
  Serial.println("start");

  // Start State Detection
  startState = 0;
  pinMode(START_PIN, INPUT);                       // define interrupt pin D2 as input to detect ~CHG
  attachInterrupt(START_PIN,startNow, FALLING);   // Attach interrupt at pin D2  (int 0 is at pin D2  for nano)

  // I2C
  Wire.begin();
  Wire.setClock(100000L); //100kHz up to 400kHz

  // UART(IO Ex. Command)
  mySerial.begin(115200);

}

void loop()
{
  uartTask();
  delay(DELAY_MS);
}
