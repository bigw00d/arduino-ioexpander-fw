// Pin connections:
// A5  ->  SCL
// A4  <-> SDA
// D2       <-  ~START
// D10(RX)  <-  TX
// D11(TX)  ->  RX
// GND      <-> GND

#include <SoftwareSerial.h>
#include <Wire.h>

#define ENABLE_DEBUG_OUT // compile switch to show debug information(UART command)

// main loop
#define DELAY_MS 10

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
#define UART_CMD_W ((UART_CMD_PREAMBLE << 1) + 0)  // uart write command
#define UART_CMD_R ((UART_CMD_PREAMBLE << 1) + 1)  // uart read command

// UART Command Register Address
#define UART_CMD_FUNC_TYPE1        (0x00)
#define UART_CMD_FUNC_TYPE2        (0x10)
#define UART_CMD_FUNC_I2C1_WRITE     (UART_CMD_FUNC_TYPE2 | (UART_FUNC_IO_I2C1_W+1))
#define UART_CMD_FUNC_I2C1_READ     (UART_CMD_FUNC_TYPE2 | (UART_FUNC_IO_I2C1_R+1))
#define UART_CMD_FUNC_ADC_READ     (UART_CMD_FUNC_TYPE2 | (UART_FUNC_ADD1+1))

#define UART_CMD_SIZE_OFST 2
#define I2C_ADR_OFST 3
#define I2C_DATA_OFST 4
#define I2C_READ_LEN_OFST 3

/**
 * ADC Handler
 */
int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

void handleReadADC(uint8_t *readUartData, int len) {
  if ( (len == 3) && (readUartData[UART_CMD_SIZE_OFST] == 2) ) {
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
  if ( len == 3 ) {
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

    // //test
    // if(readSize > 0) {
    //   uint16_t tmp = readData[0];
    //   tmp = tmp << 8;
    //   tmp = tmp | readData[1];
    //   tmp = tmp & 0xFFFC;

    //   float temp = -46.85 + (175.72 * tmp / 65536);

    //   Serial.print("Temperature: ");
    //   Serial.print(temp, 1);
    //   Serial.println(" C");
    // }

  }
  else {
    // no impl
  }
}

void handleReadSequence(uint8_t *readUartData, int len) {
  switch(readUartData[1]){ // DATA0(Function No.)
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

void handleReadI2CAddless(uint8_t *readUartData, int len) {
  if (len == 4) {
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
  if (len == 5) {
    if (readUartData[UART_CMD_SIZE_OFST] == 2) {
      Serial.print("write_i2c_byte :");
      SerialHexPrint(readUartData[I2C_ADR_OFST]);
      Serial.print(", ");
      SerialHexPrint(readUartData[I2C_ADR_OFST]);
      Serial.println("");
      write_i2c_byte(readUartData[I2C_ADR_OFST], readUartData[I2C_DATA_OFST]);
    }
    else {
      // no impl
    }
  }
}

void handleWriteSequence(uint8_t *readUartData, int len) {
  switch(readUartData[1]){ // DATA0(Function No.)
    case UART_CMD_FUNC_I2C1_WRITE:
      handleWriteI2C(readUartData, len);;
      break;
    case UART_CMD_FUNC_I2C1_READ:
      handleReadI2CAddless(readUartData, len);;
      break;
    default:
      // handleHciEvent no impl
      break;
  }
}

void handleUartReceiveData(uint8_t *readUartData, int len) {
  Serial.print("handleUartReceiveData  len:");
  SerialHexPrint(len);
  Serial.println("");
  switch(readUartData[0]){ //header
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

/**
 * Task handling UART Command 
 */
void uartTask() {
  readCnt = 0;
  while (mySerial.available()) {
    int n = mySerial.read();
    readUartData[readCnt] = n;
    readCnt++;
  }
  if(readCnt > 0) {
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
    handleUartReceiveData((uint8_t *)readUartData, readCnt);

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

  // write_i2c_byte(ADDR, CMD_SOFT_RESET);
  // uint8_t readSize = read_i2c_block_data(ADDR, CMD_READ_TEMP_HOLD, 3);
  // if(readSize > 0) {
  //   uint16_t tmp = readData[0];
  //   tmp = tmp << 8;
  //   tmp = tmp | readData[1];
  //   tmp = tmp & 0xFFFC;

  //   float temp = -46.85 + (175.72 * tmp / 65536);

  //   Serial.print("Temperature: ");
  //   Serial.print(temp, 1);
  //   Serial.println(" C");
  // }
  // else {
  //   Serial.println("fail to get temperature");
  // }

  // delay(3000);
}
