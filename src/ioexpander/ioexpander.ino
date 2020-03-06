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
 * Serial Message(for debug)
 */
void SerialHexPrint(char n) {
  Serial.print("0x");
  if ( n >= 0 )Serial.print(n < 16 ? "0" : "");
  Serial.print((n & 0x000000FF), HEX);
}

SoftwareSerial mySerial(10, 11); // RX, TX = D10, D11

#define RW_DATA_MAX_SIZE 50
int readCnt;
char readUartData[RW_DATA_MAX_SIZE];
#define RW_DATA_MAX_SIZE 50
int sendCnt;
char sendData[RW_DATA_MAX_SIZE];

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
    sendData[sendCnt++] = 0x06; // send ACK

    #ifdef ENABLE_DEBUG_OUT
    // print received data
    for(int i=0; i<readCnt; ++i) {
      int n = readUartData[i];
      SerialHexPrint(n);
      Serial.print(" ");
    }
    Serial.println(" :receive");
    #endif // ENABLE_DEBUG_OUT

    readCnt = 0;
  }

  // send
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
 * Start Interrupt(~START)
 */
#define START_PIN 0        //interrupt 0 at arduino nano pin D2
volatile int startState;    // current state of the START
void startNow(){           // Interrupt service routine or ISR  
  startState = 1;
}

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
