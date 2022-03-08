#include <SoftwareSerial.h>
const int rxPin = 2;
const int txPin = 3;
SoftwareSerial mySerial(2, 3);

byte messege[256];
uint8_t index_request = 7;
float parameters[64];

#define VOLTAGE_C_ADDRESS 0x00040002
#define CURRENT_C_ADDRESS 0x00100002
#define FREQUENCY_ADDRESS 0x00320002
#define ACTIVE_POWER_ADDRESS 0x00160002
#define REACTIVE_POWER_ADDRESS 0x001E0002
#define POWER_FACTOR_ADDRESS 0x00260002
#define ENERGY_ADDRESS 0x00300002

struct modbus_transmit
{
  uint8_t address = 0x01;
  uint8_t function = 0x03;

  uint32_t startByte_H = 0xFF000000;
  uint32_t startByte_L = 0x00FF0000;

  uint16_t endByte_H = 0x0000FF00;
  uint16_t endByte_L = 0x000000FF;

  uint8_t crc_L = 0xF0;
  uint8_t crc_H = 0x0F;

} MODBUS_REQ;


void setup()
{

  Serial.begin(57600);
  mySerial.begin(9600);
}


void loop()
{
  uint32_t param[index_request] = { VOLTAGE_C_ADDRESS, CURRENT_C_ADDRESS,
                                    FREQUENCY_ADDRESS, ACTIVE_POWER_ADDRESS,
                                    REACTIVE_POWER_ADDRESS, POWER_FACTOR_ADDRESS,
                                    ENERGY_ADDRESS
                                  };

  String command[index_request] = {"voltage", "current", "frequency", "active power",
                                   "reactive power", "power_factor", "energy"
                                  };

  //multiple request-respond
  for (int i = 0; i < index_request; i++)
  {

    READING_PARAM( param[i] );
    delay(10);

    parameters[i] = get_param();
    Serial.print(command[i]); Serial.print(" :  ");
    Serial.println( parameters[i] );
    
    Serial.println();
    delay(250);
  }
  


  // Jeda looping 8 second
  Serial.println();
  delay(10000);
}

void READING_PARAM(uint32_t PARAM)
{
  // Declare variable Transmit ..
  messege[0] = MODBUS_REQ.address;
  messege[1] = MODBUS_REQ.function;
  messege[2] = (MODBUS_REQ.startByte_H &  PARAM)  >>  24;
  messege[3] = (MODBUS_REQ.startByte_L &  PARAM )  >>  16;
  messege[4] = (MODBUS_REQ.endByte_H  &  PARAM)  >>  8;
  messege[5] = MODBUS_REQ.endByte_L &   PARAM;

  // calculate crc and porting to 8 bit
  uint16_t crc = RTU_Vcrc(messege, 6);
  uint8_t crc_L = crc >> 8;
  uint8_t crc_H = crc;

  messege[6] = crc_L;
  messege[7] = crc_H;

  // Transmit variable messege.
  mySerial.write(messege, 8);
  Serial.print("transmit : .. ");
  //print request
  for (int i = 0; i < 8; i++)
  {
    Serial.print(messege[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}


uint16_t RTU_Vcrc(uint8_t RTU_Buff[], uint16_t RTU_Leng)
{
  uint16_t temp = 0xFFFF, temp2, flag;
  for (uint16_t i = 0; i < RTU_Leng; i++)
  {
    temp = temp ^ RTU_Buff[i];
    for (uint8_t e = 1; e <= 8; e++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag) temp ^= 0xA001;
    }
  }
  // Reverse MSB LSB
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  return temp;
}

void PARSING_PARAM(uint32_t data[1])
{
  // Check uart comunication
  if (mySerial.available() > 0)
  {

    uint8_t u = 0;
    uint32_t recv [128];

    // use delay for catch all data recieved (for stable)
    delay(10);

    // catch data received from uart
    while (mySerial.available() > 0)
    {

      recv[u] = mySerial.read();
      u++;
    }


    // collect value target
    Serial.print("data masuk : ");

    for ( int i  = 0;  i  <  9;  i++)
    {

      Serial.print(recv[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
    recv[3] = recv[3] << 24;
    recv[4] = recv[4] << 16;
    recv[5] = recv[5] << 8;

    data[0] = recv[3]  | recv[4]  | recv[5]  | recv[6];

    //    Serial.print("data[0] : ");
    //    Serial.print(data[0], HEX);
    //    Serial.println();

  }
}

float get_param()
{
  uint32_t get_reg[1];

  PARSING_PARAM(get_reg);

  float val_register = get_reg[0];
  val_register =  val_register  /  1000;
  return val_register;
}
