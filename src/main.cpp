// github link: https://github.com/4-20ma/ModbusMaster
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

/* Modbus stuff */

#define MODBUS_SERIAL_BAUD 9600 // Baud rate for modbus rtu communication

// Initialize the ModbusMaster object as node
ModbusMaster node;
SoftwareSerial modbusSerial(18, 19, false);

void modbusPreTransmission()
{
    delay(500);
    digitalWrite(25, HIGH);
    digitalWrite(26, HIGH);
}

// Pin 4 made low for Modbus receive mode
void modbusPostTransmission()
{

    digitalWrite(25, LOW);
    digitalWrite(26, LOW);
    delay(500);
}
void setup()
{
    // we initialize the built-in hardware serial communication
    // using the Serial.begin() function with two parameters.
    // The first parameter is the desired baud rate (9600 bits per second),
    // and the second parameter is SERIAL_8E1,
    // which specifies the data format (8 data bits, even parity, and 1 stop bit).
    // to set these two parameter, please read your sensor datasheet first
    Serial.begin(9600, SERIAL_8E1);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    pinMode(18, INPUT);
    pinMode(19, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);

    digitalWrite(25, LOW);
    digitalWrite(26, LOW);

    modbusSerial.begin(9600);

    while (!modbusSerial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    // modbus device slave ID 14

    node.begin(0x37, modbusSerial);

    node.preTransmission(modbusPreTransmission);
    node.postTransmission(modbusPostTransmission);
}

void loop()
{

    auto ret = node.readHoldingRegisters(0x0100, 1);
    Serial.print(std::to_string(ret).c_str());
    if (ret == node.ku8MBSuccess)
    {
        for (int j = 0; j < 1; j++)
        {
            Serial.print(std::to_string(node.getResponseBuffer(j)).c_str());
        }
        Serial.println();
    }

    ret = node.readHoldingRegisters(0x0101, 1);
    Serial.print(std::to_string(ret).c_str());
    if (ret == node.ku8MBSuccess)
    {
        for (int j = 0; j < 1; j++)
        {
            Serial.print(std::to_string(node.getResponseBuffer(j)).c_str());
        }
        Serial.println();
    }
    /*
     uint8_t result;
     //for store 32-bit data
     uint16_t data[2];
     int i;
     float reading;
     // this loop is to read voltage, current and power factor register of Energy Meter

       //Modbus function code (0x04) Read Input Registers according to energy meter datasheet
       result = node.readInputRegisters(data_register[i], 1);

         if (result == node.ku8MBSuccess) {
           Serial.println("Success, Received data: ");

           //Retrieve the data from getResponseBuffer(uint8_t u8Index) function.
           //that is return 16-bit data. our energy meter return 32-bit data everytime.
           //that's why, we take the value to a array called data
            data[0]=node.getResponseBuffer(0x00);
            data[1]=node.getResponseBuffer(0x01);

            //read voltage
            if(data_register[i] == 0x0000){
             Serial.print("Volatge: ");
             reading = *((float *)data);
             Serial.print(reading);
             Serial.println(" Volt");
            }
            //read current
            if(data_register[i] == 0x0008){
             Serial.print("Current: ");
             reading = *((float *)data);
             Serial.print(reading);
             Serial.println(" Amps");
            }
            //read Frequency
            if(data_register[i] == 0x0036){
             Serial.print("Frequency: ");
             reading = *((float *)data);
             Serial.print(reading);
             Serial.println(" Hz");
            }
         } else {
           Serial.print("Failed, Response Code: ");
           Serial.print(result, HEX);
           Serial.println("");
           delay(5000);
         }


   //one second delay
   */
    delay(1000);
}