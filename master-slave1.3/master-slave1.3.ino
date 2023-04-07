// MakeItReal 2023 Nataniel Slowikowski

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <LoRa.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


byte localAddress = 0xBB;   // address of this device
byte baza = 0xFF;    
byte node1 = 0xCC;   // 0xCC or 0xDD if 0xCC ismain
byte node2 = 0xDD;   // 0xBB or 0xDD if 0xBB ismain

String inbox= "";
String incoming;
String frombaza;
String fromnode1;
String fromnode2;
String inboxmessage = "";


long lastSendTime = 0;        // last send time
int interval = 900;          // interval between sends

bool userconnected = false;
byte useraddress;

bool ismain = false;
bool sleeping = false;


const int chipSelect = 13;
int powerLEDpin =0;

bool liftoff = false;
float previouspressure = 1000;
float currentpressure;

void setup() {

  SPI1.setRX(12); //16 12
  SPI1.setTX(15); //20 15
  SPI1.setSCK(14); //19 14
  SPI1.setCS(13); //17 13 
   
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(powerLEDpin, OUTPUT);
  
  Serial.begin(115200);

  
  

////////////BMP INITIALIZATION////////////////
 
  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  bmp_temp->printSensorDetails();


//////////LoRa INITIALIZATION////////////////

  
  LoRa.setPins(8, 9, 7);
  LoRa.setSignalBandwidth(250E3);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }


///////////SD INITIZALIZATION////////////////

    if (!SD.begin(chipSelect, SPI1)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  digitalWrite(LED_BUILTIN, HIGH);   
  delay(1000);                       // starting successful led on
  digitalWrite(LED_BUILTIN, LOW);


/////////WAITING FOR LAUNCH/////////////////
  

  while (liftoff == false){  //when the pressure changes by a lot the code starts, for testing reasons manual start is also possible

      sensors_event_t pressure_event;
      bmp_pressure->getEvent(&pressure_event);  
      currentpressure = pressure_event.pressure;
      
      if(currentpressure - previouspressure >= 10){
        Serial.println("Big pressure change, starting");
      }
      else{
        previouspressure = currentpressure;
        }
    
      Serial.print(F("Pressure = "));
      Serial.print(pressure_event.pressure);
      Serial.println(" hPa");

      
      
      Serial.println("Enter data:");
      if (Serial.available() != 0) {     //wait for data available
        String teststr = Serial.readString();  //read until timeout
        teststr.trim();                        // remove any \r \n whitespace at the end of the String
        if (teststr == "start") {
          Serial.println("Starting");
          liftoff = true;
        } else {
          Serial.println("type 'start' to start");
        }
      }
  }  
}

void loop() {
  
///// COLECTING TELEMETRIC DATA SAVING THEM TO SD AND A DATA PACKET /////
  if (millis() - lastSendTime > interval) {
    
      sensors_event_t temp_event, pressure_event;
      bmp_temp->getEvent(&temp_event);
      bmp_pressure->getEvent(&pressure_event);
      
      Serial.print(F("Temperature = "));
      Serial.print(temp_event.temperature);
      Serial.println(" *C");
    
      Serial.print(F("Pressure = "));
      Serial.print(pressure_event.pressure);
      Serial.println(" hPa");
  
      String dataString = "";
      dataString += String(pressure_event.pressure);
      dataString += String(",");
      dataString += String(temp_event.temperature);
      dataString += String(",");
      if (inbox != ""){
        inboxmessage = "";
        inboxmessage += dataString;
        inboxmessage += String("");
        inboxmessage += fromnode1;
        inboxmessage += String("");
        inboxmessage += fromnode2;
        
        Serial.println(inboxmessage);
        inbox = "";
        frombaza = "";
      }


///SENDING NEEDED MESSAGES///
      
      if(sleeping == false){
        if(ismain == true){
          sendmessage(inboxmessage, localAddress, baza);
        }  
        sendmessage(dataString, localAddress, node1);
        sendmessage(dataString, localAddress, node2);
        if(userconnected == true){
          sendmessage(inboxmessage, localAddress, useraddress);
        }
      }
      
///SAVING DATA TO SD///

      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(inboxmessage);
        dataFile.close();
        // print to the serial port too:
        Serial.println("Saved to datalog.txt ");
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening datalog.txt");
      }
      Serial.println();
      lastSendTime = millis();            
    }

  // parse for a packet, and call onReceive with the result:
  receive(LoRa.parsePacket());
  digitalWrite(powerLEDpin, HIGH);
}

void sendmessage(String inboxmessage, byte sender, byte destination){
  
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(sender);             // add sender address
  LoRa.print(inboxmessage);
  LoRa.endPacket();                     // finish packet and send it
}

void receive(int packetSize){
  
  if (packetSize == 0) return;
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address

  if (recipient != localAddress) {
    //Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }

 
  // received a packet
  Serial.print("Received packet: ");
  String LoRaData = LoRa.readString();
  Serial.println(LoRaData);

  while (LoRa.available()) {
    //Serial.print((char)LoRa.read());
    incoming += (char)LoRa.read();
  }

  inbox += "Message from: ";
  definesender(sender, LoRaData);
  incoming = "";

}
void definesender(byte sender, String LoRaData){
  if(sender == 255){ //0xFF
      //sender = baza
      frombaza = LoRaData;
      if(frombaza == "main"){
        ismain = true;
      }
      if(frombaza == "nmain"){
          ismain = false;
        }
      }
      
   if(sender == 204) //that's node1 with adress 0xCC
   {
    fromnode1 = LoRaData;
   }
   if(sender == 187) { //0xBB
    fromnode2 = LoRaData;
   }
   if(sender == 221) { //0xDD
    if(localAddress == 0xCC){
    fromnode1 = LoRaData;}
    if(localAddress == 0xBB){
    fromnode2 = LoRaData;}   
   }
   
   else{
      frombaza = "no new messages";
      Serial.println("Recieved a message from a unknown adress: ");
      Serial.println(sender);
      int index = LoRaData.indexOf(' ');
      int length = LoRaData.length();
      String message = LoRaData.substring(0,index);
      byte address = LoRaData.substring(index + 1, length).toInt();
      if(message == "connect"){
        userconnected = true;
        useraddress = address;
      }
      if(message == "disconnect"){
        userconnected = false;
      }
      if(message == "main"){
        ismain = true;
      }
      if(message == "nmain"){
          ismain = false;
        }
      if(message == "sleep"){
        sleeping = true;
      }
      if(message == "wakeup"){
        sleeping = false;}
      }
      
      
}
