#include <SPI.h>
#include <LoRa.h>

byte localAddress = 0xFF;     // address of this device
byte nodemain = 0xFF;  
long lastSendTime = 0;        // last send time
int interval = 50;
bool done = false;


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("LoRa Receiver");
  LoRa.setPins(8, 9, 7);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  if (Serial.available()!= 0) {
    
    String teststr = Serial.readString();
    Serial.print(teststr);
    int index = teststr.indexOf('_');
    int length = teststr.length();
    byte address = teststr.substring(0,index).toInt();
    String message = teststr.substring(index + 1, length);
    Serial.println(address);
    Serial.print(message);
    
//    while(done == false){
    sendmessage(message, localAddress, address);
//      Serial.println("sent");
//      if (Serial.available()!= 0) {
//          done = true;
//          Serial.println("doned");}
//          delay(300);
//    }
    }
  onRecieve(LoRa.parsePacket(), localAddress);
}

void sendmessage(String inboxmessage, byte sender, byte destination){
  
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(sender);
  LoRa.print(inboxmessage);
  LoRa.endPacket();                     // finish packet and send it
}



void onRecieve(int packetSize, byte localAddress){
  if(packetSize == 0) return;
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address

  if (recipient != localAddress) {
    //Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }
  while (LoRa.available()) {
      Serial.print((char)LoRa.read());
  }
  Serial.print("");
  Serial.println(LoRa.packetRssi());
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(250);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);
  }
