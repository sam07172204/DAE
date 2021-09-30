#include <SoftwareSerial.h>
#include <Crc16.h>
#include <SPI.h>
#include <LoRa.h>
SoftwareSerial Master(5, 6); 
char val; 
int DE_RE=7;
char charBuffer[50];
byte inBuffer[20];
double Total_Energy1;
double Total_Energy0;
unsigned int index = 0;
char read_value[50];
byte read_dae_recive[20];
byte read_dae1_recive[20];
byte read_dae2_recive[20];
String receive_count;
Crc16 crc; 
int flag=0;
String s,meter1,meter2,meter3;
void setup() {
  Serial.begin(9600);   
  Master.begin(2400);
  pinMode(DE_RE,OUTPUT);  
  digitalWrite(DE_RE,LOW); 
  //Serial.println("Master is ready!");
  //while (!Serial);  //等待序列埠起始完畢
  //Serial.println("LoRa Receiver");
  if (!LoRa.begin(434E6)) { //起始 LoRa
    //Serial.println("Starting LoRa failed!");
    while (1);
    }
}

void(*resetFunc)(void) = 0;
//----------------------------------------------------receive & 判斷控制

void RoLa_receive(){
  int packetSize=LoRa.parsePacket(); 
  index = 0;
  if (packetSize) { //若有封包進來
    while (LoRa.available()) { //若接收緩衝器有內容
     read_value[index]=(char)LoRa.read();
     index++;
     delay(5);
  }
  unsigned int send_count = 0;
  if((read_value[0] != '\0') && (read_value[0] != ' '))
    send_count++;
    for(unsigned int i=0;i<50;i++)
      if(read_value[i] == ' '){
          if(((i+1) < 50) && (read_value[i+1] != '\0') && (read_value[i+1] != ' '))
            send_count++;
        }
  String outBuffer[send_count];
  send_count = 0;
  unsigned int read_value_index = 0;
  while(read_value_index < 50){
    if((read_value[read_value_index] != ' ') && (read_value[read_value_index] != '\0')){
      String s = "";
      while((read_value[read_value_index] != ' ') && (read_value[read_value_index] != '\0') && (read_value_index < 50)){
        s += read_value[read_value_index];
        read_value_index++;
      }
      outBuffer[send_count] = s;
      send_count++;
    }
    read_value_index++;
  }
  if(send_count != 0){
    for(int i=0;i<send_count;i++){
        //Serial.print(outBuffer[i]);
        ;
    }
    String receive_count = outBuffer[2];
    //------------------------------------------判斷
    if (outBuffer[0]=="Electronic_classroom"){
      if (outBuffer[1]=="2"){//讀電表
        //Serial.print(outBuffer[0]);
        read_meter();
        delay(1000);
        read_meter1();
        delay(1000);
        read_meter2();
        if((meter1.toInt() > 0) && (meter2.toInt() > 0)&& (meter3.toInt() > 0)){  
          s = "Electronic_classroom meter "+meter1 +" "+ meter2 +" "+meter3+" "+receive_count;//meter1/2是電表值 receive_count為order
          }
        else{
          s = "Electronic_classroom fail "  + receive_count;//meter1/2是電表值 receive_count為order
        }
      }
      if (outBuffer[1]=="0"){//全關
        dae_close();
        close_ac();
        delay(5000);
        open_ac();
        s = "Electronic_classroom close ok "+receive_count;//receive_count為order
      }  
      if (outBuffer[1]=="1"){//全開
        dae_open();
        s = "Electronic_classroom open ok "+receive_count;//receive_count為order
      }
      Serial.println(s);   
    }
    //------------------------------------------
  }
 }
}
//----------------------------------------------------send
void RoLa_send(){
  //Serial.print("Sending device 1 packet: ");
  LoRa.beginPacket();  //封包傳送開始
  LoRa.print(s);  //封包內容
  LoRa.endPacket();  //封包傳送結束
  //Serial.print(s);  //封包內容
  //Serial.println("");
  delay(1000);
}
//----------------------------------------------------
void loop(){
  unsigned long now_time;
  now_time = millis();
  s = "";
  /*
  if(Serial.available()) {
    index =0;
    s = Serial.read();
    delay(5);
  }
  */
  RoLa_receive();
  if (s != ""){
    Serial.println(s);
    RoLa_send();
  }
  
 if (now_time >= 15000){//reset
    Serial.println("reset");
    resetFunc();
  }
}

//-------------------------------------------------------------------------------讀電表1
void read_meter(){
  Master.flush();
  delay(2);
  byte read_dae[]={4,3,0,48,0,2};
  crc.clearCrc();
  unsigned short value = crc.Modbus(read_dae,0,6);
  //Serial.print("Modbus crc = 0x");    
  //Serial.println(value, HEX);
  byte low_value = (value&0xFF);
  byte high_value = ((value&0xFF00)>>8);
  //Serial.println(low_value);
  //Serial.println(high_value);
  digitalWrite(DE_RE,HIGH);
  delay(2);
  for(unsigned int i=0;i<6;i++)
    Master.write(read_dae[i]);
  Master.write(low_value);
  Master.write(high_value);
  digitalWrite(DE_RE,LOW);
  /*
  unsigned long now_time = millis();
  while(!Master.available()){
    if((millis()-now_time) > 2000)
      break;
  }
  */
  while(!Master.available());
  if(Master.available()){
    index = 0;
    while(Master.available()){
      read_dae_recive[index] = Master.read();
      index++;
      delay(5);
    }
    if(index != 0){
      for(unsigned int i=0;i<index;i++){
        //Serial.print(read_dae_recive[i]);
        //Serial.print(", ");
        ;
       }
      //Serial.println("");
      if((read_dae_recive[1] == 3) && (read_dae_recive[2] == 4)){
        Total_Energy0 = ((long)read_dae_recive[5]*16777216 + (long)read_dae_recive[6]*65536 + (long)read_dae_recive[3]*256 + (long)read_dae_recive[4])/100;
        //Serial.print("Total_Energy0:");
        //Serial.println(Total_Energy0);
        meter1 = Total_Energy0;
        for(unsigned int i=0;i<index;i++)
          read_dae_recive[i] = 0;
      }
    }
  }
  digitalWrite(DE_RE,LOW);
}

//-------------------------------------------------------------------------------讀電表2

void read_meter1(){
  Master.flush();
  delay(2);
  byte read_dae1[]={5,3,0,48,0,2};
  crc.clearCrc();
  unsigned short value1 = crc.Modbus(read_dae1,0,6);
  //Serial.print("Modbus crc = 0x");    
  //Serial.println(value1, HEX);
  byte low_value1 = (value1&0xFF);
  byte high_value1 = ((value1&0xFF00)>>8);
  //Serial.println(low_value1);
  //Serial.println(high_value1);
  digitalWrite(DE_RE,HIGH);
  delay(2);
  for(unsigned int i=0;i<6;i++)
    Master.write(read_dae1[i]);
  Master.write(low_value1);
  Master.write(high_value1);
  digitalWrite(DE_RE,LOW);
  /*
  unsigned long now_time = millis();
  while(!Master.available()){
    if((millis()-now_time) > 2000)
      break;
  }
  */
  while(!Master.available());
  if(Master.available()){
    index = 0;
    while(Master.available()){
      read_dae1_recive[index] = Master.read();
      index++;
      delay(5);
    }
    if(index != 0){
      for(unsigned int i=0;i<index;i++){
        //Serial.print(read_dae1_recive[i]);
        //Serial.print(", ");
        ;
      }
      //Serial.println("");
      if((read_dae1_recive[1] == 3) && (read_dae1_recive[2] == 4)){
        double Total_Energy1 = ((long)read_dae1_recive[5]*16777216 + (long)read_dae1_recive[6]*65536 + (long)read_dae1_recive[3]*256 + (long)read_dae1_recive[4])/100;
        //Serial.print("Total_Energy1:");
        //Serial.println(Total_Energy1);
        meter2 = Total_Energy1;
        for(unsigned int i=0;i<index;i++)
          read_dae1_recive[i] = 0;
       }
    }
  }
  digitalWrite(DE_RE,LOW);
}
//--------------------------------------------------------------------------
void read_meter2(){
  Master.flush();
  delay(2);
  byte read_dae2[]={9,3,0,0,0,2};
  crc.clearCrc();
  unsigned short value2 = crc.Modbus(read_dae2,0,6);
  //Serial.print("Modbus crc = 0x");    
  //Serial.println(value1, HEX);
  byte low_value2 = (value2&0xFF);
  byte high_value2 = ((value2&0xFF00)>>8);
  //Serial.println(low_value2);
  //Serial.println(high_value2);
  digitalWrite(DE_RE,HIGH);
  delay(2);
  for(unsigned int i=0;i<6;i++)
    Master.write(read_dae2[i]);
  Master.write(low_value2);
  Master.write(high_value2);
  digitalWrite(DE_RE,LOW);
  /*
  unsigned long now_time = millis();
  while(!Master.available()){
    if((millis()-now_time) > 2000)
      break;
  }
  */
  while(!Master.available());
  if(Master.available()){
    index = 0;
    while(Master.available()){
      read_dae2_recive[index] = Master.read();
      index++;
      delay(5);
    }
    if(index != 0){
      for(unsigned int i=0;i<index;i++){
        //Serial.print(read_dae2_recive[i]);
        //Serial.print(", ");
        ;
      }
      //Serial.println("");
      if((read_dae2_recive[1] == 3) && (read_dae2_recive[2] == 4)){
        double Total_Energy2 = ((long)read_dae2_recive[5]*16777216 + (long)read_dae2_recive[6]*65536 + (long)read_dae2_recive[3]*256 + (long)read_dae2_recive[4])/100;
        //Serial.print("Total_Energy2:");
        //Serial.println(Total_Energy2);
        meter3 = Total_Energy2;
        for(unsigned int i=0;i<index;i++)
          read_dae1_recive[i] = 0;
       }
    }
  }
  digitalWrite(DE_RE,LOW);
}

//-------------------------------------------------------------------------------全關
void dae_close(){
  Master.flush();
  delay(2);
  Master.end();
  delay(500);
  Master.begin(9600);
  int device[] = {1,2,3};
  int device_id[] = {1,2,3,4};
  for(int all_device=0;all_device<3;all_device++){
    for (int all=0;all<4;all++){
      byte d_close[]={device[all_device],5,0,device_id[all],0,0};
      crc.clearCrc();
      unsigned short value = crc.Modbus(d_close,0,6);
      //Serial.print("Modbus crc = 0x");    
      //Serial.println(value, HEX);
      byte low_value = (value&0xFF);
      byte high_value = ((value&0xFF00)>>8);
      //Serial.println(low_value);
      //Serial.println(high_value);
      digitalWrite(DE_RE,HIGH);
      delay(2);
      for(unsigned int i=0;i<6;i++)
        Master.write(d_close[i]);
      Master.write(low_value);
      Master.write(high_value);
      digitalWrite(DE_RE,LOW);
      delay(500);
     // s = "Electronic_classroom closeok "+receive_count;//receive_count為order   
    }
  }
  digitalWrite(DE_RE,LOW);
  Master.flush();
  delay(2);
  Master.end();
  delay(500);
  Master.begin(2400);
}
//----------------------------------------------------------------------全開
void dae_open(){
  Master.flush();
  delay(2);
  Master.end();
  delay(500);
  Master.begin(9600);
  int device[] = {1,2,3};
  int device_id[]={1,2,3,4};
  for(int all_device=0;all_device<3;all_device++){
    for (int all=0;all<4;all++){
      byte d_open[]={device[all_device],5,0,device_id[all],255,0};
      crc.clearCrc();
      unsigned short value = crc.Modbus(d_open,0,6);
      //Serial.print("Modbus crc = 0x");    
      //Serial.println(value, HEX);
      byte low_value = (value&0xFF);
      byte high_value = ((value&0xFF00)>>8);
      //Serial.println(low_value);
      //Serial.println(high_value);
      digitalWrite(DE_RE,HIGH);
      delay(2);
      for(unsigned int i=0;i<6;i++)
        Master.write(d_open[i]);
      Master.write(low_value);
      Master.write(high_value);
      digitalWrite(DE_RE,LOW);
      delay(500);
        //s = "Electronic_classroom openok "+receive_count;//receive_count為order   
     }
  }
  digitalWrite(DE_RE,LOW);
  Master.flush();
  delay(2);
  Master.end();
  delay(500);
  Master.begin(2400);
}
//-------------------------------------------------
void open_ac(){
  
  Master.flush();
  delay(2);
  int device[] = {2,3};
  for(int all=0;all<2;all++){
    byte close_ac[]={9,5,0,device[all],255,0};
    crc.clearCrc();
  
    unsigned short value = crc.Modbus(close_ac,0,6);
    //Serial.print("Modbus crc = 0x");    
    //Serial.println(value, HEX);
    byte low_value = (value&0xFF);
    byte high_value = ((value&0xFF00)>>8);
    //Serial.println(low_value);
    //Serial.println(high_value);
    digitalWrite(DE_RE,HIGH);
    delay(2);
    for(unsigned int i=0;i<6;i++)
     Master.write(close_ac[i]);
    Master.write(low_value);
    Master.write(high_value);
    digitalWrite(DE_RE,LOW);
    delay(500); 
    }
  digitalWrite(DE_RE,LOW);
}

void close_ac(){
  
  Master.flush();
  delay(2);
  int device[] = {2,3};
  for(int all=0;all<2;all++){
    byte close_ac[]={9,5,0,device[all],0,0};
    crc.clearCrc();
  
    unsigned short value = crc.Modbus(close_ac,0,6);
    //Serial.print("Modbus crc = 0x");    
    //Serial.println(value, HEX);
    byte low_value = (value&0xFF);
    byte high_value = ((value&0xFF00)>>8);
    //Serial.println(low_value);
    //Serial.println(high_value);
    digitalWrite(DE_RE,HIGH);
    delay(2);
    for(unsigned int i=0;i<6;i++)
     Master.write(close_ac[i]);
    Master.write(low_value);
    Master.write(high_value);
    digitalWrite(DE_RE,LOW);
    delay(500); 
    }
  digitalWrite(DE_RE,LOW);
}
