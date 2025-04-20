#include <EasyTransfer.h>

//create object
EasyTransfer ET; 

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t speed;
  int16_t rpm;
};

//give a name to the group of data
SEND_DATA_STRUCTURE mydata;

void setup(){
  Serial.begin(9600);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ET.begin(details(mydata), &Serial);
  
  pinMode(13, OUTPUT);
  
int pot = analogRead(A0);

}

void loop(){
  //this is how you access the variables. [name of the group].[variable name]
  mydata.speed = pot(5);
  mydata.rpm = random(5);
  //send the data
  ET.sendData();
    
    }
  
  delay(1000);
}
