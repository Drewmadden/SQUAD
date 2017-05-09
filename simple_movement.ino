#include <Servo.h>
 
 Servo throttle;
 Servo steering;
 
 int pos = 0;
 char usbRead;
 
 void setup(){
   Serial.begin(9600);
   throttle.attach(9);
   steering.attach(10);
   throttle.write(90);
   steering.write(109);
   //delay(5000);
 }
 
 void loop(){
   
   if(Serial.available()){
     usbRead = Serial.read();  
     Serial.println(usbRead);
  
     if(usbRead == 'a'){  //straight and turning extreme right
       throttle.write(80);
       steering.write(40);
       //usbRead = 0; 
     }
     else if(usbRead == 'j'){  //straight and moderate turning right
       throttle.write(81);
       steering.write(60);
       //usbRead = 0; 
     }  
     else if(usbRead == 'k'){  //straight and moderate turning right
       throttle.write(82);
       steering.write(80);
       //usbRead = 0; 
     }  
     else if(usbRead == 'c'){ //not moving and centered
       throttle.write(90);
       steering.write(109);
     }
     else if(usbRead == 'd'){ //straight ahead
      throttle.write(81); //83
      steering.write(109);
     }
     else if(usbRead == 'b'){ //straight and turning left
      throttle.write(80); //82
      steering.write(170);
     }
     else if(usbRead == 'r'){ //straight and turning left
      throttle.write(81); //82
      steering.write(150);
     }
     else if(usbRead == 'q'){ //straight and turning left
      throttle.write(82); //82
      steering.write(130);
     }
     //delay(1000);
   } //end of serial.available
   
   //else{
     //throttle.write(90);
   //}
   
 } //end of loop()
 
 /*
 speed control
   throttle:
     90 = neutral
     80 = forward and slow
     110 = backward and slow
   steering:
     109 = neutral
     80 = slightly right
     130 = slightly left
 */
