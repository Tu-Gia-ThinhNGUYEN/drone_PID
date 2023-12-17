/*
        Arduino Brushless Motor Control
     by Dejan, https://howtomechatronics.com
*/

#include <Servo.h>

Servo ESC1;     // create servo object to control the ESC
Servo ESC2;     // create servo object to control the ESC
Servo ESC3;     // create servo object to control the ESC
Servo ESC4;     // create servo object to control the ESC

int potValue;  // value from the analog pin
String rectext;
int startstr;
int endstr;
int substrstart;
int substrlength;
int recvalue;

void setup() {
  // Attach the ESC on pin 3
  ESC1.attach(3,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  // Attach the ESC on pin 9
  ESC2.attach(5,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  // Attach the ESC on pin 9
  ESC3.attach(6,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  // Attach the ESC on pin 9
  ESC4.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  Serial.begin(9600);
  ESC1.write(1000);    // Send the signal to the ESC
  ESC2.write(1000);    // Send the signal to the ESC
  ESC3.write(1000);    // Send the signal to the ESC
  ESC4.write(1000);    // Send the signal to the ESC
  Serial.print("Wait for ESC ready");
  delay(2000);
  Serial.print(".");
  delay(2000);
  Serial.print(".");
  delay(2000);
  Serial.println(".");
  delay(2000);
  Serial.println("-----------------Done!--------------- ");
  delay(2000);
}

void loop() {
  if (Serial.available() > 0)
  {
    rectext = Serial.readString();
    Serial.println(rectext);
    startstr = rectext.indexOf("$");
    endstr = rectext.indexOf("%");
    if (startstr >= 0 && endstr > 0 && endstr > startstr)
    {
      substrstart = startstr + 1;
      substrlength = endstr - startstr;
      recvalue = rectext.substring(substrstart, substrlength).toInt();
      rectext = "";
    }
  }
  ESC1.write(recvalue);    // Send the signal to the ESC
//  ESC2.write(recvalue);    // Send the signal to the ESC
//  ESC3.write(recvalue);    // Send the signal to the ESC
//  ESC4.write(recvalue);    // Send the signal to the ESC
  Serial.print(" ESC: ");
  Serial.println(recvalue);
}
