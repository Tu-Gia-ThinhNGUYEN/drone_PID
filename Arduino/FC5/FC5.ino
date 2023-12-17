#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#define MinPWM 1100
#define MaxPWM 1400
#define runEvery(t) for (static uint16_t _lasttime;\
                         (uint16_t)((uint16_t)millis()-_lasttime) >= (t);\
                         _lasttime += (t))

Servo ESC1;     // create servo object to control the ESC
Servo ESC2;     // create servo object to control the ESC
Servo ESC3;     // create servo object to control the ESC
Servo ESC4;     // create servo object to control the ESC
//PID
static float Kp_Pitch = 4.2; //1
static float Ki_Pitch = 0.002;
static float Kd_Pitch = 0.7; //0.0001
float TempKp;
float TempKi;
float TempKd;

static int pulse_ESC1;
static int pulse_ESC2;
static int pulse_ESC3;
static int pulse_ESC4;

int test = 1200;

float throttle = 1200.0;
float TempThrottle;
float PitchSP = -3.0;
float PitchPV;
float t_Pitch;
float E_Pitch, E1_Pitch, E2_Pitch, alpha_Pitch, gamma_Pitch, beta_Pitch;
float Output_Pitch = 0.0;
float LastOutput_Pitch = 0.0;
int flag = 0;
float Ts = 0.01; //thoi gian lay mau 1.5s
float timerloop;
//End of PID
//Send Serial Data
int potValue;  // value from the analog pin
String rectext;
int startstr;
int endstr;
int substrstart;
int substrlength;
int recvalue;

MPU6050 mpu6050(Wire, 0.001, 0.999);
//MPU6050 mpu6050(Wire);
void mpuReading()
{
  Serial.println("OK1");
  //mpu6050.update();
  Serial.println("OK2");
  //    Serial.print("angleX : ");
  //    Serial.print(mpu6050.getAngleX());
  //    Serial.print("\tangleY : ");
  //    Serial.print(mpu6050.getAngleY());
  //    Serial.print("\tangleZ : ");
  //    Serial.println(mpu6050.getAngleZ());

  Serial.println("OK3");
  //            Serial.println("@");
  //            Serial.print("#");
  //            Serial.print(ypr[2] * 180/M_PI);
  //            Serial.println("$");
  //            Serial.print("%");
  //            Serial.print(ypr[1] * 180/M_PI);
  //            Serial.println("^");

}
void PID()
{

}

void setup() {
  Serial.begin(115200);
  // Attach the ESC on pin 3
  ESC1.attach(3, MinPWM, MaxPWM); // (pin, min pulse width, max pulse width in microseconds)
  // Attach the ESC on pin 9
  ESC2.attach(5, MinPWM, MaxPWM); // (pin, min pulse width, max pulse width in microseconds)
  // Attach the ESC on pin 9
  ESC3.attach(6, MinPWM, MaxPWM); // (pin, min pulse width, max pulse width in microseconds)
  // Attach the ESC on pin 9
  ESC4.attach(9, MinPWM, MaxPWM); // (pin, min pulse width, max pulse width in microseconds)
  ESC1.writeMicroseconds(1000);    // Send the signal to the ESC
  ESC2.writeMicroseconds(1000);    // Send the signal to the ESC
  ESC3.writeMicroseconds(1000);    // Send the signal to the ESC
  ESC4.writeMicroseconds(1000);    // Send the signal to the ESC
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

  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(-3.13, 0.87, -2.51);
  //mpu6050.calcGyroOffsets(-1.30, -0.68, -2.55);
  Serial.print("Wait for IMU ready");
  delay(1000);
  Serial.print(".");
  delay(1000);
  Serial.print(".");
  delay(1000);
  Serial.println(".");
  delay(1000);
  Serial.println("-----------------Done!--------------- ");
  delay(1000);

  // *****************************************************************************************Sampling
  E_Pitch = 0; E1_Pitch = 0; E2_Pitch = 0;
}

void loop() {
  runEvery(10) {
    static uint32_t lastTime = micros();

    //    if (Serial.available() > 0)
    //    {
    //      rectext = Serial.readString();
    //      Serial.println(rectext);
    //      startstr = rectext.indexOf("q");
    //      endstr = rectext.indexOf("w");
    //      if (startstr >= 0 && endstr > 0 && endstr > startstr)
    //      {
    //        substrstart = startstr + 1;
    //        substrlength = endstr - startstr;
    //        recvalue = rectext.substring(substrstart, substrlength).toFloat();
    //        rectext = "";
    //        TempThrottle = recvalue;
    //      }
    //      if (TempThrottle != throttle)
    //      {
    //        throttle = TempThrottle;
    //      }
    //
    //      startstr = rectext.indexOf("k");
    //      endstr = rectext.indexOf("p");
    //      if (startstr >= 0 && endstr > 0 && endstr > startstr)
    //      {
    //        substrstart = startstr + 1;
    //        substrlength = endstr - startstr;
    //        recvalue = rectext.substring(substrstart, substrlength).toFloat();
    //        rectext = "";
    //        TempKp = recvalue;
    //      }
    //      if (TempKp != Kp_Pitch)
    //      {
    //        Kp_Pitch = TempKp / 1000.0;
    //      }
    //      startstr = rectext.indexOf("k");
    //      endstr = rectext.indexOf("i");
    //      if (startstr >= 0 && endstr > 0 && endstr > startstr)
    //      {
    //        substrstart = startstr + 1;
    //        substrlength = endstr - startstr;
    //        recvalue = rectext.substring(substrstart, substrlength).toFloat();
    //        rectext = "";
    //        TempKi = recvalue;
    //      }
    //      if (TempKi != Ki_Pitch)
    //      {
    //        Ki_Pitch = TempKi / 1000.0;
    //      }
    //      startstr = rectext.indexOf("k");
    //      endstr = rectext.indexOf("d");
    //      if (startstr >= 0 && endstr > 0 && endstr > startstr)
    //      {
    //        substrstart = startstr + 1;
    //        substrlength = endstr - startstr;
    //        recvalue = rectext.substring(substrstart, substrlength).toFloat();
    //        rectext = "";
    //        TempKd = recvalue;
    //      }
    //      if (TempKd != Kd_Pitch)
    //      {
    //        Kd_Pitch = TempKd / 1000.0;
    //      }
    //    }
    //    Serial.println("*************************************************");
    mpu6050.update();
    //    Serial.println("*************************************************OK");
    PitchPV = mpu6050.getAngleY();

    E_Pitch = PitchSP - PitchPV;
    alpha_Pitch = 2 * Ts * Kp_Pitch + Ki_Pitch * Ts * Ts + 2 * Kd_Pitch;
    beta_Pitch = Ts * Ts * Ki_Pitch - 4 * Kd_Pitch - 2 * Ts * Kp_Pitch;
    gamma_Pitch = 2 * Kd_Pitch;
    Output_Pitch = (alpha_Pitch * E_Pitch + beta_Pitch * E1_Pitch + gamma_Pitch * E2_Pitch + 2 * Ts * LastOutput_Pitch) / (2 * Ts);

    LastOutput_Pitch = Output_Pitch;
    E2_Pitch = E1_Pitch;
    E1_Pitch = E_Pitch;

    pulse_ESC1 = (throttle + Output_Pitch);
    pulse_ESC2 = (throttle - Output_Pitch);
    pulse_ESC3 = (throttle - Output_Pitch);
    pulse_ESC4 = (throttle + Output_Pitch);
    if (pulse_ESC1 > MaxPWM)
      pulse_ESC1 = MaxPWM;
    else if (pulse_ESC1 <= MinPWM)
      pulse_ESC1 = MinPWM;

    if (pulse_ESC2 > MaxPWM)
      pulse_ESC2 = MaxPWM;
    else if (pulse_ESC2 <= MinPWM)
      pulse_ESC2 = MinPWM;

    if (pulse_ESC3 > MaxPWM)
      pulse_ESC3 = MaxPWM;
    else if (pulse_ESC3 <= MinPWM)
      pulse_ESC3 = MinPWM;

    if (pulse_ESC4 > MaxPWM)
      pulse_ESC4 = MaxPWM;
    else if (pulse_ESC4 <= MinPWM)
      pulse_ESC4 = MinPWM;

//    Serial.print("E_Pitch= "); Serial.print(E_Pitch); Serial.print(" ; ");
    //    Serial.print("Throttle= "); Serial.print(throttle); Serial.print(" ; ");
    //    Serial.print("Kp= "); Serial.print(Kp_Pitch, 4); Serial.print(" ; ");
    //    Serial.print("Ki= "); Serial.print(Ki_Pitch, 4); Serial.print(" ; ");
    //    Serial.print("Kd= "); Serial.print(Kd_Pitch, 4); Serial.print(" ; ");
    Serial.print("PitchPV:"); Serial.print(PitchPV+3.0); Serial.print(",");
    Serial.print("ESC1= "); Serial.print(pulse_ESC1); Serial.print(" ; ");
    Serial.print("ESC2= "); Serial.print(pulse_ESC2); Serial.print(" ; ");
    Serial.print("ESC3= "); Serial.print(pulse_ESC3); Serial.print(" ; ");
    Serial.print("ESC4= "); Serial.print(pulse_ESC4); Serial.println(" ; ");
    ESC1.writeMicroseconds(pulse_ESC1);    // Send the signal to the ESC
    ESC2.writeMicroseconds(pulse_ESC2);    // Send the signal to the ESC
    ESC3.writeMicroseconds(pulse_ESC3);    // Send the signal to the ESC
    ESC4.writeMicroseconds(pulse_ESC4);    // Send the signal to the ESC
    // UART Comunication
    //    Serial.print("!");
    //    Serial.print(mpu6050.getAngleX());
    //    Serial.println("@");
    //    Serial.print("#");
    //    Serial.print(mpu6050.getAngleY());
    //    Serial.println("$");
    //    Serial.print("%");
    //    Serial.print(mpu6050.getAngleZ());
    //    Serial.println("^");
//    Serial.print("Ts= "); Serial.print(micros() - lastTime); Serial.println(" ; ");
    lastTime = micros();
  }
}
