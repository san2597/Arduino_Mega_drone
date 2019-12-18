//////////////////////////////////////////
//Flight Controller Program
//////////////////////////////////////////


//Wiring connections and colors
//L3G4200D Gyro Sensor
//Red = VCC     Black = Ground      White = Clock line(SCL)       Gray = Data line(SDA)


#include<Wire.h> //Include Wire library to communicate with gyro L3G4200D

//////////////////////////////////////////
//PID gain settings
//////////////////////////////////////////

//Roll and Pitch gains will be same

float pid_p_gain_roll = 0.0; //Gain for roll P-controller
float pid_i_gain_roll = 0.00; //Gain for roll I-controller
float pid_d_gain_roll = 6; //Gain for roll D-controller (9)
int pid_maxroll = 400;  //Maximum output of PID controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll; //Gain for pitch P-controller
float pid_i_gain_pitch = pid_i_gain_roll; //Gain for pitch I-controller
float pid_d_gain_pitch = pid_d_gain_roll; //Gain for pitch D-controller
int pid_maxpitch = pid_maxroll;  //Maximum output of PID controller (+/-)

float pid_p_gain_yaw = 3.0; //Gain for yaw P-controller
float pid_i_gain_yaw = 0.02; //Gain for yaw I-controller
float pid_d_gain_yaw = 0; //Gain for yaw D-controller
int pid_maxyaw = 400;  //Maximum output of PID controller (+/-)

//////////////////////////////////////////
//Declaring variables
//////////////////////////////////////////
byte channelcount_1, channelcount_2, channelcount_3, channelcount_4, lowByte, highByte;
double gyro_roll, gyro_pitch, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int receiverinput_channel_1, receiverinput_channel_2, receiverinput_channel_3, receiverinput_channel_4, start, cal_int;
int throttle, battery_voltage, esc1, esc2, esc3, esc4, led_count;
unsigned long channel_timer_1, channel_timer_2, channel_timer_3, channel_timer_4, current_time, esc_looptimer;
unsigned long loop_timer, timer_1, timer_2, timer_3, timer_4;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

//////////////////////////////////////////
//SETUP routine
//////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //Start I2C communication as master

  DDRC |= B11110000; //Setting digital pins 30,31,32,33 as output
  DDRB |= B10000000; //Setting LED Pin 13 AND 10 as output

  //Led blinks to show its on
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);

  //Power on gyro
  Wire.beginTransmission(105);
  Wire.write(0x20);
  Wire.write(0x0F);
  Wire.endTransmission();

  //Block data update for gyro (ensures data update doesn't happen during data download)
  Wire.beginTransmission(105);
  Wire.write(0x23);
  Wire.write(0x90);
  Wire.endTransmission();

  delay(250); //Gyro Initialization time

  for (cal_int = 0; cal_int < 2000; cal_int++) {
    if (cal_int % 15 == 0)
      digitalWrite(13, !(digitalRead(13)));
    gyro_signal();
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;

    //to prevent esc beeping
    PORTC |= B11110000;
    delayMicroseconds(1000); // 1000us pulse for esc
    PORTC &= B00001111;
    delay(3);
  }

  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
  gyro_yaw_cal /= 2000;

  //Enabling Pin Change Interrupts
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0); //Channel 3 PIN 52
  PCMSK0 |= (1 << PCINT1); //Channel 4 PIN 53
  PCMSK0 |= (1 << PCINT2); //Channel 2 PIN 51
  PCMSK0 |= (1 << PCINT3); //Channel 1 PIN 50

  //Wait till receiver is connected

  while (receiverinput_channel_3 < 990 || receiverinput_channel_3 > 1020) {
    start++;
    PORTC |= B11110000;
    delayMicroseconds(1000); // 1000us pulse for esc
    PORTC &= B00001111;
    delay(3); //Wait 3 ms for next loop
    if (start == 125) { // every 125 loops i.e. 500ms
      digitalWrite(13, !digitalRead(13)); //Change LED status
      start = 0; //Loop again
    }
  }
  start = 0;


  //Battery voltage calculation
  //55V equals ~5V @Analog 0
  //55V equals 1023 analogRead(0)
  //5500/1023 = 5.3763
  battery_voltage = analogRead(0) * 5.3763;
  Serial.begin(9600);
  //Initial Calculations done
  digitalWrite(13, LOW); //Turn off LED pin 13
}

//////////////////////////////////////////
//Loop routine
//////////////////////////////////////////

void loop() {

  //Get values from gyro

  gyro_signal();
  print_output();
  //Gyro pid inputs
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 57.14286) * 0.2);
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 57.14286) * 0.2);
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 57.14286) * 0.2);

  //For starting the motors: throttle low and yaw left
  if (receiverinput_channel_3 < 1050 && receiverinput_channel_4 < 1050)start = 1;
  //When yaw stick is back in center, allow start of motors;
  if (start == 1 && receiverinput_channel_3 < 1050 && receiverinput_channel_4 > 1450)
  {
    start = 2;
    //PID controllers are reset
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  //Stopping the motors : throttle down and yaw right
  if (start == 2 && receiverinput_channel_3 < 1050 && receiverinput_channel_4 > 1950)start = 0;

  //PID roll setpoint
  //dividing by 3 gives a max roll rate of 164 dps: (500-8)/3=164
  pid_roll_setpoint = 0;
  //For higher accuracy of PID roll setpoint
  if (receiverinput_channel_1 > 1508)pid_roll_setpoint = (receiverinput_channel_1 - 1508) / 3.0;
  else if (receiverinput_channel_1 < 1492)pid_roll_setpoint = (receiverinput_channel_1 - 1492) / 3.0;

  //PID pitch setpoint
  //dividing by 3 gives a max roll rate of 164 dps: (500-8)/3=164
  pid_pitch_setpoint = 0;
  //For higher accuracy of PID roll setpoint
  if (receiverinput_channel_2 > 1508)pid_pitch_setpoint = (receiverinput_channel_2 - 1508) / 3.0;
  else if (receiverinput_channel_2 < 1492)pid_pitch_setpoint = (receiverinput_channel_2 - 1492) / 3.0;

  //PID yaw setpoint
  //dividing by 3 gives a max roll rate of 164 dps: (500-8)/3=164
  pid_yaw_setpoint = 0;
  //For higher accuracy of PID roll setpoint
  //Don't yaw when motors are turned off
  if (receiverinput_channel_3 > 1050) {
    if (receiverinput_channel_4 > 1508)pid_yaw_setpoint = (receiverinput_channel_4 - 1508) / 3.0;
    else if (receiverinput_channel_4 < 1492)pid_yaw_setpoint = (receiverinput_channel_1 - 1492) / 3.0;
  }

  //PID inputs are known. Time to calculate PID output
  pid_calculate();

  //Battery voltage is needed for compensation
  //A complementary filter is used to reduce noise with 10% acceptance
  //towards continuous battery voltage measurement
  //0.08*5.3763=0.430104
  battery_voltage = battery_voltage * 0.92 + (0.08 * analogRead(0) * 5.3763);

  //Low battery LED alert
  if (battery_voltage < 1000)
  {
    if (battery_voltage > 700) {
      led_count++;
      
      if(led_count%4==0)
      digitalWrite(13, !digitalRead(13));
    }
    
    else
      digitalWrite(13, HIGH);
  }
  
  else {
    digitalWrite(13, LOW);
  }

  //Critically low battery : automated descent
  /*if (battery_voltage < 650)
  {
    automatic_descent();
  }*/


  throttle = receiverinput_channel_3; //Throttle signal as base signal for all motors

  //If start is 2, take throttle and pid outputs and set esc pulses
  if (start == 2) {
    if (throttle > 1800) throttle = 1800;
    esc1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;  //ESC 1 - Front right (CCW)
    esc2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;  //ESC 2 - Rear right (CW)
    esc3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;  //ESC 3 - Rear left (CCW)
    esc4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;  //ESC 4 - Front left (CW)

    //Adjusting for voltage drop
    if (battery_voltage < 1240 && battery_voltage > 800)  //Checking if battery is connected
    {
      esc1 += esc1 * ((1240 - battery_voltage) / (float)3500);
      esc2 += esc2 * ((1240 - battery_voltage) / (float)3500);
      esc3 += esc3 * ((1240 - battery_voltage) / (float)3500);
      esc4 += esc4 * ((1240 - battery_voltage) / (float)3500);
    }

    if (esc1 < 1000) esc1 = 1000;
    if (esc2 < 1000) esc2 = 1000;
    if (esc3 < 1000) esc3 = 1000;
    if (esc4 < 1000) esc4 = 1000;

    if (esc1 > 2000) esc1 = 2000;
    if (esc2 > 2000) esc2 = 2000;
    if (esc3 > 2000) esc3 = 2000;
    if (esc4 > 2000) esc4 = 2000;
  }

  //If start is not 2, give 1000us pulse to esc's
  else {
    esc1 = 1000;
    esc2 = 1000;
    esc3 = 1000;
    esc4 = 1000;
  }


  //ESC Refresh rate is set to 250Hz,so esc needs pulse every 4ms
  while (micros() - loop_timer < 4000);
  loop_timer = micros();

  PORTC |= B11110000;
  channel_timer_1 = esc1 + loop_timer; //Time calculation for pin 33
  channel_timer_2 = esc2 + loop_timer; //Time calculation for pin 32
  channel_timer_3 = esc3 + loop_timer; //Time calculation for pin 31
  channel_timer_4 = esc4 + loop_timer; //Time calculation for pin 30

  while (PORTC >= 16) //Execute till pins 33,32,31,30 are set low
  {
    esc_looptimer = micros();
    if (channel_timer_1 <= esc_looptimer)PORTC &= B11101111; //When delay time expires, pin 33 is set low
    if (channel_timer_2 <= esc_looptimer)PORTC &= B11011111; //When delay time expires, pin 32 is set low
    if (channel_timer_3 <= esc_looptimer)PORTC &= B10111111; //When delay time expires, pin 31 is set low
    if (channel_timer_4 <= esc_looptimer)PORTC &= B01111111; //When delay time expires, pin 30 is set low
  }
}


//////////////////////////////////////////
//Gyro output calculation
//////////////////////////////////////////

void gyro_signal()
{
  // Roll = x axis   Pitch = y axis   Yaw = rotation

  Wire.beginTransmission(105); //Start communicating with gyro at I2C address 105
  Wire.write(168);
  Wire.endTransmission();
  Wire.requestFrom(105, 6); //6 bytes of data (high byte and low byte of x,y,z axis)
  while (Wire.available() < 6); //Wait till 6 bytes are received

  //Roll
  lowByte = Wire.read();
  highByte = Wire.read();
  gyro_roll = ((highByte << 8) | lowByte);
  if (cal_int == 2000) gyro_roll -= gyro_roll_cal;

  //Pitch
  lowByte = Wire.read();
  highByte = Wire.read();
  gyro_pitch = ((highByte << 8) | lowByte);
  gyro_pitch *= -1;
  if (cal_int == 2000) gyro_pitch -= gyro_pitch_cal;

  //Yaw
  lowByte = Wire.read();
  highByte = Wire.read();
  gyro_yaw = ((highByte << 8) | lowByte);
  gyro_yaw *= -1;
  if (cal_int == 2000) gyro_yaw -= gyro_yaw_cal;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID Output Calculation Function
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pid_calculate()
{
  //Roll Calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

  if (pid_i_mem_roll > pid_maxroll)pid_i_mem_roll = pid_maxroll;
  else if (pid_i_mem_roll < pid_maxroll * -1)pid_i_mem_roll = pid_maxroll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);

  if (pid_output_roll > pid_maxroll)pid_output_roll = pid_maxroll;
  else if (pid_output_roll < pid_maxroll * -1)pid_output_roll = pid_maxroll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch Calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_maxpitch)pid_i_mem_pitch = pid_maxpitch;
  else if (pid_i_mem_pitch < pid_maxpitch * -1)pid_i_mem_pitch = pid_maxpitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

  if (pid_output_pitch > pid_maxpitch)pid_output_pitch = pid_maxpitch;
  else if (pid_output_pitch < pid_maxpitch * -1)pid_output_pitch = pid_maxpitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw Calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_maxyaw)pid_i_mem_yaw = pid_maxyaw;
  else if (pid_i_mem_yaw < pid_maxyaw * -1)pid_i_mem_yaw = pid_maxyaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);

  if (pid_output_yaw > pid_maxyaw)pid_output_yaw = pid_maxyaw;
  else if (pid_output_yaw < pid_maxyaw * -1)pid_output_yaw = pid_maxyaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt Routine PCI0 for Receiver
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(PCINT0_vect)
{
  current_time = micros();

  //Channel 1
  if (PINB & B00001000)
  {
    if (channelcount_1 == 0 )
    {
      channelcount_1 = 1;
      timer_1 = current_time;
    }
  }
  else if (channelcount_1 == 1 )
  {
    channelcount_1 = 0;
    receiverinput_channel_1 = current_time - timer_1;
  }

  //Channel 2
  if (PINB & B00000100)
  {
    if (channelcount_2 == 0 )
    {
      channelcount_2 = 1;
      timer_2 = current_time;
    }
  }
  else if (channelcount_2 == 1)
  {
    channelcount_2 = 0;
    receiverinput_channel_2 = current_time - timer_2;
  }

  //Channel 3
  if (PINB & B00000010)
  {
    if (channelcount_3 == 0 )
    {
      channelcount_3 = 1;
      timer_3 = current_time;
    }
  }
  else if (channelcount_3 == 1)
  {
    channelcount_3 = 0;
    receiverinput_channel_3 = current_time - timer_3;
  }

  //Channel 4
  if (PINB & B00000001) {
    if (channelcount_4 == 0 )
    {
      channelcount_4 = 1;
      timer_4 = current_time;
    }
  }
  else if (channelcount_4 == 1)
  {
    channelcount_4 = 0;
    receiverinput_channel_4 = current_time - timer_4;
  }
}


//////////////////////////////////////////
//Automatic Descent in case of low battery
//////////////////////////////////////////

//void automatic_descent()
//{
//}


void print_output() {
  Serial.print("Pitch:");
  if (gyro_pitch >= 0)Serial.print("+");
  Serial.print(gyro_pitch_input, 0);            //Convert to degree per second
  if (gyro_pitch_input - 2 > 0)Serial.print(" NoU");
  else if (gyro_pitch_input + 2 < 0)Serial.print(" NoD");
  else Serial.print(" ---");
  Serial.print("  Roll:");
  if (gyro_roll >= 0)Serial.print("+");
  Serial.print(gyro_roll_input, 0);             //Convert to degree per second
  if (gyro_roll_input - 2 > 0)Serial.print(" RwD");
  else if (gyro_roll_input + 2 < 0)Serial.print(" RwU");
  else Serial.print(" ---");
  Serial.print("  Yaw:");
  if (gyro_yaw >= 0)Serial.print("+");
  Serial.print(gyro_roll_input, 0);              //Convert to degree per second
  if (gyro_roll_input - 2 > 0)Serial.println(" NoR");
  else if (gyro_roll_input + 2 < 0)Serial.println(" NoL");
  else Serial.println(" ---");
}

void print_esc()
{
  Serial.print("ESC 1 = ");
  Serial.print(esc1);
  Serial.print("  ESC 2 = ");
  Serial.print(esc2);
  Serial.print("  ESC 3 = ");
  Serial.print(esc3);
  Serial.print("  ESC 4 = ");
  Serial.println(esc4);
}

