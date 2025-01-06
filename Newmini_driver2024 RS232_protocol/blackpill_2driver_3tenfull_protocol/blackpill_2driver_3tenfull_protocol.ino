/**
 *
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 *
 *
 * By using the serial terminal set the velocity value you want to motor to obtain
 *
 */
#include <SimpleFOC.h>
#include <Derivs_Limiter.h>
#include <Arduino.h>
#include <EEPROM.h>
//#include "crc8.h" 
//HardwareSerial Serial(PA10, PA9);
//HardwareSerial Serial(PA10, PA9);
// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(11);//11 chisai motor//14 oki motor
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PB6, PB7, PB8, PB5);

BLDCMotor motor2 = BLDCMotor(11);//11 chisai motor//14 oki motor
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PA0, PA1, PB0, PB1);//(3//10/11/7)(3/12/13/7)

MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5147_SPI, PA3);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5147_SPI, PA4);

StepDirListener step_dir1 = StepDirListener(PB12, PB13, 2.0f*_PI/3200);
void onStep1() { step_dir1.handle(); }
StepDirListener step_dir2 = StepDirListener(PB14, PB15, 2.0f*_PI/3200);
void onStep2() { step_dir2.handle(); }


Derivs_Limiter limiter1 = Derivs_Limiter(500, 500, 500); // velocityLimit, accelerationLimit, decelerationLimit
Derivs_Limiter limiter2 = Derivs_Limiter(500, 500, 500);
//MagneticSensorPWM sensor = MagneticSensorPWM(2, 4, 912);
//void doPWM(){sensor.handlePWM();}
//InlineCurrentSense current_sense1 = InlineCurrentSense(0.01f, 50.0f, PA0, PA4);
//InlineCurrentSense current_sense2 = InlineCurrentSense(0.01f, 50.0f,  PA1, PB0);

//StepDirListener step_dir = StepDirListener(3, 7, 2.0f*_PI/400);
//void onStep() { step_dir.handle(); }
unsigned long totalPacketsSent = 0;
uint8_t  data_telemetry8[15];
uint16_t data_telemetry16[15];
uint32_t data_telemetry32[20];
uint32_t data_float[10];
uint8_t Pos_run_test;
uint8_t error,MODE_motor,number_motor,yobi,motor_rd,motor_ss;
uint16_t yobi1 =1;
uint32_t yobi2 =11;
uint8_t stt_memory1,stt_memory2,stt_memory3,stt_memory4,stt_memory5,stt_memory6;
// velocity set point variable
float target_angle1 = 0;
float target_angle2 = 0;

float x_in = 0.00f;
float x_naka = 3.14f;
float x_out =6.28f;
float y_in = 0.00f;
float y_naka = 3.14f;
float y_out =6.28f;

float vel_x_in =50.00f;
float vel_x_naka =50.00f;
float vel_x_out=50.00f;
float vel_y_in=50.00f;
float vel_y_naka =50.00f;
float vel_y_out=50.00f;

float accel_x =100.00f;
float decel_x =100.00f;
float accel_y=100.00f;
float decel_y=100.00f;

float pid_pos_1 =20.00f;
float pid_vel_p_1 =0.25f;
float pid_vel_i_1 = 15.00f;
float vol_input_1 = 24.00f;
float vol_limit_1 =15.00f;
float ampe_limit_1 =2.00f;
float pid_pos_2 =20.00f;
float pid_vel_p_2 =0.25f;
float pid_vel_i_2 =15.00f;
float vol_input_2 = 24.00f;
float vol_limit_2 =15.00f;
float ampe_limit_2 =2.00f;

float pid_vd_1 =0.001f;
float pid_vd_2 =0.001f;

float almost_zero_velocity = 1;
byte one_time=0;
const int xilinda1_1_in = PA2; 
const int xilinda1_1_naka = PB13; 
const int xilinda1_1_out = PB2; 
const int sensor_M1_idouOK = PC13; 
const int sensor_M1_idouchu = PC14; 
//motor2
const int xilinda1_2_in = PC15;
const int xilinda1_2_naka = PB15; 
const int xilinda1_2_out = PA8; 
const int sensor_M2_idouOK = PA11; 
const int sensor_M2_idouchu = PA12; 
//error led
const int led_error_1 = PB9; 
const int led_error_2 = PB10; 
const int led_error_3 = PB3; 
const int led_error_4 = PB4; 
//motor2

float fuk1,fuk2;
int MODE_State1 = 0; 
int MODE_State2 = 0; 
int M1_IN_State1 = 0; 
int M1_IN_State2 = 0; 
int M1_IN_State3 = 0; 
int M2_IN_State1 = 0; 
int M2_IN_State2 = 0; 
int M2_IN_State3 = 0; 


//float number_motor = 2;
//float MODE_motor = 1 ;


float accell = 400;
float decell = 400;

//-----------------
float moto_pos_x_in = 0;
float moto_pos_x_naka = 0;
float moto_pos_x_out = 0;
float moto_pos_x = 0;
float moto_pos_x_in_old = 0;
float moto_pos_x_naka_old = 0;
float moto_pos_x_out_old = 0;
float moto_vel_x_in = 0;
float moto_vel_x_naka = 0;
float moto_vel_x_out = 0;
float moto_accel_x = 0;
float moto_decel_x = 0;
//--------------------
float moto_pos_y_in = 0;
float moto_pos_y_naka = 0;
float moto_pos_y_out = 0;
float moto_pos_y = 0;
float moto_pos_y_in_old = 0;
float moto_pos_y_naka_old = 0;
float moto_pos_y_out_old = 0;
float moto_vel_y_in = 0;
float moto_vel_y_naka = 0;
float moto_vel_y_out = 0;
float moto_accel_y = 0;
float moto_decel_y = 0;
//-------------------------
float moto_pos_x_now,moto_pos_y_now;

//TP----------------------------------
byte TP_pos11 =0,TP_pos13 =0,TP_pos12 =0,idou_pos1=0;
byte TP_pos21 =0,TP_pos23 =0,TP_pos22 =0,idou_pos2=0;

float moto_select;

//float target_angle1 = 0;
//float target_angle2 = 0;

float save_select;
float voltage_supply1,voltage_supply2,voltage_limit1,voltage_limit2;
int saving;
float MODE_select,genten_asix,number_select;
float defaut_ok =0;
// instantiate the commander
byte laplai1,laplai2,laplai3,laplai4,laplai5,laplai6;

// instantiate the commander
//Commander command = Commander(Serial);
////////////////////////////////////////////////////PROTOCOL
#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE];
static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;

//request MSP
#define MSP_HEADER = "$M<"
//
#define MSP_MOTOR_MODE 100
#define MSP_MOTOR_POS 101
#define MSP_MOTOR_VEL 102
#define MSP_MOTOR_ACCEL_DECCEL 103
#define MSP_MOTOR_VOL 104
#define MSP_MOTOR_VOLLIMIT 105
#define MSP_MOTOR_PID_P 106
#define MSP_MOTOR_PID_VEL_P 107
#define MSP_MOTOR_PID_VEL_I 108
#define MSP_MOTOR_PID_VEL_D 109
//////////////////
#define MSP_MOTOR_MOTOR_SET 1
#define MSP_MOTOR_MODE_SET 2
#define MSP_MOTOR_SAVE_SET 3
#define MSP_MOTOR_RESET_SET 4
#define MSP_MOTOR_RUN_SET  5

#define MSP_MOTOR_POS11_SET 10
#define MSP_MOTOR_POS12_SET 11
#define MSP_MOTOR_POS13_SET 34
#define MSP_MOTOR_POS21_SET 12
#define MSP_MOTOR_POS22_SET 13
#define MSP_MOTOR_POS23_SET 35

#define MSP_MOTOR_VEL11_SET 14
#define MSP_MOTOR_VEL12_SET 15
#define MSP_MOTOR_VEL13_SET 36
#define MSP_MOTOR_VEL21_SET 16
#define MSP_MOTOR_VEL22_SET 17
#define MSP_MOTOR_VEL23_SET 37

#define MSP_MOTOR_ACCEL1_SET 18
#define MSP_MOTOR_ACCEL2_SET 19
#define MSP_MOTOR_ACCEL3_SET 38
#define MSP_MOTOR_DECCEL1_SET 20
#define MSP_MOTOR_DECCEL2_SET 21
#define MSP_MOTOR_DECCEL3_SET 39

#define MSP_MOTOR_VOL_M1_SET 22
#define MSP_MOTOR_VOL_M2_SET 23

#define MSP_MOTOR_VOLLIMIT_M1_SET 24
#define MSP_MOTOR_VOLLIMIT_M2_SET 25
#define MSP_MOTOR_PID_P_M1_SET 26
#define MSP_MOTOR_PID_P_M2_SET 27
#define MSP_MOTOR_PID_VEL_P_M1_SET 28
#define MSP_MOTOR_PID_VEL_P_M2_SET 29
#define MSP_MOTOR_PID_VEL_I_M1_SET 30
#define MSP_MOTOR_PID_VEL_I_M2_SET 31
#define MSP_MOTOR_PID_VEL_D_M1_SET 32
#define MSP_MOTOR_PID_VEL_D_M2_SET 33

//
//void evaluateOtherData(uint8_t sr);
//void evaluateCommand(uint8_t c);

static uint8_t read8() {
  return inBuf[indRX++] & 0xff;
}
static uint16_t read16() {
  uint16_t t = read8();
  t += (uint16_t)read8() << 8;
  return t;
}
static uint32_t read32() {
  uint32_t t = read16();
  t += (uint32_t)read16() << 16;
  return t;
}

static void serialize8(uint8_t a) {
  Serial.write(a);
  checksum ^= a;
}
static void serialize16(int16_t a) {
  serialize8((a)&0xFF);
  serialize8((a >> 8) & 0xFF);
}
static void serialize32(uint32_t a) {
  serialize8((a)&0xFF);
  serialize8((a >> 8) & 0xFF);
  serialize8((a >> 16) & 0xFF);
  serialize8((a >> 24) & 0xFF);
}

static void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum = 0;  // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP);
}

static void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

static void headSerialError() {
  headSerialResponse(1, 0);
}

static void tailSerialReply() {
  serialize8(checksum);
}

static void serializeNames(PGM_P s) {
  headSerialReply(strlen_P(s));
  for (PGM_P c = s; pgm_read_byte(c); c++)
    serialize8(pgm_read_byte(c));
  tailSerialReply();
}

static void __attribute__((noinline)) s_struct_w(uint8_t* cb, uint8_t siz) {
  while (siz--) *cb++ = read8();
}

static void s_struct_partial(uint8_t* cb, uint8_t siz) {
  while (siz--) serialize8(*cb++);
}

static void s_struct(uint8_t* cb, uint8_t siz) {
  headSerialReply(siz);
  s_struct_partial(cb, siz);
  tailSerialReply();
}

static void mspAck() {
  headSerialReply(0);
  tailSerialReply();
}

enum MSP_protocol_bytes {
  IDLE,
  HEADER_START,
  HEADER_M,
  HEADER_ARROW,
  HEADER_SIZE,
  HEADER_CMD
};

void serialEvent() {
  uint8_t c, state, bytesTXBuff;
  static uint8_t offset;
  static uint8_t dataSize;
  static uint8_t c_state;


  // Serial.println(Serial.available());
  while (Serial.available() > 0) {
    //bytesTXBuff = SerialUsedTXBuff; // indicates the number of occupied bytes in TX buffer
    //if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
    c = Serial.read();
    //Serial.println(c);
    //#ifdef SUPPRESS_ALL_SERIAL_MSP
    //  evaluateOtherData(c); // no MSP handling, so go directly
    //#else //SUPPRESS_ALL_SERIAL_MSP
    state = c_state;
    // regular data handling to detect and handle MSP and other data
    if (state == IDLE) {//Serial.println(c);
      if (c == '$') {
        state = HEADER_START;
        
      } else {
        state = IDLE;
      }  //evaluateOtherData(c);} // evaluate all other incoming serial data
    } else if (state == HEADER_START) {
      state = (c == 'M') ? HEADER_M : IDLE;
    } else if (state == HEADER_M) {
      state = (c == '>') ? HEADER_ARROW : IDLE;
    } else if (state == HEADER_ARROW) {
      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        state = IDLE;
        continue;
      }
      dataSize = c;
      checksum = c;
      offset = 0;
      indRX = 0;
      state = HEADER_SIZE;  // the command is to follow
    } else if (state == HEADER_SIZE) {
      cmdMSP = c;
      checksum ^= c;
      state = HEADER_CMD;
    } else if (state == HEADER_CMD) {
      if (offset < dataSize) {
        checksum ^= c;
        inBuf[offset++] = c;
      } else {
        if (checksum == c)          // compare calculated and transferred checksum
          evaluateCommand(cmdMSP);  // we got a valid packet, evaluate it
        state = IDLE;
      }
    }
    c_state = state;

  }  // while
}







void setup() {
  Serial.begin(115200);
  delay(300);
 // motor_rd =1;
  //motor_ss =0;
  //send_stt_data();
 // delay(400);
  Serial.println(F("START SETUP"));
 //motor1
  pinMode(xilinda1_1_in, INPUT);//INPUT_PULLUP
  pinMode(xilinda1_1_naka, INPUT);
  pinMode(xilinda1_1_out, INPUT);
  pinMode(sensor_M1_idouOK, OUTPUT);
  pinMode(sensor_M1_idouchu, OUTPUT);
//motor2
  pinMode(xilinda1_2_in, INPUT);
  pinMode(xilinda1_2_naka, INPUT);
  pinMode(xilinda1_2_out, INPUT);
  pinMode(sensor_M2_idouOK, OUTPUT);
  pinMode(sensor_M2_idouchu, OUTPUT);
//led error
  //led_error_1
  pinMode(led_error_1, OUTPUT);
  pinMode(led_error_2, OUTPUT);
  pinMode(led_error_3, OUTPUT);
  pinMode(led_error_4, OUTPUT);

 //-----------------------STATUS---------------------------
 digitalWrite(sensor_M1_idouOK, HIGH);
 digitalWrite(sensor_M1_idouchu, HIGH);
 digitalWrite(sensor_M2_idouOK, HIGH);
 digitalWrite(sensor_M2_idouchu, HIGH);


 digitalWrite(led_error_1, LOW);
 digitalWrite(led_error_2, LOW);
 digitalWrite(led_error_3, LOW);
 digitalWrite(led_error_4, LOW);
 //--------------------------------
EEPROM.get(89,voltage_limit1);
EEPROM.get(85,voltage_supply1);
EEPROM.get(113,voltage_limit2);
EEPROM.get(109,voltage_supply2);

  // initialise magnetic sensor hardware
  driver1.voltage_limit = voltage_limit1;
  motor1.LPF_angle.Tf = 0.01;
  motor1.PID_velocity.output_ramp = 9000;
  sensor1.init();
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = voltage_supply1;
  driver1.init();
  // link the motor and the driver
  motor1.linkDriver(&driver1);
  //current_sense1.linkDriver(&driver1);
  
 // sensor.enableInterrupt(doPWM);
  // link the motor to the sensor
  driver2.voltage_limit = voltage_limit2;
  motor2.LPF_angle.Tf = 0.01;
  motor2.PID_velocity.output_ramp = 9000;
  sensor2.init();
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = voltage_supply2;
  driver2.init();
  motor2.linkDriver(&driver2);
 // current_sense2.linkDriver(&driver2);
  
 // choose FOC modulation (optional)
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor1.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::angle;


  // contoller configuration
  // default parameters in defaults.h
  EEPROM.get(1,moto_pos_x_in);
  EEPROM.get(17,moto_pos_x_naka);
  EEPROM.get(5,moto_pos_x_out);
  EEPROM.get(9,moto_pos_y_in);
  EEPROM.get(21,moto_pos_y_naka);
  EEPROM.get(13,moto_pos_y_out);

  
  EEPROM.get(41,moto_vel_x_in);
  EEPROM.get(25,moto_vel_x_naka);
  EEPROM.get(29,moto_vel_x_out);
  EEPROM.get(33,moto_vel_y_in);
  EEPROM.get(45,moto_vel_y_naka);
  EEPROM.get(37,moto_vel_y_out);

  
  EEPROM.get(49,moto_accel_x);
  EEPROM.get(53,moto_decel_x);
  EEPROM.get(57,moto_accel_y);
  EEPROM.get(61,moto_decel_y);

  
  EEPROM.get(73,motor1.P_angle.P);
  EEPROM.get(77,motor1.PID_velocity.P);
  EEPROM.get(81,motor1.PID_velocity.I);
  EEPROM.get(93,motor1.current_limit);

  EEPROM.get(97,motor2.P_angle.P);
  EEPROM.get(101,motor2.PID_velocity.P);
  EEPROM.get(105,motor2.PID_velocity.I);
  EEPROM.get(117,motor2.current_limit);

  
  EEPROM.get(145,motor1.PID_velocity.D);
  EEPROM.get(149,motor2.PID_velocity.D);

  
  

//mode read from eeprom
   MODE_motor =   EEPROM.read(158);
   number_motor = EEPROM.read(157);
 digitalWrite(led_error_1, HIGH);

if ( number_motor >=1 ){
  // initialize motor
  motor1.voltage_limit = voltage_limit1;
  motor1.voltage_sensor_align = 4;//24v-1
  motor1.velocity_index_search = 0.5;
  //current_sense1.init();
  //motor1.linkCurrentSense(&current_sense1);
  motor1.init();
  motor1.initFOC();//0.65, Direction::CW
  //Serial.println(motor1.initFOC().motor_status);
  }
   digitalWrite(led_error_2, HIGH);
  Serial.println(F("Motor1 ready."));
 //------------------------------------------------
 if ( number_motor >1 ){
  motor2.voltage_limit = voltage_limit2;
  motor2.voltage_sensor_align = 4;//24v-1
  motor2.velocity_index_search = 0.5;
  //current_sense2.init();
  //motor2.linkCurrentSense(&current_sense2);
  motor2.init();
//  motor2.initFOC(0.68, Direction::CW);
  motor2.initFOC();
 Serial.println(F("Motor2 ready."));
  digitalWrite(led_error_3, HIGH);
 }
 
 Serial.print("Mode_control motor_selected ");
 Serial.println(MODE_motor);
 Serial.print("number_motor_selected ");
 Serial.println(number_motor);


 step_dir1.init();
 step_dir2.init();

  // enable interrupts
 step_dir1.enableInterrupt(onStep1);
 step_dir2.enableInterrupt(onStep2);
  // attach the variable to be updated on each step (optional)
  // the same can be done asynchronously by caling motor.move(step_dir.getValue());
  
//
 if(MODE_motor == 2){
  
   step_dir1.attach(&motor1.target);
   step_dir2.attach(&motor2.target);
 }

//  motor1.useMonitoring(Serial);  
//  //motor.useMonitoring(Serial);
//  motor1.monitor_downsample = 0; // disable intially
//  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // monitor target velocity and angle
//  motor2.useMonitoring(Serial);
//  motor2.monitor_downsample = 0; // disable intially
//  motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
   

 //motor1
// 
  
//  motor.useMonitoring(Serial); 
  Serial.println(F("2 Motor ready."));
//  Serial.println(F("Set the target  using Serial terminal:"));
moto_pos_x_in_old = moto_pos_x_in ;
moto_pos_x_naka_old = moto_pos_x_naka ;
moto_pos_x_out_old = moto_pos_x_out;
moto_pos_y_in_old = moto_pos_y_in  ;
moto_pos_y_naka_old = moto_pos_y_naka ;
moto_pos_y_out_old = moto_pos_y_out;

fuk1 = sensor1.getAngle();
  if(fuk1 > 3.14)    {
    motor1.sensor_offset = 6.28;}                  //5.49 -->//0.79 <--
   if (fuk1 < 3.14){
    motor1.sensor_offset = 0;}                  //0.91 <--
//    Serial.print(motor1.sensor_offset);

fuk2 = sensor2.getAngle();
  if(fuk2 > 3.14)    {
    motor2.sensor_offset = 6.28;}                  //5.49 -->//0.79 <--
   if (fuk2 < 3.14){
    motor2.sensor_offset = 0;}              


  _delay(1000);
   digitalWrite(led_error_4, HIGH);
  motor_rd =0;
  motor_ss =1;
  //send_stt_data();
  //delay(400);
  digitalWrite(sensor_M1_idouchu, LOW);//LOW
  digitalWrite(sensor_M2_idouchu, LOW);//LOW
}

void loop() {

// if(motor1.shaft_velocity > almost_zero_velocity){
//    motor1.zero_electric_angle = 2.83;
//}else if(motor1.shaft_velocity < -almost_zero_velocity){
//    motor1.zero_electric_angle = 3;
//}else{
//    motor1.zero_electric_angle = 2.83;
//}
//
//  if(motor2.shaft_velocity > almost_zero_velocity){
//    motor2.zero_electric_angle = 3.3;
//}else if(motor2.shaft_velocity < -almost_zero_velocity){
//    motor2.zero_electric_angle = 3.3;
//}else{
//    motor2.zero_electric_angle = 3.3;
//}
driver1.voltage_limit = motor1.voltage_limit ;
driver2.voltage_limit = motor2.voltage_limit ;



if (one_time ==0){//genten SPEED 
  motor1.P_angle.limit = 5;motor2.P_angle.limit = 5;
 if(MODE_motor == 1 || MODE_motor == 2 && number_motor ==1 ){ 
 if( abs(motor1.shaft_angle - 0)<=0.05)
  {
    motor1.P_angle.limit = 5000;
    motor2.P_angle.limit = 5000;
    one_time = 1;
  }
 }

 if(MODE_motor == 1 || MODE_motor == 2 && number_motor ==2 ){ 
 if( (abs(motor1.shaft_angle - 0)<=0.05) &&(abs(motor2.shaft_angle - moto_pos_y_in)<=0.05 ))
  {
    motor1.P_angle.limit = 5000;
    motor2.P_angle.limit = 5000;
    one_time = 1;
  }
 }


    
 if(MODE_motor == 3&& number_motor ==1) {
  if(( abs(motor1.shaft_angle - moto_pos_x_in)<=0.05) &&(abs(motor2.shaft_angle - moto_pos_y_in)<=0.05 ))
  {
    motor1.P_angle.limit = 5000;
    motor2.P_angle.limit = 5000;
    one_time = 1;
  }}
 if(MODE_motor == 3&& number_motor ==2) {
  if(( abs(motor1.shaft_angle - moto_pos_x_in)<=0.05) &&(abs(motor2.shaft_angle - moto_pos_y_in)<=0.05 ))
  {
    motor1.P_angle.limit = 5000;
    motor2.P_angle.limit = 5000;
    one_time = 1;
  }}
 

  
}
//defaut setup
if (defaut_ok ==1 )
{
  defaut_set();
  defaut_ok = 0;
}



  
  //PLEASE RESET DRIVER TO MODE CHANGE MODE
if( MODE_select!= 0 ) {  
if( MODE_select == 1 ) {EEPROM.write(158, 1);}//PLC IO
if( MODE_select == 2 ) {EEPROM.write(158, 2);}//PULSE DIR
if( MODE_select == 3 ) {EEPROM.write(158, 3);}//Serial
 MODE_select = 0;
}

  //PLEASE RESET DRIVER TO MODE CHANGE MODE
if( number_select!= 0 ) {  
if( number_select == 1 ) {EEPROM.write(157, 1);}
if( number_select == 2 ) {EEPROM.write(157, 2);}
if( number_select == 3 ) {EEPROM.write(157, 3);}
 number_select = 0;
}

saving = save_select;
if( saving== 1 ) {

    EEPROM.put(1, moto_pos_x_in);
    delay(10);
    EEPROM.put(17, moto_pos_x_naka);
    delay(10);
    EEPROM.put(5, moto_pos_x_out);
    delay(10);
    EEPROM.put(25, moto_vel_x_in);
    delay(10);
    EEPROM.put(41, moto_vel_x_naka);
    delay(10);
    EEPROM.put(29, moto_vel_x_out);
    delay(10);
    EEPROM.put(49, moto_accel_x);
    delay(10);
    EEPROM.put(53, moto_decel_x);
    delay(10);
    EEPROM.put(73, motor1.P_angle.P);//pid pos
    delay(10);
    EEPROM.put(77, motor1.PID_velocity.P);//pid vel p
    delay(10);
    EEPROM.put(81, motor1.PID_velocity.I);//pid vel i
    delay(10);
    EEPROM.put(85, driver1.voltage_power_supply);//vol input
    delay(10);
    EEPROM.put(89, motor1.voltage_limit);//vol limit
    delay(10);
    EEPROM.put(93, motor1.current_limit);//ampe limit
    delay(10);
    EEPROM.put(145, motor1.PID_velocity.D);//pid vd
    delay(10);
      while(1){
   digitalWrite(PB9, LOW);
   digitalWrite(PB10, HIGH);
   digitalWrite(PB3, HIGH);
   digitalWrite(PB4, HIGH);
   delay(200);
  
   digitalWrite(PB9, HIGH);
   digitalWrite(PB10, LOW);
   digitalWrite(PB3, HIGH);
   digitalWrite(PB4, HIGH);
   delay(200);
  
   digitalWrite(PB9, HIGH);
   digitalWrite(PB10,HIGH);
   digitalWrite(PB3, LOW);
   digitalWrite(PB4, HIGH);
   delay(200);
  
   digitalWrite(PB9, HIGH);
   digitalWrite(PB10,HIGH);
   digitalWrite(PB3, HIGH);
   digitalWrite(PB4, LOW);
   delay(200);

  Serial.println("please restart");
      }
  
  
    
     save_select =0;
     
}
if( saving== 2 ) {
     
   
    EEPROM.put(9, moto_pos_y_in);
    delay(10);
    EEPROM.put(21, moto_pos_y_naka);
    delay(10);
    EEPROM.put(13, moto_pos_y_out);
    delay(10);
    EEPROM.put(33, moto_vel_y_in);
    delay(10);
    EEPROM.put(45, moto_vel_y_naka);
    delay(10);
    EEPROM.put(37, moto_vel_y_out);
    delay(10);
    EEPROM.put(57, moto_accel_y);
    delay(10);
    EEPROM.put(61, moto_decel_y);
    delay(10);
    EEPROM.put(97, motor2.P_angle.P);//pid pos
    delay(10);
    EEPROM.put(101, motor2.PID_velocity.P);//pid vel p
    delay(10);
    EEPROM.put(105, motor2.PID_velocity.I);//pid vel i
    delay(10);
    EEPROM.put(109, driver2.voltage_power_supply);//vol input
    delay(10);
    EEPROM.put(113, motor2.voltage_limit);//vol limit
    delay(10);
    EEPROM.put(117, motor2.current_limit);//ampe limit
    delay(10);
    EEPROM.put(149, motor2.PID_velocity.D);//ampe limit
    delay(10);
    while(1){
   digitalWrite(PB9, LOW);
   digitalWrite(PB10, HIGH);
   digitalWrite(PB3, HIGH);
   digitalWrite(PB4, HIGH);
   delay(200);
  
   digitalWrite(PB9, HIGH);
   digitalWrite(PB10, LOW);
   digitalWrite(PB3, HIGH);
   digitalWrite(PB4, HIGH);
   delay(200);
  
   digitalWrite(PB9, HIGH);
   digitalWrite(PB10,HIGH);
   digitalWrite(PB3, LOW);
   digitalWrite(PB4, HIGH);
   delay(200);
  
   digitalWrite(PB9, HIGH);
   digitalWrite(PB10,HIGH);
   digitalWrite(PB3, HIGH);
   digitalWrite(PB4, LOW);
   delay(200);

  Serial.println("please restart");
      }
  
  save_select =0;
  
}


//genten
 if( genten_asix ==1){motor1.velocity_limit = 5 ;accell = 10  ;decell = 10  ;moto_pos_x = 0;genten_asix =0; }
 if( genten_asix ==2){motor2.velocity_limit = 5 ;accell = 10  ;decell = 10  ;moto_pos_y = 0;genten_asix =0; }
// if(abs(motor1.shaft_angle )<=0.1){
//
//     digitalWrite(sensor_M1_idouOK, HIGH);}
//     
//  else {  
//          digitalWrite(sensor_M1_idouOK, LOW);}
//  if(abs(motor2.shaft_angle )<=0.1){
//
//     digitalWrite(sensor_M2_idouOK, HIGH);}
//     
//  else {  
//          digitalWrite(sensor_M2_idouOK, LOW);}


 if(MODE_motor == 1){
    M1_IN_State1 = digitalRead(xilinda1_1_in);
    M1_IN_State2 = digitalRead(xilinda1_1_out);
    M1_IN_State3 = digitalRead(xilinda1_1_naka);
    M2_IN_State1 = digitalRead(xilinda1_2_in);
    M2_IN_State2 = digitalRead(xilinda1_2_out);
    M2_IN_State3 = digitalRead(xilinda1_2_naka);


    //*******************************************************************************************************
    if(( M1_IN_State1 == LOW && M1_IN_State2 == HIGH&& M1_IN_State3 == HIGH )||( Pos_run_test ==1)||( moto_pos_x_in != moto_pos_x_in_old)||(stt_memory1 ==1)){
      if (abs(moto_pos_x_now - moto_pos_x_in) >= 0.05){
      motor1.velocity_limit = moto_vel_x_in ;
     accell = moto_accel_x  ;
     decell = moto_decel_x  ;
     if ((moto_pos_x_in >=-100)&&(moto_pos_x_in <=100)){
      moto_pos_x = moto_pos_x_in;
      moto_pos_x_in_old=moto_pos_x_in;}
     
     // Serial.println("MOTOR 1 IN");
     TP_pos11 =0;
     Pos_run_test = 0;


 //////digitalWrite(sensor_M1_idouOK, LOW);///////
  
  if(abs(motor1.shaft_angle - moto_pos_x_in_old)<=0.1){
    digitalWrite(sensor_M1_idouchu, LOW);
    digitalWrite(sensor_M1_idouOK, HIGH);
    moto_pos_x_now = moto_pos_x_in_old;
    stt_memory1 =0;
  }
     
   
  else {  digitalWrite(sensor_M1_idouchu, HIGH); 
          digitalWrite(sensor_M1_idouOK, LOW);
          moto_pos_x_now = 0;
          stt_memory1 =1;
    }
     
       laplai1 = 1;
    }
    
    if (abs(moto_pos_x_now - moto_pos_x_in) <= 0.05 && laplai1 == 0){
    digitalWrite(sensor_M1_idouOK, LOW);
    delay(10);
    digitalWrite(sensor_M1_idouOK, HIGH);
    laplai1 = 1;
    }
    }
    else {laplai1 = 0;}

    
 //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   if((  M1_IN_State1 == HIGH && M1_IN_State2 == LOW && M1_IN_State3 == HIGH )||( Pos_run_test == 2)|| ( moto_pos_x_out != moto_pos_x_out_old)||(stt_memory2 ==1)){
      if (abs(moto_pos_x_now - moto_pos_x_out) >= 0.05){
       motor1.velocity_limit = moto_vel_x_out ;
       accell = moto_accel_x  ;
       decell = moto_decel_x  ;
      if ((moto_pos_x_out >=-100)&&(moto_pos_x_out <=100)){
        moto_pos_x = moto_pos_x_out;
        moto_pos_x_out_old=moto_pos_x_out;}
     
   //  Serial.println("MOTOR 1 OUT");
     TP_pos12=0;
     Pos_run_test = 0;
    
 if(abs(motor1.shaft_angle - moto_pos_x_out_old)<=0.1){
      //  Serial.print("2_ok");
      digitalWrite(sensor_M1_idouOK, HIGH);
     digitalWrite(sensor_M1_idouchu, LOW);
     moto_pos_x_now = moto_pos_x_out_old;
     stt_memory2 =0;
     }
  else {  digitalWrite(sensor_M1_idouchu, HIGH);
 // Serial.print("2eo_ok");
      digitalWrite(sensor_M1_idouOK, LOW);
      stt_memory2 =1;
    //  moto_pos_x_now =0;
     }

   laplai2 = 1;
    }
    if (abs(moto_pos_x_now - moto_pos_x_out) <= 0.05 && laplai2 == 0){
    digitalWrite(sensor_M1_idouOK, LOW);
    delay(10);
    digitalWrite(sensor_M1_idouOK, HIGH);
    laplai2 = 1;
    }
    }
    else {laplai2 = 0;}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   if((  M1_IN_State1 == HIGH && M1_IN_State2 == HIGH && M1_IN_State3 == LOW )||( Pos_run_test == 3)|| ( moto_pos_x_naka != moto_pos_x_naka_old)||(stt_memory3 ==1)){
       if (abs(moto_pos_x_now - moto_pos_x_naka) >= 0.05){
       motor1.velocity_limit = moto_vel_x_naka ;
       accell = moto_accel_x  ;
       decell = moto_decel_x  ;
      if ((moto_pos_x_naka >=-100)&&(moto_pos_x_naka <=100)){
        moto_pos_x = moto_pos_x_naka;
        moto_pos_x_naka_old=moto_pos_x_naka;}
     
   //  Serial.println("MOTOR 1 OUT");
     TP_pos13=0;
     Pos_run_test = 0;
    
 if(abs(motor1.shaft_angle - moto_pos_x_naka_old)<=0.1){
    // Serial.print("3_ok");
        digitalWrite(sensor_M1_idouOK, HIGH);
     digitalWrite(sensor_M1_idouchu, LOW);
     moto_pos_x_now = moto_pos_x_naka_old;
     stt_memory3=0;
     }
  else {  
   digitalWrite(sensor_M1_idouchu, HIGH); 
   digitalWrite(sensor_M1_idouOK, LOW);
   // moto_pos_x_now = 0;
   stt_memory3=1;
   }
   laplai3 = 1;
    }
    if (abs(moto_pos_x_now - moto_pos_x_naka) <= 0.05 && laplai3 == 0){
    digitalWrite(sensor_M1_idouOK, LOW);
    delay(10);
    digitalWrite(sensor_M1_idouOK, HIGH);
    laplai3 = 1;
    }
    }
    else {laplai3 = 0;}
    
//Serial.print(M1_IN_State1);
//Serial.print(M1_IN_State2);
//Serial.print(M1_IN_State3);
//Serial.print(M2_IN_State1);
//Serial.print(M2_IN_State2);
//Serial.println(M2_IN_State3);

  
   //************************************************************************************************************
    if(( M2_IN_State1 == LOW && M2_IN_State2 == HIGH  && M2_IN_State3 == HIGH )||( Pos_run_test == 11)|| ( moto_pos_y_in != moto_pos_y_in_old)||(stt_memory4 ==1)){
      if (abs(moto_pos_y_now - moto_pos_y_in) >= 0.05){
        motor2.velocity_limit = moto_vel_y_in ;
        accell = moto_accel_y  ;
        decell = moto_decel_y  ;  
         if ((moto_pos_y_in >=-100)&&(moto_pos_y_in <=100)){
          moto_pos_y = moto_pos_y_in;
          moto_pos_y_in_old=moto_pos_y_in;}
      
        //  Serial.println(F("MOTOR 2 IN"));
         Pos_run_test = 0;
        
      if(abs(motor2.shaft_angle - moto_pos_y_in_old)<=0.1){
          digitalWrite(sensor_M2_idouOK, HIGH);
         digitalWrite(sensor_M2_idouchu, LOW);
         moto_pos_y_now = moto_pos_y_in_old;
         stt_memory4 =0;
         }
      else {  
        digitalWrite(sensor_M2_idouchu, HIGH); 
        digitalWrite(sensor_M2_idouOK, LOW);
        //moto_pos_y_now = 0;
        stt_memory4=1;
        }
        laplai4 = 1;
    }
    if (abs(moto_pos_y_now - moto_pos_y_in) <= 0.05 && laplai4 == 0){
    digitalWrite(sensor_M2_idouOK, LOW);
    delay(10);
    digitalWrite(sensor_M2_idouOK, HIGH);
    laplai4 = 1;
    }
    }
    else {laplai4 = 0;}
    
 //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   if((  M2_IN_State1 == HIGH && M2_IN_State2 == LOW && M2_IN_State3 == HIGH)|| ( Pos_run_test == 12)||( moto_pos_y_out != moto_pos_y_out_old) ||(stt_memory5 ==1)){
     if (abs(moto_pos_y_now - moto_pos_y_out) >= 0.05){
     motor2.velocity_limit = moto_vel_y_out ;
     accell = moto_accel_y  ;
     decell = moto_decel_y  ;
     if ((moto_pos_y_out >=-100)&&(moto_pos_y_out <=100)){
      moto_pos_y = moto_pos_y_out;
      moto_pos_y_out_old=moto_pos_y_out;}
     
     // Serial.println(F("MOTOR 2 OUT"));
      Pos_run_test = 0;
    
 if(abs(motor2.shaft_angle - moto_pos_y_out_old)<=0.1){///motor2.shaft_angle
     digitalWrite(sensor_M2_idouchu, LOW);
    digitalWrite(sensor_M2_idouOK, HIGH);
    moto_pos_y_now = moto_pos_y_out_old;
    stt_memory5=0;
     }
  else {  
   digitalWrite(sensor_M2_idouchu, HIGH); 
   digitalWrite(sensor_M2_idouOK, LOW);
   stt_memory5=1;
   }
    laplai5 = 1;
   }
    if (abs(moto_pos_y_now - moto_pos_y_out) <= 0.05 && laplai5 == 0){
    digitalWrite(sensor_M2_idouOK, LOW);
    delay(10);
    digitalWrite(sensor_M2_idouOK, HIGH);
    laplai5 = 1;
    }
    }
    else {laplai5 = 0;}

 //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   if((  M2_IN_State1 == HIGH && M2_IN_State2 == HIGH && M2_IN_State3 == LOW)|| ( Pos_run_test == 13)||( moto_pos_y_naka != moto_pos_y_naka_old) ||(stt_memory6 ==1)){
     if (abs(moto_pos_y_now - moto_pos_y_naka) >= 0.05){
     motor2.velocity_limit = moto_vel_y_naka ;
     accell = moto_accel_y  ;
     decell = moto_decel_y  ;
     if ((moto_pos_y_naka >=-100)&&(moto_pos_y_naka <=100)){
      moto_pos_y = moto_pos_y_naka;
      moto_pos_y_naka_old=moto_pos_y_naka;}
     
     // Serial.println(F("MOTOR 2 OUT"));
      Pos_run_test = 0;
    
 if(abs(motor2.shaft_angle - moto_pos_y_naka_old)<=0.1){
     digitalWrite(sensor_M2_idouchu, LOW);
     digitalWrite(sensor_M2_idouOK, HIGH);
     moto_pos_y_now = moto_pos_y_naka_old;
     stt_memory6 =0;
     }
  else {  digitalWrite(sensor_M2_idouchu, HIGH);  
         digitalWrite(sensor_M2_idouOK, LOW);
         stt_memory6=1;
         }
         // moto_pos_y_now = 0;
    laplai6 = 1;
    }
    
    
    if (abs(moto_pos_y_now - moto_pos_y_out) <= 0.05 && laplai6 == 0){
    digitalWrite(sensor_M2_idouOK, LOW);
    delay(10);
    digitalWrite(sensor_M2_idouOK, HIGH);
    laplai6 = 1;
    }
    }
    else {laplai6 = 0;}
 }


 
if(MODE_motor == 3){
  motor1.velocity_limit = moto_vel_x_in ;
     accell = moto_accel_x  ;
     decell = moto_decel_x  ;
  motor2.velocity_limit = moto_vel_y_in ;
     accell = moto_accel_y  ;
     decell = moto_decel_y  ;   
  
  moto_pos_x = moto_pos_x_in;
  moto_pos_y = moto_pos_y_in;

     
}


  limiter1.setVelAccelLimits(motor1.velocity_limit,accell,decell);
  limiter2.setVelAccelLimits(motor2.velocity_limit,accell,decell);


  motor1.loopFOC();
  motor2.loopFOC();


//--------------------------------シリンダー MODE---------------------------
 if(MODE_motor == 1){
 
   if (one_time==0){
   motor1.move(moto_pos_x);
   motor2.move(moto_pos_y);
  
   }
   else
   {
   motor1.move(limiter1.calc(moto_pos_x));
   motor2.move(limiter2.calc(moto_pos_y));

   
   }
 }

//--------------------------------PULSE MODE-------------------------------
  if(MODE_motor == 2){
   
    motor1.move();
    motor2.move();
   }

//-------------------------------------SERIAL MODE--------------------------
   if(MODE_motor == 3){
   motor1.move(limiter1.calc(moto_pos_x));
   motor2.move(limiter2.calc(moto_pos_y));
    
   }
//-----------------------------------------------------------------------
//  motor1.monitor(); 
//  motor2.monitor(); 
//  motor3.monitor(); 
//  command.run();
//  send_telemetry_data();
 // PhaseCurrent_s currents2 = current_sense2.getPhaseCurrents();
 //   float current_magnitude2 = current_sense2.getDCCurrent();


//    Serial.print(currents2.a);
//    Serial.print("\t");
//     Serial.print(currents2.b);
//     Serial.print("\t");
//     Serial.println(currents2.c);
//Serial.print(sensor2.getAngle());
//Serial.print("/");
//Serial.println(sensor3.getAngle());
}

void defaut_set()
{

    
    EEPROM.put(1, x_in);//moto_pos_x_in
    EEPROM.put(17, x_naka);//moto_pos_x_naka
    EEPROM.put(5, x_out);//moto_pos_x_out
    EEPROM.put(9, y_in);//moto_pos_y_in
    EEPROM.put(21, y_naka);//moto_pos_y_naka
    EEPROM.put(13,y_out);//moto_pos_y_out

    EEPROM.put(25,vel_x_in);//moto_vel_x_in
    EEPROM.put(41,vel_x_naka);//moto_vel_x_naka
    EEPROM.put(29,vel_x_out);//moto_vel_x_out
    EEPROM.put(33,vel_y_in);//moto_vel_y_in
    EEPROM.put(45,vel_y_naka);//moto_vel_x_naka
    EEPROM.put(37,vel_y_out);//moto_vel_y_out

    EEPROM.put(49, moto_accel_x);//moto_accel_x
    EEPROM.put(53, moto_decel_x);//moto_decel_x
    EEPROM.put(57, moto_accel_y);//moto_accel_y
    EEPROM.put(61, moto_decel_y);//moto_decel_y

    EEPROM.put(73, pid_pos_1);//pid pos
    EEPROM.put(77, pid_vel_p_1);//pid vel p
    EEPROM.put(81, pid_vel_i_1);//pid vel i
    EEPROM.put(85, vol_input_1);//vol input
    EEPROM.put(89, vol_limit_1);//vol limit
    EEPROM.put(93, ampe_limit_1);//ampe limit
    EEPROM.put(97, pid_pos_2);//pid pos
    EEPROM.put(101, pid_vel_p_2);//pid vel p
    EEPROM.put(105, pid_vel_i_2);//pid vel i
    EEPROM.put(109, vol_input_2);//vol input
    EEPROM.put(113, vol_limit_2);//vol limit
    EEPROM.put(117, ampe_limit_2);//ampe limit
    
    EEPROM.put(145, pid_vd_1);//pid vd1
    EEPROM.put(149, pid_vd_2);//pid_vd2

    EEPROM.write(157,2);//number_motor
    EEPROM.write(158,MODE_motor);
    
 Serial.println("defaut_ok");
  while(1){
   digitalWrite(PB9, HIGH);
   digitalWrite(PB10, LOW);
  delay(200);
  
  digitalWrite(PB10, HIGH);
  digitalWrite(PB9, LOW);
  delay(200);
  
  digitalWrite(PB9, HIGH);
   digitalWrite(PB10, LOW);
  delay(200);
  
  digitalWrite(PB10, HIGH);
  digitalWrite(PB9, LOW);
  delay(200);

  digitalWrite(PB9, HIGH);
   digitalWrite(PB10, LOW);
  delay(200);
  
  digitalWrite(PB10, HIGH);
  digitalWrite(PB9, LOW);
  delay(200);

  
  
    Serial.println("please restart");
    //delay(1000);
    }
  
  }




void evaluateCommand(uint8_t c) {
  switch (c) {
    
    case MSP_MOTOR_MODE:
      headSerialReply(5);
      serialize8(error);
      serialize8(MODE_motor);
      serialize8(number_motor);
      serialize8(motor_rd);
      serialize8(motor_ss);
      tailSerialReply();
      break;
    case MSP_MOTOR_MOTOR_SET:
      number_select = read8();
      break;
    case MSP_MOTOR_MODE_SET:
      MODE_select = read8();
      break;
    case MSP_MOTOR_SAVE_SET:
      save_select = read8();
      break;
    case MSP_MOTOR_RESET_SET:
      defaut_ok = read8();
      break;
    case MSP_MOTOR_RUN_SET:
      Pos_run_test = read8();
      break;
    case MSP_MOTOR_POS:
      headSerialReply(24);
      serialize32(FloatToUint(moto_pos_x_in));
      serialize32(FloatToUint(moto_pos_x_out));
      serialize32(FloatToUint(moto_pos_x_naka));
      serialize32(FloatToUint(moto_pos_y_in));
      serialize32(FloatToUint(moto_pos_y_out));
      serialize32(FloatToUint(moto_pos_y_naka));
      tailSerialReply();
      break;
    case MSP_MOTOR_POS11_SET:
      moto_pos_x_in = UintToFloat(read32());//moto_pos_x_in
      break;
    case MSP_MOTOR_POS12_SET:
      moto_pos_x_out = UintToFloat(read32());//moto_pos_x_in
      break;
    case MSP_MOTOR_POS13_SET:
      moto_pos_x_naka = UintToFloat(read32());//moto_pos_x_in
      break;
    case MSP_MOTOR_POS21_SET:
      moto_pos_y_in = UintToFloat(read32());//moto_pos_x_in
      break;
    case MSP_MOTOR_POS22_SET:
      moto_pos_y_out = UintToFloat(read32());//moto_pos_x_in
      break;
    case MSP_MOTOR_POS23_SET:
      moto_pos_y_naka = UintToFloat(read32());//moto_pos_x_in
      break;
    case MSP_MOTOR_VEL:
      headSerialReply(12);
      serialize16(moto_vel_x_in);
      serialize16(moto_vel_x_out);
      serialize16(moto_vel_x_naka);
      serialize16(moto_vel_y_in);
      serialize16(moto_vel_y_out);
      serialize16(moto_vel_y_naka);
      tailSerialReply();
      break;
    case MSP_MOTOR_VEL11_SET:
      moto_vel_x_in = read16();
      break;
    case MSP_MOTOR_VEL12_SET:
      moto_vel_x_out = read16();
      break;
    case MSP_MOTOR_VEL13_SET:
      moto_vel_x_naka = read16();
      break;
    case MSP_MOTOR_VEL21_SET:
      moto_vel_y_in = read16();
      break;
    case MSP_MOTOR_VEL22_SET:
      moto_vel_y_out = read16();
      break;
    case MSP_MOTOR_VEL23_SET:
      moto_vel_y_naka = read16();
      break;
    case MSP_MOTOR_ACCEL_DECCEL:
      headSerialReply(8);
      serialize16(moto_accel_x);
      serialize16(moto_accel_y);
      serialize16(moto_decel_x);
      serialize16(moto_decel_y);
      tailSerialReply();
      break;
    case MSP_MOTOR_ACCEL1_SET:
      moto_accel_x =read16();
      break;
    case MSP_MOTOR_ACCEL2_SET:
      moto_accel_y =read16();
      break;
    case MSP_MOTOR_DECCEL1_SET:
      moto_decel_x =read16();
      break;
    case MSP_MOTOR_DECCEL2_SET:
      moto_decel_y =read16();
      break;
    
    case MSP_MOTOR_VOL:
      headSerialReply(8);
      serialize32(FloatToUint(driver1.voltage_power_supply));
      serialize32(FloatToUint(driver2.voltage_power_supply));
      tailSerialReply();
      break;
    case MSP_MOTOR_VOL_M1_SET:
      driver1.voltage_power_supply = UintToFloat(read32());
      break;
    case MSP_MOTOR_VOL_M2_SET:
      driver2.voltage_power_supply = UintToFloat(read32());
      break;

    case MSP_MOTOR_VOLLIMIT:
      headSerialReply(8);
      serialize32(FloatToUint(motor1.voltage_limit));
      serialize32(FloatToUint(motor2.voltage_limit));
      tailSerialReply();
      break;
    case MSP_MOTOR_VOLLIMIT_M1_SET:
      motor1.voltage_limit = UintToFloat(read32());
      break;
    case MSP_MOTOR_VOLLIMIT_M2_SET:
      motor2.voltage_limit = UintToFloat(read32());
      break;

    case MSP_MOTOR_PID_P:
      headSerialReply(8);
      serialize32(FloatToUint(motor1.P_angle.P));
      serialize32(FloatToUint(motor2.P_angle.P));
      tailSerialReply();
      break;
    case MSP_MOTOR_PID_P_M1_SET:
      motor1.P_angle.P = UintToFloat(read32());
      break;
    case MSP_MOTOR_PID_P_M2_SET:
      motor2.P_angle.P = UintToFloat(read32());
      break;

    case MSP_MOTOR_PID_VEL_P:
      headSerialReply(8);
      serialize32(FloatToUint(motor1.PID_velocity.P));
      serialize32(FloatToUint(motor2.PID_velocity.P));
      tailSerialReply();
      break;
    case MSP_MOTOR_PID_VEL_P_M1_SET:
      motor1.PID_velocity.P = UintToFloat(read32());
      break;
    case MSP_MOTOR_PID_VEL_P_M2_SET:
      motor2.PID_velocity.P = UintToFloat(read32());
      break;

    case MSP_MOTOR_PID_VEL_I:
      headSerialReply(8);
      serialize32(FloatToUint(motor1.PID_velocity.I));
      serialize32(FloatToUint(motor2.PID_velocity.I));
      tailSerialReply();
      break;
    case MSP_MOTOR_PID_VEL_I_M1_SET:
      motor1.PID_velocity.I = UintToFloat(read32());
      break;
    case MSP_MOTOR_PID_VEL_I_M2_SET:
      motor2.PID_velocity.I = UintToFloat(read32());
      break;

    case MSP_MOTOR_PID_VEL_D:
      headSerialReply(8);
      serialize32(FloatToUint(motor1.PID_velocity.D));
      serialize32(FloatToUint(motor2.PID_velocity.D));
      tailSerialReply();
      break;
    case MSP_MOTOR_PID_VEL_D_M1_SET:
      motor1.PID_velocity.D = UintToFloat(read32());
      break;
    case MSP_MOTOR_PID_VEL_D_M2_SET:
      motor2.PID_velocity.D = UintToFloat(read32());
      break;

      //    Serial.print(currents2.a);
      //    Serial.print("\t");
      //     Serial.print(currents2.b);
      //     Serial.print("\t");
      //     Serial.println(currents2.c);
      //Serial.print(sensor2.getAngle());
      //Serial.print("/");
      //Serial.println(sensor3.getAngle());
  }
}



uint32_t FloatToUint(float n)
{
   return (uint32_t)(*(uint32_t*)&n);
}
 
float UintToFloat(uint32_t n)
{
   return (float)(*(float*)&n);
}

