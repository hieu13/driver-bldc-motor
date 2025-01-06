#include <SimpleFOC.h>
#include <Derivs_Limiter.h>
#include <Arduino.h>
#include <EEPROM.h>

//HardwareSerial Serial(PA10, PA9);
//HardwareSerial Serial(PA10, PA9);
// BLDC motor & driver instance




BLDCMotor motor1 = BLDCMotor(EEPROM.read(161));//11
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PB6, PB7, PB8, PB5);

BLDCMotor motor2 = BLDCMotor(EEPROM.read(162));
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PA0, PA1, PB0, PB1);  //(3//10/11/7)(3/12/13/7)

MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5147_SPI, PA3);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5147_SPI, PA4);

StepDirListener step_dir1 = StepDirListener(PB12, PB13, 2.0f * _PI /(100* EEPROM.read(159)));//800
void onStep1() {
  step_dir1.handle();
}
StepDirListener step_dir2 = StepDirListener(PB14, PB15, 2.0f * _PI /(100* EEPROM.read(160)));//800
void onStep2() {
  step_dir2.handle();
}


Derivs_Limiter limiter1 = Derivs_Limiter(500, 500, 500);  // velocityLimit, accelerationLimit, decelerationLimit
Derivs_Limiter limiter2 = Derivs_Limiter(500, 500, 500);
//MagneticSensorPWM sensor = MagneticSensorPWM(2, 4, 912);
//void doPWM(){sensor.handlePWM();}
//InlineCurrentSense current_sense1 = InlineCurrentSense(0.01f, 50.0f, PA0, PA4);
//InlineCurrentSense current_sense2 = InlineCurrentSense(0.01f, 50.0f,  PA1, PB0);

//StepDirListener step_dir = StepDirListener(3, 7, 2.0f*_PI/400);
//void onStep() { step_dir.handle(); }
unsigned long totalPacketsSent = 0;
uint8_t data_telemetry8[15];
uint16_t data_telemetry16[15];
uint32_t data_telemetry32[20];
uint32_t data_float[10];
uint8_t error,error_rs,error_reset, MODE_motor, number_motor, yobi, motor_rd, motor_ss,nc;
uint8_t motor1_resolution,motor2_resolution,motor1_pole,motor2_pole;
uint8_t motor1_resolution_old,motor2_resolution_old,motor1_pole_old,motor2_pole_old;

uint16_t now_velocity_1,now_velocity_2,now_acceleration_1,now_acceleration_2,now_decceleration_1,now_decceleration_2;
uint32_t time_over1,time_over2;
bool safe_dis1=false,safe_dis2=false,timer_int_cut1,timer_int_cut2;
unsigned long previousMillis = 0;  // will store last time LED was updated
int ledState = LOW;
// constants won't change:
const long interval = 100;  // interval at which to blink (milliseconds)

uint16_t yobi1 = 1;
uint32_t yobi2 = 11;
// velocity set point variable
float target_angle1 = 0;
float target_angle2 = 0;

float x_in = 0.00f;
float x_out = 6.28f;
float y_in = 0.00f;
float y_out = 6.28f;

float vel_x_in = 50.00f;
float vel_x_out = 50.00f;
float vel_y_in = 50.00f;
float vel_y_out = 50.00f;

float accel_x = 100.00f;
float decel_x = 100.00f;
float accel_y = 100.00f;
float decel_y = 100.00f;

float pid_pos_1 = 20.00f;
float pid_vel_p_1 = 0.25f;
float pid_vel_i_1 = 15.00f;
float vol_input_1 = 24.00f;
float vol_limit_1 = 15.00f;
float ampe_limit_1 = 2.00f;
float pid_pos_2 = 20.00f;
float pid_vel_p_2 = 0.25f;
float pid_vel_i_2 = 15.00f;
float vol_input_2 = 24.00f;
float vol_limit_2 = 15.00f;
float ampe_limit_2 = 2.00f;

float pid_vd_1 = 0.001f;
float pid_vd_2 = 0.001f;

float almost_zero_velocity = 1;
byte one_time = 0;
const int xilinda1_1_in = PA2;
const int xilinda1_1_out = PB2;
const int sensor_1_in = PC13;
const int sensor_1_out = PC14;
//motor2
const int xilinda1_2_in = PC15;
const int xilinda1_2_out = PA8;
const int sensor_2_in = PA11;
const int sensor_2_out = PA12;
//error led
const int led_error_1 = PB9;
const int led_error_2 = PB10;
const int led_error_3 = PB3;
const int led_error_4 = PB4;
//motor2

float fuk1, fuk2;
int MODE_State1 = 0;
int MODE_State2 = 0;
int M1_IN_State1 = 0;
int M1_IN_State2 = 0;
int M2_IN_State1 = 0;
int M2_IN_State2 = 0;
float accell = 400;
float decell = 400;

//float number_motor = 2;
//float MODE_motor = 1 ;
float moto_1_pos_set,moto_2_pos_set;


float moto_velocity_1_set =50;
float moto_velocity_2_set =50;

float accell_mode3_motor_1 = 400;
float accell_mode3_motor_2 = 400;
float deccell_mode3_motor_1 = 400;
float deccell_mode3_motor_2 = 400;
//-----------------
float moto_pos_x_in = 0;
float moto_pos_x_out = 0;
float moto_pos_x = 0;
float moto_pos_x_in_old = 0;
float moto_pos_x_out_old = 0;
float moto_vel_x_in = 0;
float moto_vel_x_out = 0;
float moto_accel_x = 0;
float moto_decel_x = 0;
//--------------------
float moto_pos_y_in = 0;
float moto_pos_y_out = 0;
float moto_pos_y = 0;
float moto_pos_y_in_old = 0;
float moto_pos_y_out_old = 0;
float moto_vel_y_in = 0;
float moto_vel_y_out = 0;
float moto_accel_y = 0;
float moto_decel_y = 0;
//-------------------------
uint8_t motor1_resolution_epp=8;//x100 =800 pulse
uint8_t motor2_resolution_epp=8;//x100 =800 pulse
uint8_t motor1_pole_epp=11;
uint8_t motor2_pole_epp=11;

//TP----------------------------------
byte TP_pos11 = 0, TP_pos12 = 0, idou_pos1 = 0;
byte TP_pos21 = 0, TP_pos22 = 0, idou_pos2 = 0;

float moto_select;

//float target_angle1 = 0;
//float target_angle2 = 0;
uint8_t motor_dis_val,data_motor1,data_motor2;
uint8_t save_select;
float voltage_supply1, voltage_supply2, voltage_limit1, voltage_limit2;
int saving;
float MODE_select, genten_asix, number_select;
float defaut_ok = 0;
// instantiate the commander
uint8_t control_motion_1,control_motion_2,control_motion_1_old,control_motion_2_old;
uint8_t Pos_run_test;
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
#define MSP_MOTOR_ERR_SET  6
#define MSP_MOTOR_NC_SET  7
#define MSP_MOTOR_DATAPOS1_SET  8
#define MSP_MOTOR_DATAPOS2_SET  9

#define MSP_MOTOR_POS11_SET 10
#define MSP_MOTOR_POS12_SET 11
#define MSP_MOTOR_POS21_SET 12
#define MSP_MOTOR_POS22_SET 13

#define MSP_MOTOR_VEL11_SET 14
#define MSP_MOTOR_VEL12_SET 15
#define MSP_MOTOR_VEL21_SET 16
#define MSP_MOTOR_VEL22_SET 17

#define MSP_MOTOR_ACCEL1_SET 18
#define MSP_MOTOR_ACCEL2_SET 19
#define MSP_MOTOR_DECCEL1_SET 20
#define MSP_MOTOR_DECCEL2_SET 21

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

#define MSP_MOTOR_PARASAVE1_SET  34
#define MSP_MOTOR_PARASAVE2_SET  35

#define MSP_MOTOR_POS13_SET 36
#define MSP_MOTOR_POS23_SET 37
#define MSP_MOTOR_VEL13_SET 38
#define MSP_MOTOR_VEL23_SET 39
#define MSP_MOTOR_ACCEL3_SET 40
#define MSP_MOTOR_DECCEL3_SET 41

#define MSP_MOTOR_FB_ICHI 50
#define MSP_MOTOR_FB_VELO_ACC_DEC_M1 51
#define MSP_MOTOR_FB_VELO_ACC_DEC_M2 52
//mode 3 serial
#define MSP_MOTOR_IDOU_1_SET 60
#define MSP_MOTOR_IDOU_2_SET 61
#define MSP_MOTOR_ACC_DEC_1_SET 62
#define MSP_MOTOR_ACC_DEC_2_SET 63
#define MSP_MOTOR_SPEED_1_SET 64
#define MSP_MOTOR_SPEED_2_SET 65

#define MSP_MOTOR_DISABLE_MOTO_SET 66
#define MSP_MOTOR_RESO_1_SET 67
#define MSP_MOTOR_RESO_2_SET 68
#define MSP_MOTOR_POLE_1_SET 69
#define MSP_MOTOR_POLE_2_SET 70
#define MSP_MOTOR_MODE_RUN 71
//
#define MSP_MOTOR_MOTION_CTRL_1 72
#define MSP_MOTOR_MOTION_CTRL_2 73
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

motor1_resolution_old = EEPROM.read(159);
motor2_resolution_old = EEPROM.read(160);
motor1_pole_old = EEPROM.read(161);
motor2_pole_old = EEPROM.read(162);
control_motion_1_old =  EEPROM.read(163);
control_motion_2_old =  EEPROM.read(164);
  Serial.begin(115200);
  Serial.println(F("START SETUP"));

  Serial.print("motor1_resolution:");
  Serial.println(motor1_resolution_old);
  Serial.print("motor2_resolution:");
  Serial.println(motor2_resolution_old);
  Serial.print("motor1_pole:");
  Serial.println(motor1_pole_old);
  Serial.print("motor2_pole:");
  Serial.println(motor2_pole_old);

  //motor1
  pinMode(xilinda1_1_in, INPUT_PULLUP);
  pinMode(xilinda1_1_out, INPUT_PULLUP);
  pinMode(sensor_1_in, OUTPUT);
  pinMode(sensor_1_out, OUTPUT);
  //motor2
  pinMode(xilinda1_2_in, INPUT_PULLUP);
  pinMode(xilinda1_2_out, INPUT_PULLUP);
  pinMode(sensor_2_in, OUTPUT);
  pinMode(sensor_2_out, OUTPUT);
  //led error
  //led_error_1
  pinMode(led_error_1, OUTPUT);
  pinMode(led_error_2, OUTPUT);
  pinMode(led_error_3, OUTPUT);
  pinMode(led_error_4, OUTPUT);

  //-----------------------STATUS---------------------------
  digitalWrite(sensor_1_in, HIGH);
  digitalWrite(sensor_1_out, HIGH);
  digitalWrite(sensor_2_in, HIGH);
  digitalWrite(sensor_2_out, HIGH);


  digitalWrite(led_error_1, LOW);
  digitalWrite(led_error_2, LOW);
  digitalWrite(led_error_3, LOW);
  digitalWrite(led_error_4, LOW);



  //--------------------------------
  EEPROM.get(89, voltage_limit1);
  EEPROM.get(85, voltage_supply1);
  EEPROM.get(113, voltage_limit2);
  EEPROM.get(109, voltage_supply2);

  // initialise magnetic sensor hardware
  driver1.voltage_limit = voltage_limit1;
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
  sensor2.init();
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = voltage_supply2;
  driver2.init();
  motor2.linkDriver(&driver2);
  // current_sense2.linkDriver(&driver2);

  // choose FOC modulation (optional)
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

 //motor2.foc_modulation= 0x01;

  // set motion control loop to be used
  //motor1.controller =MotionControlType::0x04;
if( control_motion_1_old==0){motor1.controller = MotionControlType::angle;}
else if( control_motion_1_old==1){motor1.controller = MotionControlType::torque;}
else if( control_motion_1_old==2){motor1.controller = MotionControlType::velocity;}
else if( control_motion_1_old==3){motor1.controller = MotionControlType::velocity_openloop;}
else if( control_motion_1_old==4){motor1.controller = MotionControlType::angle_openloop;}
else motor1.controller = MotionControlType::angle;
////
 if( control_motion_2_old==0){motor2.controller = MotionControlType::angle;}
else if( control_motion_2_old==1){motor2.controller = MotionControlType::torque;}
else if( control_motion_2_old==2){motor2.controller = MotionControlType::velocity;}
else if( control_motion_2_old==3){motor2.controller = MotionControlType::velocity_openloop;}
else if( control_motion_2_old==4){motor2.controller = MotionControlType::angle_openloop;}
else motor2.controller = MotionControlType::angle;


  // contoller configuration
  // default parameters in defaults.h
  EEPROM.get(1, moto_pos_x_in);
  EEPROM.get(5, moto_pos_x_out);
  EEPROM.get(9, moto_pos_y_in);
  EEPROM.get(13, moto_pos_y_out);


  EEPROM.get(25, moto_vel_x_in);
  EEPROM.get(29, moto_vel_x_out);
  EEPROM.get(33, moto_vel_y_in);
  EEPROM.get(37, moto_vel_y_out);


  EEPROM.get(49, moto_accel_x);
  EEPROM.get(53, moto_decel_x);
  EEPROM.get(57, moto_accel_y);
  EEPROM.get(61, moto_decel_y);


  EEPROM.get(73, motor1.P_angle.P);
  EEPROM.get(77, motor1.PID_velocity.P);
  EEPROM.get(81, motor1.PID_velocity.I);
  EEPROM.get(93, motor1.current_limit);

  EEPROM.get(97, motor2.P_angle.P);
  EEPROM.get(101, motor2.PID_velocity.P);
  EEPROM.get(105, motor2.PID_velocity.I);
  EEPROM.get(117, motor2.current_limit);


  EEPROM.get(145, motor1.PID_velocity.D);
  EEPROM.get(149, motor2.PID_velocity.D);




  //mode read from eeprom
  MODE_motor = EEPROM.read(0);
  number_motor = EEPROM.read(157);
  digitalWrite(led_error_1, HIGH);

  if (number_motor >= 1) {
    // initialize motor
    motor1.voltage_limit = voltage_limit1;
    motor1.voltage_sensor_align = 4;  //24v-1
    motor1.velocity_index_search = 0.5;
    //current_sense1.init();
    //motor1.linkCurrentSense(&current_sense1);
    motor1.init();

    if( control_motion_1_old!=3&&control_motion_1_old!=4){
    motor1.initFOC();  //0.65, Direction::CW
    }
    Serial.println(motor1.motor_status);
    if (motor1.motor_status ==14){motor1.disable();Serial.println("motor 1 has disabled");error = 1;data_motor1 = motor1.motor_status;}
  }
  digitalWrite(led_error_2, HIGH);
  if (motor1.motor_status !=14) {Serial.println(F("Motor1 ready."));}
  //------------------------------------------------
  if (number_motor > 1) {
    motor2.voltage_limit = voltage_limit2;
    motor2.voltage_sensor_align = 4;  //24v-1
    motor2.velocity_index_search = 0.5;
    //current_sense2.init();
    //motor2.linkCurrentSense(&current_sense2);
    motor2.init();
    //  motor2.initFOC(0.68, Direction::CW);
    if( control_motion_2_old!=3&&control_motion_2_old!=4){
    motor2.initFOC();  //0.65, Direction::CW
    }
    Serial.println(motor2.motor_status);
    if (motor2.motor_status ==14){motor2.disable();Serial.println("motor has 2 disabled");error = 2;data_motor2 = motor2.motor_status;}
    if (motor2.motor_status !=14) {Serial.println(F("Motor2 ready."));}
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
  if (MODE_motor == 2) {

    step_dir1.attach(&motor1.target);
    step_dir2.attach(&motor2.target);

  }


  Serial.println(F(" Motor ready "));
  //  Serial.println(F("Set the target  using Serial terminal:"));
  moto_pos_x_in_old = moto_pos_x_in;
  moto_pos_x_out_old = moto_pos_x_out;
  moto_pos_y_in_old = moto_pos_y_in;
  moto_pos_y_out_old = moto_pos_y_out;

  fuk1 = sensor1.getAngle();
  if (fuk1 > 3.14) {
    motor1.sensor_offset = 6.28;
  }  //5.49 -->//0.79 <--
  if (fuk1 < 3.14) {
    motor1.sensor_offset = 0;
  }  //0.91 <--
  //    Serial.print(motor1.sensor_offset);

  fuk2 = sensor2.getAngle();
  if (fuk2 > 3.14) {
    motor2.sensor_offset = 6.28;
  }  //5.49 -->//0.79 <--
  if (fuk2 < 3.14) {
    motor2.sensor_offset = 0;
  }


  _delay(1000);
  digitalWrite(led_error_4, HIGH);
  //motor_rd =0;
  //motor_ss =1;
  //send_stt_data();
  //delay(400);

  digitalWrite(sensor_1_in, LOW);
  digitalWrite(sensor_2_in, LOW);
  digitalWrite(sensor_1_out, LOW);
  digitalWrite(sensor_2_out, LOW);
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
  driver1.voltage_limit = motor1.voltage_limit;
  driver2.voltage_limit = motor2.voltage_limit;



  if (one_time == 0) {  //genten SPEED
    motor1.P_angle.limit = 5;
    motor2.P_angle.limit = 5;
    if (MODE_motor == 1 || MODE_motor == 2 && number_motor == 1) {
      if (abs(motor1.shaft_angle - 0) <= 0.05) {
        motor1.P_angle.limit = 300;
        one_time = 1;
      }
    }

    if (MODE_motor == 1 || MODE_motor == 2 && number_motor == 2) {
      if ((abs(motor1.shaft_angle - 0) <= 0.05) && (abs(motor2.shaft_angle - moto_pos_y_in) <= 0.05)) {
        motor1.P_angle.limit = 300;
        motor2.P_angle.limit = 300;
        one_time = 1;
      }
    }



    if (MODE_motor == 3 && number_motor == 1) {
      if ((abs(motor1.shaft_angle - 0) <= 0.05) ) {
        motor1.P_angle.limit = 300;
        motor2.P_angle.limit = 300;
        one_time = 1;
      }
    }
    if (MODE_motor == 3 && number_motor == 2) {
      if ((abs(motor1.shaft_angle - 0) <= 0.05) && (abs(motor2.shaft_angle - 0) <= 0.05)) {
        motor1.P_angle.limit = 300;
        motor2.P_angle.limit = 300;
        one_time = 1;
      }
    }
  }
  //defaut setup
  if (defaut_ok == 1) {
    defaut_set();
    defaut_ok = 0;
  }




  // //PLEASE RESET DRIVER TO MODE CHANGE MODE
  // if (MODE_select != 0) {
  //   if (MODE_select == 1) { EEPROM.write(0, 1); }  //PLC IO
  //   if (MODE_select == 2) { EEPROM.write(0, 2); }  //PULSE DIR
  //   if (MODE_select == 3) { EEPROM.write(0, 3); }  //Serial
  //   MODE_select = 0;
  // }

  // //PLEASE RESET DRIVER TO MODE CHANGE NUMBER MOTOR
  // if (number_select != 0) {
  //   if (number_select == 1) { EEPROM.write(157, 1); }  //1
  //   if (number_select == 2) { EEPROM.write(157, 2); }  //2
  //   if (number_select == 3) { EEPROM.write(157, 3); }  //3
  //   number_select = 0;
  // }




  //genten
  if (Pos_run_test == 5) {
    motor1.velocity_limit = 5;
    accell = 10;
    decell = 10;
    moto_pos_x = 0;
    Pos_run_test = 0;
  }
  if (Pos_run_test == 10) {
    motor2.velocity_limit = 5;
    accell = 10;
    decell = 10;
    moto_pos_y = 0;
    Pos_run_test = 0;
  }



  if (MODE_motor == 1) {
    M1_IN_State1 = digitalRead(xilinda1_1_in);
    M1_IN_State2 = digitalRead(xilinda1_1_out);
    M2_IN_State1 = digitalRead(xilinda1_2_in);
    M2_IN_State2 = digitalRead(xilinda1_2_out);

    //*****************************************//genten motor 1***************************************************************
    // if ( (Pos_run_test == 5) || (moto_pos_x_out != moto_pos_x_out_old)) {//genten idou
    //   motor1.velocity_limit = moto_vel_x_out;
    //   accell = moto_accel_x;
    //   decell = moto_decel_x;
    //   if ((moto_pos_x_out >= -100) && (moto_pos_x_out <= 100)) {
    //     moto_pos_x = 0;
    //     moto_pos_x_out_old = 0;
    //   }

    //   //  Serial.println("MOTOR 1 OUT");
    //   TP_pos12 = 0;
    //   Pos_run_test = 0;

    //   if (abs(motor1.shaft_angle - moto_pos_x_out_old) <= 0.05) {
    //     digitalWrite(sensor_1_out, HIGH);
    //   } else {
    //     digitalWrite(sensor_1_out, LOW);
    //   }
    // }
    ///////////////
    if ((M1_IN_State1 == LOW && M1_IN_State2 == HIGH) || (Pos_run_test == 1) || (moto_pos_x_in != moto_pos_x_in_old)) {
      motor1.velocity_limit = moto_vel_x_in;
      accell = moto_accel_x;
      decell = moto_decel_x;
      if ((moto_pos_x_in >= -100) && (moto_pos_x_in <= 100)) {
        moto_pos_x = moto_pos_x_in;
        moto_pos_x_in_old = moto_pos_x_in;
      }

      // Serial.println("MOTOR 1 IN");
      TP_pos11 = 0;
      Pos_run_test = 0;
      if (abs(motor1.shaft_angle - moto_pos_x_in_old) <= 0.05) {
        digitalWrite(sensor_1_in, HIGH);
      } else {
        digitalWrite(sensor_1_in, LOW);
      }
    }
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
    if ((M1_IN_State1 == HIGH && M1_IN_State2 == LOW) || (Pos_run_test == 2) || (moto_pos_x_out != moto_pos_x_out_old)) {
      motor1.velocity_limit = moto_vel_x_out;
      accell = moto_accel_x;
      decell = moto_decel_x;
      if ((moto_pos_x_out >= -100) && (moto_pos_x_out <= 100)) {
        moto_pos_x = moto_pos_x_out;
        moto_pos_x_out_old = moto_pos_x_out;
      }

      //  Serial.println("MOTOR 1 OUT");
      TP_pos12 = 0;
      Pos_run_test = 0;

      if (abs(motor1.shaft_angle - moto_pos_x_out_old) <= 0.05) {
        digitalWrite(sensor_1_out, HIGH);
      } else {
        digitalWrite(sensor_1_out, LOW);
      }
    }
    //******************************//genten motor 2******************************************************************************
// if ((Pos_run_test == 10) || (moto_pos_y_in != moto_pos_y_in_old)) {
//       motor2.velocity_limit = moto_vel_y_in;
//       accell = moto_accel_y;
//       decell = moto_decel_y;
//       if ((moto_pos_y_in >= -100) && (moto_pos_y_in <= 100)) {
//         moto_pos_y = 0;
//         moto_pos_y_in_old = 0;
//       }

//       //  Serial.println(F("MOTOR 2 IN"));
//       Pos_run_test = 0;

//       if (abs(motor2.shaft_angle - moto_pos_y_in_old) <= 0.05) {
//         digitalWrite(sensor_2_in, HIGH);
//       } else {
//         digitalWrite(sensor_2_in, LOW);
//       }
//     }
    ////
    if (M2_IN_State1 == LOW && M2_IN_State2 == HIGH || (Pos_run_test == 11) || (moto_pos_y_in != moto_pos_y_in_old)) {
      motor2.velocity_limit = moto_vel_y_in;
      accell = moto_accel_y;
      decell = moto_decel_y;
      if ((moto_pos_y_in >= -100) && (moto_pos_y_in <= 100)) {
        moto_pos_y = moto_pos_y_in;
        moto_pos_y_in_old = moto_pos_y_in;
      }

      //  Serial.println(F("MOTOR 2 IN"));
      Pos_run_test = 0;

      if (abs(motor2.shaft_angle - moto_pos_y_in_old) <= 0.05) {
        digitalWrite(sensor_2_in, HIGH);
      } else {
        digitalWrite(sensor_2_in, LOW);
      }
    }
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if (M2_IN_State1 == HIGH && M2_IN_State2 == LOW || (Pos_run_test == 12) || (moto_pos_y_out != moto_pos_y_out_old)) {
      motor2.velocity_limit = moto_vel_y_out;
      accell = moto_accel_y;
      decell = moto_decel_y;
      if ((moto_pos_y_out >= -100) && (moto_pos_y_out <= 100)) {
        moto_pos_y = moto_pos_y_out;
        moto_pos_y_out_old = moto_pos_y_out;
      }

      // Serial.println(F("MOTOR 2 OUT"));
      Pos_run_test = 0;

      if (abs(motor2.shaft_angle - moto_pos_y_out_old) <= 0.05) {
        digitalWrite(sensor_2_out, HIGH);
      } else {
        digitalWrite(sensor_2_out, LOW);
      }
    }
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (MODE_motor == 3) {

//now_acceleration_2,decceleration_1
    motor1.velocity_limit = moto_velocity_1_set;
    now_acceleration_1 =   accell_mode3_motor_1;
    now_decceleration_1 = deccell_mode3_motor_1;
    motor2.velocity_limit = moto_velocity_2_set;
    now_acceleration_2  =  accell_mode3_motor_2;
    now_decceleration_2 = deccell_mode3_motor_2;


  }
  //now_velocity_1 =motor1.velocity_limit;
  //now_velocity_2 =motor2.velocity_limit;
if (MODE_motor == 1) {
  limiter1.setVelAccelLimits(motor1.velocity_limit, accell, decell);
  limiter2.setVelAccelLimits(motor2.velocity_limit, accell, decell);
}
if (MODE_motor == 3) {
  limiter1.setVelAccelLimits(motor1.velocity_limit, accell_mode3_motor_1, deccell_mode3_motor_1);
  limiter2.setVelAccelLimits(motor2.velocity_limit, accell_mode3_motor_2, deccell_mode3_motor_2);
}

  motor1.loopFOC();
  motor2.loopFOC();


  //--------------------------------シリンダー MODE---------------------------
  if (MODE_motor == 1) {

    if (one_time == 0) {
      motor1.move(moto_pos_x);
      motor2.move(moto_pos_y);

    } else {
      motor1.move(limiter1.calc(moto_pos_x));
      motor2.move(limiter2.calc(moto_pos_y));
    }
if (motor_dis_val != 1 ){
    if (abs(motor1.shaft_angle - moto_pos_x)>0.1 && timer_int_cut1 == false ){time_over1 = millis();timer_int_cut1 = true; }
    if (millis() - time_over1 > 40000 && timer_int_cut1 == true){ safe_dis1 = true;Serial.println("OVER HEAT"); }
    if (abs(motor1.shaft_angle - moto_pos_x)<=0.1 && timer_int_cut1 == true ){timer_int_cut1 = false; }
}
 if (motor_dis_val != 3 ){
    if (abs(motor2.shaft_angle - moto_pos_y)>0.1 && timer_int_cut2 == false && number_motor > 1 ){time_over2 = millis();timer_int_cut2 = true; }
    if (millis() - time_over2 > 40000 && timer_int_cut2 == true ){ safe_dis2 = true;Serial.println("OVER HEAT"); }
    if (abs(motor2.shaft_angle - moto_pos_y)<=0.1 && timer_int_cut2 == true ){timer_int_cut2 = false; }
    //
    if (safe_dis1 == true && number_motor <= 1){motor1.disable();Serial.println("motor 1 has OVER RUN disabled");error = 3;led_error();}
     if (safe_dis2 == true && number_motor > 1){motor2.disable();Serial.println("motor 2 has OVER RUN disabled");error = 4;led_error();}
}
  }

  //--------------------------------PULSE MODE-------------------------------
  if (MODE_motor == 2) {

    motor1.move();
    motor2.move();
  }

  //-------------------------------------SERIAL MODE--------------------------
  if (MODE_motor == 3) {
    motor1.move(limiter1.calc(moto_1_pos_set));
    motor2.move(limiter2.calc(moto_2_pos_set));
    ////
    if (motor_dis_val != 1 ){
    if (abs(motor1.shaft_angle - moto_1_pos_set)>0.1 && timer_int_cut1 == false ){time_over1 = millis();timer_int_cut1 = true; }
    if (millis() - time_over1 > 40000 && timer_int_cut1 == true){ safe_dis1 = true; }
    if (abs(motor1.shaft_angle - moto_1_pos_set)<=0.1 && timer_int_cut1 == true ){timer_int_cut1 = false; }
    }
    if ( motor_dis_val != 3){
    if (abs(motor2.shaft_angle - moto_2_pos_set)>0.1 && timer_int_cut2 == false && number_motor > 1){time_over2 = millis();timer_int_cut2 = true; }
    if (millis() - time_over2 > 40000 && timer_int_cut2 == true){ safe_dis2 = true; }
    if (abs(motor2.shaft_angle - moto_2_pos_set)<=0.1 && timer_int_cut2 == true ){timer_int_cut2 = false; }
    // //
     if (safe_dis1 == true && number_motor <= 1){motor1.disable();error = 3;led_error();}
     if (safe_dis2 == true && number_motor > 1){motor2.disable();error = 4;led_error();}
    }

//Serial.println(motor1.shaft_angle);
//Serial.println(FloatToUint(motor1.shaft_angle));
//Serial.println(dtostrf(motor1.shaft_angle, 5, 1, m));

  }

}
void led_error(){
     unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(PB9, ledState);
    digitalWrite(PB10, !ledState);
  }
    
    

}

void error_reset_on(){
if (error_reset==1){
error = 0;
safe_dis1 = false;
safe_dis2 = false;
error_reset =0;
timer_int_cut1 = false;
timer_int_cut2 = false;
time_over1 = millis();
time_over2 = millis();
}


}
