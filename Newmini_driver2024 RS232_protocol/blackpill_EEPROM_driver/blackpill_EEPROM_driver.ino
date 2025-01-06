   
#include <EEPROM.h>
//HardwareSerial Serial1(PA3, PA2);
int mode =1;
int number_motor =2;
float moto_pos_x_in = 0.00f;
float moto_pos_x_out =6.28f;
float moto_pos_y_in = 0.00f;
float moto_pos_y_out =6.28f;
float moto_pos_x_naka = 3.14f;
float moto_pos_y_naka =3.14f;
float moto_vel_x_in =50.00f;
float moto_vel_x_out=50.00f;
float moto_vel_y_in=50.00f;
float moto_vel_y_out=50.00f;
float moto_vel_x_naka=50.00f;
float moto_vel_y_naka=50.00f;
float moto_accel_x =100.00f;
float moto_decel_x =100.00f;
float moto_accel_y=100.00f;
float moto_decel_y=100.00f;
float moto_accel_z=100.00f;
float moto_decel_z=100.00f;
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
float pid_pos_3 =20.00f;
float pid_vel_p_3 =0.25f;
float pid_vel_i_3 =15.00f;
float vol_input_3 = 24.00f;
float vol_limit_3 =15.00f;
float ampe_limit_3 =2.00f;
float pid_vd_1 =0.001f;
float pid_vd_2 =0.001f;
float pid_vd_3 =0.001f;
uint8_t motor1_resolution=8;//x100 =800 pulse
uint8_t motor2_resolution=8;//x100 =800 pulse
uint8_t motor1_pole=11;
uint8_t motor2_pole=11;
uint8_t control_motion_1=0;
uint8_t control_motion_2=0;

float a,b,c;

void setup() { 

  Serial.begin(115200);
  delay(500);
  pinMode(PB9, OUTPUT);
  pinMode(PB10, OUTPUT);
//  for (int i = 0 ; i < EEPROM.length() ; i++) {
//    EEPROM.write(i, 0);
//  }
  digitalWrite(PB9, LOW);
  digitalWrite(PB10, LOW);
  delay(500);
   Serial.print("start");
       
    EEPROM.put(1, moto_pos_x_in);//moto_pos_x_in
    EEPROM.put(5, moto_pos_x_out);//moto_pos_x_out
    EEPROM.put(9, moto_pos_y_in);//moto_pos_y_in
    EEPROM.put(13, moto_pos_y_out);//moto_pos_y_out
    EEPROM.put(17, moto_pos_x_naka);//moto_pos_x_naka
    EEPROM.put(21, moto_pos_y_naka);//moto_pos_y_naka
    EEPROM.put(25, moto_vel_x_in);//moto_vel_x_in
    EEPROM.put(29, moto_vel_x_out);//moto_vel_x_out
    EEPROM.put(33, moto_vel_y_in);//moto_vel_y_in
    EEPROM.put(37, moto_vel_y_out);//moto_vel_y_out
    EEPROM.put(41, moto_vel_x_naka);//moto_vel_x_naka
    EEPROM.put(45, moto_vel_y_naka);//moto_vel_y_naka
    EEPROM.put(49, moto_accel_x);//moto_accel_x
    EEPROM.put(53, moto_decel_x);//moto_decel_x
    EEPROM.put(57, moto_accel_y);//moto_accel_y
    EEPROM.put(61, moto_decel_y);//moto_decel_y
    EEPROM.put(65, moto_accel_z);//moto_accel_z
    EEPROM.put(69, moto_decel_z);//moto_decel_z
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
    EEPROM.put(121, pid_pos_3);//pid pos
    EEPROM.put(125, pid_vel_p_3);//pid vel p
    EEPROM.put(129, pid_vel_i_3);//pid vel i
    EEPROM.put(133, vol_input_3);//vol input
    EEPROM.put(137, vol_limit_3);//vol limit
    EEPROM.put(141, ampe_limit_3);//ampe limit
    EEPROM.put(145, pid_vd_1);//pid vd1
    EEPROM.put(149, pid_vd_2);//pid_vd2
    EEPROM.put(153, pid_vd_3);//pid_vd3
    EEPROM.write(157,number_motor);
    EEPROM.write(158,mode);
    EEPROM.write(159,motor1_resolution);
    EEPROM.write(160,motor2_resolution);
    EEPROM.write(161,motor1_pole);
    EEPROM.write(162,motor2_pole);
     EEPROM.write(163,control_motion_1);
     EEPROM.write(164,control_motion_2);
 delay(500);
   digitalWrite(PB9, HIGH);
  digitalWrite(PB10, HIGH);
  Serial.print("OK");

EEPROM.get(85,a);
EEPROM.get(89,b);
EEPROM.get(109,c);

Serial.print(a);
Serial.print(b);
Serial.print(c);
  
}

void loop() {
   digitalWrite(PB9, HIGH);
   digitalWrite(PB10, LOW);
  delay(100);
  
  digitalWrite(PB10, HIGH);
  digitalWrite(PB9, LOW);
  delay(100);
 // Serial1.print(EEPROM.read(0));
  
  }
