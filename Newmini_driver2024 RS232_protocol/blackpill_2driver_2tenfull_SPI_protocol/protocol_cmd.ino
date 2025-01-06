


void evaluateCommand(uint8_t c) {
  switch (c) {
    
    case MSP_MOTOR_MODE:
      data_motor1 = motor1.motor_status;
      data_motor2 = motor2.motor_status;
      headSerialReply(11);
      serialize8(error);
      serialize8(MODE_motor);
      serialize8(number_motor);
      
      serialize8(motor1_resolution_old);
      serialize8(motor2_resolution_old);
      serialize8(motor1_pole_old);
      serialize8(motor2_pole_old);

      serialize8(data_motor1);
      serialize8(data_motor2);
      serialize8(control_motion_1_old);
      serialize8(control_motion_2_old);
      tailSerialReply();
      break;
    case MSP_MOTOR_DATAPOS1_SET:
      moto_pos_x_in = UintToFloat(read32());
      moto_pos_x_out = UintToFloat(read32());
      moto_vel_x_in =read16();
      moto_vel_x_out =read16();
      moto_accel_x =read16();
      moto_decel_x =read16();

      break;
    case MSP_MOTOR_DATAPOS2_SET:
      moto_pos_y_in = UintToFloat(read32());
      moto_pos_y_out = UintToFloat(read32());
      moto_vel_y_in =read16();
      moto_vel_y_out =read16();
      moto_accel_y =read16();
      moto_decel_y =read16();

      break;
    case MSP_MOTOR_MODE_SET:
      MODE_select = read8();
      error_rs =read8();
      number_select = read8();
      nc =read8();
      motor1_resolution =read8();
      motor2_resolution =read8();
      motor1_pole = read8();
      motor2_pole = read8();
      control_motion_1 = read8();
      control_motion_2 = read8();

            //PLEASE RESET DRIVER TO MODE CHANGE MODE
        if (MODE_select != 0) {
          if (MODE_select == 1) { EEPROM.write(0, 1); }  //PLC IO
          if (MODE_select == 2) { EEPROM.write(0, 2); }  //PULSE DIR
          if (MODE_select == 3) { EEPROM.write(0, 3); }  //Serial
          MODE_select = 0;
        }
        //PLEASE RESET DRIVER TO MODE CHANGE NUMBER MOTOR
        if (number_select != 0) {
          if (number_select == 1) { EEPROM.write(157, 1); }  //1
          if (number_select == 2) { EEPROM.write(157, 2); }  //2
         // if (number_select == 3) { EEPROM.write(157, 3); }  //3
          number_select = 0;
        }
        if (motor1_resolution !=motor1_resolution_old && motor1_resolution > 0 && motor1_resolution <255 ) {
          EEPROM.write(159, motor1_resolution);
        }
        if (motor2_resolution !=motor2_resolution_old && motor2_resolution > 0 && motor2_resolution <255 ) {
          EEPROM.write(160, motor2_resolution);
        }
        if (motor1_pole != motor1_pole_old && motor1_pole > 0 && motor1_pole <200 ) {
          EEPROM.write(161, motor1_pole);
        }
        if (motor2_pole != motor2_pole_old && motor2_pole > 0 && motor2_pole <200 ) {
          EEPROM.write(162, motor2_pole);
        }
        if (control_motion_1 != control_motion_1_old && control_motion_1 >= 0 && control_motion_1 <7 ) {
          EEPROM.write(163, control_motion_1);
        }
        if (control_motion_2 != control_motion_2_old && control_motion_2 >= 0 && control_motion_2 <7 ) {
          EEPROM.write(164, control_motion_2);}
          
            digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);
      break;
case MSP_MOTOR_MODE_RUN:
MODE_select = read8();
if (MODE_select != 0) {
          if (MODE_select == 1) { EEPROM.write(0, 1); }  //PLC IO
          if (MODE_select == 2) { EEPROM.write(0, 2); }  //PULSE DIR
          if (MODE_select == 3) { EEPROM.write(0, 3); }  //Serial
          MODE_select = 0;
        }
     digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);   
break;

    case MSP_MOTOR_MOTOR_SET:
     number_select = read8();
  if (number_select != 0) {
          if (number_select == 1) { EEPROM.write(157, 1); }  //1
          if (number_select == 2) { EEPROM.write(157, 2); }  //2
         // if (number_select == 3) { EEPROM.write(157, 3); }  //3
          number_select = 0;
        }
    digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);
   
    break;
    case MSP_MOTOR_RESO_1_SET:
    motor1_resolution =read8();
    if (motor1_resolution !=motor1_resolution_old && motor1_resolution > 0 && motor1_resolution <255 ) {
          EEPROM.write(159, motor1_resolution);
        }


    break;
     case MSP_MOTOR_RESO_2_SET:
    motor2_resolution =read8();
    if (motor2_resolution !=motor2_resolution_old && motor2_resolution > 0 && motor2_resolution <255 ) {
          EEPROM.write(160, motor2_resolution);
        }
        
    break;

    case MSP_MOTOR_POLE_1_SET:
      if (motor1_pole != motor1_pole_old && motor1_pole > 0 && motor1_pole <200 ) {
          EEPROM.write(161, motor1_pole);
        }
 
    break;
    case MSP_MOTOR_POLE_2_SET:
        if (motor2_pole != motor2_pole_old && motor2_pole > 0 && motor2_pole <200 ) {
          EEPROM.write(162, motor2_pole);
        }
  
    break;
    case MSP_MOTOR_MOTION_CTRL_1:
    control_motion_1 =read8();
      if (control_motion_1 != control_motion_1_old && control_motion_1 >= 0 && control_motion_1 <7 ) {
          EEPROM.write(163, control_motion_1);
        }
        digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);
    break;

    case MSP_MOTOR_MOTION_CTRL_2:
    control_motion_2 =read8();
      if (control_motion_2 != control_motion_2_old && control_motion_2 >= 0 && control_motion_2 <7 ) {
          EEPROM.write(164, control_motion_2);}
          digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);
        
    break;
    case MSP_MOTOR_SAVE_SET:

      save_select = read8();
      saving = save_select;
  if (saving == 1) {
            digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
    EEPROM.put(1, moto_pos_x_in);
    delay(5);
    //    EEPROM.put(17, moto_pos_x_naka);
    //    delay(10);
    EEPROM.put(5, moto_pos_x_out);
    delay(5);
    EEPROM.put(25, moto_vel_x_in);
    delay(5);
    //    EEPROM.put(41, moto_vel_x_naka);
    //    delay(10);
    EEPROM.put(29, moto_vel_x_out);
    delay(5);
    EEPROM.put(49, moto_accel_x);
    delay(5);
    EEPROM.put(53, moto_decel_x);
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);

    save_select = 0;
  }
  if (saving == 2) {

            digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
    EEPROM.put(9, moto_pos_y_in);
    delay(5);
    //    EEPROM.put(21, moto_pos_y_naka);
    //    delay(10);
    EEPROM.put(13, moto_pos_y_out);
    delay(5);
    EEPROM.put(33, moto_vel_y_in);
    delay(5);
    //    EEPROM.put(45, moto_vel_y_naka);
    //    delay(10);
    EEPROM.put(37, moto_vel_y_out);
    delay(5);
    EEPROM.put(57, moto_accel_y);
    delay(5);
    EEPROM.put(61, moto_decel_y);
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);
    

    save_select = 0;
  }

    if (saving == 3) {
            digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
    EEPROM.put(73, motor1.P_angle.P);  //pid pos
    delay(5);
    EEPROM.put(77, motor1.PID_velocity.P);  //pid vel p
    delay(5);
    EEPROM.put(81, motor1.PID_velocity.I);  //pid vel i
    delay(5);
    EEPROM.put(85, driver1.voltage_power_supply);  //vol input
    delay(5);
    EEPROM.put(89, motor1.voltage_limit);  //vol limit
    delay(5);
    EEPROM.put(93, motor1.current_limit);  //ampe limit
    delay(5);
    EEPROM.put(145, motor1.PID_velocity.D);  //pid vd
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);
    save_select = 0;
    }
   if (saving == 4) {
            digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
    EEPROM.put(97, motor2.P_angle.P);  //pid pos
    delay(5);
    EEPROM.put(101, motor2.PID_velocity.P);  //pid vel p
    delay(5);
    EEPROM.put(105, motor2.PID_velocity.I);  //pid vel i
    delay(5);
    EEPROM.put(109, driver2.voltage_power_supply);  //vol input
    delay(5);
    EEPROM.put(113, motor2.voltage_limit);  //vol limit
    delay(5);
    EEPROM.put(117, motor2.current_limit);  //ampe limit
    delay(5);
    EEPROM.put(149, motor2.PID_velocity.D);  //ampe limit
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);
save_select = 0;
   }
//
if (saving == 5) {  
            

      if (motor1_resolution !=motor1_resolution_old && motor1_resolution > 0 && motor1_resolution <255 ) {
          EEPROM.write(159, motor1_resolution);
        }
        if (motor2_resolution !=motor2_resolution_old && motor2_resolution > 0 && motor2_resolution <255 ) {
          EEPROM.write(160, motor2_resolution);
        }
        if (motor1_pole != motor1_pole_old && motor1_pole > 0 && motor1_pole <200 ) {
          EEPROM.write(161, motor1_pole);
        }
        if (motor2_pole != motor2_pole_old && motor2_pole > 0 && motor2_pole <200 ) {
          EEPROM.write(162, motor2_pole);
        }
            digitalWrite(PB10, LOW);
            digitalWrite(PB9, LOW);
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);
            delay(200);
            digitalWrite(PB10, HIGH);
            digitalWrite(PB9, HIGH);
save_select = 0;
   }


      
      break;
    case MSP_MOTOR_RESET_SET:
      defaut_ok = read8();
      break;
    case MSP_MOTOR_RUN_SET:
      Pos_run_test = read8();
      break;
    case MSP_MOTOR_ERR_SET:
      error_reset = read8();
      error_reset_on();
      break;
    case MSP_MOTOR_NC_SET:
      // = read8();
      break;

    case MSP_MOTOR_PARASAVE1_SET:
      motor1.P_angle.P = UintToFloat(read32());
      motor1.PID_velocity.P = UintToFloat(read32());
      motor1.PID_velocity.I = UintToFloat(read32());
      motor1.PID_velocity.D = UintToFloat(read32());
      driver1.voltage_power_supply = UintToFloat(read32());
      motor1.voltage_limit = UintToFloat(read32());
      break;
    case MSP_MOTOR_PARASAVE2_SET:
      motor2.P_angle.P = UintToFloat(read32());
      motor2.PID_velocity.P = UintToFloat(read32());
      motor2.PID_velocity.I = UintToFloat(read32());
      motor2.PID_velocity.D = UintToFloat(read32());
      driver2.voltage_power_supply = UintToFloat(read32());
      motor2.voltage_limit = UintToFloat(read32());
      break;

    case MSP_MOTOR_POS:
      headSerialReply(16);
      serialize32(FloatToUint(moto_pos_x_in));
      serialize32(FloatToUint(moto_pos_x_out));
      serialize32(FloatToUint(moto_pos_y_in));
      serialize32(FloatToUint(moto_pos_y_out));
      tailSerialReply();
      break;
    case MSP_MOTOR_POS11_SET://Pos_run_test==1
      moto_pos_x_in = UintToFloat(read32());//moto_pos_x_in
      break;
    case MSP_MOTOR_POS12_SET:
      moto_pos_x_out = UintToFloat(read32());//moto_pos_x_in
      break;
    case MSP_MOTOR_POS21_SET:
      moto_pos_y_in = UintToFloat(read32());//moto_pos_x_in
      break;
    case MSP_MOTOR_POS22_SET:
      moto_pos_y_out = UintToFloat(read32());//moto_pos_x_in
      break;

    case MSP_MOTOR_VEL:
      headSerialReply(8);
      serialize16(moto_vel_x_in);
      serialize16(moto_vel_x_out);
      serialize16(moto_vel_y_in);
      serialize16(moto_vel_y_out);
      tailSerialReply();
      break;
    case MSP_MOTOR_VEL11_SET:
      moto_vel_x_in = read16();
      break;
    case MSP_MOTOR_VEL12_SET:
      moto_vel_x_out = read16();
      break;
    case MSP_MOTOR_VEL21_SET:
      moto_vel_y_in = read16();
      break;
    case MSP_MOTOR_VEL22_SET:
      moto_vel_y_out = read16();
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
    case MSP_MOTOR_FB_ICHI:
      headSerialReply(8);

      serialize32(FloatToUint(round(100*motor1.shaft_angle)));
      //serialize32(FloatToUint(motor2.shaft_angle));
      serialize32(FloatToUint(round(100*motor2.shaft_angle)));
      tailSerialReply();
      

      break;
    case MSP_MOTOR_FB_VELO_ACC_DEC_M1:
      headSerialReply(6);
      serialize16(round(motor1.velocity_limit));
      serialize16(round(now_acceleration_1));
      serialize16(round(now_decceleration_1));
      tailSerialReply();
      break;
    case MSP_MOTOR_FB_VELO_ACC_DEC_M2:
      headSerialReply(6);
      serialize16(round(motor2.velocity_limit));
      serialize16(round(now_acceleration_2));
      serialize16(round(now_decceleration_2));
      tailSerialReply();
      break;
    case MSP_MOTOR_IDOU_1_SET:
        moto_velocity_1_set = read16();
        accell_mode3_motor_1 =read16();
        deccell_mode3_motor_1=read16();
        moto_1_pos_set =0.01*UintToFloat(read32());

      break;  
    case MSP_MOTOR_IDOU_2_SET:
        moto_velocity_2_set = read16();
        accell_mode3_motor_2 =read16();
        deccell_mode3_motor_2=read16();
        moto_2_pos_set =0.01*UintToFloat(read32());

      break;  
      
    case MSP_MOTOR_DISABLE_MOTO_SET:
        motor_dis_val=read8();
        if (motor_dis_val==1){motor1.disable();}
        if (motor_dis_val==2){motor1.enable();}
        //
        if (motor_dis_val==3){motor2.disable();}
        if (motor_dis_val==4){motor2.enable();}
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




uint32_t FloatToUint(float n) {
  return (uint32_t)(*(uint32_t*)&n);
}

float UintToFloat(uint32_t n) {
  return (float)(*(float*)&n);
}
