void defaut_set() {

  EEPROM.write(1, MODE_motor);
  EEPROM.put(1, x_in);    //moto_pos_x_in
  EEPROM.put(5, x_out);   //moto_pos_x_out
  EEPROM.put(9, y_in);    //moto_pos_y_in
  EEPROM.put(13, y_out);  //moto_pos_y_out

  EEPROM.put(25, vel_x_in);   //moto_vel_x_in
  EEPROM.put(29, vel_x_out);  //moto_vel_x_out
  EEPROM.put(33, vel_y_in);   //moto_vel_y_in
  EEPROM.put(37, vel_y_out);  //moto_vel_y_out

  EEPROM.put(49, moto_accel_x);  //moto_accel_x
  EEPROM.put(53, moto_decel_x);  //moto_decel_x
  EEPROM.put(57, moto_accel_y);  //moto_accel_y
  EEPROM.put(61, moto_decel_y);  //moto_decel_y

  EEPROM.put(73, pid_pos_1);      //pid pos
  EEPROM.put(77, pid_vel_p_1);    //pid vel p
  EEPROM.put(81, pid_vel_i_1);    //pid vel i
  EEPROM.put(85, vol_input_1);    //vol input
  EEPROM.put(89, vol_limit_1);    //vol limit
  EEPROM.put(93, ampe_limit_1);   //ampe limit
  EEPROM.put(97, pid_pos_2);      //pid pos
  EEPROM.put(101, pid_vel_p_2);   //pid vel p
  EEPROM.put(105, pid_vel_i_2);   //pid vel i
  EEPROM.put(109, vol_input_2);   //vol input
  EEPROM.put(113, vol_limit_2);   //vol limit
  EEPROM.put(117, ampe_limit_2);  //ampe limit

  EEPROM.put(145, pid_vd_1);  //pid vd1
  EEPROM.put(149, pid_vd_2);  //pid_vd2

  EEPROM.write(157, 2);  //number_motor
  EEPROM.write(159,motor1_resolution_epp);
  EEPROM.write(160,motor2_resolution_epp);
  EEPROM.write(161,motor1_pole_epp);
  EEPROM.write(162,motor2_pole_epp);


  Serial.println("defaut_ok");
  while (1) {
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



