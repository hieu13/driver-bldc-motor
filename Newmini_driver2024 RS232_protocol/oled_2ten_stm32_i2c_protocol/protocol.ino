void messenger_siji(){

       
       
       u8g2.clearDisplay();

       u8g2.setFont(u8g2_font_unifont_t_symbols);
       u8g2.drawGlyph(10, 20, 0x2716);  
       u8g2.setFont(u8g_font_7x14);
       if (error==1){u8g2.drawStr(40, 20, "MOTOR 1");}
       if (error==2 && cur_number_motor ==2){u8g2.drawStr(40, 20, "MOTOR 2");}
       u8g2.drawStr(30, 35, "CALIB ERROR"); 
       u8g2.drawStr(15, 50, "PLEASE RESTART"); 
      //
    //  u8g2.setFont(u8g2_font_helvR08_tr);
    //  u8g2.drawButtonUTF8(62, 20, U8G2_BTN_SHADOW1|U8G2_BTN_HCENTER|U8G2_BTN_BW2, 34,  2,  2, "OK" );
       u8g2.sendBuffer();
      delay(3000);

}

void messenger_err(){
u8g2.clearDisplay();

       u8g2.setFont(u8g2_font_unifont_t_symbols);
       u8g2.drawGlyph(10, 20, 0x2716);  
       u8g2.setFont(u8g_font_7x14);
       if (error ==3){u8g2.drawStr(40, 20, "MOTOR 1");}
       if (error ==4){u8g2.drawStr(40, 20, "MOTOR 2");}
       u8g2.drawStr(30, 35, "OVER ERROR"); 
       u8g2.drawStr(15, 50, "PLEASE RESTART"); 
      //
    //  u8g2.setFont(u8g2_font_helvR08_tr);
    //  u8g2.drawButtonUTF8(62, 20, U8G2_BTN_SHADOW1|U8G2_BTN_HCENTER|U8G2_BTN_BW2, 34,  2,  2, "OK" );
       u8g2.sendBuffer();
      delay(3000);
      current_screen=99;


}
void messenger_alert(){
    u8g2.setFont(u8g_font_7x14);
  u8g2.drawStr(25, 12, "ACCESS_DENY:");
  u8g2.drawStr(15, 25, "CURRENT VALUE"); 
  u8g2.drawStr(35, 38, "NOT READ"); 
  u8g2.drawStr(45, 51, "<!!!>"); 
  u8g2.sendBuffer();
  delay(2000);
  u8g2.clearDisplay();
}



void messenger_save(){

       
       
       u8g2.clearDisplay();

       u8g2.setFont(u8g2_font_unifont_t_symbols);
       u8g2.drawGlyph(10, 20, 0x2716);  
       u8g2.setFont(u8g_font_7x14);
       if (error==1){u8g2.drawStr(40, 20, "MOTOR 1");}
       if (error==2 && cur_number_motor ==2){u8g2.drawStr(40, 20, "MOTOR 2");}
       u8g2.drawStr(30, 35, "SAVED OK"); 
       u8g2.drawStr(15, 50, "PLEASE WAIT"); 
      //
    //  u8g2.setFont(u8g2_font_helvR08_tr);
    //  u8g2.drawButtonUTF8(62, 20, U8G2_BTN_SHADOW1|U8G2_BTN_HCENTER|U8G2_BTN_BW2, 34,  2,  2, "OK" );
       u8g2.sendBuffer();
      delay(3000);

}

void evaluateCommand(uint8_t c) {
  uint32_t tmp=0; 

  switch(c) {
    // adding this message as a comment will return an error status for MSP_PRIVATE (end of switch), allowing third party tools to distinguish the implementation of this message
    //case MSP_PRIVATE:
    //  headSerialError();tailSerialReply(); // we don't have any custom msp currently, so tell the gui we do not use that
    //  break;
    case MSP_MOTOR_MODE:
      //mspAck();
      error=read8();
      cur_mode=read8();
      cur_number_motor=read8();

      motor1_resolution=read8();
      motor2_resolution=read8();
      motor1_pole=read8();
      motor2_pole=read8();
      //
      data_motor_1=read8();
      data_motor_2=read8();
      control_motion_1=read8();
      control_motion_2=read8();
      break;
    case MSP_MOTOR_POS:
      cur_pos11 = UintToFloat(read32());
      cur_pos12 = UintToFloat(read32());
      cur_pos21 = UintToFloat(read32());
      cur_pos22 = UintToFloat(read32());
      break;
    case MSP_MOTOR_VEL:
      cur_vel11 = read16();
      cur_vel12 = read16();
      cur_vel21 = read16();
      cur_vel22 = read16();
      break;
    case MSP_MOTOR_ACCEL_DECCEL:
      cur_accel1 = read16();
      cur_accel2 = read16();
      cur_decel1 = read16();
      cur_decel2 = read16();
      break;
        case MSP_MOTOR_VOL:
      cur_volIN1 = UintToFloat(read32());
      cur_volIN2 = UintToFloat(read32());
      break;
              case MSP_MOTOR_VOLLIMIT:
      cur_volLIMIT1 = UintToFloat(read32());
      cur_volLIMIT2 = UintToFloat(read32());
      break;
    case MSP_MOTOR_PID_P:
      cur_posP1 = UintToFloat(read32());
      cur_posP2 = UintToFloat(read32());
      break;
          case MSP_MOTOR_PID_VEL_P:
      cur_velP1 = UintToFloat(read32());
      cur_velP2 = UintToFloat(read32());
      break;
      case MSP_MOTOR_PID_VEL_I:
      cur_velI1 = UintToFloat(read32());
      cur_velI2 = UintToFloat(read32());
      break;
      case MSP_MOTOR_PID_VEL_D:
      cur_velD1 = UintToFloat(read32());
      cur_velD2 = UintToFloat(read32());
      break;
  }
}

void protocol_send_cmd(void){

//   static unsigned long loop_protocol_1_prev = 0;
//   static unsigned long loop_protocol_2_prev = 0;
//   static unsigned long loop_protocol_3_prev = 0;
//   //Serial.println("OK");
// unsigned long loop_protocol_1 = millis() - loop_protocol_1_prev;
//     if (loop_protocol_1 >= 100) {
//       loop_protocol_1_prev = millis();
if (i_cout ==0){
      cmdMSP = MSP_MOTOR_POS;
      mspAck();
      
      cmdMSP = MSP_MOTOR_VOL;
      mspAck();
     
      cmdMSP = MSP_MOTOR_PID_VEL_P;
      mspAck();
}
      /////////////
if (i_cout ==1){
      cmdMSP = MSP_MOTOR_VEL;
      mspAck();
      
      cmdMSP = MSP_MOTOR_VOLLIMIT;
      mspAck();
      
      cmdMSP = MSP_MOTOR_PID_P;
      mspAck();
}
      ////////////////////
if (i_cout ==2){
      cmdMSP = MSP_MOTOR_PID_VEL_D;
      mspAck();
      cmdMSP = MSP_MOTOR_ACCEL_DECCEL;
      mspAck();
     
}
if (i_cout ==3){
      cmdMSP = MSP_MOTOR_PID_VEL_I;
      mspAck();
}
if (i_cout ==4){
      cmdMSP = MSP_MOTOR_MODE;
      mspAck();

}
 i_cout+=1;
 if (i_cout >4)i_cout=0;
    // } else {
    //   return;
//    }


// unsigned long loop_protocol_2 = millis() - loop_protocol_2_prev;
//     if (loop_protocol_2 >= 150) {
//       loop_protocol_2_prev = millis();
      
//     } else {
//       return;
//     }



// unsigned long loop_protocol_3 = millis() - loop_protocol_3_prev;
//     if (loop_protocol_3 >= 200) {
//       loop_protocol_3_prev = millis();
      
//     } else {
//       return;
//     }

}