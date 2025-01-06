void menuCheck() {

 /////////////
        if (current_screen != 49&&current_screen != 50&&current_screen != 51&&current_screen != 52&&current_screen != 53&&current_screen != 54){
        if (tani ==1){ u8g2.drawStr(89, 12, "x0.01");donvi=0.01; 
                       u8g2.drawStr(89, 12, "x0.0_");donvi=0.01; 
        }
        if (tani ==2){ u8g2.drawStr(89, 12, "x0.10");donvi=0.1;
                       u8g2.drawStr(89, 12, "x0._0");
        }
        if (tani ==3){ u8g2.drawStr(89, 12, "x1.00");donvi=1; 
                       u8g2.drawStr(89, 12, "x_.00");
        }}
    ///////////
 if ((lock != 1)&&(lock_tani !=1)){
    if ((Kaiten == 1)&& (huong == 1)){menuCount = menuCount -1;}
    if ((Kaiten == 1)&& (huong == -1)){menuCount = menuCount +1;}  
    if(menuCount <= 1) { menuCount =1;}
    if(menuCount > 5)  {menuCount =5;}
 }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if ((menuCount == 1)&&(clicked_tani == 0)&&current_screen !=  53 && current_screen !=  54) 
  {
  uint8_t btn8 = digitalRead(BUTTON_SELECT_PIN);
    if (btn8 == LOW && btn_prev8 == HIGH)
  {
       lock_tani=1;
       clicked_tani =1;
       delay(50);
  }
  btn_prev8 = digitalRead(BUTTON_SELECT_PIN);
  }
//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
  if (lock_tani == 1){
  uint8_t btn9 = digitalRead(BUTTON_SELECT_PIN);
    if (btn9 == LOW && btn_prev9 == HIGH)
  {
       lock_tani =0;
       clicked_tani =0;
  }
  btn_prev9 = digitalRead(BUTTON_SELECT_PIN);
  
     if (current_screen==49||current_screen==50||current_screen==51||current_screen==52){u8g2.drawFrame(9,0, 119, 13);   }
     else u8g2.drawFrame(9,0, 79, 13); 

       if ((Kaiten == 1)&& (huong == 1)) {tani--;}
       if ((Kaiten == 1)&& (huong == -1)){tani++;}  
       if(tani >= 3) { tani =3;}
       if(tani <= 1)  {tani =1;}
        if (tani ==1){donvi=0.01;i=0 ; }
        if (tani ==2){donvi=0.1;i=0 ; }
        if (tani ==3){donvi=1;i=0 ;}
        
    }
 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Serial.print(clicked);Serial.print("-");Serial.print(current_screen);Serial.print("-");Serial.println(menuCount);
  if ((menuCount == 3)&&(clicked == 0)){
    
  uint8_t btn10 = digitalRead(BUTTON_SELECT_PIN);
    if (btn10 == LOW && btn_prev10 == HIGH)
  {
   lock =1;
   clicked =1;
   pos11 = cur_pos11 ;pos12 = cur_pos12 ;vel11 = cur_vel11 ;vel12 = cur_vel12 ;
   accel1= cur_accel1;decel1= cur_decel1;posP1 = cur_posP1;velP1 = cur_velP1;velI1 = cur_velI1;
   volIN1= cur_volIN1;volLIMIT1=cur_volLIMIT1;amp_limit1=cur_AMP1;velD1 = cur_velD1;
   //------------------------
   pos21 = cur_pos21 ;pos22 = cur_pos22 ;vel21 = cur_vel21 ;vel22 = cur_vel22 ;
   accel2= cur_accel2;decel2= cur_decel2;posP2 = cur_posP2;velP2 = cur_velP2;velI2 = cur_velI2;
   volIN2= cur_volIN2;volLIMIT2=cur_volLIMIT2;amp_limit2=cur_AMP2;velD2 = cur_velD2;
   //-----------------------------
    resolution_1_set =motor1_resolution;resolution_2_set =motor2_resolution;
    pole_1_set= motor1_pole;pole_2_set= motor2_pole;
  }
  delay(50);
  btn_prev10 = digitalRead(BUTTON_SELECT_PIN);
  }

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
  if (lock == 1){

  uint8_t btn11 = digitalRead(BUTTON_SELECT_PIN);
  if (btn11 == LOW && btn_prev11 == HIGH)
  {
       lock =0;
       clicked =0;
  }
  btn_prev11 = digitalRead(BUTTON_SELECT_PIN);

  if (current_screen !=  53 || current_screen !=  54){u8g2.drawFrame(9,26, 63, 13);}
      //volum increse 
    if ((Kaiten == 1)&& (huong == 1)) {
       ii= 1;
       saving =0;
      switch(current_screen)
    {
      case 6:
                                pos11 = pos11 +ii*donvi;i=0;
                                if(pos11 >= 100) { pos11 =100;}
                                cmdMSP = MSP_MOTOR_POS11_SET;headSerialReply(4);serialize32(FloatToUint(pos11));tailSerialReply();break;
      case 7 : 
                                pos12 = pos12 +ii*donvi;i=0;
                                if(pos12 >= 100) { pos12 =100;}
                                cmdMSP = MSP_MOTOR_POS12_SET;headSerialReply(4);serialize32(FloatToUint(pos12));tailSerialReply();break;
      case 8 :
                                vel11 = vel11 +ii*donvi*100;i=0;
                                if(vel11 >= 1000) { vel11 =1000;}
                                cmdMSP = MSP_MOTOR_VEL11_SET;headSerialReply(2);serialize16(vel11);tailSerialReply();break;
      case 9 :
                                vel12 = vel12 +ii*donvi*100;i=0;
                                if(vel12 >= 1000) { vel12 =1000;}
                                cmdMSP = MSP_MOTOR_VEL12_SET;headSerialReply(2);serialize16(vel12);tailSerialReply();break;
      case 10 : 
                                accel1 = accel1 +ii*donvi*100;i=0;
                                if(accel1 >= 2000) { accel1 =2000;}
                                cmdMSP = MSP_MOTOR_ACCEL1_SET;headSerialReply(2);serialize16(accel1);tailSerialReply();break;
       case 11 :
                                decel1 = decel1 +ii*donvi*100;i=0;
                                if(decel1 >= 2000) { decel1 =2000;}
                                cmdMSP = MSP_MOTOR_DECCEL1_SET;headSerialReply(2);serialize16(decel1);tailSerialReply();break;
       case 12 :
                                posP1 = posP1 +ii*donvi;i=0; 
                                if(posP1 >= 500) { posP1 =500;}
                                cmdMSP = MSP_MOTOR_PID_P_M1_SET;headSerialReply(4);serialize32(FloatToUint(posP1));tailSerialReply();break;
       case 13 :
                                velP1 = velP1 +ii*donvi;i=0;
                                if(velP1 >= 500) { velP1 =500;}
                                cmdMSP = MSP_MOTOR_PID_VEL_P_M1_SET;headSerialReply(4);serialize32(FloatToUint(velP1));tailSerialReply();break;
       case 14 :
                                velI1 = velI1 +ii*donvi;i=0;
                                if(velI1 >= 500) { velI1 =500;}
                                cmdMSP = MSP_MOTOR_PID_VEL_I_M1_SET;headSerialReply(4);serialize32(FloatToUint(velI1));tailSerialReply();break;
       case 15 :
                                volIN1 = volIN1 +ii*donvi;i=0;
                                if(volIN1 >= 500) { volIN1 =500;}
                                cmdMSP = MSP_MOTOR_VOL_M1_SET;headSerialReply(4);serialize32(FloatToUint(volIN1));tailSerialReply();break;
       case 16 :
                                volLIMIT1 = volLIMIT1 +ii*donvi;i=0;
                                if(volLIMIT1 >= 500) { volLIMIT1 =500;}
                                cmdMSP = MSP_MOTOR_VOLLIMIT_M1_SET;headSerialReply(4);serialize32(FloatToUint(volLIMIT1));tailSerialReply();break;
       case 18 :
                                pos21 = pos21 +ii*donvi;i=0;
                                if(pos21 >= 100) { pos21 =100;}
                                cmdMSP = MSP_MOTOR_POS21_SET;headSerialReply(4);serialize32(FloatToUint(pos21));tailSerialReply();break;
       case 19 :
                                pos22 = pos22 +ii*donvi;i=0;
                                if(pos22 >= 100) { pos22 =100;}
                                cmdMSP = MSP_MOTOR_POS22_SET;headSerialReply(4);serialize32(FloatToUint(pos22));tailSerialReply();break;
       case 20 :
                                vel21 = vel21 +ii*donvi*100;i=0;
                                if(vel21 >= 1000) { vel21 =1000;}
                                cmdMSP = MSP_MOTOR_VEL21_SET;headSerialReply(2);serialize16(vel21);tailSerialReply();break;
       case 21 :
                                vel22 = vel22 +ii*donvi*100;i=0;
                                if(vel22 >= 1000) { vel22 =1000;}
                                cmdMSP = MSP_MOTOR_VEL22_SET;headSerialReply(2);serialize16(vel22);tailSerialReply();break;
       case 22 :
                                accel2 = accel2 +ii*donvi*100;i=0;
                                if(accel2 >= 2000) { accel2 =2000;}
                                cmdMSP = MSP_MOTOR_ACCEL2_SET;headSerialReply(2);serialize16(accel2);tailSerialReply();break;
       case 23 :
                                decel2 = decel2 +ii*donvi*100;i=0;
                                if(decel2 >= 2000) { decel2 =2000;}
                                cmdMSP = MSP_MOTOR_DECCEL2_SET;headSerialReply(2);serialize16(decel2);tailSerialReply();break;
       case 24 :
                                posP2 = posP2 +ii*donvi;i=0;
                                if(posP2 >= 500) { posP2 =500;}
                                cmdMSP = MSP_MOTOR_PID_P_M2_SET;headSerialReply(4);serialize32(FloatToUint(posP2));tailSerialReply();break;
       case 25 :
                                velP2 = velP2 +ii*donvi;i=0;
                                if(velP2 >= 500) { velP2 =500;}
                                cmdMSP = MSP_MOTOR_PID_VEL_P_M2_SET;headSerialReply(4);serialize32(FloatToUint(velP2));tailSerialReply();break;
       case 26 :
                                velI2 = velI2 +ii*donvi;i=0;
                                if(velI2 >= 500) { velI2 =500;}
                                cmdMSP = MSP_MOTOR_PID_VEL_I_M2_SET;headSerialReply(4);serialize32(FloatToUint(velI2));tailSerialReply();break;
       case 27 :
                                volIN2 = volIN2 +ii*donvi;i=0;
                                if(volIN2 >= 500) { volIN2 =500;}
                                cmdMSP = MSP_MOTOR_VOL_M2_SET;headSerialReply(4);serialize32(FloatToUint(volIN2));tailSerialReply();break;
       case 28 :
                                volLIMIT2 = volLIMIT2 +ii*donvi;i=0;
                                if(volLIMIT2 >= 500) { volLIMIT2 =500;}
                                cmdMSP = MSP_MOTOR_VOLLIMIT_M2_SET;headSerialReply(4);serialize32(FloatToUint(volLIMIT2));tailSerialReply();break;
       
        case 42 :
                                velD1 = velD1 +ii*donvi*0.1;i=0;
                                if(velD1 >= 50) { velD1 =50;}if(velD1<0){velD1 =0;} 
                                cmdMSP = MSP_MOTOR_PID_VEL_D_M1_SET;headSerialReply(4);serialize32(FloatToUint(velD1));tailSerialReply();break;
       case 43 :
                                velD2 = velD2 +ii*donvi*0.1;i=0;
                                if(velD2 >= 50) { velD2 =50;}if(velD2<0){ velD2 =0; }
                                cmdMSP = MSP_MOTOR_PID_VEL_D_M2_SET;headSerialReply(4);serialize32(FloatToUint(velD2));tailSerialReply();break;
       case 49 :
                                resolution_1_set = resolution_1_set +ii;i=0;
                                if(resolution_1_set >= 250) { resolution_1_set =250;}if(resolution_1_set<0){ resolution_1_set =1; }
                                cmdMSP = MSP_MOTOR_RESO_1_SET;headSerialReply(1);serialize8(resolution_1_set);tailSerialReply();break;
      case 50 :
                                resolution_2_set = resolution_2_set +ii;i=0;
                                if(resolution_2_set >= 250) { resolution_2_set =250;}if(resolution_2_set<0){ resolution_2_set =1; }
                                cmdMSP = MSP_MOTOR_RESO_2_SET;headSerialReply(1);serialize8(resolution_2_set);tailSerialReply();break;
      case 51 :
                                pole_1_set = pole_1_set +ii;i=0;
                                if(pole_1_set >= 250) { pole_1_set =250;}if(pole_1_set<0){ pole_1_set =1; }
                                cmdMSP = MSP_MOTOR_POLE_1_SET;headSerialReply(1);serialize8(pole_1_set);tailSerialReply();break;
      case 52 :
                                pole_2_set = pole_2_set +ii;i=0;
                                if(pole_2_set >= 250) { pole_2_set =250;}if(pole_2_set<0){ pole_2_set =1; }
                                cmdMSP = MSP_MOTOR_POLE_2_SET;headSerialReply(1);serialize8(pole_2_set);tailSerialReply();break;

       ii=0;
       }}
       //volum decrease
       if ((Kaiten == 1)&& (huong == -1)){
      i=-1;
       saving =0;
       switch(current_screen)
    {
      case 6:
                              pos11 = pos11 +i*donvi;ii=0;
                              if(pos11 <= -100)  {pos11 =-100;}
                              cmdMSP = MSP_MOTOR_POS11_SET;headSerialReply(4);serialize32(FloatToUint(pos11));tailSerialReply();break;
      case 7:                        
                                pos12 = pos12 +i*donvi;ii=0;
                                if(pos12 <= -100)  {pos12 =-100;}
                                cmdMSP = MSP_MOTOR_POS12_SET;headSerialReply(4);serialize32(FloatToUint(pos12));tailSerialReply();break;
       case 8:
                                vel11 = vel11 +i*donvi*100;ii=0;
                                if(vel11 <= 1)  {vel11 =1;}
                                cmdMSP = MSP_MOTOR_VEL11_SET;headSerialReply(2);serialize16(vel11);tailSerialReply();break;
       case 9:
                                vel12 = vel12 +i*donvi*100;ii=0;
                                if(vel12 <= 1)  {vel12 =1;}
                                cmdMSP = MSP_MOTOR_VEL12_SET;headSerialReply(2);serialize16(vel12);tailSerialReply();break;
       case 10:
                                accel1 = accel1 +i*donvi*100;ii=0;
                                if(accel1 <= 1)  {accel1 =1;}
                                cmdMSP = MSP_MOTOR_ACCEL1_SET;headSerialReply(2);serialize16(accel1);tailSerialReply();break;
       case 11:
                                decel1 = decel1 +i*donvi*100;ii=0;
                                if(decel1 <= 1)  {decel1 =1;}
                                cmdMSP = MSP_MOTOR_DECCEL1_SET;headSerialReply(2);serialize16(decel1);tailSerialReply();break;
       case 12:
                                posP1 = posP1 +i*donvi;ii=0;
                                if(posP1 <= 0)  {posP1 =0;}
                                cmdMSP = MSP_MOTOR_PID_P_M1_SET;headSerialReply(4);serialize32(FloatToUint(posP1));tailSerialReply();break;
       case 13:
                                velP1 = velP1 +i*donvi;ii=0;
                                if(velP1 <= 0)  {velP1 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_P_M1_SET;headSerialReply(4);serialize32(FloatToUint(velP1));tailSerialReply();break;
       case 14:
                                velI1 = velI1 +i*donvi;ii=0;
                                if(velI1 <= 0)  {velI1 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_I_M1_SET;headSerialReply(4);serialize32(FloatToUint(velI1));tailSerialReply();break;
       case 15:
                                volIN1 = volIN1 +i*donvi;ii=0;
                                if(volIN1 <= 1)  {volIN1 =1;}
                                cmdMSP = MSP_MOTOR_VOL_M1_SET;headSerialReply(4);serialize32(FloatToUint(volIN1));tailSerialReply();break;
       case 16:
                                volLIMIT1 = volLIMIT1 +i*donvi;ii=0;
                                if(volLIMIT1 <= 1)  {volLIMIT1 =1;}
                                cmdMSP = MSP_MOTOR_VOLLIMIT_M1_SET;headSerialReply(4);serialize32(FloatToUint(volLIMIT1));tailSerialReply();break;
      case 18:
                                pos21 = pos21 +i*donvi;ii=0;
                                if(pos21 <= -100)  {pos21 =-100;}
                                cmdMSP = MSP_MOTOR_POS21_SET;headSerialReply(4); serialize32(FloatToUint(pos21));tailSerialReply();break;
       case 19:
                                pos22 = pos22 +i*donvi;ii=0;
                                if(pos22 <= -100)  {pos22 =-100;}
                                cmdMSP = MSP_MOTOR_POS22_SET;headSerialReply(4);serialize32(FloatToUint(pos22));tailSerialReply();break;
       case 20:
                                vel21 = vel21 +i*donvi*100;ii=0;
                                if(vel21 <= 1)  {vel21 =1;}
                                cmdMSP = MSP_MOTOR_VEL21_SET;headSerialReply(2);serialize16(vel21);tailSerialReply();break;
       case 21:
                                vel22 = vel22 +i*donvi*100;ii=0;
                                if(vel22 <= 1)  {vel22 =1;}
                                cmdMSP = MSP_MOTOR_VEL22_SET;headSerialReply(2);serialize16(vel22);tailSerialReply();break;
       case 22:
                                accel2 = accel2 +i*donvi*100;ii=0;
                                if(accel2 <= 1)  {accel2 =1;}
                                cmdMSP = MSP_MOTOR_ACCEL2_SET;headSerialReply(2);serialize16(accel2);tailSerialReply();break;
       case 23:
                                decel2 = decel2 +i*donvi*100;ii=0;
                                if(decel2 <= 1)  {decel2 =1;}
                                cmdMSP = MSP_MOTOR_DECCEL2_SET;headSerialReply(2); serialize16(decel2);tailSerialReply();break;
       case 24:
                                posP2 = posP2 +i*donvi;ii=0;
                                if(posP2 <= 0)  {posP2 =0;}
                                cmdMSP = MSP_MOTOR_PID_P_M2_SET; headSerialReply(4); serialize32(FloatToUint(posP2));tailSerialReply();break;
       case 25:
                                velP2 = velP2 +i*donvi;ii=0;
                                if(velP2 <= 0)  {velP2 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_P_M2_SET;headSerialReply(4);serialize32(FloatToUint(velP2));tailSerialReply();break;
       case 26:
                                velI2 = velI2 +i*donvi;ii=0;
                                if(velI2 <= 0)  {velI2 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_I_M2_SET;headSerialReply(4);serialize32(FloatToUint(velI2));tailSerialReply();break;
       case 27:
                                volIN2 = volIN2 +i*donvi;ii=0;
                                if(volIN2 <= 1)  {volIN2 =1;}
                                cmdMSP = MSP_MOTOR_VOL_M2_SET;headSerialReply(4);serialize32(FloatToUint(volIN2));tailSerialReply();break;
       case 28:
                                volLIMIT2 = volLIMIT2 +i*donvi;ii=0;
                                if(volLIMIT2 <= 1)  {volLIMIT2 =1;}
                                cmdMSP = MSP_MOTOR_VOLLIMIT_M2_SET;headSerialReply(4);serialize32(FloatToUint(volLIMIT2));tailSerialReply();break;
      
     //------------------
       case 42:
                                velD1 = velD1 +i*donvi*0.1;ii=0;
                                if(velD1 <= 0)  {velD1 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_D_M1_SET;headSerialReply(4);serialize32(FloatToUint(velD1));tailSerialReply();break;
       case 43:
                                velD2 = velD2 +i*donvi*0.1;ii=0;
                                if(velD2 <= 0)  {velD2 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_D_M2_SET;headSerialReply(4);serialize32(FloatToUint(velD2));tailSerialReply();break;
       case 49:
                                resolution_1_set = resolution_1_set +i;ii=0;
                                if(resolution_1_set >= 250) { resolution_1_set =250;}
                                if(resolution_1_set<0){             resolution_1_set =1; }
                                cmdMSP = MSP_MOTOR_RESO_1_SET;headSerialReply(1); serialize8(resolution_1_set);tailSerialReply();break;
      case 50:
                                resolution_2_set = resolution_2_set +i;ii=0;
                                if(resolution_2_set >= 250) { resolution_2_set =250;}
                                if(resolution_2_set<0){             resolution_2_set =1; }
                                cmdMSP = MSP_MOTOR_RESO_2_SET;headSerialReply(1);serialize8(resolution_2_set);tailSerialReply();break;
      case 51:
                                pole_1_set = pole_1_set +i;ii=0;
                                if(pole_1_set >= 250) { pole_1_set =250;}
                                if(pole_1_set<0){             pole_1_set =1; }
                                cmdMSP = MSP_MOTOR_POLE_1_SET;headSerialReply(1);serialize8(pole_1_set);tailSerialReply();break;
      case 52:
                                pole_2_set = pole_2_set +i;ii=0;
                                if(pole_2_set >= 250) { pole_2_set =250;}
                                if(pole_2_set<0){             pole_2_set =1; }
                                cmdMSP = MSP_MOTOR_POLE_2_SET;headSerialReply(1);serialize8(pole_2_set);tailSerialReply();break;
       ///
       i=0;
       }  

    }

  }


if ((menuCount == 5)&& current_screen !=  53 && current_screen !=  54) {
  uint8_t btn12 = digitalRead(BUTTON_SELECT_PIN);
  if (btn12 == LOW && btn_prev12 == HIGH)
  {
        current_screen = 0;
        menuCount = 0;
        access3 =0;
        saving =0;
  }
  btn_prev12 = digitalRead(BUTTON_SELECT_PIN);
   
  }
}
void menuCheck_moderun() {
 if ((lock != 1)&&(lock_tani !=1)){
    if ((Kaiten == 1)&& (huong == 1)){menuCount = menuCount -1;}
    if ((Kaiten == 1)&& (huong == -1)){menuCount = menuCount +1;}  
    if(menuCount <= 1) { menuCount =1;}
    if(menuCount > 5)  {menuCount =5;}
 }
/////////////////////////////////////////////////////////////////////////////
if ((menuCount == 1)&& (output_select_State == LOW)&&(current_screen ==  53 || current_screen ==  54)) {//angle
    output_select_State =HIGH;
u8g2.drawBox(9,0, 119, 13);
delay(10);
if (current_screen ==  53){cmdMSP = MSP_MOTOR_MOTION_CTRL_1;}
else {cmdMSP = MSP_MOTOR_MOTION_CTRL_2;}
                                headSerialReply(1);
                                serialize8(0);
                                tailSerialReply();
}

if ((menuCount == 2)&& (output_select_State == LOW)&&(current_screen ==  53 || current_screen ==  54)) {//torque
    output_select_State =HIGH;
    u8g2.drawBox(9,13, 119, 13);
delay(10);
if (current_screen ==  53){cmdMSP = MSP_MOTOR_MOTION_CTRL_1;}
else {cmdMSP = MSP_MOTOR_MOTION_CTRL_2;}
                                headSerialReply(1);
                                serialize8(1);
                                tailSerialReply();
}

if ((menuCount == 3)&& (output_select_State == LOW)&&(current_screen ==  53 || current_screen ==  54)) {//velocity
    output_select_State =HIGH;
    u8g2.drawBox(9,26, 119, 13);
delay(10);
if (current_screen ==  53){cmdMSP = MSP_MOTOR_MOTION_CTRL_1;}
else {cmdMSP = MSP_MOTOR_MOTION_CTRL_2;}
                                headSerialReply(1);
                                serialize8(2);
                                tailSerialReply();
}

if ((menuCount == 4)&& (output_select_State == LOW)&&(current_screen ==  53 || current_screen ==  54)) {//velocity_openloop
    output_select_State =HIGH;
    u8g2.drawBox(9,39, 119, 13);
delay(10);
if (current_screen ==  53){cmdMSP = MSP_MOTOR_MOTION_CTRL_1;}
else {cmdMSP = MSP_MOTOR_MOTION_CTRL_2;}
                                headSerialReply(1);
                                serialize8(3);
                                tailSerialReply();
                                
}

if ((menuCount == 5)&& (output_select_State == LOW)&&(current_screen ==  53 || current_screen ==  54)) {//angle_openloop
   output_select_State =HIGH;
   u8g2.drawBox(9,52, 119, 13);
delay(10);
if (current_screen ==  53){cmdMSP = MSP_MOTOR_MOTION_CTRL_1;}
else {cmdMSP = MSP_MOTOR_MOTION_CTRL_2;}
                                headSerialReply(1);
                                serialize8(4);
                                tailSerialReply();
  }   



}