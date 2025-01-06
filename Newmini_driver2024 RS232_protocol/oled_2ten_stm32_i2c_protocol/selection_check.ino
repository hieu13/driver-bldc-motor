

void selection_check(){
  
 if ((lock_select !=1)){
   if ((Kaiten == 1)&& (huong == 1)){posCount = posCount -1;
}
   if ((Kaiten == 1)&& (huong ==-1)){posCount = posCount +1;
}  
    if(posCount <= 1) { posCount =1;}
    if(posCount > 5)  {posCount =5;}
}
//
 if ((posCount == 1)&&(clicked_select == 0)&&(nikai_select == 0)&&current_screen !=  53 && current_screen !=  54) {
    uint8_t btn6 = digitalRead(BUTTON_SELECT_PIN);
    if (btn6 == LOW && btn_prev6 == HIGH)
  {
      lock_select=1;
      clicked_select =1;
  }
  btn_prev6 = digitalRead(BUTTON_SELECT_PIN);
  }
//
  if (lock_select == 1){
    uint8_t btn7 = digitalRead(BUTTON_SELECT_PIN);

    if (btn7 == LOW && btn_prev7 == HIGH)
  {
      lock_select=0;
      clicked_select =0;
      nikai_select=1;
  }
  btn_prev7 = digitalRead(BUTTON_SELECT_PIN);
    ////
     if (nikai_select == 1){lock_select =0;nikai_select =0;}
     
     if (current_screen !=  53 || current_screen !=  54){
      u8g2.drawFrame(9,0, 79, 13);}
       if ((Kaiten == 1)&& (huong == 1)) {select_sl--;}
       if ((Kaiten == 1)&& (huong == -1)){select_sl++;}  
       if(select_sl >= 2) { select_sl =2;}
       if(select_sl <= 1)  {select_sl =1;}
      
    }
 //
 if ((posCount == 2)&&current_screen !=  53 && current_screen !=  54) {
  uint8_t btn13 = digitalRead(BUTTON_SELECT_PIN);
    if (btn13 == LOW && btn_prev13 == HIGH)
  {
       if (select_sl ==1 ){
  cmdMSP = MSP_MOTOR_RUN_SET;
      headSerialReply(1);
      serialize8(0);
      tailSerialReply();
  }
  if (select_sl ==2 ){
  cmdMSP = MSP_MOTOR_RUN_SET;
      headSerialReply(1);
      serialize8(10);
      tailSerialReply();
  }
  u8g2.drawBox(9,13, 119, 13);
  }
  btn_prev13 = digitalRead(BUTTON_SELECT_PIN);


  }



  if ((posCount == 3)&&current_screen !=  53 && current_screen !=  54) {
    uint8_t btn14 = digitalRead(BUTTON_SELECT_PIN);
    if (btn14 == LOW && btn_prev14 == HIGH)
  {
    if (select_sl ==1 ){
      cmdMSP = MSP_MOTOR_RUN_SET;
      headSerialReply(1);
      serialize8(1);
      tailSerialReply();
  }
 if (select_sl ==2 ){
  cmdMSP = MSP_MOTOR_RUN_SET;
      headSerialReply(1);
      serialize8(11);
      tailSerialReply();
  }
  u8g2.drawBox(9,26, 119, 13);
  }
  btn_prev14 = digitalRead(BUTTON_SELECT_PIN);
  }
  

if ((posCount == 4)&&current_screen !=  53 && current_screen !=  54) {//pos2
uint8_t btn15 = digitalRead(BUTTON_SELECT_PIN);
    if (btn15 == LOW && btn_prev15 == HIGH)
  {
     if (select_sl ==1 ){
  cmdMSP = MSP_MOTOR_RUN_SET;
      headSerialReply(1);
      serialize8(2);
      tailSerialReply();
  }
 if (select_sl ==2 ){
  cmdMSP = MSP_MOTOR_RUN_SET;
      headSerialReply(1);
      serialize8(12);
      tailSerialReply();
  }
  u8g2.drawBox(9,39, 119, 13);
  }
  btn_prev15 = digitalRead(BUTTON_SELECT_PIN);
  }




if ((posCount == 5)&& current_screen !=  53 && current_screen !=  54) {//exit
   uint8_t btn16 = digitalRead(BUTTON_SELECT_PIN);
    if (btn16 == LOW && btn_prev16 == HIGH)
  {
   current_screen = 0;
   posCount = 0;
   }
  btn_prev16 = digitalRead(BUTTON_SELECT_PIN);
  }
////////////////////////////////////////////////////////////CONTROL MOTION?????????????????????????????






}