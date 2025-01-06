void number_motor_screen(){
  if (num_enable_onetime== 0) { num_enable = 1;}
      num_enable_onetime = 1;

if (cur_number_motor ==1 ){
   u8g2.setFont(u8g2_font_unifont_t_symbols);
    u8g2.drawGlyph(110, 25, 0x25c0); 
  }
if (cur_number_motor ==2 ){
   u8g2.setFont(u8g2_font_unifont_t_symbols);
    u8g2.drawGlyph(110, 38, 0x25c0);  
  }


if (number_motor ==0) {
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2610); 
  
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 38, 0x2610);  

  
  
  }
if (number_motor ==1) {
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2611); 
  
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 38, 0x2610);  

 
  
  }
if (number_motor ==2) {
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2610); 
  
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 38, 0x2611);  

 
  
  }

      
   u8g2.setFont(u8g_font_7x14);
   u8g2.drawStr(15, 12, "SELECT_:");

  u8g2.setFont(u8g_font_7x14);
  u8g2.drawStr(25, 25, "ONE_MOTOR"); 

  u8g2.setFont(u8g_font_7x14);
  u8g2.drawStr(25, 38, "TWO_MOTOR"); 
  
 
   //
  u8g2.drawStr(25, 51, "EXIT"); 
   //
  u8g2.drawStr(2, (number_motor_select * 13) - 1, ">"); 


    if ((Kaiten == 1)&& (huong == 1)){number_motor_select = number_motor_select -1;}
    if ((Kaiten == 1)&& (huong == -1)){number_motor_select = number_motor_select +1;}  
    if(number_motor_select <= 2) { number_motor_select =2;}
    if(number_motor_select > 4)  {number_motor_select =4;}

  if ((number_motor_select == 2)&& (output_select_State == LOW)) {
    output_select_State =HIGH;
      number_motor=1;//one
      cmdMSP = MSP_MOTOR_MOTOR_SET;
      headSerialReply(1);
      serialize8(number_motor);
      tailSerialReply();
  delay(10);
  if (num_enable_onetime== 0) { num_enable = 1;}
      num_enable_onetime = 1;
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2611);  
  }
  if ((number_motor_select == 3)&& (output_select_State == LOW)) {
    output_select_State =HIGH;
      number_motor=2;//two
      cmdMSP = MSP_MOTOR_MOTOR_SET;
      headSerialReply(1);
      serialize8(number_motor);
      tailSerialReply();

  delay(10);
  if (num_enable_onetime== 0) { num_enable = 1;}
      num_enable_onetime = 1;
  }
  
if ((number_motor_select == 4)&& (output_select_State == LOW)) {
  output_select_State =HIGH;

   u8g2.clearDisplay();  
   u8g2.setFont(u8g_font_7x14);
   u8g2.drawStr(25, 12, "Please reset ");
   u8g2.drawStr(15, 25, "driver to apply"); 
   u8g2.sendBuffer();
   delay(1000);
   u8g2.clearDisplay();
   current_screen = 0;
   num_enable_onetime=0;
   number_motor_select = 0;
  }

}