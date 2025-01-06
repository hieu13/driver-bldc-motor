void defaut_screen(){
u8g2.setFont(u8g_font_7x14);
  u8g2.drawStr(5, 12, "reset to defaut:");
  u8g2.setFont(u8g_font_7x14);
  u8g2.drawStr(10, 38, "NO"); 
  u8g2.setFont(u8g_font_7x14);
  u8g2.drawStr(50, 38, "YES"); 
   //
  u8g2.drawStr(90, 38, "EXT"); 
   //
  u8g2.drawStr((defaut_select *40) + 1,38 , "["); 
  u8g2.drawStr((defaut_select * 40) +30,38 , "]"); 


    if ((Kaiten == 1)&& (huong == 1)){defaut_select = defaut_select -1;}
    if ((Kaiten == 1)&& (huong == -1)){defaut_select = defaut_select +1;}  
    if(defaut_select <= 0) { defaut_select =0;}
    if(defaut_select > 2)  {defaut_select =2;}

  if ((defaut_select == 0)&& (output_select_State == LOW)) {
    output_select_State =HIGH;
  //defaut no
  cmdMSP = MSP_MOTOR_RESET_SET;
      headSerialReply(1);
      serialize8(0);
      tailSerialReply();
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2611); 
  current_screen = 0;
  defaut_select = 0;
  }
  if ((defaut_select == 1)&& (output_select_State == LOW)) {
    output_select_State =HIGH;
  //defaut yes
    cmdMSP = MSP_MOTOR_RESET_SET;
      headSerialReply(1);
      serialize8(2);
      tailSerialReply();
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(50, 25, 0x2611);  
  u8g2.clearDisplay();  
   u8g2.setFont(u8g_font_7x14);
   u8g2.drawStr(25, 12, "Please reset ");
   u8g2.drawStr(15, 25, "driver to apply"); 
   u8g2.sendBuffer();
   delay(1000);
   u8g2.clearDisplay();
   
  }
 
if ((defaut_select == 2)&& (output_select_State == LOW)) {
  output_select_State =HIGH;

   current_screen = 0;
   defaut_select = 0;
  }

}