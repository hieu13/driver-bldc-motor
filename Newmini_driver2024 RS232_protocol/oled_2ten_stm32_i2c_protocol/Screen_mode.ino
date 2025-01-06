void screen_mode(){
 if (mode_enable_onetime== 0) { mode_enable = 1;}
    mode_enable_onetime = 1;

if (cur_mode ==1 ){
   u8g2.setFont(u8g2_font_unifont_t_symbols);
    u8g2.drawGlyph(110, 25, 0x25c0);  /* dec 9731/hex 2603 Snowman */
  }
if (cur_mode ==2 ){
   u8g2.setFont(u8g2_font_unifont_t_symbols);
    u8g2.drawGlyph(110, 38, 0x25c0);  /* dec 9731/hex 2603 Snowman */
  }
if (cur_mode ==3 ){
   u8g2.setFont(u8g2_font_unifont_t_symbols);
    u8g2.drawGlyph(110, 51, 0x25c0);  /* dec 9731/hex 2603 Snowman */
  }

if (mode ==0) {
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2610);  /* hako */
  
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 38, 0x2610);  /* hako */

  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 51, 0x2610);  /* hako */
  
  }
if (mode ==1) {
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2611);  /* hako */
  
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 38, 0x2610);  /* hako */

  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 51, 0x2610);  /* hako */
  
  }
if (mode ==2) {
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2610);  /* hako */
  
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 38, 0x2611);  /* hako */

  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 51, 0x2610);  /* hako */
  
  }
if (mode ==3) {
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2610);  /* hako */
  
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 38, 0x2610);  /* hako */

  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 51, 0x2611);  /* hako */
  
  }
      
       u8g2.setFont(u8g_font_7x14);
   u8g2.drawStr(15, 12, "SELECT_MODE:");

  
  u8g2.setFont(u8g_font_7x14);
  u8g2.drawStr(25, 25, "<PLC_IO>"); 
 // u8g2.drawStr(80, 25, "RUN"); 
  
  u8g2.setFont(u8g_font_7x14);
  u8g2.drawStr(25, 38, "<PULSE_DIR>"); 
 // u8g2.drawStr(80, 38, "RUN"); 
  
  u8g2.setFont(u8g_font_7x14);
  u8g2.drawStr(25, 51, "<SERIAL>"); 
   //
  u8g2.drawStr(10, 64, "EXIT"); 
   //
  u8g2.drawStr(2, (modeCount * 13) - 1, ">"); 


    if ((Kaiten == 1)&& (huong == 1)){modeCount = modeCount -1;}
    if ((Kaiten == 1)&& (huong == -1)){modeCount = modeCount +1;}  
    if(modeCount <= 2) { modeCount =2;}
    if(modeCount > 5)  {modeCount =5;}

  if ((modeCount == 2)&& (output_select_State == LOW)) {
    output_select_State =HIGH;
  mode=1;//PLC IO
      cmdMSP = MSP_MOTOR_MODE_RUN;
      headSerialReply(1);
      serialize8(mode);
      tailSerialReply();
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2611);  /* hako */
  u8g2.clearDisplay();  
   u8g2.setFont(u8g_font_7x14);
   u8g2.drawStr(25, 12, "OK Please reset ");
   u8g2.drawStr(45, 25, "driver"); 
   u8g2.sendBuffer();
   delay(1000);
   u8g2.clearDisplay();
  
  }
  if ((modeCount == 3)&& (output_select_State == LOW)) {
    output_select_State =HIGH;
  mode=2;//pulse
      cmdMSP = MSP_MOTOR_MODE_RUN;
      headSerialReply(1);
      serialize8(mode);
      tailSerialReply();
  u8g2.clearDisplay();  
   u8g2.setFont(u8g_font_7x14);
   u8g2.drawStr(25, 12, "OK Please reset ");
   u8g2.drawStr(45, 25, "driver"); 
   u8g2.sendBuffer();
   delay(1000);
   u8g2.clearDisplay();
  
  }
  if ((modeCount == 4)&& (output_select_State == LOW)) {
    output_select_State =HIGH;
  mode=3;//serial
      cmdMSP = MSP_MOTOR_MODE_RUN;
      headSerialReply(1);
      serialize8(mode);
      tailSerialReply(); 
  u8g2.clearDisplay();  
   u8g2.setFont(u8g_font_7x14);
   u8g2.drawStr(25, 12, "OK Please reset ");
   u8g2.drawStr(45, 25, "driver"); 
   u8g2.sendBuffer();
   delay(1000);
   u8g2.clearDisplay();
  
  }
if ((modeCount == 5)&& (output_select_State == LOW)) {

   output_select_State =HIGH;
   current_screen = 0;
   mode_enable_onetime=0;
   modeCount = 0;

      
     }


}
void screen_mode_base(){

if ((Kaiten == 1)&& (huong == 1)) {item_select1--;}
       if ((Kaiten == 1)&& (huong == -1)){item_select1++;}  
       
      if(item_select1 >= 0 && item_select1 <=6)  {item_prev1 = item_select1 - 1;item_next1 = item_select1 + 1;  }
      if (item_prev1 < 0) {item_prev1 = 6 ;}
      if (item_next1 > 6) {item_next1 = 0;}
   
     if(item_select1 < 0) { item_select1 =6;item_prev1 = item_select1 - 1;item_next1 = 0;}
     if(item_select1 > 6)  {item_select1 =0;item_prev1 = 6;item_next1 = item_select1+1;}
     
     
       u8g2.clearBuffer();
      // draw previous item as icon + label
            // selected item background
      u8g2.setFont(u8g2_font_ncenB12_tf);      
      u8g2.setFontDirection(1);
      u8g2.drawStr(3, 3, "BASE");//90do 
      u8g2.setFontDirection(0);      
      u8g2.drawLine(19, 0, 19, 64);
      u8g2.drawRFrame(22,23,103,20,6);
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(25, 15, base_motor[item_prev1]); 
      // draw selected item as icon + label in bold font
      u8g2.setFont(u8g_font_7x14B); //  u8g2_font_ncenB08_tr 
      u8g2.drawStr(25, 15+20+2, base_motor[item_select1]);   
      // draw next item as icon + label
      u8g2.setFont(u8g2_font_ncenB08_tr);     //u8g_font_7x14
      u8g2.drawStr(25, 15+20+20+2+2, base_motor[item_next1]);   
      // draw scrollbar background
      u8g2.drawXBMP(128-8, 0, 8, 64, bitmap_scrollbar_background);
      // draw scrollbar handle
      u8g2.drawBox(125, 64/NUMBER_ITEMS * item_select1, 3, 64/NUMBER_ITEMS); 


      
      
      
  }
  




