    void screen_axis_2(){



if ((Kaiten == 1)&& (huong == 1)) {item_select2--;}
       if ((Kaiten == 1)&& (huong == -1)){item_select2++;}  
       
      if(item_select2 >= 0 && item_select2 <=6)  {item_prev2 = item_select2 - 1;item_next2 = item_select2 + 1;  }
      if (item_prev2 < 0) {item_prev2 = 6 ;}
      if (item_next2 > 6) {item_next2 = 0;}
   
     if(item_select2 < 0) { item_select2 =6;item_prev2 = item_select2 - 1;item_next2 = 0;}
     if(item_select2 > 6)  {item_select2 =0;item_prev2 = 6;item_next2 = item_select2+1;}
     
     
       u8g2.clearBuffer();
      // draw previous item as icon + label
            // selected item background
      u8g2.setFont(u8g2_font_ncenB12_tf);      
      u8g2.setFontDirection(1);
      u8g2.drawStr(3, 3, "AXIS 2");//90do 
      u8g2.setFontDirection(0);      
      u8g2.drawLine(19, 0, 19, 64);
      u8g2.drawRFrame(22,23,103,20,6);
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(25, 15, axis2_items[item_prev2]); 
      // draw selected item as icon + label in bold font
      u8g2.setFont(u8g_font_7x14B); //  u8g2_font_ncenB08_tr 
      u8g2.drawStr(25, 15+20+2, axis2_items[item_select2]);   
      // draw next item as icon + label
      u8g2.setFont(u8g2_font_ncenB08_tr);     //u8g_font_7x14
      u8g2.drawStr(25, 15+20+20+2+2, axis2_items[item_next2]);   
      // draw scrollbar background
      u8g2.drawXBMP(128-8, 0, 8, 64, bitmap_scrollbar_background);
      // draw scrollbar handle
      u8g2.drawBox(125, 64/NUMBER_ITEMS * item_select2, 3, 64/NUMBER_ITEMS); 

    }
void screen_axis_2_para(){
if ((Kaiten == 1)&& (huong == 1)) {item_select2--;}
       if ((Kaiten == 1)&& (huong == -1)){item_select2++;}  
       
      if(item_select2 >= 0 && item_select2 <=6)  {item_prev2 = item_select2 - 1;item_next2 = item_select2 + 1;  }
      if (item_prev2 < 0) {item_prev2 = 6 ;}
      if (item_next2 > 6) {item_next2 = 0;}
   
     if(item_select2 < 0) { item_select2 =6;item_prev2 = item_select2 - 1;item_next2 = 0;}
     if(item_select2 > 6)  {item_select2 =0;item_prev2 = 6;item_next2 = item_select2+1;}
     
     
       u8g2.clearBuffer();
      // draw previous item as icon + label
            // selected item background
      u8g2.setFont(u8g2_font_ncenB12_tf);      
      u8g2.setFontDirection(1);
      u8g2.drawStr(3, 3, "AXIS 2");//90do 
      u8g2.setFontDirection(0);      
      u8g2.drawLine(19, 0, 19, 64);
      u8g2.drawRFrame(22,23,103,20,6);
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(25, 15, axis2_paras[item_prev2]); 
      // draw selected item as icon + label in bold font
      u8g2.setFont(u8g_font_7x14B); //  u8g2_font_ncenB08_tr 
      u8g2.drawStr(25, 15+20+2, axis2_paras[item_select2]);   
      // draw next item as icon + label
      u8g2.setFont(u8g2_font_ncenB08_tr);     //u8g_font_7x14
      u8g2.drawStr(25, 15+20+20+2+2, axis2_paras[item_next2]);   
      // draw scrollbar background
      u8g2.drawXBMP(128-8, 0, 8, 64, bitmap_scrollbar_background);
      // draw scrollbar handle
      u8g2.drawBox(125, 64/NUMBER_ITEMS * item_select2, 3, 64/NUMBER_ITEMS); 

}

        
      