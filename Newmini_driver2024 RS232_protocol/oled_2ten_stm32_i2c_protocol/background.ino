void background_main(){

 switch(error)
  {
    case 0:
  draw("Motor is good!", SUN, error);
  break;
    case 1:
  draw("Motor 1 calib error!", RAIN, error);
  break;  
  case 2:
    draw("Motor 2 calib error!", RAIN, error);
    break;
    case 3:
  draw("Motor 1 is over", THUNDER, error);
  break;
  case 4:
  draw("Motor 2 is over", THUNDER, error);
    break;
  case 5:
  draw("Motor please fix it", CLOUD, error);
    break;
   case 6:
  draw("Motor 1 error!", SUN_CLOUD, error);
  break; 
}
}
void drawWeatherSymbol(u8g2_uint_t x, u8g2_uint_t y, uint8_t symbol)
{
  // fonts used:
  // u8g2_font_open_iconic_embedded_6x_t
  // u8g2_font_open_iconic_weather_6x_t
  // encoding values, see: https://github.com/olikraus/u8g2/wiki/fntgrpiconic
  
  switch(symbol)
  {
    case SUN:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 69);	
      break;
    case SUN_CLOUD:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 65);	
      break;
    case CLOUD:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 64);	
      break;
    case RAIN:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 67);	
      break;
    case THUNDER:
      u8g2.setFont(u8g2_font_open_iconic_embedded_6x_t);
      u8g2.drawGlyph(x, y, 67);
      break;      
  }
}

void drawWeather(uint8_t symbol, int degree)
{
  drawWeatherSymbol(0, 48, symbol);
  u8g2.setFont(u8g2_font_logisoso22_tf);
  u8g2.setCursor(48+3, 42);
  u8g2.print("Err:");
  u8g2.setFont(u8g2_font_logisoso32_tf);
  u8g2.print(degree);
  		// requires enableUTF8Print()
}
void drawScrollString(int16_t offset, const char *s)
{
  static char buf[36];	// should for screen with up to 256 pixel width 
  size_t len;
  size_t char_offset = 0;
  u8g2_uint_t dx = 0;
  size_t visible = 0;
  len = strlen(s);
  if ( offset < 0 )
  {
    char_offset = (-offset)/8;
    dx = offset + char_offset*8;
    if ( char_offset >= u8g2.getDisplayWidth()/8 )
      return;
    visible = u8g2.getDisplayWidth()/8-char_offset+1;
    strncpy(buf, s, visible);
    buf[visible] = '\0';
    u8g2.setFont(u8g2_font_8x13_mf);
    u8g2.drawStr(char_offset*8-dx, 62, buf);
  }
  else
  {
    char_offset = offset / 8;
    if ( char_offset >= len )
      return;	// nothing visible
    dx = offset - char_offset*8;
    visible = len - char_offset;
    if ( visible > u8g2.getDisplayWidth()/8+1 )
      visible = u8g2.getDisplayWidth()/8+1;
    strncpy(buf, s+char_offset, visible);
    buf[visible] = '\0';
    u8g2.setFont(u8g2_font_8x13_mf);
    u8g2.drawStr(-dx, 62, buf);
  }
  
}

void draw(const char *s, uint8_t symbol, int degree)
{
  int16_t offset = -(int16_t)u8g2.getDisplayWidth();
  int16_t len = strlen(s);
  for(;;)
  {
    u8g2.firstPage();
    do {
      drawWeather(symbol, degree);
      drawScrollString(offset, s);
      if (digitalRead(BUTTON_UP_PIN) == LOW||digitalRead(BUTTON_DOWN_PIN)==LOW){current_screen=0;error=0;return;}
      //if (current_screen!=99){}
          protocol_send_cmd();
          serial_Event();
    } while ( u8g2.nextPage() &&current_screen==99 &&error==0);
   // delay(20);
    offset+=2;
    if ( offset > len*8+1 ){background_main();}//return;}
  //    break;
  }
}


void serial_Event() {
  uint8_t c, port, state, bytesTXBuff;
  static uint8_t offset;
  static uint8_t dataSize;
  static uint8_t c_state;



  while (Serial.available() > 0) {
    //bytesTXBuff = SerialUsedTXBuff; // indicates the number of occupied bytes in TX buffer
    //if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
    c = Serial.read();
    //#ifdef SUPPRESS_ALL_SERIAL_MSP
    //  evaluateOtherData(c); // no MSP handling, so go directly
    //#else //SUPPRESS_ALL_SERIAL_MSP
    state = c_state;
    // regular data handling to detect and handle MSP and other data
    if (state == IDLE) {
      if (c == '$') state = HEADER_START;
      else { state = HEADER_START; }  //evaluateOtherData(c);} // evaluate all other incoming serial data
    } else if (state == HEADER_START) {
      state = (c == 'M') ? HEADER_M : IDLE;
    } else if (state == HEADER_M) {
      state = (c == '>') ? HEADER_ARROW : IDLE;
    } else if (state == HEADER_ARROW) {
      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        state = IDLE;
        continue;
      }
      dataSize = c;
      checksum = c;
      offset = 0;
      indRX = 0;
      state = HEADER_SIZE;  // the command is to follow
    } else if (state == HEADER_SIZE) {
      cmdMSP = c;
      checksum ^= c;
      state = HEADER_CMD;
    } else if (state == HEADER_CMD) {
      if (offset < dataSize) {
        checksum ^= c;
        inBuf[offset++] = c;
      } else {
        if (checksum == c)          // compare calculated and transferred checksum
          evaluateCommand(cmdMSP);  // we got a valid packet, evaluate it
        state = IDLE;
        
      }
    }
    c_state = state;

  }     // while
}