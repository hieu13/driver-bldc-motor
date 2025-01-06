#include <Arduino.h>
#include <U8g2lib.h>
//#include <RotaryEncoder.h>
#include "crc8.h" 

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif




U8G2LOG u8g2log;
#define U8LOG_WIDTH 18
#define U8LOG_HEIGHT 5
uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];
//#define BEEPER             9
//#define BTN_EN1            31
//#define BTN_EN2            33
//#define BTN_ENC            35


#define BUTTON_SELECT_PIN PA0 // pin for SELECT button
#define BUTTON_EXIT_PIN PA1 // pin for DOWN button
#define BUTTON_UP_PIN PA3
#define BUTTON_DOWN_PIN PA2

//#define SS1_LED_PIN PA2

//// SD card settings
//#define SDPOWER            -1
//#define SDSS               53
//#define SDCARDDETECT       49

//#define KILL_PIN           41

uint8_t  data_telemetry8[15];
uint16_t data_telemetry16[15];
uint32_t data_telemetry32[20];
uint8_t motor_ready,motor_ss,error,i_cout;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 5;    // the debounce time; increase if the output flickers
int buttonState,select_button_state;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin

unsigned long t = 0;
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C.setI2CAddress(0x3C * 2);
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

//U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1106_128X64_NONAME_F_HW_I2C 
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C
//RotaryEncoder *encoder = nullptr;
//U8G2_SSD1312_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PB6, /* data=*/ PB7);


const unsigned char upir_logo [] PROGMEM = {  
  0xEA, 0x3A, 0xAA, 0x28, 0x6A, 0x1A, 0x26, 0x2A, };


// 'icon_3dcube', 16x16px
const unsigned char bitmap_icon_3dcube [] PROGMEM = {
  0x00, 0x00, 0x80, 0x01, 0xE0, 0x06, 0x98, 0x18, 0x86, 0x60, 0x8A, 0x50, 
  0xA2, 0x45, 0x82, 0x40, 0xA2, 0x44, 0x82, 0x40, 0xA2, 0x45, 0x8A, 0x50, 
  0x86, 0x60, 0x98, 0x18, 0xE0, 0x06, 0x80, 0x01, };  
// 'icon_battery', 16x16px
const unsigned char bitmap_icon_battery [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x1F, 0x02, 0x20, 
  0xDA, 0x66, 0xDA, 0x66, 0xDA, 0x66, 0x02, 0x20, 0xFC, 0x1F, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
// 'icon_dashboard', 16x16px
const unsigned char bitmap_icon_dashboard [] PROGMEM = {
  0xE0, 0x07, 0x18, 0x18, 0x84, 0x24, 0x0A, 0x40, 0x12, 0x50, 0x21, 0x80, 
  0xC1, 0x81, 0x45, 0xA2, 0x41, 0x82, 0x81, 0x81, 0x05, 0xA0, 0x02, 0x40, 
  0xD2, 0x4B, 0xC4, 0x23, 0x18, 0x18, 0xE0, 0x07, };
// 'icon_fireworks', 16x16px
const unsigned char bitmap_icon_fireworks [] PROGMEM = {
  0x00, 0x00, 0x00, 0x10, 0x00, 0x29, 0x08, 0x10, 0x08, 0x00, 0x36, 0x00, 
  0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x63, 0x00, 0x00, 0x00, 0x08, 
  0x20, 0x08, 0x50, 0x00, 0x20, 0x00, 0x00, 0x00, };
// 'icon_gps_speed', 16x16px
const unsigned char bitmap_icon_gps_speed [] PROGMEM = {
  0x00, 0x00, 0xC0, 0x0F, 0x00, 0x10, 0x80, 0x27, 0x00, 0x48, 0x00, 0x53, 
  0x60, 0x54, 0xE0, 0x54, 0xE0, 0x51, 0xE0, 0x43, 0xE0, 0x03, 0x50, 0x00, 
  0xF8, 0x00, 0x04, 0x01, 0xFE, 0x03, 0x00, 0x00, };
// 'icon_knob_over_oled', 16x16px
const unsigned char bitmap_icon_knob_over_oled [] PROGMEM = {
  0x00, 0x00, 0xF8, 0x0F, 0xC8, 0x0A, 0xD8, 0x0D, 0x88, 0x0A, 0xF8, 0x0F, 
  0xC0, 0x01, 0x80, 0x00, 0x00, 0x00, 0x90, 0x04, 0x92, 0x24, 0x04, 0x10, 
  0x00, 0x80, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, };
// 'icon_parksensor', 16x16px
const unsigned char bitmap_icon_parksensor [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x44, 0x00, 0xA4, 0x00, 
  0x9F, 0x00, 0x00, 0x81, 0x30, 0xA1, 0x48, 0xA9, 0x4B, 0xA9, 0x30, 0xA0, 
  0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
// 'icon_turbo', 16x16px
const unsigned char bitmap_icon_turbo [] PROGMEM = {
  0x00, 0x70, 0xE0, 0x8F, 0x18, 0x80, 0x04, 0x80, 0x02, 0x80, 0xC2, 0x8F, 
  0x21, 0x72, 0x51, 0x05, 0x91, 0x44, 0x51, 0x45, 0x21, 0x42, 0xC2, 0x21, 
  0x02, 0x20, 0x04, 0x10, 0x18, 0x0C, 0xE0, 0x03, };

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 384)
const unsigned char* bitmap_icons[7] = {

  bitmap_icon_gps_speed,
  bitmap_icon_dashboard,
  bitmap_icon_turbo,
  bitmap_icon_fireworks,
  bitmap_icon_battery,
  bitmap_icon_parksensor,
  bitmap_icon_knob_over_oled
  

 
  
};


// 'scrollbar_background', 8x64px
const unsigned char bitmap_scrollbar_background [] PROGMEM = {
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x00, };


// 'item_sel_outline', 128x21px
const unsigned char bitmap_item_sel_outline [] PROGMEM = {
  0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0xF8, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 
  };





// ------------------ end generated bitmaps from image2cpp ---------------------------------



const int NUM_ITEMS = 6; // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LENGTH = 20; // maximum characters for the item name

char menu_items [NUM_ITEMS] [MAX_ITEM_LENGTH] = {  // array with item names
  { "AXIS<1>" }, 
  { "AXIS<2>" },
  { "RUN TEST_io" },
  { "SET CTRL Mode" },
  { "SET NUM Motor" },
  { "SET Defaut" }
 };

const int NUMBER_ITEMS = 15; // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LEN = 20; // maximum characters for the item name

char axis1_items [NUMBER_ITEMS] [MAX_ITEM_LEN] = {  // array with item names
  
  { "POSITION<1>" }, 
  { "POSITION<2>" },
  { "POSITION<3>" }, 
  { "SPEED POS<1>" },
  { "SPEED POS<2>" }, 
  { "SPEED POS<3>" }, 
  { "Accelerate" }, 
  { "Decelerate" },   
  { "PID_position" }, 
  { "PID_P_vel" },
  { "PID_I_vel" },
  { "PID_D_vel" },
  { "Set_VACC_IN" },
  { "Set_VolLimit" },
  { "SAVE Axis1" }
 };
char axis2_items [NUMBER_ITEMS] [MAX_ITEM_LEN] = {  // array with item names
  
  { "POSITION<1>" }, 
  { "POSITION<2>" }, 
  { "POSITION<3>" },
  { "SPEED POS<1>" }, 
  { "SPEED POS<2>" },
  { "SPEED POS<3>" }, 
  { "Accelerate" }, 
  { "Decelerate" },   
  { "PID_position" }, 
  { "PID_P_vel" },
  { "PID_I_vel" },
  { "PID_D_vel" },
  { "Set_VACC_IN" },
  { "Set_VolLimit" },
  { "SAVE Axis2" }
 };




const int NUM_ITEMS1 = 2; // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LENGTH1 = 20; // maximum characters for the item name
 char ok_items [NUM_ITEMS1] [MAX_ITEM_LENGTH1] = {  // array with item names
  { "YES" }, 
  { "NO" }
 };


int button_up_clicked = 0; // only perform action when button is clicked, and wait until another press
int button_select_clicked = 0; // same as above
int button_clicked = 0; // same as above
int button_down_clicked = 0; // same as above
int button_click1 =0;
int button_click2 =0;
int button_click3 =0;

int item_selected = 0; // which item in the menu is selected
int clicked=0;
int clicked_tani=0;
int clicked_select=0;
int item_sel_previous; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next; // next item - used in the menu screen to draw next item after the selected one

int item_select1 = 0;
int item_prev1;
int item_next1; 
int item_select2 = 0;
int item_prev2;
int item_next2; 
int item_select3 = 0;
int item_prev3;
int item_next3; 


int current_screen = 0;   // 0 = menu, 1 = screenshot, 2 = qr

int demo_mode = 0; // when demo mode is set to 1, it automatically goes over all the screens, 0 = control menu with buttons
int demo_mode_state = 0; // demo mode state = which screen and menu item to display
int demo_mode_delay = 0; // demo mode delay = used to slow down the screen switching
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
float cur_pos11,cur_pos12,cur_pos13,cur_vel11,cur_vel12,cur_vel13,cur_accel1,cur_decel1,cur_posP1,cur_velP1,cur_velI1,cur_velD1,cur_volIN1,cur_volLIMIT1,cur_AMP1;
float pos11,pos12,pos13,vel11,vel12,vel13,accel1,decel1,posP1,velP1,velI1,velD1,volIN1,volLIMIT1,amp_limit1;
float cur_pos21,cur_pos22,cur_pos23,cur_vel21,cur_vel22,cur_vel23,cur_accel2,cur_decel2,cur_posP2,cur_velP2,cur_velI2,cur_velD2,cur_volIN2,cur_volLIMIT2,cur_AMP2;
float pos21,pos22,pos23,vel21,vel22,vel23,accel2,decel2,posP2,velP2,velI2,velD2,volIN2,volLIMIT2,amp_limit2;

uint8_t cur_mode,mode;

float ichi11,ichi12,ichi21,ichi22,ichi13,ichi23;
uint8_t number_motor,cur_number_motor ;
float number_motor_select =2 ;
int defaut_select=3;


byte pos11_enable,pos12_enable,pos13_enable,pos21_enable,pos22_enable,pos23_enable;
byte vel11_enable,vel12_enable,vel13_enable,vel21_enable,vel22_enable,vel23_enable;
byte acc1_enable,acc2_enable,dec1_enable,dec2_enable;
byte pid_p1_enable,pid_p2_enable;
byte pid_vp1_enable,pid_vp2_enable;
byte pid_vi1_enable,pid_vi2_enable;
byte pid_vd1_enable,pid_vd2_enable;
byte volin1_enable,volin2_enable;
byte vol_li1_enable,vol_li2_enable;
byte amp_li1_enable,amp_li2_enable;
byte mode_enable,mode_enable_onetime,num_enable,num_enable_onetime;


byte parameter_axis2= 2;
byte parameter_axis1 = 1;
int tani_timer;

byte access3;
byte menuCount  = 1;
byte posCount  = 1;
byte modeCount  = 2;
volatile unsigned int saving  = 0;
bool save_State = false;
byte Kaiten = 0;
int huong =0;
//byte lock;
int i=0;
int ii=0;
byte lock,lock_tani,lock_select;
byte nikai_tani,nikai,select_sl,nikai_select;
float donvi =0.1;
byte tani =2;
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
boolean stringComplete = false;  // whether the string is complete
String inputString;         // a string to hold incoming data


//void checkPosition()
//{
//  encoder->tick(); // just call tick() to check the state.
//}
/////////////////////////////////////////
#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE];
static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;

//request MSP
#define MSP_HEADER = "$M<"
//
#define MSP_MOTOR_MODE 100
#define MSP_MOTOR_POS 101
#define MSP_MOTOR_VEL 102
#define MSP_MOTOR_ACCEL_DECCEL 103
#define MSP_MOTOR_VOL 104
#define MSP_MOTOR_VOLLIMIT 105
#define MSP_MOTOR_PID_P 106
#define MSP_MOTOR_PID_VEL_P 107
#define MSP_MOTOR_PID_VEL_I 108
#define MSP_MOTOR_PID_VEL_D 109
//////////////////
#define MSP_MOTOR_MOTOR_SET 1
#define MSP_MOTOR_MODE_SET 2
#define MSP_MOTOR_SAVE_SET 3
#define MSP_MOTOR_RESET_SET 4
#define MSP_MOTOR_RUN_SET  5

#define MSP_MOTOR_POS11_SET 10
#define MSP_MOTOR_POS12_SET 11
#define MSP_MOTOR_POS13_SET 34
#define MSP_MOTOR_POS21_SET 12
#define MSP_MOTOR_POS22_SET 13
#define MSP_MOTOR_POS23_SET 35

#define MSP_MOTOR_VEL11_SET 14
#define MSP_MOTOR_VEL12_SET 15
#define MSP_MOTOR_VEL13_SET 36
#define MSP_MOTOR_VEL21_SET 16
#define MSP_MOTOR_VEL22_SET 17
#define MSP_MOTOR_VEL23_SET 37

#define MSP_MOTOR_ACCEL1_SET 18
#define MSP_MOTOR_ACCEL2_SET 19
#define MSP_MOTOR_ACCEL3_SET 38
#define MSP_MOTOR_DECCEL1_SET 20
#define MSP_MOTOR_DECCEL2_SET 21
#define MSP_MOTOR_DECCEL3_SET 39

#define MSP_MOTOR_VOL_M1_SET 22
#define MSP_MOTOR_VOL_M2_SET 23

#define MSP_MOTOR_VOLLIMIT_M1_SET 24
#define MSP_MOTOR_VOLLIMIT_M2_SET 25
#define MSP_MOTOR_PID_P_M1_SET 26
#define MSP_MOTOR_PID_P_M2_SET 27
#define MSP_MOTOR_PID_VEL_P_M1_SET 28
#define MSP_MOTOR_PID_VEL_P_M2_SET 29
#define MSP_MOTOR_PID_VEL_I_M1_SET 30
#define MSP_MOTOR_PID_VEL_I_M2_SET 31
#define MSP_MOTOR_PID_VEL_D_M1_SET 32
#define MSP_MOTOR_PID_VEL_D_M2_SET 33
//
void evaluateOtherData(uint8_t sr);
void evaluateCommand(uint8_t c);

static uint8_t read8() {
  return inBuf[indRX++] & 0xff;
}
static uint16_t read16() {
  uint16_t t = read8();
  t += (uint16_t)read8() << 8;
  return t;
}
static uint32_t read32() {
  uint32_t t = read16();
  t += (uint32_t)read16() << 16;
  return t;
}

static void serialize8(uint8_t a) {
  Serial.write(a);
  checksum ^= a;
}
static void serialize16(int16_t a) {
  serialize8((a)&0xFF);
  serialize8((a >> 8) & 0xFF);
}
static void serialize32(uint32_t a) {
  serialize8((a)&0xFF);
  serialize8((a >> 8) & 0xFF);
  serialize8((a >> 16) & 0xFF);
  serialize8((a >> 24) & 0xFF);
}

static void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum = 0;  // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP);
}

static void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

static void headSerialError() {
  headSerialResponse(1, 0);
}

static void tailSerialReply() {
  serialize8(checksum);
}

static void serializeNames(PGM_P s) {
  headSerialReply(strlen_P(s));
  for (PGM_P c = s; pgm_read_byte(c); c++)
    serialize8(pgm_read_byte(c));
  tailSerialReply();
}

static void __attribute__((noinline)) s_struct_w(uint8_t *cb, uint8_t siz) {
  while (siz--) *cb++ = read8();
}

static void s_struct_partial(uint8_t *cb, uint8_t siz) {
  while (siz--) serialize8(*cb++);
}

static void s_struct(uint8_t *cb, uint8_t siz) {
  headSerialReply(siz);
  s_struct_partial(cb, siz);
  tailSerialReply();
}

static void mspAck() {
  headSerialReply(0);
  tailSerialReply();
}

enum MSP_protocol_bytes {
  IDLE,
  HEADER_START,
  HEADER_M,
  HEADER_ARROW,
  HEADER_SIZE,
  HEADER_CMD
};

void serialEvent() {
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
      motor_ready=read8();
      motor_ss=read8();
      break;
    case MSP_MOTOR_POS:
      cur_pos11 = UintToFloat(read32());
      cur_pos12 = UintToFloat(read32());
      cur_pos13 = UintToFloat(read32());
      cur_pos21 = UintToFloat(read32());
      cur_pos22 = UintToFloat(read32());
      cur_pos23 = UintToFloat(read32());
      break;
    case MSP_MOTOR_VEL:
      cur_vel11 = read16();
      cur_vel12 = read16();
      cur_vel13 = read16();
      cur_vel21 = read16();
      cur_vel22 = read16();
      cur_vel23 = read16();
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
      
      cmdMSP = MSP_MOTOR_MODE;
      mspAck();
   
      cmdMSP = MSP_MOTOR_ACCEL_DECCEL;
      mspAck();
     
}
if (i_cout ==3){
      cmdMSP = MSP_MOTOR_PID_VEL_I;
      mspAck();
}
 i_cout+=1;
 if (i_cout >3)i_cout=0;
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


void setup(void) {
  Serial.begin(115200);
  u8g2.begin();
  u8g2.setBitmapMode(1);

 // u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
  u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8g2log.setLineHeightOffset(0); 
  u8g2log.setRedrawMode(0);
 
  pinMode(BUTTON_SELECT_PIN, INPUT); // select button
  pinMode(BUTTON_EXIT_PIN, INPUT); // down button
  pinMode(BUTTON_DOWN_PIN, INPUT); // down button
  pinMode(BUTTON_UP_PIN, INPUT); // down button INPUT_PULLUP

 // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
 //encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);
 // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);
 // register interrupt routine
 // attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

  
}

void loop(void) {

protocol_send_cmd();
  
//  u8g2.clearBuffer();          // clear the internal memory
//  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
//  u8g2.drawStr(0,10,"Hello World!");  // write something to the internal memory
//  u8g2.sendBuffer();          // transfer internal memory to the display
//  delay(1000);  
 static int pos = 0;
 static int menu = 0;
//  encoder->tick(); // just call tick() to check the state.
  Kaiten =0;
//  int newPos = encoder->getPosition();
//      huong =  (int)encoder->getDirection();

////////////////////debouce button
//int reading = digitalRead(BUTTON_SELECT_PIN);
//if (reading != lastButtonState) {
//    // reset the debouncing timer
//    lastDebounceTime = millis();
//  }
// if ((millis() - lastDebounceTime) > debounceDelay) {
//
//    if (reading != buttonState) {
//      buttonState = reading;
//
//      // only toggle the LED if the new button state is HIGH
//      if (buttonState == LOW) {
//       //////////////
//        select_button_state = LOW;
//       //////////////
//      }
//    }
//  }



if (digitalRead(BUTTON_UP_PIN) == LOW){
  Kaiten = 1;
 huong =1;
 if(current_screen == 0){
  item_selected = item_selected -1;}
}

if (digitalRead(BUTTON_DOWN_PIN) == LOW){
  Kaiten = 1;
 huong =-1;
 if(current_screen == 0){
  item_selected = item_selected +1;
 }
}
if (digitalRead(BUTTON_UP_PIN) == LOW && digitalRead(BUTTON_DOWN_PIN) == LOW)
{
  huong = 0;
  }
if (digitalRead(BUTTON_UP_PIN) == HIGH && digitalRead(BUTTON_DOWN_PIN) == HIGH)
{
  huong = 0;
  }
////////////////////simulater      
/*  if (pos != newPos) {
 
   Kaiten = 1;
   
    if ((pos != newPos)&& (huong == 1)&&(current_screen == 0)){
    item_selected = item_selected -1;
   
  
}
if ((pos != newPos)&& (huong == -1)&&(current_screen == 0)){
    item_selected = item_selected +1;
    
 
}  
  } 
*/
 


   
   

   if(item_selected >= 0 && item_selected <=5)  {item_sel_previous = item_selected - 1;item_sel_next = item_selected + 1;  }
   if (item_sel_previous < 0) {item_sel_previous = 5 ;}
   if (item_sel_next > 5) {item_sel_next = 0;}
   
   if(item_selected < 0) { item_selected =5;item_sel_previous = item_selected - 1;item_sel_next = 0;}
   if(item_selected > 5)  {item_selected =0;item_sel_previous = 5;item_sel_next = item_selected+1;}
   
   
   
  
  
  //if (item_sel_previous == 0) {item_sel_previous = NUM_ITEMS ;} // previous item would be below first = make it the last
  
//  if (item_sel_next >= NUM_ITEMS) {item_sel_next = 0;} // next item would be after last = make it the first
   
////////////////////////////////////////////
//digitalRead(BUTTON_SELECT_PIN)
//select_button_state
  if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_select_clicked == 0)) { // select button clicked, jump between screens
    button_select_clicked = 1; // set button to clicked to only perform the action once
      if ((current_screen == 0)&&(item_selected==0)) {current_screen = 1; delay(150);}//pos1
      if ((current_screen == 0)&&(item_selected==1)) {current_screen = 2;delay(150);}//pos2
      if ((current_screen == 0)&&(item_selected==2)) {current_screen = 3;delay(150);}//run test
      if ((current_screen == 0)&&(item_selected==3)) {current_screen = 4;delay(150);}//set control
      if ((current_screen == 0)&&(item_selected==4)) {current_screen = 5;delay(150);}//number motor
      if ((current_screen == 0)&&(item_selected==5)) {current_screen = 45;delay(150);}//set defaut
      //select_button_state = HIGH;
  
  }
  
if ((current_screen == 1)&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 0;
   button_clicked = 1; } 
if ((current_screen == 2)&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 0;
   button_clicked = 1; } 
if ((current_screen == 3)&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 0;
   button_clicked = 1; } 
if ((current_screen == 4)&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 0;
   button_clicked = 1; } 
if ((current_screen == 5)&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 0;
   button_clicked = 1; } 
if ((current_screen == 45)&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 0;
   button_clicked = 1; } 
if ((((current_screen >= 6)&&(current_screen <= 17))||(current_screen == 30)||(current_screen == 31)||(current_screen == 42))&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 1;
  saving =0;
   button_clicked = 1; } 
if ((((current_screen >= 18)&&(current_screen <= 29))||(current_screen == 32)||(current_screen == 33)||(current_screen == 43))&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 2;
  saving =0;
   button_clicked = 1; }





 if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_select_clicked == 1)) { // unclick 
    button_select_clicked = 0;
  }
if ((digitalRead(BUTTON_EXIT_PIN) == HIGH) && (button_clicked == 1)) { // unclick 
    button_clicked = 0;
    access3  = 0;
    saving =0;
  }

 


 
     u8g2.clearBuffer();  // clear buffer for storing display content in RAM

     if (current_screen == 0) { // MENU SCREEN

      // selected item background
      u8g2.drawXBMP(0, 22, 128, 21, bitmap_item_sel_outline);

      // draw previous item as icon + label
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(25, 15, menu_items[item_sel_previous]); 
      u8g2.drawXBMP( 4, 2, 16, 16, bitmap_icons[item_sel_previous]);          

      // draw selected item as icon + label in bold font
      u8g2.setFont(u8g_font_7x14B); //  u8g2_font_ncenB08_tr 
      u8g2.drawStr(25, 15+20+2, menu_items[item_selected]);   
      u8g2.drawXBMP( 4, 24, 16, 16, bitmap_icons[item_selected]);     

      // draw next item as icon + label
      u8g2.setFont(u8g2_font_ncenB08_tr);     //u8g_font_7x14
      u8g2.drawStr(25, 15+20+20+2+2, menu_items[item_sel_next]);   
      u8g2.drawXBMP( 4, 46, 16, 16, bitmap_icons[item_sel_next]);  

      // draw scrollbar background
      u8g2.drawXBMP(128-8, 0, 8, 64, bitmap_scrollbar_background);

      // draw scrollbar handle
      u8g2.drawBox(125, 64/NUM_ITEMS * item_selected, 3, 64/NUM_ITEMS); 

      // draw upir logo
    //  u8g2.drawXBMP(128-16-4, 64-4, 16, 4, upir_logo);               

    } 
//################################PAGE1###########################################
//################################################################################
     if (current_screen == 1) { //pos1


      if ((Kaiten == 1)&& (huong == 1)) {item_select1--;}
       if ((Kaiten == 1)&& (huong == -1)){item_select1++;}  
       
      if(item_select1 >= 0 && item_select1 <= (NUMBER_ITEMS-1))  {item_prev1 = item_select1 - 1;item_next1 = item_select1 + 1;  }
      if (item_prev1 < 0) {item_prev1 = (NUMBER_ITEMS-1) ;}
      if (item_next1 > (NUMBER_ITEMS-1)) {item_next1 = 0;}
   
     if(item_select1 < 0) { item_select1 =(NUMBER_ITEMS-1);item_prev1 = item_select1 - 1;item_next1 = 0;}
     if(item_select1 > (NUMBER_ITEMS-1))  {item_select1 =0;item_prev1 = (NUMBER_ITEMS-1);item_next1 = item_select1+1;}
     
     
       u8g2.clearBuffer();
      // draw previous item as icon + label
            // selected item background
      u8g2.setFont(u8g2_font_ncenB12_tf);      
      u8g2.setFontDirection(1);
      u8g2.drawStr(3, 3, "AXIS 1");//90do 
      u8g2.setFontDirection(0);      
      u8g2.drawLine(19, 0, 19, 64);
      u8g2.drawRFrame(22,23,103,20,6);
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(25, 15, axis1_items[item_prev1]); 
      // draw selected item as icon + label in bold font
      u8g2.setFont(u8g_font_7x14B); //  u8g2_font_ncenB08_tr 
      u8g2.drawStr(25, 15+20+2, axis1_items[item_select1]);   
      // draw next item as icon + label
      u8g2.setFont(u8g2_font_ncenB08_tr);     //u8g_font_7x14
      u8g2.drawStr(25, 15+20+20+2+2, axis1_items[item_next1]);   
      // draw scrollbar background
      u8g2.drawXBMP(128-8, 0, 8, 64, bitmap_scrollbar_background);
      // draw scrollbar handle
      u8g2.drawBox(125, 64/NUMBER_ITEMS * item_select1, 3, 64/NUMBER_ITEMS); 


      if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_click1 == 0)) { // select button clicked, jump between screens
          button_click1 = 1; // set button to clicked to only perform the action once
      if ((current_screen == 1)&&(item_select1==0)) {current_screen = 6;}
      if ((current_screen == 1)&&(item_select1==1)) {current_screen = 7;}
      if ((current_screen == 1)&&(item_select1==2)) {current_screen = 30;}
      if ((current_screen == 1)&&(item_select1==3)) {current_screen = 8;}
      if ((current_screen == 1)&&(item_select1==4)) {current_screen = 9;}
      if ((current_screen == 1)&&(item_select1==5)) {current_screen = 31;}
      if ((current_screen == 1)&&(item_select1==6)) {current_screen = 10;}
      if ((current_screen == 1)&&(item_select1==7)) {current_screen = 11;}
      if ((current_screen == 1)&&(item_select1==8)) {current_screen = 12;}
      
      if ((current_screen == 1)&&(item_select1==9)) {current_screen = 13;}
      if ((current_screen == 1)&&(item_select1==10)) {current_screen = 14;}
      if ((current_screen == 1)&&(item_select1==11)) {current_screen = 42;}
      if ((current_screen == 1)&&(item_select1==12)) {current_screen = 15;}
      //update
      if ((current_screen == 1)&&(item_select1==13)) {current_screen = 16;}
      if ((current_screen == 1)&&(item_select1==14)) {current_screen = 17;}
      
      
      
  }
  

      if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_click1 == 1)) { // unclick 
       button_click1 = 0;
       }



    }
//################################PAGE2###########################################
//################################################################################
     if (current_screen == 2) {//pos2

if ((Kaiten == 1)&& (huong == 1)) {item_select2--;}
       if ((Kaiten == 1)&& (huong == -1)){item_select2++;}  
       
      if(item_select2 >= 0 && item_select2 <=(NUMBER_ITEMS-1))  {item_prev2 = item_select2 - 1;item_next2 = item_select2 + 1;  }
      if (item_prev2 < 0) {item_prev2 = (NUMBER_ITEMS-1) ;}
      if (item_next2 > (NUMBER_ITEMS-1)) {item_next2 = 0;}
   
     if(item_select2 < 0) { item_select2 =(NUMBER_ITEMS-1);item_prev2 = item_select2 - 1;item_next2 = 0;}
     if(item_select2 > (NUMBER_ITEMS-1))  {item_select2 =0;item_prev2 = (NUMBER_ITEMS-1);item_next2 = item_select2+1;}
     
     
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


      if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_click2 == 0)) { // select button clicked, jump between screens
          button_click2 = 1; // set button to clicked to only perform the action once
      if ((current_screen == 2)&&(item_select2==0)) {current_screen = 18;}
      if ((current_screen == 2)&&(item_select2==1)) {current_screen = 19;}
      if ((current_screen == 2)&&(item_select2==2)) {current_screen = 32;}
      if ((current_screen == 2)&&(item_select2==3)) {current_screen = 20;}
      if ((current_screen == 2)&&(item_select2==4)) {current_screen = 21;}
      if ((current_screen == 2)&&(item_select2==5)) {current_screen = 33;}
      if ((current_screen == 2)&&(item_select2==6)) {current_screen = 22;}
      if ((current_screen == 2)&&(item_select2==7)) {current_screen = 23;}
      if ((current_screen == 2)&&(item_select2==8)) {current_screen = 24;}
      
      if ((current_screen == 2)&&(item_select2==9)) {current_screen = 25;}
      if ((current_screen == 2)&&(item_select2==10)) {current_screen = 26;}
      if ((current_screen == 2)&&(item_select2==11)) {current_screen = 43;}
      if ((current_screen == 2)&&(item_select2==12)) {current_screen = 27;}
      //update
      if ((current_screen == 2)&&(item_select2==13)) {current_screen = 28;}
      if ((current_screen == 2)&&(item_select2==14)) {current_screen = 29;}
     
      
      
  }
  

      if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_click2 == 1)) { // unclick 
       button_click2 = 0;
       }



        
       
    }
//#######################################PAGE4####################################
//################################################################################
     if (current_screen == 3) {// run
       position_run();
       selection_check();   
    }   
//#######################################PAGE5####################################
//################################################################################
     if (current_screen == 4) { //mode
        if (mode_enable_onetime== 0) { mode_enable = 1;Serial.print("w");Serial.write('\n');}
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

  if ((modeCount == 2)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
  mode=1;//PLC IO
  cmdMSP = MSP_MOTOR_MODE_SET;
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
  if ((modeCount == 3)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
  mode=2;//pulse
  cmdMSP = MSP_MOTOR_MODE_SET;
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
  if ((modeCount == 4)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
  mode=3;//serial
  cmdMSP = MSP_MOTOR_MODE_SET;
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
if ((modeCount == 5)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {

   
   current_screen = 0;
   mode_enable_onetime=0;
   modeCount = 0;

      
     }
 }   
//######################################PAGE45####################################
//################################################################################
     if (current_screen == 5) { //set number motor
       if (num_enable_onetime== 0) { num_enable = 1;Serial.print("X");Serial.write('\n');}
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

  if ((number_motor_select == 2)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
  number_motor=1;//one
  cmdMSP = MSP_MOTOR_MOTOR_SET;
      headSerialReply(1);
      serialize8(number_motor);
      tailSerialReply();
  delay(10);
  if (num_enable_onetime== 0) { num_enable = 1;Serial.print("X");Serial.write('\n');}
      num_enable_onetime = 1;
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(10, 25, 0x2611);  
  }
  if ((number_motor_select == 3)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
  number_motor=2;//two
  cmdMSP = MSP_MOTOR_MOTOR_SET;
      headSerialReply(1);
      serialize8(number_motor);
      tailSerialReply();
  delay(10);
  if (num_enable_onetime== 0) { num_enable = 1;Serial.print("X");Serial.write('\n');}
      num_enable_onetime = 1;
  }
  
if ((number_motor_select == 4)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {

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
//#######################################PAGE46###################################
//################################################################################
     if (current_screen == 45) { //set defaut motor

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

  if ((defaut_select == 0)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
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
  if ((defaut_select == 1)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
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
 
if ((defaut_select == 2)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {

   current_screen = 0;
   defaut_select = 0;
  }
}

     
//################################################################################
//###########################################CHU Y################################
//################################################################################

if (current_screen != 0 && current_screen != 1 && current_screen != 2 && current_screen != 3 && current_screen != 4 && current_screen != 5&& current_screen != 45) {
     staticMenu();
      menuCheck();
}

  u8g2.sendBuffer(); // send buffer from RAM to display controller

  


//  lastButtonState = reading;

//  if(motor_ready == 1 && motor_ss ==0){
//    char c;
//    u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
//    
//    while(Serial.available() >0){
//      c = Serial.read();              // read from Serial Monitor
//      u8g2.setCursor(5,5);
//      u8g2.print(c);               // print to display
//      u8g2.sendBuffer();
//    }
// }
//  if(motor_ready == 0 && motor_ss ==1){
//    u8g2.clearBuffer();
//}

 
// serialEvent();
}
void position_run(){

 
   u8g2.setFont(u8g_font_7x14);
   u8g2.drawStr(10, 12, "SELECT Axis:");
   if (select_sl ==  1){ u8g2.drawStr(95, 12, "A_1"); }
   if (select_sl ==  2){ u8g2.drawStr(95, 12, "A_2"); }
  

  u8g2.drawStr(10, 25, "<POSITION_1>"); 
 // u8g2.drawStr(80, 25, "RUN"); 
  u8g2.drawStr(10, 38, "<POSITION_2>"); 
 // u8g2.drawStr(80, 38, "RUN"); 
  u8g2.drawStr(10, 51, "<POSITION_3>"); 
   //
  u8g2.drawStr(10, 64, "EXIT"); 
   //
  u8g2.drawStr(2, (posCount * 13) - 1, ">"); 

   
}
void selection_check(){
  
 if ((lock_select !=1)){
   if ((Kaiten == 1)&& (huong == 1)){
    posCount = posCount -1;
  
}
if ((Kaiten == 1)&& (huong == -1)){
    posCount = posCount +1;

}  

    if(posCount <= 1) { posCount =1;}
    if(posCount > 5)  {posCount =5;}

 }

 if ((posCount == 1)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)&&(clicked_select == 0)&&(nikai_select == 0)) {
   lock_select=1;
   clicked_select =1;
  }

  if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (clicked_select == 1)) { // unclick 
    clicked_select = 0;
    nikai_select =1;
  }
 
  if (lock_select == 1){
     if ((nikai_select == 1)&&(digitalRead(BUTTON_SELECT_PIN) == LOW)){lock_select =0;nikai_select =0;delay(10);}
     u8g2.drawFrame(9,0, 79, 13); 
       if ((Kaiten == 1)&& (huong == 1)) {select_sl--;}
       if ((Kaiten == 1)&& (huong == -1)){select_sl++;}  
       if(select_sl >= 2) { select_sl =2;}
       if(select_sl <= 1)  {select_sl =1;}
      
    }
 
 if ((posCount == 2)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
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
    
  }
//  Serial.write("z1");
//  Serial.write('\n');
//  }
//  if (select_sl ==2 ){
//  Serial.write("z2");
//  Serial.write('\n');
//  }

//  }



  if ((posCount == 3)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
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

  }
  

if ((posCount == 4)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {//pos2
     if (select_sl ==1 ){
  cmdMSP = MSP_MOTOR_RUN_SET;
      headSerialReply(1);
      serialize8(3);
      tailSerialReply();
  }
 if (select_sl ==2 ){
  cmdMSP = MSP_MOTOR_RUN_SET;
      headSerialReply(1);
      serialize8(13);
      tailSerialReply();
  }
  
  }


if ((posCount == 5)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {//exit
   current_screen = 0;
   posCount = 0;
  }
   




}
void staticMenu() {
  u8g2.setFont(u8g_font_7x14);
  tani_timer = millis();
// cur_pos31 = 3.6,cur_pos32,cur_vel13,cur_vel32,cur_accel3,cur_decel3,cur_posP3,cur_velP3,cur_velI3,cur_volIN3,cur_volLIMIT3,cur_AMP3;
// pos31,pos32,vel31,vel32,accel3,decel3,posP3,velP3,velI3,volIN3,volLIMIT3,amp_limit3;
  if (current_screen ==  6){
  u8g2.drawStr(10, 12, "Position1_1"); 
  u8g2.drawStr(10, 25, "Pos11_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_pos11,2);
  
  u8g2.drawStr(10, 38, "Pos11_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(pos11,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  
  }
//////////////////
 if (current_screen ==  30){
  u8g2.drawStr(10, 12, "Position1_3"); 
  u8g2.drawStr(10, 25, "Pos13_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_pos13,2);
  
  u8g2.drawStr(10, 38, "Pos13_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(pos13,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  
  }
///////////////////



  
  if (current_screen ==  7){
  u8g2.drawStr(10, 12, "Position1_2"); 
  u8g2.drawStr(10, 25, "Pos12_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_pos12,2);
  
  u8g2.drawStr(10, 38, "Pos12_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(pos12,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }
   if (current_screen == 8){
  u8g2.drawStr(10, 12, "Velocity1_1"); 
  u8g2.drawStr(10, 25, "Vel11_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_vel11,2);
  
  u8g2.drawStr(10, 38, "Vel11_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(vel11,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }
   if (current_screen == 9){
  u8g2.drawStr(10, 12, "Velocity1_2");
   u8g2.drawStr(10, 25, "Vel12_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_vel12,2);
  
  u8g2.drawStr(10, 38, "Vel12_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(vel12,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }
///////////////////
if (current_screen == 31){
  u8g2.drawStr(10, 12, "Velocity1_3");
   u8g2.drawStr(10, 25, "Vel13_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_vel13,2);
  
  u8g2.drawStr(10, 38, "Vel13_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(vel13,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }
  ///////////////////
   if (current_screen == 10){
  u8g2.drawStr(10, 12, "Accell_1"); 
   u8g2.drawStr(10, 25, "Acc1_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_accel1,2);
  
  u8g2.drawStr(10, 38, "Acc1_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(accel1,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
   if (current_screen == 11){
  u8g2.drawStr(10, 12, "Decell_1"); 
   u8g2.drawStr(10, 25, "Dec1_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_decel1,2);
  
  u8g2.drawStr(10, 38, "Dec1_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(decel1,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
  if (current_screen == 12){
  u8g2.drawStr(10, 12, "PID_pos_P1");
   u8g2.drawStr(10, 25, "Ppos1_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_posP1,2);
  
  u8g2.drawStr(10, 38, "Ppos1_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(posP1,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }
    if (current_screen == 13){
  u8g2.drawStr(10, 12, "PID_Vel_P1"); 
     u8g2.drawStr(10, 25, "Pvel1_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_velP1,2);
  
  u8g2.drawStr(10, 38, "Pvel1_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(velP1,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
    if (current_screen == 14){
  u8g2.drawStr(10, 12, "PID_vel_I1");
     u8g2.drawStr(10, 25, "Ivel1_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_velI1,2);
  
  u8g2.drawStr(10, 38, "Ivel1_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(velI1,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }

//
if (current_screen == 42){
  u8g2.drawStr(10, 12, "PID_vel_D1");
     u8g2.drawStr(10, 25, "Dvel1_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_velD1,3);
  
  u8g2.drawStr(10, 38, "Dvel1_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(velD1,3);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
  //
  
    if (current_screen == 15){
  u8g2.drawStr(10, 12, "VolIN_1(V)");
     u8g2.drawStr(10, 25, "VIN1_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_volIN1,2);
  
  u8g2.drawStr(10, 38, "VIN1_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(volIN1,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
  if (current_screen == 16){
  u8g2.drawStr(10, 12, "VolLI_1(V)"); 
     u8g2.drawStr(10, 25, "VLI1_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_volLIMIT1,2);
  
  u8g2.drawStr(10, 38, "VLI1_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(volLIMIT1,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
  if (current_screen == 17){
      
       
       cmdMSP = MSP_MOTOR_SAVE_SET;
      headSerialReply(1);
      serialize8(1);
      tailSerialReply();
       u8g2.setFont(u8g2_font_unifont_t_symbols);
       u8g2.drawGlyph(50, 25, 0x2611);  
       u8g2.clearDisplay();  
       u8g2.setFont(u8g_font_7x14);
       u8g2.drawStr(25, 12, "Please reset ");
       u8g2.drawStr(15, 25, "driver to apply"); 
       u8g2.drawStr(25, 40, "ASIX 1 SAVED");
       u8g2.sendBuffer();
       delay(50000);
       u8g2.clearDisplay();}
  //--------------------------------------------
  if (current_screen ==  18){
  u8g2.drawStr(10, 12, "Position2_1"); 
  u8g2.drawStr(10, 25, "Pos21_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_pos21,2);
  
  u8g2.drawStr(10, 38, "Pos21_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(pos21,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  
  }
  if (current_screen ==  19){
  u8g2.drawStr(10, 12, "Position2_2"); 
  u8g2.drawStr(10, 25, "Pos22_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_pos22,2);
  
  u8g2.drawStr(10, 38, "Pos22_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(pos22,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }


if (current_screen ==  32){
  u8g2.drawStr(10, 12, "Position2_3"); 
  u8g2.drawStr(10, 25, "Pos23_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_pos23,2);
  
  u8g2.drawStr(10, 38, "Pos23_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(pos23,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }

  
   if (current_screen == 20){
  u8g2.drawStr(10, 12, "Velocity2_1"); 
  u8g2.drawStr(10, 25, "Vel21_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_vel21,2);
  
  u8g2.drawStr(10, 38, "Vel21_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(vel21,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }
   if (current_screen == 21){
  u8g2.drawStr(10, 12, "Velocity2_2");
   u8g2.drawStr(10, 25, "Vel22_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_vel22,2);
  
  u8g2.drawStr(10, 38, "Vel22_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(vel22,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }


   if (current_screen == 33){
  u8g2.drawStr(10, 12, "Velocity2_3");
   u8g2.drawStr(10, 25, "Vel23_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_vel23,2);
  
  u8g2.drawStr(10, 38, "Vel23_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(vel23,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }
  
   if (current_screen == 22){
  u8g2.drawStr(10, 12, "Accell_2"); 
   u8g2.drawStr(10, 25, "Acc2_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_accel2,2);
  
  u8g2.drawStr(10, 38, "Acc2_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(accel2,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
   if (current_screen == 23){
  u8g2.drawStr(10, 12, "Decell_2"); 
   u8g2.drawStr(10, 25, "Dec2_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_decel2,2);
  
  u8g2.drawStr(10, 38, "Dec2_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(decel2,2);
  
//  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
  if (current_screen ==  24){
  u8g2.drawStr(10, 12, "PID_pos_P2");
   u8g2.drawStr(10, 25, "Ppos2_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_posP2,2);
  
  u8g2.drawStr(10, 38, "Ppos1_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(posP2,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
  }
    if (current_screen == 25){
  u8g2.drawStr(10, 12, "PID_Vel_P2"); 
     u8g2.drawStr(10, 25, "Pvel2_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_velP2,2);
  
  u8g2.drawStr(10, 38, "Pvel2_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(velP2,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
    if (current_screen == 26){
  u8g2.drawStr(10, 12, "PID_vel_I2");
     u8g2.drawStr(10, 25, "Ivel2_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_velI2,2);
  
  u8g2.drawStr(10, 38, "Ivel2_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(velI2,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }

if (current_screen == 43){
  u8g2.drawStr(10, 12, "PID_vel_D2");
     u8g2.drawStr(10, 25, "Dvel2_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_velD2,3);
  
  u8g2.drawStr(10, 38, "Dvel2_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(velD2,3);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
  //

  
    if (current_screen == 27){
  u8g2.drawStr(10, 12, "VolIN_2(V)");
     u8g2.drawStr(10, 25, "VIN2_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_volIN2,2);
  
  u8g2.drawStr(10, 38, "VIN2_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(volIN2,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
  if (current_screen == 28){
  u8g2.drawStr(10, 12, "VolLI_2(V)"); 
     u8g2.drawStr(10, 25, "VLI2_cur:"); 
  u8g2.setCursor(80,25);
  u8g2.print(cur_volLIMIT2,2);
  
  u8g2.drawStr(10, 38, "VLI2_set:"); 
  u8g2.setCursor(80,38);
  u8g2.print(volLIMIT2,2);
  
  u8g2.drawStr(10, 51, ""); 
//  if (saving  ==1 ) {
//    u8g2.drawStr(60, 51, "OK");
//    save_State = true;
//  } else {
//    u8g2.drawStr(60, 51, "NotOK");
//    save_State = false;
//  }
 u8g2.drawStr(10, 64, "Exit"); 

  u8g2.drawStr(2, (menuCount * 13) - 1, ">"); }
  if (current_screen == 29){

    
 
       
       cmdMSP = MSP_MOTOR_SAVE_SET;
      headSerialReply(1);
      serialize8(2);
      tailSerialReply();
       u8g2.setFont(u8g2_font_unifont_t_symbols);
       u8g2.drawGlyph(50, 25, 0x2611);  
       u8g2.clearDisplay();  
       u8g2.setFont(u8g_font_7x14);
       u8g2.drawStr(25, 12, "Please reset ");
       u8g2.drawStr(15, 25, "driver to apply"); 
       u8g2.drawStr(25, 40, "ASIX 2 SAVED"); 
       u8g2.sendBuffer();
       delay(50000);
       u8g2.clearDisplay();
     
  }
 
  //--------------------------------------------

  //--------------------------------------------
 //----------------------------------------------
  
  
        if (tani ==1){ u8g2.drawStr(89, 12, "x0.01");donvi=0.01; 
                       //delay(100);
                       u8g2.drawStr(89, 12, "x0.0_");donvi=0.01; 
        }
        if (tani ==2){ u8g2.drawStr(89, 12, "x0.10");donvi=0.1;
                      // delay(100);
                       u8g2.drawStr(89, 12, "x0._0");
        }
        if (tani ==3){ u8g2.drawStr(89, 12, "x1.00");donvi=1; 
                       //delay(100);
                       u8g2.drawStr(89, 12, "x_.00");
        }
  
  //---------------------------------
  
   
  
}
void menuCheck() {

 
    
 if ((lock != 1)&&(lock_tani !=1)){
   if ((Kaiten == 1)&& (huong == 1)){
    menuCount = menuCount -1;
  //  Serial.print(" akaka:");
}
if ((Kaiten == 1)&& (huong == -1)){
    menuCount = menuCount +1;
 //   Serial.print(" ahuhu:");
}  

    if(menuCount <= 1) { menuCount =1;}
    if(menuCount > 5)  {menuCount =5;}

 }

  if ((menuCount == 1)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)&&(clicked_tani == 0)&&(nikai_tani == 0)) {
   lock_tani=1;
   clicked_tani =1;
  }

  if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (clicked_tani == 1)) { // unclick 
    clicked_tani = 0;
    nikai_tani =1;
  }
 
  if (lock_tani == 1){
     if ((nikai_tani == 1)&&(digitalRead(BUTTON_SELECT_PIN) == LOW)){lock_tani =0;nikai_tani =0;delay(10);}
     u8g2.drawFrame(9,0, 79, 13);   
       if ((Kaiten == 1)&& (huong == 1)) {tani--;}
       if ((Kaiten == 1)&& (huong == -1)){tani++;}  
       if(tani >= 3) { tani =3;}
       if(tani <= 1)  {tani =1;}
        if (tani ==1){donvi=0.01;i=0 ; }
        if (tani ==2){donvi=0.1;i=0 ; }
        if (tani ==3){donvi=1;i=0 ;}
    }
 
  

  if ((menuCount == 3)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)&&(clicked == 0)&&(nikai == 0)){
   lock =1;
   clicked =1;
   pos11 = cur_pos11 ;
   pos12 = cur_pos12 ;
   pos13 = cur_pos13 ;
   vel11 = cur_vel11 ;
   vel12 = cur_vel12 ;
   vel13 = cur_vel13 ;
   accel1= cur_accel1;
   decel1= cur_decel1;
   posP1 = cur_posP1;
   velP1 = cur_velP1;
   velI1 = cur_velI1;
   volIN1= cur_volIN1;
   volLIMIT1=cur_volLIMIT1;
   
   velD1 = cur_velD1;
   //------------------------
   pos21 = cur_pos21 ;
   pos22 = cur_pos22 ;
   pos23 = cur_pos23 ;
   vel21 = cur_vel21 ;
   vel22 = cur_vel22 ;
   vel23 = cur_vel23 ;
   accel2= cur_accel2;
   decel2= cur_decel2;
   posP2 = cur_posP2;
   velP2 = cur_velP2;
   velI2 = cur_velI2;
   volIN2= cur_volIN2;
   volLIMIT2=cur_volLIMIT2;
   
   velD2 = cur_velD2;
   //-----------------------------
  

   
  }
//if ((menuCount == 3)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)&&(clicked == 0)&&(nikai == 0)&& (access3 ==0)){  
//   u8g2.clearDisplay();
//   u8g2.setFont(u8g_font_7x14);
//   u8g2.drawStr(25, 12, "ACCESS_DENY:");
//   u8g2.drawStr(15, 25, "CURRENT VALUE"); 
//   u8g2.drawStr(35, 38, "NOT READ"); 
//   u8g2.drawStr(45, 51, "<!!!>"); 
//   u8g2.sendBuffer();
//   delay(2000);
//   u8g2.clearDisplay();
//   }
  
  if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (clicked == 1)) { // unclick 
    clicked = 0;
    nikai =1;
  }

  // cur_pos31 = 3.6,cur_pos32,cur_vel13,cur_vel32,cur_accel3,cur_decel3,cur_posP3,cur_velP3,cur_velI3,cur_volIN3,cur_volLIMIT3,cur_AMP3;
  // pos31,pos32,vel31,vel32,accel3,decel3,posP3,velP3,velI3,volIN3,volLIMIT3,amp_limit3;
   if (lock == 1){
     if ((nikai == 1)&&(digitalRead(BUTTON_SELECT_PIN) == LOW)){lock =0;nikai =0;delay(10);}
       u8g2.drawFrame(9,26, 63, 13);
      //volum increse 
       if ((Kaiten == 1)&& (huong == 1)) {ii= 1;
       saving =0;
       if(current_screen == 6){
                                pos11 = pos11 +ii*donvi;i=0;
                                if(pos11 >= 100) { pos11 =100;}
                                cmdMSP = MSP_MOTOR_POS11_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos11));
                                tailSerialReply();
                              }
       if(current_screen == 7){pos12 = pos12 +ii*donvi;i=0;
                                if(pos12 >= 100) { pos12 =100;}
                                cmdMSP = MSP_MOTOR_POS12_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos12));
                                tailSerialReply();
       }
       if(current_screen == 8){vel11 = vel11 +ii*donvi*100;i=0;
                                if(vel11 >= 1000) { vel11 =1000;}
                                cmdMSP = MSP_MOTOR_VEL11_SET;
                                headSerialReply(2);
                                serialize16(vel11);
                                tailSerialReply();}
       if(current_screen == 9){vel12 = vel12 +ii*donvi*100;i=0;
                                if(vel12 >= 1000) { vel12 =1000;}
                                cmdMSP = MSP_MOTOR_VEL12_SET;
                                headSerialReply(2);
                                serialize16(vel12);
                                tailSerialReply();}
       if(current_screen == 10){accel1 = accel1 +ii*donvi*100;i=0;
                                if(accel1 >= 2000) { accel1 =2000;}
                                cmdMSP = MSP_MOTOR_ACCEL1_SET;
                                headSerialReply(2);
                                serialize16(accel1);
                                tailSerialReply();}
       if(current_screen == 11){decel1 = decel1 +ii*donvi*100;i=0;
                                if(decel1 >= 2000) { decel1 =2000;}
                                cmdMSP = MSP_MOTOR_DECCEL1_SET;
                                headSerialReply(2);
                                serialize16(decel1);
                                tailSerialReply();}
       if(current_screen == 12){posP1 = posP1 +ii*donvi;i=0; 
                                if(posP1 >= 500) { posP1 =500;}
                                cmdMSP = MSP_MOTOR_PID_P_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(posP1));
                                tailSerialReply();}
       if(current_screen == 13){velP1 = velP1 +ii*donvi;i=0;
                                if(velP1 >= 500) { velP1 =500;}
                                cmdMSP = MSP_MOTOR_PID_VEL_P_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velP1));
                                tailSerialReply();}
       if(current_screen == 14){velI1 = velI1 +ii*donvi;i=0;
                                if(velI1 >= 500) { velI1 =500;}
                                cmdMSP = MSP_MOTOR_PID_VEL_I_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velI1));
                                tailSerialReply();}
       if(current_screen == 15){volIN1 = volIN1 +ii*donvi;i=0;
                                if(volIN1 >= 500) { volIN1 =500;}
                                cmdMSP = MSP_MOTOR_VOL_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(volIN1));
                                tailSerialReply();}
       if(current_screen == 16){volLIMIT1 = volLIMIT1 +ii*donvi;i=0;
                                if(volLIMIT1 >= 500) { volLIMIT1 =500;}
                                cmdMSP = MSP_MOTOR_VOLLIMIT_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(volLIMIT1));
                                tailSerialReply();}
       if(current_screen == 17){amp_limit1 = amp_limit1 +ii*donvi;i=0;
       
       //Serial.print("Z");Serial.print(amp_limit1,2);Serial.write('\n');
       }
       //----------------------------------------
       if(current_screen == 18){pos21 = pos21 +ii*donvi;i=0;
                                if(pos21 >= 100) { pos21 =100;}
                                cmdMSP = MSP_MOTOR_POS21_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos21));
                                tailSerialReply();}
       if(current_screen == 19){pos22 = pos22 +ii*donvi;i=0;
                                if(pos22 >= 100) { pos22 =100;}
                                cmdMSP = MSP_MOTOR_POS22_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos22));
                                tailSerialReply();}
       if(current_screen == 20){vel21 = vel21 +ii*donvi*100;i=0;
                                if(vel21 >= 1000) { vel21 =1000;}
                                cmdMSP = MSP_MOTOR_VEL21_SET;
                                headSerialReply(2);
                                serialize16(vel21);
                                tailSerialReply();}
       if(current_screen == 21){vel22 = vel22 +ii*donvi*100;i=0;
                                if(vel22 >= 1000) { vel22 =1000;}
                                cmdMSP = MSP_MOTOR_VEL22_SET;
                                headSerialReply(2);
                                serialize16(vel22);
                                tailSerialReply();}
       if(current_screen == 22){accel2 = accel2 +ii*donvi*100;i=0;
                                if(accel2 >= 2000) { accel2 =2000;}
                                cmdMSP = MSP_MOTOR_ACCEL2_SET;
                                headSerialReply(2);
                                serialize16(accel2);
                                tailSerialReply();}
       if(current_screen == 23){decel2 = decel2 +ii*donvi*100;i=0;
                                if(decel2 >= 2000) { decel2 =2000;}
                                cmdMSP = MSP_MOTOR_DECCEL2_SET;
                                headSerialReply(2);
                                serialize16(decel2);
                                tailSerialReply();}
       if(current_screen == 24){posP2 = posP2 +ii*donvi;i=0;
                                if(posP2 >= 500) { posP2 =500;}
                                cmdMSP = MSP_MOTOR_PID_P_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(posP2));
                                tailSerialReply();}
       if(current_screen == 25){velP2 = velP2 +ii*donvi;i=0;
                                if(velP2 >= 500) { velP2 =500;}
                                cmdMSP = MSP_MOTOR_PID_VEL_P_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velP2));
                                tailSerialReply();}
       if(current_screen == 26){velI2 = velI2 +ii*donvi;i=0;
                                if(velI2 >= 500) { velI2 =500;}
                                cmdMSP = MSP_MOTOR_PID_VEL_I_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velI2));
                                tailSerialReply();}
       if(current_screen == 27){volIN2 = volIN2 +ii*donvi;i=0;
                                if(volIN2 >= 500) { volIN2 =500;}
                                cmdMSP = MSP_MOTOR_VOL_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(volIN2));
                                tailSerialReply();}
       if(current_screen == 28){volLIMIT2 = volLIMIT2 +ii*donvi;i=0;
                                if(volLIMIT2 >= 500) { volLIMIT2 =500;}
                                cmdMSP = MSP_MOTOR_VOLLIMIT_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(volLIMIT2));
                                tailSerialReply();}
       if(current_screen == 29){amp_limit2 = amp_limit2 +ii*donvi;i=0;
       
       //Serial.print("L");Serial.print(amp_limit2,2);Serial.write('\n');
       }
       //-----------------------------------------------------
       
       //----------------------------------
      if(current_screen == 42){velD1 = velD1 +ii*donvi*0.1;i=0;
       if(velD1 >= 50) { velD1 =50;}
       if(velD1<0){             velD1 =0;   
                                cmdMSP = MSP_MOTOR_PID_VEL_D_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velD1));
                                tailSerialReply();}}
       if(current_screen == 43){velD2 = velD2 +ii*donvi*0.1;i=0;
       if(velD2 >= 50) { velD2 =50;}
       if(velD2<0){             velD2 =0; 
                                cmdMSP = MSP_MOTOR_PID_VEL_D_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velD2));
                                tailSerialReply();}
       
       }


       //
       if(current_screen == 30){pos13 = pos13 +ii*donvi;i=0;
                                if(pos13 >= 100) { pos13 =100;}
                                cmdMSP = MSP_MOTOR_POS13_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos13));
                                tailSerialReply();}
       if(current_screen == 31){vel13 = vel13 +ii*donvi*100;i=0;
                                if(vel13 >= 1000) { vel13 =1000;}
                                cmdMSP = MSP_MOTOR_VEL13_SET;
                                headSerialReply(2);
                                serialize16(vel13);
                                tailSerialReply();}
       if(current_screen == 32){pos23 = pos23 +ii*donvi;i=0;
                                if(pos23 >= 100) { pos23 =100;}
                                cmdMSP = MSP_MOTOR_POS23_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos23));
                                tailSerialReply();}
       if(current_screen == 33){vel23 = vel23 +ii*donvi*100;i=0;
                                if(vel23 >= 1000) { vel23 =1000;}
                                cmdMSP = MSP_MOTOR_VEL23_SET;
                                headSerialReply(2);
                                serialize16(vel23);
                                tailSerialReply();}


       
       ii=0;
       }
       //volum decrease
       if ((Kaiten == 1)&& (huong == -1)){i=-1;
       saving =0;
       if(current_screen == 6){pos11 = pos11 +i*donvi;ii=0;
                              if(pos11 <= -100)  {pos11 =-100;}
                              cmdMSP = MSP_MOTOR_POS11_SET;
                              headSerialReply(4);
                              serialize32(FloatToUint(pos11));
                              tailSerialReply();
                              }
       if(current_screen == 7){pos12 = pos12 +i*donvi;ii=0;
                                if(pos12 <= -100)  {pos12 =-100;}
                                cmdMSP = MSP_MOTOR_POS12_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos12));
                                tailSerialReply();}
       if(current_screen == 8){vel11 = vel11 +i*donvi*100;ii=0;
                                if(vel11 <= 1)  {vel11 =1;}
                                cmdMSP = MSP_MOTOR_VEL11_SET;
                                headSerialReply(2);
                                serialize16(vel11);
                                tailSerialReply();}
       if(current_screen == 9){vel12 = vel12 +i*donvi*100;ii=0;
                                if(vel12 <= 1)  {vel12 =1;}
                                cmdMSP = MSP_MOTOR_VEL12_SET;
                                headSerialReply(2);
                                serialize16(vel12);
                                tailSerialReply();}
       if(current_screen == 10){accel1 = accel1 +i*donvi*100;ii=0;
                                if(accel1 <= 1)  {accel1 =1;}
                                cmdMSP = MSP_MOTOR_ACCEL1_SET;
                                headSerialReply(2);
                                serialize16(accel1);
                                tailSerialReply();}
       if(current_screen == 11){decel1 = decel1 +i*donvi*100;ii=0;
                                if(decel1 <= 1)  {decel1 =1;}
                                cmdMSP = MSP_MOTOR_DECCEL1_SET;
                                headSerialReply(2);
                                serialize16(decel1);
                                tailSerialReply();}
       if(current_screen == 12){posP1 = posP1 +i*donvi;ii=0;
                                if(posP1 <= 0)  {posP1 =0;}
                                cmdMSP = MSP_MOTOR_PID_P_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(posP1));
                                tailSerialReply();}
       if(current_screen == 13){velP1 = velP1 +i*donvi;ii=0;
                                if(velP1 <= 0)  {velP1 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_P_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velP1));
                                tailSerialReply();}
       if(current_screen == 14){velI1 = velI1 +i*donvi;ii=0;
                                if(velI1 <= 0)  {velI1 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_I_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velI1));
                                tailSerialReply();}
       if(current_screen == 15){volIN1 = volIN1 +i*donvi;ii=0;
                                if(volIN1 <= 1)  {volIN1 =1;}
                                cmdMSP = MSP_MOTOR_VOL_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(volIN1));
                                tailSerialReply();}
       if(current_screen == 16){volLIMIT1 = volLIMIT1 +i*donvi;ii=0;
                                if(volLIMIT1 <= 1)  {volLIMIT1 =1;}
                                cmdMSP = MSP_MOTOR_VOLLIMIT_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(volLIMIT1));
                                tailSerialReply();}
       if(current_screen == 17){amp_limit1 = amp_limit1 +i*donvi;ii=0;
       
       //Serial.print("Z");Serial.print(amp_limit1,2);Serial.write('\n');
       }
         //-------------------------------------------------
       if(current_screen == 18){pos21 = pos21 +i*donvi;ii=0;
                                if(pos21 <= -100)  {pos21 =-100;}
                                cmdMSP = MSP_MOTOR_POS21_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos21));
                                tailSerialReply();}
       if(current_screen == 19){pos22 = pos22 +i*donvi;ii=0;
                                if(pos22 <= -100)  {pos22 =-100;}
                                cmdMSP = MSP_MOTOR_POS22_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos22));
                                tailSerialReply();}
       if(current_screen == 20){vel21 = vel21 +i*donvi*100;ii=0;
                                if(vel21 <= 1)  {vel21 =1;}
                                cmdMSP = MSP_MOTOR_VEL21_SET;
                                headSerialReply(2);
                                serialize16(vel21);
                                tailSerialReply();}
       if(current_screen == 21){vel22 = vel22 +i*donvi*100;ii=0;
                                if(vel22 <= 1)  {vel22 =1;}
                                cmdMSP = MSP_MOTOR_VEL22_SET;
                                headSerialReply(2);
                                serialize16(vel22);
                                tailSerialReply();}
       if(current_screen == 22){accel2 = accel2 +i*donvi*100;ii=0;
                                if(accel2 <= 1)  {accel2 =1;}
                                cmdMSP = MSP_MOTOR_ACCEL2_SET;
                                headSerialReply(2);
                                serialize16(accel2);
                                tailSerialReply();}
       if(current_screen == 23){decel2 = decel2 +i*donvi*100;ii=0;
                                if(decel2 <= 1)  {decel2 =1;}
                                cmdMSP = MSP_MOTOR_DECCEL2_SET;
                                headSerialReply(2);
                                serialize16(decel2);
                                tailSerialReply();}
       if(current_screen == 24){posP2 = posP2 +i*donvi;ii=0;
                                if(posP2 <= 0)  {posP2 =0;}
                                cmdMSP = MSP_MOTOR_PID_P_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(posP2));
                                tailSerialReply();}
       if(current_screen == 25){velP2 = velP2 +i*donvi;ii=0;
                                if(velP2 <= 0)  {velP2 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_P_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velP2));
                                tailSerialReply();}
       if(current_screen == 26){velI2 = velI2 +i*donvi;ii=0;
                                if(velI2 <= 0)  {velI2 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_I_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velI2));
                                tailSerialReply();}
       if(current_screen == 27){volIN2 = volIN2 +i*donvi;ii=0;
                                if(volIN2 <= 1)  {volIN2 =1;}
                                cmdMSP = MSP_MOTOR_VOL_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(volIN2));
                                tailSerialReply();}
       if(current_screen == 28){volLIMIT2 = volLIMIT2 +i*donvi;ii=0;
                                if(volLIMIT2 <= 1)  {volLIMIT2 =1;}
                                cmdMSP = MSP_MOTOR_VOLLIMIT_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(volLIMIT2));
                                tailSerialReply();}
       if(current_screen == 29){amp_limit2 = amp_limit2 +i*donvi;ii=0;
       
       //Serial.print("L");Serial.print(amp_limit2,2);Serial.write('\n');
       }
       //--------------------------------------------------------
       
     //------------------
       if(current_screen == 42){velD1 = velD1 +i*donvi*0.1;ii=0;
       if(velD1 <= 0)  {velD1 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_D_M1_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velD1));
                                tailSerialReply();}
       if(current_screen == 43){velD2 = velD2 +i*donvi*0.1;ii=0;
       if(velD2 <= 0)  {velD2 =0;}
                                cmdMSP = MSP_MOTOR_PID_VEL_D_M2_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(velD2));
                                tailSerialReply();}
       //
       if(current_screen == 30){pos13 = pos13 +i*donvi;ii=0;
                                if(pos13 <= -100) { pos13 =-100;}
                                cmdMSP = MSP_MOTOR_POS13_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos13));
                                tailSerialReply();}
       if(current_screen == 31){vel13 = vel13 +i*donvi*100;ii=0;
                                if(vel13 <= 1) { vel13 =1;}
                                cmdMSP = MSP_MOTOR_VEL13_SET;
                                headSerialReply(2);
                                serialize16(vel13);
                                tailSerialReply();}
       if(current_screen == 32){pos23 = pos23 +i*donvi;ii=0;
                                if(pos23 <= -100) { pos23 =-100;}
                                cmdMSP = MSP_MOTOR_POS23_SET;
                                headSerialReply(4);
                                serialize32(FloatToUint(pos23));
                                tailSerialReply();}
       if(current_screen == 33){vel23 = vel23 +i*donvi*100;ii=0;
                                if(vel23 <= 1) { vel23 =1;}
                                cmdMSP = MSP_MOTOR_VEL23_SET;
                                headSerialReply(2);
                                serialize16(vel23);
                                tailSerialReply();}
       
       
       i=0;
       }  
     // limiter
    
    
 
    }




if ((menuCount == 5)&& (digitalRead(BUTTON_SELECT_PIN) == LOW)) {
   current_screen = 0;
   menuCount = 0;
   access3 =0;
   saving =0;
  }
}




uint32_t FloatToUint(float n)
{
   return (uint32_t)(*(uint32_t*)&n);
}
 
float UintToFloat(uint32_t n)
{
   return (float)(*(float*)&n);
}
