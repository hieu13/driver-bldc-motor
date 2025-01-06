#include <Arduino.h>
#include <U8g2lib.h>
//#include <RotaryEncoder.h>
//#include "crc8.h" 

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
uint8_t btn_prev,btn_prev1,btn_prev2,btn_prev3,btn_prev4,btn_prev5,btn_prev6,btn_prev7,btn_prev8,
btn_prev9,btn_prev10,btn_prev11,btn_prev12,btn_prev13,btn_prev14,btn_prev15,btn_prev16;

//#define SS1_LED_PIN PA2

//// SD card settings
//#define SDPOWER            -1
//#define SDSS               53
//#define SDCARDDETECT       49

//#define KILL_PIN           41

uint8_t  data_telemetry8[15];
uint16_t data_telemetry16[15];
uint32_t data_telemetry32[20];
uint8_t motor_ready,motor_ss,motor1_resolution,motor1_pole,motor2_resolution,motor2_pole;


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

int8_t i_cout =0;
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



const int NUM_ITEMS = 9; // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LENGTH = 20; // maximum characters for the item name

char menu_items [NUM_ITEMS] [MAX_ITEM_LENGTH] = {  // array with item names
  { "AXIS<1>" }, 
  { "AXIS<2>" },
  { "PARA_AXIS<1>" },
  { "PARA_AXIS<2>" },
  { "RUN TEST_io" },
  { "SET CTRL Mode" },
  { "SET NUM Motor" },
  { "SET BASE Motor" },
  { "SET Defaut" }
 };

const int NUMBER_ITEMS = 7; // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LEN = 20; // maximum characters for the item name

char axis1_items [NUMBER_ITEMS] [MAX_ITEM_LEN] = {  // array with item names
  
  { "POSITION<1>" }, 
  { "POSITION<2>" }, 
  { "SPEED POS<1>" }, 
  { "SPEED POS<2>" }, 
  { "Accelerate" }, 
  { "Decelerate" },   
  { "SAVE<Axis1>" }
 };
char axis2_items [NUMBER_ITEMS] [MAX_ITEM_LEN] = {  // array with item names
  
    { "POSITION<1>" }, 
  { "POSITION<2>" }, 
  { "SPEED POS<1>" }, 
  { "SPEED POS<2>" }, 
  { "Accelerate" }, 
  { "Decelerate" },   
  { "SAVE<Axis2>" }
 };

char axis1_paras [NUMBER_ITEMS] [MAX_ITEM_LEN] = {  // array with item names
  
    
  { "PID_position" }, 
  { "PID_P_vel" },
  { "PID_I_vel" },
  { "PID_D_vel" },
  { "Set_VACC_IN" },
  { "Set_VolLimit" },
  { "SAVE<Axis1>" }
 };

char axis2_paras [NUMBER_ITEMS] [MAX_ITEM_LEN] = {  // array with item names
  
    
  { "PID_position" }, 
  { "PID_P_vel" },
  { "PID_I_vel" },
  { "PID_D_vel" },
  { "Set_VACC_IN" },
  { "Set_VolLimit" },
  { "SAVE<Axis2>" }
 };

const int NUMBER_ITEMS_BASE = 7; // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
//const int MAX_ITEM_LEN = 20; // maximum characters for the item name
char base_motor [NUMBER_ITEMS_BASE] [MAX_ITEM_LEN] = {  // array with item names
  
    
  { "PULSE_1 res" }, 
  { "PULSE_2 res" },
  { "MOTOR 1 POLE" },
  { "MOTOR 2 POLE" },
  { "MOTION_CTRL 1" },
  { "MOTION_CTRL 2" },
  { "SAVE<data>" }
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
bool one_click;

int current_screen = 0;   // 0 = menu, 1 = screenshot, 2 = qr

int demo_mode = 0; // when demo mode is set to 1, it automatically goes over all the screens, 0 = control menu with buttons
int demo_mode_state = 0; // demo mode state = which screen and menu item to display
int demo_mode_delay = 0; // demo mode delay = used to slow down the screen switching
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
float cur_pos11,cur_pos12,cur_vel11,cur_vel12,cur_accel1,cur_decel1,cur_posP1,cur_velP1,cur_velI1,cur_velD1,cur_volIN1,cur_volLIMIT1,cur_AMP1;
float pos11,pos12,vel11,vel12,accel1,decel1,posP1,velP1,velI1,velD1,volIN1,volLIMIT1,amp_limit1;
float cur_pos21,cur_pos22,cur_vel21,cur_vel22,cur_accel2,cur_decel2,cur_posP2,cur_velP2,cur_velI2,cur_velD2,cur_volIN2,cur_volLIMIT2,cur_AMP2;
float pos21,pos22,vel21,vel22,accel2,decel2,posP2,velP2,velI2,velD2,volIN2,volLIMIT2,amp_limit2;
int resolution_1_set,resolution_2_set,pole_1_set,pole_2_set;
uint8_t cur_mode,mode,error;

float ichi11,ichi12,ichi21,ichi22,ichi31,ichi32;
uint8_t number_motor,cur_number_motor ;
float number_motor_select =2 ;
int defaut_select=3;


byte pos11_enable,pos12_enable,pos21_enable,pos22_enable;
byte vel11_enable,vel12_enable,vel21_enable,vel22_enable;
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
/////////////////////////////////////////
uint8_t data_motor_1,data_motor_2;
uint8_t control_motion_1,control_motion_2;
/////////////////////////////////////////
#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE];
static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;
//

#define SUN	0
#define SUN_CLOUD  1
#define CLOUD 2
#define RAIN 3
#define THUNDER 4

uint32_t timer_setbackground;
//button
int output_select_State = HIGH;
int buttonState;            // the current reading from the input pin
int lastButtonState = HIGH;  // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay =5;    // the debounce time; increase if the output flickers



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
#define MSP_MOTOR_ERR_SET  6
#define MSP_MOTOR_NC_SET  7
#define MSP_MOTOR_DATAPOS1_SET  8
#define MSP_MOTOR_DATAPOS2_SET  9

#define MSP_MOTOR_POS11_SET 10
#define MSP_MOTOR_POS12_SET 11
#define MSP_MOTOR_POS21_SET 12
#define MSP_MOTOR_POS22_SET 13

#define MSP_MOTOR_VEL11_SET 14
#define MSP_MOTOR_VEL12_SET 15
#define MSP_MOTOR_VEL21_SET 16
#define MSP_MOTOR_VEL22_SET 17

#define MSP_MOTOR_ACCEL1_SET 18
#define MSP_MOTOR_ACCEL2_SET 19
#define MSP_MOTOR_DECCEL1_SET 20
#define MSP_MOTOR_DECCEL2_SET 21

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

#define MSP_MOTOR_PARASAVE1_SET  34
#define MSP_MOTOR_PARASAVE2_SET  35
//
#define MSP_MOTOR_POS13_SET 36
#define MSP_MOTOR_POS23_SET 37
#define MSP_MOTOR_VEL13_SET 38
#define MSP_MOTOR_VEL22_SET 39
#define MSP_MOTOR_ACCEL3_SET 40
#define MSP_MOTOR_DECCEL3_SET 41

#define MSP_MOTOR_FB_ICHI 50
#define MSP_MOTOR_FB_VELO_ACC_DEC_M1 51
#define MSP_MOTOR_FB_VELO_ACC_DEC_M2 52

#define MSP_MOTOR_IDOU_1_SET 60
#define MSP_MOTOR_IDOU_2_SET 61
#define MSP_MOTOR_ACC_DEC_1_SET 62
#define MSP_MOTOR_ACC_DEC_2_SET 63
#define MSP_MOTOR_SPEED_1_SET 64
#define MSP_MOTOR_SPEED_2_SET 65

#define MSP_MOTOR_DISABLE_MOTO_SET 66
#define MSP_MOTOR_RESO_1_SET 67
#define MSP_MOTOR_RESO_2_SET 68
#define MSP_MOTOR_POLE_1_SET 69
#define MSP_MOTOR_POLE_2_SET 70
#define MSP_MOTOR_MODE_RUN 71

#define MSP_MOTOR_MOTION_CTRL_1 72
#define MSP_MOTOR_MOTION_CTRL_2 73

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

void stt_button_select() {

  int reading = digitalRead(BUTTON_SELECT_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        output_select_State = !output_select_State;
      }
    }
  }
  lastButtonState = reading;

}
void stt_button_exit() {}
void stt_button_up() {}
void stt_button_down() {}



void setup(void) {
  Serial.begin(115200);
  u8g2.begin();
  u8g2.setBitmapMode(1);
  u8g2.enableUTF8Print();
 // u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
  u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8g2log.setLineHeightOffset(0); 
  u8g2log.setRedrawMode(0);
 
  //pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(BUTTON_EXIT_PIN, INPUT_PULLUP); // down button
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP); // down button
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP); // down button INPUT_PULLUP

  current_screen=99;//background

pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);// select button

  btn_prev = digitalRead(BUTTON_SELECT_PIN);
  //digitalWrite(LED_BUILTIN, LOW);
}

void single_click_menu_main(){
  uint8_t btn = digitalRead(BUTTON_SELECT_PIN);
 if (btn == LOW && btn_prev == HIGH)
  {
    if ((current_screen == 0)&&(item_selected==0)) {current_screen = 1;delay(150); }//pos1
      if ((current_screen == 0)&&(item_selected==1)) {current_screen = 2;delay(150);}//pos2
      if ((current_screen == 0)&&(item_selected==2)) {current_screen = 46;delay(150);}//para pos1
      if ((current_screen == 0)&&(item_selected==3)) {current_screen = 47;delay(150);}//para pos2
      if ((current_screen == 0)&&(item_selected==4)) {current_screen = 3;delay(150);}//run test
      if ((current_screen == 0)&&(item_selected==5)) {current_screen = 4;delay(150);}//set control
      if ((current_screen == 0)&&(item_selected==6)) {current_screen = 5;delay(150);}//number motor
      if ((current_screen == 0)&&(item_selected==7)) {current_screen = 48;delay(150);}//komakai setting
      if ((current_screen == 0)&&(item_selected==8)) {current_screen = 45;delay(150);}//set defaut
  }
  btn_prev = digitalRead(BUTTON_SELECT_PIN);
}
void single_click_menu_1(){
uint8_t btn1 = digitalRead(BUTTON_SELECT_PIN);

 if (btn1 == LOW && btn_prev1 == HIGH)
  {delay(10);
          
          lock_tani=0;clicked_tani =0;lock =0;clicked =0;
          if (item_select1==0) {current_screen = 6;}
          if (item_select1==1) {current_screen = 7;}
          if (item_select1==2) {current_screen = 8;}
          if (item_select1==3) {current_screen = 9;}
          if (item_select1==4) {current_screen = 10;}
          if (item_select1==5) {current_screen = 11;}
          if (item_select1==6) {current_screen = 17;}
  }
  btn_prev1 = digitalRead(BUTTON_SELECT_PIN);
}
void single_click_menu_2(){
uint8_t btn2 = digitalRead(BUTTON_SELECT_PIN);

 if (btn2 == LOW && btn_prev2 == HIGH)
  {delay(10);
  lock_tani=0;clicked_tani =0;lock =0;clicked =0;
      if (item_select2==0) {current_screen = 18;}
      if (item_select2==1) {current_screen = 19;}
      if (item_select2==2) {current_screen = 20;}
      if (item_select2==3) {current_screen = 21;}
      if (item_select2==4) {current_screen = 22;}
      if (item_select2==5) {current_screen = 23;}
      if (item_select2==6) {current_screen = 29;}
  }
  btn_prev2 = digitalRead(BUTTON_SELECT_PIN);
}
void single_click_menu_3(){
uint8_t btn3 = digitalRead(BUTTON_SELECT_PIN);

 if (btn3 == LOW && btn_prev3 == HIGH)
  {delay(10);
  lock_tani=0;clicked_tani =0;lock =0;clicked =0;
      if (item_select1==0) {current_screen = 12;}
      if (item_select1==1) {current_screen = 13;}
      if (item_select1==2) {current_screen = 14;}
      if (item_select1==3) {current_screen = 42;}
      if (item_select1==4) {current_screen = 15;}
      if (item_select1==5) {current_screen = 16;}
      if (item_select1==6) {current_screen = 56;}
  }
  btn_prev3 = digitalRead(BUTTON_SELECT_PIN);
}

void single_click_menu_4(){
uint8_t btn4 = digitalRead(BUTTON_SELECT_PIN);

 if (btn4 == LOW && btn_prev4 == HIGH)
  {delay(10);
  lock_tani=0;clicked_tani =0;lock =0;clicked =0;
      if (item_select2==0) {current_screen = 24;}
      if (item_select2==1) {current_screen = 25;}
      if (item_select2==2) {current_screen = 26;}
      if (item_select2==3) {current_screen = 43;}
      if (item_select2==4) {current_screen = 27;}
      if (item_select2==5) {current_screen = 28;}
      if (item_select2==6) {current_screen = 57;}
  }
  btn_prev4 = digitalRead(BUTTON_SELECT_PIN);
}
void single_click_menu_5(){
uint8_t btn5 = digitalRead(BUTTON_SELECT_PIN);

 if (btn5 == LOW && btn_prev5 == HIGH)
  {delay(10);
  lock_tani=0;clicked_tani =0;lock =0;clicked =0;
      if (item_select1==0) {current_screen = 49;}
      if (item_select1==1) {current_screen = 50;}
      if (item_select1==2) {current_screen = 51;}
      if (item_select1==3) {current_screen = 52;}
      if (item_select1==4) {current_screen = 53;}
      if (item_select1==5) {current_screen = 54;}
      if (item_select1==6) {current_screen = 55;}
  }
  btn_prev5 = digitalRead(BUTTON_SELECT_PIN);
}
void loop(void) {

  if (digitalRead(BUTTON_EXIT_PIN) == LOW && one_click == false){timer_setbackground = millis();one_click=true;}
  if (digitalRead(BUTTON_EXIT_PIN) == HIGH){timer_setbackground = millis();one_click=false;}
  if (digitalRead(BUTTON_EXIT_PIN) == LOW && (millis()- timer_setbackground ) >2000 && one_click==true){
    current_screen=99;
    one_click == false;
    
    }

  stt_button_select();


  if (current_screen==99){background_main();}

  protocol_send_cmd();

  if (error==1||error==2){messenger_siji();}
  if (error ==3||error ==4){messenger_err();}

 static int pos = 0;
 static int menu = 0;
//  encoder->tick(); // just call tick() to check the state.
  Kaiten =0;


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


   if(item_selected >= 0 && item_selected <=8)  {item_sel_previous = item_selected - 1;item_sel_next = item_selected + 1;  }
   if (item_sel_previous < 0) {item_sel_previous = 8 ;}
   if (item_sel_next > 8) {item_sel_next = 0;}
   
   if(item_selected < 0) { item_selected =8;item_sel_previous = item_selected - 1;item_sel_next = 0;}
   if(item_selected > 8)  {item_selected =0;item_sel_previous = 8;item_sel_next = item_selected+1;}

single_click_menu_main();

/////////////////////////////EXIT/////////////////////////////////////////      

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
if ((current_screen == 46)&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 0;
   button_clicked = 1; }
if ((current_screen == 47)&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 0;
   button_clicked = 1; }
if ((current_screen == 48)&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 0;
   button_clicked = 1; }
//exit button 
/////////////////////////////EXIT/////////////////////////////////////////   
if ((((current_screen >= 6)&&(current_screen <= 11)))&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 1;
  
   button_clicked = 1; posCount=0;} 
if ((((current_screen >= 12)&&(current_screen <= 16))||(current_screen == 42))&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 46;
   
   button_clicked = 1; } 

if ((((current_screen >= 18)&&(current_screen <= 23)))&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 2;
   
   button_clicked = 1; }
if ((((current_screen >= 24)&&(current_screen <= 28))||(current_screen == 43))&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 47;
   
   button_clicked = 1; }
if ((((current_screen >= 49)&&(current_screen <= 54)))&&(digitalRead(BUTTON_EXIT_PIN) == LOW) && (button_clicked == 0)) {
  current_screen = 48;
   button_clicked = 1; } 




if ((digitalRead(BUTTON_EXIT_PIN) == HIGH) && (button_clicked == 1)) { // unclick 
    button_clicked = 0;
    button_click1 = 0;
    button_click2 = 0;
    access3  = 0;
    saving =0;
  }

 

/////////////////////////////  MAIN   MENU SCREEN//////////////////////////////
 
     u8g2.clearBuffer();  // clear buffer for storing display content in RAM
    switch (current_screen) {
      case 0://main
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
      break;
      case 1://pos1
          screen_axis_1();
          single_click_menu_1();
          //stt_button_select();
          
      break;
      case 2://pos2
          screen_axis_2();
          single_click_menu_2();
      break;
      case 3://rub mode test
          position_run();
          selection_check();
      break;
      case 4://mode
           screen_mode();
      break;
      case 5://num motor
          number_motor_screen();
      break;
      case 45://default
          defaut_screen();
      break;
      case 46://pos1 para
          screen_axis_1_para();
          single_click_menu_3();
      break;
      case 47://pos2 para
          screen_axis_2_para();
          single_click_menu_4();
      break;
      case 48://base
          screen_mode_base();
          single_click_menu_5();
      break;
      case 6:
                              
                          u8g2.drawStr(10, 12, "Position1_1");u8g2.drawStr(10, 25, "Pos11_cur:");u8g2.setCursor(80,25);u8g2.print(cur_pos11,2);u8g2.drawStr(10, 38, "Pos11_set:"); 
                          u8g2.setCursor(80,38);u8g2.print(pos11,2);u8g2.drawStr(10, 51, ""); u8g2.drawStr(10, 64, "Exit"); u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                          menuCheck();
      break;
      case 7:
                          u8g2.drawStr(10, 12, "Position1_2"); u8g2.drawStr(10, 25, "Pos12_cur:");u8g2.setCursor(80,25);u8g2.print(cur_pos12,2);u8g2.drawStr(10, 38, "Pos12_set:");
                          u8g2.setCursor(80,38);u8g2.print(pos12,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
                          menuCheck();
      break;
      case 8:
                          u8g2.drawStr(10, 12, "Velocity1_1");u8g2.drawStr(10, 25, "Vel11_cur:");u8g2.setCursor(80,25);u8g2.print(cur_vel11,2);u8g2.drawStr(10, 38, "Vel11_set:"); 
                          u8g2.setCursor(80,38);u8g2.print(vel11,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
                          menuCheck();
      break;
      case 9:
                          u8g2.drawStr(10, 12, "Velocity1_2");u8g2.drawStr(10, 25, "Vel12_cur:");u8g2.setCursor(80,25);u8g2.print(cur_vel12,2);u8g2.drawStr(10, 38, "Vel12_set:"); 
                          u8g2.setCursor(80,38);u8g2.print(vel12,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");       
                          menuCheck();               
      break;
      case 10:
                          u8g2.drawStr(10, 12, "Accell_1");u8g2.drawStr(10, 25, "Acc1_cur:");u8g2.setCursor(80,25);u8g2.print(cur_accel1,2);u8g2.drawStr(10, 38, "Acc1_set:"); 
                          u8g2.setCursor(80,38);u8g2.print(accel1,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                          menuCheck();
      break;
      case 11:
                          u8g2.drawStr(10, 12, "Decell_1");u8g2.drawStr(10, 25, "Dec1_cur:");u8g2.setCursor(80,25);u8g2.print(cur_decel1,2);u8g2.drawStr(10, 38, "Dec1_set:"); 
                          u8g2.setCursor(80,38);u8g2.print(decel1,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                          menuCheck();
      break;
      case 12:
                          u8g2.drawStr(10, 12, "PID_pos_P1");u8g2.drawStr(10, 25, "Ppos1_cur:");u8g2.setCursor(80,25);u8g2.print(cur_posP1,2);u8g2.drawStr(10, 38, "Ppos1_set:"); 
                          u8g2.setCursor(80,38);u8g2.print(posP1,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
                          menuCheck();
      break;
      case 13:
                          u8g2.drawStr(10, 12, "PID_Vel_P1");u8g2.drawStr(10, 25, "Pvel1_cur:");u8g2.setCursor(80,25);u8g2.print(cur_velP1,2);u8g2.drawStr(10, 38, "Pvel1_set:"); 
                          u8g2.setCursor(80,38);u8g2.print(velP1,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                          menuCheck();
      break;
      case 14:
                          u8g2.drawStr(10, 12, "PID_vel_I1");u8g2.drawStr(10, 25, "Ivel1_cur:");u8g2.setCursor(80,25);u8g2.print(cur_velI1,2);u8g2.drawStr(10, 38, "Ivel1_set:"); 
                          u8g2.setCursor(80,38);u8g2.print(velI1,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");   
                          menuCheck();     
      break;
      case 15:
                          u8g2.drawStr(10, 12, "VolIN_1(V)");u8g2.drawStr(10, 25, "VIN1_cur:");u8g2.setCursor(80,25);u8g2.print(cur_volIN1,2);
                          u8g2.drawStr(10, 38, "VIN1_set:");u8g2.setCursor(80,38);u8g2.print(volIN1,2);u8g2.drawStr(10, 51, ""); 
                          u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                          menuCheck();
      break;
      case 16:
                          u8g2.drawStr(10, 12, "VolLI_1(V)");u8g2.drawStr(10, 25, "VLI1_cur:");u8g2.setCursor(80,25);u8g2.print(cur_volLIMIT1,2);
                          u8g2.drawStr(10, 38, "VLI1_set:");u8g2.setCursor(80,38);u8g2.print(volLIMIT1,2);u8g2.drawStr(10, 51, ""); 
                          u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                          menuCheck();
      break;
      case 17://///////////////////////////////////////////SAVE POSITION 1
                      cmdMSP = MSP_MOTOR_SAVE_SET;headSerialReply(1);serialize8(1);tailSerialReply();
                      u8g2.clearDisplay();u8g2.setFont(u8g2_font_unifont_t_symbols);u8g2.drawGlyph(5, 25, 0x2714);u8g2.setFont(u8g_font_7x14);
                      u8g2.drawStr(30, 25, "position");u8g2.drawStr(25, 45, "ASIX 1 SAVED");u8g2.sendBuffer();
                      delay(2000);
                      current_screen=1;
                      u8g2.clearDisplay();
      break;
      case 18:
                        u8g2.drawStr(10, 12, "Position2_1");u8g2.drawStr(10, 25, "Pos21_cur:");u8g2.setCursor(80,25);u8g2.print(cur_pos21,2);u8g2.drawStr(10, 38, "Pos21_set:");
                        u8g2.setCursor(80,38);u8g2.print(pos21,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
                        menuCheck();
      break;
      case 19:
                      u8g2.drawStr(10, 12, "Position2_2"); u8g2.drawStr(10, 25, "Pos22_cur:");u8g2.setCursor(80,25);u8g2.print(cur_pos22,2);u8g2.drawStr(10, 38, "Pos22_set:");
                      u8g2.setCursor(80,38);u8g2.print(pos22,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
                      menuCheck();
      break;
      case 20:
                      u8g2.drawStr(10, 12, "Velocity2_1");u8g2.drawStr(10, 25, "Vel21_cur:");u8g2.setCursor(80,25); u8g2.print(cur_vel21,2); u8g2.drawStr(10, 38, "Vel21_set:"); 
                      u8g2.setCursor(80,38);u8g2.print(vel21,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");     
                      menuCheck();        
      break;
      case 21:
                      u8g2.drawStr(10, 12, "Velocity2_2");u8g2.drawStr(10, 25, "Vel22_cur:");u8g2.setCursor(80,25);u8g2.print(cur_vel22,2);u8g2.drawStr(10, 38, "Vel22_set:"); 
                      u8g2.setCursor(80,38);u8g2.print(vel22,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                      menuCheck();
      break;
      case 22:
                      u8g2.drawStr(10, 12, "Accell_2");u8g2.drawStr(10, 25, "Acc2_cur:");u8g2.setCursor(80,25);u8g2.print(cur_accel2,2);u8g2.drawStr(10, 38, "Acc2_set:"); 
                      u8g2.setCursor(80,38);u8g2.print(accel2,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                      menuCheck();
      break;
      case 23:
                        u8g2.drawStr(10, 12, "Decell_2");u8g2.drawStr(10, 25, "Dec2_cur:");u8g2.setCursor(80,25);u8g2.print(cur_decel2,2);u8g2.drawStr(10, 38, "Dec2_set:"); 
                        u8g2.setCursor(80,38);u8g2.print(decel2,2);u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                        menuCheck();
      break;
      case 24:
                      u8g2.drawStr(10, 12, "PID_pos_P2");u8g2.drawStr(10, 25, "Ppos2_cur:");u8g2.setCursor(80,25);u8g2.print(cur_posP2,2);u8g2.drawStr(10, 38, "Ppos1_set:"); 
                      u8g2.setCursor(80,38);u8g2.print(posP2,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                      menuCheck();
      break;
      case 25:
                      u8g2.drawStr(10, 12, "PID_Vel_P2");u8g2.drawStr(10, 25, "Pvel2_cur:");u8g2.setCursor(80,25);u8g2.print(cur_velP2,2);u8g2.drawStr(10, 38, "Pvel2_set:"); 
                      u8g2.setCursor(80,38);u8g2.print(velP2,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                      menuCheck();
      break;
      case 26:
                      u8g2.drawStr(10, 12, "PID_vel_I2");u8g2.drawStr(10, 25, "Ivel2_cur:");u8g2.setCursor(80,25);u8g2.print(cur_velI2,2);u8g2.drawStr(10, 38, "Ivel2_set:"); 
                      u8g2.setCursor(80,38);u8g2.print(velI2,2);u8g2.drawStr(10, 51, ""); u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                      menuCheck();
      break;
      case 27:
                        u8g2.drawStr(10, 12, "VolIN_2(V)");u8g2.drawStr(10, 25, "VIN2_cur:");u8g2.setCursor(80,25);u8g2.print(cur_volIN2,2);u8g2.drawStr(10, 38, "VIN2_set:"); 
                        u8g2.setCursor(80,38);u8g2.print(volIN2,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                        menuCheck();
      break;
      case 28:
                        u8g2.drawStr(10, 12, "VolLI_2(V)");u8g2.drawStr(10, 25, "VLI2_cur:");u8g2.setCursor(80,25);u8g2.print(cur_volLIMIT2,2);u8g2.drawStr(10, 38, "VLI2_set:"); 
                        u8g2.setCursor(80,38);u8g2.print(volLIMIT2,2);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                        menuCheck();
      break;
      case 29://///////////////////////////////////////////SAVE POSITION 2
                        cmdMSP = MSP_MOTOR_SAVE_SET;headSerialReply(1);serialize8(2);tailSerialReply();
                        u8g2.clearDisplay();u8g2.setFont(u8g2_font_unifont_t_symbols);u8g2.drawGlyph(5, 25, 0x2714);u8g2.setFont(u8g_font_7x14);u8g2.drawStr(30, 25, "position"); 
                        u8g2.drawStr(25, 45, "ASIX 2 SAVED");u8g2.sendBuffer();
                        delay(2000);
                        current_screen=2;
                        u8g2.clearDisplay();
      break;
      case 42:
                        u8g2.drawStr(10, 12, "PID_vel_D1");u8g2.drawStr(10, 25, "Dvel1_cur:");u8g2.setCursor(80,25);u8g2.print(cur_velD1,3);u8g2.drawStr(10, 38, "Dvel1_set:"); 
                        u8g2.setCursor(80,38);u8g2.print(velD1,3);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                        menuCheck();
      break;
      case 43:
                        u8g2.drawStr(10, 12, "PID_vel_D2");u8g2.drawStr(10, 25, "Dvel2_cur:");u8g2.setCursor(80,25);u8g2.print(cur_velD2,3);u8g2.drawStr(10, 38, "Dvel2_set:"); 
                        u8g2.setCursor(80,38);u8g2.print(velD2,3);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                        menuCheck();
      break;
      case 49:
                        u8g2.drawStr(10, 12, "Pulse_resolution_1");u8g2.drawStr(10, 25, "cur_res1:");u8g2.setCursor(80,25);u8g2.print(motor1_resolution);u8g2.drawStr(10, 38, "res1_set:"); 
                        u8g2.setCursor(80,38);u8g2.print(resolution_1_set);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                        menuCheck();
      break;
      case 50:
                        u8g2.drawStr(10, 12, "Pulse_resolution_2");u8g2.drawStr(10, 25, "cur_res2:");u8g2.setCursor(80,25);u8g2.print(motor2_resolution);u8g2.drawStr(10, 38, "res2_set:"); 
                        u8g2.setCursor(80,38);u8g2.print(resolution_2_set);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
                        menuCheck();
      break;
      case 51:
                        u8g2.drawStr(10, 12, "Pole_motor_1");u8g2.drawStr(10, 25, "cur_Pole1:");u8g2.setCursor(80,25);u8g2.print(motor1_pole);u8g2.drawStr(10, 38, "Pole1_set:"); 
                        u8g2.setCursor(80,38);u8g2.print(pole_1_set);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
                        menuCheck();
      break;
      case 52:
                        u8g2.drawStr(10, 12, "Pole_motor_2");u8g2.drawStr(10, 25, "cur_Pole2:");u8g2.setCursor(80,25); u8g2.print(motor2_pole);u8g2.drawStr(10, 38, "Pole2_set:"); 
                        u8g2.setCursor(80,38);u8g2.print(pole_2_set);u8g2.drawStr(10, 51, "");u8g2.drawStr(10, 64, "Exit");u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
                        menuCheck();
      break;
      case 53:
                    u8g2.drawStr(10, 12, "angle"); 
                    u8g2.drawStr(10, 25, "torque"); u8g2.drawStr(65, 10, "<MOTOR 1>"); 
                    u8g2.drawStr(10, 38, "velocity");   
                    u8g2.drawStr(10, 51, "velocity_openloop"); 
                    u8g2.drawStr(10, 64, "angle_openloop"); 
                    u8g2.drawStr(2, (menuCount * 13) - 1, ">"); 
                    menuCheck_moderun();
      break;
      case 54:
                    u8g2.drawStr(10, 12, "angle"); 
                    u8g2.drawStr(10, 25, "torque"); u8g2.drawStr(65, 10, "<MOTOR 2>"); 
                    u8g2.drawStr(10, 38, "velocity");   
                    u8g2.drawStr(10, 51, "velocity_openloop"); 
                    u8g2.drawStr(10, 64, "angle_openloop"); 
                    u8g2.drawStr(2, (menuCount * 13) - 1, ">");
                    menuCheck_moderun();
      break;
      case 55:
                    cmdMSP = MSP_MOTOR_SAVE_SET;headSerialReply(1);serialize8(5);tailSerialReply();
                    u8g2.clearDisplay();u8g2.setFont(u8g2_font_unifont_t_symbols);u8g2.drawGlyph(5, 25, 0x2714);u8g2.setFont(u8g_font_7x14);u8g2.drawStr(25, 12, "Please <reset> ");
                    u8g2.drawStr(30, 25, "Base para");u8g2.drawStr(25, 45, "ASIX 1_2 SAVED");u8g2.sendBuffer();
                    delay(2000);
                    current_screen=48;
                    u8g2.clearDisplay();
      break;
      case 56:
                    cmdMSP = MSP_MOTOR_SAVE_SET;headSerialReply(1);serialize8(3);tailSerialReply();u8g2.clearDisplay();
                    u8g2.setFont(u8g2_font_unifont_t_symbols);u8g2.drawGlyph(5, 25, 0x2714);u8g2.setFont(u8g_font_7x14);u8g2.drawStr(30, 25, "parameter");u8g2.drawStr(25, 45, "ASIX 1 SAVED");u8g2.sendBuffer();
                    delay(2000);
                    current_screen=46;
                    u8g2.clearDisplay();
      break;
      case 57:
                    cmdMSP = MSP_MOTOR_SAVE_SET;headSerialReply(1);serialize8(4);tailSerialReply();u8g2.clearDisplay();u8g2.setFont(u8g2_font_unifont_t_symbols);u8g2.drawGlyph(5, 25, 0x2714); 
                    u8g2.setFont(u8g_font_7x14);u8g2.drawStr(30, 25, "parameter"); u8g2.drawStr(25, 45, "ASIX 2 SAVED");u8g2.sendBuffer();
                    delay(2000);
                    current_screen=47;
                    u8g2.clearDisplay();
      break;
      }

    

    u8g2.sendBuffer(); // send buffer from RAM to display controller
     

// if (current_screen != 0 && current_screen != 1 && current_screen != 2 && current_screen != 3 && current_screen != 4 && current_screen != 5&& current_screen != 45&& current_screen != 46&& current_screen != 47&& current_screen != 48&& current_screen != 99) {
//       Display_current_screen();
//       menuCheck();
// }

  

}



void position_run(){

 
   u8g2.setFont(u8g_font_7x14);
   u8g2.drawStr(10, 12, "SELECT Axis:");
   if (select_sl ==  1){ u8g2.drawStr(95, 12, "A_1"); }
   if (select_sl ==  2){ u8g2.drawStr(95, 12, "A_2"); }
  

  u8g2.drawStr(10, 25, "<GENTEN>"); 
 // u8g2.drawStr(80, 25, "RUN"); 
  u8g2.drawStr(10, 38, "<POSITION_1>"); 
 // u8g2.drawStr(80, 38, "RUN"); 
  u8g2.drawStr(10, 51, "<POSITION_2>"); 
   //
  u8g2.drawStr(10, 64, "EXIT"); 
   //
  u8g2.drawStr(2, (posCount * 13) - 1, ">"); 

   
}







uint32_t FloatToUint(float n)
{
   return (uint32_t)(*(uint32_t*)&n);
}
 
float UintToFloat(uint32_t n)
{
   return (float)(*(float*)&n);
}
