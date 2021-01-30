#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "ss_oled.h"

// RPI Pico
#define SDA_PIN 4
#define SCL_PIN 5
#define RESET_PIN -1
int rc;
SSOLED oled;
static uint8_t ucBuffer[1024];
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

int main() {
uint8_t uc[8];
int i, j;
char szTemp[32];
    
  rc = oledInit(&oled, OLED_128x64, 0x3c, 0, 0, 1, SDA_PIN, SCL_PIN, RESET_PIN, 1000000L);
  if (rc != OLED_NOT_FOUND)
  { 
    oledFill(&oled, 0,1);
    oledSetContrast(&oled, 127);
    oledWriteString(&oled, 0,0,0,(char *)"**************** ", FONT_8x8, 0, 1);
    oledWriteString(&oled, 0,4,1,(char *)"BitBank SS_OLED", FONT_8x8, 0, 1);
    oledWriteString(&oled, 0,8,2,(char *)"running on the", FONT_8x8, 0, 1);
    oledWriteString(&oled, 0,8,3,(char *)"SSD1306 128x64", FONT_8x8, 0, 1);
    oledWriteString(&oled, 0,4,4,(char *)"monochrome OLED", FONT_8x8, 0, 1);
    oledWriteString(&oled, 0,24,5,(char *)"Written By", FONT_8x8, 0, 1);
    oledWriteString(&oled, 0,24,6,(char *)"Larry Bank", FONT_8x8, 0, 1);
    oledWriteString(&oled, 0,0,7,(char *)"**************** ", FONT_8x8, 0, 1);
    sleep_ms(4000);
  }
  oledSetBackBuffer(&oled, ucBuffer);
  oledFill(&oled, 0,1);
  oledWriteString(&oled, 0,0,0,(char *)"Now with 5 font sizes", FONT_6x8, 0, 1);
  oledWriteString(&oled, 0,0,1,(char *)"6x8 8x8 16x16", FONT_8x8, 0, 1);
  oledWriteString(&oled, 0,0,2,(char *)"16x32 and a new", FONT_8x8, 0, 1);
  oledWriteString(&oled, 0,0,3,(char *)"Stretched", FONT_12x16, 0, 1);
  oledWriteString(&oled, 0,0,5,(char *)"from 6x8", FONT_12x16, 0, 1);
  sleep_ms(3000);
  
  while (1) {
  int x, y;
  oledFill(&oled, 0, 1);
  oledWriteString(&oled, 0,0,0,(char *)"Backbuffer Test", FONT_NORMAL,0,1);
  oledWriteString(&oled, 0,0,1,(char *)"96 lines", FONT_NORMAL,0,1);
  sleep_ms(2000);
  for (x=0; x<OLED_WIDTH-1; x+=2)
  {
    oledDrawLine(&oled, x, 0, OLED_WIDTH-x, OLED_HEIGHT-1, 1);
  }
  for (y=0; y<OLED_HEIGHT-1; y+=2)
  {
    oledDrawLine(&oled, OLED_WIDTH-1,y, 0,OLED_HEIGHT-1-y, 1);
  }
  oledWriteString(&oled, 0,0,1,(char *)"Without backbuffer", FONT_SMALL,0,1);
  sleep_ms(2000);
  oledFill(&oled, 0,1);
  for (x=0; x<OLED_WIDTH-1; x+=2)
  {
    oledDrawLine(&oled, x, 0, OLED_WIDTH-1-x, OLED_HEIGHT-1, 0);
  }
  for (y=0; y<OLED_HEIGHT-1; y+=2)
  {
    oledDrawLine(&oled, OLED_WIDTH-1,y, 0,OLED_HEIGHT-1-y, 0);
  }
  oledDumpBuffer(&oled, ucBuffer);
  oledWriteString(&oled, 0,0,1,(char *)"With backbuffer", FONT_SMALL,0,1);
  sleep_ms(2000);
  }

  return 0;
} // main
