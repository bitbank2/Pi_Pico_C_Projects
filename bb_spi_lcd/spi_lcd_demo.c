//
// Simple demo of the bb_spi_lcd library
//

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "bb_spi_lcd.h"
#include "AnimatedGIF.h"
//#include "badgers_320.h"
#include "raspberrypi.h"
//#include "homer.h"
//#include "badgers.h"

SPILCD lcd;
GIFIMAGE gif;

#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 135

#define LCD_CS 17
#define LCD_DC 16
#define LCD_SCK 18
#define LCD_MOSI 19
#define LCD_MISO -1
#define LCD_BACKLIGHT -1
#define LCD_RESET 20
#define CLOCK_SPEED (64 * 1024 * 1024)

// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW *pDraw)
{
    uint8_t *s;
    uint16_t *d, *usPalette, usTemp[320];
    int x, y, iWidth;

//printf("line %d\n\r", pDraw->y);

    usPalette = pDraw->pPalette;
    y = pDraw->iY + pDraw->y; // current line
    iWidth = pDraw->iWidth;
    if (iWidth > DISPLAY_WIDTH)
       iWidth = DISPLAY_WIDTH;
    s = pDraw->pPixels;
    if (pDraw->ucDisposalMethod == 2) // restore to background color
    {
      for (x=0; x<iWidth; x++)
      {
        if (s[x] == pDraw->ucTransparent)
           s[x] = pDraw->ucBackground;
      }
      pDraw->ucHasTransparency = 0;
    }
    // Apply the new pixels to the main image
    if (pDraw->ucHasTransparency) // if transparency used
    {
      uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
      int x, iCount;
      pEnd = s + iWidth;
      x = 0;
      iCount = 0; // count non-transparent pixels
      while(x < iWidth)
      {
        c = ucTransparent-1;
        d = usTemp;
        while (c != ucTransparent && s < pEnd)
        {
          c = *s++;
          if (c == ucTransparent) // done, stop
          {
            s--; // back up to treat it like transparent
          }
          else // opaque
          {
             *d++ = usPalette[c];
             iCount++;
          }
        } // while looking for opaque pixels
        if (iCount) // any opaque pixels?
        {
          spilcdSetPosition(&lcd, pDraw->iX+x, y, iCount, 1, DRAW_TO_LCD);
          spilcdWriteDataBlock(&lcd, (uint8_t *)usTemp, iCount*2, DRAW_TO_LCD | DRAW_WITH_DMA);
          x += iCount;
          iCount = 0;
        }
        // no, look for a run of transparent pixels
        c = ucTransparent;
        while (c == ucTransparent && s < pEnd)
        {
          c = *s++;
          if (c == ucTransparent)
             iCount++;
          else
             s--;
        }
        if (iCount)
        {
          x += iCount; // skip these
          iCount = 0;
        }
      }
    }
    else
    {
      s = pDraw->pPixels;
      // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
      for (x=0; x<iWidth; x++)
        usTemp[x] = usPalette[*s++];
      spilcdSetPosition(&lcd, pDraw->iX, y, iWidth, 1, DRAW_TO_LCD);
      spilcdWriteDataBlock(&lcd, (uint8_t *)usTemp, iWidth*2, DRAW_TO_LCD | DRAW_WITH_DMA);
    }
} /* GIFDraw() */



void setup() {
   stdio_init_all();
//int spilcdInit(SPILCD *, int iLCDType, int iFlags, int32_t iSPIFreq, int iCSPin, int iDCPin, int iResetPin, int iLEDPin, int iMISOPin, int iMOSIPin, int iCLKPin);
   memset(&lcd, 0, sizeof(lcd));
   spilcdInit(&lcd, LCD_ST7789_135, FLAGS_NONE, CLOCK_SPEED, LCD_CS, LCD_DC, LCD_RESET, LCD_BACKLIGHT, LCD_MISO, LCD_MOSI, LCD_SCK);

//   spilcdSetOrientation(&lcd, LCD_ORIENTATION_90);
   spilcdFill(&lcd, 0, DRAW_TO_LCD);
   GIF_begin(&gif, BIG_ENDIAN_PIXELS, GIF_PALETTE_RGB565);
} /* setup() */

int main() {
   setup();

  while (1) {
    if (GIF_openRAM(&gif, (uint8_t *)raspberrypi, sizeof(raspberrypi), GIFDraw)) {
//    if (GIF_openRAM(&gif, (uint8_t *)badgers_320, sizeof(badgers_320), GIFDraw)) {
      int delay;
      while (GIF_playFrame(&gif, &delay)) {
         //sleep_ms(delay);
      }
      GIF_close(&gif);
    }
  } // while 1
} // main
