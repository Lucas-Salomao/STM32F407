/*
 * ILI9341 LCD driver v2020.05.27
 * 2019.11. Add RGB mode with memory frame buffer (in sram or sdram)
 * 2020.02. Add analog touchscreen (only 8bit paralell mode)
 * 2020.05  Add Scroll function
 * 2021.05  Touch screen driver modify
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "lcd.h"
#include "bmp.h"
#include "ili9341.h"
#include "fonts.h"

void     ili9341_Init(void);
uint16_t ili9341_ReadID(void);
void     ili9341_DisplayOn(void);
void     ili9341_DisplayOff(void);
void     ili9341_SetCursor(uint16_t Xpos, uint16_t Ypos);
void     ili9341_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code);
uint16_t ili9341_ReadPixel(uint16_t Xpos, uint16_t Ypos);
void     ili9341_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     ili9341_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     ili9341_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     ili9341_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t RGBCode);
uint16_t ili9341_GetLcdPixelWidth(void);
uint16_t ili9341_GetLcdPixelHeight(void);
void     ili9341_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);
void     ili9341_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData);
void     ili9341_ReadRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData);
void     ili9341_Scroll(int16_t Scroll, uint16_t TopFix, uint16_t BottonFix);
void	 ili9341_FillCircle(int16_t Xpos, int16_t Ypos, int16_t r, uint16_t RGBCode);
void 	 LCD_TxBMP(unsigned char *data, unsigned int BitmapStart);


uint8_t cursor_y  =0, cursor_x    = 0;
uint8_t textsize  = 1;
uint16_t textcolor =0xffff,  textbgcolor = 0xFFFF;
uint8_t wrap      = 1;

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned long *)(addr))	//com essa correção passa a ler toda a memória
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))

#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

LCD_DrvTypeDef   ili9341_drv =
{
  ili9341_Init,
  ili9341_ReadID,
  ili9341_DisplayOn,
  ili9341_DisplayOff,
  ili9341_SetCursor,
  ili9341_WritePixel,
  ili9341_ReadPixel,
  ili9341_SetDisplayWindow,
  ili9341_DrawHLine,
  ili9341_DrawVLine,
  ili9341_GetLcdPixelWidth,
  ili9341_GetLcdPixelHeight,
  ili9341_DrawBitmap,
  ili9341_DrawRGBImage,
  ili9341_FillRect,
  ili9341_ReadRGBImage,
  ili9341_Scroll,
};

LCD_DrvTypeDef  *lcd_drv = &ili9341_drv;

#define ILI9341_NOP            0x00
#define ILI9341_SWRESET        0x01

#define ILI9341_RDDID          0x04
#define ILI9341_RDDST          0x09
#define ILI9341_RDMODE         0x0A
#define ILI9341_RDMADCTL       0x0B
#define ILI9341_RDPIXFMT       0x0C
#define ILI9341_RDIMGFMT       0x0D
#define ILI9341_RDSELFDIAG     0x0F

#define ILI9341_SLPIN          0x10
#define ILI9341_SLPOUT         0x11
#define ILI9341_PTLON          0x12
#define ILI9341_NORON          0x13

#define ILI9341_INVOFF         0x20
#define ILI9341_INVON          0x21
#define ILI9341_GAMMASET       0x26
#define ILI9341_DISPOFF        0x28
#define ILI9341_DISPON         0x29

#define ILI9341_CASET          0x2A
#define ILI9341_PASET          0x2B
#define ILI9341_RAMWR          0x2C
#define ILI9341_RAMRD          0x2E

#define ILI9341_PTLAR          0x30
#define ILI9341_VSCRDEF        0x33
#define ILI9341_MADCTL         0x36
#define ILI9341_VSCRSADD       0x37     /* Vertical Scrolling Start Address */
#define ILI9341_PIXFMT         0x3A     /* COLMOD: Pixel Format Set */

#define ILI9341_RGB_INTERFACE  0xB0     /* RGB Interface Signal Control */
#define ILI9341_FRMCTR1        0xB1
#define ILI9341_FRMCTR2        0xB2
#define ILI9341_FRMCTR3        0xB3
#define ILI9341_INVCTR         0xB4
#define ILI9341_DFUNCTR        0xB6     /* Display Function Control */

#define ILI9341_PWCTR1         0xC0
#define ILI9341_PWCTR2         0xC1
#define ILI9341_PWCTR3         0xC2
#define ILI9341_PWCTR4         0xC3
#define ILI9341_PWCTR5         0xC4
#define ILI9341_VMCTR1         0xC5
#define ILI9341_VMCTR2         0xC7

#define ILI9341_RDID1          0xDA
#define ILI9341_RDID2          0xDB
#define ILI9341_RDID3          0xDC
#define ILI9341_RDID4          0xDD

#define ILI9341_GMCTRP1        0xE0
#define ILI9341_GMCTRN1        0xE1

#define ILI9341_PWCTR6         0xFC
#define ILI9341_INTERFACE      0xF6   /* Interface control register */

/* Extend register commands */
#define ILI9341_POWERA         0xCB   /* Power control A register */
#define ILI9341_POWERB         0xCF   /* Power control B register */
#define ILI9341_DTCA           0xE8   /* Driver timing control A */
#define ILI9341_DTCB           0xEA   /* Driver timing control B */
#define ILI9341_POWER_SEQ      0xED   /* Power on sequence register */
#define ILI9341_3GAMMA_EN      0xF2   /* 3 Gamma enable register */
#define ILI9341_PRC            0xF7   /* Pump ratio control register */

//-----------------------------------------------------------------------------
#define ILI9341_MAD_RGB        0x08
#define ILI9341_MAD_BGR        0x00

#define ILI9341_MAD_VERTICAL   0x20
#define ILI9341_MAD_X_LEFT     0x00
#define ILI9341_MAD_X_RIGHT    0x40
#define ILI9341_MAD_Y_UP       0x80
#define ILI9341_MAD_Y_DOWN     0x00

#if ILI9341_COLORMODE == 0
#define ILI9341_MAD_COLORMODE  ILI9341_MAD_RGB
#else
#define ILI9341_MAD_COLORMODE  ILI9341_MAD_BGR
#endif

#if (ILI9341_ORIENTATION == 0)
#define ILI9341_SIZE_X                     ILI9341_LCD_PIXEL_WIDTH
#define ILI9341_SIZE_Y                     ILI9341_LCD_PIXEL_HEIGHT
#define ILI9341_MAD_DATA_RIGHT_THEN_UP     ILI9341_MAD_COLORMODE | ILI9341_MAD_X_RIGHT | ILI9341_MAD_Y_UP
#define ILI9341_MAD_DATA_RIGHT_THEN_DOWN   ILI9341_MAD_COLORMODE | ILI9341_MAD_X_RIGHT | ILI9341_MAD_Y_DOWN
#define ILI9341_MAD_DATA_RGBMODE           ILI9341_MAD_COLORMODE | ILI9341_MAD_X_LEFT | ILI9341_MAD_Y_DOWN
#define XPOS                               Xpos
#define YPOS                               Ypos
#define XSIZE                              Xsize
#define YSIZE                              Ysize
#define XSTEP                              1
#define YSTEP                              ILI9341_LCD_PIXEL_WIDTH
#elif (ILI9341_ORIENTATION == 1)
#define ILI9341_SIZE_X                     ILI9341_LCD_PIXEL_HEIGHT
#define ILI9341_SIZE_Y                     ILI9341_LCD_PIXEL_WIDTH
#define ILI9341_MAD_DATA_RIGHT_THEN_UP     ILI9341_MAD_COLORMODE | ILI9341_MAD_X_RIGHT | ILI9341_MAD_Y_DOWN | ILI9341_MAD_VERTICAL
#define ILI9341_MAD_DATA_RIGHT_THEN_DOWN   ILI9341_MAD_COLORMODE | ILI9341_MAD_X_LEFT  | ILI9341_MAD_Y_DOWN | ILI9341_MAD_VERTICAL
#define ILI9341_MAD_DATA_RGBMODE           ILI9341_MAD_COLORMODE | ILI9341_MAD_X_RIGHT | ILI9341_MAD_Y_DOWN
#define XPOS                               Ypos
#define YPOS                               Xpos
#define XSIZE                              Ysize
#define YSIZE                              Xsize
#define XSTEP                              ILI9341_LCD_PIXEL_WIDTH
#define YSTEP                              1
#elif (ILI9341_ORIENTATION == 2)
#define ILI9341_SIZE_X                     ILI9341_LCD_PIXEL_WIDTH
#define ILI9341_SIZE_Y                     ILI9341_LCD_PIXEL_HEIGHT
#define ILI9341_MAD_DATA_RIGHT_THEN_UP     ILI9341_MAD_COLORMODE | ILI9341_MAD_X_LEFT  | ILI9341_MAD_Y_DOWN
#define ILI9341_MAD_DATA_RIGHT_THEN_DOWN   ILI9341_MAD_COLORMODE | ILI9341_MAD_X_LEFT  | ILI9341_MAD_Y_UP
#define ILI9341_MAD_DATA_RGBMODE           ILI9341_MAD_COLORMODE | ILI9341_MAD_X_RIGHT | ILI9341_MAD_Y_UP
#define XPOS                               Xpos
#define YPOS                               Ypos
#define XSIZE                              Xsize
#define YSIZE                              Ysize
#define XSTEP                              1
#define YSTEP                              ILI9341_LCD_PIXEL_WIDTH
#elif (ILI9341_ORIENTATION == 3)
#define ILI9341_SIZE_X                     ILI9341_LCD_PIXEL_HEIGHT
#define ILI9341_SIZE_Y                     ILI9341_LCD_PIXEL_WIDTH
#define ILI9341_MAD_DATA_RIGHT_THEN_UP     ILI9341_MAD_COLORMODE | ILI9341_MAD_X_LEFT  | ILI9341_MAD_Y_UP   | ILI9341_MAD_VERTICAL
#define ILI9341_MAD_DATA_RIGHT_THEN_DOWN   ILI9341_MAD_COLORMODE | ILI9341_MAD_X_RIGHT | ILI9341_MAD_Y_UP   | ILI9341_MAD_VERTICAL
#define ILI9341_MAD_DATA_RGBMODE           ILI9341_MAD_COLORMODE | ILI9341_MAD_X_LEFT  | ILI9341_MAD_Y_UP
#define XPOS                               Ypos
#define YPOS                               Xpos
#define XSIZE                              Ysize
#define YSIZE                              Xsize
#define XSTEP                              ILI9341_LCD_PIXEL_WIDTH
#define YSTEP                              1
#endif

#define ILI9341_SETCURSOR(x, y)            {LCD_IO_WriteCmd8(ILI9341_CASET); LCD_IO_WriteData16_to_2x8(x); LCD_IO_WriteData16_to_2x8(x); \
                                            LCD_IO_WriteCmd8(ILI9341_PASET); LCD_IO_WriteData16_to_2x8(y); LCD_IO_WriteData16_to_2x8(y); }

//-----------------------------------------------------------------------------
#define ILI9341_LCD_INITIALIZED    0x01
#define ILI9341_IO_INITIALIZED     0x02
static  uint8_t   Is_ili9341_Initialized = 0;

#if ILI9341_INTERFACE_MODE == 1
static  uint16_t  yStart, yEnd;

#if      ILI9341_MULTITASK_MUTEX == 1 && ILI9341_TOUCH == 1
volatile uint8_t io_lcd_busy = 0;
volatile uint8_t io_ts_busy = 0;
#define  ILI9341_LCDMUTEX_PUSH()    while(io_ts_busy); io_lcd_busy++;
#define  ILI9341_LCDMUTEX_POP()     io_lcd_busy--
#else
#define  ILI9341_LCDMUTEX_PUSH()
#define  ILI9341_LCDMUTEX_POP()
#endif

#endif   // #if ILI9341_INTERFACE_MODE == 1

//-----------------------------------------------------------------------------

#if ILI9341_INTERFACE_MODE == 2

#define  ILI9341_LCDMUTEX_PUSH()
#define  ILI9341_LCDMUTEX_POP()

typedef struct
{
  uint16_t   Xsize;
  uint16_t   Ysize;
  uint16_t * Pixels;
}LCD_FramebufferDef;

LCD_FramebufferDef   FrameBuffer;
#endif

//-----------------------------------------------------------------------------
/* Link function for LCD peripheral */
void     LCD_Delay (uint32_t delay);
void     LCD_IO_Init(void);
void     LCD_IO_Bl_OnOff(uint8_t Bl);

void     LCD_IO_WriteCmd8(uint8_t Cmd);
void     LCD_IO_WriteCmd16(uint16_t Cmd);
void     LCD_IO_WriteData8(uint8_t Data);
void     LCD_IO_WriteData16(uint16_t Data);

void     LCD_IO_WriteCmd8DataFill16(uint8_t Cmd, uint16_t Data, uint32_t Size);
void     LCD_IO_WriteCmd8MultipleData8(uint8_t Cmd, uint8_t *pData, uint32_t Size);
void     LCD_IO_WriteCmd8MultipleData16(uint8_t Cmd, uint16_t *pData, uint32_t Size);
void     LCD_IO_WriteCmd16DataFill16(uint16_t Cmd, uint16_t Data, uint32_t Size);
void     LCD_IO_WriteCmd16MultipleData8(uint16_t Cmd, uint8_t *pData, uint32_t Size);
void     LCD_IO_WriteCmd16MultipleData16(uint16_t Cmd, uint16_t *pData, uint32_t Size);

void     LCD_IO_ReadCmd8MultipleData8(uint8_t Cmd, uint8_t *pData, uint32_t Size, uint32_t DummySize);
void     LCD_IO_ReadCmd8MultipleData16(uint8_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize);
void     LCD_IO_ReadCmd8MultipleData24to16(uint8_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize);
void     LCD_IO_ReadCmd16MultipleData8(uint16_t Cmd, uint8_t *pData, uint32_t Size, uint32_t DummySize);
void     LCD_IO_ReadCmd16MultipleData16(uint16_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize);
void     LCD_IO_ReadCmd16MultipleData24to16(uint16_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize);

#define  LCD_IO_WriteData16_to_2x8(dt)    {LCD_IO_WriteData8((dt) >> 8); LCD_IO_WriteData8(dt); }

//-----------------------------------------------------------------------------
/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOn(void)
{
  LCD_IO_Bl_OnOff(1);
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8(ILI9341_SLPOUT);    // Exit Sleep
  ILI9341_LCDMUTEX_POP();
}

//-----------------------------------------------------------------------------
/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOff(void)
{
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8(ILI9341_SLPIN);    // Sleep
  ILI9341_LCDMUTEX_POP();
  LCD_IO_Bl_OnOff(0);
}

//-----------------------------------------------------------------------------
/**
  * @brief  Get the LCD pixel Width.
  * @param  None
  * @retval The Lcd Pixel Width
  */
uint16_t ili9341_GetLcdPixelWidth(void)
{
  return ILI9341_SIZE_X;
}

//-----------------------------------------------------------------------------
/**
  * @brief  Get the LCD pixel Height.
  * @param  None
  * @retval The Lcd Pixel Height
  */
uint16_t ili9341_GetLcdPixelHeight(void)
{
  return ILI9341_SIZE_Y;
}

//-----------------------------------------------------------------------------
/**
  * @brief  Get the ILI9341 ID.
  * @param  None
  * @retval The ILI9341 ID
  * @rem    On the my lcd is unusable (stm32f429 discovery)
  */
uint16_t ili9341_ReadID(void)
{
  uint32_t dt = 0;
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_ReadCmd8MultipleData8(0xD3, (uint8_t *)&dt, 3, 1);
  ILI9341_LCDMUTEX_POP();
  if(dt == 0x419300)
    return 0x9341;
  else
    return 0;
}

/* SPI or paralell mode */
#if ILI9341_INTERFACE_MODE == 1

//-----------------------------------------------------------------------------
void ili9341_Init(void)
{
  if((Is_ili9341_Initialized & ILI9341_LCD_INITIALIZED) == 0)
  {
    Is_ili9341_Initialized |= ILI9341_LCD_INITIALIZED;
    if((Is_ili9341_Initialized & ILI9341_IO_INITIALIZED) == 0)
      LCD_IO_Init();
    Is_ili9341_Initialized |= ILI9341_IO_INITIALIZED;
  }

  LCD_Delay(10);
  LCD_IO_WriteCmd8(ILI9341_SWRESET);
  LCD_Delay(10);

  LCD_IO_WriteCmd8MultipleData8(0xEF, (uint8_t *)"\x03\x80\x02", 3);
  LCD_IO_WriteCmd8MultipleData8(0xCF, (uint8_t *)"\x00\xC1\x30", 3);
  LCD_IO_WriteCmd8MultipleData8(0xED, (uint8_t *)"\x64\x03\x12\x81", 4);
  LCD_IO_WriteCmd8MultipleData8(0xE8, (uint8_t *)"\x85\x00\x78", 3);
  LCD_IO_WriteCmd8MultipleData8(0xCB, (uint8_t *)"\x39\x2C\x00\x34\x02", 5);
  LCD_IO_WriteCmd8MultipleData8(0xF7, (uint8_t *)"\x20", 1);
  LCD_IO_WriteCmd8MultipleData8(0xEA, (uint8_t *)"\x00\x00", 2);

  // Power Control 1 (Vreg1out, Verg2out)
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PWCTR1, (uint8_t *)"\x23", 1);

  // Power Control 2 (VGH,VGL)
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PWCTR2, (uint8_t *)"\x10", 1);

  // Power Control 3 (Vcom)
  LCD_IO_WriteCmd8MultipleData8(ILI9341_VMCTR1, (uint8_t *)"\x3E\x28", 2);

  // Power Control 3 (Vcom)
  LCD_IO_WriteCmd8MultipleData8(ILI9341_VMCTR2, (uint8_t *)"\x86", 1);

  // Vertical scroll zero
  LCD_IO_WriteCmd8MultipleData8(ILI9341_VSCRSADD, (uint8_t *)"\x00", 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PIXFMT, (uint8_t *)"\x55", 1);

  // LCD_IO_WriteCmd8MultipleData8(0xF6, (uint8_t *)"\x01\x00\x06", 3);

  LCD_IO_WriteCmd8MultipleData8(ILI9341_FRMCTR1, (uint8_t *)"\x00\x18", 2);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_DFUNCTR, (uint8_t *)"\x08\x82\x27", 3);  // Display Function Control
  LCD_IO_WriteCmd8MultipleData8(0xF2, (uint8_t *)"\x00", 1);            // 3Gamma Function Disable
  LCD_IO_WriteCmd8MultipleData8(ILI9341_GAMMASET, (uint8_t *)"\x01", 1);// Gamma curve selected

  // positive gamma control
  LCD_IO_WriteCmd8MultipleData8(ILI9341_GMCTRP1, (uint8_t *)"\x0F\x31\x2B\x0C\x0E\x08\x4E\xF1\x37\x07\x10\x03\x0E\x09\x00", 15);

  // negative gamma control
  LCD_IO_WriteCmd8MultipleData8(ILI9341_GMCTRN1, (uint8_t *)"\x00\x0E\x14\x03\x11\x07\x31\xC1\x48\x08\x0F\x0C\x31\x36\x0F", 15);

  LCD_IO_WriteCmd8(ILI9341_MADCTL); LCD_IO_WriteData8(ILI9341_MAD_DATA_RIGHT_THEN_DOWN);
  LCD_IO_WriteCmd8(ILI9341_SLPOUT);    // Exit Sleep
  LCD_Delay(10);

  #if ILI9341_INITCLEAR == 1
  ili9341_FillRect(0, 0, ILI9341_SIZE_X, ILI9341_SIZE_Y, COLOR_BLUE);
  LCD_Delay(10);
  #endif
  
  LCD_IO_WriteCmd8(ILI9341_DISPON);    // Display on
  LCD_Delay(10);
}

//-----------------------------------------------------------------------------
/**
  * @brief  Set Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void ili9341_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  ILI9341_LCDMUTEX_PUSH();
  ILI9341_SETCURSOR(Xpos, Ypos);
  ILI9341_LCDMUTEX_POP();
}

//-----------------------------------------------------------------------------
/**
  * @brief  Write pixel.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGBCode: the RGB pixel color
  * @retval None
  */
void ili9341_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{
  ILI9341_LCDMUTEX_PUSH();
  ILI9341_SETCURSOR(Xpos, Ypos);
  LCD_IO_WriteCmd8(ILI9341_RAMWR); LCD_IO_WriteData16(RGBCode);
  ILI9341_LCDMUTEX_POP();
}


//-----------------------------------------------------------------------------
/**
  * @brief  Read pixel.
  * @param  None
  * @retval the RGB pixel color
  */
uint16_t ili9341_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  uint16_t ret;
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PIXFMT, (uint8_t *)"\x66", 1); // Read: only 24bit pixel mode
  ILI9341_SETCURSOR(Xpos, Ypos);
  LCD_IO_ReadCmd8MultipleData24to16(ILI9341_RAMRD, (uint16_t *)&ret, 1, 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PIXFMT, (uint8_t *)"\x55", 1); // Return to 16bit pixel mode
  ILI9341_LCDMUTEX_POP();
  return(ret);
}

//-----------------------------------------------------------------------------
/**
  * @brief  Sets a display window
  * @param  Xpos:   specifies the X bottom left position.
  * @param  Ypos:   specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width:  display window width.
  * @retval None
  */
void ili9341_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  yStart = Ypos; yEnd = Ypos + Height - 1;
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8(ILI9341_CASET); LCD_IO_WriteData16_to_2x8(Xpos); LCD_IO_WriteData16_to_2x8(Xpos + Width - 1);
  LCD_IO_WriteCmd8(ILI9341_PASET); LCD_IO_WriteData16_to_2x8(Ypos); LCD_IO_WriteData16_to_2x8(Ypos + Height - 1);
  ILI9341_LCDMUTEX_POP();
}

//-----------------------------------------------------------------------------
/**
  * @brief  Draw vertical line.
  * @param  RGBCode: Specifies the RGB color
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.
  * @retval None
  */
void ili9341_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8(ILI9341_CASET); LCD_IO_WriteData16_to_2x8(Xpos); LCD_IO_WriteData16_to_2x8(Xpos + Length - 1);
  LCD_IO_WriteCmd8(ILI9341_PASET); LCD_IO_WriteData16_to_2x8(Ypos); LCD_IO_WriteData16_to_2x8(Ypos);
  LCD_IO_WriteCmd8DataFill16(ILI9341_RAMWR, RGBCode, Length);
  ILI9341_LCDMUTEX_POP();
}

//-----------------------------------------------------------------------------
/**
  * @brief  Draw vertical line.
  * @param  RGBCode: Specifies the RGB color
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.
  * @retval None
  */
void ili9341_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8(ILI9341_CASET); LCD_IO_WriteData16_to_2x8(Xpos); LCD_IO_WriteData16_to_2x8(Xpos);
  LCD_IO_WriteCmd8(ILI9341_PASET); LCD_IO_WriteData16_to_2x8(Ypos); LCD_IO_WriteData16_to_2x8(Ypos + Length - 1);
  LCD_IO_WriteCmd8DataFill16(ILI9341_RAMWR, RGBCode, Length);
  ILI9341_LCDMUTEX_POP();
}

//-----------------------------------------------------------------------------
/**
  * @brief  Draw Filled rectangle
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Xsize:    specifies the X size
  * @param  Ysize:    specifies the Y size
  * @param  RGBCode:  specifies the RGB color
  * @retval None
  */
void ili9341_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t RGBCode)
{
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8(ILI9341_CASET); LCD_IO_WriteData16_to_2x8(Xpos); LCD_IO_WriteData16_to_2x8(Xpos + Xsize - 1);
  LCD_IO_WriteCmd8(ILI9341_PASET); LCD_IO_WriteData16_to_2x8(Ypos); LCD_IO_WriteData16_to_2x8(Ypos + Ysize - 1);
  LCD_IO_WriteCmd8DataFill16(ILI9341_RAMWR, RGBCode, Xsize * Ysize);
  ILI9341_LCDMUTEX_POP();
}

void  ili9341_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	ili9341_FillRect(x, y, 1, h, color);
}
void  ili9341_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	ili9341_FillRect(x, y, w, 1, color);
}

void ili9341_FillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color)
{

    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;

    delta++; // Avoid some +1's in the loop

    while(x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if(x < (y + 1)) {
            if(corners & 1) ili9341_DrawFastVLine(x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) ili9341_DrawFastVLine(x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py) {
            if(corners & 1) ili9341_DrawFastVLine(x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) ili9341_DrawFastVLine(x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

void ili9341_FillCircle(int16_t Xpos, int16_t Ypos, int16_t r, uint16_t RGBColor)
{
	ili9341_DrawFastVLine(Xpos, Ypos-r, 2*r+1, RGBColor);
	ili9341_FillCircleHelper(Xpos, Ypos, r, 3, 0, RGBColor);
}

//-----------------------------------------------------------------------------
/**
  * @brief  Displays a 16bit bitmap picture..
  * @param  BmpAddress: Bmp picture address.
  * @param  Xpos:  Bmp X position in the LCD
  * @param  Ypos:  Bmp Y position in the LCD
  * @retval None
  * @brief  Draw direction: right then up
  */
void ili9341_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint32_t index, size;
  /* Read bitmap size */
  size = ((BITMAPSTRUCT *)pbmp)->fileHeader.bfSize;
  /* Get bitmap data address offset */
  index = ((BITMAPSTRUCT *)pbmp)->fileHeader.bfOffBits;
  size = (size - index) / 2;
  pbmp += index;

  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8(ILI9341_MADCTL); LCD_IO_WriteData8(ILI9341_MAD_DATA_RIGHT_THEN_UP);
  LCD_IO_WriteCmd8(ILI9341_PASET); LCD_IO_WriteData16_to_2x8(ILI9341_SIZE_Y - 1 - yEnd); LCD_IO_WriteData16_to_2x8(ILI9341_SIZE_Y - 1 - yStart);
  LCD_IO_WriteCmd8MultipleData16(ILI9341_RAMWR, (uint16_t *)pbmp, size);
  LCD_IO_WriteCmd8(ILI9341_MADCTL); LCD_IO_WriteData8(ILI9341_MAD_DATA_RIGHT_THEN_DOWN);
  ILI9341_LCDMUTEX_POP();
}

//-----------------------------------------------------------------------------
/**
  * @brief  Displays 16bit/pixel picture..
  * @param  pdata: picture address.
  * @param  Xpos:  Image X position in the LCD
  * @param  Ypos:  Image Y position in the LCD
  * @param  Xsize: Image X size in the LCD
  * @param  Ysize: Image Y size in the LCD
  * @retval None
  * @brief  Draw direction: right then down
  */
void ili9341_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData)
{
  ili9341_SetDisplayWindow(Xpos, Ypos, Xsize, Ysize);
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8MultipleData16(ILI9341_RAMWR, pData, Xsize * Ysize);
  ILI9341_LCDMUTEX_POP();
}

//-----------------------------------------------------------------------------
/**
  * @brief  Read 16bit/pixel picture from Lcd and store to RAM
  * @param  pdata: picture address.
  * @param  Xpos:  Image X position in the LCD
  * @param  Ypos:  Image Y position in the LCD
  * @param  Xsize: Image X size in the LCD
  * @param  Ysize: Image Y size in the LCD
  * @retval None
  * @brief  Draw direction: right then down
  */
void ili9341_ReadRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData)
{
  ili9341_SetDisplayWindow(Xpos, Ypos, Xsize, Ysize);
  ILI9341_LCDMUTEX_PUSH();
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PIXFMT, (uint8_t *)"\x66", 1); // Read: only 24bit pixel mode
  LCD_IO_ReadCmd8MultipleData24to16(ILI9341_RAMRD, pData, Xsize * Ysize, 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PIXFMT, (uint8_t *)"\x55", 1); // Return to 16bit pixel mode
  ILI9341_LCDMUTEX_POP();
}

//-----------------------------------------------------------------------------
/**
  * @brief  Set display scroll parameters
  * @param  Scroll    : Scroll size [pixel]
  * @param  TopFix    : Top fix size [pixel]
  * @param  BottonFix : Botton fix size [pixel]
  * @retval None
  */
void ili9341_Scroll(int16_t Scroll, uint16_t TopFix, uint16_t BottonFix)
{
  static uint16_t scrparam[4] = {0, 0, 0, 0};
  ILI9341_LCDMUTEX_PUSH();
  #if (ILI9341_ORIENTATION == 0)
  if((TopFix != scrparam[1]) || (BottonFix != scrparam[3]))
  {
    scrparam[1] = TopFix;
    scrparam[3] = BottonFix;
    scrparam[2] = ILI9341_LCD_PIXEL_HEIGHT - TopFix - BottonFix;
    LCD_IO_WriteCmd8MultipleData16(ILI9341_VSCRDEF, &scrparam[1], 3);
  }
  Scroll = (0 - Scroll) % scrparam[2];
  if(Scroll < 0)
    Scroll = scrparam[2] + Scroll + scrparam[1];
  else
    Scroll = Scroll + scrparam[1];
  #elif (ILI9341_ORIENTATION == 1)
  if((TopFix != scrparam[1]) || (BottonFix != scrparam[3]))
  {
    scrparam[1] = TopFix;
    scrparam[3] = BottonFix;
    scrparam[2] = ILI9341_LCD_PIXEL_HEIGHT - TopFix - BottonFix;
    LCD_IO_WriteCmd8MultipleData16(ILI9341_VSCRDEF, &scrparam[1], 3);
  }
  Scroll = (0 - Scroll) % scrparam[2];
  if(Scroll < 0)
    Scroll = scrparam[2] + Scroll + scrparam[1];
  else
    Scroll = Scroll + scrparam[1];
  #elif (ILI9341_ORIENTATION == 2)
  if((TopFix != scrparam[3]) || (BottonFix != scrparam[1]))
  {
    scrparam[3] = TopFix;
    scrparam[1] = BottonFix;
    scrparam[2] = ILI9341_LCD_PIXEL_HEIGHT - TopFix - BottonFix;
    LCD_IO_WriteCmd8MultipleData16(ILI9341_VSCRDEF, &scrparam[1], 3);
  }
  Scroll %= scrparam[2];
  if(Scroll < 0)
    Scroll = scrparam[2] + Scroll + scrparam[1];
  else
    Scroll = Scroll + scrparam[1];
  #elif (ILI9341_ORIENTATION == 3)
  if((TopFix != scrparam[3]) || (BottonFix != scrparam[1]))
  {
    scrparam[3] = TopFix;
    scrparam[1] = BottonFix;
    scrparam[2] = ILI9341_LCD_PIXEL_HEIGHT - TopFix - BottonFix;
    LCD_IO_WriteCmd8MultipleData16(ILI9341_VSCRDEF, &scrparam[1], 3);
  }
  Scroll %= scrparam[2];
  if(Scroll < 0)
    Scroll = scrparam[2] + Scroll + scrparam[1];
  else
    Scroll = Scroll + scrparam[1];
  #endif
  if(Scroll != scrparam[0])
  {
    scrparam[0] = Scroll;
    LCD_IO_WriteCmd8DataFill16(ILI9341_VSCRSADD, scrparam[0], 1);
  }
  ILI9341_LCDMUTEX_POP();
}

#endif /* #if ILI9341_INTERFACE_MODE == 1 */

//=============================================================================
/* RGB mode */

#if ILI9341_INTERFACE_MODE == 2

//-----------------------------------------------------------------------------
void ili9341_Init(void)
{
  if((Is_ili9341_Initialized & ILI9341_LCD_INITIALIZED) == 0)
  {
    Is_ili9341_Initialized |= ILI9341_LCD_INITIALIZED;
    if((Is_ili9341_Initialized & ILI9341_IO_INITIALIZED) == 0)
      LCD_IO_Init();
    Is_ili9341_Initialized |= ILI9341_IO_INITIALIZED;
  }

  LCD_Delay(10);
  LCD_IO_WriteCmd8(ILI9341_SWRESET);
  LCD_Delay(10);

  LCD_IO_WriteCmd8MultipleData8(0xCA, (uint8_t *)"\xC3\x08\x50", 3);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_POWERB, (uint8_t *)"\x00\xC1\x30", 3);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_POWER_SEQ, (uint8_t *)"\x64\x03\x12\x81", 4);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_DTCA, (uint8_t *)"\x85\x00\x78", 3);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_POWERA, (uint8_t *)"\x39\x2C\x00\x34\x02", 5);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PRC, (uint8_t *)"\x20", 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_DTCB, (uint8_t *)"\x00\x00", 2);

  LCD_IO_WriteCmd8MultipleData8(ILI9341_FRMCTR1, (uint8_t *)"\x00\x18", 2);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_DFUNCTR, (uint8_t *)"\x0A\xA2", 2);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PWCTR1, (uint8_t *)"\x10", 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_PWCTR2, (uint8_t *)"\x10", 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_VMCTR1, (uint8_t *)"\x45\x15", 2);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_VMCTR2, (uint8_t *)"\x90", 1);
  LCD_IO_WriteCmd8(ILI9341_MADCTL); LCD_IO_WriteData8(ILI9341_MAD_DATA_RGBMODE);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_3GAMMA_EN, (uint8_t *)"\x00", 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_RGB_INTERFACE, (uint8_t *)"\xC2", 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_DFUNCTR, (uint8_t *)"\x0A\xA7\x27\x04", 4);

  LCD_IO_WriteCmd8(ILI9341_CASET); LCD_IO_WriteData16(0); LCD_IO_WriteData16(ILI9341_LCD_PIXEL_WIDTH - 1);
  LCD_IO_WriteCmd8(ILI9341_PASET); LCD_IO_WriteData16(0); LCD_IO_WriteData16(ILI9341_LCD_PIXEL_HEIGHT - 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_INTERFACE, (uint8_t *)"\x01\x00\x06", 3);
  LCD_IO_WriteCmd8(ILI9341_RAMWR);
  LCD_Delay(200);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_GAMMASET, (uint8_t *)"\x01", 1);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_GMCTRP1, (uint8_t *)"\x0F\x29\x24\x0C\x0E\x09\x4E\x78\x3C\x09\x13\x05\x17\x11\x00", 15);
  LCD_IO_WriteCmd8MultipleData8(ILI9341_GMCTRN1, (uint8_t *)"\x00\x16\x1B\x04\x11\x07\x31\x33\x42\x05\x0C\x0A\x28\x2F\x0F", 15);
  LCD_IO_WriteCmd8(ILI9341_SLPOUT);    // Exit Sleep
  LCD_Delay(200);
  LCD_IO_WriteCmd8(ILI9341_DISPON);    // Display on
  LCD_IO_WriteCmd8(ILI9341_RAMWR);
  LCD_Delay(10);

  /* Memory alloc from frame buffer and LTDC address setting */
  FrameBuffer.Pixels = ILI9341_MALLOC(ILI9341_LCD_PIXEL_WIDTH * ILI9341_LCD_PIXEL_HEIGHT * 2);
  if(FrameBuffer.Pixels)
  {
    FrameBuffer.Xsize = ILI9341_LCD_PIXEL_WIDTH;
    FrameBuffer.Ysize = ILI9341_LCD_PIXEL_HEIGHT;
    HAL_LTDC_SetAddress(&hltdc, (uint32_t)FrameBuffer.Pixels, 0);
  }
  else
    while(1); // Can't allocate the framebuffer memory
}

//-----------------------------------------------------------------------------
/**
  * @brief  Set Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void ili9341_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
}

//-----------------------------------------------------------------------------
/**
  * @brief  Write pixel.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGBCode: the RGB pixel color
  * @retval None
  */
void ili9341_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{
  *(volatile uint16_t *)((YPOS * FrameBuffer.Xsize + XPOS) + FrameBuffer.Pixels) = RGBCode;
}

//-----------------------------------------------------------------------------
/**
  * @brief  Read pixel.
  * @param  None
  * @retval the RGB pixel color
  */
uint16_t ili9341_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  return *(uint16_t *)((YPOS * FrameBuffer.Xsize + XPOS) + FrameBuffer.Pixels);
}

//-----------------------------------------------------------------------------
/**
  * @brief  Sets a display window
  * @param  Xpos:   specifies the X bottom left position.
  * @param  Ypos:   specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width:  display window width.
  * @retval None
  */
void ili9341_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
}

//-----------------------------------------------------------------------------
/**
  * @brief  Draw vertical line.
  * @param  RGBCode: Specifies the RGB color
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.
  * @retval None
  */
void ili9341_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint16_t * p = ((YPOS * FrameBuffer.Xsize + XPOS) + FrameBuffer.Pixels);
  while(Length--)
  {
    *p = RGBCode;
    p += XSTEP;
  }
}

//-----------------------------------------------------------------------------
/**
  * @brief  Draw vertical line.
  * @param  RGBCode: Specifies the RGB color
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.
  * @retval None
  */
void ili9341_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint16_t * p = ((YPOS * FrameBuffer.Xsize + XPOS) + FrameBuffer.Pixels);
  while(Length--)
  {
    *p = RGBCode;
    p += YSTEP;
  }
}

//-----------------------------------------------------------------------------
/**
  * @brief  Draw Filled rectangle
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Xsize:    specifies the X size
  * @param  Ysize:    specifies the Y size
  * @param  RGBCode:  specifies the RGB color
  * @retval None
  */
void ili9341_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t RGBCode)
{
  uint16_t * p = ((YPOS * FrameBuffer.Xsize + XPOS) + FrameBuffer.Pixels);
  uint16_t yp = (FrameBuffer.Xsize - XSIZE);
  while(YSIZE--)
  {
    for(uint16_t x = 0; x < XSIZE; x++)
    {
      *p = RGBCode;
      p++;
    }
    p += yp;
  }
}

//-----------------------------------------------------------------------------
/**
  * @brief  Displays a 16bit bitmap picture..
  * @param  BmpAddress: Bmp picture address.
  * @param  Xpos:  Bmp X position in the LCD
  * @param  Ypos:  Bmp Y position in the LCD
  * @retval None
  * @brief  Draw direction: right then up
  */
void ili9341_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint16_t Xsize = ((BITMAPSTRUCT *)pbmp)->infoHeader.biWidth;
  uint16_t Ysize = ((BITMAPSTRUCT *)pbmp)->infoHeader.biHeight;
  Ypos = Ypos + Ysize - 1;
  uint16_t * yp = ((YPOS * FrameBuffer.Xsize + XPOS) + FrameBuffer.Pixels);

  pbmp += ((BITMAPSTRUCT *)pbmp)->fileHeader.bfOffBits;
  while(Ysize--)
  {
    uint16_t * p = yp;
    for(uint16_t x = 0; x < Xsize; x++)
    {
      *p = *(uint16_t *)pbmp;
      p += XSTEP;
      pbmp += 2;
    }
    yp -= YSTEP;
  }
}

//-----------------------------------------------------------------------------
/**
  * @brief  Displays 16bit/pixel picture..
  * @param  pdata: picture address.
  * @param  Xpos:  Image X position in the LCD
  * @param  Ypos:  Image Y position in the LCD
  * @param  Xsize: Image X size in the LCD
  * @param  Ysize: Image Y size in the LCD
  * @retval None
  * @brief  Draw direction: right then down
  */
void ili9341_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData)
{
  uint16_t * p = ((YPOS * FrameBuffer.Xsize + XPOS) + FrameBuffer.Pixels);
  uint16_t * yp = p;
  while(Ysize--)
  {
    p = yp;
    for(uint16_t x = 0; x < Xsize; x++)
    {
      *p = *pData;
      p += XSTEP;
      pData += 2;
    }
    yp += YSTEP;
  }
}

//-----------------------------------------------------------------------------
/**
  * @brief  Read 16bit/pixel picture from Lcd and store to RAM
  * @param  pdata: picture address.
  * @param  Xpos:  Image X position in the LCD
  * @param  Ypos:  Image Y position in the LCD
  * @param  Xsize: Image X size in the LCD
  * @param  Ysize: Image Y size in the LCD
  * @retval None
  * @brief  Draw direction: right then down
  */
void ili9341_ReadRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData)
{
  uint16_t * p = ((YPOS * FrameBuffer.Xsize + XPOS) + FrameBuffer.Pixels);
  uint16_t * yp = p;
  while(Ysize--)
  {
    p = yp;
    for(uint16_t x = 0; x < Xsize; x++)
    {
      *pData = *p;
      p += XSTEP;
      pData += 2;
    }
    yp += YSTEP;
  }
}

//-----------------------------------------------------------------------------
/**
  * @brief  Set display scroll parameters
  * @param  Scroll    : Scroll size [pixel]
  * @param  TopFix    : Top fix size [pixel]
  * @param  BottonFix : Botton fix size [pixel]
  * @retval None
  */
void ili9341_Scroll(int16_t Scroll, uint16_t TopFix, uint16_t BottonFix)
{
}

#endif /* #if ILI9341_INTERFACE_MODE == 2 */

//=============================================================================
#if ILI9341_TOUCH == 1

#include "ts.h"

#define TS_MULTITASK_MUTEX    ILI9341_MULTITASK_MUTEX
#define TOUCH_FILTER          16
#define TOUCH_MAXREPEAT       8

#define ABS(N)   (((N)<0) ? (-(N)) : (N))

void     ili9341_ts_Init(uint16_t DeviceAddr);
uint8_t  ili9341_ts_DetectTouch(uint16_t DeviceAddr);
void     ili9341_ts_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y);

TS_DrvTypeDef   ili9341_ts_drv =
{
  ili9341_ts_Init,
  0,
  0,
  0,
  ili9341_ts_DetectTouch,
  ili9341_ts_GetXY,
  0,
  0,
  0,
  0,
};

TS_DrvTypeDef  *ts_drv = &ili9341_ts_drv;

#if (ILI9341_ORIENTATION == 0)
int32_t  ts_cindex[] = TS_CINDEX_0;
#elif (ILI9341_ORIENTATION == 1)
int32_t  ts_cindex[] = TS_CINDEX_1;
#elif (ILI9341_ORIENTATION == 2)
int32_t  ts_cindex[] = TS_CINDEX_2;
#elif (ILI9341_ORIENTATION == 3)
int32_t  ts_cindex[] = TS_CINDEX_3;
#endif

uint16_t tx, ty;

/* Link function for Touchscreen */
uint8_t  TS_IO_DetectToch(void);
uint16_t TS_IO_GetX(void);
uint16_t TS_IO_GetY(void);
uint16_t TS_IO_GetZ1(void);
uint16_t TS_IO_GetZ2(void);

//-----------------------------------------------------------------------------
void ili9341_ts_Init(uint16_t DeviceAddr)
{
  if((Is_ili9341_Initialized & ILI9341_IO_INITIALIZED) == 0)
    LCD_IO_Init();
  Is_ili9341_Initialized |= ILI9341_IO_INITIALIZED;
}

//-----------------------------------------------------------------------------
uint8_t ili9341_ts_DetectTouch(uint16_t DeviceAddr)
{
  static uint8_t ret = 0;
  int32_t x1, x2, y1, y2, i;

  #if TS_MULTITASK_MUTEX == 1
  io_ts_busy = 1;

  if(io_lcd_busy)
  {
    io_ts_busy = 0;
    return ret;
  }
  #endif

  ret = 0;
  if(TS_IO_DetectToch())
  {
    x1 = TS_IO_GetX();
    y1 = TS_IO_GetY();
    i = TOUCH_MAXREPEAT;
    while(i--)
    {
      x2 = TS_IO_GetX();
      y2 = TS_IO_GetY();
      if((ABS(x1 - x2) < TOUCH_FILTER) && (ABS(y1 - y2) < TOUCH_FILTER))
      {
        x1 = (x1 + x2) >> 1;
        y1 = (y1 + y2) >> 1;
        i = 0;
        if(TS_IO_DetectToch())
        {
          tx = x1;
          ty = y1;
          ret = 1;
        }
      }
      else
      {
        x1 = x2;
        y1 = y2;
      }
    }
  }

  #if TS_MULTITASK_MUTEX == 1
  io_ts_busy = 0;
  #endif

  return ret;
}

//-----------------------------------------------------------------------------
void ili9341_ts_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
  *X = tx,
  *Y = ty;
}
#endif // #if ILI9341_TOUCH == 1

void setCursor(int16_t x, int16_t y)
{ cursor_x = x; cursor_y = y; }

void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
	{ // Custom font

        // Character is assumed previously filtered by write() to eliminate
        // newlines, returns, non-printable characters, etc.  Calling
        // drawChar() directly with 'bad' characters of font may cause mayhem!

        c -= (uint8_t)pgm_read_byte(&gfxFont->first);
        GFXglyph *glyph  = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
        uint8_t  *bitmap = (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);

        uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
        uint8_t  w  = pgm_read_byte(&glyph->width),
                 h  = pgm_read_byte(&glyph->height);
        int8_t   xo = pgm_read_byte(&glyph->xOffset),
                 yo = pgm_read_byte(&glyph->yOffset);
        uint8_t  xx, yy, bits = 0, bit = 0;
        int16_t  xo16 = 0, yo16 = 0;

        if(size > 1) {
            xo16 = xo;
            yo16 = yo;
        }

        for(yy=0; yy<h; yy++) {
            for(xx=0; xx<w; xx++) {
                if(!(bit++ & 7)) {
                    bits = pgm_read_byte(&bitmap[bo++]);
                }
                if(bits & 0x80) {
                    if(size == 1) {
                        ili9341_WritePixel(x+xo+xx, y+yo+yy, color);
                    } else {
                    	ili9341_FillRect(x+(xo16+xx)*size, y+(yo16+yy)*size,
                          size, size, color);
                    }
                }
                bits <<= 1;
            }
        }

    } // End classic vs custom font
}

/**************************************************************************/

/*!
    @brief  Print one byte/character of data, used to support print()
    @param  c  The 8-bit ascii character to write
*/
/**************************************************************************/
size_t write(uint8_t c)
{
	{

        if(c == '\n') {
            cursor_x  = 0;
            cursor_y += (int16_t)textsize *
                        (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        } else if(c != '\r') {
            uint8_t first = pgm_read_byte(&gfxFont->first);
            if((c >= first) && (c <= (uint8_t)pgm_read_byte(&gfxFont->last))) {
                GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(
                  &gfxFont->glyph))[c - first]);
                uint8_t   w     = pgm_read_byte(&glyph->width),
                          h     = pgm_read_byte(&glyph->height);
                if((w > 0) && (h > 0)) { // Is there an associated bitmap?
                    int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset); // sic
                    if(wrap && ((cursor_x + textsize * (xo + w)) > ILI9341_LCD_PIXEL_WIDTH)) {
                        cursor_x  = 0;
                        cursor_y += (int16_t)textsize *
                          (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
                    }
                    drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
                }
                cursor_x += (uint8_t)pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize;
            }
        }

    }
    return 1;
}

/**************************************************************************/
/*!
    @brief Set the font to display when print()ing, either custom or default
    @param  f  The GFXfont object, if NULL use built in 6x8 font
*/
/**************************************************************************/
void setFont(const GFXfont *f) {
    if(f) {            // Font struct pointer passed in?
        if(!gfxFont) { // And no current font struct?
            // Switching from classic to new font behavior.
            // Move cursor pos down 6 pixels so it's on baseline.
            cursor_y += 6;
        }
    } else if(gfxFont) { // NULL passed.  Current font struct defined?
        // Switching from new to classic font behavior.
        // Move cursor pos up 6 pixels so it's at top-left of char.
        cursor_y -= 6;
    }
    gfxFont = (GFXfont *)f;
}

void ili9341_tftWriteText (int x, int y, uint16_t txtcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str)
{
	setFont(f);
	textcolor = txtcolor;
	textsize = (txtsize > 0) ? txtsize : 1;
	setCursor(x, y);
	//while (*str) write (*str++);
	while (*str) write(*str++);
}

/**************************************************************************/
/*!
    @brief Print a new string on the TFT LCD
    @param  row	The vertical pixel of botton left (BL) cursor
    @param  txtcolor	The color of the text
    @param  txtcolor	The color of the text
    @param	txtsize	The size of de text
    @param	str	A point to the string
*/
/**************************************************************************/
void ili9341_Printnewtstr (int row, uint16_t txtcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str)
{
	setFont(f);
	textcolor = txtcolor;
	textsize = (txtsize > 0) ? txtsize : 1;
	setCursor(0, row);
	//while (*str) write (*str++);
	while (*str) write(*str++);
}
void ili9341_WriteLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
        uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
        	ili9341_WritePixel(y0, x0, color);
        } else {
        	ili9341_WritePixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

void ili9341_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    if(x0 == x1){
        if(y0 > y1) _swap_int16_t(y0, y1);
        ili9341_DrawFastVLine(x0, y0, y1 - y0 + 1, color);
    } else if(y0 == y1){
        if(x0 > x1) _swap_int16_t(x0, x1);
        ili9341_DrawFastHLine(x0, y0, x1 - x0 + 1, color);
    } else {
    	ili9341_WriteLine(x0, y0, x1, y1, color);
    }
}

void ili9341_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ili9341_WritePixel(x0  , y0+r, color);
    ili9341_WritePixel(x0  , y0-r, color);
    ili9341_WritePixel(x0+r, y0  , color);
    ili9341_WritePixel(x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ili9341_WritePixel(x0 + x, y0 + y, color);
        ili9341_WritePixel(x0 - x, y0 + y, color);
        ili9341_WritePixel(x0 + x, y0 - y, color);
        ili9341_WritePixel(x0 - x, y0 - y, color);
        ili9341_WritePixel(x0 + y, y0 + x, color);
        ili9341_WritePixel(x0 - y, y0 + x, color);
        ili9341_WritePixel(x0 + y, y0 - x, color);
        ili9341_WritePixel(x0 - y, y0 - x, color);
    }
}


void ili9341_DrawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
            ili9341_WritePixel(x0 + x, y0 + y, color);
            ili9341_WritePixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            ili9341_WritePixel(x0 + x, y0 - y, color);
            ili9341_WritePixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            ili9341_WritePixel(x0 - y, y0 + x, color);
            ili9341_WritePixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            ili9341_WritePixel(x0 - y, y0 - x, color);
            ili9341_WritePixel(x0 - x, y0 - y, color);
        }
    }
}

void ili9341_DrawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    ili9341_DrawFastHLine(x+r  , y    , w-2*r, color); // Top
    ili9341_DrawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
    ili9341_DrawFastVLine(x    , y+r  , h-2*r, color); // Left
    ili9341_DrawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
    // draw four corners
    ili9341_DrawCircleHelper(x+r    , y+r    , r, 1, color);
    ili9341_DrawCircleHelper(x+w-r-1, y+r    , r, 2, color);
    ili9341_DrawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
    ili9341_DrawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}


void ili9341_FillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    ili9341_FillRect(x+r, y, w-2*r, h, color);
    // draw four corners
    ili9341_FillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
    ili9341_FillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

void ili9341_DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	ili9341_DrawLine(x0, y0, x1, y1, color);
	ili9341_DrawLine(x1, y1, x2, y2, color);
	ili9341_DrawLine(x2, y2, x0, y0, color);
}

void ili9341_FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    int16_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }
    if (y1 > y2) {
        _swap_int16_t(y2, y1); _swap_int16_t(x2, x1);
    }
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }

    if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if(x1 < a)      a = x1;
        else if(x1 > b) b = x1;
        if(x2 < a)      a = x2;
        else if(x2 > b) b = x2;
        ili9341_DrawFastHLine(a, y0, b-a+1, color);
        return;
    }

    int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
    int32_t
    sa   = 0,
    sb   = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if(y1 == y2) last = y1;   // Include y1 scanline
    else         last = y1-1; // Skip it

    for(y=y0; y<=last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        ili9341_DrawFastHLine(a, y, b-a+1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for(; y<=y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
        a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        ili9341_DrawFastHLine(a, y, b-a+1, color);
    }
}

/****************** Integração com câmera ***********************/
//Testado com LCF TFT ILI9340
void desenhaPixel(uint16_t pixel)
{
	LCD_IO_WriteData16(pixel);
}


void inicioDados(void)
{
	//CS_ACTIVE;
	LCD_IO_WriteCmd8(0x2C);
}


void fimDados(void)
{
	//CS_IDLE;
}

void LCD_TxBMP(unsigned char *data, unsigned int BitmapStart)
{
    static unsigned int tamanho = 0;
    static unsigned char sobrou[3] = {0, 0, 0};
    unsigned int i = 0;
    unsigned int setor = 512;
    unsigned short int cor;
    unsigned char tam[11];
    // Leandro (22/09/2020) - Ajuste para ler imagens com dimensões maiores que um byte (320x240)
    //  unsigned char altura, x = 0, y = 127, bits_por_pixel;
    unsigned char bits_por_pixel;
    unsigned int altura, x = 0, y = 239; // Formato "paisagem"
    //unsigned int altura, x = 0, y = 319; //Formato "retrato"
    const unsigned int lim_altura = 240, lim_largura = 320; // Formato "paisagem" - enviar comando setRotation(1);
    //const unsigned int lim_altura = 320, lim_largura = 240; //Formato "retrato" - enviar comando setRotation(0);
    // static unsigned char largura = 0, erro_bits = 0, bytes_extras = 0, pixels_por_linha = 0;
    static unsigned int largura = 0, pixels_por_linha = 0;
    static unsigned char erro_bits = 0, bytes_extras = 0;
    // Se é o primeiro setor do arquivo, possui o cabeçalho
    if (BitmapStart)
    {
        BitmapStart = 0;
        // Reseta variável estática
        pixels_por_linha = 0;
        // Pula o cabeçalho
        i = 54;
        // Lê o tamanho da área de dados do arquivo em bytes
        tamanho = data[0x22] + (unsigned int)(data[0x23] << 8) + (unsigned int)(data[0x24] << 16) + (unsigned int)(data[0x25] << 24);
        // Leandro (22/09/2020) - Ajuste para ler imagens com dimensões maiores que um byte (320x240)
        //  //Leandro (01/09/2019) - Lê a largura e a altura da imagem para definir o tamanho da janela
        //  largura = data[18];
        //  altura = data[22];
        largura = data[18] + (unsigned int)(data[19] << 8);
        altura = data[22] + (unsigned int)(data[23] << 8);
        // Configura a janela
        //setAddrWindow(x, y - altura + 1, x + largura - 1, y);
        ili9341_SetDisplayWindow(x, y - altura + 1, x + largura - 1, y);
        // Envia para o LCD sinalização de início de envio de dados
        inicioDados();
        // Verifica se existirão bytes extras no arquivo em função da largura da imagem
        // Obervação: Existe uma restrição de que cada linha deva ter N bytes, sendo N um número
        // divisível por 4. Caso contrário, o BMP deve ser preenchido com bytes não válidos. Por
        // exemplo, se a imagem tem 1 x 100 pixels em 24 bits/pixel, o BMP teria 3 bytes válidos em
        // cada linha e mais 1 byte que não tem qualquer significado.
        switch ((largura * 3) % 4)
        {
        case 1:
            bytes_extras = 3;
            break;
        case 2:
            bytes_extras = 2;
            break;
        case 3:
            bytes_extras = 1;
            break;
        default:
            bytes_extras = 0;
            break;
        }
        // Lê a quantidade de bits por pixel (neste caso é aceito apenas 24 bits por pixel)
        bits_por_pixel = data[28];
        // Testa a quatidade de bits
        // Leandro (22/09/2020) - Ajuste para ler imagens com dimensões maiores que um byte (320x240)
        //  if((bits_por_pixel != 24) || (largura > 128) || (altura > 128))
        if ((bits_por_pixel != 24) || (largura > lim_largura) || (altura > lim_altura))
            erro_bits = 1;
        else
            erro_bits = 0;
    }
    // Se houver erro na quantidade de bits retorna e não envia para o LCD
    if (erro_bits)
    {
        return;
    }
    // Envia os pixels enquanto não acabar o setor ou o Bitmap
    while ((i <= (512 - 3)) && (tamanho >= 3)) // 24 bits por pixels
    {
        // Se completou uma linha
        if (pixels_por_linha == largura)
        {
            // Zera o contador
            pixels_por_linha = 0;
            // Verifica se tem bytes nulos para ignorar
            if (bytes_extras >= sobrou[0])
            {
                // Desconta os bytes_extras-sobrou[0] do tamanho do setor
                tamanho -= (bytes_extras - sobrou[0]);
                // Incrementa a posição do byte a ser lido do setor
                i += (bytes_extras - sobrou[0]);
                // Atualiza o valor da sobra
                sobrou[0] = 0;
                // Verifica se não cabe mais nenhum pixel, encerra o loop
                if ((i > (512 - 3)) || (tamanho < 3))
                    break;
            }
            else
            {
                // Atualiza o valor da sobra
                sobrou[0] -= bytes_extras;
            }
            // break;
            if (tamanho < 3)
                break;
        }
        if (sobrou[0] == 0) // Tamanho -= 3
        {
            //((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            // Seguencia BGR (24 bits) --> RGB (565)
            cor = (data[i] >> 3) | ((data[i + 1] & 0xFC) << 3) | ((data[i + 2] & 0xF8) << 8);
            i += 3;
            tamanho -= 3;
        }
        else if (sobrou[0] == 1) // Tamanho -= 2
        {
            // Sobrou a cor Azul
            cor = (sobrou[2] >> 3) | ((data[i] & 0xFC) << 3) | ((data[i + 1] & 0xF8) << 8);
            i += 2;
            tamanho -= 2;
        }
        else if (sobrou[0] == 2) // Tamanho -= 1
        {
            // Sobrou a cor Azul e Verde
            cor = (sobrou[1] >> 3) | ((sobrou[2] & 0xFC) << 3) | ((data[i] & 0xF8) << 8);
            i += 1;
            tamanho -= 1;
        }
        else
        {
            i = 512;
            setor = 0;
            tamanho = 0;
            break;
        }
        // Envia pixel 565 para o LCD
        desenhaPixel(cor);
        sobrou[0] = 0; // Sobra algum byte apenas no final do setor (i>= 510)
        // Incrementa o número de pixels enviados por linha e testa
        pixels_por_linha++;
    }
    // Se ainda não acabou o arquivo
    if (tamanho >= 3)
    {
        // Salva o número de bytes que sobraram para formar um pixel
        sobrou[0] = 512 - i;
        // Completa os 512 bytes do setor
        tamanho -= sobrou[0];
        // Salva o penúltimo byte
        sobrou[1] = data[510];
        // Salva o último byte
        sobrou[2] = data[511];
    }
    else
    {
        // Envia para o LCD sinalização de fim de envio de dados
        fimDados();
    }
}

