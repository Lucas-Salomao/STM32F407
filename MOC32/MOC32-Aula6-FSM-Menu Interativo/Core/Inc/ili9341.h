//List of colors
#define COLOR_BLACK           0x0000
#define COLOR_NAVY            0x000F
#define COLOR_DGREEN          0x03E0
#define COLOR_DCYAN           0x03EF
#define COLOR_MAROON          0x7800
#define COLOR_PURPLE          0x780F
#define COLOR_OLIVE           0x7BE0
#define COLOR_LGRAY           0xC618
#define COLOR_DGRAY           0x7BEF
#define COLOR_BLUE            0x001F
#define COLOR_BLUE2			      0x051D
#define COLOR_GREEN           0x07E0
#define COLOR_GREEN2		      0xB723
#define COLOR_GREEN3		      0x8000
#define COLOR_CYAN            0x07FF
#define COLOR_RED             0xF800
#define COLOR_MAGENTA         0xF81F
#define COLOR_YELLOW          0xFFE0
#define COLOR_WHITE           0xFFFF
#define COLOR_ORANGE          0xFD20
#define COLOR_GREENYELLOW     0xAFE5
#define COLOR_BROWN 			    0XBC40

/* Interface mode
   - 1: SPI or paralell interface mode
   - 2: RGB mode (LTDC hardware, HSYNC, VSYNC, pixel clock, RGB bits data, framebuffer memory)
*/
#define  ILI9341_INTERFACE_MODE   1

/* Orientation
   - 0: 240x320 portrait (plug in top)
   - 1: 320x240 landscape (plug in left)
   - 2: 240x320 portrait (plug in botton)
   - 3: 320x240 landscape (plug in right)
*/
#define  ILI9341_ORIENTATION      0

/* Color mode
   - 0: RGB565 (R:bit15..11, G:bit10..5, B:bit4..0)
   - 1: BRG565 (B:bit15..11, G:bit10..5, R:bit4..0)
*/
#define  ILI9341_COLORMODE        0

/* To clear the screen before display turning on ?
   - 0: does not clear
   - 1: clear
*/
#define  ILI9341_INITCLEAR        1

/* Analog touchscreen (only INTERFACE_MODE == 1, 8bit paralell IO mode)
   - 0: touchscreen disabled
   - 1: touchscreen enabled
*/
#define  ILI9341_TOUCH            0

/* Touchscreen calibration data for 4 orientations */
#define  TS_CINDEX_0        {1444723, 5348, -114234, 421806850, -131233, -975, 521525308}
#define  TS_CINDEX_1        {1444723, -131233, -975, 521525308, -5348, 114234, -76518053}
#define  TS_CINDEX_2        {1444723, -5348, 114234, -76518053, 131233, 975, -60658671}
#define  TS_CINDEX_3        {1444723, 131233, 975, -60658671, 5348, -114234, 421806850}

/* For multi-threaded or intermittent use, Lcd and Touchscreen simultaneous use can cause confusion (since it uses common I/O resources)
   Lcd functions wait for the touchscreen header, the touchscreen query is not executed when Lcd is busy.
   Note: If the priority of the Lcd is higher than that of the Touchscreen, it may end up in an infinite loop!
   - 0: multi-threaded protection disabled
   - 1: multi-threaded protection enabled
*/
#define  ILI9341_MULTITASK_MUTEX   0

#if  ILI9341_INTERFACE_MODE == 2

/* please see in the main.c what is the LTDC_HandleTypeDef name */
extern   LTDC_HandleTypeDef       hltdc;

/* Frambuffer memory alloc, free */
#define  ILI9341_MALLOC           malloc
#define  ILI9341_FREE             free

/* include for memory alloc/free */
#include <stdlib.h>

#endif  /* #if ILI9341_INTERFACE_MODE == 2 */

//-----------------------------------------------------------------------------
// ILI9341 physic resolution (in 0 orientation)
#define  ILI9341_LCD_PIXEL_WIDTH  240
#define  ILI9341_LCD_PIXEL_HEIGHT 320
