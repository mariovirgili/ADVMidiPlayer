#pragma once

#ifndef USER_SETUP_LOADED
#define USER_SETUP_LOADED

// External TFT setup derived from the Cardputer dual-display reference project.
// This display is optional and only initialized when the user selects Dual Display.

#define ILI9341_DRIVER
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

#define SPI_FREQUENCY       70000000
#define SPI_READ_FREQUENCY   6000000

// Keep the external TFT on a dedicated SPI host, matching the reference wiring.
#define USE_HSPI_PORT

#define TFT_CS     5
#define TFT_DC     6
#define TFT_RST    3

#define TFT_SCLK  15
#define TFT_MOSI  13
#define TFT_MISO  -1

#define TOUCH_CS  -1

#define TFT_RGB_ORDER TFT_BGR
#define TFT_INVERSION_OFF

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_GFXFF
#define SMOOTH_FONT

#endif
