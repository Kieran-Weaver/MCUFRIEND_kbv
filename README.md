#MCUFRIEND_kbv 

Library for ST7735 only

MCUFRIEND_kbv inherits all the methods from 
the Adafruit_GFX class: https://learn.adafruit.com/adafruit-gfx-graphics-library/overview 
and Print class: https://www.arduino.cc/en/Serial/Print

The only "new" methods are hardware related: 
vertScroll(), readGRAM(), readPixel(), setAddrWindow(), pushColors(), readID(), begin()

readReg(), pushCommand() access the controller registers
