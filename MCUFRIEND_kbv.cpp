#define SUPPORT_7735

#include "MCUFRIEND_kbv.h"
#include "def_serial.h"

#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)

#define USING_16BIT_BUS 0

MCUFRIEND_kbv::MCUFRIEND_kbv(int CS, int RS, int WR, int RD, int _RST):Adafruit_GFX(240, 320)
{
    // we can not access GPIO pins until AHB has been enabled.
}

static uint8_t done_reset, is8347, is555, is9797;
static uint16_t color565_to_555(uint16_t color) {
    return (color & 0xFFC0) | ((color & 0x1F) << 1) | ((color & 0x01));  //lose Green LSB, extend Blue LSB
}
static uint16_t color555_to_565(uint16_t color) {
    return (color & 0xFFC0) | ((color & 0x0400) >> 5) | ((color & 0x3F) >> 1); //extend Green LSB
}
static uint8_t color565_to_r(uint16_t color) {
    return ((color & 0xF800) >> 8);  // transform to rrrrrxxx
}
static uint8_t color565_to_g(uint16_t color) {
    return ((color & 0x07E0) >> 3);  // transform to ggggggxx
}
static uint8_t color565_to_b(uint16_t color) {
    return ((color & 0x001F) << 3);  // transform to bbbbbxxx
}
static void write24(uint16_t color) {
    uint8_t r = color565_to_r(color);
    uint8_t g = color565_to_g(color);
    uint8_t b = color565_to_b(color);
    write8(r);
    write8(g);
    write8(b);
}

void MCUFRIEND_kbv::reset(void)
{
    done_reset = 1;
    setWriteDir();
    CTL_INIT();
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
    RESET_IDLE;
    delay(50);
    RESET_ACTIVE;
    delay(100);
    RESET_IDLE;
    delay(100);
	WriteCmdData(0xB0, 0x0000);   //R61520 needs this to read ID
}

static void writecmddata(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}

void MCUFRIEND_kbv::WriteCmdData(uint16_t cmd, uint16_t dat) { writecmddata(cmd, dat); }

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    while (N-- > 0) {
        uint8_t u8 = *block++;
        write8(u8);
        if (N && is8347) {
            cmd++;
            WriteCmd(cmd);
        }
    }
    CS_IDLE;
}

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
    uint8_t d[4];
    d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
    WriteCmdParamN(cmd, 4, d);
}

//#define WriteCmdParam4(cmd, d1, d2, d3, d4) {uint8_t d[4];d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;WriteCmdParamN(cmd, 4, d);}
void MCUFRIEND_kbv::pushCommand(uint16_t cmd, uint8_t * block, int8_t N) { WriteCmdParamN(cmd, N, block); }

static uint16_t read16bits(void)
{
    uint16_t ret;
    uint8_t lo;
    delay(1);    //1us should be adequate
    READ_8(ret);
    //all MIPI_DCS_REV1 style params are 8-bit
    delay(1);    //1us should be adequate
    READ_8(lo);
    return (ret << 8) | lo;
}

uint16_t MCUFRIEND_kbv::readReg(uint16_t reg, int8_t index)
{
    uint16_t ret;
    uint8_t lo;
    if (!done_reset)
        reset();
    CS_ACTIVE;
    WriteCmd(reg);
    setReadDir();
    delay(1);    //1us should be adequate
    //    READ_16(ret);
    do { ret = read16bits(); }while (--index >= 0);  //need to test with SSD1963
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ret;
}

uint32_t MCUFRIEND_kbv::readReg32(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t l = readReg(reg, 1);
    return ((uint32_t) h << 16) | (l);
}

uint32_t MCUFRIEND_kbv::readReg40(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t m = readReg(reg, 1);
    uint16_t l = readReg(reg, 2);
    return ((uint32_t) h << 24) | (m << 8) | (l >> 8);
}

uint16_t MCUFRIEND_kbv::readID(void)
{
    return 0x7735;
}

 // independent cursor and window registers.   S6D0154, ST7781 increments.  ILI92320/5 do not.  
int16_t MCUFRIEND_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    uint16_t ret, dummy, _MR = _MW;
    int16_t n = w * h, row = 0, col = 0;
    uint8_t r, g, b, tmp;
    _MR = 0x2E;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    while (n > 0) {
        CS_ACTIVE;
        WriteCmd(_MR);
        setReadDir();
        READ_8(r);
        while (n) {
            READ_8(r);
            READ_8(g);
            READ_8(b);
            ret = color565(r, g, b);
            *block++ = ret;
            n--;
        }
        if (++col >= w) {
            col = 0;
            if (++row >= h)
                row = 0;
        }
        RD_IDLE;
        CS_IDLE;
        setWriteDir();
    }
    return 0;
}

void MCUFRIEND_kbv::setRotation(uint8_t r)
{
    uint16_t GS, SS_v, ORG, REV = _lcd_rev;
    uint8_t val, d[3];
    rotation = r & 3;           // just perform the operation ourselves on the protected variables
    _width = (rotation & 1) ? HEIGHT : WIDTH;
    _height = (rotation & 1) ? WIDTH : HEIGHT;
    switch (rotation) {
    case 0:                    //PORTRAIT:
        val = 0x48;             //MY=0, MX=1, MV=0, ML=0, BGR=1
        break;
    case 1:                    //LANDSCAPE: 90 degrees
        val = 0x28;             //MY=0, MX=0, MV=1, ML=0, BGR=1
        break;
    case 2:                    //PORTRAIT_REV: 180 degrees
        val = 0x98;             //MY=1, MX=0, MV=0, ML=1, BGR=1
        break;
    case 3:                    //LANDSCAPE_REV: 270 degrees
        val = 0xF8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
        break;
    }
    _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
    WriteCmdParamN(is8347 ? 0x16 : 0x36, 1, &val);
    _lcd_madctl = val;
    setAddrWindow(0, 0, width() - 1, height() - 1);
    vertScroll(0, HEIGHT, 0);   //reset scrolling after a rotation
}

void MCUFRIEND_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // MCUFRIEND just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
    setAddrWindow(x, y, x, y);
    WriteCmdData(_MW, color);
}

void MCUFRIEND_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
    WriteCmdParam4(_SC, x >> 8, x, x1 >> 8, x1);   //Start column instead of _MC
    WriteCmdParam4(_SP, y >> 8, y, y1 >> 8, y1);   //
}

void MCUFRIEND_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
    if (w < 0) {
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(_MW);
    if (h > w) {
        end = h;
        h = w;
        w = end;
    }
    uint8_t hi = color >> 8, lo = color & 0xFF;
    while (h-- > 0) {
        end = w;
        do {
            write8(hi);
            write8(lo);
        } while (--end != 0);
    }
    CS_IDLE;
}

static void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, bool first, uint8_t flags)
{
    uint16_t color;
    uint8_t h, l;
	bool isconst = flags & 1;
	bool isbigend = (flags & 2) != 0;
    CS_ACTIVE;
    if (first) {
        WriteCmd(cmd);
    }

    if (!isconst && !isbigend) {
        uint16_t *block16 = (uint16_t*)block;
        while (n-- > 0) {
            color = *block16++;
            write16(color);
        }
    } else

    while (n-- > 0) {
        if (isconst) {
            h = pgm_read_byte(block++);
            l = pgm_read_byte(block++);
        } else {
		    h = (*block++);
            l = (*block++);
		}
        color = (isbigend) ? (h << 8 | l) :  (l << 8 | h);
        write16(color);
    }
    CS_IDLE;
}

void MCUFRIEND_kbv::pushColors(uint16_t * block, int16_t n, bool first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 0);
}
void MCUFRIEND_kbv::pushColors(uint8_t * block, int16_t n, bool first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 2);   //regular bigend
}
void MCUFRIEND_kbv::pushColors(const uint8_t * block, int16_t n, bool first, bool bigend)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, bigend ? 3 : 1);
}

void MCUFRIEND_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    int16_t sea = top;
    if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
	vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    sea = top + scrollines - 1;
    uint8_t d[6];           // for multi-byte parameters
    d[0] = top >> 8;        //TFA
    d[1] = top;
    d[2] = scrollines >> 8; //VSA
    d[3] = scrollines;
    d[4] = bfa >> 8;        //BFA
    d[5] = bfa;
    WriteCmdParamN(0x33, 6, d);
	d[0] = vsp >> 8;        //VSP
	d[1] = vsp;
    WriteCmdParamN(0x37, 2, d);
    if (offset == 0) 
	WriteCmdParamN(0x13, 0, NULL);    //NORMAL i.e. disable scroll
}

void MCUFRIEND_kbv::invertDisplay(bool i)
{
    WriteCmdParamN(i ? 0x21 : 0x20, 0, NULL);
}

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0x7F
static void init_table(const void *table, int16_t size)
{
    //copes with any uint8_t table.  Even HX8347 style
    uint8_t *p = (uint8_t *) table;
    while (size > 0) {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8) {
            delay(len);
            len = 0;
        } else {
            CS_ACTIVE;
            CD_COMMAND;
            write8(cmd);
            for (uint8_t d = 0; d++ < len; ) {
                uint8_t x = pgm_read_byte(p++);
                CD_DATA;
                write8(x);
                if (is8347 && d < len) {
                    CD_COMMAND;
                    cmd++;
                    write8(cmd);
                }
            }
            CS_IDLE;
        }
        size -= len + 2;
    }
}

static void init_table16(const void *table, int16_t size)
{
    uint16_t *p = (uint16_t *) table;
    while (size > 0) {
        uint16_t cmd = pgm_read_word(p++);
        uint16_t d = pgm_read_word(p++);
        if (cmd == TFTLCD_DELAY)
            delay(d);
        else {
			writecmddata(cmd, d);                      //static function
        }
        size -= 2 * sizeof(int16_t);
    }
}

void MCUFRIEND_kbv::begin(uint16_t ID)
{
    int16_t *p16;               //so we can "write" to a const protected variable.
    const uint8_t *table8_ads = NULL;
    int16_t table_size;
    reset();
    _lcd_xor = 0;
    _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | REV_SCREEN | READ_24BITS;
    static const uint8_t PROGMEM table7735S[] = {
	    //  (COMMAND_BYTE), n, data_bytes....
	    0xB1, 3, 0x01, 0x2C, 0x2D,  // [05 3C 3C] FRMCTR1 if GM==11
	    0xB2, 3, 0x01, 0x2C, 0x2D,  // [05 3C 3C]
	    0xB3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D, // [05 3C 3C 05 3C 3C]
	    0xB4, 1, 0x07,              // [07] INVCTR Column inversion
	    //ST7735XR Power Sequence
	    0xC0, 3, 0xA2, 0x02, 0x84,  // [A8 08 84] PWCTR1
	    0xC1, 1, 0xC5,              // [C0]
	    0xC2, 2, 0x0A, 0x00,        // [0A 00]
	    0xC3, 2, 0x8A, 0x2A,        // [8A 26]
	    0xC4, 2, 0x8A, 0xEE,        // [8A EE]
	    0xC5, 1, 0x0E,              // [05] VMCTR1 VCOM
    };
    table8_ads = table7735S, table_size = sizeof(table7735S);
    p16 = (int16_t *) & HEIGHT;
    *p16 = 160;
    p16 = (int16_t *) & WIDTH;
    *p16 = 128;
}
