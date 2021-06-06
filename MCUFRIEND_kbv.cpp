//#define SUPPORT_0139              //S6D0139 +280 bytes
//#define SUPPORT_0154              //S6D0154 +320 bytes
//#define SUPPORT_1289              //SSD1289,SSD1297 (ID=0x9797) +626 bytes, 0.03s
//#define SUPPORT_1580              //R61580 Untested
//#define SUPPORT_1963              //only works with 16BIT bus anyway
//#define SUPPORT_4532              //LGDP4532 +120 bytes.  thanks Leodino
//#define SUPPORT_4535              //LGDP4535 +180 bytes
//#define SUPPORT_68140             //RM68140 +52 bytes defaults to PIXFMT=0x55
//#define SUPPORT_7735
//#define SUPPORT_7781              //ST7781 +172 bytes
//#define SUPPORT_8230              //UC8230 +118 bytes
//#define SUPPORT_8347D             //HX8347-D, HX8347-G, HX8347-I, HX8367-A +520 bytes, 0.27s
//#define SUPPORT_8347A             //HX8347-A +500 bytes, 0.27s
//#define SUPPORT_8352A             //HX8352A +486 bytes, 0.27s
//#define SUPPORT_8352B             //HX8352B
//#define SUPPORT_8357D_GAMMA       //monster 34 byte
//#define SUPPORT_9163              //
//#define SUPPORT_9225              //ILI9225-B, ILI9225-G ID=0x9225, ID=0x9226, ID=0x6813 +380 bytes
//#define SUPPORT_9326_5420         //ILI9326, SPFD5420 +246 bytes
//#define SUPPORT_9342              //costs +114 bytes
//#define SUPPORT_9806              //UNTESTED
//#define SUPPORT_9488_555          //costs +230 bytes, 0.03s / 0.19s
//#define SUPPORT_B509_7793         //R61509, ST7793 +244 bytes
// #define SUPPORT_OTHERS
// #define OFFSET_9327 32 //costs about 103 bytes, 0.08s

#include "MCUFRIEND_kbv.h"
#if defined(USE_SERIAL)
#include "utility/mcufriend_serial.h"
//uint8_t running;
#elif defined(__MBED__)
#include "utility/mcufriend_mbed.h"
#elif defined(__CC_ARM) || defined(__CROSSWORKS_ARM)
#include "utility/mcufriend_keil.h"
#else
#include "utility/mcufriend_shield.h"
#endif

#define MIPI_DCS_REV1 (1 << 0)
#define AUTO_READINC (1 << 1)
#define READ_24BITS (1 << 4)
#define MV_AXIS (1 << 10)

#if (defined(USES_16BIT_BUS)) //only comes from SPECIALs
#define USING_16BIT_BUS 1
#else
#define USING_16BIT_BUS 0
#endif

MCUFRIEND_kbv::MCUFRIEND_kbv(int CS, int RS, int WR, int RD, int _RST) : Adafruit_GFX(240, 320)
{
    // we can not access GPIO pins until AHB has been enabled.
}

static uint8_t done_reset;
static uint16_t color565_to_555(uint16_t color)
{
    return (color & 0xFFC0) | ((color & 0x1F) << 1) | ((color & 0x01)); //lose Green LSB, extend Blue LSB
}
static uint16_t color555_to_565(uint16_t color)
{
    return (color & 0xFFC0) | ((color & 0x0400) >> 5) | ((color & 0x3F) >> 1); //extend Green LSB
}
static uint8_t color565_to_r(uint16_t color)
{
    return ((color & 0xF800) >> 8); // transform to rrrrrxxx
}
static uint8_t color565_to_g(uint16_t color)
{
    return ((color & 0x07E0) >> 3); // transform to ggggggxx
}
static uint8_t color565_to_b(uint16_t color)
{
    return ((color & 0x001F) << 3); // transform to bbbbbxxx
}
static void write24(uint16_t color)
{
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
    WriteCmdData(0xB0, 0x0000); //R61520 needs this to read ID
}

static void writecmddata(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}

void MCUFRIEND_kbv::WriteCmdData(uint16_t cmd, uint16_t dat) { writecmddata(cmd, dat); }

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t *block)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    while (N-- > 0)
    {
        uint8_t u8 = *block++;
        write8(u8);
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
void MCUFRIEND_kbv::pushCommand(uint16_t cmd, uint8_t *block, int8_t N) { WriteCmdParamN(cmd, N, block); }

static uint16_t read16bits(void)
{
    uint16_t ret;
    uint8_t lo;
#if USING_16BIT_BUS
    READ_16(ret);  //single strobe to read whole bus
    if (ret > 255) //ID might say 0x00D3
        return ret;
#else
    delay(1); //1us should be adequate
    READ_8(ret);
#endif
    //all MIPI_DCS_REV1 style params are 8-bit
    delay(1); //1us should be adequate
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
    delay(1); //1us should be adequate
    //    READ_16(ret);
    do
    {
        ret = read16bits();
    } while (--index >= 0); //need to test with SSD1963
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ret;
}

uint32_t MCUFRIEND_kbv::readReg32(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t l = readReg(reg, 1);
    return ((uint32_t)h << 16) | (l);
}

uint32_t MCUFRIEND_kbv::readReg40(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t m = readReg(reg, 1);
    uint16_t l = readReg(reg, 2);
    return ((uint32_t)h << 24) | (m << 8) | (l >> 8);
}

uint16_t MCUFRIEND_kbv::readID(void)
{
    return 0x9341;
}

// independent cursor and window registers.   S6D0154, ST7781 increments.  ILI92320/5 do not.
int16_t MCUFRIEND_kbv::readGRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h)
{
    uint16_t ret, dummy, _MR = _MW;
    int16_t n = w * h, row = 0, col = 0;
    uint8_t r, g, b, tmp;
    _MR = 0x2E;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    while (n > 0)
    {
        CS_ACTIVE;
        WriteCmd(_MR);
        setReadDir();
        READ_8(r);
        while (n)
        {
            READ_8(r);
            READ_8(g);
            READ_8(b);
            ret = color565(r, g, b);
            *block++ = ret;
            n--;
        }
        if (++col >= w)
        {
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
    rotation = r & 3; // just perform the operation ourselves on the protected variables
    _width = (rotation & 1) ? HEIGHT : WIDTH;
    _height = (rotation & 1) ? WIDTH : HEIGHT;
    switch (rotation)
    {
    case 0:         //PORTRAIT:
        val = 0x48; //MY=0, MX=1, MV=0, ML=0, BGR=1
        break;
    case 1:         //LANDSCAPE: 90 degrees
        val = 0x28; //MY=0, MX=0, MV=1, ML=0, BGR=1
        break;
    case 2:         //PORTRAIT_REV: 180 degrees
        val = 0x98; //MY=1, MX=0, MV=0, ML=1, BGR=1
        break;
    case 3:         //LANDSCAPE_REV: 270 degrees
        val = 0xF8; //MY=1, MX=1, MV=1, ML=1, BGR=1
        break;
    }
    {
        _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
        WriteCmdParamN(0x36, 1, &val);
        _lcd_madctl = val;
        //	    if (_lcd_ID	== 0x1963) WriteCmdParamN(0x13, 0, NULL);   //NORMAL mode
    }
    setAddrWindow(0, 0, width() - 1, height() - 1);
    vertScroll(0, HEIGHT, 0); //reset scrolling after a rotation
}

void MCUFRIEND_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // MCUFRIEND just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
#if defined(SUPPORT_9488_555)
    if (is555)
        color = color565_to_555(color);
#endif
    setAddrWindow(x, y, x, y);
    //    CS_ACTIVE; WriteCmd(_MW); write16(color); CS_IDLE; //-0.01s +98B
    WriteCmdData(_MW, color);
}

void MCUFRIEND_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
#if 1
    if (_lcd_ID == 0x1526 && (rotation & 1))
    {
        int16_t dx = x1 - x, dy = y1 - y;
        if (dy == 0)
        {
            y1++;
        }
        else if (dx == 0)
        {
            x1 += dy;
            y1 -= dy;
        }
    }
#endif
    WriteCmdParam4(_SC, x >> 8, x, x1 >> 8, x1); //Start column instead of _MC
    WriteCmdParam4(_SP, y >> 8, y, y1 >> 8, y1); //
}

void MCUFRIEND_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
#if defined(SUPPORT_9488_555)
    if (is555)
        color = color565_to_555(color);
#endif
    if (w < 0)
    {
        w = -w;
        x -= w;
    } //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0)
    {
        h = -h;
        y -= h;
    } //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(_MW);
    if (h > w)
    {
        end = h;
        h = w;
        w = end;
    }
    uint8_t hi = color >> 8, lo = color & 0xFF;
    while (h-- > 0)
    {
        end = w;
#if USING_16BIT_BUS
#if defined(__MK66FX1M0__) //180MHz M4
#define STROBE_16BIT \
    {                \
        WR_ACTIVE4;  \
        WR_ACTIVE;   \
        WR_IDLE4;    \
        WR_IDLE;     \
    }                      //56ns
#elif defined(__SAM3X8E__) //84MHz M3
#define STROBE_16BIT \
    {                \
        WR_ACTIVE4;  \
        WR_ACTIVE2;  \
        WR_IDLE4;    \
        WR_IDLE2;    \
    } //286ns ?ILI9486
//#define STROBE_16BIT {WR_ACTIVE4;WR_ACTIVE;WR_IDLE4;WR_IDLE;} //238ns SSD1289
//#define STROBE_16BIT {WR_ACTIVE2;WR_ACTIVE;WR_IDLE2;}      //119ns RM68140
#else //16MHz AVR
#define STROBE_16BIT \
    {                \
        WR_ACTIVE;   \
        WR_ACTIVE;   \
        WR_IDLE;     \
    } //375ns ?ILI9486
#endif
        write_16(color); //we could just do the strobe
        lo = end & 7;
        hi = end >> 3;
        if (hi)
            do
            {
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
                STROBE_16BIT;
            } while (--hi > 0);
        while (lo-- > 0)
        {
            STROBE_16BIT;
        }
#else
#if defined(SUPPORT_1289)
        if (is9797)
        {
            uint8_t r = color565_to_r(color);
            uint8_t g = color565_to_g(color);
            uint8_t b = color565_to_b(color);
            do
            {
                write8(r);
                write8(g);
                write8(b);
            } while (--end != 0);
        }
        else
#endif
            do
            {
                write8(hi);
                write8(lo);
            } while (--end != 0);
#endif
    }
    CS_IDLE;
}

static void pushColors_any(uint16_t cmd, uint8_t *block, int16_t n, bool first, uint8_t flags)
{
    uint16_t color;
    uint8_t h, l;
    bool isconst = flags & 1;
    bool isbigend = (flags & 2) != 0;
    CS_ACTIVE;
    if (first)
    {
        WriteCmd(cmd);
    }

    if (!isconst && !isbigend)
    {
        uint16_t *block16 = (uint16_t *)block;
        while (n-- > 0)
        {
            color = *block16++;
            write16(color);
        }
    }
    else

        while (n-- > 0)
        {
            if (isconst)
            {
                h = pgm_read_byte(block++);
                l = pgm_read_byte(block++);
            }
            else
            {
                h = (*block++);
                l = (*block++);
            }
            color = (isbigend) ? (h << 8 | l) : (l << 8 | h);
#if defined(SUPPORT_9488_555)
            if (is555)
                color = color565_to_555(color);
#endif
            write16(color);
        }
    CS_IDLE;
}

void MCUFRIEND_kbv::pushColors(uint16_t *block, int16_t n, bool first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 0);
}
void MCUFRIEND_kbv::pushColors(uint8_t *block, int16_t n, bool first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 2); //regular bigend
}
void MCUFRIEND_kbv::pushColors(const uint8_t *block, int16_t n, bool first, bool bigend)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, bigend ? 3 : 1);
}

void MCUFRIEND_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
#if defined(OFFSET_9327)
    if (_lcd_ID == 0x9327)
    {
        if (rotation == 2 || rotation == 3)
            top += OFFSET_9327;
    }
#endif
    int16_t bfa = HEIGHT - top - scrollines; // bottom fixed area
    int16_t vsp;
    int16_t sea = top;
    if (_lcd_ID == 0x9327)
        bfa += 32;
    if (offset <= -scrollines || offset >= scrollines)
        offset = 0;     //valid scroll
    vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines; //keep in unsigned range
    sea = top + scrollines - 1;
    uint8_t d[6];    // for multi-byte parameters
                     /*
        if (_lcd_ID == 0x9327) {        //panel is wired for 240x432 
            if (rotation == 2 || rotation == 3) { //180 or 270 degrees
                if (scrollines == HEIGHT) {
                    scrollines = 432;   // we get a glitch but hey-ho
                    vsp -= 432 - HEIGHT;
                }
                if (vsp < 0)
                    vsp += 432;
            }
            bfa = 432 - top - scrollines;
        }
*/
    d[0] = top >> 8; //TFA
    d[1] = top;
    d[2] = scrollines >> 8; //VSA
    d[3] = scrollines;
    d[4] = bfa >> 8; //BFA
    d[5] = bfa;
    WriteCmdParamN(0x33, 6, d);
    //        if (offset == 0 && rotation > 1) vsp = top + scrollines;   //make non-valid
    d[0] = vsp >> 8; //VSP
    d[1] = vsp;
    WriteCmdParamN(0x37, 2, d);
    if (offset == 0)
    {
        WriteCmdParamN(0x13, 0, NULL); //NORMAL i.e. disable scroll
    }
}

void MCUFRIEND_kbv::invertDisplay(bool i)
{
    uint8_t val;
    _lcd_rev = (false) ^ i;
    WriteCmdParamN(_lcd_rev ? 0x21 : 0x20, 0, NULL);
    return;
}

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0x7F
static void init_table(const void *table, int16_t size)
{
    //copes with any uint8_t table.  Even HX8347 style
    uint8_t *p = (uint8_t *)table;
    while (size > 0)
    {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8)
        {
            delay(len);
            len = 0;
        }
        else
        {
            CS_ACTIVE;
            CD_COMMAND;
            write8(cmd);
            for (uint8_t d = 0; d++ < len;)
            {
                uint8_t x = pgm_read_byte(p++);
                CD_DATA;
                write8(x);
            }
            CS_IDLE;
        }
        size -= len + 2;
    }
}

static void init_table16(const void *table, int16_t size)
{
    uint16_t *p = (uint16_t *)table;
    while (size > 0)
    {
        uint16_t cmd = pgm_read_word(p++);
        uint16_t d = pgm_read_word(p++);
        if (cmd == TFTLCD_DELAY)
            delay(d);
        else
        {
            writecmddata(cmd, d); //static function
        }
        size -= 2 * sizeof(int16_t);
    }
}

void MCUFRIEND_kbv::begin(uint16_t ID)
{
    int16_t *p16; //so we can "write" to a const protected variable.
    const uint8_t *table8_ads = NULL;
    int16_t table_size;
    reset();
    _lcd_xor = 0;
    _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
    static const uint8_t ILI9341_regValues_2_4[] PROGMEM = {
        // BOE 2.4"
        0xF6,
        3,
        0x01,
        0x01,
        0x00, //Interface Control needs EXTC=1 MV_EOR=0, TM=0, RIM=0
        0xCF,
        3,
        0x00,
        0x81,
        0x30, //Power Control B [00 81 30]
        0xED,
        4,
        0x64,
        0x03,
        0x12,
        0x81, //Power On Seq [55 01 23 01]
        0xE8,
        3,
        0x85,
        0x10,
        0x78, //Driver Timing A [04 11 7A]
        0xCB,
        5,
        0x39,
        0x2C,
        0x00,
        0x34,
        0x02, //Power Control A [39 2C 00 34 02]
        0xF7,
        1,
        0x20, //Pump Ratio [10]
        0xEA,
        2,
        0x00,
        0x00, //Driver Timing B [66 00]
        0xB0,
        1,
        0x00, //RGB Signal [00]
        0xB1,
        2,
        0x00,
        0x1B, //Frame Control [00 1B]
        //            0xB6, 2, 0x0A, 0xA2, 0x27, //Display Function [0A 82 27 XX]    .kbv SS=1
        0xB4,
        1,
        0x00, //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
        0xC0,
        1,
        0x21, //Power Control 1 [26]
        0xC1,
        1,
        0x11, //Power Control 2 [00]
        0xC5,
        2,
        0x3F,
        0x3C, //VCOM 1 [31 3C]
        0xC7,
        1,
        0xB5, //VCOM 2 [C0]
        0x36,
        1,
        0x48, //Memory Access [00]
        0xF2,
        1,
        0x00, //Enable 3G [02]
        0x26,
        1,
        0x01, //Gamma Set [01]
        0xE0,
        15,
        0x0f,
        0x26,
        0x24,
        0x0b,
        0x0e,
        0x09,
        0x54,
        0xa8,
        0x46,
        0x0c,
        0x17,
        0x09,
        0x0f,
        0x07,
        0x00,
        0xE1,
        15,
        0x00,
        0x19,
        0x1b,
        0x04,
        0x10,
        0x07,
        0x2a,
        0x47,
        0x39,
        0x03,
        0x06,
        0x06,
        0x30,
        0x38,
        0x0f,
    };
    static const uint8_t ILI9341_regValues_ada[] PROGMEM = {
        // Adafruit_TFTLCD only works with EXTC=0
        //                     0xF6, 3, 0x00, 0x01, 0x00,  //Interface Control needs EXTC=1 TM=0, RIM=0
        //            0xF6, 3, 0x01, 0x01, 0x03,  //Interface Control needs EXTC=1 RM=1, RIM=1
        0xF6, 3, 0x09, 0x01, 0x03, //Interface Control needs EXTC=1 RM=0, RIM=1
        0xB0, 1, 0x40,             //RGB Signal [40] RCM=2
        0xB4, 1, 0x00,             //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
        0xC0, 1, 0x23,             //Power Control 1 [26]
        0xC1, 1, 0x10,             //Power Control 2 [00]
        0xC5, 2, 0x2B, 0x2B,       //VCOM 1 [31 3C]
        0xC7, 1, 0xC0,             //VCOM 2 [C0]
        0x36, 1, 0x88,             //Memory Access [00]
        0xB1, 2, 0x00, 0x1B,       //Frame Control [00 1B]
        0xB7, 1, 0x07,             //Entry Mode [00]
    };
    table8_ads = ILI9341_regValues_2_4, table_size = sizeof(ILI9341_regValues_2_4); //
    _lcd_rev = 1;
    if (table8_ads != NULL)
    {
        static const uint8_t reset_off[] PROGMEM = {
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 150, // .kbv will power up with ONLY reset, sleep out, display on
            0x28, 0,            //Display Off
            0x3A, 1, 0x55,      //Pixel read=565, write=565.
        };
        static const uint8_t wake_on[] PROGMEM = {
            0x11, 0, //Sleep Out
            TFTLCD_DELAY8, 150,
            0x29, 0, //Display On
        };
        init_table(&reset_off, sizeof(reset_off));
        init_table(table8_ads, table_size); //can change PIXFMT
        init_table(&wake_on, sizeof(wake_on));
    }
    setRotation(0); //PORTRAIT
    invertDisplay(false);
}
