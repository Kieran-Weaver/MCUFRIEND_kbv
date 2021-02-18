#include <SPI.h>

#define SPI_INIT()   { DDRB |= (1<<5)|(1<<3)|(1<<2); SPCR = (1<<SPE)|(1<<MSTR); SPSR = (1<<SPI2X); SPSR; SPDR; }
static inline uint8_t spi_xfer(uint8_t c)
{
    SPDR = c;
	while ((SPSR & (1<<SPIF)) == 0) ;
	return SPDR;
}
extern uint8_t running;
static inline void write8(uint8_t x)    {
                         if (running) {
							 while ((SPSR & 0x80) == 0);
							 SPDR;
						 }
                         SPDR = x;
                         running = 1;
                     }
static inline uint8_t read8(void)    {
                         if (running) while ((SPSR & 0x80) == 0);
                         running = 0;
						 return SPDR;
                     }
static inline uint8_t xchg8(uint8_t x) { write8(x); return read8(); }
static inline void flush(void)   {
                      if (running) {
                          while ((SPSR & 0x80) == 0);
                      }
                      running = 0;
    				  SPDR;
                  }

#define CD_PORT PORTB
#define CD_PIN  1
#define CS_PORT PORTB
#define CS_PIN  2
#define RESET_PORT PORTB
#define RESET_PIN  0
#define RD_IDLE
#define WR_IDLE

#define setWriteDir() { }
#define setReadDir()  { }
//#define write8(x)     spi_xfer(x)
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { dst = xchg8(0); }
#define READ_16(dst)  { dst = xchg8(0); dst = (dst << 8) | xchg8(0);  }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))

#define CD_COMMAND {flush(); PIN_LOW(CD_PORT, CD_PIN); }
#define CD_DATA    {flush(); PIN_HIGH(CD_PORT, CD_PIN); }
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE  PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE    {flush(); PIN_HIGH(CS_PORT, CS_PIN); }
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)

// General macros.   IOCLR registers are 1 cycle when optimised.

#define CTL_INIT()   { CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; SPI_INIT(); }
#define WriteCmd(x)  { CD_COMMAND; write8(x); }
#define WriteData(x) { CD_DATA; write16(x); }
