////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  LPC Flash Recovery using Teensy 4.0
//
////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Globals & Includes
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <LittleFS.h>

LittleFS_Program myfs;

#define MAX_MSG_SIZE    256
#define MAXARG           32

#include <stdint.h>

char m_ChipType[128];
char m_FlashType[128];
char m_FlashDetails[128];
int m_FlashIndex = 0;

uint8_t FLASH_CMD_SET = 0xFF;

uint32_t FLASH_IDREG_BASE = 0xFF800000;
uint32_t FLASH_REG_BASE = 0xFF800000;
uint32_t FLASH_MEM_BASE = 0xFFC00000;

uint32_t FLASH_MEM_SIZE = 0x040000;
uint32_t FLASH_BLOCK_SIZE = 0x04000;
uint32_t FLASH_SECTOR_SIZE = 0x01000;

#define FLASH_CHIP_MASK   (FLASH_MEM_SIZE - 1)
#define FLASH_BLOCK_MASK  (FLASH_BLOCK_SIZE - 1)
#define FLASH_SECTOR_MASK (FLASH_SECTOR_SIZE - 1)

#define FLASH_BLOCK_PAGE_MASK  (FLASH_CHIP_MASK & ~FLASH_BLOCK_MASK)
#define FLASH_SECTOR_PAGE_MASK (FLASH_CHIP_MASK & ~FLASH_SECTOR_MASK)

#define MAX_RETRY 128
#define LPC_TIMEOUT 128

boolean timeout_occured = false;
boolean CHIP_INVALID = true;
boolean USBAttached = false;

unsigned char _bios_buffer[131072];

typedef struct chiplib_s {
  uint8_t Mid;
  uint8_t Cid;

  char Name[32];

  uint8_t IDCount;
  uint8_t IDShift[4];

  uint8_t CommandSet;

  uint32_t IdRegBase;
  uint32_t RegBase;
  uint32_t MemBase;
  uint32_t MemSize;
  uint32_t BlockSize;
  uint32_t SectorSize;
} chiplib_t;

#define CMDSET_SST49LF020  0x00
#define CMDSET_SST49LF020A 0x01
#define CMDSET_SST49LF160C 0x02

#define CMDSET_WINBOND040 0x10
#define CMDSET_WINBOND002 0x12

chiplib_t CHIPLIB[] = {
  { 0xbf, 0x61, "SST 49LF020",       1, { 32, 32, 32, 32 }, CMDSET_SST49LF020, 0xFF800000, 0xFF800000, 0xFFC00000, 0x040000, 0x004000, 0x001000 },
  { 0xbf, 0x52, "SST 49LF020A",     16, { 18, 19, 20, 21 }, CMDSET_SST49LF020A, 0xFF800000, 0xFF800000, 0xFFC00000, 0x040000, 0x004000, 0x001000 },
  { 0xbf, 0x1C, "SST 49LF030A",     16, { 19, 20, 21, 23 }, CMDSET_SST49LF020A, 0xFF040000, 0xFF040000, 0xFF420000, 0x060000, 0x010000, 0x001000 },
  { 0xbf, 0x53, "SST 49LF040A",     16, { 19, 20, 21, 23 }, CMDSET_SST49LF020A, 0xFF040000, 0xFF040000, 0xFF400000, 0x080000, 0x010000, 0x001000 },
  { 0xbf, 0x5b, "SST 49LF080A",     16, { 20, 21, 23, 24 }, CMDSET_SST49LF020A, 0xFE0C0000, 0xFE0C0000, 0xFE400000, 0x100000, 0x010000, 0x001000 },
  { 0xbf, 0x4c, "SST 49LF160C",     16, { 21, 23, 24, 25 }, CMDSET_SST49LF160C, 0xFC1C0000, 0xFC000000, 0xFC400000, 0x200000, 0x010000, 0x001000 },

  { 0xda, 0x3d, "WINBOND W39V040A",  8, { 19, 20, 21, 32 }, CMDSET_WINBOND040, 0xFFC00000, 0xFFC00000, 0xFFC00000, 0x080000, 0x004000, 0x001000 },
};

#define MAX_CHIP_TYPE (sizeof(CHIPLIB) / sizeof(chiplib_t))

unsigned char chip_type = 0xff;

////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Pin Definitions
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#define PIN_LAD0 2
#define PIN_LAD1 3
#define PIN_LAD2 4
#define PIN_LAD3 5

#define PIN_LCLOCK 10
#define PIN_LRESET 11
#define PIN_LFRAME 12

////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Timing Macros
//
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Some trial and error required, the Teensy 4.0 is 600MHz, but it's I/O pins can't slew that fast.
//  LPC Spec calls for 33MHz, but most flash chips implement a state machine that can operate slower.
inline void lpc_delay() {
//  asm("nop");  asm("nop");  asm("nop");  asm("nop");
//  asm("nop");  asm("nop");  asm("nop");  asm("nop");
//  asm("nop");  asm("nop");  asm("nop");  asm("nop");
  asm("nop");  asm("nop");  asm("nop");  asm("nop");
  asm("nop");  asm("nop");
}

#define lpc_delay1()  { lpc_delay(); }
#define lpc_delay2()  { lpc_delay1(); lpc_delay1(); }
#define lpc_delay3()  { lpc_delay2(); lpc_delay1(); }
#define lpc_delay4()  { lpc_delay3(); lpc_delay1(); }
#define lpc_delay5()  { lpc_delay4(); lpc_delay1(); }
#define lpc_delay6()  { lpc_delay5(); lpc_delay1(); }
#define lpc_delay7()  { lpc_delay6(); lpc_delay1(); }
#define lpc_delay8()  { lpc_delay7(); lpc_delay1(); }
#define lpc_delay9()  { lpc_delay8(); lpc_delay1(); }
#define lpc_delay10() { lpc_delay9(); lpc_delay1(); }
#define lpc_delay11() { lpc_delay10(); lpc_delay1(); }
#define lpc_delay12() { lpc_delay11(); lpc_delay1(); }
#define lpc_delay13() { lpc_delay12(); lpc_delay1(); }
#define lpc_delay14() { lpc_delay13(); lpc_delay1(); }
#define lpc_delay15() { lpc_delay14(); lpc_delay1(); }
#define lpc_delay16() { lpc_delay15(); lpc_delay1(); }
#define lpc_delay17() { lpc_delay16(); lpc_delay1(); }
#define lpc_delay18() { lpc_delay17(); lpc_delay1(); }
#define lpc_delay19() { lpc_delay18(); lpc_delay1(); }
#define lpc_delay20() { lpc_delay19(); lpc_delay1(); }
#define lpc_delay21() { lpc_delay20(); lpc_delay1(); }
#define lpc_delay22() { lpc_delay21(); lpc_delay1(); }
#define lpc_delay23() { lpc_delay22(); lpc_delay1(); }
#define lpc_delay24() { lpc_delay23(); lpc_delay1(); }
#define lpc_delay25() { lpc_delay24(); lpc_delay1(); }

////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Low level I/O routines
//
////////////////////////////////////////////////////////////////////////////////////////////////////

inline void lpc_in() {
  GPIO9_GDIR = 0x0000;
}

inline void lpc_out() {
  GPIO9_GDIR = 0x0170;
}

unsigned long bm[16] = {
  0x000, 0x010, 0x020, 0x030, 0x040, 0x050, 0x060, 0x070,
  0x100, 0x110, 0x120, 0x130, 0x140, 0x150, 0x160, 0x170
};

unsigned char bs[32] = {
  0x0,  0x1,  0x2,  0x3,  0x4,  0x5,  0x6,  0x7,
  0x0,  0x1,  0x2,  0x3,  0x4,  0x5,  0x6,  0x7,
  0x8,  0x9,  0xa,  0xb,  0xc,  0xd,  0xe,  0xf,
  0x8,  0x9,  0xa,  0xb,  0xc,  0xd,  0xe,  0xf
};

inline void lpc_setnyb(unsigned long b) {
  GPIO9_DR = bm[b & 0xf];
}

inline unsigned char lpc_getnyb() {
  unsigned long s = GPIO9_PSR;
  return bs[(s >> 4) & 0x17];
}

#define BIT_LCLOCK (1<<0)
#define BIT_LRESET (1<<2)
#define BIT_LFRAME (1<<1)

inline void lpc_assertclockframe() {
  GPIO7_DR = BIT_LRESET;
}

inline void lpc_assertclock() {
  GPIO7_DR = BIT_LFRAME | BIT_LRESET;
}

inline void lpc_deassertclock() {
  GPIO7_DR_SET   = BIT_LCLOCK;
}

inline void lpc_clockcycle() {
  lpc_assertclock();
  lpc_deassertclock();
}

inline void lpc_assertframe() {
  GPIO7_DR_CLEAR = BIT_LFRAME;
}

inline void lpc_deassertframe() {
  GPIO7_DR_SET   = BIT_LFRAME;
}

inline void lpc_assertreset() {
  GPIO7_DR_CLEAR = BIT_LRESET;
}

inline void lpc_deassertreset() {
  GPIO7_DR_SET   = BIT_LRESET;
}

////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Mid level I/O routines
//
////////////////////////////////////////////////////////////////////////////////////////////////////

inline void lpc_newframe(unsigned char b) {
  lpc_out();
  lpc_setnyb(b);
  lpc_assertclockframe();
  lpc_delay17();
  lpc_deassertclock();
  lpc_delay13();
  lpc_delay();
}

inline void lpc_abort() {
  lpc_newframe(0xf);
  lpc_deassertframe();
}

inline void lpc_writenyb(unsigned char b) {
  lpc_setnyb(b);
  lpc_assertclock();
  lpc_delay17();
  lpc_deassertclock();
  lpc_delay14();
}

inline unsigned char lpc_readnyb() {
  unsigned char b;

  lpc_assertclock();
  lpc_delay15();
  b = lpc_getnyb();
  lpc_deassertclock();
  lpc_delay15();

  return b;
}

inline void lpc_turnaround(bool out) {
  if(out) {
    lpc_readnyb();
    lpc_out();
    lpc_writenyb(0b1111);
  } else {
    lpc_writenyb(0b1111);
    lpc_in();
//    lpc_readnyb();
  }
}

inline void lpc_writebyte(unsigned char b) {
  lpc_writenyb(b);
  lpc_writenyb(b >> 4);
}

inline unsigned char lpc_readbyte() {
  unsigned char b;

  b  = lpc_readnyb();
  b |= lpc_readnyb() << 4;

  return b;
}

inline bool lpc_waitsync() {
  unsigned char tmp;
  int timeout = LPC_TIMEOUT;

  while(timeout > 0) {
    tmp = lpc_readnyb();                // Wait for SYNC

    switch(tmp) {
      case 0b0000:                      // Ready
        return true;                    //

      case 0b0101:                      // Short Wait
        timeout -= 3;                  //
        break;                          //

      case 0b0110:                      // Long Wait
        timeout --;                     //
        break;                          //

      case 0b1010:                      // Error
        //Serial.print("E");
        lpc_abort();
        return false;
        break;

      default:
        timeout --;                     //
        break;                          //
    }
  };

  //Serial.print("T");
  lpc_abort();                         // Timeout waiting for SYNC
  return false;                        // return to calling function
}

////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  High level I/O routines
//
////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char lpc_readflashbyte(unsigned long addr) {
  unsigned char tmp;

  lpc_newframe(0b1101);                 // Start Firmware Memory Read Cycle
  lpc_writenyb(0b1111);                 // IDSEL

  lpc_writenyb((addr >> 24) & 0xf);     // MADDR
  lpc_writenyb((addr >> 20) & 0xf);     // 
  lpc_writenyb((addr >> 16) & 0xf);     // 
  lpc_writenyb((addr >> 12) & 0xf);     // 
  lpc_writenyb((addr >>  8) & 0xf);     // 
  lpc_writenyb((addr >>  4) & 0xf);     // 
  lpc_writenyb((addr >>  0) & 0xf);     // 

  lpc_writenyb(0b0000);                 // MSIZE = 1 Byte

  lpc_turnaround(0);                    // Turn-Around Out -> In

  if(!lpc_waitsync()) {
    timeout_occured = true;
    return 0xff;
  }
  
  tmp  = lpc_readbyte();                // DATA

  lpc_turnaround(1);                    // Turn-Around In -> Out

  timeout_occured = false;
  return tmp;                           // return to calling function
}

bool lpc_writeflashbyte(unsigned long addr, unsigned char b) {
  lpc_newframe(0b1110);                 // Start Firmware Memory Write Cycle
  lpc_writenyb(0b1111);                 // IDSEL

  lpc_writenyb((addr >> 24) & 0xf);     // MADDR
  lpc_writenyb((addr >> 20) & 0xf);     // 
  lpc_writenyb((addr >> 16) & 0xf);     // 
  lpc_writenyb((addr >> 12) & 0xf);     // 
  lpc_writenyb((addr >>  8) & 0xf);     // 
  lpc_writenyb((addr >>  4) & 0xf);     // 
  lpc_writenyb((addr >>  0) & 0xf);     // 

  lpc_writenyb(0b0000);                 // MSIZE = 1 Byte

  lpc_writebyte(b);                     // DATA

  lpc_turnaround(0);                    // Turn-Around Out -> In

  if(!lpc_waitsync()) {
    timeout_occured = true;
    return false;
  }

  lpc_turnaround(1);                    // Turn-Around In -> Out

  timeout_occured = false;
  return true;                          // return to calling function
}

unsigned char lpc_readiobyte(unsigned short addr) {
  unsigned char tmp;
  unsigned long tries = 0;

  while(tries < MAX_RETRY) {
    lpc_newframe(0b0000);                 // Start IO Read Cycle
    lpc_writenyb(0b0000);                 // Read
  
    lpc_writenyb((addr >> 12) & 0xf);     // IOADDR
    lpc_writenyb((addr >>  8) & 0xf);     // 
    lpc_writenyb((addr >>  4) & 0xf);     // 
    lpc_writenyb((addr >>  0) & 0xf);     // 
  
    lpc_turnaround(0);                    // Turn-Around Out -> In
  
    if(!lpc_waitsync()) {
      lpc_abort();
      tries ++;
    } else {
      tmp  = lpc_readbyte();                // DATA
  
      lpc_turnaround(1);                    // Turn-Around In -> Out
  
      timeout_occured = false;
      return tmp;                           // return to calling function
    }
  }

  timeout_occured = true;
  return 0xff;
}

bool lpc_writeiobyte(unsigned short addr, unsigned char b) {
  unsigned long tries = 0;

  while(tries < MAX_RETRY) {
    lpc_newframe(0b0000);                 // Start IO Write Cycle
    lpc_writenyb(0b0010);                 // Write
  
    lpc_writenyb((addr >> 12) & 0xf);     // IOADDR
    lpc_writenyb((addr >>  8) & 0xf);     // 
    lpc_writenyb((addr >>  4) & 0xf);     // 
    lpc_writenyb((addr >>  0) & 0xf);     // 
  
    lpc_writebyte(b);                     // DATA
  
    lpc_turnaround(0);                    // Turn-Around Out -> In
  
    if(!lpc_waitsync()) {
      lpc_abort();
      tries ++;
    } else {
      lpc_turnaround(1);                    // Turn-Around In -> Out

      timeout_occured = false;
      return true;                          // return to calling function
    }
  }
  timeout_occured = true;
  return false;
}


unsigned char lpc_readmembyte(unsigned long addr) {
  unsigned char tmp;
  unsigned long tries = 0;

  while(tries < MAX_RETRY) {
    lpc_newframe(0b0000);                 // Start Memory Read Cycle
    lpc_writenyb(0b0100);                 // Read
  
    lpc_writenyb((addr >> 28) & 0xf);     // MADDR
    lpc_writenyb((addr >> 24) & 0xf);     // 
    lpc_writenyb((addr >> 20) & 0xf);     // 
    lpc_writenyb((addr >> 16) & 0xf);     // 
    lpc_writenyb((addr >> 12) & 0xf);     // 
    lpc_writenyb((addr >>  8) & 0xf);     // 
    lpc_writenyb((addr >>  4) & 0xf);     // 
    lpc_writenyb((addr >>  0) & 0xf);     // 
  
    lpc_turnaround(0);                    // Turn-Around Out -> In
  
    if(!lpc_waitsync()) {
      lpc_abort();
      tries ++;
    } else {
      tmp  = lpc_readbyte();                // DATA
  
      lpc_turnaround(1);                    // Turn-Around In -> Out
  
      timeout_occured = false;
      
      //Serial.printf("R 0x%08x: 0x%02x\r\n", addr, tmp);
      return tmp;                           // return to calling function
    }
  }

  timeout_occured = true;
  return 0xff;
}

bool lpc_writemembyte(unsigned long addr, unsigned char b) {
  unsigned long tries = 0;

  //Serial.printf("W 0x%08x: 0x%02x\r\n", addr, b);

  while(tries < MAX_RETRY) {
    lpc_newframe(0b0000);                 // Start Memory Write Cycle
    lpc_writenyb(0b0110);                 // Write
  
    lpc_writenyb((addr >> 28) & 0xf);     // MADDR
    lpc_writenyb((addr >> 24) & 0xf);     // 
    lpc_writenyb((addr >> 20) & 0xf);     // 
    lpc_writenyb((addr >> 16) & 0xf);     // 
    lpc_writenyb((addr >> 12) & 0xf);     // 
    lpc_writenyb((addr >>  8) & 0xf);     // 
    lpc_writenyb((addr >>  4) & 0xf);     // 
    lpc_writenyb((addr >>  0) & 0xf);     // 
  
    lpc_writebyte(b);                     // DATA
  
    lpc_turnaround(0);                    // Turn-Around Out -> In
  
    if(!lpc_waitsync()) {
      lpc_abort();
      tries ++;
    } else {
      lpc_turnaround(1);                    // Turn-Around In -> Out

      timeout_occured = false;
      return true;                          // return to calling function
    }
  }
  timeout_occured = true;
  return false;
}

// Read until a value wins
unsigned char lpc_multireadmembyte(unsigned long addr) {
  unsigned long timeout = MAX_RETRY;
  unsigned char tmp;
  unsigned char buf[256];
  memset(buf, 0, 256);

  while(timeout > 0) {
    tmp = lpc_readmembyte(addr);
    if(!timeout_occured) {
      buf[tmp]++;
      if(buf[tmp] > MAX_RETRY) return tmp;
    } else {
      timeout--;
    }
  }

  return 0xff;
}
////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  LPC Flash routines
//
////////////////////////////////////////////////////////////////////////////////////////////////////

void lpc_unprotect() {
  unsigned long addr;
  if(FLASH_CMD_SET == CMDSET_SST49LF160C) {
    // Unprotect the whole damn chip.
    for(int i = 0x00; i < 0x23; i++) {
      if(i == 0x20) {
        addr = FLASH_REG_BASE | 0x1F8002;
      } else if(i == 0x21) {
        addr = FLASH_REG_BASE | 0x1FA002;
      } else if(i == 0x22) {
        addr = FLASH_REG_BASE | 0x1FC002;
      } else {
        addr = FLASH_REG_BASE | (i << 16) | 0x000002;
      }
      lpc_writemembyte(addr, 0x00);
    }
  }
  lpc_writemembyte(FLASH_MEM_BASE, 0xFF);
}

void lpc_isprotect() {
  unsigned long addr;
  if(FLASH_CMD_SET == CMDSET_SST49LF160C) {
    for(int i = 0x00; i < 0x23; i++) {
      unsigned char tmp;
      if(i == 0x20) {
        addr = FLASH_REG_BASE | 0x1F8002;
      } else if(i == 0x21) {
        addr = FLASH_REG_BASE | 0x1FA002;
      } else if(i == 0x22) {
        addr = FLASH_REG_BASE | 0x1FC002;
      } else {
        addr = FLASH_REG_BASE | (i << 16) | 0x000002;
      }
      tmp = lpc_readmembyte(addr);
      switch(tmp) {
        case 0x00:
          Serial.print("u");
          break;
        case 0x01:
          Serial.print("w");
          break;
        case 0x02:
          Serial.print("U");
          break;
        case 0x03:
          Serial.print("W");
          break;
        case 0x04:
          Serial.print("r");
          break;
        case 0x05:
          Serial.print("b");
          break;
        case 0x06:
          Serial.print("R");
          break;
        case 0x07:
          Serial.print("B");
          break;
        default:
          Serial.print("?");
          break;
      }
    }
    Serial.println();
  }
  lpc_writemembyte(FLASH_MEM_BASE, 0xFF);
}

bool lpc_program(unsigned long base, unsigned char *buf, unsigned long len) {
  unsigned long timeout;
  unsigned long addr;
  unsigned char tmp;

  if(chip_type >= MAX_CHIP_TYPE) {
    Serial.printf("\r\nERROR: Can't program unknown chip type!\r\n");
    return false;
  }

  switch(FLASH_CMD_SET) {
    case CMDSET_SST49LF020:
    case CMDSET_SST49LF020A:
      {
        for(addr = 0; addr < len; addr ++) {
          if((addr & FLASH_SECTOR_MASK) == 0)
            dlg_draw_progress_good("Flashing", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
      
          lpc_writemembyte(FLASH_MEM_BASE | 0x5555, 0xAA);
          lpc_writemembyte(FLASH_MEM_BASE | 0x2AAA, 0x55);
          lpc_writemembyte(FLASH_MEM_BASE | 0x5555, 0xA0);  // Byte Program
          lpc_writemembyte(base + addr, buf[addr]);
      
          timeout = LPC_TIMEOUT;
          while(((tmp = lpc_readmembyte(base + addr)) != buf[addr]) && (timeout > 0)) {
            timeout--;
          }
          if(!timeout)
            lpc_writemembyte(base + addr, 0xff);  // Exit Program Mode
        }
      }
      break;
    case CMDSET_SST49LF160C:
      {
        lpc_unprotect();
        
        for(addr = 0; addr < len; addr ++) {
          if((addr & FLASH_SECTOR_MASK) == 0)
            dlg_draw_progress_good("Flashing", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);

          lpc_writemembyte(base + addr, 0xff);  // Reset to Read Mode
          lpc_writemembyte(base + addr, 0x50);  // Clear Status Register
          lpc_writemembyte(base + addr, 0x40);  // Enter Program Mode
          lpc_writemembyte(base + addr, buf[addr]);

          lpc_writemembyte(base + addr, 0x70);  // Read Status Register

          do {
            tmp = lpc_readmembyte(base + addr);
            switch(tmp) {
              case 0x80: // Write Complete
                lpc_writemembyte(base + addr, 0x50);  // Clear Status Register
                timeout = 0;
                break;
              case 0x82: // Write Error
                dlg_draw_progress_bad("Flashing", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
                //Serial.printf("\r\nByte Program failure at 0x%08x, Block Write-Lock Protected\r\n ", base + addr);
                lpc_writemembyte(base + addr, 0x50);  // Clear Status Register
                timeout = 0;
                break;
            }
          } while((tmp & 0x80) == 0x00); // Write Busy
        }
      }
      break;
  }
  return true;
}

bool lpc_sectorerase(unsigned long base) {
  unsigned long timeout;
  unsigned char tmp;

  if(chip_type >= MAX_CHIP_TYPE) {
    dlg_draw_error("Error: Can't erase unknown chip type!");
    return false;
  }

  switch(FLASH_CMD_SET) {
    case CMDSET_SST49LF020:
    case CMDSET_SST49LF020A:
      {
        lpc_writemembyte(FLASH_MEM_BASE | 0x5555, 0xAA);
        lpc_writemembyte(FLASH_MEM_BASE | 0x2AAA, 0x55);
        lpc_writemembyte(FLASH_MEM_BASE | 0x5555, 0x80); // Unprotect
        lpc_writemembyte(FLASH_MEM_BASE | 0x5555, 0xAA);
        lpc_writemembyte(FLASH_MEM_BASE | 0x2AAA, 0x55);
        lpc_writemembyte(base, 0x30);       // Erase 4KB Sector
      
        timeout = LPC_TIMEOUT;
        while((tmp = lpc_readmembyte(base)) != 0xFF) {
          delay(1);
          timeout--;
          if(timeout == 0) {
            //Serial.printf("\r\nSector Erase timeout at 0x%08x: %02x != ff\r\n ", base, tmp);
            timeout_occured = true;
            return false;
          }
        }
      }
      break;
    case CMDSET_SST49LF160C:
      {
        lpc_unprotect();

        lpc_writemembyte(base, 0xff);  // Reset to Read Mode
        lpc_writemembyte(base, 0x50);  // Clear Status Register

        lpc_writemembyte(base, 0x30);
        lpc_writemembyte(base, 0xD0);
      
        lpc_writemembyte(base, 0x70);  // Read Status Register

        do {
          tmp = lpc_readmembyte(base);
          switch(tmp) {
            case 0x80: // Write Complete
              lpc_writemembyte(base, 0x50);  // Clear Status Register
              return true;

            case 0x82: // Write Error
              //Serial.printf("\r\nSector Erase failure at 0x%08x, Block Write-Lock Protected\r\n ", base);
              lpc_writemembyte(base, 0x50);  // Clear Status Register
              return false;
          }
        } while((tmp & 0x80) == 0x00); // Write Busy
      }
      break;
  }

  timeout_occured = false;
  return false;
}

bool lpc_blockerase(unsigned long base) {
  unsigned long timeout;
  unsigned char tmp;

  if(chip_type >= MAX_CHIP_TYPE) {
    dlg_draw_error("Error: Can't erase unknown chip type!");
    return false;
  }

  switch(FLASH_CMD_SET) {
    case CMDSET_SST49LF020:
    case CMDSET_SST49LF020A:
      {
        lpc_writemembyte(FLASH_MEM_BASE | 0x5555, 0xAA);
        lpc_writemembyte(FLASH_MEM_BASE | 0x2AAA, 0x55);
        lpc_writemembyte(FLASH_MEM_BASE | 0x5555, 0x80); // Unprotect
        lpc_writemembyte(FLASH_MEM_BASE | 0x5555, 0xAA);
        lpc_writemembyte(FLASH_MEM_BASE | 0x2AAA, 0x55);
        lpc_writemembyte(base, 0x50);       // Erase 16KB Block
      
        timeout = LPC_TIMEOUT;
        while((tmp = lpc_readmembyte(base)) != 0xFF) {
          delay(1);
          timeout--;
          if(timeout == 0) {
            //Serial.printf("\r\nBlock Erase timeout at 0x%08x: %02x != ff\r\n ", base, tmp);
            timeout_occured = true;
            return false;
          }
        }
      }
      break;
    case CMDSET_SST49LF160C:
      {
        lpc_unprotect();

        lpc_writemembyte(base, 0xff);  // Reset to Read Mode
        lpc_writemembyte(base, 0x50);  // Clear Status Register

        lpc_writemembyte(base, 0x20);
        lpc_writemembyte(base, 0xD0);
      
        lpc_writemembyte(base, 0x70);  // Read Status Register

        do {
          tmp = lpc_readmembyte(base);
          switch(tmp) {
            case 0x80: // Write Complete
              lpc_writemembyte(base, 0x50);  // Clear Status Register
              return true;

            case 0x82: // Write Error
              //Serial.printf("\r\nBlock Erase failure at 0x%08x, Block Write-Lock Protected\r\n ", base);
              lpc_writemembyte(base, 0x50);  // Clear Status Register
              return false;
          }
        } while((tmp & 0x80) == 0x00); // Write Busy

      }
      break;
  }

  timeout_occured = false;
  return false;
}

bool lpc_compare(unsigned long base, unsigned char *buf, unsigned long len, boolean quiet) {
  unsigned long addr;
  unsigned char tmp;
  lpc_abort();
  for(addr = 0; (addr < len); addr++) {
    if((addr & FLASH_SECTOR_MASK) == 0)
      dlg_draw_progress_good("Verifying", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);

    lpc_writemembyte(base + addr, 0xff);  // Reset to Read Mode
    if((tmp = lpc_readmembyte(base + addr)) != buf[addr]) {
      dlg_draw_progress_bad("Verifying", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
      return false;
    }
  }

  return true;
}

bool lpc_programmable(unsigned long base, unsigned char *buf, unsigned long len) {
  unsigned long addr;
  unsigned char tmp;
  lpc_abort();
  for(addr = 0; (addr < len); addr++) {
    if((addr & FLASH_SECTOR_MASK) == 0)
      dlg_draw_progress_good("Checking", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);

    lpc_writemembyte(base + addr, 0xff);  // Reset to Read Mode
    tmp = lpc_readmembyte(base + addr);
    if((tmp & buf[addr]) != buf[addr]) {
      dlg_draw_progress_pending("Checking", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
      return false;
    }
  }

  return true;
}

bool lpc_blank(unsigned long base, unsigned long len, boolean quiet) {
  unsigned long addr;
  unsigned char tmp;
  lpc_abort();
  for(addr = 0; (addr < len); addr++) {
    if((addr & FLASH_SECTOR_MASK) == 0)
      dlg_draw_progress_good("Checking", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);

    lpc_writemembyte(base + addr, 0xff);  // Reset to Read Mode
    if((tmp = lpc_readmembyte(base + addr)) != 0xff) {
      if(!quiet)
      dlg_draw_progress_pending("Checking", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
      return false;
    }
  }

  return true;
}

bool lpc_read(unsigned long base, unsigned char *buf, unsigned long len) {
  unsigned long addr;

  lpc_abort();
  for(addr = 0; (addr < len); addr++) {
    if((addr & FLASH_SECTOR_MASK) == 0)
      dlg_draw_progress_good("Reading", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);

    lpc_writemembyte(base + addr, 0xff);  // Reset to Read Mode
    buf[addr] = lpc_readmembyte(base + addr);
  }

  return true;
}

bool lpc_chipsectorerase(unsigned long base) {
  unsigned long addr;
  unsigned long fail = 0;

  for(addr = 0; addr < FLASH_MEM_SIZE; addr += FLASH_SECTOR_SIZE) {
    unsigned long blockfail = 0;
    if(!lpc_blank((base + addr), FLASH_SECTOR_SIZE, true)) {
      if(!lpc_sectorerase(base + addr)) {
        dlg_draw_progress_bad("Erasing", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
        blockfail = 1;
      } else {
        dlg_draw_progress_good("Erasing", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
      }
    }
    if(!lpc_blank((base + addr), FLASH_SECTOR_SIZE, false)) {
        dlg_draw_progress_bad("Checking", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
        blockfail = 1;
    }

    fail += blockfail;
  }

  return (fail == 0);
}

bool lpc_chipblockerase(unsigned long base) {
  unsigned long addr;
  unsigned long fail = 0;

  for(addr = 0; addr < FLASH_MEM_SIZE; addr += FLASH_BLOCK_SIZE) {
    unsigned long blockfail = 0;
    if(!lpc_blank((base + addr), FLASH_BLOCK_SIZE, true)) {
      if(!lpc_blockerase(base + addr)) {
        dlg_draw_progress_bad("Erasing", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
        blockfail = 1;
      } else {
        dlg_draw_progress_good("Erasing", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
      }
    }
    if(!lpc_blank((base + addr), FLASH_BLOCK_SIZE, false)) {
        dlg_draw_progress_bad("Checking", base + addr - FLASH_MEM_BASE, FLASH_MEM_SIZE);
        blockfail = 1;
    }

    fail += blockfail;
  }

  return (fail == 0);
}

void lpc_init() {
  pinMode(PIN_LRESET, OUTPUT);
  lpc_assertreset();
  
  pinMode(PIN_LCLOCK, OUTPUT);
  lpc_deassertclock();

  pinMode(PIN_LFRAME, OUTPUT);
  lpc_deassertframe();

  pinMode(PIN_LAD0, OUTPUT);
  pinMode(PIN_LAD1, OUTPUT);
  pinMode(PIN_LAD2, OUTPUT);
  pinMode(PIN_LAD3, OUTPUT);

  lpc_out();
  lpc_setnyb(0);

  for(int i = 0; i < 24; i++)
    lpc_writenyb(0);
  
  lpc_deassertreset();
  asm("nop");

  for(int i = 0; i < 64; i++)
    lpc_writenyb(0);

  lpc_abort();
}
  
void lpc_getid(int ChipIndex) {
  unsigned char type;
  unsigned char m_mid;
  unsigned char m_cid;
  uint32_t baseoffset;

  ChipIndex--;

  m_FlashIndex = ChipIndex;

  // Place Chip in Bypass if LPCMangler, 1MB for OpenXenium

  strcpy(m_ChipType, "Generic");
  if(lpc_readiobyte(0x00EE) == 0x55) {
    strcpy(m_ChipType, "Xenium");
    lpc_writeiobyte(0x00EF, 0xF9);
  } else if(lpc_readiobyte(0xF500) == 0xE1) {
    strcpy(m_ChipType, "X3cuter");
  } else if(lpc_readiobyte(0x00EF) == 0xAA) {
    strcpy(m_ChipType, "LPCMangler");
    if(ChipIndex > 1) goto nochip;

    ChipIndex = ChipIndex ? 0x2 : 0x1;
  }

  for(type = 0; type < MAX_CHIP_TYPE; type++) {
    if(ChipIndex < CHIPLIB[type].IDCount) {
      baseoffset = 0;
      if(CHIPLIB[type].IDShift[0] >= 0) {
        baseoffset |= ((ChipIndex & 1) ? 0 : (1 << CHIPLIB[type].IDShift[0]));
      }
      if(CHIPLIB[type].IDShift[1] >= 0) {
        baseoffset |= ((ChipIndex & 2) ? 0 : (1 << CHIPLIB[type].IDShift[1]));
      }
      if(CHIPLIB[type].IDShift[2] >= 0) {
        baseoffset |= ((ChipIndex & 4) ? 0 : (1 << CHIPLIB[type].IDShift[2]));
      }
      if(CHIPLIB[type].IDShift[3] >= 0) {
        baseoffset |= ((ChipIndex & 8) ? 0 : (1 << CHIPLIB[type].IDShift[3]));
      }

      FLASH_IDREG_BASE = CHIPLIB[type].IdRegBase + baseoffset;
      FLASH_REG_BASE = CHIPLIB[type].RegBase + baseoffset;
      FLASH_MEM_BASE = CHIPLIB[type].MemBase + baseoffset;

      FLASH_MEM_SIZE = CHIPLIB[type].MemSize;
      FLASH_BLOCK_SIZE = CHIPLIB[type].BlockSize;
      FLASH_SECTOR_SIZE = CHIPLIB[type].SectorSize;

      FLASH_CMD_SET = CHIPLIB[type].CommandSet;

      m_mid = 0xff;
      m_cid = 0xff;
      switch(CHIPLIB[type].CommandSet) {
        case CMDSET_SST49LF020:
          {
            lpc_writemembyte(FLASH_IDREG_BASE | 0x5555, 0xAA);
            lpc_writemembyte(FLASH_IDREG_BASE | 0x2AAA, 0x55);
            lpc_writemembyte(FLASH_IDREG_BASE | 0x5555, 0x90);
            m_mid = lpc_readmembyte(FLASH_IDREG_BASE+0);
            m_cid = lpc_readmembyte(FLASH_IDREG_BASE+1);
            lpc_writemembyte(FLASH_IDREG_BASE | 0x5555, 0xAA);
            lpc_writemembyte(FLASH_IDREG_BASE | 0x2AAA, 0x55);
            lpc_writemembyte(FLASH_IDREG_BASE | 0x5555, 0xF0);
          }
          break;
        case CMDSET_SST49LF020A:
          {
            m_mid = lpc_readmembyte(FLASH_IDREG_BASE+0);
            m_cid = lpc_readmembyte(FLASH_IDREG_BASE+1);
          }
          break;
        case CMDSET_SST49LF160C:
          {
            lpc_writemembyte(FLASH_MEM_BASE, 0xFF);
            lpc_writemembyte(FLASH_MEM_BASE, 0x90);
            m_mid = lpc_readmembyte(FLASH_IDREG_BASE+0);
            m_cid = lpc_readmembyte(FLASH_IDREG_BASE+1);
            lpc_writemembyte(FLASH_MEM_BASE, 0xFF);
          }
          break;
      }

      if((m_mid == CHIPLIB[type].Mid) && (m_cid == CHIPLIB[type].Cid)) {
        chip_type = type;
        CHIP_INVALID = false;
        sprintf(m_FlashType, "%s # %d @ 0x%08lX [ID:%02X%02X]", CHIPLIB[type].Name, ChipIndex, FLASH_MEM_BASE, m_mid, m_cid);
        sprintf(m_FlashDetails, "%ld KB Flash, %ld KB Blocks, %ld KB Sectors", (FLASH_MEM_SIZE >> 10), (FLASH_BLOCK_SIZE >> 10), (FLASH_SECTOR_SIZE >> 10));
        dlg_draw_header();
        return;
      }
    }
  }

nochip:
  FLASH_CMD_SET = 0xFF;
  CHIP_INVALID = true;
  chip_type = 0xff;
  m_FlashIndex = 0;
  Serial.println("Unknown chip.");
}

bool lpc_sectorflash(unsigned long base, unsigned char *buf) {
  // Check if we need to do anything to this sector
  if(lpc_compare(base, buf, FLASH_SECTOR_SIZE, true)) {
    return true;
  }

  // Check if we need erase this sector
  if(!lpc_blank(base, FLASH_SECTOR_SIZE, true) && !lpc_programmable(base, buf, FLASH_SECTOR_SIZE)) {
    dlg_draw_error("Device requires erasing.");
    return false;
  }

  // Write the sector
  if(!lpc_program(base, buf, FLASH_SECTOR_SIZE)) {
    return false;
  }

  // Verify the sector
  if(!lpc_compare(base, buf, FLASH_SECTOR_SIZE, false)) {
    return false;
  }

  // Success!
  return true;
}

bool lpc_blockflash(unsigned long base, unsigned char *buf) {
  // Check if we need to do anything to this block
  if(lpc_compare(base, buf, FLASH_BLOCK_SIZE, true)) {
    return true;
  }

  // Check if we need erase this block
  if(!lpc_blank(base, FLASH_BLOCK_SIZE, true) && !lpc_programmable(base, buf, FLASH_BLOCK_SIZE)) {
    dlg_draw_error("Device requires erasing.");
    return false;
  }

  // Write the sector
  if(!lpc_program(base, buf, FLASH_BLOCK_SIZE)) {
    return false;
  }

  // Verify the sector
  if(!lpc_compare(base, buf, FLASH_BLOCK_SIZE, false)) {
    return false;
  }

  // Success!
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Dialog TUI
//
////////////////////////////////////////////////////////////////////////////////////////////////////

// Colors:

// Border  = Bright Blue on Black
#define dlg_color_border() Serial.print("\x1b[0;1;40;34m")

// Header  = Bright White on Blue
#define dlg_color_header() Serial.print("\x1b[0;1;44;37m")

// Screen  = White on Black
#define dlg_color_screen() Serial.print("\x1b[0;40;37m")

// Warning = Bright Red on Black
#define dlg_color_warning() Serial.print("\x1b[0;1;40;31m")

// Status  = Bright Yellow on Blue
#define dlg_color_status() Serial.print("\x1b[0;1;47;33m")

// Good    = Bright Green on Grey
#define dlg_color_good() Serial.print("\x1b[0;1;47;32m")

// Bad     = Bright Red on Grey
#define dlg_color_bad() Serial.print("\x1b[0;1;47;31m")

// Pending = Bright White on Grey
#define dlg_color_pending() Serial.print("\x1b[0;1;47;37m")

// Window  = Bright White on Black
#define dlg_color_window() Serial.print("\x1b[0;1;40;37m")

// Legend  = Dim Grey (Bright Black) on Black
#define dlg_color_legend() Serial.print("\x1b[0;1;40;30m")

// Home Cursor
#define dlg_home_cursor(y, x) Serial.printf("\x1b[%d;%dH", y, x)

#define dlg_set_flashing() Serial.print("\x1b[5m");

// Clear Screen
#define dlg_clear_screen() Serial.print("\x1b[2J")
#define dlg_clear_line() Serial.print("\x1b[2K")

#define dlg_block_top() Serial.print("\xdf")
#define dlg_block_full() Serial.print("\xdb")
#define dlg_block_bottom() Serial.print("\xdc")

void dlg_draw_header() {
  int x,y;

  dlg_color_screen();
  dlg_home_cursor(1, 1);
  dlg_clear_screen();

  dlg_color_border();
  for(x = 1; x<28; x++)
    dlg_block_bottom();
  Serial.print("\xdb");

  dlg_color_header();
  Serial.print(" LPC Recovery Tool v1.0 ");
  
  dlg_color_border();
  Serial.print("\xdb");
  for(x = 54; x<81; x++)
    dlg_block_bottom();

  for(y = 2; y < 7; y++) {
    dlg_home_cursor(y, 1);
    dlg_color_header();
    dlg_clear_line();
    dlg_home_cursor(y, 1);
    dlg_color_border();
    dlg_block_full();
    dlg_home_cursor(y, 80);
    dlg_block_full();
  }

  dlg_home_cursor(7, 1);
  for(x = 1; x<81; x++)
    dlg_block_top();

  dlg_color_header();
  dlg_home_cursor(3, 4);
  Serial.print("MODCHIP TYPE : ");
  Serial.print(m_ChipType);

  dlg_home_cursor(4, 5);
  if(m_FlashIndex == 0) {
    Serial.print("BOOT FLASH  : ");
  } else {
    Serial.print("DATA FLASH  : ");
  }
  Serial.print(m_FlashType);

  dlg_home_cursor(5, 19);
  Serial.print(m_FlashDetails);

  dlg_color_screen();
  dlg_home_cursor(8, 1);
}

void dlg_draw_dialog(const char *Operation, uint32_t Total) {
  int x, y;

  uint32_t divisor = Total / 256;

  dlg_draw_header();

  dlg_color_window();
  dlg_home_cursor(10, 23);
  for(x = 23; x < 57; x++) {
    dlg_block_bottom();
  }

  for(y = 11; y < 19; y++) {
    dlg_home_cursor(y, 23);
    dlg_color_window();
    dlg_block_full();
    dlg_color_pending();
    for(x = 24; x < 56; x++) {
      Serial.printf(" ");
    }
    dlg_color_window();
    dlg_block_full();

    dlg_color_legend();
    dlg_home_cursor(y, 15);
    Serial.printf("%06X", (y-11) * divisor * 32);
    dlg_home_cursor(y, 59);
    Serial.printf("%06X", ((y-10) * divisor * 32)-1);
  }

  dlg_home_cursor(19, 23);
  dlg_color_window();
  for(x = 23; x < 57; x++) {
    dlg_block_top();
  }

  dlg_color_warning();
  dlg_set_flashing();
  dlg_home_cursor(21, 33);
  Serial.print("Please Wait!");

  dlg_home_cursor(23, 1);
  dlg_color_header();
  dlg_clear_line();
  dlg_home_cursor(23, 21);
  Serial.print("Current Operation: ");
  Serial.printf("%10s", Operation);
  Serial.printf(" 0x%06X", 0);

  dlg_color_screen();
}

void dlg_draw_progress_good(const char *Operation, uint32_t Position, uint32_t Total) {
  int x, y;

  uint32_t divisor = Total >> 8;
  uint32_t Progress = Position / divisor;

  y = 11 + (Progress >> 5);
  x = 24 + (Progress & 0x1f);
  dlg_home_cursor(y, x);
  dlg_color_good();
  dlg_block_full();

  dlg_color_warning();
  dlg_set_flashing();
  dlg_home_cursor(21, 33);
  Serial.print("Please Wait!");

  dlg_home_cursor(23, 1);
  dlg_color_header();
  dlg_clear_line();
  dlg_home_cursor(23, 21);
  Serial.print("Current Operation: ");
  Serial.printf("%10s", Operation);
  Serial.printf(" 0x%06X", Position);

  dlg_color_screen();
}

void dlg_draw_progress_bad(const char *Operation, uint32_t Position, uint32_t Total) {
  int x, y;

  uint32_t divisor = Total >> 8;
  uint32_t Progress = Position / divisor;

  y = 11 + (Progress >> 5);
  x = 24 + (Progress & 0x1f);
  dlg_home_cursor(y, x);
  dlg_color_bad();
  dlg_block_full();

  dlg_color_warning();
  dlg_set_flashing();
  dlg_home_cursor(21, 33);
  Serial.print("Please Wait!");
  
  dlg_home_cursor(23, 1);
  dlg_color_header();
  dlg_clear_line();
  dlg_home_cursor(23, 21);
  Serial.print("Current Operation: ");
  Serial.printf("%10s", Operation);
  Serial.printf(" 0x%06X", Position);

  dlg_color_screen();
}

void dlg_draw_progress_pending(const char *Operation, uint32_t Position, uint32_t Total) {
  int x, y;

  uint32_t divisor = Total >> 8;
  uint32_t Progress = Position / divisor;

  y = 11 + (Progress >> 5);
  x = 24 + (Progress & 0x1f);
  dlg_home_cursor(y, x);
  dlg_color_pending();
  dlg_block_full();

  dlg_color_warning();
  dlg_set_flashing();
  dlg_home_cursor(21, 33);
  Serial.print("Please Wait!");
  
  dlg_home_cursor(23, 1);
  dlg_color_header();
  dlg_clear_line();
  dlg_home_cursor(23, 21);
  Serial.print("Current Operation: ");
  Serial.printf("%10s", Operation);
  Serial.printf(" 0x%06X", Position);

  dlg_color_screen();
}

void dlg_draw_error(const char *Message) {
  dlg_color_warning();
  dlg_clear_line();
  Serial.print(Message);

  dlg_color_screen();
}

////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Command Console
//
////////////////////////////////////////////////////////////////////////////////////////////////////


// command line structure
typedef struct Commands_s {
  const char *Name;
  int MinParams;
  const char *ShortHelp;
  void (*Func)(int argc, char **argv);
} Commands_t;

Commands_t GlobalCommands[] = {
  // Command      Req. Params    Help                             Handler
  { "ls",         0,             "list files",                    cmd_ls },
  { "rm",         1,             "remove file",                   cmd_unlink },

  { "init",       0,             "reinitialize LPC bus",          cmd_init },
  { "id",         0,             "print flash ID",                cmd_id },

  { "read_mem",   1,             "read byte",                     cmd_read },
  { "rd",         1,             NULL,                            cmd_read },
  { "write_mem",  2,             "write byte",                    cmd_write },
  { "wr",         2,             NULL,                            cmd_write },

  { "read_io",    1,             "read io",                       cmd_readio },
  { "ri",         1,             NULL,                            cmd_readio },
  { "write_io",   2,             "write io",                      cmd_writeio },
  { "wi",         2,             NULL,                            cmd_writeio },

  { "uart_tx",    1,             "uart tx",                       cmd_txd },
  { "txd",        1,             NULL,                            cmd_txd },

  { "isprot",     0,             "is protected?",                 cmd_isprot },
  { "unprot",     0,             "unprotect",                     cmd_unprot },

  { "dump",       1,             "dump flash",                    cmd_dump },

  { "erase",      0,             "erase chip (sector mode)",      cmd_sectorerase },
  { "berase",     0,             "erase chip (block mode)",       cmd_blockerase },

  { "flash",      1,             "write flash (sector mode)",     cmd_sectorflash },
  { "bflash",     1,             "write flash (block mode)",      cmd_blockflash },

  { "blank",      0,             "check if blank",                cmd_blank },
  { "verify",     1,             "verify flash",                  cmd_verify },

  { "sendhex",    1,             "send hexdump",                  cmd_sendhex },
//{ "sx",         1,             "send xmodem",                   cmd_sxmodem },
  { "rx",         1,             "receive xmodem",                cmd_rxmodem },
  { "help",       0,             NULL,                            cmd_help },
  { NULL,         0,             NULL,                            NULL }
};

// command line message buffer and pointer
static uint8_t msg[MAX_MSG_SIZE] = "";
static uint8_t *msg_ptr = msg;

extern uint8_t mbr_bin[];

void cmdDisplay() {
  Serial.print(">>> ");
}

/*
 * Like strtok, but with quotes
 */
char *strmbtok ( char *input, const char *delimit, const char *openblock, const char *closeblock) {
    static char *token = NULL;
    char *lead = NULL;
    char *block = NULL;
    int iBlock = 0;
    int iBlockIndex = 0;

    if ( input != NULL) {
        token = input;
        lead = input;
    }
    else {
        lead = token;
        if ( *token == '\0') {
            lead = NULL;
        }
    }

    while ( *token != '\0') {
        // We are in a quote, is this a matching close quote character?
        if ( iBlock) {
            if ( closeblock[iBlockIndex] == *token) {
                iBlock = 0;
            }
            token++;
            continue;
        }

        // Is this a valid opening quote?
        if ( ( block = strchr ( openblock, *token)) != NULL) {
            iBlock = 1;
            iBlockIndex = block - openblock;
            token++;
            continue;
        }
        if ( strchr ( delimit, *token) != NULL) {
            *token = '\0';
            token++;
            break;
        }
        token++;
    }
    return lead;
}

/*
 * Remove quotes from quoted strings.
 */
void unquote(char *out, char *in) {
  char tmp_str[256];

  if(!in) return;
  if(!out) return;

  char *pout = tmp_str;
  char inQuote = 0;
  while(*in != 0) {
    if(inQuote && (*in == inQuote)) {
      inQuote = 0;
    } else if(!inQuote && (*in == '"')) {
      inQuote = '"';
    } else if(!inQuote && (*in == '\'')) {
      inQuote = '\'';
    } else {
      *pout = *in;
      pout++;
    }
    in++;
  }
  *pout = 0;

  // This way you can have out == in
  strcpy(out, tmp_str);
}

void cmdParse(char *cmd) {
  uint8_t argc, i = 0;
  char *argv[MAXARG];

  // leading whitespace and blank lines are ignored
  while((*cmd == ' ') || (*cmd == '\t')) cmd++;
  if(!cmd[0]) return;

  // ignore labels for regular command parsing
  if(cmd[0] == ':') return;

  // parse the statement and tokenize it (with quote handling)
  argv[i] = strmbtok(cmd, " ", "\"'", "\"'");
  while ((argv[i] != NULL) && (i < MAXARG)) {
    // unquote the previous argument (last argument is always NULL)
    unquote(argv[i], argv[i]);
    argv[++i] = strmbtok(NULL, " ", "\"'", "\"'");
  };
  
  // save the number of arguments
  argc = i;

  // find the handler responsible for this command
  for(int ii=0; GlobalCommands[ii].Name; ii++) {
    if (!strcmp(argv[0], GlobalCommands[ii].Name)) {
      GlobalCommands[ii].Func(argc, argv);
      return;
    }
  }

  // command not recognized. print message and re-generate prompt.
  Serial.printf("ERROR: Unknown command '%s'.\r\n", argv[0]);
}

void cmd_handler() {
  char c = Serial.read();

  switch (c) {
    case '\n':
      break;
    case '\r':
      // terminate the msg and reset the msg ptr. then send
      // it to the handler for processing.
      *msg_ptr = '\0';
      if(msg[0] != 0) {
        Serial.print("\r\n");
        cmdParse((char *)msg);
        cmdDisplay();
      } else {
        Serial.print("\r\n");
        cmdDisplay();
      }
      msg_ptr = msg;
      break;
  
    case '\b':
      // backspace 
      if (msg_ptr > msg)
      {
          *msg_ptr = '\0';
          Serial.print(c);
          msg_ptr--;
      } else {
          Serial.print('\x07');
          msg_ptr = msg;
          msg[0] = 0;
      }
      break;
  
    default:
      // normal character entered. add it to the buffer
      if(msg_ptr < (msg + sizeof(msg) - 1)) {
        Serial.print(c);
        *msg_ptr++ = c;
        *msg_ptr = '\0';
      } else {
        Serial.print('\x07');
      }
      break;
  }
}

void cmdPoll() {
  if (Serial.available()) {
    cmd_handler();
  }
}

void cmdCommandHelp(boolean singleCommand, int cmd) {
  if(GlobalCommands[cmd].ShortHelp) {
    Serial.print("\r\n");
    Serial.print(GlobalCommands[cmd].Name);
    for(uint8_t i = 0; i < (16 - strlen(GlobalCommands[cmd].Name)); i++) {
      Serial.print(" ");
    }
    Serial.printf("%s\r\n", GlobalCommands[cmd].ShortHelp);
    if(GlobalCommands[cmd].MinParams) {
      Serial.printf("                (Requires %d %s)\r\n", GlobalCommands[cmd].MinParams, (GlobalCommands[cmd].MinParams>1) ? "Parameters" : "Parameter");
    }
    if(singleCommand)
      Serial.print("\r\n");
  }
}

void cmd_help(int argc, char **argv) {
  if(argc<2) {
    for(int ii=0; GlobalCommands[ii].Name; ii++) {
      cmdCommandHelp(false, ii);
    }
    Serial.print("\r\n");
    return;
  }
  for(int ii=0; GlobalCommands[ii].Name; ii++) {
    if(!strcmp(argv[1], GlobalCommands[ii].Name)) {
      cmdCommandHelp(true, ii);
      return;
    }
  }
}

void cmdDispatch(int argc, char **argv) {
  int ii;

  argc--;
  if(argc<1) {
    for(ii=0; GlobalCommands[ii].Name; ii++) {
      cmdCommandHelp(false, ii);
    }
    return;
  }
  for(ii=0; GlobalCommands[ii].Name; ii++) {
    if(!strcmp(argv[1], GlobalCommands[ii].Name)) {
      if(argc < GlobalCommands[ii].MinParams) {
        Serial.printf("ERROR: Not enough parameters given\r\n");
        return;
      }
      GlobalCommands[ii].Func(argc, &argv[1]);
    }
  }
}

char tmp_left[256];
char tmp_right[256];

void printDirectory(int level, uint8_t lines, const char *l, const char *r) {
  for(int pad = 0; pad < level; pad++) {
    if(lines & (1 << (level - pad))) {
      Serial.print("| ");
    } else {
      Serial.print("  ");
    }
  }
  Serial.print("o- ");
  int dots = 78 - ((level*2) + 3 + strlen(l) + strlen(r));
  Serial.print(l);
  while(dots--) {
    Serial.print(".");
  }
  Serial.print(r);
  Serial.print("\r\n");
}

void cmd_ls(int argc, char **argv) {
  File root = myfs.open("/");

  while(true) {
    File entry = root.openNextFile();

    if(! entry) {
      break;
    }

    if(!entry.isDirectory()) {
      sprintf(tmp_left, "%s ", entry.name());
      if(entry.size() >= 8192) {
        sprintf(tmp_right, " [%llu KB]", entry.size() >> 10);
      } else {
        sprintf(tmp_right, " [%llu Bytes]", entry.size());
      }
      printDirectory(1, 0, tmp_left, tmp_right);
  
    }

    entry.close();
  }

  root.close();

  Serial.printf("%d KB Free\r\n", (myfs.totalSize() - myfs.usedSize()) >> 10);
}

void cmd_dump(int argc, char **argv) {
  unsigned long addr = 0;
  unsigned long tries;
  boolean success = true;
  boolean blockok = false;

  if(argc < 2) {
    dlg_draw_error("Syntax Error, see 'help dump'.");
    return;
  }

  if(CHIP_INVALID) {
    dlg_draw_error("Error: No device selected.");
    return;
  }

  if(myfs.exists(argv[1])) {
    myfs.remove(argv[1]);
  }

  File file = myfs.open(argv[1], FILE_WRITE);
  if(!file) {
    dlg_draw_error("Error: Unable to open file for reading.");
    return;
  }

  dlg_draw_dialog("Dumping", FLASH_MEM_SIZE);

  while((addr < FLASH_MEM_SIZE) && success) {
    tries = 0;
    do {
      if(tries > MAX_RETRY) {
        success = false;
        break;
      }
      blockok = true;
      lpc_read(FLASH_MEM_BASE + addr, _bios_buffer, FLASH_SECTOR_SIZE);
      blockok &= lpc_compare(FLASH_MEM_BASE + addr, _bios_buffer, FLASH_SECTOR_SIZE, false);
      blockok &= lpc_compare(FLASH_MEM_BASE + addr, _bios_buffer, FLASH_SECTOR_SIZE, false);
      if(!blockok) {
        tries++;
      }
    } while (!blockok);
    file.write(_bios_buffer, FLASH_SECTOR_SIZE);
    addr += FLASH_SECTOR_SIZE;
  }

  file.close();

  dlg_color_screen();
  dlg_home_cursor(23,1);
  dlg_clear_line();
  dlg_home_cursor(21,1);
  dlg_clear_line();

  if(success) {
    Serial.println("OK.");
  } else {
    dlg_color_warning();
    Serial.println("Failed.");
    dlg_color_screen();
  }
}

void cmd_sectorflash(int argc, char **argv) {
  unsigned long addr = 0;
  unsigned long filesize = 0;
  unsigned long times = 1;

  if(argc < 2) {
    dlg_draw_error("Syntax Error, see 'help flash'.");
    return;
  }

  if(argc >= 3) {
    addr = strtoul(argv[2], NULL, 0);
  }

  if(argc >= 4) {
    times = strtoul(argv[3], NULL, 0);
  }

  if(CHIP_INVALID) {
    dlg_draw_error("Error: No device selected.");
    return;
  }

  dlg_draw_dialog("Flashing", FLASH_MEM_SIZE);
  
  while(times--) {
    File file = myfs.open(argv[1], FILE_READ);
    if(!file) {
      dlg_draw_error("Error: Unable to open file for reading.");
      return;
    }
    filesize = file.size();

    while((addr < FLASH_MEM_SIZE) && (filesize > 0)) {
      file.read(_bios_buffer, FLASH_SECTOR_SIZE);
      lpc_sectorflash(FLASH_MEM_BASE + addr, _bios_buffer);
      dlg_draw_progress_good("Flashing", addr, FLASH_MEM_SIZE);
      addr += FLASH_SECTOR_SIZE;
      filesize -= FLASH_SECTOR_SIZE;
    }
  
    file.close();
  }

  dlg_color_screen();
  dlg_home_cursor(23,1);
  dlg_clear_line();
  dlg_home_cursor(21,1);
  dlg_clear_line();

  Serial.println("Complete.");
}

void cmd_blockflash(int argc, char **argv) {
  unsigned long addr = 0;
  unsigned long filesize = 0;
  unsigned long times = 1;

  if(argc < 2) {
    dlg_draw_error("Syntax Error, see 'help flash'.");
    return;
  }

  if(argc >= 3) {
    addr = strtoul(argv[2], NULL, 0);
  }

  if(argc >= 4) {
    times = strtoul(argv[3], NULL, 0);
  }

  if(CHIP_INVALID) {
    dlg_draw_error("Error: No device selected.");
    return;
  }

  dlg_draw_dialog("Flashing", FLASH_MEM_SIZE);

  while(times--) {
    File file = myfs.open(argv[1], FILE_READ);
    if(!file) {
      dlg_draw_error("Error: Unable to open file for reading.");
      return;
    }
    filesize = file.size();
  
    while((addr < FLASH_MEM_SIZE) && (filesize > 0)) {
      file.read(_bios_buffer, FLASH_BLOCK_SIZE);
      lpc_blockflash(FLASH_MEM_BASE + addr, _bios_buffer);
      dlg_draw_progress_good("Flashing", addr, FLASH_MEM_SIZE);
      addr += FLASH_BLOCK_SIZE;
      filesize -= FLASH_BLOCK_SIZE;
    }
  
    file.close();
  }

  dlg_color_screen();
  dlg_home_cursor(23,1);
  dlg_clear_line();
  dlg_home_cursor(21,1);
  dlg_clear_line();

  Serial.println("Complete.");
}

void cmd_verify(int argc, char **argv) {
  unsigned long addr = 0;
  unsigned long filesize = 0;
  unsigned long times = 1;
  boolean success = true;
  boolean status;

  if(argc < 2) {
    dlg_draw_error("Syntax Error, see 'help verify'.");
    return;
  }

  if(argc >= 3) {
    addr = strtoul(argv[2], NULL, 0);
  }

  if(argc >= 4) {
    times = strtoul(argv[3], NULL, 0);
  }

  if(CHIP_INVALID) {
    dlg_draw_error("Error: No device selected.");
    return;
  }

  dlg_draw_dialog("Verifying", FLASH_MEM_SIZE);

  while(times--) {
    File file = myfs.open(argv[1], FILE_READ);
    if(!file) {
      dlg_draw_error("Error: Unable to open file for reading.");
      return;
    }
    filesize = file.size();
  
    while((addr < FLASH_MEM_SIZE) && (filesize > 0)) {
      file.read(_bios_buffer, FLASH_BLOCK_SIZE);
      status = lpc_compare(FLASH_MEM_BASE + addr, _bios_buffer, FLASH_BLOCK_SIZE, false);
      if(!status) {
        dlg_draw_progress_bad("Verifying", addr, FLASH_MEM_SIZE);
      } else {
        dlg_draw_progress_good("Verifying", addr, FLASH_MEM_SIZE);
      }
      success &= status;
      addr += FLASH_BLOCK_SIZE;
      filesize -= FLASH_BLOCK_SIZE;
    }
  
    file.close();
  }

  dlg_color_screen();
  dlg_home_cursor(23,1);
  dlg_clear_line();
  dlg_home_cursor(21,1);
  dlg_clear_line();

  if(success) {
    Serial.println("Device matches file.");
  } else {
    dlg_color_warning();
    Serial.println("Device does not match file.");
    dlg_color_screen();
  }
}

void cmd_sectorerase(int argc, char **argv) {
  if(CHIP_INVALID) {
    dlg_draw_error("Error: No device selected.");
    return;
  }

  dlg_draw_dialog("Erasing", FLASH_MEM_SIZE);

  boolean success = lpc_chipsectorerase(FLASH_MEM_BASE);

  dlg_color_screen();
  dlg_home_cursor(23,1);
  dlg_clear_line();
  dlg_home_cursor(21,1);
  dlg_clear_line();

  if(success) {
    Serial.println("OK.");
  } else {
    dlg_color_warning();
    Serial.println("Failed.");
    dlg_color_screen();
  }
}

void cmd_blockerase(int argc, char **argv) {
  if(CHIP_INVALID) {
    dlg_draw_error("Error: No device selected.");
    return;
  }

  dlg_draw_dialog("Erasing", FLASH_MEM_SIZE);

  boolean success = lpc_chipblockerase(FLASH_MEM_BASE);

  dlg_color_screen();
  dlg_home_cursor(23,1);
  dlg_clear_line();
  dlg_home_cursor(21,1);
  dlg_clear_line();

  if(success) {
    Serial.println("OK.");
  } else {
    dlg_color_warning();
    Serial.println("Failed.");
    dlg_color_screen();
  }
}

void cmd_unprot(int argc, char **argv) {
  lpc_unprotect();
}

void cmd_isprot(int argc, char **argv) {
  lpc_isprotect();
}

void cmd_blank(int argc, char **argv) {
  if(CHIP_INVALID) {
    dlg_draw_error("Error: No device selected.");
    return;
  }

  dlg_draw_dialog("Checking", FLASH_MEM_SIZE);

  boolean success = lpc_blank(FLASH_MEM_BASE, FLASH_MEM_SIZE, false);

  dlg_color_screen();
  dlg_home_cursor(23,1);
  dlg_clear_line();
  dlg_home_cursor(21,1);
  dlg_clear_line();

  if(success) {
    Serial.println("Device is blank.");
  } else {
    dlg_color_warning();
    Serial.println("Device is not blank.");
    dlg_color_screen();
  }
}

void cmd_writeio(int argc, char **argv) {
  unsigned long addr;
  unsigned char data;

  if(argc < 3) {
    dlg_draw_error("Syntax Error, see 'help write_io'.");
    return;
  }

  addr = strtoul(argv[1], NULL, 0);
  data = strtoul(argv[2], NULL, 0);

  lpc_writeiobyte(addr, data);

  Serial.printf("0x%04x: 0x%02x\r\n", addr, data);
}

void cmd_readio(int argc, char **argv) {
  unsigned long addr;

  if(argc < 2) {
    dlg_draw_error("Syntax Error, see 'help read_io'.");
    return;
  }

  addr = strtoul(argv[1], NULL, 0);
  Serial.printf("0x%04x: 0x%02x\r\n", addr, lpc_readiobyte(addr));
}

void cmd_txd(int argc, char **argv) {
  unsigned char c;

  if(argc < 2) {
    dlg_draw_error("Syntax Error, see 'help uart_tx'.");
    return;
  }

  c = argv[1][0];
  while(lpc_readiobyte(0x3fd) == 0);
  lpc_writeiobyte(0x3f8, c);
  while(lpc_readiobyte(0x3fd) == 0);
}


void cmd_write(int argc, char **argv) {
  unsigned long addr;
  unsigned char data;

  if(argc < 3) {
    dlg_draw_error("Syntax Error, see 'help write_mem'.");
    return;
  }

  addr = strtoul(argv[1], NULL, 0);
  data = strtoul(argv[2], NULL, 0);

  lpc_writemembyte(addr, data);

  Serial.printf("0x%08x: 0x%02x\r\n", addr, data);
}

void cmd_read(int argc, char **argv) {
  unsigned long addr;

  if(argc < 2) {
    dlg_draw_error("Syntax Error, see 'help read_mem'.");
    return;
  }

  addr = strtoul(argv[1], NULL, 0);
  Serial.printf("0x%08x: 0x%02x\r\n", addr, lpc_readmembyte(addr));
}

void cmd_sendhex(int argc, char **argv) {
  unsigned long base, addr;

  if(argc < 2) {
    dlg_draw_error("Syntax Error, see 'help send_hex'.");
    return;
  }

  File file = myfs.open(argv[1], FILE_READ);
  if(!file) {
    dlg_draw_error("Error: Unable to open file for reading.");
    return;
  }

  for(base = 0; base < FLASH_MEM_SIZE; base += FLASH_BLOCK_SIZE) {
    file.read(_bios_buffer, FLASH_BLOCK_SIZE);
    for(addr = 0; addr < FLASH_BLOCK_SIZE; addr++) {
      if((addr & 0xf) == 0)
        Serial.printf("%08x: ", base + addr);

      Serial.printf("%02x ", _bios_buffer[addr]);

      if((addr & 0xf) == 0xf)
        Serial.println();
    }
  }

  Serial.println();

  file.close();
}

void cmd_id(int argc, char **argv) {
  if(argc >= 2) {
    lpc_getid(strtoul(argv[1], NULL, 0));
  } else {
    lpc_getid(1);
  }
}

void cmd_init(int argc, char **argv) {
  lpc_init();
}

void cmd_unlink(int argc, char **argv) {
  if(argc < 2) {
    dlg_draw_error("Syntax Error, see 'help rm'.");
    return;
  }

  if(myfs.exists(argv[1])) {
    myfs.remove(argv[1]);
  } else {
    dlg_draw_error("Error: Unable to delete file.");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  XModem Protocol
//
////////////////////////////////////////////////////////////////////////////////////////////////////

// xmodem control characters
#define SOH     0x01
#define STX     0x02
#define EOT     0x04
#define ACK     0x06
#define NAK     0x15
#define CAN     0x18
#define CTRLZ   0x1A

// xmodem timeout/retry parameters
#define XMODEM_TIMEOUT_DELAY  1000
#define XMODEM_RETRY_LIMIT    16

// error return codes
#define XMODEM_ERROR_REMOTECANCEL -1
#define XMODEM_ERROR_OUTOFSYNC    -2
#define XMODEM_ERROR_RETRYEXCEED  -3

//#define XMODEM_BUFFER_SIZE    128
#define XMODEM_BUFFER_SIZE    1024

long xmodemReceive(File file)
{
  // create xmodem buffer
  // 1024b for Xmodem 1K
  // 128 bytes for Xmodem std.
  // + 5b header/crc + NULL
  unsigned char xmbuf[XMODEM_BUFFER_SIZE+6];
  unsigned char seqnum=1;   // xmodem sequence number starts at 1
  unsigned short pktsize=128; // default packet size is 128 bytes
  unsigned char response='C'; // solicit a connection with CRC enabled
  char retry=XMODEM_RETRY_LIMIT;
  unsigned char crcflag=0;
  unsigned long totalbytes=0;
  int i,c;

  while(retry > 0)
  {
    // solicit a connection/packet
    Serial.write(response);
    // wait for start of packet
    if( (c = xmodemInTime(XMODEM_TIMEOUT_DELAY)) >= 0)
    {
      switch(c)
      {
      case SOH:
        pktsize = 128;
        break;
      #if(XMODEM_BUFFER_SIZE>=1024)
      case STX:
        pktsize = 1024;
        break;
      #endif
      case EOT:
        xmodemInFlush();
        Serial.write(ACK);
        // completed transmission normally
        return totalbytes;
      case CAN:
        if((c = xmodemInTime(XMODEM_TIMEOUT_DELAY)) == CAN)
        {
          xmodemInFlush();
          Serial.write(ACK);
          // transaction cancelled by remote node
          return XMODEM_ERROR_REMOTECANCEL;
        }
      default:
        break;
      }
    }
    else
    {
      // timed out, try again
      // no need to flush because receive buffer is already empty
      retry--;
      //response = NAK;
      continue;
    }

    // check if CRC mode was accepted
    if(response == 'C') crcflag = 1;
    // got SOH/STX, add it to processing buffer
    xmbuf[0] = c;
    // try to get rest of packet
    for(i=0; i<(pktsize+crcflag+4-1); i++)
    {
      if((c = xmodemInTime(XMODEM_TIMEOUT_DELAY)) >= 0)
      {
        xmbuf[1+i] = c;
      }
      else
      {
        // timed out, try again
        retry--;
        xmodemInFlush();
        response = NAK;
        break;
      }
    }
    // packet was too small, retry
    if(i<(pktsize+crcflag+4-1))
      continue;

    // got whole packet
    // check validity of packet
    if( (xmbuf[1] == (unsigned char)(~xmbuf[2])) &&   // sequence number was transmitted w/o error
      xmodemCrcCheck(crcflag, &xmbuf[3], pktsize) ) // packet is not corrupt
    {
      // is this the packet we were waiting for?
      if(xmbuf[1] == seqnum)
      {
        // write/deliver data
        file.write(&xmbuf[3], pktsize);

        totalbytes += pktsize;
        // next sequence number
        seqnum++;
        // reset retries
        retry = XMODEM_RETRY_LIMIT;
        // reply with ACK
        response = ACK;
        continue;
      }
      else if(xmbuf[1] == (unsigned char)(seqnum-1))
      {
        // this is a retransmission of the last packet
        // ACK and move on
        response = ACK;
        continue;
      }
      else
      {
        // we are completely out of sync
        // cancel transmission
        xmodemInFlush();
        Serial.write(CAN);
        Serial.write(CAN);
        Serial.write(CAN);
        return XMODEM_ERROR_OUTOFSYNC;
      }
    }
    else
    {
      // packet was corrupt
      // NAK it and try again
      retry--;
      xmodemInFlush();
      response = NAK;
      continue;
    }
  }

  // exceeded retry count
  xmodemInFlush();
  Serial.write(CAN);
  Serial.write(CAN);
  Serial.write(CAN);
  return XMODEM_ERROR_RETRYEXCEED;
}


long xmodemTransmit(File file)
{
  // still to be written
  return 0;
}

uint16_t crc_xmodem_update(uint16_t crc, uint8_t data)
{
  int i;

  crc = crc ^ ((uint16_t)data << 8);
  for (i=0; i<8; i++)
  {
    if(crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }

  return crc;
}

int xmodemCrcCheck(int crcflag, const unsigned char *buffer, int size)
{
  // crcflag=0 - do regular checksum
  // crcflag=1 - do CRC checksum

  if(crcflag)
  {
    unsigned short crc=0;
    unsigned short pktcrc = (buffer[size]<<8)+buffer[size+1];
    // do CRC checksum
    while(size--)
      crc = crc_xmodem_update(crc, *buffer++);
    // check checksum against packet
    if(crc == pktcrc)
      return 1;
  }
  else
  {
    int i;
    unsigned char cksum = 0;
    // do regular checksum
    for(i=0; i<size; ++i)
    {
      cksum += buffer[i];
    }
    // check checksum against packet
    if(cksum == buffer[size])
      return 1;
  }

  return 0;
}


int xmodemInTime(unsigned short timeout)
{
  int c=-1;

  while( (timeout--) && ((c=Serial.read()) < 0) )
    delay(1);

  return c;
}

void xmodemInFlush(void)
{
  while(xmodemInTime(XMODEM_TIMEOUT_DELAY) >= 0);
}

void cmd_rxmodem(int argc, char **argv) {
  delay(3000);

  if(argc < 2) {
    Serial.println("SYNTAX ERROR");
    return;
  }

  if(myfs.exists(argv[1])) {
    myfs.remove(argv[1]);
  }

  File file = myfs.open(argv[1], FILE_WRITE);
  if(!file) {
    Serial.println("FILE ERROR");
    return;
  }

  Serial.println("Transmit file now.");

  if(xmodemReceive(file) > 0) {
    Serial.println("OK.");
  } else {
    Serial.println("Failed.");
  }

  file.close();
}
////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Main
//
////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  lpc_init();
  lpc_abort();

  myfs.begin(1792 * 1024); // 1.75MB

  strcpy(m_ChipType, "Unknown");
  strcpy(m_FlashType, "Unknown");
  strcpy(m_FlashDetails, "Run the 'id' command first.");

  if(Serial) {
    USBAttached = true;

    dlg_draw_header();
    cmdDisplay();
  }
}

void loop() {
  if (!Serial) {
    USBAttached = false;
  }
  if (!USBAttached && Serial) {
    USBAttached = true;
    msg_ptr = msg;
    msg[0] = 0;
    dlg_draw_header();
    cmdDisplay();
  }
  cmdPoll();
}
