//----------------------------------------------------------------------------
//
//  File generated by S1D13506CFG.EXE
//
//  Copyright (c) 2000,2001 Epson Research and Development, Inc.
//  All rights reserved.
//
//  PLEASE NOTE: If you FTP this file to a non-Windows platform, make
//               sure you transfer this file using ASCII, not BINARY mode.
//
//  This file was tested using SDU1356B0C REV. 1.0 Board with a CRT monitor
//  attached. 
//
//  Dip switch settings: Closed: 1,2,3,4,6,10  Open: 5,7,8,9
//  Jumpers: JP1:1-2, JP2:1-2, JP3: All out, JP4:1-2, JP5:Out, JP6:1-2
//           JP7:1-2, JP8:1-2, JP9: Out
//  BEM: Rev 1.1
//----------------------------------------------------------------------------

// CRT:    (active)   640x480 60Hz (PCLK=CLKI2=25.175MHz)
// Memory: 50ns EDO-DRAM 2-CAS#: 8ms refresh (MCLK=BUSCLK=33.333MHz) 


#define S1D_DISPLAY_WIDTH           640
#define S1D_DISPLAY_HEIGHT          480
#define S1D_DISPLAY_BPP             16
#define S1D_DISPLAY_SCANLINE_BYTES  1280
#define S1D_PHYSICAL_VMEM_ADDR      0x00000000L
#define S1D_PHYSICAL_VMEM_SIZE      0x200000L
#define S1D_PHYSICAL_REG_ADDR       0x00000000L
#define S1D_PHYSICAL_REG_SIZE       0x200
#define S1D_DISPLAY_PCLK            40000
#define S1D_PALETTE_SIZE            256
#define S1D_FRAME_RATE              60
#define S1D_POWER_DELAY_ON          50
#define S1D_POWER_DELAY_OFF         1200
#define S1D_CRT
#define S1D_HWBLT
#define S1D_REGDELAYOFF             0xFFFE
#define S1D_REGDELAYON              0xFFFF

#define S1D_WRITE_PALETTE(p,i,r,g,b)  \
{  \
    ((volatile S1D_VALUE*)(p))[0x1E2/sizeof(S1D_VALUE)] = (S1D_VALUE)(i);  \
    ((volatile S1D_VALUE*)(p))[0x1E4/sizeof(S1D_VALUE)] = (S1D_VALUE)(r);  \
    ((volatile S1D_VALUE*)(p))[0x1E4/sizeof(S1D_VALUE)] = (S1D_VALUE)(g);  \
    ((volatile S1D_VALUE*)(p))[0x1E4/sizeof(S1D_VALUE)] = (S1D_VALUE)(b);  \
}

#define S1D_READ_PALETTE(p,i,r,g,b)  \
{  \
    ((volatile S1D_VALUE*)(p))[0x1E2/sizeof(S1D_VALUE)] = (S1D_VALUE)(i);  \
    r = ((volatile S1D_VALUE*)(p))[0x1E4/sizeof(S1D_VALUE)];  \
    g = ((volatile S1D_VALUE*)(p))[0x1E4/sizeof(S1D_VALUE)];  \
    b = ((volatile S1D_VALUE*)(p))[0x1E4/sizeof(S1D_VALUE)];  \
}

typedef unsigned short S1D_INDEX;
typedef unsigned char  S1D_VALUE;

typedef struct
{
    S1D_INDEX Index;
    S1D_VALUE Value;
} S1D_REGS;

static S1D_REGS aS1DRegs[] = 
{
    {0x0001,0x00},   // Miscellaneous Register
    {0x01FC,0x00},   // Display Mode Register
    {0x0004,0x00},   // General IO Pins Configuration Register
    {0x0008,0x00},   // General IO Pins Control Register
    {0x0010,0x01},   // Memory Clock Configuration Register
    {0x0014,0x00},   // LCD Pixel Clock Configuration Register
    {0x0018,0x02},   // CRT/TV Pixel Clock Configuration Register
    {0x001C,0x02},   // MediaPlug Clock Configuration Register
    {0x001E,0x01},   // CPU To Memory Wait State Select Register
    {0x0020,0x00},   // Memory Configuration Register
    {0x0021,0x04},   // DRAM Refresh Rate Register
    {0x002A,0x12},   // DRAM Timings Control Register 0
    {0x002B,0x02},   // DRAM Timings Control Register 1
    {0x0030,0x25},   // Panel Type Register
    {0x0031,0x00},   // MOD Rate Register
    {0x0032,0x4F},   // LCD Horizontal Display Width Register
    {0x0034,0x12},   // LCD Horizontal Non-Display Period Register
    {0x0035,0x01},   // TFT FPLINE Start Position Register
    {0x0036,0x0B},   // TFT FPLINE Pulse Width Register
    {0x0038,0xDF},   // LCD Vertical Display Height Register 0
    {0x0039,0x01},   // LCD Vertical Display Height Register 1
    {0x003A,0x2C},   // LCD Vertical Non-Display Period Register
    {0x003B,0x0A},   // TFT FPFRAME Start Position Register
    {0x003C,0x01},   // TFT FPFRAME Pulse Width Register
    {0x0040,0x03},   // LCD Display Mode Register
    {0x0041,0x00},   // LCD Miscellaneous Register
    {0x0042,0x00},   // LCD Display Start Address Register 0
    {0x0043,0x00},   // LCD Display Start Address Register 1
    {0x0044,0x00},   // LCD Display Start Address Register 2
    {0x0046,0x40},   // LCD Memory Address Offset Register 0
    {0x0047,0x01},   // LCD Memory Address Offset Register 1
    {0x0048,0x00},   // LCD Pixel Panning Register
    {0x004A,0x00},   // LCD Display FIFO High Threshold Control Register
    {0x004B,0x00},   // LCD Display FIFO Low Threshold Control Register
    {0x0050,0x4F},   // CRT/TV Horizontal Display Width Register
    {0x0052,0x13},   // CRT/TV Horizontal Non-Display Period Register
    {0x0053,0x01},   // CRT/TV HRTC Start Position Register
    {0x0054,0x0B},   // CRT/TV HRTC Pulse Width Register
    {0x0056,0xDF},   // CRT/TV Vertical Display Height Register 0
    {0x0057,0x01},   // CRT/TV Vertical Display Height Register 1
    {0x0058,0x2B},   // CRT/TV Vertical Non-Display Period Register
    {0x0059,0x09},   // CRT/TV VRTC Start Position Register
    {0x005A,0x01},   // CRT/TV VRTC Pulse Width Register
    {0x005B,0x18},   // TV Output Control Register
    {0x0060,0x05},   // CRT/TV Display Mode Register
    {0x0062,0x00},   // CRT/TV Display Start Address Register 0
    {0x0063,0x00},   // CRT/TV Display Start Address Register 1
    {0x0064,0x00},   // CRT/TV Display Start Address Register 2
    {0x0066,0x80},   // CRT/TV Memory Address Offset Register 0
    {0x0067,0x02},   // CRT/TV Memory Address Offset Register 1
    {0x0068,0x00},   // CRT/TV Pixel Panning Register
    {0x006A,0x00},   // CRT/TV Display FIFO High Threshold Control Register
    {0x006B,0x00},   // CRT/TV Display FIFO Low Threshold Control Register
    {0x0070,0x00},   // LCD Ink/Cursor Control Register
    {0x0071,0x01},   // LCD Ink/Cursor Start Address Register
    {0x0072,0x00},   // LCD Cursor X Position Register 0
    {0x0073,0x00},   // LCD Cursor X Position Register 1
    {0x0074,0x00},   // LCD Cursor Y Position Register 0
    {0x0075,0x00},   // LCD Cursor Y Position Register 1
    {0x0076,0x00},   // LCD Ink/Cursor Blue Color 0 Register
    {0x0077,0x00},   // LCD Ink/Cursor Green Color 0 Register
    {0x0078,0x00},   // LCD Ink/Cursor Red Color 0 Register
    {0x007A,0x1F},   // LCD Ink/Cursor Blue Color 1 Register
    {0x007B,0x3F},   // LCD Ink/Cursor Green Color 1 Register
    {0x007C,0x1F},   // LCD Ink/Cursor Red Color 1 Register
    {0x007E,0x00},   // LCD Ink/Cursor FIFO Threshold Register
    {0x0080,0x00},   // CRT/TV Ink/Cursor Control Register
    {0x0081,0x01},   // CRT/TV Ink/Cursor Start Address Register
    {0x0082,0x00},   // CRT/TV Cursor X Position Register 0
    {0x0083,0x00},   // CRT/TV Cursor X Position Register 1
    {0x0084,0x00},   // CRT/TV Cursor Y Position Register 0
    {0x0085,0x00},   // CRT/TV Cursor Y Position Register 1
    {0x0086,0x00},   // CRT/TV Ink/Cursor Blue Color 0 Register
    {0x0087,0x00},   // CRT/TV Ink/Cursor Green Color 0 Register
    {0x0088,0x00},   // CRT/TV Ink/Cursor Red Color 0 Register
    {0x008A,0x1F},   // CRT/TV Ink/Cursor Blue Color 1 Register
    {0x008B,0x3F},   // CRT/TV Ink/Cursor Green Color 1 Register
    {0x008C,0x1F},   // CRT/TV Ink/Cursor Red Color 1 Register
    {0x008E,0x00},   // CRT/TV Ink/Cursor FIFO Threshold Register
    {0x0100,0x00},   // BitBlt Control Register 0
    {0x0101,0x00},   // BitBlt Control Register 1
    {0x0102,0x00},   // BitBlt ROP Code/Color Expansion Register
    {0x0103,0x00},   // BitBlt Operation Register
    {0x0104,0x00},   // BitBlt Source Start Address Register 0
    {0x0105,0x00},   // BitBlt Source Start Address Register 1
    {0x0106,0x00},   // BitBlt Source Start Address Register 2
    {0x0108,0x00},   // BitBlt Destination Start Address Register 0
    {0x0109,0x00},   // BitBlt Destination Start Address Register 1
    {0x010A,0x00},   // BitBlt Destination Start Address Register 2
    {0x010C,0x00},   // BitBlt Memory Address Offset Register 0
    {0x010D,0x00},   // BitBlt Memory Address Offset Register 1
    {0x0110,0x00},   // BitBlt Width Register 0
    {0x0111,0x00},   // BitBlt Width Register 1
    {0x0112,0x00},   // BitBlt Height Register 0
    {0x0113,0x00},   // BitBlt Height Register 1
    {0x0114,0x00},   // BitBlt Background Color Register 0
    {0x0115,0x00},   // BitBlt Background Color Register 1
    {0x0118,0x00},   // BitBlt Foreground Color Register 0
    {0x0119,0x00},   // BitBlt Foreground Color Register 1
    {0x01E0,0x00},   // Look-Up Table Mode Register
    {0x01E2,0x00},   // Look-Up Table Address Register
    {0x01F0,0x00},   // Power Save Configuration Register
    {0x01F1,0x00},   // Power Save Status Register
    {0x01F4,0x00},   // CPU-to-Memory Access Watchdog Timer Register
    {0x01FC,0x02},   // Display Mode Register
};

