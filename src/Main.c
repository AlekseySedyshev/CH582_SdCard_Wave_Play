#include "CH58x_common.h"
#include "diskio.h"

#define FREQ_SYS 60000000   //Sys frequency 60MHz
#define FCC(c1, c2, c3, c4) (((DWORD)c4 << 24) + ((DWORD)c3 << 16) + ((WORD)c2 << 8) + (BYTE)c1) /* FourCC */
#define BUF_SIZE 512
typedef enum
{
    BUFF,
    STREAM
} play_mode;
play_mode pl;

FATFS fatfs; /* File system object */
DIR dir;     /* Directory object */
FILINFO fno; /* File information object */
UINT bw, br, tmp_br;
BYTE sd_buf[BUF_SIZE];

volatile BYTE new_sample = 0;
FRESULT rc;
uint8_t i;
uint16_t led_timer;
typedef enum
{
    BUF_LOW,
    BUF_HI,
    BUF_STOP
} state_t;
static state_t st = 0;

__attribute__((aligned(4))) UINT8 spiBuff[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
__attribute__((aligned(4))) UINT8 spiBuffrev[16];


static WORD LD_WORD(const BYTE *ptr)    //Load a 2-byte little-endian word
{
    WORD rv;
    rv = ptr[1];
    rv = rv << 8 | ptr[0];
    return rv;
}
static DWORD LD_DWORD(const BYTE *ptr) // Load a 4-byte little-endian word
{
    DWORD rv;

    rv = ptr[3];
    rv = rv << 8 | ptr[2];
    rv = rv << 8 | ptr[1];
    rv = rv << 8 | ptr[0];
    return rv;
}


void DebugInit(void)
{
    GPIOA_SetBits(GPIO_Pin_9);
    GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
}

static DWORD load_head(void)
{
    DWORD fcc, sz;
    UINT i;
    FRESULT res;

    res = pf_read(sd_buf, 256, &br); /* Read a chunk of file */
    if (res || !br)
        return res; /* Error or end of file */
    if (LD_DWORD(sd_buf + 8) != FCC('W', 'A', 'V', 'E'))
        return 12;

    i = 12;
    while (i < 200) // while (i < 200)
    {
        fcc = LD_DWORD(&sd_buf[i]);    /* FCC */
        sz = LD_DWORD(&sd_buf[i + 4]); /* Chunk size */
        i += 8;
        switch (fcc)
        {

        case FCC('f', 'm', 't', ' '): /* 'fmt ' chunk */
            if (sz > 100 || sz < 16)  /* Check chunk size */
                return 10;
            //  if (sd_buf[i + 0] != 1) /* Check coding type (1) */
            //      return 11;
            if (sd_buf[i + 2] != 1 && sd_buf[i + 2] != 2) /* Check channels (1/2) */
                return 11;
            //  TEMP_PTR = sd_buf[i + 2];                        /* Channel flag */
            if (sd_buf[i + 14] != 8 && sd_buf[i + 14] != 16) /* Check resolution (8/16) */
                return 11;
            //  TEMP_PTR |= sd_buf[i + 14];                                   /* Resolution flag */
            uint32_t ttt = (uint32_t)(FREQ_SYS / LD_WORD(&sd_buf[i + 4])) - 1; // Sampling freq */
            R32_TMR0_CNT_END = ttt;

            break;

        case FCC('f', 'a', 'c', 't'): /* 'fact' chunk (skip) */
            break;

        case FCC('d', 'a', 't', 'a'): /* 'data' chunk (start to play) */
            fatfs.fptr = i;
            return sz;

        default: /* Unknown chunk (error) */
            return 14;
        }
        i += sz;
    }

    return 15;
}
static FRESULT play(
    const char *fn)
{
    DWORD sz;
    FRESULT res;

    if ((res = pf_open(fn)) == FR_OK)
    {
        sz = load_head(); /* Load file header */
                          //        if (sz < 63)  //            return 99;

        if (pl == STREAM)
            res = pf_read(0, BUF_SIZE - fatfs.fptr, &br);
        else
            res = pf_read(sd_buf, (BUF_SIZE / 2) - fatfs.fptr, &br);
        sz -= br;

        R32_TMR0_COUNT=0;
        R8_TMR0_CTRL_MOD = RB_TMR_COUNT_EN;
        R8_PWM6_DATA = 0;
        R8_PWM_OUT_EN |= CH_PWM6;

        if (pl == STREAM)
        {
            do
            {
                bw = (sz > BUF_SIZE) ? BUF_SIZE : (WORD)sz;
                rc = pf_read(0, bw, &br);
                sz -= br;
                if (rc != FR_OK || bw != br)
                    break;
            } while (br == BUF_SIZE);
        }
        fno.fsize = sz;
        st = BUF_LOW;
    }
    return res;
}

int main()
{
    SetSysClock(CLK_SOURCE_PLL_60MHz);

  //  SysTick->CMP = FREQ_SYS/60000;
  //  SysTick->CTLR = SysTick_CTLR_INIT | SysTick_CTLR_STRE | SysTick_CTLR_STCLK | SysTick_CTLR_STIE | SysTick_CTLR_STE;
  //  PFIC_EnableIRQ(SysTick_IRQn);

    DebugInit();
    GPIOB_SetBits(GPIO_Pin_18 | GPIO_Pin_19);
    GPIOB_ModeCfg(GPIO_Pin_18 | GPIO_Pin_19,GPIO_ModeOut_PP_5mA);

    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
   // ------------------SPI INIT---------------------------
    //PA15 - MISO, PA14 - MOSI, PA13 - SCK, PA12 - CS, PB16 - CARD ON SLOT
   GPIOA_SetBits(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);
   GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
   GPIOA_ModeCfg(GPIO_Pin_13 | GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
   GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeOut_PP_20mA);
   R8_SPI0_CLOCK_DIV = 5;
   R8_SPI0_CTRL_MOD = RB_SPI_ALL_CLEAR;
   R8_SPI0_CTRL_MOD = RB_SPI_MOSI_OE | RB_SPI_SCK_OE | RB_SPI_MISO_OE;
   R8_SPI0_CTRL_CFG |= RB_SPI_AUTO_IF;
   //R8_SPI0_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;

   //-------------------PWM--------------------------------
   GPIOB_ModeCfg(GPIO_Pin_0, GPIO_ModeOut_PP_5mA);  // PB0 - PWM6
   R8_PWM_CLOCK_DIV = 1; // FCPU /10 = 60 MHz
   //R8_PWM_CONFIG = R8_PWM_CONFIG & 0xf0; //256
   R8_PWM_CONFIG = (R8_PWM_CONFIG & 0xf0) | 0x01;//255
   //R8_PWM6_DATA = 127;
   //R8_PWM_OUT_EN |= CH_PWM6;

   //--------------------Timer0--------------------------------

   R32_TMR0_CNT_END =FREQ_SYS / 22050;
   R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
   R8_TMR0_CTRL_MOD = RB_TMR_COUNT_EN;
   R8_TMR0_INTER_EN |= TMR0_3_IT_CYC_END;
   PFIC_EnableIRQ(TMR0_IRQn);
/*
   //--------------- TMR3 - PA2 - PWM_MODE ----------------
   //счет импульсов от  timer0
   //GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeOut_PP_5mA);  // PA2 - PWM

   R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;
   //R8_TMR3_CTRL_MOD = RB_TMR_OUT_EN & (~RB_TMR_PWM_REPEAT ) & (~RB_TMR_OUT_POLAR); //Repeat once

   R8_TMR3_CTRL_MOD = RB_TMR_CAP_EDGE | RB_TMR_CAP_COUNT ; //Capture Rise to Rise edge
   R32_TMR3_CNT_END =255;
   R32_TMR3_FIFO = 127;
   R8_TMR3_INTER_EN |= RB_TMR_IE_DATA_ACT;
   R8_TMR3_CTRL_MOD |= RB_TMR_COUNT_EN;
   PFIC_EnableIRQ(TMR3_IRQn);

   R8_TMR3_CTRL_MOD = RB_TMR_COUNT_EN;
   R8_TMR3_INTER_EN |= TMR0_3_IT_CYC_END;
*/

   pl = BUFF;
   //pl = STREAM;

   rc = pf_mount(&fatfs);
       if (rc)
           return 0;

       rc = play("W.WAV");
       if (rc)
           return 0;



 while(1)
    {
 /*
     R32_PB_OUT ^= GPIO_Pin_18 ^ GPIO_Pin_19;
     for (i=0;i<255;i++)
     {
         DelayMs(10);
         R8_PWM6_DATA = i;
     }
     R32_PB_OUT ^= GPIO_Pin_18 ^ GPIO_Pin_19;
     for (i=0;i<255;i++)
         {
             DelayMs(10);
             R8_PWM6_DATA = 255-i;
         }
*/
     switch (st)
             {
             case BUF_LOW:
                 if (tmp_br < BUF_SIZE / 2)
                 {
                     bw = (fno.fsize > BUF_SIZE / 2) ? BUF_SIZE / 2 : (WORD)fno.fsize;
                     rc = pf_read(&sd_buf[BUF_SIZE / 2], bw, &br);
                     fno.fsize -= br;
                     new_sample = 1;
                     if (rc != FR_OK || bw != br)
                     {
                         st = BUF_STOP;
                         break;
                     }
                     else
                         st = BUF_HI;
                 }
                 if (tmp_br >= BUF_SIZE)
                     tmp_br = 0;
                 break;
             case BUF_HI:

                 if (tmp_br >= BUF_SIZE / 2)
                 {
                     bw = (fno.fsize > BUF_SIZE / 2) ? BUF_SIZE / 2 : (WORD)fno.fsize;
                     rc = pf_read(sd_buf, bw, &br);
                     fno.fsize -= br;
                     new_sample = 1;
                     if (rc != FR_OK || bw != br)
                     {
                         st = BUF_STOP;
                         break;
                     }
                     else
                         st = BUF_LOW;
                 }
                 break;
             case BUF_STOP:
             default:
                 R8_PWM_OUT_EN &=(~CH_PWM6);
                 R8_TMR0_CTRL_MOD  &=(~RB_TMR_COUNT_EN);
                 break;
             }


    }//End While
while(1); // <-- Заглушка
}// END main
//----------------------------------------IRQ HANDLERS----------------------------

//------------------TMR0----------------------------------
__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler(void)
{
        TMR0_ClearITFlag(TMR0_3_IT_CYC_END);

        if (pl == BUFF)
          {
            R8_PWM6_DATA = sd_buf[tmp_br];
            tmp_br++;
            new_sample = 0;
          }
        if (pl == STREAM)
              new_sample = 1;


}
//SysTick_IRQn
__INTERRUPT
__HIGH_CODE
void SysTick_IRQHandler(void)
{
    SysTick->SR &=(~SysTick_SR_CNTIF);
    if(led_timer)led_timer--;
    else {
        led_timer=500;
        R32_PB_OUT ^= GPIO_Pin_18 ^ GPIO_Pin_19;
    }
}


//--------------- UART1_IRQHandler------------------------
__INTERRUPT
__HIGH_CODE
void UART1_IRQHandler(void)
{

}
