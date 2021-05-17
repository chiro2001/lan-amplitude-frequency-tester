#ifndef _AD9833_H_
#define _AD9833_H_

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
// #include "sys.h"
#define AD9833_FSYNC PBout(12)  //这个是NSS管脚
/******************************************************************************/
/* AD9833                                                                    */
/******************************************************************************/
/* Registers */

#define AD9833_REG_CMD (0 << 14)
#define AD9833_REG_FREQ0 (1 << 14)
#define AD9833_REG_FREQ1 (2 << 14)
#define AD9833_REG_PHASE0 (6 << 13)
#define AD9833_REG_PHASE1 (7 << 13)

/* Command Control Bits */

#define AD9833_B28 (1 << 13)
#define AD9833_HLB (1 << 12)
#define AD9833_FSEL0 (0 << 11)
#define AD9833_FSEL1 (1 << 11)
#define AD9833_PSEL0 (0 << 10)
#define AD9833_PSEL1 (1 << 10)
#define AD9833_PIN_SW (1 << 9)
#define AD9833_RESET (1 << 8)
#define AD9833_SLEEP1 (1 << 7)
#define AD9833_SLEEP12 (1 << 6)
#define AD9833_OPBITEN (1 << 5)
#define AD9833_SIGN_PIB (1 << 4)
#define AD9833_DIV2 (1 << 3)
#define AD9833_MODE (1 << 1)

#define AD9833_OUT_SINUS ((0 << 5) | (0 << 1) | (0 << 3))
#define AD9833_OUT_TRIANGLE ((0 << 5) | (1 << 1) | (0 << 3))
#define AD9833_OUT_MSB ((1 << 5) | (0 << 1) | (1 << 3))
#define AD9833_OUT_MSB2 ((1 << 5) | (0 << 1) | (0 << 3))
/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Initializes the SPI communication peripheral and resets the part. */
unsigned char AD9833_Init(void);
/* Sets the Reset bit of the AD9833. */
void AD9833_Reset(void);
/* Clears the Reset bit of the AD9833. */
void AD9833_ClearReset(void);
/* Writes the value to a register. */
void AD9833_SetRegisterValue(unsigned short regValue);
/* Writes to the frequency registers. */
void AD9833_SetFrequency(unsigned short reg, float fout);
/* Writes to the phase registers. */
void AD9833_SetPhase(unsigned short reg, unsigned short val);
/* Selects the Frequency,Phase and Waveform type. */
void AD9833_Setup(unsigned short freq, unsigned short phase,
                  unsigned short type);
void AD9833_SetFrequencyQuick(float fout, unsigned short type);

unsigned char AD9833_SPI_Write(unsigned char* data, unsigned char bytesNumber);
#endif  // _AD9833_H
