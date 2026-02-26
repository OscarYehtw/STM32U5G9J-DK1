/*----------------------------------------------------------------------------
 *      R T L   F l a s h   F i l e   S y s t e m   E x a m p l e
 *----------------------------------------------------------------------------
 *      Name:    SD_CARD.H
 *      Purpose: File manipulation example definitions
 *      Rev.:    V3.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#define CNTLQ       0x11
#define CNTLS       0x13
#define DEL         0x7F
#define BACKSPACE   0x08
#define CR          0x0D
#define LF          0x0A
#define ESC         0x1B

/* Command definitions structure. */
typedef struct scmd {
   char val[9];
   void (*func)(char *par);
} SCMD;

/* External functions */

/* Command Functions */
void cmd_i2cdetect (char *par);
void cmd_i2cread (char *par);
void cmd_i2cwrite (char *par);
void cmd_spiread(char *par);
void cmd_spiwrite(char *par);
void cmd_alsread (char *par);
void cmd_alswrite (char *par);
void cmd_ioexpr (char *par);
void cmd_ioexpw (char *par);
void cmd_enable (char *par);
void cmd_disable (char *par);
void cmd_sai (char *par);
void cmd_pon (char *par);
void cmd_config (char *par);
void cmd_config_als (char *par);
void cmd_config_fd (char *par);
void cmd_config_fifo (char *par);
void cmd_setup (char *par);
void cmd_id (char *par);
void cmd_dump (char *par);
void cmd_isUP (char *par);
void cmd_status (char *par);
void cmd_version (char *par);
void cmd_irq (char *par);
void cmd_callux (char *par);
void cmd_dialstart (char *par);
void cmd_gpio_mode(char *par);
void cmd_gpio(char *par);
void cmd_gpio_read(char *par);
void cmd_gpio_list(char *par);
void cmd_adc1 (char *par);
void cmd_adc4 (char *par);
void cmd_fill (char *par);
void cmd_backlight (char *par);
void cmd_temp (char *par);
void cmd_hum (char *par);
void cmd_heat (char *par);
void cmd_monitor (char *par);
void cmd_flashr (char *par);
void cmd_flashw (char *par);
void cmd_fbtest (char *par);
void cmd_showpic (char *par);
void cmd_help (char *par);

/* Local Function Prototypes */
char *get_entry (char *cp, char **pNext);
void DispatchCmd (void);

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
