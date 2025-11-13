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
   char val[8];
   void (*func)(char *par);
} SCMD;

/* External functions */

/* Command Functions */
void cmd_alsread (char *par);
void cmd_alswrite (char *par);
void cmd_tmradc (char *par);
void cmd_help (char *par);

/* Local Function Prototypes */
char *get_entry (char *cp, char **pNext);
void DispatchCmd (void);

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
