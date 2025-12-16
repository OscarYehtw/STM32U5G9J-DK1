/*----------------------------------------------------------------------------
 *      Name:    CLI.C
 *      Purpose: File manipulation example program
 *      Rev.:    V3.24
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                    /* standard I/O .h-file                */
#include <string.h>                   /* string and memory functions         */
#include <ctype.h>                    /* character functions                 */
#include <stdint.h>
#include <stdbool.h>
#include "mxplatform.h"
#include "cli.h"

const char Cli_Help[] = 
   "+ COMMAND ------------------+ FUNCTION ---------------------------------+\n"
   "| I2CDETECT  <bus>          | Scan I2C bus and list detected addresses  |\n"
   "| ALSR <reg> <bytes>        | Read <bytes> from sensor register <reg>   |\n"
   "| ALSW <reg> <data >        | Write <data> to sensor register <reg>     |\n"
   "| enable  <als|flckr|all>   | Enable AMS measurement function           |\n"
   "| disable <als|flckr|all>   | Disable AMS measurement function          |\n"
   "| sai <clear|enable|disable>| Sleep-After-Interrupt control             |\n"
   "| pon <on|off>              | Power-On control for AMS device           |\n"
   "| config <sample_time>      | Configure AMS device                      |\n"
   "|        <mod_trigger>      |                                           |\n"
   "|        <wait_time>        |                                           |\n"
   "|        <agc_mode>         |                                           |\n"
   "|        <agc_nr_samples>   |                                           |\n"
   "| als <als_nr_samples>      | Total measurement time for ALS atime      |\n"
   "| fd <fd_nr_samples>        | Samples + 1 measured in one sequencer step|\n"
   "| fifo <reset> <threshold>  | Configure AMS FIFO (reset = 0/1,          |\n"
   "|                           |                     threshold = 0~ 047)   |\n"
   "| setup                     | Show current device configuration         |\n"
   "| id                        | Read device ID / revision                 |\n"
   "| dump                      | Dumps all registers                       |\n"
   "| up                        | Check if device is operational            |\n"
   "| status <als|flckr|fifo|   | Show historical measurement data          |\n"
   "|         freq|lux|cct>     |                                           |\n"
   "| version                   | Show version of current device driver     |\n"
   "| irq <on|off>              | Turn on/off IRQ logs                      |\n"
   "| DIAL                      | Show TMR-ADC values                       |\n"
   "| FILL <rgb888>             | Fill screen with rgb color                |\n"
   "| BL   <brightness>         | set backlight to brightness [0-100%%]      |\n"
   "| TEMP <sht|sts>            | Read temperature from sensor              |\n"
   "| HUM                       | Read humidity from sensor                 |\n"
   "| HEAT <pwr> <dur>          | Activate heater (pwr: HIGH/MED/LOW)       |\n"
   "| MONITOR                   | Monitor Env Sensors on LCD                |\n"
   "| FLASHR <addr> <len>       | Read <len> bytes from OSPI flash at <addr>|\n"
   "| FLASHW <addr>             | Write 256 bytes to OSPI flash at <addr>   |\n"
   "| HELP  or  ?               | displays this help                        |\n"
   "+---------------------------+-------------------------------------------+\n";

const SCMD cmd[] = {
	{ "I2CDETECT", cmd_i2cdetect },
	{ "ALSR",      cmd_alsread },
	{ "ALSW",      cmd_alswrite },
   { "ENABLE",    cmd_enable },
   { "DISABLE",   cmd_disable },
   { "SAI",       cmd_sai },
   { "PON",       cmd_pon },
   { "CONFIG",    cmd_config },
   { "ALS",       cmd_config_als },
   { "FD",        cmd_config_fd },
   { "FIFO",      cmd_config_fifo },
   { "SETUP",     cmd_setup },
   { "ID",        cmd_id },
   { "DUMP",      cmd_dump },
   { "UP",        cmd_isUP },
   { "STATUS",    cmd_status },
   { "VERSION",   cmd_version },
   { "IRQ",       cmd_irq },
	{ "DIAL",      cmd_dialstart },
	{ "FILL",      cmd_fill },
	{ "BL",        cmd_backlight },
	{ "TEMP",      cmd_temp },
	{ "HUM",       cmd_hum },
	{ "HEAT",      cmd_heat },
	{ "MONITOR",   cmd_monitor },
   { "FLASHR",    cmd_flashr },
   { "FLASHW",    cmd_flashw },
	{ "HELP",      cmd_help },
	{ "?",         cmd_help }
};

#define CMD_COUNT   (sizeof (cmd) / sizeof (cmd[0]))

char in_line[160];

/*----------------------------------------------------------------------------
 *        Display Command Syntax help
 *---------------------------------------------------------------------------*/
void cmd_help (char *par) {
   printf (Cli_Help);
}

/*----------------------------------------------------------------------------
 *      Line Editor
 *---------------------------------------------------------------------------*/
bool getline (char *lp, uint32_t n) {
   uint32_t cnt = 0;
   char c;

   do {
      c = getkey ();
      switch (c) {
         case CNTLQ:                       /* ignore Control S/Q             */
         case CNTLS:
            break;;
         case BACKSPACE:
         case DEL:
            if (cnt == 0) {
               break;
            }
            cnt--;                         /* decrement count                */
            lp--;                          /* and line pointer               */
#if defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6*/
            putchar (0x08);                /* echo backspace                 */
            putchar (' ');
            putchar (0x08);
#elif defined(__GNUC__)
            sendchar(0x08);
            sendchar(' ');
            sendchar(0x08);
#endif /* __ICCARM__ */
            break;
         case ESC:
            *lp = 0;                       /* ESC - stop editing line        */
            return (false);
         case CR:                          /* CR - done, stop editing line   */
            *lp = c;
            lp++;                          /* increment line pointer         */
            cnt++;                         /* and count                      */
            c = LF;
         default:
#if defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6*/
            putchar (*lp = c);             /* echo and store character       */
#elif defined(__GNUC__)
            sendchar(*lp = c);
#endif /* __ICCARM__ */
            lp++;                          /* increment line pointer         */
            cnt++;                         /* and count                      */
            break;
      }
   } while (cnt < n - 2  &&  c != LF);     /* check limit and CR             */
   *lp = 0;                                /* mark end of string             */
   return (true);
}

/*----------------------------------------------------------------------------
 *        Process input string for long or short name entry
 *---------------------------------------------------------------------------*/
char *get_entry (char *cp, char **pNext) {
   char *sp, lfn = 0, sep_ch = ' ';

   if (cp == NULL) {                          /* skip NULL pointers          */
      *pNext = cp;
      return (cp);
   }

   for ( ; *cp == ' ' || *cp == '\"'; cp++) { /* skip blanks and starting  " */
      if (*cp == '\"') { sep_ch = '\"'; lfn = 1; }
      *cp = 0;
   }
 
   for (sp = cp; *sp != CR && *sp != LF; sp++) {
      if ( lfn && *sp == '\"') break;
      if (!lfn && *sp == ' ' ) break;
   }

   for ( ; *sp == sep_ch || *sp == CR || *sp == LF; sp++) {
      *sp = 0;
      if ( lfn && *sp == sep_ch) { sp ++; break; }
   }

   *pNext = (*sp) ? sp : NULL;                /* next entry                  */
   return (cp);
}

/*----------------------------------------------------------------------------
 *        DispatchCmd: 
 *---------------------------------------------------------------------------*/
void DispatchCmd (void) {
	char *sp,*cp,*next;
	uint32_t i;

   /* display prompt */
#if defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6*/
   printf ("\nCmd> ");
#elif defined(__GNUC__)
   sendchar('\n');
   sendchar('C');
   sendchar('m');
   sendchar('d');
   sendchar('>');
#endif /* __ICCARM__ */

   /* get command line input */
	if (getline (in_line, sizeof (in_line)) == false) {
		return;
	}

	sp = get_entry (&in_line[0], &next);
	if (*sp == 0) {
		return;
    }
	
	for (cp = sp; *cp && *cp != ' '; cp++) {
      /* command to upper-case */
		*cp = toupper (*cp);
	}
    
	for (i = 0; i < CMD_COUNT; i++) {
   
		if (strcmp (sp, (const char *)&cmd[i].val)) {
			continue;
		}

		/* execute command function */
		cmd[i].func (next);
		break;                           
	}
	
	if (i == CMD_COUNT) {
		printf ("\nCommand error\n");
	}
	return;
}
