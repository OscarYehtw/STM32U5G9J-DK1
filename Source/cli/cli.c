/*----------------------------------------------------------------------------
 *      R T L   F l a s h   F i l e   S y s t e m   E x a m p l e
 *----------------------------------------------------------------------------
 *      Name:    SD_FILE.C
 *      Purpose: File manipulation example program
 *      Rev.:    V3.24
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                    /* standard I/O .h-file                */
#include <string.h>                   /* string and memory functions         */
#include "cli.h"

const char Cli_Help[] = 
   "+ COMMAND ------------------+ FUNCTION ---------------------------------+\n"
   "| HELP  or  ?               | displays this help                        |\n"
   "+---------------------------+-------------------------------------------+\n";

const SCMD cmd[] = {
   "HELP",   cmd_help,
   "?",      cmd_help};

#define CMD_COUNT   (sizeof (cmd) / sizeof (cmd[0]))

char in_line[160];

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
 *        Display Command Syntax help
 *---------------------------------------------------------------------------*/
void cmd_help (char *par) {
   printf (Cli_Help);
}
