/*----------------------------------------------------------------------------
 *      Name:    als_rw.c
 *      Purpose: File manipulation example program
 *      Rev.:    V3.24
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                    /* standard I/O .h-file                */
#include <stdlib.h>
#include <stdbool.h>
#include "mxplatform.h"
#include "cli.h"

/*----------------------------------------------------------------------------
 *        cmd_flashr  — Read Flash via OSPI
 *---------------------------------------------------------------------------*/
void cmd_flashr(char *par)
{
    char *next;
    char *pAddr;
    char *pLen;
    unsigned int addr = 0;
    unsigned int len  = 0;
    uint8_t buffer[256];
    uint32_t i;

    /* Check input */
    if (par == NULL || *par == 0)
    {
        printf("Usage: FLASHR <address_hex> <length 1-256>\r\n");
        return;
    }

    /* Parse address */
    pAddr = get_entry(par, &next);
    if (pAddr == NULL)
    {
        printf("Invalid address.\r\n");
        return;
    }

    /* Hex address */
    if (sscanf(pAddr, "%x", &addr) != 1)
    {
        printf("Invalid address format.\r\n");
        return;
    }

    /* Parse length */
    pLen = get_entry(next, &next);
    if (pLen == NULL)
    {
        printf("Missing length.\r\n");
        return;
    }

    /* Length could be decimal or hex (same as ALSR) */
    if (sscanf(pLen, "%x", &len) != 1 || len == 0 || len > sizeof(buffer))
    {
        printf("Invalid length (must be 1~256)\r\n");
        return;
    }

    /* Enable Memory-Mapped mode */
    if (OSPI_NOR_EnableMemoryMappedMode(0) != BSP_ERROR_NONE)
    {
        printf("OSPI NOR Mem-Mapped Mode : Failed\r\n");
        //return;
    }

    /* Memory mapped base address */
    uint8_t *mem_addr = (uint8_t *)(OCTOSPI1_BASE + addr);

    /* Copy data out */
    for (i = 0; i < len; i++)
    {
        buffer[i] = mem_addr[i];
    }

    /* Disable MM mode (very important!) */
    OSPI_NOR_DisableMemoryMappedMode(0);
    
    /* Print result */
    printf("FLASH READ @%p (%u bytes):\r\n", mem_addr, len);

    // column header alignment
    printf("    ");
    for (i = 0; i < 16; i++) {
        printf("%02lX ", i);
    }
    printf(" | ASCII\n");

    /* Dump */
    addr = 0;

    while (addr < len) {
        // Print left address (00, 10, 20... F0)
        printf("%02X: ", addr);

        // Print row bytes
        for (i = 0; i < 16; i++) {
            if ((addr + i) < len)
                printf("%02X ", buffer[addr + i]);
            else
                printf("   "); // padding
        }

        /* Separator */
        printf(" | ");

        /* ASCII bytes */
        for (i = 0; i < 16; i++) {
            if (addr + i < len) {
                uint8_t c = buffer[addr + i];
                if (c >= 32 && c <= 126)
                    putchar(c);
                else
                    putchar('.');
            } else {
                 putchar(' ');
            }
        }

        printf("\n");
        addr += 16;
    }

    printf("\r\n");
}
