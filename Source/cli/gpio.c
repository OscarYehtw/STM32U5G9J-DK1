/*----------------------------------------------------------------------------
 *      Name:    gpio.c
 *      Purpose: File manipulation example program
 *      Rev.:    V3.24
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                    /* standard I/O .h-file                */
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include "mxplatform.h"
#include "cli.h"

/* forward declarations */
static const char *gpio_mode_str(uint32_t mode);
static const char *gpio_pull_str(uint32_t pull);

GPIO_TypeDef *cli_parse_gpio_port(const char *s)
{
    if (s == NULL || strlen(s) != 1)
        return NULL;

    switch (toupper(s[0]))
    {
        case 'A': return GPIOA;
        case 'B': return GPIOB;
        case 'C': return GPIOC;
        case 'D': return GPIOD;
        case 'E': return GPIOE;
        case 'F': return GPIOF;
        case 'G': return GPIOG;
        case 'H': return GPIOH;
        case 'I': return GPIOI;
        case 'J': return GPIOJ;
        default:  return NULL;
    }
}

uint16_t cli_parse_gpio_pin(const char *s)
{
    uint32_t pin = strtoul(s, NULL, 0);

    if (pin > 15)
        return 0xFFFF;

    return (uint16_t)(1U << pin);
}

void cmd_gpio_mode(char *par)
{
    char *p1, *p2, *p3, *next;
    GPIO_TypeDef *GPIOx;
    uint16_t pin;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (par == NULL || *par == 0)
    {
        printf("Usage: GPIO MODE <port> <pin> <in|out|af|analog>\r\n");
        return;
    }

    p1 = get_entry(par, &next);   // port
    p2 = get_entry(next, &next);  // pin
    p3 = get_entry(next, &next);  // mode

    if (p1 == NULL || p2 == NULL || p3 == NULL)
    {
        printf("Usage: GPIO MODE <port> <pin> <in|out|af|analog>\r\n");
        return;
    }

    GPIOx = cli_parse_gpio_port(p1);
    pin   = cli_parse_gpio_pin(p2);

    if (GPIOx == NULL || pin == 0xFFFF)
    {
        printf("Invalid port or pin\r\n");
        return;
    }

    for (char *c = p3; *c; c++)
        *c = toupper(*c);

    HAL_GPIO_DeInit(GPIOx, pin);

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    if (strcmp(p3, "IN") == 0)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    }
    else if (strcmp(p3, "OUT") == 0)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    }
    else if (strcmp(p3, "AF") == 0)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Alternate = 0;  // Alternate function is not configured here. User can re-configure it later if needed.
    }
    else if (strcmp(p3, "ANALOG") == 0)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    }
    else
    {
        printf("Invalid mode: %s\r\n", p3);
        return;
    }

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    printf("GPIO %s%s mode set to %s\r\n", p1, p2, p3);
}

/*----------------------------------------------------------------------------
 *  GPIO <port> <pin> <high|low|toggle>
 *---------------------------------------------------------------------------*/
void cmd_gpio(char *par)
{
    char *p1, *p2, *p3;
    char *next;

    GPIO_TypeDef *GPIOx;
    uint16_t pin;
    
    /* Parameter check */
    if (par == NULL || *par == 0)
    {
        printf("Usage: GPIO <port> <pin> <high|low|toggle>\r\n");
        return;
    }

    /* parse parameters */
    p1 = get_entry(par, &next);     // port
    p2 = get_entry(next, &next);    // pin
    p3 = get_entry(next, &next);    // action

    if (p1 == NULL || p2 == NULL || p3 == NULL)
    {
        printf("Usage: GPIO <port> <pin> <high|low|toggle>\r\n");
        return;
    }

    /* parse port */
    GPIOx = cli_parse_gpio_port(p1);
    if (GPIOx == NULL)
    {
        printf("Invalid GPIO port: %s\r\n", p1);
        return;
    }

    /* parse pin */
    pin = cli_parse_gpio_pin(p2);
    if (pin == 0xFFFF)
    {
        printf("Invalid GPIO pin: %s\r\n", p2);
        return;
    }

    /* action */
    for (char *c = p3; *c; c++)
        *c = toupper(*c);

    if (strcmp(p3, "HIGH") == 0)
    {
        HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_SET);
    }
    else if (strcmp(p3, "LOW") == 0)
    {
        HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);
    }
    else if (strcmp(p3, "TOGGLE") == 0)
    {
        HAL_GPIO_TogglePin(GPIOx, pin);
    }
    else
    {
        printf("Invalid action: %s (high|low|toggle)\r\n", p3);
        return;
    }

    printf("GPIO %s %s %s OK\r\n", p1, p2, p3);
}

void cmd_gpio_read(char *par)
{
    char *p1, *p2, *next;
    GPIO_TypeDef *GPIOx;
    uint16_t pin;
    GPIO_PinState state;

    if (par == NULL || *par == 0)
    {
        printf("Usage: GPIO READ <port> <pin>\r\n");
        return;
    }

    p1 = get_entry(par, &next);   // port
    p2 = get_entry(next, &next);  // pin

    if (p1 == NULL || p2 == NULL)
    {
        printf("Usage: GPIO READ <port> <pin>\r\n");
        return;
    }

    GPIOx = cli_parse_gpio_port(p1);
    pin   = cli_parse_gpio_pin(p2);

    if (GPIOx == NULL || pin == 0xFFFF)
    {
        printf("Invalid port or pin\r\n");
        return;
    }

    state = HAL_GPIO_ReadPin(GPIOx, pin);

    printf("GPIO %s%s = %s\r\n",
           p1, p2,
           (state == GPIO_PIN_SET) ? "HIGH" : "LOW");
}

static const char *gpio_mode_str(uint32_t mode)
{
    switch (mode)
    {
        case 0: return "IN ";
        case 1: return "OUT";
        case 2: return "AF ";
        case 3: return "ANA";
        default: return "???";
    }
}

static const char *gpio_pull_str(uint32_t pull)
{
    switch (pull)
    {
        case 0: return "NOPULL";
        case 1: return "PULLUP";
        case 2: return "PULLDN";
        default: return "?????";
    }
}

static void gpio_dump_port(char port_name, GPIO_TypeDef *GPIOx)
{
    uint32_t moder  = GPIOx->MODER;
    uint32_t idr    = GPIOx->IDR;
    uint32_t pupdr  = GPIOx->PUPDR;
    uint32_t afr_l  = GPIOx->AFR[0];
    uint32_t afr_h  = GPIOx->AFR[1];

    printf("\r\nGPIO%c:\r\n", port_name);
    printf("PIN  MODE  LV  PULL    AF\r\n");
    printf("----------------------------\r\n");

    for (uint32_t pin = 0; pin < 16; pin++)
    {
        uint32_t mode = (moder >> (pin * 2)) & 0x3;
        uint32_t pull = (pupdr >> (pin * 2)) & 0x3;
        uint32_t lv   = (idr >> pin) & 0x1;
        uint32_t af   = (pin < 8) ?
                        ((afr_l >> (pin * 4)) & 0xF) :
                        ((afr_h >> ((pin - 8) * 4)) & 0xF);

        printf("%2lu   %s   %s  %-6s ",
               pin,
               gpio_mode_str(mode),
               lv ? "H" : "L",
               gpio_pull_str(pull));

        if (mode == 2)
            printf("AF%lu", af);
        else
            printf(" --");

        printf("\r\n");
    }
}

void cmd_gpio_list(char *par)
{
    (void)par;

    gpio_dump_port('A', GPIOA);
    gpio_dump_port('B', GPIOB);
    gpio_dump_port('C', GPIOC);
    gpio_dump_port('D', GPIOD);
    gpio_dump_port('E', GPIOE);
    gpio_dump_port('F', GPIOF);
    gpio_dump_port('G', GPIOG);
    gpio_dump_port('H', GPIOH);
    gpio_dump_port('I', GPIOI);
    gpio_dump_port('J', GPIOJ);
}
