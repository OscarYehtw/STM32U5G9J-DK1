# STM32U5G9J-DK1 TBS

The default IDE is set to STM32CubeIDE, to change IDE open the STM32U5G9J-DK1.ioc with CubeMX and select from the supported IDEs (EWARM, MDK-ARM and STM32CubeIDE). Supports flashing of the STM32U5G9J-DK1 board directly from TouchGFX Designer using GCC and STM32CubeProgrammer. Flashing the board requires STM32CubeProgrammer which can be downloaded from the ST webpage.

This TBS is configured for 480 x 480 pixels 24 bpp screen resolution.

Performance testing can be done using the GPIO pins designated with the following signals in CN10 connector on the underside of the board:

- VSYNC_FREQ  - Pin PD1
- RENDER_TIME - Pin PD0
- FRAME_RATE  - Pin PF14
- MCU_ACTIVE  - Pin PF12
