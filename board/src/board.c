/* Copyright 2017, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2017, Diego Alegrechi
 * Copyright 2017, Gustavo Muro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/
#include "board.h"
#include "MKL46Z4.h"
#include "fsl_port_hal.h"
#include "fsl_gpio_hal.h"
#include "fsl_sim_hal.h"
#include "fsl_sim_hal_MKL46Z4.h"
#include "fsl_clock_MKL46Z4.h"
#include "fsl_i2c_hal.h"
#include "fsl_lpsci_hal.h"
#include "mma8451.h"

/*==================[macros and definitions]=================================*/

/* EXTAL0 PTA18 */
#define EXTAL0_PORT   PORTA
#define EXTAL0_PIN    18
#define EXTAL0_PINMUX kPortPinDisabled

/* XTAL0 PTA19 */
#define XTAL0_PORT   PORTA
#define XTAL0_PIN    19
#define XTAL0_PINMUX kPortPinDisabled

/** \brief definiciones para el Led rojo */
#define LED_ROJO_PORT       PORTE
#define LED_ROJO_GPIO       GPIOE
#define LED_ROJO_PIN        29

/** \brief definiciones para el Led verde */
#define LED_VERDE_PORT      PORTD
#define LED_VERDE_GPIO      GPIOD
#define LED_VERDE_PIN       5

/** \brief definiciones para el SW1 */
#define SW1_PORT            PORTC
#define SW1_GPIO            GPIOC
#define SW1_PIN             3

/** \brief definiciones para el SW3 */
#define SW3_PORT            PORTC
#define SW3_GPIO            GPIOC
#define SW3_PIN             12

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void board_init(void)
{
    int32_t busClock;

    /* Activación de clock para los puertos utilizados */
    SIM_HAL_EnableClock(SIM, kSimClockGatePortA);
    SIM_HAL_EnableClock(SIM, kSimClockGatePortC);
    SIM_HAL_EnableClock(SIM, kSimClockGatePortD);
    SIM_HAL_EnableClock(SIM, kSimClockGatePortE);

	/* =========== LED ROJO =============== */
	
    PORT_HAL_SetMuxMode(LED_ROJO_PORT, LED_ROJO_PIN, kPortMuxAsGpio);
    ledRojo_off();
	GPIO_HAL_SetPinDir(LED_ROJO_GPIO, LED_ROJO_PIN, kGpioDigitalOutput);
	
	/* =========== LED VERDE ============== */
	
	PORT_HAL_SetMuxMode(LED_VERDE_PORT, LED_VERDE_PIN, kPortMuxAsGpio);
    ledVerde_off();
    GPIO_HAL_SetPinDir(LED_VERDE_GPIO, LED_VERDE_PIN, kGpioDigitalOutput);
	
	/* =========== SW1 =================== */
    PORT_HAL_SetMuxMode(SW1_PORT, SW1_PIN, kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(SW1_GPIO, SW1_PIN, kGpioDigitalInput);
    PORT_HAL_SetPullCmd(SW1_PORT, SW1_PIN, true);
    PORT_HAL_SetPullMode(SW1_PORT, SW1_PIN, kPortPullUp);
	
	/* =========== SW3 =================== */

    PORT_HAL_SetMuxMode(SW3_PORT, SW3_PIN, kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(SW3_GPIO, SW3_PIN, kGpioDigitalInput);
    PORT_HAL_SetPullCmd(SW3_PORT, SW3_PIN, true);
    PORT_HAL_SetPullMode(SW3_PORT, SW3_PIN, kPortPullUp);

    /* =========== I2C =================== */

	/* seleccion función alternativa 5 (I2C) */
	PORT_HAL_SetMuxMode(PORTE, 24,kPortMuxAlt5);

	/* seleccion función alternativa 5 (I2C) */
	PORT_HAL_SetMuxMode(PORTE, 25,kPortMuxAlt5);

	/* activa clock para I2C */
	SIM_HAL_EnableClock(SIM, kSimClockGateI2c0);

	/* inicializa el I2C */
	I2C_HAL_Init(I2C0);

	/* configura baudrate */
	I2C_HAL_SetBaudRate(I2C0, SystemCoreClock, 100, NULL);

	/* activa el I2C */
	I2C_HAL_Enable(I2C0);

	/* =========== MMA8451 ================ */

    mma8451_init();

    /* =========== ADC ================ */

    PORT_HAL_SetMuxMode(PORTE, 22u, kPortPinDisabled);
    CLOCK_SYS_EnableAdcClock(ADC0_IDX);


}

int8_t pulsadorSw1_get(void)
{
    if (GPIO_HAL_ReadPinInput(SW1_GPIO, SW1_PIN))
        return 0;
    else
        return 1;
}

int8_t pulsadorSw3_get(void)
{
    if (GPIO_HAL_ReadPinInput(SW3_GPIO, SW3_PIN))
        return 0;
    else
        return 1;
}

void ledRojo_on(void)
{
    GPIO_HAL_ClearPinOutput(LED_ROJO_GPIO, LED_ROJO_PIN);
}

void ledRojo_off(void)
{
    GPIO_HAL_SetPinOutput(LED_ROJO_GPIO, LED_ROJO_PIN);
}

void ledRojo_toggle(void)
{
    GPIO_HAL_TogglePinOutput(LED_ROJO_GPIO, LED_ROJO_PIN);
}

void ledVerde_on(void)
{
    GPIO_HAL_ClearPinOutput(LED_VERDE_GPIO, LED_VERDE_PIN);
}

void ledVerde_off(void)
{
    GPIO_HAL_SetPinOutput(LED_VERDE_GPIO, LED_VERDE_PIN);
}

void ledVerde_toggle(void)
{
    GPIO_HAL_TogglePinOutput(LED_VERDE_GPIO, LED_VERDE_PIN);
}

/*==================[end of file]============================================*/
