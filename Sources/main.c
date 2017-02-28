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
#include "fsl_clock_manager.h"
#include "fsl_pit_hal.h"
#include "fsl_lpsci_hal.h"
#include "fsl_port_hal.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

uint8_t  rxChar = 0;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

int main(void)
{
	// Enable clock for PORTs, setup board clock source, config pin
    board_init();

    /* =========== UART ================ */

	PORT_HAL_SetMuxMode(PORTA, 1u, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTA, 2u, kPortMuxAlt2);

	// LPSCI: Low Power Serial Communication Interface

	/* selecciona clock de PLLFLLSEL */
	CLOCK_HAL_SetLpsciSrc(SIM, 0, kClockLpsciSrcPllFllSel);

	/* habilita clock a la UART0 (Lpsci0) */
	SIM_HAL_EnableClock(SIM, kSimClockGateLpsci0);

	/* setea baudrate */
	LPSCI_HAL_SetBaudRate(UART0, SystemCoreClock, 115200);

	/* configura 8 bits de datos */
	LPSCI_HAL_SetBitCountPerChar(UART0, kLpsci8BitsPerChar);

	/* deshabilita paridad */
	LPSCI_HAL_SetParityMode(UART0, kLpsciParityDisabled);

	/* 1 bit de stop */
	LPSCI_HAL_SetStopBitCount(UART0, kLpsciOneStopBit);

    // Habilita interrupción de recepción
    LPSCI_HAL_SetIntMode(UART0, kLpsciIntRxDataRegFull, true);

    /* habilita transmisor y receptor */
    LPSCI_HAL_EnableTransmitter(UART0);
    LPSCI_HAL_EnableReceiver(UART0);

	NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);

    while(true)
    {
    }
}


void UART0_IRQHandler(void)
{
	if (LPSCI_HAL_GetStatusFlag(UART0, kLpsciRxDataRegFull))
	{
		// Funcion Eco
		LPSCI_HAL_Getchar(UART0, &rxChar);
		LPSCI_HAL_Putchar(UART0, rxChar);

		LPSCI_HAL_ClearStatusFlag(UART0, kLpsciRxDataRegFull);
	}
}

/*==================[end of file]============================================*/




