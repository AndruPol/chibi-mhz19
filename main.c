/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#define MHZ19_GAS			0x86	// Read Gas Concentration
#define MHZ19_ABC			0x79	// Set ABC logic on/off (ABC = automatic baseline correction)
#define MHZ19_RANGE			0x99	// Set sensor detection range in 6-7 byte

#define MHZ19_CMDLEN		9

#include "mhz19.h"
#include "usbcfg.h"

static const SerialConfig serial_cfg = {
  .speed = 9600,
};

static uint8_t cmd[MHZ19_CMDLEN] 	= { 0xFF, 0x01, MHZ19_GAS,   0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };	// read gas concentration
static uint8_t range[MHZ19_CMDLEN] 	= { 0xFF, 0x01, MHZ19_RANGE, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F };	// set range = 2000 ppm
static uint8_t abc[MHZ19_CMDLEN]	= { 0xFF, 0x01, MHZ19_ABC,   0x00, 0x00, 0x00, 0x00, 0x00, 0x86 };	// set ABC off
static uint8_t buf[MHZ19_CMDLEN];


// MH-Z19 checksum calculate
static uint8_t mhz19_checksum(uint8_t *buf) {
	uint8_t checksum=0;
	for (uint8_t i=1; i < MHZ19_CMDLEN-1; i++ )
		checksum += buf[i];
	checksum = 255 - checksum;
	checksum += 1;
	return checksum;
}

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

// called on kernel panic
void halt(void){
	port_disable();
	while(TRUE)
	{
	}
}

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  // USART1 serial connected to MH-Z19: PA9 - TX, PA10 - RX
  sdStart(&SD1, &serial_cfg);
  palSetPadMode(GPIOA, GPIOA_MB_TX, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(GPIOA, GPIOA_MB_RX, PAL_MODE_INPUT);

#if 1
  // set ABC off
  sdWrite(&SD1, abc, MHZ19_CMDLEN);

  chThdSleepMilliseconds(100);

  // set range to 2000 for PWM output
  sdWrite(&SD1, range, MHZ19_CMDLEN);
#endif

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  // TIM3 PB4 connected to MH-Z19 PWM output
  mhz19_init();

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
  */
  while (TRUE) {

    sdWrite(&SD1, cmd, MHZ19_CMDLEN);
    uint8_t len = sdRead(&SD1, buf, MHZ19_CMDLEN);

    uint16_t co2ser = 0;
    if (len == MHZ19_CMDLEN) {
    	if (buf[0] == 0xFF && buf[1] == MHZ19_GAS && mhz19_checksum(buf) == buf[MHZ19_CMDLEN - 1]) {
    		co2ser = (uint16_t) (buf[2] << 8) + buf[3];
    	}
    }

    uint16_t co2pwm;
    if (mhz19_read(&co2pwm) != MHZ19_OK)
    	co2pwm = 0;

   	// print MH-Z19 values from PWM & Serial output
    if (SDU1.config->usbp->state == USB_ACTIVE)
    	chprintf((BaseSequentialStream *) &SDU1, "PWM: %d, SERIAL: %d\r\n", co2pwm, co2ser);

   	chThdSleepMilliseconds(3000);

  }
}
