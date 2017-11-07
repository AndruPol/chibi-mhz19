/*
 * mhz19.c - MH-Z19 CO2 sensor driver
 *
 *  Created on: 03/11/2017
 *      Author: andru
 */

#include "ch.h"
#include "hal.h"

#include "mhz19.h"

#define MHZ19_TIMEOUT_MS 	(2 * 1004)	// 2 * 1004 mS max wait time
#define MHZ19_PERIOD_MIN 	9500		// 1004 mS -5% minimal period len * 10
#define MHZ19_PERIOD_MAX 	10550		// 1004 mS +5% maximal period len * 10
#define MHZ19_PULSE_MIN 	19			// 2 mS -5% minimal pulse * 10

#define ICUDRIVER			ICUD3

#define MHZ19_CH_GPIO		GPIOB 			/* TIM3 channel 1 GPIOB */
#define MHZ19_CH_PIN		GPIOB_MHZ19		/* TIM channel 1 PB4 w partial remap */
#define MHZ19_CH			ICU_CHANNEL_1

/*-----------------------------------------------------------------------------*/
#define MHZ19_PRIO	(NORMALPRIO+1)

static thread_t *MHZ19Thread_p;
static THD_WORKING_AREA(waMHZ19Thread, 192);

typedef struct _icu_capture_t icu_capture_t;
struct _icu_capture_t {
	uint16_t period;
	uint16_t high;
};

typedef struct _mhz19_read mhz19_read_t;
struct _mhz19_read {
	mhz19_error_t error;	/* out */
	uint16_t data;		 	/* out */
};

static binary_semaphore_t icusem, cb_sem;
static volatile icu_capture_t icu_data;
static mhz19_read_t rd;

static void icuwidthcb(ICUDriver *icup) {
  icu_data.high = icuGetWidthX(icup);
}

static void icuperiodcb(ICUDriver *icup) {
  icu_data.period = icuGetPeriodX(icup);
  chSysLockFromISR();
  chBSemSignalI(&cb_sem);
  chSysUnlockFromISR();
}

static void icuoverflowcb(ICUDriver *icup) {
  (void)icup;
  icu_data.period = 0;
  chSysLockFromISR();
  chBSemSignalI(&cb_sem);
  chSysUnlockFromISR();
}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  10000,                    /* 10kHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  icuoverflowcb,
  MHZ19_CH,
  0
};

/*
 * MHZ19 read thread.
 */
static THD_FUNCTION(MHZ19Thread, arg) {
  (void)arg;
  chRegSetThreadName("MHZ19Thd");

  chBSemObjectInit(&cb_sem, TRUE);
  while (TRUE) {
	/* wait for read request */
	mhz19_read_t *req;
	thread_t *tp;
	tp = chMsgWait();
	req = (mhz19_read_t *) chMsgGet(tp);
	chMsgRelease(tp, (msg_t) req);

	icuStartCapture(&ICUDRIVER);
	icuEnableNotifications(&ICUDRIVER);

	// IRQ timeout or receive timeout
	if(chBSemWaitTimeout(&cb_sem, MS2ST(MHZ19_TIMEOUT_MS)) == MSG_TIMEOUT) {
		req->error = MHZ19_IRQ_TIMEOUT;
		goto reply;
	}
	if(! icu_data.period) {
		req->error = MHZ19_TIMEOUT;
		goto reply;
	}

	if(icu_data.period > MHZ19_PERIOD_MAX || icu_data.period < MHZ19_PERIOD_MIN) {
		req->error = MHZ19_ERROR;
		goto reply;
	}

	if(icu_data.high < MHZ19_PULSE_MIN) {
		req->error = MHZ19_ERROR;
		goto reply;
	}

	req->data = 2000 * (icu_data.high - 20) / (icu_data.period - 40);
	req->error = MHZ19_OK;

reply:
	icuDisableNotifications(&ICUDRIVER);
	icuStopCapture(&ICUDRIVER);
	chBSemSignal(&icusem);
  }
}

mhz19_error_t mhz19_read(uint16_t *co2) {
	mhz19_read_t *rd_p = &rd;

	chBSemWait(&icusem); /* to be sure */

	chMsgSend(MHZ19Thread_p, (msg_t) rd_p);

	/* wait for reply */
	if(chBSemWaitTimeout(&icusem, MS2ST(MHZ19_TIMEOUT_MS)) == MSG_TIMEOUT) {
		chBSemReset(&icusem, FALSE);
		return MHZ19_TIMEOUT;
	}
	chBSemReset(&icusem, FALSE);

	if(rd.error != MHZ19_OK) {
		return rd.error;
	}

	*co2 = rd.data;

	return MHZ19_OK;
}

void mhz19_init(void){
	icuStart(&ICUDRIVER, &icucfg);
	AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_PARTIALREMAP;
	palSetPadMode(GPIOB, GPIOB_MHZ19, PAL_MODE_INPUT);

	chBSemObjectInit(&icusem, FALSE);
	MHZ19Thread_p = chThdCreateStatic(waMHZ19Thread, sizeof(waMHZ19Thread), MHZ19_PRIO, MHZ19Thread, NULL);
}
