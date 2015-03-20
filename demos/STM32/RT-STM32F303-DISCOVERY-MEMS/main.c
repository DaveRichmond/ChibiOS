/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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

#include "ch.h"
#include "hal.h"
#include "test.h"

#include "chprintf.h"
#include "shell.h"

#include "usbcfg.h"

#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_iochannel.h"
#include "simple.pb.h"

/* Virtual serial port over USB */
SerialUSBDriver SDU1;

static const I2CConfig i2ccfg = {
	0x00902025, // Timing
	0, // CR1
	0, // CR2
};

static const SPIConfig spicfg = {
	NULL,
	
	/* Hardware dependent */
	GPIOE, GPIOE_SPI1_CS,
	SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,
	0
};

/*
 * Command line related.
 */
#define SHELL_WA_SIZE		THD_WORKING_AREA_SIZE(2048)
#define TEST_WA_SIZE		THD_WORKING_AREA_SIZE(256)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]){
	size_t n, size;
	
	(void)argv;
	
	if(argc > 0){
		chprintf(chp, "Usage: mem\r\n");
		return;
	}
	
	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]){
	static const char *states[] = { CH_STATE_NAMES };
	thread_t *tp;
	
	(void)argv;
	if(argc > 0){
		chprintf(chp, "Usage: threads\r\n");
		return;
	}
	
	chprintf(chp, "         addr     stack     prio refs    state\r\n");
	tp = chRegFirstThread();
	do {
		chprintf(chp, "%-8s %08lx %02lx %4lu %4lu %9s\r\n",
			tp->p_name,
			(uint32_t)tp, (uint32_t)tp->p_ctx.r13,
			(uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
			states[tp->p_state]);
		tp = chRegNextThread(tp);
	} while(tp != NULL);
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]){
	thread_t *tp;
	
	(void)argv;
	if(argc > 0){
		chprintf(chp, "Usage: test\r\n");
		return;
	}
	tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriorityX(),
														TestThread, chp);
	if(tp == NULL){
		chprintf(chp, "out of memory\r\n");
		return;
	}
	chThdWait(tp);
}

static const ShellCommand commands[] = {
	{ "mem", 			cmd_mem },
	{ "threads", 	cmd_threads },
	{ "test", 		cmd_test },
	{ NULL, 			NULL },
};
static const ShellConfig shell_cfg1 = {
	(BaseSequentialStream *)&SDU1,
	commands
};

/*
 * Blinker thread #1.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  chRegSetThreadName("blinker");
  while (true) {
    palSetPad(GPIOE, GPIOE_LED3_RED);
    chThdSleepMilliseconds(125);
    palClearPad(GPIOE, GPIOE_LED3_RED);
    chThdSleepMilliseconds(125);
    palSetPad(GPIOE, GPIOE_LED7_GREEN);
    chThdSleepMilliseconds(125);
    palClearPad(GPIOE, GPIOE_LED7_GREEN);
    chThdSleepMilliseconds(125);
    palSetPad(GPIOE, GPIOE_LED10_RED);
    chThdSleepMilliseconds(125);
    palClearPad(GPIOE, GPIOE_LED10_RED);
    chThdSleepMilliseconds(125);
    palSetPad(GPIOE, GPIOE_LED6_GREEN);
    chThdSleepMilliseconds(125);
    palClearPad(GPIOE, GPIOE_LED6_GREEN);
    chThdSleepMilliseconds(125);
  }
}

/*
 * Blinker thread #2.
 */
static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;

  chRegSetThreadName("blinker");
  while (true) {
    chThdSleepMilliseconds(125);
    palSetPad(GPIOE, GPIOE_LED5_ORANGE);
    chThdSleepMilliseconds(125);
    palClearPad(GPIOE, GPIOE_LED5_ORANGE);
    chThdSleepMilliseconds(125);
    palSetPad(GPIOE, GPIOE_LED9_BLUE);
    chThdSleepMilliseconds(125);
    palClearPad(GPIOE, GPIOE_LED9_BLUE);
    chThdSleepMilliseconds(125);
    palSetPad(GPIOE, GPIOE_LED8_ORANGE);
    chThdSleepMilliseconds(125);
    palClearPad(GPIOE, GPIOE_LED8_ORANGE);
    chThdSleepMilliseconds(125);
    palSetPad(GPIOE, GPIOE_LED4_BLUE);
    chThdSleepMilliseconds(125);
    palClearPad(GPIOE, GPIOE_LED4_BLUE);
  }
}

static void magAccelWriteRegister(I2CDriver *i2c, uint8_t addr, uint8_t reg, uint8_t val){
	uint8_t cmd[] = { reg, val };
	i2cAcquireBus(i2c);
	i2cMasterTransmitTimeout(i2c, addr, cmd, 2, NULL, 0, TIME_INFINITE);
	i2cReleaseBus(i2c);
}
static uint8_t magAccelReadRegister(I2CDriver *i2c, uint8_t addr, uint8_t reg){
	uint8_t data;
	i2cAcquireBus(i2c);
	i2cMasterTransmitTimeout(i2c, addr, &reg, 1, &data, 1, TIME_INFINITE);
	i2cReleaseBus(i2c);
	return data;
}
static void magAccelInit(I2CDriver *i2c){
	// enable all axes at the highest speed
	magAccelWriteRegister(i2c, 0x19, 0x20, 0x97);
	
	// enable at the highest speed
	magAccelWriteRegister(i2c, 0x1E, 0x00, 0x1C);
	magAccelWriteRegister(i2c, 0x1E, 0x02, 0x00);
}

static uint8_t accelRead(I2CDriver *i2c, float *data){
	uint8_t start_reg = 0x27 | 0x80;
	uint8_t out[7];
	
	i2cAcquireBus(i2c);
	i2cMasterTransmitTimeout(i2c, 0x19, &start_reg, 1, out, 7, TIME_INFINITE);
	i2cReleaseBus(i2c);
	
	if(out[0] & 0x08){
		int16_t val_x = (out[2] << 8) | out[1];
		int16_t val_y = (out[4] << 8) | out[3];
		int16_t val_z = (out[6] << 8) | out[5];
		
		data[0] = ((float)val_x) * (4.0 / 65535.0) * 9.81;
		data[1] = ((float)val_y) * (4.0 / 65535.0) * 9.81;
		data[2] = ((float)val_z) * (4.0 / 65535.0) * 9.81;
		
		return 1;
	}
	return 0;
}

static uint8_t magRead(I2CDriver *i2c, float *data){
	uint8_t start_reg = 0x03;
	uint8_t out[7];
	
	i2cAcquireBus(i2c);
	i2cMasterTransmitTimeout(i2c, 0x1E, &start_reg, 1, out, 7, TIME_INFINITE);
	i2cReleaseBus(i2c);
	
	int16_t val_x = (out[0] << 8) | out[1];
	int16_t val_y = (out[2] << 8) | out[3];
	int16_t val_z = (out[4] << 8) | out[5];
	
	data[0] = ((float)val_x) * 1.22;
	data[1] = ((float)val_y) * 1.22;
	data[2] = ((float)val_z) * 1.22;
	
	return 1;
}

/*
 * Application entry point.
 */
int main(void) {
	thread_t *shelltp = NULL;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
	
	/*
	 * Shell manager initialisation.
	 */
	shellInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   * PA9(TX) and PA10(RX) are routed to USART1.
   */
  sdStart(&SD1, NULL);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
	
	/* 
	 * Initialises the serial-over-USB CDC driver.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);
	
	/*
	 * Activates the USB driver.
	 */
	usbStart(serusbcfg.usbp, &usbcfg);
	
	/* 
	 * Initialise I2C
	 */
	i2cStart(&I2CD1, &i2ccfg);
	magAccelInit(&I2CD1);
	 
	spiStart(&SPID1, &spicfg);

  /*
   * Creates the example threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL);

  while (TRUE) {
		if(SDU1.config->usbp->state == USB_ACTIVE){
			//SimpleMessage message;			
			//uint8_t message_len;
			//chSequentialStreamRead((BaseSequentialStream *)&SDU1, &message_len, 1);
			//pb_istream_t usb_stream = pb_istream_from_iochannel((BaseSequentialStream *)&SDU1, message_len);
		}
    //if(!shelltp){
		//	if(SDU1.config->usbp->state == USB_ACTIVE){
		//		/* Spawns a new shell */
		//		shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		//	}
		//} else {
		//	/* If the previous shell exited */
		//	if(chThdTerminatedX(shelltp)){
		//		/* Recovers the memory of the previous shell */
		//		chThdRelease(shelltp);
		//		shelltp = NULL;
		//	}
		//}
		
		//if(SDU1.config->usbp->state == USB_ACTIVE){
		//	float data[3];
		//	
		//	if(accelRead(&I2CD1, data)){
		//		chprintf((BaseSequentialStream *)&SDU1, "Accel: %f %f %f\r\n", data[0], data[1], data[2]);
		//	}
		//	if(magRead(&I2CD1, data)){
		//		chprintf((BaseSequentialStream *)&SDU1, "Mag: %f %f %f\r\n", data[0], data[1], data[2]);
		//	}
		//}
		chThdSleepMilliseconds(250);
  }
}
