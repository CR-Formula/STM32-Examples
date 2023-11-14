/* Header for CAN configuration
 * This file handles all of the configuration of the CAN peripherals
 * The order should be
 * Clocks
 * Bit timing
 * Global receive -- try with everything first
 * CCCR register --> enable FDCAN
 */

#include "cmsis_os.h"


static inline can_clock() {

}

static inline can_cc() {
	FDCAN1->CCCR = FDCAN_CCCR_TXP;
}
