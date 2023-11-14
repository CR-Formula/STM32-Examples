/* Header for CAN configuration
 * This file handles all of the configuration of the CAN peripherals
 * The order should be
 * Bit timing
 * Global receive -- try with everything first
 * CCCR register --> enable FDCAN
 */

#include "stm32h743xx.h"

static inline can_bit_timming() {
	FDCAN1->CCCR = FDCAN_CCCR_CCE;
	FDCAN1->CCCR = FDCAN_CCCR_INIT;
	FDCAN1->DBTP = (20UL << FDCAN_DBTP_DBRP_Pos);
}

static inline can_cc() {
	FDCAN1->CCCR = FDCAN_CCCR_TXP;
}
