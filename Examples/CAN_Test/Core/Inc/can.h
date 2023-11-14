/* Alex Bashara
 * Cyclone Racing 2023
 * CR-28
 *
 * can.h
 * Configures the CAN peripheral
 * The order should be
 * Bit timing
 * Global receive -- try with everything first
 * CCCR register --> enable FDCAN
 *
 * Clocks are handled in the main function by STM32Cube
 */

#include "stm32h743xx.h"


static inline void can_baud() {
	// Value of these registers is what is programmed +1
	FDCAN1->NBTP
			= FDCAN_NBTP_NSJW(12) // 13
			| FDCAN_NBTP_NBRP(0) // 1
			| FDCAN_NBTP_NTSEG1(85) // 86
			| FDCAN_NBTP_NTSEG2(12); // 13

}

static inline void can_config() {
	// Set the CCR Register
	FDCAN1->CCCR
		= FDCAN_CCCR_CCE
		| FDCAN_CCCR_INIT
		| FDCAN_CCCR_TXP
		| FDCAN_CCCR_DAR;
	// Set the TTOCF register
	FDCAN1->TTOCF
			= FDCAN_TTOCF_OM(00);

}
