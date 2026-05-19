/*
 * wizchip_port.h
 *
 *  Created on: Oct 27, 2025
 *      Author: arunrawat
 */

#ifndef INC_WIZCHIP_PORT_H_
#define INC_WIZCHIP_PORT_H_

int W5500_Init(void);
/* Re-apply static IP to W5500 (call when socket open fails, e.g. IP was lost) */
void W5500_ApplyNetInfo(void);

#endif /* INC_WIZCHIP_PORT_H_ */
