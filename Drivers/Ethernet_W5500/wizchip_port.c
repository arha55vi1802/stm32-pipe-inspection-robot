/*
 * wizchip_port.c
 *
 *  Created on: Oct 27, 2025
 *      Author: controllerstech
 */

#include "main.h"
#include "wizchip_conf.h"
#include "stdio.h"
#include "string.h"
#include "socket.h"
#include "stdbool.h"
#include "DHCP/dhcp.h"
#include "DNS/dns.h"

/* W5500 on SPI2: SCK=PB10, MISO=PC2, MOSI=PC3. CS=PA5, RESET=PA4. */
#define W5500_SPI hspi2
/* Log/printf: use huart2 (PA2/PA3) = board USB serial, or huart6 (PC6/PC7) for external adapter */
#define LOG_UART  huart2
#define USE_DHCP  0


/* Direct cable: PC Ethernet <-> W5500 (no router). PC Ethernet gets 169.254.x.x (APIPA). */
/* Board IP 169.254.52.10 - same subnet as PC (e.g. 169.254.52.100). Ping and open http://169.254.52.10 */
wiz_NetInfo netInfo = {
    .mac = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF},
    .ip = {169, 254, 52, 10},          /* Board IP - use this in browser: http://169.254.52.10 */
    .sn = {255, 255, 0, 0},             /* Subnet for 169.254.x.x */
    .gw = {169, 254, 52, 1},           /* Gateway (not used for direct link) */
    .dns = {8, 8, 8, 8},
#if USE_DHCP
	.dhcp = NETINFO_DHCP
#else
    .dhcp = NETINFO_STATIC
#endif
};

/*************************************************   NO Changes After This   ***************************************************************/

#define W5500_CS_LOW()     HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define W5500_CS_HIGH()    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)
#define W5500_RST_LOW()    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET)
#define W5500_RST_HIGH()   HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET)


extern SPI_HandleTypeDef W5500_SPI;
extern UART_HandleTypeDef LOG_UART;

int _write(int fd, unsigned char *buf, int len) {
  if (fd == 1 || fd == 2) {                     // stdout or stderr ?
    HAL_UART_Transmit(&LOG_UART, buf, len, 999);  // Print to the UART
  }
  return len;
}

// SPI transmit/receive
void W5500_Select(void)   { W5500_CS_LOW(); }
void W5500_Unselect(void) { W5500_CS_HIGH(); }

uint8_t W5500_ReadByte(void)
{
    uint8_t rx;
    uint8_t tx = 0xFF;
    HAL_SPI_TransmitReceive(&W5500_SPI, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

void W5500_WriteByte(uint8_t byte)
{
    HAL_SPI_Transmit(&W5500_SPI, &byte, 1, HAL_MAX_DELAY);
}


#if USE_DHCP
volatile bool ip_assigned = false;
#define DHCP_SOCKET   7  // last available socket

uint8_t DHCP_buffer[548];


void Callback_IPAssigned(void) {
    ip_assigned = true;
}

void Callback_IPConflict(void) {
    ip_assigned = false;
}
#endif

#define DNS_SOCKET	  6  // 2nd last socket
uint8_t DNS_buffer[512];

int W5500_Init(void)
{
    uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};

    /***** Reset Sequence  *****/
    W5500_RST_LOW();
    HAL_Delay(50);
    W5500_RST_HIGH();
    HAL_Delay(200);

    /***** Register callbacks  *****/
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);

    /***** Initialize the chip  *****/
    if (ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1){
    	printf("Error while initializing WIZCHIP\r\n");
    	return -1;
    }
    printf("WIZCHIP Initialized\r\n");

    /***** check communication by reading Version  *****/
    uint8_t ver = getVERSIONR();
    if (ver != 0x04){
    	printf("Error Communicating with W5500\t Version: 0x%02X\r\n", ver);
    	return -2;
    }
    printf("Checking Link Status..\r\n");
    ctlwizchip(CW_RESET_PHY, NULL);
    HAL_Delay(800);

 	/*****  Check Link Status (wait up to ~25 s for cable / negotiation) *****/
    uint8_t link = PHY_LINK_OFF;
    uint16_t retries = 50;
    while ((link != PHY_LINK_ON) && (retries > 0)){
        ctlwizchip(CW_GET_PHYLINK, &link);
        if (link == PHY_LINK_ON) {
            printf("Link: UP\r\n");
            break;
        }
        if (retries % 10 == 0)
            printf("Link: DOWN Retrying : %d (plug cable before power-on)\r\n", 50 - (int)retries);
        retries--;
        HAL_Delay(500);
    }
    if (link != PHY_LINK_ON){
    	printf ("Link is Down. Plug cable PC<->board, use another cable, then power cycle.\r\n");
    	return 3;
    }
    HAL_Delay(200);  /* let PHY settle before setting IP */

    /***** Use DHCP or Static IP  *****/
#if USE_DHCP
    printf ("Using DHCP.. Please Wait..\r\n");
    setSHAR(netInfo.mac);
    DHCP_init(DHCP_SOCKET, DHCP_buffer);

    reg_dhcp_cbfunc(Callback_IPAssigned, Callback_IPAssigned, Callback_IPConflict);

    retries = 20;
    while((!ip_assigned) && (retries > 0)) {
        DHCP_run();
        HAL_Delay(500);
        retries--;
    }
    if(!ip_assigned) {
    	// DHCP Failed, switch to static IP
    	printf ("DHCP Failed, switching to static IP\r\n");
    	ctlnetwork(CN_SET_NETINFO, (void*)&netInfo);
    }
    else {
    	// if IP is allocated, read it
        getIPfromDHCP(netInfo.ip);
        getGWfromDHCP(netInfo.gw);
        getSNfromDHCP(netInfo.sn);
        getDNSfromDHCP(netInfo.dns);

        // Now apply them to the chip
        ctlnetwork(CN_SET_NETINFO, (void*)&netInfo);
        printf("DHCP IP assigned successfully\r\n");
    }

#else
    // use static IP (Not DHCP)
    printf ("Using Static IP..\r\n");
    ctlnetwork(CN_SET_NETINFO, (void*)&netInfo);
#endif

    /***** Configure DNS  *****/
    HAL_Delay(100);
    printf("Configuring DNS..\r\n");
    DNS_init(DNS_SOCKET, DNS_buffer);

    /* Re-apply static IP so chip definitely has 169.254.52.10 (avoids IP: 0.0.0.0 / socket open fail) */
    ctlnetwork(CN_SET_NETINFO, (void*)&netInfo);
    HAL_Delay(50);

    /***** Print assigned IP on the console  *****/
    wiz_NetInfo tmpInfo;
    ctlnetwork(CN_GET_NETINFO, &tmpInfo);
    printf("IP: %d.%d.%d.%d\r\n", tmpInfo.ip[0], tmpInfo.ip[1], tmpInfo.ip[2], tmpInfo.ip[3]);
    printf("SUBNET: %d.%d.%d.%d\r\n", tmpInfo.sn[0], tmpInfo.sn[1], tmpInfo.sn[2], tmpInfo.sn[3]);
    printf("GATEWAY: %d.%d.%d.%d\r\n", tmpInfo.gw[0], tmpInfo.gw[1], tmpInfo.gw[2], tmpInfo.gw[3]);
    printf("DNS: %d.%d.%d.%d\r\n", tmpInfo.dns[0], tmpInfo.dns[1], tmpInfo.dns[2], tmpInfo.dns[3]);
    return 0;
}

void W5500_ApplyNetInfo(void)
{
    ctlnetwork(CN_SET_NETINFO, (void*)&netInfo);
}
