#pragma once

#include <math.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "hal/uart_ll.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RX_BUF_SIZE      8
#define TX_BUF_SIZE      8
#define RESP_BUF_SIZE    25
#define UPDATE_TIME      200

typedef struct pz_conf_t {
    uart_port_t pzem_uart;
    uint8_t pzem_rx_pin;
    uint8_t pzem_tx_pin;
    uint8_t pzem_addr;
} pzem_setup_t;

/***
 * https://en.wikipedia.org/wiki/AC_power
*/
typedef struct _current_values {
    float voltage;
    float current;
    float power;            // Active Power P. or Real Power W.
    float energy;
    float frequency;
    float pf;               // Ratio of active to apparent power, cos(fi), eg pf = 0.77, 77% of current is doing the real work
    float apparent_power;   // product of RMS values (VA)
    float reactive_power;   // Reactive Power Q. exist when V and I are not in Phase. (VAr)
    float fi;               // Phase angle between S and P (Apparent and Real).
    uint16_t alarms;
} _current_values_t;         /* Measured values */

void PzemInit( pzem_setup_t *pzSetup );
bool PzemCheckCRC( const uint8_t *buf, uint16_t len );
uint16_t PzemReceive( pzem_setup_t *pzSetup, uint8_t *resp, uint16_t len );
bool PzemSendCmd8( pzem_setup_t *pzSetup, uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slave_addr );
void PzemSetCRC( uint8_t *buf, uint16_t len );
bool PzemGetValues( pzem_setup_t *pzSetup, _current_values_t *pmonValues );
uint8_t PzReadAddress( pzem_setup_t *pzSetup);
bool PzResetEnergy( pzem_setup_t *pzSetup );
void PzemZeroValues( _current_values_t *currentValues );
bool PzSetAddress(pzem_setup_t *pzSetup, uint8_t new_addr);

#define millis( x )              ( esp_timer_get_time( x ) / 1000 )
//#define UART_LL_GET_HW( num )    ( ( ( num ) == 0 ) ? ( &UART0 ) : ( ( ( num ) == 1 ) ? ( &UART1 ) : ( &UART2 ) ) )


#define PZ_DEFAULT_ADDRESS    0xF8
#define PZ_BAUD_RATE          9600
#define PZ_READ_TIMEOUT       100

/*
 * REGISTERS
 */
#define RG_VOLTAGE            0x0000
#define RG_CURRENT_L          0x0001 /* Lo Bit */
#define RG_CURRENT_H          0X0002 /* Hi Bit */
#define RG_POWER_L            0x0003 /* Lo Bit */
#define RG_POWER_H            0x0004 /* Hi Bit */
#define RG_ENERGY_L           0x0005 /* Lo Bit */
#define RG_ENERGY_H           0x0006 /* Hi Bit */
#define RG_FREQUENCY          0x0007
#define RG_PF                 0x0008
#define RG_ALARM              0x0009

#define CMD_RHR               0x03
#define CMD_RIR               0X04
#define CMD_WSR               0x06
#define CMD_CAL               0x41
#define CMD_REST              0x42

#define WREG_ALARM_THR        0x0001
#define WREG_ADDR             0x0002

#define INVALID_ADDRESS       0x00

#define PROGMEM    /* empty */
#define pgm_read_byte( x )     ( *( x ) )
#define pgm_read_word( x )     ( *( x ) )
#define pgm_read_float( x )    ( *( x ) )
#define PSTR( STR )            STR

/* Pre Calculated CRC lookup table */
/* source: https://www.modbustools.com/modbus_crc16.html */

static const DRAM_ATTR uint16_t crcTable[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};


#ifdef __cplusplus
}
#endif
