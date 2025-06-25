/*
 * lib_ke_protocol.h
 *
 *  Created on: Aug 20, 2019
 *      Author: mkaiser
 */

#ifndef LIB_KE_PROTOCOL_H_
#define LIB_KE_PROTOCOL_H_

#include "stdio.h"
#include "stdint.h"
#include "lib_pid.h"
#include "ke_communicaton_protocol.h"

#ifndef KE_MAX_TX_PAYLOAD
#define KE_MAX_TX_PAYLOAD             0xFFFF + 0xF
#endif
#ifndef KE_MAX_RX_PAYLOAD
#define KE_MAX_RX_PAYLOAD             819200 + 16
#endif
#define KE_MAX_PIDS                   0xF
#define KE_SOL_BYTE0                  0xDE
#define KE_SOL_BYTE1                  0xAD
#define KE_SOL_BYTE2                  0xBE
#define KE_SOL_BYTE3                  0xEF

#define KE_PCKT_SOL_BYTE0_POS        0x00
#define KE_PCKT_SOL_BYTE1_POS        0x01
#define KE_PCKT_SOL_BYTE2_POS        0x02
#define KE_PCKT_SOL_BYTE3_POS        0x03
#define KE_PCKT_LEN_BYTE0_POS        0x04
#define KE_PCKT_LEN_BYTE1_POS        0x05
#define KE_PCKT_LEN_BYTE2_POS        0x06
#define KE_PCKT_LEN_BYTE3_POS        0x07
#define KE_PCKT_CMD_POS              0x08
#define KE_PCKT_DATA_START_POS       0x09

#define BYTES_PER_STREAM_REQ         0x05

#define KE_TIMEOUT                   200
#define MAX_RETRIES                  25

typedef int (*TRANSMIT_DATA)(uint8_t *data, uint8_t len);
typedef void (*REQUEST_ACTIVE_COOLING)(uint8_t level);

typedef enum _ke_status {
    KE_ERROR,
    KE_START_OF_NEW_MSG,
    KE_BUFFER_FULL,
    KE_OUT_OF_SYNC,
    KE_PACKET_COMPLETE,
    KE_PID_REQ_UPDATE,
    KE_REBOOT,
    KE_SYSTEM_READY,
    KE_OK
} KE_STATUS, *PKE_STATUS;

typedef enum _ke_dev_role {
    KE_PRIMARY,
	KE_SECONDARY
} KE_DEV_ROLE, *PKE_DEV_ROLE;

typedef struct _kep_init {
	KE_DEV_ROLE role;
    TRANSMIT_DATA transmit;
    clear_pid_request clear_pid;
    request_pid_data req_pid;
    REQUEST_ACTIVE_COOLING cooling;
    uint8_t firmware_version_major;
    uint8_t firmware_version_minor;
    uint8_t firmware_version_hotfix;
} KEP_INIT, *PKEP_INIT;

typedef struct _kep_diagnostics {
    uint32_t tx_abort_count;
    uint32_t rx_abort_count;
    uint32_t rx_count;
} KEP_DIAGNOSTICS, *PKEP_DIAGNOSTICS;

typedef struct _ke_packet_manager {
    uint8_t stream_unit[KE_MAX_PIDS];
	PTR_PID_DATA stream[KE_MAX_PIDS];
    uint32_t ke_time;
    KEP_INIT init;
    KEP_DIAGNOSTICS diagnostic;
    uint32_t num_retries;
    uint16_t status_flags;
        #define KE_RX_IN_PROGRESS  (1 << 0)
        #define KE_PCKT_CMPLT      (1 << 1)
        #define KE_PENDING_ACK     (1 << 2)
        #define KE_SYSTEM_READY    (1 << 3)
        #define KE_STREAM_ACTIVE   (1 << 4)
        #define KE_PID_UPDATED     (1 << 5)
        #define KE_SYSTEM_REBOOT   (1 << 6)
        #define KE_NEW_DATA        (1 << 7)
    uint8_t tx_buffer[KE_MAX_TX_PAYLOAD];
    uint32_t tx_byte_count;
    uint8_t rx_buffer[KE_MAX_RX_PAYLOAD];
    uint32_t rx_byte_count;
    uint8_t num_pids;
} KE_PACKET_MANAGER, *PKE_PACKET_MANAGER;

void KE_tick( void );
KE_STATUS KE_Initialize( PKE_PACKET_MANAGER dev );
KE_STATUS KE_Service( PKE_PACKET_MANAGER dev );
KE_STATUS KE_Add_UART_Byte( PKE_PACKET_MANAGER dev, uint8_t byte );

uint32_t get_KE_rx_count( PKE_PACKET_MANAGER dev );
uint32_t get_KE_rx_abort_count( PKE_PACKET_MANAGER dev );
uint32_t get_KE_tx_abort_count( PKE_PACKET_MANAGER dev );

void Generate_TX_Message(  PKE_PACKET_MANAGER dev, KE_CP_OP_CODES cmd, uint32_t arg );



#endif /* LIB_KE_PROTOCOL_H_ */
