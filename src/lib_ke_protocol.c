/*
 * lib_ke_protocol.c
 *
 *  Created on: Aug 20, 2019
 *      Author: Matthew Kaiser
 */

#include "lib_ke_protocol.h"
#include "string.h"
#include "stdio.h"

static uint32_t ke_tick = 0;

static KE_STATUS KE_Process_Packet( PKE_PACKET_MANAGER dev );
static void Generate_TX_Message( PKE_PACKET_MANAGER dev, KE_CP_OP_CODES cmd, uint32_t arg );
static void clear_diagnostics( PKE_PACKET_MANAGER dev );
static void flush_tx_buffer( PKE_PACKET_MANAGER dev );
static void flush_rx_buffer( PKE_PACKET_MANAGER dev );
static void clear_pid_entries( PKE_PACKET_MANAGER dev );
static void reset_idle_time( PKE_PACKET_MANAGER dev );
static uint8_t crc8(const uint8_t *data, size_t len);

uint32_t get_KE_rx_count( PKE_PACKET_MANAGER dev )
{
    return dev->diagnostic.rx_count;
}

uint32_t get_KE_rx_abort_count( PKE_PACKET_MANAGER dev )
{
    return dev->diagnostic.rx_abort_count;
}

uint32_t get_KE_tx_abort_count( PKE_PACKET_MANAGER dev )
{
    return dev->diagnostic.tx_abort_count;
}

KE_STATUS KE_Initialize( PKE_PACKET_MANAGER dev )
{
    for( uint8_t i = 0; i < KE_MAX_PIDS; i++ )
        dev->stream[i] = NULL;
    dev->status_flags = 0;
    dev->num_retries = 0;
    clear_diagnostics( dev );
    flush_tx_buffer( dev );
    flush_rx_buffer( dev );
    clear_pid_entries( dev );
    dev->ke_time = ke_tick;
    return KE_OK;
}

KE_STATUS KE_Service( PKE_PACKET_MANAGER dev )
{
    /*********************************************************
     * Packet has been received and is ready for
     * processing.
     *********************************************************/
    if( dev->status_flags & KE_PCKT_CMPLT )
    {
        KE_Process_Packet(dev);

        dev->num_retries = 0;

        dev->status_flags &= ~KE_PCKT_CMPLT;

        reset_idle_time( dev );
    }

    /* See if the stream is active */
    else if( dev->status_flags & KE_STREAM_ACTIVE )
    {
        /* If so, see if a packet has been sent and is awaiting acknowledgment */
        if( dev->status_flags & KE_PENDING_ACK )
        {
            /* Verify the message hasn't timed out */
            if( ke_tick > (dev->ke_time + KE_TIMEOUT) )
            {
                /* If so, clear the pending ack flag in order to re-send the data */
                dev->status_flags &= ~KE_PENDING_ACK;

                /* Increment the consecutive retry count */
                dev->num_retries++;

                if( dev->num_retries > MAX_RETRIES )
                {
                    dev->num_retries = 0;
                    clear_pid_entries( dev );
                }

                /* Abort the tx message and increment the transmission abort counter */
                dev->diagnostic.tx_abort_count++;
            }
        } else {

            reset_idle_time( dev );

            dev->status_flags |= KE_NEW_DATA;

            if( dev->status_flags & KE_NEW_DATA )
            {
                /* There are no pending message, send the new data */
                Generate_TX_Message(dev, KE_PID_STREAM_REPORT, 0);

                dev->status_flags |= KE_PENDING_ACK;
            }
        }
    }

    if( dev->status_flags & KE_PID_UPDATED )
    {
        dev->status_flags &= ~KE_PID_UPDATED;
        return KE_PID_REQ_UPDATE;
    } else if ( dev->status_flags & KE_SYSTEM_REBOOT )
    {
        dev->status_flags &= ~KE_SYSTEM_REBOOT;
        return KE_REBOOT;
    } else {
        return KE_OK;
    }
}

static KE_STATUS KE_Process_Packet( PKE_PACKET_MANAGER dev )
{
    switch( dev->rx_buffer[KE_PCKT_CMD_POS] )
    {
        case KE_ACK:
            /* ACK received, clear the pending ACK flag */
            dev->status_flags &= ~KE_PENDING_ACK;

			#if FAN_CTRL_ACTIVE
            /* The active cooling byte is optional in an ACK */
            if( dev->rx_byte_count == 0x05 )
            	if( dev->init.cooling != NULL )
            		dev->init.cooling( dev->rx_buffer[3] );
			#endif

            break;

        case KE_NACK:
            //TODO
            break;

        case KE_POWER_CYCLE:

            /* Acknowledge the message */
            Generate_TX_Message( dev, KE_ACK, 0 );

            /* System is shutting down */
            KE_Initialize( dev );

            /* Indicate the system rebooted */
            dev->status_flags |= KE_SYSTEM_REBOOT;

            break;

        case KE_SYS_READY:

            /* Acknowledge the message */
            Generate_TX_Message( dev, KE_ACK, 0 );

            /* ACK the successfully received message */
            dev->status_flags |= KE_SYSTEM_READY;

            break;

        case KE_FIRMWARE_REQ:

            /* Report the firmware */
            Generate_TX_Message( dev, KE_FIRMWARE_REPORT, 0 );


            dev->status_flags &= ~KE_STREAM_ACTIVE;

            break;

        case KE_HEARTBEAT:

            Generate_TX_Message( dev, KE_ACK, 0 );

            break;

        case KE_PID_STREAM_NEW:

            /* TODO for now an empty request will act like a heartbeat */
            if( dev->rx_byte_count == 3 )
                Generate_TX_Message( dev, KE_ACK, 0 );

            /* Make sure the packet has data */
            if( dev->rx_byte_count > KE_PCKT_DATA_START_POS )
            {
                clear_pid_entries( dev );

                //TODO verify there aren't too many PIDs
                dev->num_pids = (dev->rx_byte_count - KE_PCKT_DATA_START_POS) / BYTES_PER_STREAM_REQ;

                PID_DATA tmp_pid;

                for( uint8_t i = 0; i < dev->num_pids; i++)
                {
                    tmp_pid.pid_unit  =  dev->rx_buffer[((i*BYTES_PER_STREAM_REQ) + 1) + KE_PCKT_DATA_START_POS];
                    tmp_pid.pid_uuid =
                        ((uint32_t)dev->rx_buffer[((i * BYTES_PER_STREAM_REQ) + 2) + KE_PCKT_DATA_START_POS] << 24) |
                        ((uint32_t)dev->rx_buffer[((i * BYTES_PER_STREAM_REQ) + 3) + KE_PCKT_DATA_START_POS] << 16) |
                        ((uint32_t)dev->rx_buffer[((i * BYTES_PER_STREAM_REQ) + 4) + KE_PCKT_DATA_START_POS] << 8)  |
                        ((uint32_t)dev->rx_buffer[((i * BYTES_PER_STREAM_REQ) + 5) + KE_PCKT_DATA_START_POS]);
                    dev->stream_unit[i] = tmp_pid.pid_unit;
                    if( dev->init.req_pid != NULL )
                    	dev->stream[i] = dev->init.req_pid( &tmp_pid );
                }

                dev->status_flags |= KE_STREAM_ACTIVE;

                dev->status_flags |= KE_PID_UPDATED;

                dev->status_flags &= ~KE_PENDING_ACK;
            }

            Generate_TX_Message(  dev, KE_PID_STREAM_REPORT, 0 );

            break;

        default:
            return KE_ERROR;
            break;
    }

    return KE_OK;
}

KE_STATUS KE_Add_UART_Byte( PKE_PACKET_MANAGER dev, uint8_t byte )
{
    // Add the byte to the buffer
    if (dev->rx_byte_count < KE_MAX_RX_PAYLOAD) {
        dev->rx_buffer[dev->rx_byte_count++] = byte;
    } else {
        dev->diagnostic.rx_abort_count++;
        dev->status_flags &= ~KE_RX_IN_PROGRESS;
        dev->rx_byte_count = 0;
        return KE_BUFFER_FULL;
    }

    // Check for SOL sequence using a sliding window
    if (dev->rx_byte_count >= 4) {
        int i = dev->rx_byte_count - 4;
        if (dev->rx_buffer[i + 0] == KE_SOL_BYTE0 &&
            dev->rx_buffer[i + 1] == KE_SOL_BYTE1 &&
            dev->rx_buffer[i + 2] == KE_SOL_BYTE2 &&
            dev->rx_buffer[i + 3] == KE_SOL_BYTE3)
        {
            // Found SOL — restart buffer from this point
            if (dev->status_flags & KE_RX_IN_PROGRESS) {
                dev->diagnostic.rx_abort_count++;
            }

            // Shift SOL to index 0
            memmove(dev->rx_buffer, &dev->rx_buffer[i], dev->rx_byte_count - i);
            dev->rx_byte_count = dev->rx_byte_count - i;

            dev->status_flags |= KE_RX_IN_PROGRESS;
            dev->status_flags &= ~KE_PCKT_CMPLT;

            return KE_START_OF_NEW_MSG;
        }
    }

    // If already receiving, check for message complete
    if (dev->status_flags & KE_RX_IN_PROGRESS) {

    	// Verify the length data has been rx'd
    	if(dev->rx_byte_count <= KE_PCKT_LEN_BYTE3_POS)
    		return KE_OK;

    	uint32_t len = ((uint32_t)dev->rx_buffer[KE_PCKT_LEN_BYTE0_POS] << 24) |
    	               ((uint32_t)dev->rx_buffer[KE_PCKT_LEN_BYTE1_POS] << 16) |
    	               ((uint32_t)dev->rx_buffer[KE_PCKT_LEN_BYTE2_POS] << 8)  |
    	               ((uint32_t)dev->rx_buffer[KE_PCKT_LEN_BYTE3_POS]);

        if (dev->rx_byte_count == len)
        {
            dev->status_flags &= ~KE_RX_IN_PROGRESS;
            dev->diagnostic.rx_count++;
            dev->status_flags |= KE_PCKT_CMPLT;
            return KE_PACKET_COMPLETE;
        }

        else if (dev->rx_byte_count > len) {
            dev->diagnostic.rx_abort_count++;
            dev->status_flags &= ~KE_RX_IN_PROGRESS;
            dev->rx_byte_count = 0;
            return KE_OUT_OF_SYNC;;
        }

        return KE_OK;
    }

    return KE_OK;
}

static void Generate_TX_Message(  PKE_PACKET_MANAGER dev, KE_CP_OP_CODES cmd, uint32_t arg )
{
    /* Clear the buffer */
    flush_tx_buffer( dev );

    /* Populate the Start of Line bytes */
    dev->tx_buffer[KE_PCKT_SOL_BYTE0_POS] = KE_SOL_BYTE0;
    dev->tx_buffer[KE_PCKT_SOL_BYTE1_POS] = KE_SOL_BYTE1;
    dev->tx_buffer[KE_PCKT_SOL_BYTE2_POS] = KE_SOL_BYTE2;
    dev->tx_buffer[KE_PCKT_SOL_BYTE3_POS] = KE_SOL_BYTE3;

    /* Command */
    dev->tx_buffer[KE_PCKT_CMD_POS] = cmd;

    /* Align the buffer to start of the data bytes */
    dev->tx_byte_count = KE_PCKT_DATA_START_POS;

    //XXX The MCU may never need to send any data.

    /* Populate supporting data */
    switch( cmd )
    {
        case KE_ACK:
            /* No additional data necessary */
            break;
        case KE_NACK:
            /* No additional data necessary */
            break;
        case KE_HEARTBEAT:
            /* No additional data necessary */
            break;
        case KE_SYS_READY:
            /* No additional data necessary */
            break;
        case KE_PID_STREAM_NEW:
            /*TODO: Add support to be a host */
            break;
        case KE_PID_STREAM_ADD:
            /*TODO: Add support to be a host */
            break;
        case KE_PID_STREAM_REMOVE:
            /*TODO: Add support to be a host */
            break;
        case KE_PID_STREAM_CLEAR:
            /*TODO: Add support to be a host */
            break;
        case KE_PID_STREAM_REPORT:
            for( uint8_t i = 0; i < dev->num_pids; i++)
            {
                float value = dev->stream[i]->pid_value;
                uint8_t units = dev->stream[i]->base_unit;
                uint8_t remaining_delim = dev->num_pids - 1;

                if( dev->stream[i]->timestamp == 0 ) {
                    remaining_delim--;
                } else {
                    // TODO remove convert_units
                    //if( dev->stream_unit[i] != dev->stream[i]->base_unit ) {
                    //    units = convert_units( dev->stream[i]->base_unit, dev->stream_unit[i], &value );
                    //}

                        /* Data stream format: <pid>:<units>:<value> */

                        /* Check if this is a 2 byte PID */
                        if( ((dev->stream[i]->pid_uuid >> 8) & 0xFF) || 0 )
                            dev->tx_byte_count += snprintf((char*)(&dev->tx_buffer[dev->tx_byte_count]), KE_MAX_TX_PAYLOAD ,
                                    "0x%02X%04X:%u:%.2f", (uint8_t)(dev->stream[i]->pid_uuid >> 16), (uint16_t)(dev->stream[i]->pid_uuid), units, value);

                        /* If not, assume it is a single byte PID */
                        else
                            dev->tx_byte_count += snprintf((char*)(&dev->tx_buffer[dev->tx_byte_count]), KE_MAX_TX_PAYLOAD ,
                                    "0x%02X%02X:%u:%.2f", (uint8_t)(dev->stream[i]->pid_uuid >> 16), (uint8_t)(dev->stream[i]->pid_uuid & 0xFF), units, value);

                        /* Add a semi-colon after every PID except the last */
                        if( i < remaining_delim )
                            dev->tx_buffer[dev->tx_byte_count++] = ',';
                }
            }
            break;
        case KE_LCD_ENABLE:
            /*TODO: Add support to be a host */
            break;
        case KE_LCD_DISABLE:
            /*TODO: Add support to be a host */
            break;
        case KE_LCD_POWER_CYCLE:
            /*TODO: Add support to be a host */
            break;
        case KE_LCD_FORCE_BRIGHTNESS:
            /*TODO: Add support to be a host */
            break;
        case KE_LCD_AUTO_BRIGHTNESS:
            /*TODO: Add support to be a host */
            break;
        case KE_USB_ENABLE:
            /*TODO: Add support to be a host */
            break;
        case KE_USB_DISABLE:
            /*TODO: Add support to be a host */
            break;
        case KE_USB_POWER_CYCLE:
            /*TODO: Add support to be a host */
            break;
        case KE_POWER_ENABLE:
            /*TODO: Add support to be a host */
            break;
        case KE_POWER_DISABLE:
            /* No additional data necessary */
            break;
        case KE_POWER_CYCLE:
            /* No additional data necessary */
            break;
        case KE_FIRMWARE_REQ:
            /*TODO: Add support to be a host */
            break;
        case KE_FIRMWARE_REPORT:
            dev->tx_byte_count += snprintf( (char*)(&dev->tx_buffer[dev->tx_byte_count]), KE_MAX_TX_PAYLOAD , "%02d.%02d.%02d",
                    dev->init.firmware_version_major ,
                    dev->init.firmware_version_minor,
                    dev->init.firmware_version_hotfix );
            break;
        case KE_BACKGROUND_SEND:

        	break;
        case KE_BACKGROUND_RECEIVE:

        	break;
        case KE_CONFIG_SEND:
        	break;
        case KE_CONFIG_RECEIVE:
        	break;
        default:
            break;
    }

    // Calculate CRC over the entire packet before adding CRC byte
    uint8_t crc = crc8(dev->tx_buffer, dev->tx_byte_count);

    /* Packet is complete */
    dev->tx_buffer[ dev->tx_byte_count++ ] = crc;

    uint32_t len = dev->tx_byte_count;

    /* Populate the length */
    dev->tx_buffer[KE_PCKT_LEN_BYTE0_POS] = (len >> 24) & 0xFF;  // Most significant byte
    dev->tx_buffer[KE_PCKT_LEN_BYTE1_POS] = (len >> 16) & 0xFF;
    dev->tx_buffer[KE_PCKT_LEN_BYTE2_POS] = (len >> 8)  & 0xFF;
    dev->tx_buffer[KE_PCKT_LEN_BYTE3_POS] = (len >> 0)  & 0xFF;  // Least significant byte

    /* Send the packet */
    dev->init.transmit( dev->tx_buffer, dev->tx_byte_count );
}

void KE_tick( void )
{
    ke_tick++;
}

static void reset_idle_time(  PKE_PACKET_MANAGER dev )
{
    dev->ke_time = ke_tick;
}

static void clear_diagnostics( PKE_PACKET_MANAGER dev )
{
    dev->diagnostic.tx_abort_count = 0;
    dev->diagnostic.rx_abort_count = 0;
    dev->diagnostic.rx_count       = 0;
}


static void flush_tx_buffer( PKE_PACKET_MANAGER dev )
{
    /* Clear the buffer */
    memset( dev->tx_buffer, 0, KE_MAX_TX_PAYLOAD );

    /* Reset the byte count */
    dev->tx_byte_count = 0;
}

static void flush_rx_buffer( PKE_PACKET_MANAGER dev )
{
    /* Clear the buffer */
    memset( dev->rx_buffer, 0, KE_MAX_RX_PAYLOAD );

    /* Reset the byte count */
    dev->rx_byte_count = 0;
}

static void clear_pid_entries( PKE_PACKET_MANAGER dev )
{
    for( uint8_t i = 0; i < KE_MAX_PIDS; i++ )
    {
        if( dev->stream[i] != NULL )
        {
        	if( dev->init.clear_pid != NULL )
        		dev->init.clear_pid( dev->stream[i] );
            dev->stream[i] = NULL;
            dev->stream_unit[i] = PID_UNITS_RESERVED;
        }
    }

    /* Reset the byte count */
    dev->num_pids = 0;
}

// CRC-8 calculation function (poly 0x07, initial 0x00)
static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}
