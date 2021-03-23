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
static void Generate_TX_Message( PKE_PACKET_MANAGER dev, KE_CP_OP_CODES cmd );
static void clear_diagnostics( PKE_PACKET_MANAGER dev );
static void flush_tx_buffer( PKE_PACKET_MANAGER dev );
static void flush_rx_buffer( PKE_PACKET_MANAGER dev );
static void clear_pid_entries( PKE_PACKET_MANAGER dev );
static void reset_idle_time( PKE_PACKET_MANAGER dev );

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

                /* Abort the tx message and increment the transmission abort counter */
                dev->diagnostic.tx_abort_count++;
            }
        } else {

            reset_idle_time( dev );

            dev->status_flags |= KE_NEW_DATA;

            if( dev->status_flags & KE_NEW_DATA )
            {
                /* There are no pending message, send the new data */
                Generate_TX_Message(dev, KE_PID_STREAM_REPORT);

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

            /* The active cooling byte is optional in an ACK */
            if( dev->rx_byte_count == 0x05 )
                dev->init.cooling( dev->rx_buffer[3] );

            break;

        case KE_NACK:
            //TODO
            break;

        case KE_POWER_CYCLE:

            /* Acknowledge the message */
            Generate_TX_Message( dev, KE_ACK  );

            /* System is shutting down */
            KE_Initialize( dev );

            /* Indicate the system rebooted */
            dev->status_flags |= KE_SYSTEM_REBOOT;

            break;

        case KE_SYS_READY:

            /* Acknowledge the message */
            Generate_TX_Message( dev, KE_ACK  );

            /* ACK the successfully received message */
            dev->status_flags |= KE_SYSTEM_READY;

            break;

        case KE_FIRMWARE_REQ:

            /* Report the firmware */
            Generate_TX_Message( dev, KE_FIRMWARE_REPORT  );


            dev->status_flags &= ~KE_STREAM_ACTIVE;

            break;

        case KE_HEARTBEAT:

            Generate_TX_Message( dev, KE_ACK  );

            break;

        case KE_PID_STREAM_NEW:

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
                    tmp_pid.mode      =  dev->rx_buffer[((i*BYTES_PER_STREAM_REQ) + 2) + KE_PCKT_DATA_START_POS];
                    tmp_pid.pid       = (dev->rx_buffer[((i*BYTES_PER_STREAM_REQ) + 3) + KE_PCKT_DATA_START_POS] & 0xFF) << 8;
                    tmp_pid.pid      |= (dev->rx_buffer[((i*BYTES_PER_STREAM_REQ) + 4) + KE_PCKT_DATA_START_POS] & 0xFF);
                    dev->stream[i] = dev->init.req_pid( &tmp_pid );
                }

                dev->status_flags |= KE_STREAM_ACTIVE;

                dev->status_flags |= KE_PID_UPDATED;

                dev->status_flags &= ~KE_PENDING_ACK;
            }

            Generate_TX_Message(  dev, KE_ACK  );

            break;

        default:
            return KE_ERROR;
            break;
    }

    return KE_OK;
}

KE_STATUS KE_Add_UART_Byte( PKE_PACKET_MANAGER dev, uint8_t byte )
{
    /* Look for a start of line byte */
    if( byte == KE_SOL )
    {
        /* Check if a current RX is in progress */
        if ( dev->status_flags & KE_RX_IN_PROGRESS )
        {
            /* Increment the number of aborted RX messages */
            dev->diagnostic.rx_abort_count++;
        }

        /* Start of a new message, reset the buffer */
        memset( dev->rx_buffer, 0, KE_MAX_PAYLOAD );

        /* Reset the byte count */
        dev->rx_byte_count = 0x00;

        /* Add the byte to the buffer */
        dev->rx_buffer[dev->rx_byte_count++] = byte;

        /* Indicate an RX is in progress */
        dev->status_flags |= KE_RX_IN_PROGRESS;

        return KE_START_OF_NEW_MSG;
    }

    /* A Message is in progress */
    else if ( dev->status_flags & KE_RX_IN_PROGRESS )
    {
        /* Verify the UART buffer has room */
        if( dev->rx_byte_count >= KE_MAX_PAYLOAD )
        {
            /* Increment the number of aborted RX messages */
            dev->diagnostic.rx_abort_count++;

            /* Indicate an RX has ended */
            dev->status_flags &= ~KE_RX_IN_PROGRESS;

            /* Reset the UART buffer, something has gone horribly wrong */
            memset( dev->rx_buffer, 0, KE_MAX_PAYLOAD );

            /* Reset the byte count */
            dev->rx_byte_count = 0;

            return KE_BUFFER_FULL;
        }

        /* Add the byte to the buffer */
        dev->rx_buffer[dev->rx_byte_count++] = byte;

        /* See if the message is complete */
        if( dev->rx_byte_count == dev->rx_buffer[ KE_PCKT_LEN_POS ] )
        {
            /* Indicate an RX has ended */
            dev->status_flags &= ~KE_RX_IN_PROGRESS;

            /* Increment the number of received RX messages */
            dev->diagnostic.rx_count++;

            /* Set the Message complete flag */
            dev->status_flags |= KE_PCKT_CMPLT;

            return KE_PACKET_COMPLETE;
        }

        /* This should not have happened, abort! */
        else if ( dev->rx_byte_count > dev->rx_buffer[ KE_PCKT_LEN_POS ] )
        {
            /* Increment the number of aborted RX messages */
            dev->diagnostic.rx_abort_count++;

            /* Indicate an RX has ended */
            dev->status_flags &= ~KE_RX_IN_PROGRESS;

            /* Reset the UART buffer, something has gone horribly wrong */
            memset( dev->rx_buffer, 0, KE_MAX_PAYLOAD );

            /* Reset the byte count */
            dev->rx_byte_count = 0;
        }
        return KE_OK;
    }

    else {
        return KE_OUT_OF_SYNC;
    }
    return KE_ERROR;
}

static void Generate_TX_Message(  PKE_PACKET_MANAGER dev, KE_CP_OP_CODES cmd )
{
    /* Clear the buffer */
    flush_tx_buffer( dev );

    /* Populate the Start of Line byte */
    dev->tx_buffer[KE_PCKT_SOL_POS] = KE_SOL;

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

                if( dev->stream[i]->timestamp == 0 ) {
                    value = 0;
                } else if( dev->stream[i]->pid_unit != dev->stream[i]->base_unit ) {
                    units = convert_units( dev->stream[i]->base_unit, dev->stream[i]->pid_unit, &value );
                }

                /* Data stream format: <pid>:<units>:<value> */

                /* Check if this is a 2 byte PID */
                if( ((dev->stream[i]->pid >> 8) & 0xFF) || 0 )
                    dev->tx_byte_count += snprintf((char*)(&dev->tx_buffer[dev->tx_byte_count]), KE_MAX_PAYLOAD ,
                            "0x%02X%04X:%u:%.2f", dev->stream[i]->mode, (uint16_t)(dev->stream[i]->pid), units, value);

                /* If not, assume it is a single byte PID */
                else
                    dev->tx_byte_count += snprintf((char*)(&dev->tx_buffer[dev->tx_byte_count]), KE_MAX_PAYLOAD ,
                            "0x%02X%02X:%u:%.2f", dev->stream[i]->mode, (uint8_t)(dev->stream[i]->pid & 0xFF), units, value);

                /* Add a semi-colon after every PID except the last */
                if( i < dev->num_pids - 1 )
                    dev->tx_buffer[dev->tx_byte_count++] = ',';
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
            dev->tx_byte_count += snprintf( (char*)(&dev->tx_buffer[dev->tx_byte_count]), KE_MAX_PAYLOAD , "%02d.%02d.%02d",
                    dev->init.firmware_version_major ,
                    dev->init.firmware_version_minor,
                    dev->init.firmware_version_hotfix );
            break;
        default:
            break;
    }

    /* Packet is complete */
    dev->tx_buffer[ dev->tx_byte_count++ ] = KE_EOL;

    /* Populate the length */
    dev->tx_buffer[KE_PCKT_LEN_POS] = dev->tx_byte_count;

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
    memset( dev->tx_buffer, 0, KE_MAX_PAYLOAD );

    /* Reset the byte count */
    dev->tx_byte_count = 0;
}

static void flush_rx_buffer( PKE_PACKET_MANAGER dev )
{
    /* Clear the buffer */
    memset( dev->rx_buffer, 0, KE_MAX_PAYLOAD );

    /* Reset the byte count */
    dev->rx_byte_count = 0;
}

static void clear_pid_entries( PKE_PACKET_MANAGER dev )
{
    for( uint8_t i = 0; i < KE_MAX_PIDS; i++ )
    {
        if( dev->stream[i] != NULL )
        {
            dev->init.clear_pid( dev->stream[i] );
            dev->stream[i] = NULL;
        }
    }

    /* Reset the byte count */
    dev->num_pids = 0;
}
