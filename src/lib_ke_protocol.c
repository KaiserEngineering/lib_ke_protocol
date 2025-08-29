/*
 * lib_ke_protocol.c
 *
 *  Created on: Aug 20, 2019
 *      Author: Matthew Kaiser
 */

#include "lib_ke_protocol.h"
#include "string.h"
#include "stdio.h"

#define DEBUG_LIB_KE_PROTOCOL 1
#if DEBUG_LIB_KE_PROTOCOL

    #ifdef ESP_PLATFORM
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #define LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
    #define LOGE(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)
	static const char *TAG = "KE";
    #else
    // Define no-op or alternative logging for non-ESP platforms
    #define LOGI(tag, fmt, ...)
    #define LOGE(tag, fmt, ...)
    #endif
#else
    // Define no-op
    #define LOGI(tag, fmt, ...)
    #define LOGE(tag, fmt, ...)
#endif

static uint32_t ke_tick = 0;

static KE_STATUS KE_Process_Packet( PKE_PACKET_MANAGER dev );
static void clear_diagnostics( PKE_PACKET_MANAGER dev );
static void flush_tx_buffer( PKE_PACKET_MANAGER dev );
static void flush_rx_buffer( PKE_PACKET_MANAGER dev );
static void clear_pid_entries( PKE_PACKET_MANAGER dev );
static void reset_idle_time( PKE_PACKET_MANAGER dev );
static uint8_t crc8(const uint8_t *data, size_t len);

static inline void KE_set_flag(PKE_PACKET_MANAGER dev, uint32_t flag) {
    dev->status_flags |= flag;
}

static inline void KE_clear_flag(PKE_PACKET_MANAGER dev, uint32_t flag) {
    dev->status_flags &= ~flag;
}

static inline bool KE_get_flag(PKE_PACKET_MANAGER dev, uint32_t flag) {
    return (dev->status_flags & flag) != 0;
}

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
    if( KE_get_flag(dev, KE_PCKT_CMPLT) )
    {
        KE_Process_Packet(dev);

        dev->num_retries = 0;

        KE_clear_flag(dev, KE_PCKT_CMPLT);

        reset_idle_time( dev );
    }

    /* See if the stream is active */
    else if( KE_get_flag(dev, KE_STREAM_ACTIVE) )
    {
        /* If so, see if a packet has been sent and is awaiting response */
        if( KE_get_flag(dev, KE_PENDING_RESPONSE) )
        {
            /* Verify the message hasn't timed out */
            if( ke_tick > (dev->ke_time + KE_TIMEOUT) )
            {
                /* If so, clear the pending response flag in order to re-send the data */
                KE_clear_flag(dev, KE_PENDING_RESPONSE);

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

            KE_set_flag(dev, KE_NEW_DATA);

            if( KE_get_flag(dev, KE_NEW_DATA) )
            {
                /* There are no pending message, send the new data */
                Generate_TX_Message(dev, KE_PID_STREAM_REPORT, 0);
            }
        }
    }

    if( KE_get_flag(dev, KE_PID_UPDATED) )
    {
        KE_clear_flag(dev, KE_PID_UPDATED);
        return KE_PID_REQ_UPDATE;
    } else if ( KE_get_flag(dev, KE_SYSTEM_REBOOT) )
    {
        KE_clear_flag(dev, KE_SYSTEM_REBOOT);
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
            /* response received, clear the pending response flag */
            KE_clear_flag(dev, KE_PENDING_RESPONSE);

			#if FAN_CTRL_ACTIVE
            /* The active cooling byte is optional in an ACK */
            if( dev->rx_byte_count == 0x05 )
            	if( dev->init.cooling != NULL )
            		dev->init.cooling( dev->rx_buffer[3] );
			#endif

            LOGI(TAG, "ACK Received");
            break;

        case KE_NACK:
            //TODO
            LOGI(TAG, "NACK Received");
            break;

        case KE_POWER_CYCLE:

            /* Acknowledge the message */
            Generate_TX_Message( dev, KE_ACK, 0 );

            /* System is shutting down */
            KE_Initialize( dev );

            /* Indicate the system rebooted */
            KE_set_flag(dev, KE_SYSTEM_REBOOT);

            LOGI(TAG, "Power Cylce Requested");
            break;

        case KE_SYS_READY:

            /* Acknowledge the message */
            Generate_TX_Message( dev, KE_ACK, 0 );

            /* ACK the successfully received message */
            KE_set_flag(dev, KE_SYSTEM_READY);

            LOGI(TAG, "System Ready Received");
            break;

        case KE_FIRMWARE_REQ:

            /* Report the firmware */
            Generate_TX_Message( dev, KE_FIRMWARE_REPORT, 0 );


            KE_clear_flag(dev, KE_STREAM_ACTIVE);

            LOGI(TAG, "Firmware Requested");
            break;

        case KE_HEARTBEAT:

            Generate_TX_Message( dev, KE_ACK, 0 );

            LOGI(TAG, "Heartbeat Received");
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

                KE_set_flag(dev, KE_STREAM_ACTIVE);

                KE_set_flag(dev, KE_PID_UPDATED);

                KE_clear_flag(dev, KE_PENDING_RESPONSE);
            }

            Generate_TX_Message(  dev, KE_PID_STREAM_REPORT, 0 );

            LOGI(TAG, "New PID Stream Requested");
            break;

        case KE_BACKGROUND_SEND:
        	uint32_t expected_size = 1024 * 200 * 4; // TODO - DO NOT HARDCODE THIS
        	uint32_t start = KE_PCKT_DATA_START_POS + 1;
        	uint32_t end = start + expected_size;
        	if (end > dev->rx_byte_count) {
            	LOGI(TAG, "Image larger than payload");
            	Generate_TX_Message( dev, KE_NACK, 0 );
        	} else if (dev->init.save_rgba) {
        		dev->init.save_rgba((char*)&dev->rx_buffer[start], expected_size, dev->rx_buffer[KE_PCKT_DATA_START_POS]);
        		Generate_TX_Message( dev, KE_ACK, 0 );
        	} else {
            	LOGI(TAG, "No save_rgba() registered.");
            	Generate_TX_Message( dev, KE_NACK, 0 );
        	}
        	break;

        case KE_BACKGROUND_CRC_SEND:
            if(dev->init.receive_rgba_crc) {
                // Cast and dereference
                uint8_t background_idx = dev->rx_buffer[KE_PCKT_DATA_START_POS];
                uint32_t background_crc = ((uint32_t)dev->rx_buffer[KE_PCKT_DATA_START_POS+1] << 24) |
                                          ((uint32_t)dev->rx_buffer[KE_PCKT_DATA_START_POS+2] << 16) |
                                          ((uint32_t)dev->rx_buffer[KE_PCKT_DATA_START_POS+3] << 8)  |
                                          ((uint32_t)dev->rx_buffer[KE_PCKT_DATA_START_POS+4]);

                dev->init.receive_rgba_crc(background_idx, background_crc);
            } else {
            	LOGI(TAG, "No receive_rgba_crc() registered.");
            }

            // KE_BACKGROUND_CRC_REQUEST has been responded to
            KE_clear_flag(dev, KE_PENDING_RESPONSE);
            break;

        case KE_BACKGROUND_CRC_REQUEST:
        	uint8_t idx = dev->rx_buffer[KE_PCKT_DATA_START_POS];
        	Generate_TX_Message(dev, KE_BACKGROUND_CRC_SEND, &idx);
            break;

        case KE_CONFIG_SEND:
            if (dev->init.json_to_config) {
                // Overwrite the CRC with a NULL terminator for the string. (Message has already been verified) 
                dev->rx_buffer[dev->rx_byte_count-1] = '\0';
                dev->init.json_to_config((char*)&dev->rx_buffer[KE_PCKT_DATA_START_POS]);
                Generate_TX_Message( dev, KE_ACK, 0 );
            } else {
            	LOGI(TAG, "No json_to_config() registered.");
            	Generate_TX_Message( dev, KE_NACK, 0 );
            }

            // KE_CONFIG_REQUEST has been responded to
            KE_clear_flag(dev, KE_PENDING_RESPONSE);
            break;

        case KE_CONFIG_REQUEST:
        	Generate_TX_Message(dev, KE_CONFIG_SEND, 0);
            break;

        case KE_OPTION_LIST_SEND:
            if (dev->init.json_to_options) {
                // Overwrite the CRC with a NULL terminator for the string. (Message has already been verified)
                dev->rx_buffer[dev->rx_byte_count-1] = '\0';
                dev->init.json_to_options((char*)&dev->rx_buffer[KE_PCKT_DATA_START_POS]);
                Generate_TX_Message( dev, KE_ACK, 0 );
            } else {
            	LOGI(TAG, "No json_to_options() registered.");
            	Generate_TX_Message( dev, KE_NACK, 0 );
            }

            // KE_OPTION_LIST_REQUEST has been responded to
            KE_clear_flag(dev, KE_PENDING_RESPONSE);
        	break;

        case KE_OPTION_LIST_REQUEST:
        	Generate_TX_Message(dev, KE_OPTION_LIST_SEND, 0);
        	break;

        case KE_PID_LIST_SEND:
            if (dev->init.json_to_pid_list) {
                // Overwrite the CRC with a NULL terminator for the string. (Message has already been verified)
                dev->rx_buffer[dev->rx_byte_count-1] = '\0';
                dev->init.json_to_pid_list((char*)&dev->rx_buffer[KE_PCKT_DATA_START_POS]);
                Generate_TX_Message( dev, KE_ACK, 0 );
            } else {
            	LOGI(TAG, "No json_to_pid_list() registered.");
            	Generate_TX_Message( dev, KE_NACK, 0 );
            }

            // KE_OPTION_LIST_REQUEST has been responded to
            KE_clear_flag(dev, KE_PENDING_RESPONSE);
        	break;

        case KE_PID_LIST_REQUEST:
        	Generate_TX_Message(dev, KE_PID_LIST_SEND, 0);
        	break;

        case KE_BINARY_SEND_CHUNK:
        	if (dev->init.binary_to_flash) {
        		uint32_t offset =
        		    ((uint32_t)dev->rx_buffer[KE_PCKT_DATA_START_POS] << 24) |
        		    ((uint32_t)dev->rx_buffer[KE_PCKT_DATA_START_POS + 1] << 16) |
        		    ((uint32_t)dev->rx_buffer[KE_PCKT_DATA_START_POS + 2] << 8) |
        		    ((uint32_t)dev->rx_buffer[KE_PCKT_DATA_START_POS + 3]);

                // Pointer to the start of binary data (after the offset)
                uint8_t *binary_data = (uint8_t *)&dev->rx_buffer[KE_PCKT_DATA_START_POS + sizeof(uint32_t)];

                // Calculate the size of the binary chunk
                uint32_t chunk_size = dev->rx_byte_count - KE_PCKT_DATA_START_POS - sizeof(uint32_t) - 1;

                dev->init.binary_to_flash(binary_data, chunk_size, offset);

            	Generate_TX_Message(dev, KE_ACK, 0);
        	} else {
            	LOGI(TAG, "No binary_to_flash() registered.");
            	Generate_TX_Message( dev, KE_NACK, 0 );
            }
        	break;

        case KE_ENTER_BOOTLOADER:
        	if (dev->init.enter_bootloader) {
        		dev->init.enter_bootloader();

        		// DO NOT ACT, the bootloader should send an ACK once it's ready
        	} else {
            	LOGI(TAG, "No enter_bootloader() registered.");
            	Generate_TX_Message( dev, KE_NACK, 0 );
        	}
        	break;

        default:
            LOGI(TAG, "Protocol Error on Receive");
            return KE_ERROR;
            break;
    }

    return KE_OK;
}

KE_STATUS KE_Add_UART_Byte( PKE_PACKET_MANAGER dev, uint8_t byte )
{
    // Check for timeout
    if (ke_tick - dev->rx_time > RX_TIMEOUT_MS) {
        dev->rx_byte_count = 0;
        KE_clear_flag(dev, KE_RX_IN_PROGRESS);
        KE_clear_flag(dev, KE_PCKT_CMPLT);
        flush_rx_buffer(dev);
        dev->diagnostic.rx_abort_count++;
        LOGI(TAG, "RX timeout, resetting buffer");
    }

    dev->rx_time = ke_tick;

    // Add the byte to the buffer
    if (dev->rx_byte_count < dev->rx_buffer_size) {
        dev->rx_buffer[dev->rx_byte_count++] = byte;
    } else {
        dev->diagnostic.rx_abort_count++;
        KE_clear_flag(dev, KE_RX_IN_PROGRESS);
        dev->rx_byte_count = 0;
        LOGI(TAG, "Buffer Full");
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
            if (KE_get_flag(dev, KE_RX_IN_PROGRESS) ) {
                dev->diagnostic.rx_abort_count++;
            }

            // Shift SOL to index 0
            memmove(dev->rx_buffer, &dev->rx_buffer[i], dev->rx_byte_count - i);
            dev->rx_byte_count = dev->rx_byte_count - i;

            KE_set_flag(dev, KE_RX_IN_PROGRESS);
            KE_clear_flag(dev, KE_PCKT_CMPLT);

            //LOGI(TAG, "Start of new message");
            return KE_START_OF_NEW_MSG;
        }
    }

    // If already receiving, check for message complete
    if (KE_get_flag(dev, KE_RX_IN_PROGRESS) ) {

    	// Verify the length data has been rx'd
    	if(dev->rx_byte_count <= KE_PCKT_LEN_BYTE3_POS)
    		return KE_OK;

    	uint32_t len = ((uint32_t)dev->rx_buffer[KE_PCKT_LEN_BYTE0_POS] << 24) |
    	               ((uint32_t)dev->rx_buffer[KE_PCKT_LEN_BYTE1_POS] << 16) |
    	               ((uint32_t)dev->rx_buffer[KE_PCKT_LEN_BYTE2_POS] << 8)  |
    	               ((uint32_t)dev->rx_buffer[KE_PCKT_LEN_BYTE3_POS]);

        if (dev->rx_byte_count == len)
        {
            KE_clear_flag(dev, KE_RX_IN_PROGRESS);
            dev->diagnostic.rx_count++;
            KE_set_flag(dev, KE_PCKT_CMPLT);
            //LOGI(TAG, "Packet completed");
            return KE_PACKET_COMPLETE;
        }

        else if (dev->rx_byte_count > len) {
            dev->diagnostic.rx_abort_count++;
            KE_clear_flag(dev, KE_RX_IN_PROGRESS);
            dev->rx_byte_count = 0;
            LOGI(TAG, "Out of sync");
            return KE_OUT_OF_SYNC;
        }

        return KE_OK;
    }
    
    return KE_OK;
}

void Generate_TX_Message( PKE_PACKET_MANAGER dev, KE_CP_OP_CODES cmd, void *args )
{
    /* Clear the buffer */
    //flush_tx_buffer( dev );
    dev->tx_byte_count = 0;

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
            LOGI(TAG, "ACK Sent");
            /* No additional data necessary */
            break;
        case KE_NACK:
            LOGI(TAG, "NACK Sent");
            /* No additional data necessary */
            break;
        case KE_HEARTBEAT:
            LOGI(TAG, "Heatbeat Sent");
            /* No additional data necessary */
            break;
        case KE_SYS_READY:
            LOGI(TAG, "System Ready Sent");
            /* No additional data necessary */
            break;
        case KE_PID_STREAM_NEW:
            LOGI(TAG, "New PID Stream Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_PID_STREAM_ADD:
            LOGI(TAG, "Add to PID Stream Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_PID_STREAM_REMOVE:
            LOGI(TAG, "Remove from PID Stream Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_PID_STREAM_CLEAR:
            LOGI(TAG, "Clear PID Stream Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_PID_STREAM_REPORT:
            LOGI(TAG, "PID Stream Report Sent");
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
                            dev->tx_byte_count += snprintf((char*)(&dev->tx_buffer[dev->tx_byte_count]), dev->tx_buffer_size ,
                                    "0x%02X%04X:%u:%.2f", (uint8_t)(dev->stream[i]->pid_uuid >> 16), (uint16_t)(dev->stream[i]->pid_uuid), units, value);

                        /* If not, assume it is a single byte PID */
                        else
                            dev->tx_byte_count += snprintf((char*)(&dev->tx_buffer[dev->tx_byte_count]), dev->tx_buffer_size ,
                                    "0x%02X%02X:%u:%.2f", (uint8_t)(dev->stream[i]->pid_uuid >> 16), (uint8_t)(dev->stream[i]->pid_uuid & 0xFF), units, value);

                        /* Add a semi-colon after every PID except the last */
                        if( i < remaining_delim )
                            dev->tx_buffer[dev->tx_byte_count++] = ',';
                }
            }
            // A response is needed.
            KE_set_flag(dev, KE_PENDING_RESPONSE);
            break;
        case KE_LCD_ENABLE:
            LOGI(TAG, "LCD Enable Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_LCD_DISABLE:
            LOGI(TAG, "LCD Disable Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_LCD_POWER_CYCLE:
            LOGI(TAG, "LCD Power Cylce Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_LCD_FORCE_BRIGHTNESS:
            LOGI(TAG, "LCD Force Brightness Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_LCD_AUTO_BRIGHTNESS:
            LOGI(TAG, "LCD Auto Brightness Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_USB_ENABLE:
            LOGI(TAG, "USB Enable Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_USB_DISABLE:
            LOGI(TAG, "USB Disable Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_USB_POWER_CYCLE:
            LOGI(TAG, "USB Power Cycle Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_POWER_ENABLE:
            LOGI(TAG, "Power Enable Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_POWER_DISABLE:
            LOGI(TAG, "Power Disable Sent");
            /* No additional data necessary */
            break;
        case KE_POWER_CYCLE:
            LOGI(TAG, "Power Cylce Sent");
            /* No additional data necessary */
            break;
        case KE_FIRMWARE_REQ:
            LOGI(TAG, "Firmware Request Sent");
            /*TODO: Add support to be a host */
            break;
        case KE_FIRMWARE_REPORT:
            LOGI(TAG, "Firmware Report Sent");
            dev->tx_byte_count += snprintf( (char*)(&dev->tx_buffer[dev->tx_byte_count]), dev->tx_buffer_size , "%02d.%02d.%02d",
                    dev->init.firmware_version_major ,
                    dev->init.firmware_version_minor,
                    dev->init.firmware_version_hotfix );
            break;
        case KE_BACKGROUND_SEND:
            if (dev->init.png_to_rgba) {

                // Cast and dereference
                uint8_t background_idx = *((uint8_t *)args);

                // First byte is the index
                dev->tx_buffer[dev->tx_byte_count++] = background_idx;

                // Convert the png to rgba data
                dev->tx_byte_count += dev->init.png_to_rgba((char*)&dev->tx_buffer[dev->tx_byte_count], dev->tx_buffer_size - dev->tx_byte_count - 1, background_idx);

                // A response is needed.
                KE_set_flag(dev, KE_PENDING_RESPONSE);
            } else {
            	LOGI(TAG, "No png_to_rgba() registered.");
            }
        	break;
        case KE_BACKGROUND_REQUEST:
            LOGI(TAG, "Background Image Request Sent");
            /* No additional data necessary */
        	break;
        case KE_BACKGROUND_CRC_SEND:
            if(dev->init.get_rgba_crc) {
                // Cast and dereference
                uint8_t background_idx = *((uint8_t *)args);
                uint32_t background_crc = dev->init.get_rgba_crc(background_idx, 0);

                dev->tx_buffer[dev->tx_byte_count++] = background_idx;
                dev->tx_buffer[dev->tx_byte_count++] = (uint8_t)(background_crc >> 24);
                dev->tx_buffer[dev->tx_byte_count++] = (uint8_t)(background_crc >> 16);
                dev->tx_buffer[dev->tx_byte_count++] = (uint8_t)(background_crc >> 8);
                dev->tx_buffer[dev->tx_byte_count++] = (uint8_t)(background_crc);
            } else {
            	LOGI(TAG, "No rgba_crc() registered.");
            }
            LOGI(TAG, "Background CRC Sent");
            break;
        case KE_BACKGROUND_CRC_REQUEST:
            // Cast and dereference
            uint8_t background_idx = *((uint8_t *)args);

            dev->tx_buffer[dev->tx_byte_count++] = background_idx;

            LOGI(TAG, "Background CRC Request Sent");

            // A response is needed.
            KE_set_flag(dev, KE_PENDING_RESPONSE);
            break;
        case KE_CONFIG_SEND:
            if (dev->init.config_to_json) {
                dev->tx_byte_count += dev->init.config_to_json((char*)&dev->tx_buffer[KE_PCKT_DATA_START_POS], dev->tx_buffer_size - KE_PCKT_DATA_START_POS - 1);
            } else {
            	LOGI(TAG, "No config_to_json() registered.");
            }
            LOGI(TAG, "Config Sent");
            
            // A response is needed.
            KE_set_flag(dev, KE_PENDING_RESPONSE);
        	break;
        case KE_CONFIG_REQUEST:
            LOGI(TAG, "Config Request Sent");

            // A response is needed.
            KE_set_flag(dev, KE_PENDING_RESPONSE);
            /* No additional data necessary */
        	break;
        case KE_OPTION_LIST_SEND:
            if (dev->init.options_to_json) {
                dev->tx_byte_count += dev->init.options_to_json((char*)&dev->tx_buffer[KE_PCKT_DATA_START_POS], dev->tx_buffer_size - KE_PCKT_DATA_START_POS - 1);
            } else {
            	LOGI(TAG, "No options_to_json() registered.");
            }
            LOGI(TAG, "Option List Sent");

            // A response is needed.
            KE_set_flag(dev, KE_PENDING_RESPONSE);
        	break;
        case KE_OPTION_LIST_REQUEST:
            LOGI(TAG, "Option list Request Sent");

            // A response is needed.
            KE_set_flag(dev, KE_PENDING_RESPONSE);
            /* No additional data necessary */
        	break;
        case KE_PID_LIST_SEND:
            if (dev->init.pid_list_to_json) {
                dev->tx_byte_count += dev->init.pid_list_to_json((char*)&dev->tx_buffer[KE_PCKT_DATA_START_POS], dev->tx_buffer_size - KE_PCKT_DATA_START_POS - 1);
            } else {
            	LOGI(TAG, "No pid_list_to_json() registered.");
            }
            LOGI(TAG, "PID List Sent");
        	break;
        case KE_PID_LIST_REQUEST:
        	LOGI(TAG, "PID list Request Sent");

            // A response is needed.
            KE_set_flag(dev, KE_PENDING_RESPONSE);
        	break;
        case KE_BINARY_SEND_CHUNK:
            if (dev->init.binary_get_chunk) {
                
                // Cast and dereference
                uint32_t start_byte = *((uint32_t *)args);

                // Copy the 4 bytes of the chunk into tx_buffer
                dev->tx_buffer[dev->tx_byte_count++] = (start_byte >> 24) & 0xFF;
                dev->tx_buffer[dev->tx_byte_count++] = (start_byte >> 16) & 0xFF;
                dev->tx_buffer[dev->tx_byte_count++] = (start_byte >>  8) & 0xFF;
                dev->tx_buffer[dev->tx_byte_count++] =  start_byte        & 0xFF;

                dev->tx_byte_count += dev->init.binary_get_chunk((char*)&dev->tx_buffer[dev->tx_byte_count], dev->tx_buffer_size - KE_PCKT_DATA_START_POS - 1);

                // A response is needed.
                KE_set_flag(dev, KE_PENDING_RESPONSE);
            } else {
            	LOGI(TAG, "No pid_list_to_json() registered.");
            }
            LOGI(TAG, "Firmware binary sent");
            break;
        case KE_ENTER_BOOTLOADER:
            LOGI(TAG, "Bootloader Activation Sent");
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

void KE_wait_for_response( PKE_PACKET_MANAGER dev, uint32_t timeout )
{
    uint32_t start_t = ke_tick;

    // Non-blocking when timeout is 0
    if( timeout == 0 ) {
        LOGI(TAG, "No timeout, immediately continue");
        return;
    }
    
    // Wait for a response
    while ((ke_tick - start_t) < timeout) {
        KE_Service(dev);

        // Exit once a response has been received
        if ( KE_get_flag(dev, KE_PENDING_RESPONSE) == 0 )
            return;

        #ifdef ESP_PLATFORM
        vTaskDelay(pdMS_TO_TICKS(1)); // Allow WDT refresh
        #endif
    }

    LOGI(TAG, "Timeout at %lums, no response received", (unsigned long)ke_tick);
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
    memset( dev->tx_buffer, 0, dev->tx_buffer_size );

    /* Reset the byte count */
    dev->tx_byte_count = 0;
}

static void flush_rx_buffer( PKE_PACKET_MANAGER dev )
{
    /* Clear the buffer */
    memset( dev->rx_buffer, 0, dev->rx_buffer_size );

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
