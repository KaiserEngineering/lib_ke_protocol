/* File auto-generated on 7/14/2019 at 01:46 */

#ifndef KE_COMMUNICATON_PROTOCOL_H_
#define KE_COMMUNICATON_PROTOCOL_H_

typedef enum KE_CP_OP_CODES {
    KE_RESERVED                = (uint8_t)0x00,    /* Reserved                                     */
    KE_ACK                     = (uint8_t)0x01,    /* Positive acknowledgment                      */
    KE_NACK                    = (uint8_t)0x02,    /* Negative acknowledgment                      */
    KE_HEARTBEAT               = (uint8_t)0x03,    /* Heartbeat                                    */
    KE_SYS_READY               = (uint8_t)0x04,    /* System ready (GUI)                           */
    KE_PID_STREAM_NEW          = (uint8_t)0x05,    /* Clear current PID request and add new PID(s) */
    KE_PID_STREAM_ADD          = (uint8_t)0x06,    /* Add PID request to current stream            */
    KE_PID_STREAM_REMOVE       = (uint8_t)0x07,    /* Remove PID request from current stream       */
    KE_PID_STREAM_CLEAR        = (uint8_t)0x08,    /* Clear all PID request from current stream    */
    KE_PID_STREAM_REPORT       = (uint8_t)0x09,    /* Report PID Data                              */
    KE_LCD_ENABLE              = (uint8_t)0x0A,    /* Enable the LCD display                       */
    KE_LCD_DISABLE             = (uint8_t)0x0B,    /* Disable the LCD display                      */
    KE_LCD_POWER_CYCLE         = (uint8_t)0x0C,    /* Power cycle the LCD display                  */
    KE_LCD_FORCE_BRIGHTNESS    = (uint8_t)0x0D,    /* Force an LCD brightness (volatile)           */
    KE_LCD_AUTO_BRIGHTNESS     = (uint8_t)0x0E,    /* Re-enable standard LCD brightness control    */
    KE_USB_ENABLE              = (uint8_t)0x0F,    /* Enable the USB power                         */
    KE_USB_DISABLE             = (uint8_t)0x10,    /* Disable the USB power                        */
    KE_USB_POWER_CYCLE         = (uint8_t)0x11,    /* Power cycle the USB power                    */
    KE_POWER_ENABLE            = (uint8_t)0x12,    /* Enable Power                                 */
    KE_POWER_DISABLE           = (uint8_t)0x13,    /* Disable Power                                */
    KE_POWER_CYCLE             = (uint8_t)0x14,    /* Power cycle                                  */
    KE_FIRMWARE_REQ            = (uint8_t)0x15,    /* Request firmware version                     */
    KE_FIRMWARE_REPORT         = (uint8_t)0x16,    /* Report firmware version                      */
    KE_FIRMWARE_UPDATE         = (uint8_t)0x17     /* Place device in firmware update mode         */
} KE_CP_OP_CODES;

#endif /* KE_COMMUNICATON_PROTOCOL_H_ */
