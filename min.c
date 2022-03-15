#include "min.h"
enum {
    HEADER_BYTE = 0xaaU,
    STUFF_BYTE = 0x55U,
    EOF_BYTE = 0x55U,
};

// Receiving state machine
enum {
    SEARCHING_FOR_SOF,
    RECEIVING_ID_CONTROL,
    RECEIVING_SEQ,
    RECEIVING_LENGTH,
    RECEIVING_PAYLOAD,
    RECEIVING_CHECKSUM_3,
    RECEIVING_CHECKSUM_2,
    RECEIVING_CHECKSUM_1,
    RECEIVING_CHECKSUM_0,
    RECEIVING_EOF,
};
//prototype
static void valid_frame_received(struct min_context *self);

/* set up context for usart 2*/
void min_init_context(struct min_context *self, uint8_t port)
{
    self ->rx_frame_state = SEARCHING_FOR_SOF;
    self ->rx_header_bytes_seen = 0;
    self ->rx_frame_checksum = 0;
    self ->tx_checksum.crc = (uint32_t) 0 ;
    self ->rx_checksum.crc = (uint32_t) 0;
    self ->tx_header_byte_countdown = 2;
    self ->rx_control = 0;
    self ->rx_frame_length = 0;
    self ->port = port;
}
/* checksum cho 32 bit*/
static void crc32_init_context(struct crc32_context *context)
{
    context->crc = 0xffffffffU;
}

static void crc32_step(struct crc32_context *context, uint8_t byte)
{
    uint32_t j;
    context->crc ^= byte;
    for(j = 0; j < 8; j++) {
        uint32_t mask = (uint32_t) -(context->crc & 1U);
        context->crc = (context->crc >> 1) ^ (0xedb88320U & mask);
    }
}

static uint32_t crc32_finalize(struct crc32_context *context)
{
    return ~context->crc;
}
/* send byte and stuff byte*/
static void stuffed_tx_byte(struct min_context *self, uint8_t byte, bool crc)
{
    // Transmit the byte
    min_tx_byte(self->port, byte);
    if(crc)         
    {
        crc32_step(&self->tx_checksum, byte);
    }

    // See if an additional stuff byte is needed
    if(byte == HEADER_BYTE) 
    {
        if(--self->tx_header_byte_countdown == 0) 
        {
            min_tx_byte(self->port, STUFF_BYTE);        // Stuff byte
            self->tx_header_byte_countdown = 2U;
        }
    }
    else
    {
        self->tx_header_byte_countdown = 2U;
    }
}
/********** send a whole frame ************/
void min_send_frame(struct min_context *self, uint8_t min_id, uint8_t const *payload, uint8_t payload_len)
{
    uint32_t checksum;
    self->tx_header_byte_countdown = 2U;
    crc32_init_context(&self->tx_checksum);
    min_tx_byte(0, HEADER_BYTE);
    min_tx_byte(0, HEADER_BYTE);
    min_tx_byte(0, HEADER_BYTE);
    stuffed_tx_byte(self, min_id & 0x3f, true);
    stuffed_tx_byte(self, payload_len, true);
    for (uint8_t i = 0; i < payload_len; i++)
    {
        stuffed_tx_byte(self, *(payload +i), true);
    }
//    self ->tx_checksum.crc = (uint32_t) ( min_id + lenOfFrameByte + payload_len);
    checksum = crc32_finalize(&self->tx_checksum);
    stuffed_tx_byte(self, (uint8_t)((checksum >> 24) & 0xffU), false);
    stuffed_tx_byte(self, (uint8_t)((checksum >> 16) & 0xffU), false);
    stuffed_tx_byte(self, (uint8_t)((checksum >> 8) & 0xffU), false);
    stuffed_tx_byte(self, (uint8_t)((checksum >> 0) & 0xffU), false);
    min_tx_byte(self->port, EOF_BYTE);
}


/********* recieve byte by byte ***********/
void rx_byte(struct min_context *self, uint8_t byte)
{
    // Regardless of state, three header bytes means "start of frame" and
    // should reset the frame buffer and be ready to receive frame data
    //
    // Two in a row in over the frame means to expect a stuff byte.
    uint32_t crc;

    if(self->rx_header_bytes_seen == 2) 
    {
        self->rx_header_bytes_seen = 0;
        if(byte == HEADER_BYTE) 
        {
            self->rx_frame_state = RECEIVING_ID_CONTROL;
            return;
        }
        if(byte == STUFF_BYTE) 
        {
            /* Discard this byte; carry on receiving on the next character */
            return;
        }
        else 
        {
            /* Something has gone wrong, give up on this frame and look for header again */
            self->rx_frame_state = SEARCHING_FOR_SOF;
            return;
        }
    }

    if(byte == HEADER_BYTE) 
    {
        self->rx_header_bytes_seen++;
    }
    else 
    {
        self->rx_header_bytes_seen = 0;
    }

    switch(self->rx_frame_state) {
        case SEARCHING_FOR_SOF:
            break;
       case RECEIVING_ID_CONTROL:
            self->rx_frame_id_control = byte;
            self->rx_frame_payload_bytes = 0;
            crc32_init_context(&self->rx_checksum);
            crc32_step(&self->rx_checksum, byte);
            self->rx_frame_seq = 0;
            self->rx_frame_state = RECEIVING_LENGTH;
            break;
//           if(byte & 0x80U) {
//#ifdef TRANSPORT_PROTOCOL
//                self->rx_frame_state = RECEIVING_SEQ;
//#else
//                // If there is no transport support compiled in then all transport frames are ignored
//                self->rx_frame_state = SEARCHING_FOR_SOF;
//#endif // TRANSPORT_PROTOCOL
//            }
//            else {
//                self->rx_frame_seq = 0;
//                self->rx_frame_state = RECEIVING_LENGTH;
//            }
//            break;
//        case RECEIVING_SEQ:
//            self->rx_frame_seq = byte;
//            crc32_step(&self->rx_checksum, byte);
//            self->rx_frame_state = RECEIVING_LENGTH;
//            break;
        case RECEIVING_LENGTH:
            self->rx_frame_length = byte;
            self->rx_control = byte;
            crc32_step(&self->rx_checksum, byte);
            if(self->rx_frame_length > 0) 
            {
                // Can reduce the RAM size by compiling limits to frame sizes
                if(self->rx_frame_length <= MAX_PAYLOAD) 
                {
                    self->rx_frame_state = RECEIVING_PAYLOAD;
                }
                else {
                    // Frame dropped because it's longer than any frame we can buffer
 //                   min_debug_print("Dropping frame because length %d > MAX_PAYLOAD %d", self->rx_frame_length, MAX_PAYLOAD);
                    self->rx_frame_state = SEARCHING_FOR_SOF;
                }
            }
            else {
                self->rx_frame_state = RECEIVING_CHECKSUM_3;
            }
            break;
        case RECEIVING_PAYLOAD:
            self->rx_frame_payload_buf[self->rx_frame_payload_bytes++] = byte;
            crc32_step(&self->rx_checksum, byte);
            if(--self->rx_frame_length == 0) 
            {
                self->rx_frame_state = RECEIVING_CHECKSUM_3;
            }
            break;
        case RECEIVING_CHECKSUM_3:
            self->rx_frame_checksum = ((uint32_t)byte) << 24;
            self->rx_frame_state = RECEIVING_CHECKSUM_2;
            break;
        case RECEIVING_CHECKSUM_2:
            self->rx_frame_checksum |= ((uint32_t)byte) << 16;
            self->rx_frame_state = RECEIVING_CHECKSUM_1;
            break;
        case RECEIVING_CHECKSUM_1:
            self->rx_frame_checksum |= ((uint32_t)byte) << 8;
            self->rx_frame_state = RECEIVING_CHECKSUM_0;
            break;
        case RECEIVING_CHECKSUM_0:
            self->rx_frame_checksum |= byte;
            crc = crc32_finalize(&self->rx_checksum);
            if(self->rx_frame_checksum != crc) {
 //               min_debug_print("Checksum failed, received 0x%08X, computed 0x%08X", self->rx_frame_checksum, crc);
                // Frame fails the checksum and so is dropped
                self->rx_frame_state = SEARCHING_FOR_SOF;
            }
            else {
                // Checksum passes, go on to check for the end-of-frame marker
                self->rx_frame_state = RECEIVING_EOF;
            }
            break;
        case RECEIVING_EOF:
            if(byte == 0x55u) {
                // Frame received OK, pass up data to handler
                valid_frame_received(self);
            } else {
                // else discard
//                min_debug_print("Received invalid EOF 0x%02X", byte);
            }
            // Look for next frame */
            self->rx_frame_state = SEARCHING_FOR_SOF;
            break;
        default:
            // Should never get here but in case we do then reset to a safe state
            min_debug_print("Received byte 0x%02X in invalid state %d", byte, self->rx_frame_state);
            self->rx_frame_state = SEARCHING_FOR_SOF;
            break;
    }
}
/* *********** valid frame to call handler *********** */
static void valid_frame_received(struct min_context *self)
{
    uint8_t id_control = self->rx_frame_id_control;
    uint8_t *payload = self->rx_frame_payload_buf;
    uint8_t payload_len = self->rx_control;
    min_application_handler(id_control & (uint8_t)0x3fU, payload, payload_len, self->port);
}
/************** handler (callback) handle data loaded *********/
//void min_application_handler(uint8_t min_id, uint8_t const *min_payload, uint8_t len_payload, uint8_t port)
//{
//    if (min_id ==  1)
//    {
//        HAL_UART_Transmit (&huart6, (uint8_t *) min_payload, (uint16_t) len_payload, 100);
//    }
//}
///**************** interrupt callback receive data *********/
