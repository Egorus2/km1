#ifndef PARSER_H
#define PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*parser_callback_t)(uint8_t msg_type, uint8_t *payload, uint8_t len);

void parser_set_callback(parser_callback_t cb);
void send_uart_packet (uint8_t msg_type, uint8_t * data, uint8_t len, UART_HandleTypeDef *huart);
void parser_process_byte(uint8_t b);


#ifdef __cplusplus
}
#endif

#endif
