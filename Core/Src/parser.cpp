extern "C" {
#include "stm32g4xx_hal.h"
}

#include <vector>
#include "stdint.h"

class packet_t
{
public:
	uint16_t preambule = 0xFFFA;
	uint8_t msg_type;
	uint8_t len_packet;
	uint8_t len_payload;
	uint8_t *data;
	uint16_t crc;
	uint8_t end = 0x0A;
private:
	uint16_t crc16_ccitt(uint8_t *data, uint32_t len)
	{
	    uint16_t crc = 0xFFFF;
	    uint16_t i;

	    while (len--) {
	        crc ^= (uint16_t)(*data++) << 8;
	        for (i = 0; i < 8; i++) {
	            if (crc & 0x8000)
	                crc = (crc << 1) ^ 0x1021;
	            else
	                crc <<= 1;
	        }
	    }

	    return crc;
	}
public:
	packet_t(uint8_t *d, uint8_t l, uint8_t msg_t) : data(d), len_payload(l), len_packet(l + 6), msg_type(msg_t)
	{
		crc = crc16_ccitt(d, l);
	}
	~packet_t()
	{

	}

	packet_t(const packet_t &packet_t)
	{

	}

	std::vector <uint8_t> make_buff()
	{
		std::vector <uint8_t> buff;
		buff.reserve(25);
		buff.push_back(static_cast <uint8_t> (0xFF));
		buff.push_back(static_cast <uint8_t> (0xFA));
		buff.push_back(static_cast <uint8_t> (msg_type));
		buff.push_back(static_cast <uint8_t> (len_packet));
		buff.insert(buff.end(), data, data + len_payload);
		buff.push_back(crc & 0xFF);
		buff.push_back((crc >> 8) & 0xFF);
		buff.push_back(static_cast <uint8_t> (end));
		return buff;
	}

};

extern "C" void send_uart_packet(uint8_t msg_type, uint8_t * data, uint8_t len, UART_HandleTypeDef *huart)
{
	packet_t packet(data, len, msg_type);
	std::vector<uint8_t> buff = packet.make_buff();
	HAL_UART_Transmit(huart, buff.data(), buff.size(), 100);
}
