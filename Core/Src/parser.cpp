extern "C" {
#include "stm32g4xx_hal.h"
}

#include <vector>
#include "stdint.h"
#include "parser.h"

static parser_callback_t g_callback = nullptr;

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
	packet_t(uint8_t *d, uint8_t l, uint8_t msg_t)
	    : preambule(0xFFFA), msg_type(msg_t), len_packet(l+6), len_payload(l), data(d), crc(crc16_ccitt(d, l)), end(0x0A) {}

	packet_t(uint8_t *d, uint8_t l):msg_type(0), len_packet(l+6), len_payload(l), data(d)
	{
		crc = crc16_ccitt(d, l);
	}

	packet_t():preambule(0xFFFA), msg_type(0), len_packet(0), len_payload(0), data(nullptr), crc(0), end(0x0A)
	{

	}

	~packet_t()
	{

	}

	packet_t(const packet_t& other)
	{
	    preambule   = other.preambule;
	    msg_type    = other.msg_type;
	    len_packet  = other.len_packet;
	    len_payload = other.len_payload;
	    data        = other.data;
	    crc         = other.crc;
	    end         = other.end;
	}

	packet_t& operator=(const packet_t& other)
	{
	    if (this != &other)
	    {
	        preambule   = other.preambule;
	        msg_type    = other.msg_type;
	        len_packet  = other.len_packet;
	        len_payload = other.len_payload;
	        data        = other.data;
	        crc         = other.crc;
	        end         = other.end;
	    }
	    return *this;
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

class parser_t
{
private:
	packet_t* packet;
	uint8_t buff [128];
	uint8_t payload_index;
	uint8_t crc_low = 0;
	enum class state
	{
		PREAMBULE_1 = 0,
		PREAMBULE_2,
		MSG_TYPE,
		LEN,
		DATA,
		CRC_LOW,
		CRC_HIGH,
		END
	};
	state index;

public:
	parser_t()
	{
		packet = new packet_t();
		payload_index = 0;
		index = state::PREAMBULE_1;
	}
	~parser_t()
	{
		delete(packet);
	}

	void process_byte(uint8_t b)
	{
	    switch(index)
	    {
	        case state::PREAMBULE_1:
	        	if (b == 0xFF)
	        	{
	        		index = state::PREAMBULE_2;
	        	}
	            break;

	        case state::PREAMBULE_2:
	        	if (b == 0xFA)
	        	{
	        		index = state::MSG_TYPE;
	        	}
	        	else
	        	{
	        		index = state::PREAMBULE_1;
	        	}
	        	break;

	        case state::MSG_TYPE:
	        	packet->msg_type = b;
	        	index = state::LEN;
	            break;

	        case state::LEN:
	        	packet->len_payload = b;
	        	packet->len_packet = b + 6;
	        	payload_index = 0;
	        	index = state::DATA;
	            break;

	        case state::DATA:
	            buff[payload_index++] = b;

	            if (payload_index == packet->len_payload)
	            {
	                packet->data = buff;
	                index = state::CRC_LOW;
	            }
	            break;

	        case state::CRC_LOW:
	        {
	        	crc_low = b ;
	        	index = state::CRC_HIGH;
	            break;
	        }

	        case state::CRC_HIGH:
	        {
	        	packet->crc = crc_low | (uint16_t(b)<<8);
	        	index = state::END;
	            break;
	        }

	        case state::END:
	        {
	        	if (b == 0x0A)
	        	    {
	        		payload_index = 0;
	        		crc_low = 0;
	                if (g_callback)
	                    g_callback(packet->msg_type, packet->data, packet->len_payload);
	        	    }

	        	index = state::PREAMBULE_1;
	        	break;
	        }
	        default: index = state::PREAMBULE_1;
	    }
	}

	parser_t(const parser_t&) = delete;
	parser_t& operator=(const parser_t&) = delete;
};

static parser_t parser;

extern "C" void parser_set_callback(parser_callback_t cb)
{
    g_callback = cb;
}

extern "C" void parser_process_byte(uint8_t b)
{
    parser.process_byte(b);
}

extern "C" void send_uart_packet(uint8_t msg_type, uint8_t * data, uint8_t len, UART_HandleTypeDef *huart)
{
	packet_t packet(data, len, msg_type);
	std::vector<uint8_t> buff = packet.make_buff();
	HAL_UART_Transmit(huart, buff.data(), buff.size(), 100);
}
