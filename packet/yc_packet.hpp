#pragma once
#include <functional>

using packet_id_type = int8_t;
using packet_size_type = int16_t;

constexpr const int __counter = __COUNTER__;
constexpr const int __packets_max__ = 1000;

#define PACKET(name, field) \
struct packet_##name { \
constexpr const static int __packet__id = __COUNTER__ - (__counter + 1); \
field \
static void bind(std::function<void(packet_##name, size_t)> f) { \
packet_events[packet_##name::__packet__id] = [f](void* data, packet_size_type size, size_t client_id) { \
f(*((packet_##name*)data), client_id); \
}; \
} \
};

#define PACKET_VAR(name, type, value) \
struct packet_var_##name { \
constexpr const static int __packet__id = __COUNTER__ - (__counter + 1); \
packet_size_type size;\
type value \
constexpr const static int type_size = sizeof(type); \
static void bind(std::function<void(packet_var_##name, size_t)> f) { \
packet_events[packet_var_##name::__packet__id] = [f](void* data, packet_size_type size, size_t client_id) { \
packet_var_##name data_;\
memcpy(&data_, data, size); \
f(data_, client_id); \
}; \
} \
};

std::vector<std::function<void(void*, packet_size_type, size_t)>> packet_events(__packets_max__);
auto call_packet_event(void* data, packet_id_type packet_id, packet_size_type size, size_t client_id) {
	packet_events[packet_id](data, size, client_id);
};

namespace yc_pack {
	template <typename PACKET_VAR>
	concept is_packet_var_tpye = requires { PACKET_VAR::__packet__id; PACKET_VAR::size; PACKET_VAR::type_size; };

	template <typename PACKET>
	concept is_packet_tpye = requires { PACKET::__packet__id; } && !is_packet_var_tpye<PACKET>;

	struct raw_packet {
		packet_size_type size;
		packet_id_type id;
		char* body;
	};

	constexpr const int HEADER_SIZE = sizeof(packet_size_type) + sizeof(packet_id_type);

	template <is_packet_tpye T>
	static raw_packet pack(T& packet_data) {
		return raw_packet{
			.size = sizeof(T) + HEADER_SIZE,
			.id = (packet_id_type)T::__packet__id,
			.body = ((char*)&packet_data)
		};
	}

	template <is_packet_var_tpye T>
	static raw_packet pack(T& packet_data) {
		return raw_packet{
			.size = packet_size_type((packet_data.size * packet_data.type_size) + HEADER_SIZE + sizeof(packet_size_type)), //PACKET_VAR::size = value(packet_size_type) size 
			.id = (packet_id_type)T::__packet__id,
			.body = ((char*)&packet_data)
		};
	}

	static raw_packet unpack(char* byte_code) {
		return raw_packet{
			.size = *((packet_size_type*)byte_code),
			.id = *((packet_id_type*)(byte_code + sizeof(packet_size_type))),
			.body = byte_code + HEADER_SIZE
		};
	}
}