#pragma once
#include "yc_packet.hpp"

#pragma pack(push, 1)
enum class player_input_data_type : int8_t {
	SPACE,
	CTRL,
	ESC,
	MOUSE_LEFT,
	MOUSE_RIGHT
};

struct vector2_t {
	float x, y;
};
struct vector3_t {
	float x, y, z;
};

struct player_input_t {
	player_input_data_type type;
};

struct player_movement_start_t {
	vector2_t velocity;
	vector2_t location;
	uint64_t timestamp;
};

struct player_movement_t {
	size_t player_id;
	vector2_t velocity;
	vector2_t location;
	uint64_t timestamp;
};

struct player_spawn_t {
	size_t player_id;
	wchar_t name[20];
	vector3_t location;
	vector2_t velocity;
	uint64_t timestamp;
};

PACKET(player_movement_start, player_movement_start_t move_data;)
PACKET_VAR(players_location, player_movement_t, player_movements[50];);
PACKET_VAR(players_spawn, player_spawn_t, spawn_data[10];);

#pragma pack(pop)