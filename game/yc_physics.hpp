#pragma once
#include <unordered_map>
#include <vector>
#include <ranges>
#include <format>
#include <functional>
#include <unordered_set>
#include <variant>

#include "yc_math.hpp"

namespace yc_physics
{
	using namespace yc_math;

	template <typename Vector> concept upper_case_vec = requires { Vector::X; Vector::Y; Vector::Z; };
	template <typename Vector> concept lower_case_vec = requires { Vector::x; Vector::y; Vector::z; };
	template <typename Quaternion> concept upper_case_quat = requires { Quaternion::X; Quaternion::Y; Quaternion::Z; Quaternion::W; };
	template <typename Quaternion> concept lower_case_quat = requires { Quaternion::x; Quaternion::y; Quaternion::z; Quaternion::w; };
	
	template<class... Ts> struct loaded : Ts... { using Ts::operator()...; };
	template<class... Ts> loaded(Ts...)->loaded<Ts...>;

	struct capsule_t {
		double half_height;
		double radius;
		capsule_t(const double half_height, const double radius) : half_height(half_height), radius(radius) { }
	};

	using shape_t = std::variant<capsule_t>;
	inline shape_t make_capsule(const double half_height, const double radius) { return shape_t(capsule_t(half_height, radius)); }
	
	struct terrain_t {
		struct cell_t {
			vec3_t root_pos;
			vec3_t vertex[4];
			int height[4];
			int all_height = -1;
			std::vector<triangle_t*> triangles;
			bool is_print = false;
		};

		std::vector<cell_t> cells;
		std::vector<vec3_t> vertex;
		std::vector<size_t> index_buffs;
		std::vector<triangle_t> triangles;
		std::vector<triangle_t> interpolated_triangles;

		int width, height;
		double scale;

		vec3_t root_pos;

		[[nodiscard]] bool is_cell_all_height_same(const size_t i) const {
			const int h = cells[i].height[0];
			return (h == cells[i].height[1]) &&
				(h == cells[i].height[2]) &&
				(h == cells[i].height[3]);
		}

		bool build(const uint8_t* heightmap, const int in_width, const int in_height, const double in_scale) {
			width = in_width;
			height = in_height;
			scale = in_scale;

			vertex.clear();
			vertex.reserve(width * height);
			for (int i = 0; i < width * height; i++) {
				auto h = heightmap[i] - 128;
				int x = i % width;
				int y = i / width;
				vertex.emplace_back(root_pos + vec3_t{x * scale, y * scale, h * scale});
			}
			//make triangles form vertex index
			index_buffs.clear();
			index_buffs.reserve(width * height * 6);
			for (int x = 0; x < width - 1; x++) {
				for (int y = 0; y < height - 1; y++) {
					size_t i = y * width + x;
					index_buffs.emplace_back(i);
					index_buffs.emplace_back(i + 1);
					index_buffs.emplace_back(i + width);
					index_buffs.emplace_back(i + 1);
					index_buffs.emplace_back(i + 1 + width);
					index_buffs.emplace_back(i + width);
				}
			}
			// make triangles
			triangles.clear();
			triangles.reserve(index_buffs.size() / 3);
			for (size_t i = 0; i < index_buffs.size() / 3; i++) {
				triangle_t t{};
				t.a = vertex[index_buffs[i * 3]];
				t.b = vertex[index_buffs[i * 3 + 1]];
				t.c = vertex[index_buffs[i * 3 + 2]];
				triangles.emplace_back(t);
			}

			auto get_cell = [&](const int x, const int y) -> cell_t& { return cells[y * (width - 1) + x]; };
			auto get_cell_index = [&](const int x, const int y) -> size_t { return y * (width - 1) + x; };
			auto get_vertex = [&](const int x, const int y) -> vec3_t& { return vertex[y * width + x]; };
			auto get_vertex_index = [&](const int x, const int y) -> size_t { return y * width + x; };
			auto get_height = [&](const int x, const int y) -> int { return heightmap[x + y * width]; };

			cells.clear();
			cells.reserve((width - 1) * (height - 1));
			for (int i = 0; i < (width - 1) * (height - 1); i++) {
				int x = i % (width - 1);
				int y = i / (width - 1);
				cells.emplace_back(cell_t{
					get_vertex(x, y) + vec3_t{0.5, 0.5, 0},
					{
						get_vertex(x, y),
						get_vertex(x + 1, y),
						get_vertex(x, y + 1),
						get_vertex(x + 1, y + 1)
					},
					{
						get_height(x, y),
						get_height(x + 1, y),
						get_height(x, y + 1),
						get_height(x + 1, y + 1)
					},
					-1,
					{}
				});

				cells[i].all_height = is_cell_all_height_same(i) ? cells[i].height[0] : -1;
				if (cells[i].all_height != -1) { cells[i].is_print = true; }
			}
			std::vector mark(cells.size(), false);
			auto check_box = [&](int x, int y, int w, int h) {
				if (x < 0 || y < 0 || w > width - 1 || h > height - 1) return false;
				for (int i = x; i < w + 1; i++) {
					for (int j = y; j < h + 1; j++) {
						if (mark[get_cell_index(i, j)]) return false;
						if (get_cell(i, j).all_height == -1) return false;
						if (get_cell(i, j).all_height != get_cell(x, y).all_height) return false;
					}
				}
				return true;
			};

			interpolated_triangles.reserve(cells.size() * 2);
			while (std::ranges::count(mark, false)) {
				for (size_t i = 0; i < cells.size(); i++) {
					if (!mark[i]) {
						int x = i % (width - 1);  // NOLINT(bugprone-narrowing-conversions, cppcoreguidelines-narrowing-conversions)
						int y = i / (width - 1);  // NOLINT(bugprone-narrowing-conversions, cppcoreguidelines-narrowing-conversions, clang-diagnostic-shorten-64-to-32)
						int w = x; 
						int h = y;

						bool f = true;
						bool s = true;
						bool t = true;
						while (f || s || t) {
							if (t && check_box(x, y, w + 1, h + 1)) {
								w++;
								h++;
							}
							else {
								t = false;
								f = check_box(x, y, w + 1, h);
								s = check_box(x, y, w, h + 1);
								if (f) w++;
								if (s) h++;
							}
						}
						if (w == x && h == y) {
							mark[get_cell_index(x, y)] = true;
							auto& cell = get_cell(x, y);
							triangle_t t1{};
							t1.a = cell.vertex[0];
							t1.b = cell.vertex[1];
							t1.c = cell.vertex[2];
							triangle_t t2{};
							t2.a = cell.vertex[1];
							t2.b = cell.vertex[2];
							t2.c = cell.vertex[3];

							interpolated_triangles.push_back(t1);
							interpolated_triangles.push_back(t2);
							auto size = interpolated_triangles.size();
							cell.triangles.push_back(&interpolated_triangles[size - 2]);
							cell.triangles.push_back(&interpolated_triangles[size - 1]);
						}
						else {
							cell_t* cell[4] = {
								&get_cell(x, y),
								&get_cell(w, y),
								&get_cell(x, h),
								&get_cell(w, h)
							};
							vec3_t vr[] = {
								{cell[0]->vertex[0]},
								{cell[1]->vertex[1]},
								{cell[2]->vertex[2]},
								{cell[3]->vertex[3]}
							};

							triangle_t t1{};
							t1.a = vr[0];
							t1.b = vr[1];
							t1.c = vr[2];
							triangle_t t2{};
							t2.a = vr[1];
							t2.b = vr[2];
							t2.c = vr[3];

							interpolated_triangles.push_back(t1);
							interpolated_triangles.push_back(t2);
							for (int k = x; k < w + 1; k++) {
								for (int l = y; l < h + 1; l++) {
									mark[get_cell_index(k, l)] = true;
									auto size = interpolated_triangles.size();
									get_cell(k, l).triangles.push_back(&interpolated_triangles[size - 2]);
									get_cell(k, l).triangles.push_back(&interpolated_triangles[size - 1]);
								}
							}
						}
						break;
					}
				}
			}

			return true;
		}

		//쌍선형 보간.
		static double get_interpolated_height(double h_left_up, double h_right_up, double h_left_down, double h_right_down,
		                                      double u, double v) {
			double h_up = h_left_up + (h_right_up - h_left_up) * u;
			double h_down = h_left_down + (h_right_down - h_left_down) * u;
			return h_down + (h_up - h_down) * v;
		}

		[[nodiscard]] double get_height_interpolation(const double in_x, const double in_y) const {
			const double real_x = in_x / scale;
			const double real_y = in_y / scale;
			int grid_x = static_cast<int>(floor(in_x / scale));
			int grid_y = static_cast<int>(floor(in_y / scale));
			grid_x = std::max(0, std::min(width - 1, grid_x));
			grid_y = std::max(0, std::min(height - 1, grid_y));
			//Interpolated height data is obtained by analyzing the surrounding nodes.

			//그리드의 중점을 기점으로 4개의 방향중 어느 방향에 있는지 검사한다.
			const double grid_center_x = (grid_x + 0.5) * scale;
			const double grid_center_y = (grid_y + 0.5) * scale;

			//left down
			if (real_x < grid_center_x && real_y < grid_center_y) { }
			//left up
			if (real_x < grid_center_x && real_y > grid_center_y) { }
			//right down
			if (real_x > grid_center_x && real_y < grid_center_y) { }
			//right up
			if (real_x > grid_center_x && real_y > grid_center_y) { }

			return 0;
		}
	};

	struct rigid_body_t {
		shape_t target;
		vec3_t pos {};
		qut_t rot {};
		vec3_t vel {};
		bool use_physic_simulate = false;
	};
	
	inline bool col(const capsule_t& a, const capsule_t& b) {
		/*
		const vec3_t a_pos = a.get_pos();
		const qut_t a_rot = a.get_rot();
		const vec3_t a_scale = a.get_scale();
		const double a_half_height = a.half_height * a_scale.z;
		const double a_radius = a.radius * a_scale.x;
		const vec3_t b_pos = b.get_pos();
		const qut_t b_rot = b.get_rot();
		const vec3_t b_scale = b.get_scale();
		const double b_half_height = b.half_height * b_scale.z;
		const double b_radius = b.radius * b_scale.x;

		const auto a_half_len = a_rot * vec3_t(0, 0, a_half_height - a_radius);
		const auto b_half_len = b_rot * vec3_t(0, 0, b_half_height - b_radius);
		const line_segment_t a_segment = {a_pos + a_half_len, a_pos - a_half_len};
		const line_segment_t b_segment = {b_pos + b_half_len, b_pos - b_half_len};

		if (yc_math::col(a_segment, b_segment, a_radius + b_radius)) { return true; }
		if (distance(a_pos, b_pos) < a_radius + b_radius) { return true; }
		*/
		return false;
	}

	inline std::vector<vec3_t> get_capsule_cells(
		const capsule_t& capsule,
		const vec3_t& in_pos,
		const qut_t& in_rot,
		const terrain_t& terrain)
	{
		std::vector<vec3_t> cells;
		const vec3_t pos = in_pos - terrain.root_pos;
		if (pos.x < 0 ||
			pos.y < 0 ||
			pos.x >= (terrain.width - 1) * terrain.scale ||
			pos.y >= (terrain.height - 1) * terrain.scale)
			return {};
		const auto s = pos + in_rot * vec3_t(0, 0, capsule.half_height);
		const auto e = pos - in_rot * vec3_t(0, 0, capsule.half_height);
		const auto r = capsule.radius;
		const auto x_min = std::min(s.x, e.x) - r;
		const auto x_max = std::max(s.x, e.x) + r;
		const auto y_min = std::min(s.y, e.y) - r;
		const auto y_max = std::max(s.y, e.y) + r;
		const int start_x = static_cast<int>(floor(x_min / terrain.scale));
		const int end_x = static_cast<int>(floor(x_max / terrain.scale));
		const int start_y = static_cast<int>(floor(y_min / terrain.scale));
		const int end_y = static_cast<int>(floor(y_max / terrain.scale));
		for (int x = start_x; x <= end_x; ++x) {
			for (int y = start_y; y <= end_y; ++y) { cells.push_back(vec3_t(x, y)); }
		}
		return cells;
	}

	inline line_segment_t closest_line_segment(const line_segment_t& line, const triangle_t& triangle) {
		const vec3_t norm = normalize(cross(triangle.b - triangle.a, triangle.c - triangle.a));
		const vec3_t midpoint = line.start + 0.5 * (line.end - line.start);
		const double d = dot(norm, triangle.a - midpoint);
		vec3_t closest_point = midpoint + norm * d;
		if (closest_point.x < min(triangle.a.x, min(triangle.b.x, triangle.c.x)) ||
			closest_point.x > max(triangle.a.x, max(triangle.b.x, triangle.c.x)) ||
			closest_point.y < min(triangle.a.y, min(triangle.b.y, triangle.c.y)) ||
			closest_point.y > max(triangle.a.y, max(triangle.b.y, triangle.c.y)) ||
			closest_point.z < min(triangle.a.z, min(triangle.b.z, triangle.c.z)) ||
			closest_point.z > max(triangle.a.z, max(triangle.b.z, triangle.c.z))) {
				const double da = distance(midpoint, triangle.a);
				const double db = distance(midpoint, triangle.b);
				const double dc = distance(midpoint, triangle.c);
				if (da <= db && da <= dc) closest_point = triangle.a;
				else if (db <= da && db <= dc) closest_point = triangle.b;
				else closest_point = triangle.c;
			}
		vec3_t cp_line = line.start + dot(closest_point - line.start, line.end - line.start) / dot(
			line.end - line.start, line.end - line.start) * (line.end - line.start);
		if (dot(cp_line - line.start, line.end - line.start) < 0) cp_line = line.start;
		if (dot(cp_line - line.end, line.start - line.end) < 0) cp_line = line.end;

		if (dot(cross(cp_line - triangle.a, norm), norm) > 0 &&
			dot(cross(cp_line - triangle.b, norm), norm) > 0 &&
			dot(cross(cp_line - triangle.c, norm), norm) > 0) {
			
			return {closest_point, cp_line};
		} 
		return {closest_point, cp_line};
	}

	inline std::vector<vec3_t> col(
		const rigid_body_t& rb,
		const terrain_t& terrain)
	{
		static auto get_cells_fm_terrain_to = [](const terrain_t& in_terrain, const vec3_t& xy) { return &in_terrain.cells[xy.x + xy.y * (in_terrain.width - 1)]; };
		auto get_cells_to = std::bind_front(get_cells_fm_terrain_to, terrain);
		
		std::vector<vec3_t> r;
		
		std::visit(loaded {
			[&](const capsule_t& capsule) {
				for (const auto cell : get_capsule_cells(capsule, rb.pos, rb.rot, terrain) | std::views::transform(get_cells_to)) {
					for (auto& tri : cell->triangles) {
						auto line = closest_line_segment({ rb.pos + rb.rot * vec3_t(0, 0, capsule.half_height + capsule.radius),
														   rb.pos + rb.rot * vec3_t(0, 0, -capsule.half_height + capsule.radius) }, *tri);
						if (distance(line.start, line.end) < capsule.radius) {
							r.push_back(line.start);
							r.push_back(line.end);
						}
					}
				}
			},
			[](auto&&) {}
		}, rb.target);
		
		return r;
	}

	template<double Dsec>
	class physics_world {
		std::vector<rigid_body_t*> rigid_bodies;
		std::vector<terrain_t*> terrains;
		size_t start_timestamp;
		size_t tick_count;
		std::chrono::time_point<std::chrono::steady_clock> last_tick_time;
		void simulate() {
			for(const auto& rb : rigid_bodies) {
				//update rigid body
				rb->pos = rb->vel * Dsec;
				for(const auto& terrain : terrains) {
					std::visit(loaded {
						[&](const capsule_t& capsule) {
							const auto hit_result = col(*rb, *terrain);
							if(hit_result.size()) {
								const auto hit_point = hit_result[0];
								const auto on_line_point = hit_result[1];
								const auto depth = capsule.radius - distance(hit_point, on_line_point);
								const auto direction = normalize(on_line_point - hit_point);
								rb->pos = rb->pos + direction * depth;
							}
						},
						[](auto&&) {}
						}, rb->target);
				}
			}
		}
	public:
		physics_world(): tick_count(0) {
			start_timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
		}

		void add_rigid_body(rigid_body_t* rigid) {
			rigid_bodies.push_back(rigid);
		}
		void add_terrain(terrain_t* terrain) {
			terrains.push_back(terrain);
		}
		
		void simulate_physics() {
			const auto start_time = std::chrono::high_resolution_clock::now();
			const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(start_time - last_tick_time);
			double sec = std::chrono::duration_cast<std::chrono::seconds>(ms).count();
			while(sec >= Dsec) {
				simulate();
				sec -= Dsec;
				tick_count++;
				last_tick_time = start_time;
			}
		}
	};
}
