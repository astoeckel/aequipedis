/*
 *  AEQVIPEDIS -- SVG Placeholder Generator
 *  Copyright (C) 2017, 2018  Andreas Stöckel
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "aequipedis.hpp"

// Type aliases
using Point = aequipedis::Triangulation::Point;
using Rect = aequipedis::Triangulation::Rect;
using Edge = aequipedis::Triangulation::Edge;
using Triangle = aequipedis::Triangulation::Triangle;
using Color = aequipedis::Triangulation::Color;

namespace aequipedis {

/******************************************************************************
 * Struct Triangulation::Edge                                                 *
 ******************************************************************************/

/**
 * The Edge class is used in the implementation of the Bowyer-Watson algorithm
 * to uniquely identify edges in the triangulation.
 */
struct Triangulation::Edge {
	/**
	 * Edge indices corresponding ot the points in the point list.
	 */
	uint16_t i0, i1;

	/**
	 * Returns the same edge but ensures that i0 <= i1.
	 */
	Edge sort() {
		if (i0 <= i1) {
			return Edge{i0, i1};
		} else {
			return Edge{i1, i0};
		}
	}

	/**
	 * Returns true if two values correspond to the same edge. Takes the order
	 * of i0 and i1 into account.
	 */
	friend bool operator==(const Edge &e1, const Edge &e2) {
		return (e1.i0 == e2.i0) && (e1.i1 == e2.i1);
	}
};
}

namespace std {
/**
 * Implementation of std::hash for Edge
 */
template <>
struct hash<Edge> {
	size_t operator()(const Edge &e) const {
		// See https://stackoverflow.com/q/4948780/2188211
		return e.i0 ^ (e.i1 + 0x9e3779b9 + (e.i0 << 6) + (e.i0 << 2));
	}
};
}

namespace aequipedis {

/******************************************************************************
 * Helper functions                                                           *
 ******************************************************************************/

/**
 * Calculates a the integer square root of the given 16 bit number. See
 *
 * http://ww1.microchip.com/downloads/en/AppNotes/91040a.pdf
 */
static uint8_t sqrti(uint16_t x) {
	uint8_t res = 0;
	uint8_t add = 0x80;
	for (uint8_t i = 0; i < 8; i++) {
		uint8_t tmp = res | add;
		uint16_t g2 = tmp * tmp;
		if (x >= g2) {
			res = tmp;
		}
		add >>= 1;
	}
	return res;
}

/**
 * Evaluates a polynomial with weights ws at point x. See numpy polyval
 * https://docs.scipy.org/doc/numpy/reference/generated/numpy.polyval.html
 */
template <typename W>
static float polyval(W ws, float x) {
	float res = 0.0;
	for (float w : ws) {
		res = res * x + w;
	}
	return res;
}

/**
 * Applies gamma correction to convert a sRGB to a linear RGB value.
 * See https://en.wikipedia.org/wiki/SRGB
 *
 * @param v the sRGB pixel brightness normalised to a value between 0.0 and 1.0.
 */
static float srgb_to_linear(float v) {
	if (v <= 0.04045f) {
		return v / 12.92f;
	}

	// 3rd order polynomial fit of
	// return std::pow((v + 0.055f) / 1.055f, 2.4f);

	static constexpr std::array<float, 4> ws{0.28672069f, 0.72119587f,
	                                         -0.01065344f, 0.00360641f};
	return polyval(ws, v);
}

/**
 * Applies gamma correction to convert a linear RGB value to sRGB.
 *
 * See https://en.wikipedia.org/wiki/SRGB
 *
 * @param v the linear pixel brightness normalised to [0, 1]
 */
static float linear_to_srgb(float v) {
	if (v <= 0.0031308) {
		return v * 12.92;
	}

	// 6th order polynomial fit of
	// return 1.055f * std::pow(v, 1.0f / 2.4f) - 0.055f;

	static constexpr std::array<float, 7> ws{
	    -13.2979683f,  45.1882718f, -60.8107299f, 41.49907764f,
	    -15.60044293f, 3.94046415f, 0.07552462f};
	return polyval(ws, v);
}

/**
 * Computes the luminance of a pixel. First converts the given color to linear
 * color space, then weights the color components accordingly.
 *
 * See: https://en.wikipedia.org/wiki/Relative_luminance
 */
static float luminance(const Color &c) {
	return 0.2126f * srgb_to_linear(c.r / 255.0f) +
	       0.7152f * srgb_to_linear(c.g / 255.0f) +
	       0.0722f * srgb_to_linear(c.b / 255.0f);
}

static Rect bounds(const std::vector<Point> &pnts) {
	// Find the bounding rectangle
	Rect r{pnts[0], pnts[0]};
	for (const Point &p : pnts) {
		r.p0.x = std::min(r.p0.x, p.x), r.p0.y = std::min(r.p0.y, p.y);
		r.p1.x = std::max(r.p1.x, p.x), r.p1.y = std::max(r.p1.y, p.y);
	}
	return r;
}

/**
 * Helper function which constructs a triangle encompassing all points in the
 * given point list and adds it to the triangle list. Returns the index of the
 * first added point.
 *
 * @param pnts is the list of points for which the super-triangle should be
 * constructed. Adds the three triangle vertices too the list.
 * @param triangles is a list of triangles. The super-triangle is appended to
 * this list.
 * @return the index of the first new point in the point list.
 */
static size_t bowyer_watson_build_super_triangle(
    std::vector<Point> &pnts, std::vector<Triangle> &triangles) {
	// Extend the bounding rectangle a little (avoid potential for trouble with
	// points lying directly on a super-triangle edge)
	const Rect r = bounds(pnts);
	const int16_t w = (r.w() + 1) * 2, h = (r.h() + 1) * 2;
	const int16_t x0 = r.p0.x - w / 4, y0 = r.p0.y - h / 4;
	const int16_t x1 = r.p1.x + w / 4, y1 = r.p1.y + h / 4;

	// Add the points, remember the index of the first super-triangle point
	const uint16_t idx = uint16_t(pnts.size());
	pnts.emplace_back(Point{int16_t(x0 + w / 2), int16_t(y1 + h / 2)});
	pnts.emplace_back(Point{int16_t(x0 - w / 2), int16_t(y0)});
	pnts.emplace_back(Point{int16_t(x1 + w / 2), int16_t(y0)});

	// Add the triangle
	triangles.emplace_back(
	    Triangle{uint16_t(idx), uint16_t(idx + 1U), uint16_t(idx + 2U)});

	return idx;
}

/**
 * Removes the super-triangle from the triangle and point list. I.e. removes
 * all triangles sharing vertices with the super triangle from the triangle
 * list.
 *
 * @param super_triangle is the index of the first super-triangle point as
 * returned by bowyer_watson_build_super_triangle()
 * @param pnts is the list of points from which the super-triangle should
 * be removed.
 * @param triangles is the list of triangles from which the super triangle
 * should be removed. Removes all triangles sharing vertices with the super
 * triangle.
 */
static void bowyer_watson_remove_super_triangle(
    size_t super_triangle, std::vector<Point> &pnts,
    std::vector<Triangle> &triangles) {
	// Remove all triangles sharing edges with a super triangle point
	triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
	                               [super_triangle](const Triangle &t) {
		                               return (t.i0 >= super_triangle) ||
		                                      (t.i1 >= super_triangle) ||
		                                      (t.i2 >= super_triangle);
		                           }),
	                triangles.end());

	// Remove the points corresponding to the super-triangle
	pnts.resize(super_triangle);
}

/**
 * Collects all unique edges in the triangles indexed by the "bad_triangles"
 * list and removes the corresponding triangles from the triangle list.
 */
static void bowyer_watson_punch_hole(std::vector<uint32_t> &bad_triangles,
                                     std::vector<Triangle> &triangles,
                                     std::unordered_set<Edge> &edges) {
	// Collect all edges that are not shared by any other triangle
	for (size_t j = 0; j < bad_triangles.size(); j++) {
		// Function which collects unique edges, i.e. edges that only appear
		// in exactly one triangle
		auto collect_unique_edges = [&edges](uint16_t i0, uint16_t i1) {
			const Edge e = Edge{i0, i1}.sort();
			auto it = edges.find(e);
			if (it == edges.end()) {
				edges.insert(e);
			} else {
				edges.erase(it);
			}
		};

		// Collect all edges of the current triangle
		Triangle &t = triangles[bad_triangles[j]];
		collect_unique_edges(t.i0, t.i1);
		collect_unique_edges(t.i1, t.i2);
		collect_unique_edges(t.i2, t.i0);

		// Mark the triangle as invalid
		t.i0 = 0, t.i1 = 0, t.i2 = 0;
	}

	// Remove invalid triangles from the triangle list
	triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
	                               [](const Triangle &t) {
		                               return (t.i0 == 0) && (t.i1 == 0) &&
		                                      (t.i2 == 0);
		                           }),
	                triangles.end());
}

/**
 * Class used to comfortably write a stream of bits into a stream of bytes.
 */
class Bitstream {
private:
	std::vector<uint8_t> m_bytes;
	uint8_t m_buf = 0;
	uint8_t m_bit_cursor = 0;

public:
	/**
	 * Stores the least-significant n_bits bits of the integer i in the
	 * bitstream.
	 *
	 * @param i is an integer containing the data that should be written to the
	 * bitstream.
	 * @param n_bits is the number of least-significant bits stored in i that
	 * should be written to the bitstream. Less significant bits are written
	 * first.
	 */
	void write(uint64_t i, uint8_t n_bits) {
		while (n_bits > 0) {
			// Calculate the number of bits to read
			const uint8_t c = std::min<uint8_t>(n_bits, 8 - m_bit_cursor);

			// Calculate a mask for fetching the bits from the input buffer
			const uint64_t mask = (1ULL << c) - 1ULL;

			m_buf = (m_buf << c) | (i & mask);  // Mask the corresponding bits
			i = i >> c;                         // Discard the read bits
			m_bit_cursor += c;                  // Advance the bit cursor
			n_bits -= c;  // Decrease the number of bits to read

			// After reading 8 bits add the byte to the bit buffer
			if (m_bit_cursor == 8) {
				m_bytes.push_back(m_buf);
				m_buf = 0;
				m_bit_cursor = 0;
			}
		}
	}

	/**
	 * Code storing a variable length integer. These are incoded by a
	 * continuation bit followed by 7 bit of payload data.
	 */
	void write_varint(uint64_t i) {
		while (i > 0) {
			if (i > 127) {
				write(1, 1);
			} else {
				write(0, 1);
			}
			write(i, 7);
			i = i >> 7;
		}
	}

	/**
	 * Writes the last pending byte to the result list.
	 */
	void sync() {
		if (m_bit_cursor) {
			write(0, 8 - m_bit_cursor);
			m_bytes.push_back(m_buf);
			m_buf = 0;
			m_bit_cursor = 0;
		}
	}

	/**
	 * Returns a reference at the underlying byte buffer.
	 */
	std::vector<uint8_t> &bytes() {
		sync();
		return m_bytes;
	}
};

/**
 * Sorts the triangles in the triangle list according to the coordinates of
 * their top-left vertex.
 */
static void sort_triangles_spatially(const std::vector<Point> &pnts,
                                     std::vector<Triangle> &triangles) {
	std::sort(triangles.begin(), triangles.end(),
	          [&](const Triangle &t0, const Triangle &t1) -> bool {
		          // Find the left-most point in each triangle, then sort for
		          // the y-coordinate of that point
		          auto select = [&](const Triangle &t) -> Point {
			          return std::min({pnts[t.i0], pnts[t.i1], pnts[t.i2]});
			      };
		          return select(t0) < select(t1);
		      });
}

/**
 * Helper function encoding a byte array as base64. Adapted from
 * https://stackoverflow.com/a/6782480/2188211
 */
std::string base64_encode(const uint8_t *c, size_t n) {
	static const char tbl[] = {
	    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M',
	    'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
	    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
	    'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
	    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'};
	static const int mod_table[] = {0, 2, 1};

	std::string res(4 * ((n + 2) / 3), '\0');
	for (size_t i = 0, j = 0; i < n;) {
		const uint32_t octet_a = i < n ? c[i++] : 0;
		const uint32_t octet_b = i < n ? c[i++] : 0;
		const uint32_t octet_c = i < n ? c[i++] : 0;

		const uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

		res[j++] = tbl[(triple >> 18) & 0x3F];
		res[j++] = tbl[(triple >> 12) & 0x3F];
		res[j++] = tbl[(triple >> 6) & 0x3F];
		res[j++] = tbl[(triple >> 0) & 0x3F];
	}

	for (int i = 0; i < mod_table[n % 3]; i++) {
		res[res.size() - 1 - i] = '=';
	}

	return res;
}

/**
 * Datastructure storing the indices of the adjacent triangles.
 */
using TriangleAdjacency = std::vector<std::array<uint16_t, 3>>;

/**
 * Builds a triangle adjacency, i.e. for each triangle a list is constructed
 * which contains the -- at most three -- neighbouring triangles. Note that the
 * given list of triangles must have sorted indices.
 */
static TriangleAdjacency compute_triangle_adjacency(
    const std::vector<Triangle> &triangles) {
	static constexpr uint16_t F = 0xFFFF;  // Free triangle index
	TriangleAdjacency triangle_adjacency(triangles.size(),
	                                     std::array<uint16_t, 3>{F, F, F});

	// Stores adjacency between triangle i and j
	auto add_triangle_adjacency = [&](uint16_t i, uint16_t j) -> void {
		for (int k = 0; k < 3; k++) {
			if (triangle_adjacency[i][k] == F) {
				triangle_adjacency[i][k] = j;
				break;
			}
		}
	};

	// Temporary datastructure for detecting adjacent edges. Any Edge is at most
	// shared by two triangles, so this structure only stores associations
	// of any Edge to the first triangle. As soon as a second triangle is found
	// the information stored in the map about this edge is no longer relevant
	// and the entry will be removed.
	std::unordered_map<Edge, uint16_t> edge_adjacency;

	// Iterate over all triangles and each edge of every triangle. Fill the
	// "edge_adjacency" structure. As soon as two triangles sharing an edge
	// are found, update the triangle_adjacency structure
	for (uint16_t i = 0; i < triangles.size(); i++) {
		auto process_edge = [&](Edge e) {
			auto it = edge_adjacency.find(e);
			if (it != edge_adjacency.end()) {
				add_triangle_adjacency(i, it->second);
				add_triangle_adjacency(it->second, i);
				edge_adjacency.erase(it);
			} else {
				edge_adjacency.emplace(e, i);
			}
		};

		// Add all edges. Make sure the edges are sorted by index.
		const Triangle &t = triangles[i];
		process_edge(Edge{t.i0, t.i1});
		process_edge(Edge{t.i1, t.i2});
		process_edge(Edge{t.i0, t.i2});
	}

	// Sort the adjacency lists to spatially direct the graph traversal
	for (uint16_t i = 0; i < triangles.size(); i++) {
		std::sort(triangle_adjacency[i].begin(), triangle_adjacency[i].end());
	}
	return triangle_adjacency;
}

/**
 * Analyses the two adjacent triangles t0 and t1 and decomposes their indices
 * into two unshared indices and two shared indices. The result is structured
 * as follows
 *
 *                       0
 *                      / \
 *                     /   \      <----- t0
 *                    /     \
 *                   1-------2
 *                    \     /
 *                     \   /      <----- t1
 *                      \ /
 *                       3
 *
 * i.e. indices 0 and 3 of the result array are not shared, indices 1 and 2 are
 * shared.
 */
static std::array<uint16_t, 4> decompose_adjacent_triangles(
    const Triangle &t0, const Triangle &t1) {
	static constexpr std::array<uint8_t, 3> P0{1, 0, 0};
	static constexpr std::array<uint8_t, 3> P1{2, 2, 1};

	const std::array<uint16_t, 3> a{t0.i0, t0.i1, t0.i2};
	const std::array<uint16_t, 3> b{t1.i0, t1.i1, t1.i2};

	for (uint8_t k0 = 0; k0 < 3; k0++) {
		for (uint8_t k1 = 0; k1 < 3; k1++) {
			if (a[P0[k0]] == b[P0[k1]] && a[P1[k0]] == b[P1[k1]]) {
				return {a[k0], a[P0[k0]], a[P1[k0]], b[k1]};
			}
		}
	}
	return {0, 0, 0, 0};  // must not happen
}

/**
 * Greedily searches for a triangle strip in the adjacency graph.
 */
template <typename Callback>
static void traverse_triangle_adjacency_graph(
    const TriangleAdjacency &adj, const std::vector<Triangle> &triangles,
    uint16_t max_strip_len,
    Callback f) {
	static constexpr uint16_t F = 0xFFFF;  // Free triangle index
	std::vector<bool> visited(triangles.size(), false);
	size_t i0 = 0;  // Unvisited node search start index
	size_t n_visited = 0;
	while (n_visited < triangles.size()) {
		uint16_t cur_strip_len = 0;
		uint16_t cur_i = F;
		std::array<uint16_t, 2> last_idcs{F, F};

		// Helper function which visits the triangle with index i
		auto visit = [&](uint16_t i) {
			// Mark this triangle as the current triangle
			cur_i = i;

			// Visit the given triangle
			f(i);
			visited[i] = true;
			n_visited++;
			cur_strip_len++;

			// If possible, start searching for univisted triangles at this
			// index
			if (i == i0) {
				i0++;
			}
		};

		// Search for a start triangle
		for (size_t i = i0; i < triangles.size(); i++) {
			if (!visited[i]) {
				visit(i);
				last_idcs = {F, F};
				break;
			}
		}

		// Continue until no triangle that continues the triangle strip has been
		// found
		while (cur_i != F && cur_strip_len < max_strip_len) {
			// Select the current triangle, search for an adjacent triangle that
			// continues the triangle strip
			const uint16_t i = cur_i;
			cur_i = F;
			for (int k = 2; k >= 0; k--) {
				const uint16_t j = adj[i][k];
				if (j != F && !visited[j]) {
					const auto idcs = decompose_adjacent_triangles(
					    triangles[i], triangles[j]);
					if ((idcs[1] == last_idcs[0] && idcs[2] == last_idcs[1]) ||
					    (idcs[2] == last_idcs[0] && idcs[1] == last_idcs[1]) ||
					    (last_idcs[0] == F && last_idcs[1] == F)) {
						visit(j);
						if (last_idcs[1] == F) {
							last_idcs = {idcs[2], idcs[3]};
						} else {
							last_idcs = {last_idcs[1], idcs[3]};
						}
						break;
					}
				}
			}
		}

		// Send a "end of triangle strip" marker, since the above condition
		// was false
		f(F);
	}
}

/******************************************************************************
 * Struct Triangulation::Triangle                                             *
 ******************************************************************************/

bool Triangle::is_ccw(const std::vector<Point> &pnts) const {
	const Point &A = pnts[i0], &B = pnts[i1], &C = pnts[i2];
	return (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y) > 0;
}

Triangle Triangulation::Triangle::sort_ccw(
    const std::vector<Point> &pnts) const {
	if (is_ccw(pnts)) {
		return Triangle{i0, i1, i2};
	} else {
		return Triangle{i0, i2, i1};
	}
}

Triangle Triangulation::Triangle::sort() const {
	if (i0 <= i1) {
		if (i1 <= i2) {
			return Triangle{i0, i1, i2};
		} else if (i0 <= i2) {
			return Triangle{i0, i2, i1};
		} else {
			return Triangle{i2, i0, i1};
		}
	} else {
		if (i0 <= i2) {
			return Triangle{i1, i0, i2};
		} else if (i1 <= i2) {
			return Triangle{i1, i2, i0};
		} else {
			return Triangle{i2, i1, i0};
		}
	}
}

bool Triangulation::Triangle::in_circumcircle(const std::vector<Point> &pnts,
                                              size_t i3) const {
	// See https://en.wikipedia.org/wiki/Delaunay_triangulation
	// as well as https://en.wikipedia.org/wiki/Determinant
	auto sq = [](int16_t i) -> int { return int(i) * int(i); };
	const Point &A = pnts[i0], &B = pnts[i1], &C = pnts[i2], &D = pnts[i3];
	int64_t a = A.x - D.x, b = A.y - D.y, c = sq(A.x - D.x) + sq(A.y - D.y);
	int64_t d = B.x - D.x, e = B.y - D.y, f = sq(B.x - D.x) + sq(B.y - D.y);
	int64_t g = C.x - D.x, h = C.y - D.y, i = sq(C.x - D.x) + sq(C.y - D.y);

	return a * (e * i - f * h) + b * (f * g - d * i) + c * (d * h - e * g) > 0;
}

/******************************************************************************
 * Struct Triangulation                                                       *
 ******************************************************************************/

void Triangulation::extract_features(std::vector<Point> &pnts,
                                     std::vector<float> &values,
                                     const uint8_t *buf, int16_t width,
                                     int16_t height, size_t stride,
                                     uint8_t resolution) {
	// Check the parameters for soundness, use some handy aliases
	const int w = std::max<int16_t>(0, width);
	const int h = std::max<int16_t>(0, height);
	const int res = std::min<uint8_t>(254, resolution);
	if (w == 0 || h == 0 || res < 3 || 2 * res > w || 2 * res > h ||
	    stride < size_t(w * 3)) {
		throw std::runtime_error("Invalid argument");
	}

	// Maximum size of the inner patches. Add border for the sobel filter.
	const int patch_size_w = ((w + res - 2) / (res - 1)) + 2;
	const int patch_size_h = ((h + res - 2) / (res - 1)) + 2;
	std::vector<float> L(patch_size_w * patch_size_h);

	// Routine for fetching a pixel value
	auto get = [&](int x, int y) -> const Color & {
		return *(reinterpret_cast<const Color *>(&buf[x * 3 + y * stride]));
	};

	// Routine for fetching a luminance value from the "L" patch
	auto get_L = [&](int i, int j) -> float { return L[i + j * patch_size_w]; };

	// Reserve enough memory for all result points
	pnts.resize(res * res);
	values.resize(res * res, 0.0f);
	for (int ix = 0; ix < res; ix++) {
		for (int iy = 0; iy < res; iy++) {
			// Current index within the result lists
			const size_t idx = iy + ix * res;

			// Check whether this is a point on the border -- if yes, just add
			// this point to the result and continue
			if ((ix == 0) || (iy == 0) || (ix + 1 == res) || (iy + 1 == res)) {
				pnts[idx] = Point{int16_t(ix * (w - 1) / (res - 1)),
				                  int16_t(iy * (h - 1) / (res - 1))};
				continue;
			}

			// Determine the coordinates of the rectangle over which to
			// sweep for this point
			const int x0 = (w * ix - w / 2) / (res - 1);
			const int x1 = (w * ix + w / 2) / (res - 1);
			const int y0 = (h * iy - h / 2) / (res - 1);
			const int y1 = (h * iy + h / 2) / (res - 1);
			const int pw = x1 - x0, ph = y1 - y0;

			// Step one: Calculate the brightness of each pixel in the patch.
			// The brightness is stored in memory since calculation of the
			// luminance is computationally intensive and each pixel is accessed
			// multiple times when calculating the Sobel filtered version.
			for (int x = x0 - 1, i = 0; x < x1 + 1; x++, i++) {
				for (int y = y0 - 1, j = 0; y < y1 + 1; y++, j++) {
					L[i + j * patch_size_w] = luminance(get(x, y));
				}
			}

			// Step two: compute the squared magnitude of the vector
			// corresponding to the Sobel-filtered image in both horizontal and
			// vertical direction, track the maximum value
			float max_L_sq = 0.0;
			int px = (x1 + x0) / 2,
			    py = (y1 + y0) / 2;  // Center if no gradient
			for (int i = 1; i <= pw; i++) {
				for (int j = 1; j <= ph; j++) {
					// clang-format off
					const float f1 =
					    -        get_L(i - 1, j - 1)
					    - 2.0f * get_L(i - 1, j    )
					    -        get_L(i - 1, j + 1)
					    +        get_L(i + 1, j - 1)
					    + 2.0f * get_L(i + 1, j    )
					    +        get_L(i + 1, j + 1);
					const float f2 =
					    -        get_L(i - 1, j - 1)
					    - 2.0f * get_L(i    , j - 1)
					    -        get_L(i + 1, j - 1)
					    +        get_L(i - 1, j + 1)
					    + 2.0f * get_L(i    , j + 1)
					    +        get_L(i + 1, j + 1);
					// clang-format on
					const float L_sq = f1 * f1 + f2 * f2;
					if (L_sq > max_L_sq) {
						max_L_sq = L_sq, px = x0 + i - 1, py = y0 + j - 1;
						values[idx] = std::sqrt(L_sq);
					}
				}
			}

			// Step three: add the x, y coordinate with the maximum gradient
			// to the result list
			pnts[idx] = Point{int16_t(px), int16_t(py)};
		}
	}
}

void Triangulation::select_features(const std::vector<Point> &pnts,
                                    const std::vector<float> &values,
                                    std::vector<bool> &selected, int16_t width,
                                    int16_t height, uint16_t max_count,
                                    float threshold) {
	// Sanity checks
	const int res = sqrti(pnts.size());
	const int w = std::max<int16_t>(0, width);
	const int h = std::max<int16_t>(0, height);
	if (w == 0 || h == 0 || (pnts.size() >= (1 << 16)) || (res < 3) ||
	    (size_t(res * res) != pnts.size()) || pnts.size() != values.size()) {
		throw std::runtime_error("Invalid argument");
	}
	if (max_count == 0) {
		max_count = res * res;
	}

	// Resize the result list to the total resolution
	selected.resize(pnts.size(), false);

	// Select some border points
	const int nb = std::max<int>(3, sqrti(max_count) / 2);
	std::vector<int> border_idcs(nb);
	for (int i = 0; i < nb; i++) {
		border_idcs[i] = (i * res - 1) / (nb - 1);
	}
	for (int i = 0; i < nb; i++) {
		for (int j = 0; j < nb; j++) {
			if (i == 0 || i == nb - 1 || j == 0 || j == nb - 1) {
				selected[border_idcs[i] * res + border_idcs[j]] = true;
			}
		}
	}
	size_t count = 4 * (nb - 1);  // Current number of points

	// Calculate the maximum value, threshold is relative to that value
	const float max_value = *std::max_element(values.begin(), values.end());
	threshold *= max_value;

	// Slowly increase the number of subdivisions
	for (int s = 3; s <= res; s++) {
		for (int ix = 0; ix < s; ix++) {
			for (int iy = 0; iy < s; iy++) {
				// Calculate the bounds of this cell in pixel
				const int x0 = std::max(0, (w * ix - w / 2) / (s - 1));
				const int x1 = std::min(w - 1, (w * ix + w / 2) / (s - 1));
				const int y0 = std::max(0, (h * iy - h / 2) / (s - 1));
				const int y1 = std::min(h - 1, (h * iy + h / 2) / (s - 1));

				// Calculate the search area by discretising to the maximum
				// resolution (pnts and values are organised in a grid pattern)
				const int cx0 = (x0 * (res - 1) + w / 2) / w;
				const int cx1 = (x1 * (res - 1) + w / 2) / w;
				const int cy0 = (y0 * (res - 1) + h / 2) / h;
				const int cy1 = (y1 * (res - 1) + h / 2) / h;

				// Within the cell, search for the point with the maximum value
				size_t max_idx = 0;
				float max_value = -1.0f;
				for (int cx = cx0; cx <= cx1; cx++) {
					for (int cy = cy0; cy <= cy1; cy++) {
						// Calculate the index of this point, skip it if it is
						// already selected
						const size_t idx = cy + cx * res;
						if (selected[idx]) {
							continue;
						}

						// Fetch the point and the value
						const Point &p = pnts[idx];
						const float &v = values[idx];

						// Make sure the point is within bounds, track the point
						// with the maximum value
						if (p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1 &&
						    v > max_value) {
							max_value = v;
							max_idx = idx;
						}
					}
				}

				// Select the maximum point, abort if the number of points is
				// equal to max_count
				if (max_value >= threshold) {
					selected[max_idx] = true;
					count++;  // Increment the point count
					if (count >= max_count) {
						return;
					}
				}
			}
		}
	}
}

void Triangulation::delaunay(std::vector<Point> &pnts,
                             std::vector<Triangle> &triangles) {
	// Add a triangle to the triangle list encompassing all points in the list
	const size_t super_triangle =
	    bowyer_watson_build_super_triangle(pnts, triangles);

	// Sequentially add all points from the point list to the triangulation
	std::vector<uint32_t> bad_triangles;
	std::unordered_set<Edge> edges;
	for (size_t i = 0; i < super_triangle; i++) {
		// Reset the temporary lists
		bad_triangles.clear();
		edges.clear();

		// Mark all triangles where p is inside the circumcircle as "bad"
		for (size_t j = 0; j < triangles.size(); j++) {
			if (triangles[j].in_circumcircle(pnts, i)) {
				bad_triangles.push_back(j);
			}
		}

		// Remove bad triangles and collect the outer edge of the resulting hole
		bowyer_watson_punch_hole(bad_triangles, triangles, edges);

		// Fill the hole by adding triangles back in
		for (const Edge &e : edges) {
			triangles.emplace_back(
			    Triangle{e.i0, e.i1, uint16_t(i)}.sort_ccw(pnts));
		}
	}

	// Remove the super-triangle
	bowyer_watson_remove_super_triangle(super_triangle, pnts, triangles);
}

void Triangulation::mean_colors(const std::vector<Point> &pnts,
                                const std::vector<Triangle> &triangles,
                                std::vector<Color> &colors, const uint8_t *buf,
                                int16_t width, int16_t height, size_t stride) {
	// Check the parameters for soundness, use some handy aliases
	const int w = std::max<int16_t>(0, width);
	const int h = std::max<int16_t>(0, height);
	if (w == 0 || h == 0 || stride < size_t(w * 3)) {
		throw std::runtime_error("Invalid argument");
	}

	// Iterate over the triangles, rasterise them and calculate the average
	// color
	colors.resize(triangles.size());
	for (size_t i = 0; i < triangles.size(); i++) {
		// Rasterise the triangle, count the number of pixels and accumulate
		// linearised r, g, b
		size_t n = 0;
		float r = 0.0f, g = 0.0f, b = 0.0f;
		triangles[i].rasterize(pnts, [&](const Point &p) {
			const Color &c = *(
			    reinterpret_cast<const Color *>(&buf[p.x * 3 + p.y * stride]));
			r += srgb_to_linear(c.r / 255.0f);
			g += srgb_to_linear(c.g / 255.0f);
			b += srgb_to_linear(c.b / 255.0f);
			n++;
		});

		// Calculate the average and convert to sRGB
		r = linear_to_srgb(r / n) * 255.0f;
		g = linear_to_srgb(g / n) * 255.0f;
		b = linear_to_srgb(b / n) * 255.0f;
		colors[i] = Color{uint8_t(r), uint8_t(g), uint8_t(b)};
	}
}

Triangulation Triangulation::from_image(const uint8_t *buf, int16_t width,
                                        int16_t height, size_t stride,
                                        uint8_t resolution, uint16_t max_count,
                                        float threshold) {
	Triangulation res;

	{
		// Temporary information about the selected features
		std::vector<float> values;   // Feature strength
		std::vector<bool> selected;  // Selected index

		// Extract the feature points from the image
		extract_features(res.pnts, values, buf, width, height, stride,
		                 resolution);

		// If max_count is not set to zero and the number of points is larger
		// than max_count, select a subset of the extracted features
		if ((max_count > 0 && max_count < res.pnts.size()) || threshold > 0.0) {
			// Select good feature points among those generated above
			select_features(res.pnts, values, selected, width, height,
			                max_count, threshold);

			// Remove points that have not been selected
			res.pnts.erase(
			    std::remove_if(res.pnts.begin(), res.pnts.end(),
			                   [&selected, &res](const Point &p) {
				                   return !selected[&p - &res.pnts[0]];
				               }),
			    res.pnts.end());
		}
	}

	// Generate a triangulation of the points
	delaunay(res.pnts, res.triangles);

	// Sort the generated triangles spatially
	sort_triangles_spatially(res.pnts, res.triangles);

	// Calculate the average colors
	mean_colors(res.pnts, res.triangles, res.colors, buf, width, height,
	            stride);

	return res;
}

void Triangulation::encode_svg(std::ostream &os, float blur) const {
	const Rect r = bounds(pnts);
	os << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << r.w()
	   << " " << r.h() << "\">";
	if (blur > 0.0f) {
		os << "<filter id=\"blur\"><feGaussianBlur in=\"SourceGraphic\" "
		      "stdDeviation=\""
		   << blur << "\"/></filter>";
		os << "<g filter=\"url(#blur)\"";
	} else {
		os << "<g";
	}
	os << " stroke-width=\"1\" stroke-linecap=\"round\">";
	for (size_t i = 0; i < triangles.size(); i++) {
		const Triangle &t = triangles[i];
		const Color &c = colors[i];
		os << "<path d=\""
		   << "M " << pnts[t.i0].x << " " << pnts[t.i0].y << " "
		   << "L " << pnts[t.i1].x << " " << pnts[t.i1].y << " "
		   << "L " << pnts[t.i2].x << " " << pnts[t.i2].y << " Z\""
		   << " fill=\"rgb(" << int(c.r) << "," << int(c.g) << "," << int(c.b)
		   << ")\""
		   << " stroke=\"rgb(" << int(c.r) << "," << int(c.g) << "," << int(c.b)
		   << ")\"/>";
	}
	os << "</g>";
	os << "</svg>";
}

std::vector<uint8_t> Triangulation::encode_bitstream() {
	// Fetch the bounding rectangle and calculate the number of bits needed
	// to encode the various entities
	const Rect r = bounds(pnts);
	const int16_t max_dim = std::max(r.w(), r.h());
	const uint8_t n_bit_pnt = std::ceil(std::log2(max_dim));
	const uint8_t n_bit_pnt_idx = std::ceil(std::log2(pnts.size()));

	// Write the header
	Bitstream bs;
	bs.write_varint(pnts.size());
	bs.write_varint(triangles.size());
	bs.write(n_bit_pnt, 8);

	// Write the point coordinates
	for (const Point &p : pnts) {
		const int64_t x =
		    (float(p.x - r.p0.x) / max_dim) * ((1ULL << n_bit_pnt) - 1ULL);
		const int64_t y =
		    (float(p.y - r.p0.y) / max_dim) * ((1ULL << n_bit_pnt) - 1ULL);
		bs.write(x, n_bit_pnt);
		bs.write(y, n_bit_pnt);
	}

	// Sort the triangle indices by index
	std::vector<Triangle> triangles_sorted(triangles.size());
	std::transform(triangles.begin(), triangles.end(), triangles_sorted.begin(),
	               [](const Triangle &t) { return t.sort(); });

	// Compute the triangle adjacency graph on the sorted triangles
	TriangleAdjacency triangle_adjacency =
	    compute_triangle_adjacency(triangles_sorted);

	// Traverse the triangle adjacency graph. Serialise the triangles as
	// triangle strips whenever possible. Store the order in which the triangles
	// are serialised in order to serialise the colors correctly
	std::vector<uint16_t> triangle_order;
	std::vector<uint16_t> strip;
	size_t cnt = 0;
	traverse_triangle_adjacency_graph(
	    triangle_adjacency, triangles_sorted, 17, [&](uint16_t idx) {
		    // If there is still space in the current triangle strip and the
		    // triangle index is not a discontinuity marker, add the index to
		    // the triangle strip
		    if (idx != 0xFFFF) {
			    strip.emplace_back(idx);
			    triangle_order.emplace_back(idx);
			    return;
		    }

		    // Write the current triangle strip
		    if (strip.size() == 1) {
			    bs.write(0, 1);  // Not a triangle strip
			    const Triangle &t = triangles_sorted[strip[0]];
			    bs.write(t.i0, n_bit_pnt_idx);
			    bs.write(t.i1, n_bit_pnt_idx);
			    bs.write(t.i2, n_bit_pnt_idx);
			    cnt++;
		    } else {
			    bs.write(1, 1);                 // A triangle strip
			    bs.write(strip.size() - 2, 4);  // Triangle strip length

			    for (size_t i = 0; i < strip.size() - 1; i++) {
				    // Find the edge common to the two triangles; write the
				    // index belonging to the edge that is NOT shared between
				    // the two
				    const Triangle &t0 = triangles_sorted[strip[i + 0]];
				    const Triangle &t1 = triangles_sorted[strip[i + 1]];
				    std::array<uint16_t, 4> idcs =
				        decompose_adjacent_triangles(t0, t1);
				    if (i == 0) {  // Write the first triangle
					    bs.write(idcs[0], n_bit_pnt_idx);
					    bs.write(idcs[1], n_bit_pnt_idx);
					    bs.write(idcs[2], n_bit_pnt_idx);
					    cnt++;
				    }
				    bs.write(idcs[3], n_bit_pnt_idx);
				    cnt++;
			    }
		    }

		    // Clear the current strip data
		    strip.clear();
		});

	// Write the colors
	for (size_t i = 0; i < triangles.size(); i++) {
		const Color &c = colors[triangle_order[i]];
		bs.write((c.r / 255.0f) * ((1ULL << 5) - 1), 5);
		bs.write((c.g / 255.0f) * ((1ULL << 6) - 1), 6);
		bs.write((c.b / 255.0f) * ((1ULL << 5) - 1), 5);
	}

	// Return the constructed bitstream as a sequence of bytes
	return std::move(bs.bytes());
}

std::string Triangulation::encode_base64() {
	const std::vector<uint8_t> input = encode_bitstream();
	return base64_encode(&input[0], input.size());
}
}
