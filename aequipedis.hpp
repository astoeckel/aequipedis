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

/**
 * @file aequipedis.hpp
 *
 * Code for the generation of a compact SVG placeholder image. This image can be
 * used as a placeholder in client application until the actual image has been
 * fetched from the server. See https://jmperezperez.com/svg-placeholders/
 * for an overview of similar techniques.
 *
 * @author Andreas Stöckel
 */

#ifndef AEQUIPEDIS_HPP
#define AEQUIPEDIS_HPP

#include <algorithm>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace aequipedis {
/**
 * The Triangulation class representes a triangle-mesh approximation of a
 * bitmap. It can either be converted to an SVG or to a space-efficient base64
 * encoded string that can be converted to a SVG using some JavaScript.
 */
struct Triangulation {
	/**
	 * Class corresponding to a single 2D point with integer coordinates.
	 * Limited to 16 bit precision. Don't need more here.
	 */
	struct Point {
		int16_t x, y;

		friend bool operator==(const Point &p1, const Point &p2) {
			return p1.x == p2.x && p1.y == p2.y;
		}
	};

	/**
	 * An axis-aligned rectangle defined by the top-left and bottom-right point.
	 */
	struct Rect {
		/**
		 * Top-left point of the rectangle.
		 */
		Point p0;

		/**
		 * Bottom-right point of the rectangle.
		 */
		Point p1;

		/**
		 * Returns the width of the rectangle.
		 */
		int16_t w() const { return p1.x - p0.x; }

		/**
		 * Returns the height of the rectangle.
		 */
		int16_t h() const { return p1.y - p0.y; }
	};

	/**
	 * Internally used structure.
	 */
	struct Edge;

	/**
	 * Triangle as index list.
	 */
	struct Triangle {
		uint16_t i0, i1, i2;

		/**
		 * Returns true if the triangle is in counter-clockwise orientation.
		 */
		bool is_ccw(const std::vector<Point> &pnts) const;

		/**
		 * Returns a triangle that is guaranteed to be in counter-clockwise
		 * orientation.
		 */
		Triangle sort_ccw(const std::vector<Point> &pnts) const;

		/**
		 * Sorts the triangle indices, creating a unique representation of the
		 * triangle.
		 */
		Triangle sort() const;

		/**
		 * Returns the top-left corner of the triangle bounding box.
		 */
		Point min(const std::vector<Point> &pnts) const {
			return Point{std::min({pnts[i0].x, pnts[i1].x, pnts[i2].x}),
			             std::min({pnts[i0].y, pnts[i1].y, pnts[i2].y})};
		}

		/**
		 * Returns the bottom-right corner of the triangle bounding box.
		 */
		Point max(const std::vector<Point> &pnts) const {
			return Point{std::max({pnts[i0].x, pnts[i1].x, pnts[i2].x}),
			             std::max({pnts[i0].y, pnts[i1].y, pnts[i2].y})};
		}

		/**
		 * Returns true if the given point is inside the triangle.
		 *
		 * @param pnts is the list of points underlying the triangle indices.
		 * @param p is the point that should be checked.
		 * @param
		 */
		bool in_triangle(const std::vector<Point> &pnts, const Point &p) const {
			// See
			// https://fgiesen.wordpress.com/2013/02/08/triangle-rasterization-in-practice/
			auto orient2d = [](const Point &a, const Point &b,
			                   const Point &c) -> int {
				return int(b.x - a.x) * int(c.y - a.y) -
				       int(b.y - a.y) * int(c.x - a.x);
			};

			int w0 = orient2d(pnts[i1], pnts[i2], p);
			int w1 = orient2d(pnts[i2], pnts[i0], p);
			int w2 = orient2d(pnts[i0], pnts[i1], p);
			return (w0 >= 0 && w1 >= 0 && w2 >= 0);
		}

		/**
		 * Rasterises the triangle. Calls "f" for each pixel within the
		 * triangle. This is used internally to sample the average color under
		 * a triangle.
		 *
		 * @param pnts is the list of points underlying the triangle indices.
		 * @param f is a function that is being called for each point in the
		 * triangle.
		 */
		template <typename F>
		void rasterize(const std::vector<Point> &pnts, F f) const {
			// Fetch the bounding box
			const Point pmin = min(pnts);
			const Point pmax = max(pnts);

			// Iterate over the bounding box and check whether the pixel is
			// inside the triangle. If this is the case, call the given callback
			// function.
			Point p;
			for (p.x = pmin.x; p.x <= pmax.x; p.x++) {
				for (p.y = pmin.y; p.y <= pmax.y; p.y++) {
					if (in_triangle(pnts, p)) {
						f(p);
					}
				}
			}
		}

		/**
		 * Returns true if the point with index i3 is within the circumcircle
		 * of the triangle.
		 *
		 * @param pnts is the list of points the triangle indices are refering
		 * to.
		 * @param i3 is the index of the point that should be checked for lying
		 * within the circumcircle of the triangle.
		 */
		bool in_circumcircle(const std::vector<Point> &pnts, size_t i3) const;
	};


#pragma pack(push, 1)  // Make sure the color struct is packed into 24 bits
	/**
	 * RGB color value.
	 */
	struct Color {
		uint8_t r, g, b;
	};
#pragma pack(pop)

	/**
	 * Underlying points. Points are in pixel space.
	 */
	std::vector<Point> pnts;

	/**
	 * List of triangles that are part of the triangulation. Triangle indices
	 * refer to the points in the list "pnts".
	 */
	std::vector<Triangle> triangles;

	/**
	 * Colors of the triangles listed in the list "triangles". Color are the
	 * mean color over the triangle (mean is calculated in linear color space).
	 * Colors in the color list are in sRGB space.
	 */
	std::vector<Color> colors;

	/**
	 * Extracts the initial feature points from the given image data. Note that
	 * this also includes a set of "border points" that are uniformly spaced
	 * around the image bounding rectangle.
	 *
	 * @param pnts is a list of 2D points to which the extracted feature points
	 * will be written. The resulting list size is resolution squared.
	 * @param values is an array of floating point numbers corresponding to the
	 * strength of each feature point.
	 * @param buf is a pointer at the RGB data of the image. The image data is
	 * assumed to be in sRGB color space and is linearised before processing.
	 * Color channels are interleaved and packed at 24 bit per pixel.
	 * @param width is the width of the image in pixels.
	 * @param height is the height of the image in pixels.
	 * @param stride is the number of bytes in one line in pixels.
	 * @param resolution determines the number of points along one axis used
	 * in the triangulation (not counting the points on the boundary).
	 * Resolution is clamped to 254 to prevent more than 65536 points in the
	 * resulting point list (the triangle indices are only 16 bit).
	 */
	static void extract_features(std::vector<Point> &pnts,
	                             std::vector<float> &values, const uint8_t *buf,
	                             int16_t width, int16_t height, size_t stride,
	                             uint8_t resolution);

	/**
	 * Selects the given number of feature points from the list of extracted
	 * feature points. The selection process takes the spatial distribution
	 * of the points into account, i.e. there are no completely empty regions
	 * in the image without any feature points. Still, the algorithm tries to
	 * concentrate feature points in regions with higher feature intensity.
	 *
	 * @param pnts is a list of feature points as extracted by the
	 * extract_features method.
	 * @param values is a list of values associated with each feature point.
	 * @param selected is the target array to which a flag indicating whether
	 * the corresponding point is selected or not is written.
	 * @param width is the width of the original image.
	 * @param height is the height of the original image.
	 * @param max_count is the maximum number of points. If zero all points
	 * that are above the threshold are added.
	 * @param threshold is the relative feature strength below which points
	 * are rejected.
	 */
	static void select_features(const std::vector<Point> &pnts,
	                            const std::vector<float> &values,
	                            std::vector<bool> &selected, int16_t width,
	                            int16_t height, uint16_t max_count,
	                            float threshold);

	/**
	 * Calculates a Delaunay triangulation of the given points using the
	 * BowyerWatson algorithm.
	 *
	 * @param pnts is a list of 2D points. This function will temporarily add
	 * points to the list which may invalidate pointers to the points in the
	 * point list.
	 * @param triangles is the list of triangles (indicated by indices into the
	 * point list
	 */
	static void delaunay(std::vector<Point> &pnts,
	                     std::vector<Triangle> &triangles);

	static void mean_colors(const std::vector<Point> &pnts,
	                        const std::vector<Triangle> &triangles,
	                        std::vector<Color> &colors, const uint8_t *buf,
	                        int16_t width, int16_t height, size_t stride);

	/**
	 * Generates a triangle approximation of the given bitmap. This is used to
	 * generate placeholder images for the cover art which are included in the
	 * album information and displayed while the actual cover art image is being
	 * downloaded.
	 *
	 * The process for the generation of these placeholder images is as follows:
	 * First, a set of feature points is extractec from the images. These
	 * feature points are located on edges of the image. The number of feature
	 * points depends on the given resolution (number of feature points is
	 * resolution squared). Then, the feature points as well as a number of
	 * points on the boundary of the image are triangulated into a Delaunay
	 * triangulation using the BowyerWatson algorithm. These are then
	 *
	 * @param buf is a pointer at the RGB data of the image. The image data is
	 * assumed to be in sRGB color space and is linearised before processing.
	 * Color channels are interleaved and packed at 24 bit per pixel.
	 * @param width is the width of the image in pixels.
	 * @param height is the height of the image in pixels.
	 * @param stride is the number of bytes in one line in pixels.
	 * @param resolution determines the number of points along one axis used
	 * in the triangulation (not counting the points on the boundary).
	 * Resolution is clamped to 254 to prevent more than 65536 points in the
	 * resulting point list (the triangle indices are only 16 bit).
	 * @param max_count is the maximum number of points. If zero, all points
	 * that are above the threshold are added.
	 * @param threshold is the relative feature strength below which points
	 * are rejected.
	 */
	static Triangulation from_image(const uint8_t *buf, int16_t width,
	                                int16_t height, size_t stride,
	                                uint8_t resolution = 64,
	                                uint16_t max_count = 32,
	                                float threshold = 0.1);

	/**
	 * Generates an SVG encoding the triangulation.
	 *
	 * @param os is an output stream to which the SVG should be written.
	 * @param blur is an optional standard deviation of a Gaussian blur filter.
	 * Set to zero or a negative value to deactivate bluring entirely.
	 */
	void encode_svg(std::ostream &os, float blur = 0.0f) const;

	/**
	 * Encodes the triangulation as an efficiently packed bitstream. Stores
	 * colors in a 16 bit format. Layout is as follows:
	 *
	 * 8/16/24 bit - Number of points
	 *               --> least significant byte first, if 8th bit is set, read
	 *                   an additional byte containing 7bit of data.
	 *               --> use to determine the number of bits required to index
	 *                   individual points.
	 * 8/16/24 bit - Number of triangles
	 *               --> least significant byte first, if 8th bit is set, read
	 *                   an additional byte containing 7bit of data.
	 * 8 bit       - Number of bit per point
	 *
	 *           ==> Repeat for each point
	 * nBitPnt     - X coords, fixed point decimal in [0, 1]
	 * nBitPnt     - Y coords, fixed point decimal in [0, 1]
	 *
	 *           ==> Repeat for each triangle/triangle strip
	 * 1 bit       - Is triangle strip? 1 => true, 0 => false
	 *
	 *            => Triangle strip
	 * 5 bit       - Length of the triangle strip
	 * nBitPntIdx  - 2 * n + 1 repetitions of the triangle strip point indices
	 *
	 *            => Triangle
	 * nBitPntIdx  - i0
	 * nBitPntIdx  - i1
	 * nBitPntIdx  - i2
	 *
	 *           ==> Repeat for each triangle in the order as they appeared
	 *               above
	 * 5 bit       - Red color information
	 * 6 bit       - Green color information
	 * 5 bit       - Blue color information
	 *
	 */
	std::vector<uint8_t> encode_bitstream();

	/**
	 * Same as encode_bitstream but encodes the binary data as a base64 string
	 * that can be easily incorporated into text based data formats such as
	 * JSON.
	 */
	std::string encode_base64();
};
}

#endif /* AEQUIPEDIS_HPP */
