/*
 * AEQVIPEDIS -- SVG Placeholder Generator
 * Copyright (C) 2017, 2018  Andreas Stöckel
 *
 * This file is licensed under the MIT License.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

var aequipedis = (function (window) {
	'use strict';

	/**
	 * Decodes the given base64 string into a Uint8Array for byte-wise access to
	 * the binary data.
	 */
	function _decode_base64(s) {
		let raw = window.atob(s);
		let a = new Uint8Array(new ArrayBuffer(raw.length));
		for(let i = 0; i < raw.length; i++) {
			a[i] = raw.charCodeAt(i);
		}
		return a;
	}

	/**
	 * Decodes the triangulation bitstream from the given byte array "a".
	 */
	function _decode_bitstream(a) {
		// Code for conveniently reading individual bits from the bitstream
		let byte_cursor = 0, bit_cursor = 0, byte = a[0];
		function read(n) {
			let res = 0, s = 0;
			while (s < n) {
				// Number of bits to read
				const c = Math.min(8 - bit_cursor, n - s) | 0;

				// Mask containing the LSB to read
				const s0 = 8 - c - bit_cursor
				const mask = (((1 << c) - 1) | 0) << s0;

				// Read the bytes from the mask
				res |= ((byte & mask) >> s0) << s;

				// Shift the global and local bit cursor
				bit_cursor += c;
				s += c;

				// After reading 8 bits, go to the next byte in the byte buffer
				if (bit_cursor === 8) {
					bit_cursor = 0;
					byte = a[++byte_cursor]; // Read the next byte
				}
				if (byte_cursor > a.length) {
					throw "Invalid data";
				}
			}
			return res;
		};

		// Code reading a varint. These are encoded by a continuation bit
		// followed by 7 bit of data
		function read_varint() {
			let res = 0, has_next = 0, offs = 0;
			do {
				has_next = read(1);
				let r = read(7)
				res |= (r << offs) | 0;
				offs += 7;
			} while (has_next);
			return res;
		}

		// Read the header, calculate the number of bits per triangle index
		const n_pnts = read_varint();
		const n_triangles = read_varint();
		if (n_pnts > 0x7FFF || n_triangles > 0x7FFF) {
			throw "Invalid data";
		}
		const n_bit_pnt = read(8);
		if (n_bit_pnt > 16) {
			throw "Invalid data";
		}
		const n_bit_pnt_idx = Math.ceil(Math.log(n_pnts) / Math.log(2)) | 0;

		// Reserve some memory for the triangle/point data
		const pnts = new Float32Array(new ArrayBuffer(4 * 2 * n_pnts));
		const triangles = new Uint16Array(new ArrayBuffer(2 * 3 * n_triangles));
		const colors = new Uint8Array(new ArrayBuffer(3 * n_triangles));

		// Read the point coordinates from the input data and track the maximum
		// point coordinates
		let max_x = 0, max_y = 0;
		for (let i = 0; i < n_pnts; i++) {
			// Read the points
			pnts[2 * i + 0] = read(n_bit_pnt);
			pnts[2 * i + 1] = read(n_bit_pnt);

			// Update the bounding box
			max_x = Math.max(max_x, pnts[2 * i + 0]) | 0;
			max_y = Math.max(max_y, pnts[2 * i + 1]) | 0;
		}

		// Read the triangle indices from the input data
		let idx = 0;
		while (idx < n_triangles) {
			// Number of triangles in the strip
			let n_strip = read(1) ? (read(4) + 2) : 1;

			// Read the first triangle
			triangles[3 * idx + 0] = read(n_bit_pnt_idx);
			triangles[3 * idx + 1] = read(n_bit_pnt_idx);
			triangles[3 * idx + 2] = read(n_bit_pnt_idx);
			idx++;

			// Read the remaining triangles from the triangle strip
			for (let i = 0; i < n_strip - 1; i++) {
				const p = read(n_bit_pnt_idx);
				triangles[3 * idx + 0] = triangles[3 * (idx - 1) + 1];
				triangles[3 * idx + 1] = triangles[3 * (idx - 1) + 2];
				triangles[3 * idx + 2] = p;
				idx++;
			}
		}

		// Read the colors from the input data
		for (let i = 0; i < n_triangles; i++) {
			colors[3 * i + 0] = (read(5) * 255 / 31) | 0;
			colors[3 * i + 1] = (read(6) * 255 / 63) | 0;
			colors[3 * i + 2] = (read(5) * 255 / 31) | 0;
		}

		return [pnts, triangles, colors, max_x, max_y]
	}

	/**
	 * Creates a triangulation from the bitstream encoded in the given
	 * Uint8Array instance. See Triangulation::encode_bitstream() in the C++
	 * code for a description of the data layout.
	 */
	function from_uint8_array(a, blur=0.0) {
		// Decode the bitstream into lists of points, triangle indices and
		// colors
		let [pnts, triangles, colors, max_x, max_y] = _decode_bitstream(a);

		// Quick access routines
		const tx = (i, j) => { return pnts[2 * triangles[3 * i + j] + 0]; };
		const ty = (i, j) => { return pnts[2 * triangles[3 * i + j] + 1]; };
		const cr = (i) => { return colors[3 * i + 0]; }
		const cg = (i) => { return colors[3 * i + 1]; }
		const cb = (i) => { return colors[3 * i + 2]; }

		// Build the SVG root node
		const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
		svg.setAttribute('viewBox', `0 0 ${max_x} ${max_y}`);

		// Add a filter node if bluring is requested
		if (blur > 0.0) {
			const filter = document.createElementNS('http://www.w3.org/2000/svg', 'filter');
			filter.setAttribute('id', 'blur');
			svg.appendChild(filter);

			const feGaussianBlur = document.createElementNS('http://www.w3.org/2000/svg', 'feGaussianBlur');
			feGaussianBlur.setAttribute('in', 'SourceGraphic');
			feGaussianBlur.setAttribute('stdDeviation', blur);
			filter.appendChild(feGaussianBlur);
		}

		// Group all triangles in an SVG group to set stroke properties globally
		const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
		g.setAttribute('stroke-width', '1');
		g.setAttribute('stroke-linecap', 'round');
		if (blur > 0.0) {
			g.setAttribute('filter', 'url(#blur)');
		}
		svg.appendChild(g);

		// Add all triangles
		for (let i = 0; i < triangles.length / 3; i++) {
			const path = document.createElementNS('http://www.w3.org/2000/svg', 'path')
			path.setAttribute('d', `M ${tx(i, 0)} ${ty(i, 0)} L ${tx(i, 1)} ${ty(i, 1)} L ${tx(i, 2)} ${ty(i, 2)} Z`);
			path.setAttribute('fill', `rgb(${cr(i)}, ${cg(i)}, ${cb(i)})`);
			path.setAttribute('stroke', `rgb(${cr(i)}, ${cg(i)}, ${cb(i)})`);
			g.appendChild(path);
		}

		return svg;
	}

	/**
	 * Creates a triangulation from the given base64 encoded string.
	 */
	function from_base64(s, blur=0.0) {
		return from_uint8_array(_decode_base64(s), blur);
	}

	return {
		"from_uint8_array": from_uint8_array,
		"from_base64": from_base64
	}
})(this);
