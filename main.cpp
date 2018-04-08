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

#include <argp.h> // GNU argparse

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include "aequipedis.hpp"
using namespace aequipedis;

#define STB_IMAGE_IMPLEMENTATION
#include "lib/stb_image.h"

/******************************************************************************
 * Argument parsing                                                           *
 ******************************************************************************/

struct Arguments {
	const char *image = nullptr;
	uint8_t res = 16;
	uint16_t max_count = 0;
	float threshold = 0.0;
	bool svg = false;
	std::string output = "-";
};

const char *argp_program_version = "aequipedis 1.0";

const char *argp_program_bug_address =
    "https://github.com/astoeckel/aequipedis";

static char doc[] =
    "AEQVIPEDIS -- a program generating triangular vector graphics "
    "placeholders";

static char args_doc[] = "IMAGE";

/* Parse a single option. */
static error_t parse_opt(int key, char *arg, struct argp_state *state)
{
	Arguments *args = static_cast<Arguments *>(state->input);

	switch (key) {
		case 'r': {
			uint64_t i = strtoull(arg, nullptr, 10);
			if (i < 3 || i > 254) {
				std::cerr << "Invalid resolution" << std::endl;
				argp_usage(state);
			}
			args->res = i;
			break;
		}

		case 'm': {
			uint64_t i = strtoull(arg, nullptr, 10);
			if (i >= (1ULL << 16)) {
				std::cerr << "Invalid maximum point count" << std::endl;
				argp_usage(state);
			}
			args->max_count = i;
			break;
		}

		case 't': {
			char *end = arg + strlen(arg);
			double f = strtod(arg, &end);
			if (end != arg + strlen(arg) || f < 0.0 || f > 1.0) {
				std::cerr << "Invalid relative threshold" << std::endl;
				argp_usage(state);
			}
			args->threshold = f;
			break;
		}

		case 's':
			args->svg = true;
			break;

		case 'o':
			args->output = arg;
			break;

		case ARGP_KEY_ARG:
			if (state->arg_num >= 1) {
				argp_usage(state);
			}
			args->image = arg;
			break;

		case ARGP_KEY_END:
			if (state->arg_num != 1) {
				argp_usage(state);
			}
			break;

		default:
			return ARGP_ERR_UNKNOWN;
	}
	return 0;
}

static struct argp_option options[] = {
    {"res", 'r', "i", 0,
     "Finest subdivision resolution min 3, max 254 (default 16)", 0},
    {"max_count", 'm', "i", 0,
     "Maximum number of extracted features (default 0, unlimited)", 0},
    {"threshold", 't', "f", 0,
     "Relative edge detection threshold between 0.0 and 1.0 (default 0.0)", 0},
    {"svg", 's', 0, 0, "Output an SVG image", 0},
    {"output", 'o', "FILE", 0, "Output to FILE instead of standard output", 0},
    {0, 0, 0, 0, 0, 0}};

static struct argp argp = {options, parse_opt, args_doc, doc, 0, 0, 0};

/******************************************************************************
 * Image loading                                                              *
 ******************************************************************************/

struct Image {
	int w, h, n;
	uint8_t *data;

	Image(const char *fn) { data = stbi_load(fn, &w, &h, &n, 3); }

	~Image() { stbi_image_free(data); }

	explicit operator bool() const {
		return data && (w > 0) && (h > 0) && (n > 0);
	}
};

/******************************************************************************
 * Main program                                                               *
 ******************************************************************************/

int main(int argc, char *argv[])
{
	// Parse the command line arguments
	Arguments args;
	argp_parse(&argp, argc, argv, 0, 0, &args);

	// Load the image from the given file
	Image img(args.image);
	if (img && img.w <= 0x7FFF && img.h <= 0x7FFF) {
		// Triangulate the image
		Triangulation triangulation =
		    Triangulation::from_image(img.data, img.w, img.h, img.w * 3,
		                              args.res, args.max_count, args.threshold);

		// Fetch the output stream
		std::ostream *os = &std::cout;
		std::ofstream osf;
		if (args.output != "-") {
			osf.open(args.output);
			os = &osf;
		}

		// Write the encoded image to the output
		if (args.svg) {
			triangulation.encode_svg(*os);
		}
		else {
			(*os) << triangulation.encode_base64();
		}
		(*os) << std::endl;
	}
	else {
		std::cerr << "Error while reading the given image file!" << std::endl;
		return 1;
	}
	return 0;
}
