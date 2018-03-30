#!/usr/bin/env python3

#  AEQVIPEDIS -- SVG Placeholder Generator
#  Copyright (C) 2017, 2018  Andreas Stöckel
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Affero General Public License as
#  published by the Free Software Foundation, either version 3 of the
#  License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Affero General Public License for more details.
#
#  You should have received a copy of the GNU Affero General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Script which generates the demo panels included with the README

import subprocess
import sys
import gzip
import os.path

res_list = [8, 16, 32]
max_count_list = [32, 64, 128, 256, 0]

if len(sys.argv) == 1:
	print("Usage: ./demo.py <IMAGE 1> .. <IMAGE N>");
	sys.exit(1);

h = ""
for res in res_list[::-1]:
	h += " *r* = {} | ".format(res)
print("| Original | " + h + "      |")
print("| :------- | " + ("  ---: | " * len(res_list)) + " :--- |")

for fn in sys.argv[1:]:

	tbl_images = [[None for _ in range(len(max_count_list))] for _ in range(len(res_list) + 1)]
	tbl_captions = [[None for _ in range(len(max_count_list))] for _ in range(len(res_list) + 1)]

	tbl_images[0][0] = fn
	tbl_captions[0][0] = ""

	for i, res in enumerate(res_list):
		for j, max_count in enumerate(max_count_list):
			if (max_count == 0):
				max_count = res * res

			# Generate the output picture
			out = "demo/out/" + os.path.basename(fn) + "_r" + str(res) + "_m" + str(max_count) + ".svg"
			out_jpg = out[:-3] + "jpg"
			args = ["./aequipedis", "-r", str(res), "-m", str(max_count), "-t", "0.05"]
			subprocess.run(args + ["-s", "-o", out, fn])
			subprocess.run(["convert", out, out_jpg])
			os.remove(out)

			# Measure the generated output size in bytes
			process = subprocess.Popen(args + [fn], stdout=subprocess.PIPE)
			data, _ = process.communicate()
			size_bytes = len(data)
			size_bytes_gzip = len(gzip.compress(data))

			tbl_images[i + 1][j] = out_jpg
			tbl_captions[i + 1][j] = "{:.1f}k ({:.1f}k)".format(size_bytes / 1000, size_bytes_gzip / 1000, res, max_count)

	for j in list(range(len(max_count_list)))[::-1]:
		s = ""
		for i in [0] + list(range(len(res_list) + 1))[1::][::-1]:
			jR = len(max_count_list) - 1 - j if i == 0 else j
			if tbl_images[i][jR] is None:
				s += "| "
			else:
				s += "| ![{}]({}) ".format(tbl_captions[i][jR], tbl_images[i][jR])
		if max_count_list[j] > 0:
			s += "| *n* ≤ " + str(max_count_list[j])
		else:
			s += "| all points "
		print(s)

		s = ""
		for i in [0] + list(range(len(res_list) + 1))[1::][::-1]:
			jR = 0 if i == 0 and j == len(max_count_list) - 1 else j
			if tbl_images[i][jR] is None:
				s += "| "
			else:
				s += "| {} ".format(tbl_captions[i][jR])
		s += "|"
		print(s)
