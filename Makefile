all: aequipedis

lib/stb_image.h:
	mkdir -p lib
	curl 'https://raw.githubusercontent.com/nothings/stb/master/stb_image.h' -o lib/stb_image.h

aequipedis: aequipedis.cpp aequipedis.hpp main.cpp lib/stb_image.h
	g++ -s -o aequipedis -std=c++11 -Wall -Wextra -O3 main.cpp aequipedis.cpp aequipedis.hpp lib/stb_image.h

aequipedis_static: aequipedis.cpp aequipedis.hpp main.cpp lib/stb_image.h
	g++ -static -static-libgcc -static-libstdc++ -s -o aequipedis_static -std=c++11 -Wall -Wextra -O3 main.cpp aequipedis.cpp aequipedis.hpp lib/stb_image.h

clean:
	rm -f aequipedis aequipedis_static
	rm -f lib/stb_image.h
