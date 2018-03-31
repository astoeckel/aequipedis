all: aequipedis

lib/stb_image.h:
	mkdir -p lib
	curl 'https://raw.githubusercontent.com/nothings/stb/master/stb_image.h' -o lib/stb_image.h

aequipedis: aequipedis.cpp aequipedis.hpp main.cpp lib/stb_image.h
	$(CXX) -s -o aequipedis -std=c++11 -Wall -Wextra -O3 main.cpp aequipedis.cpp

aequipedis_debug: aequipedis.cpp aequipedis.hpp main.cpp lib/stb_image.h
	$(CXX) -o aequipedis_debug -std=c++11 -Wall -Wextra -O0 -g main.cpp aequipedis.cpp

aequipedis_static: aequipedis.cpp aequipedis.hpp main.cpp lib/stb_image.h
	$(CXX) -static -static-libgcc -static-libstdc++ -s -o aequipedis_static -std=c++11 -Wall -Wextra -O3 main.cpp aequipedis.cpp

clean:
	rm -f aequipedis aequipedis_static
#	rm -f lib/stb_image.h
