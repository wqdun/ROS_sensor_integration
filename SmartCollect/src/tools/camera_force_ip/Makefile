INCS = -I/usr/include/flycapture
LIBS = -lflycapture -lglog
objects := $(patsubst %.cpp, %.o, $(wildcard *.cpp) )

main: $(objects)
	g++ -o cameraIpForcer $(objects) ${LIBS}

camera_force_ip_main.o: camera_force_ip_main.cpp camera_force_ip.h
	g++ ${INCS} -c $< -o $@

camera_force_ip.o: camera_force_ip.cpp camera_force_ip.h
	g++ ${INCS} -c $< -o $@

.PHONY: clean
clean:
	-rm *.o
	-rm cameraIpForcer

print: *.cpp
	lpr -p $(objects)
	touch print

