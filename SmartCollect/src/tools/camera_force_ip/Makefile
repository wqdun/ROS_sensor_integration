INC = -I/usr/include/flycapture
LIBS = -lflycapture -lglog
OBJ = camera_force_ip_main.o camera_force_ip.o

main: $(OBJ)
	g++ -o cameraIpForcer $(OBJ) ${LIBS}

camera_force_ip_main.o: camera_force_ip_main.cpp
	g++ ${INC} -c camera_force_ip_main.cpp

camera_force_ip.o: camera_force_ip.cpp
	g++ ${INC} -c camera_force_ip.cpp

.PHONY: clean
clean:
	-rm *.o
	-rm cameraIpForcer

print: *.cpp
	lpr -p $?
	touch print

