CC = g++ -g
INCS = -I/usr/include/flycapture
LIBS = -lflycapture -lglog
objects = $(patsubst %.cpp, %.o, $(wildcard *.cpp) )

.PHONY: all
all: main

main: ${objects}
	@echo "make $@"
	${CC} -o cameraIpForcer ${objects} ${LIBS}

camera_force_ip_main.o: camera_force_ip_main.cpp camera_force_ip.h
	@echo "make $@"
	${CC} ${INCS} -c $< -o $@

camera_force_ip.o: camera_force_ip.cpp camera_force_ip.h
	@echo "make $@"
	${CC} ${INCS} -c $< -o $@

.PHONY: clean
clean:
	@echo "make $@"
	-${RM} *.o
	-${RM} cameraIpForcer

.PHONY: install
install:
	@echo "make $@"
	cp cameraIpForcer /usr/bin/

print: *.cpp
	@echo "make $@"
	@echo $?
