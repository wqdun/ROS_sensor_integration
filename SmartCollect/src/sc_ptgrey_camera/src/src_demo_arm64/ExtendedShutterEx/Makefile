################################################################################
# ExtendedShutterEx Makefile.
################################################################################
PROJECT_ROOT=../../
OPT_INC = ${PROJECT_ROOT}/common/make/common.mk
-include ${OPT_INC}
# Handle environment variables
ifeq ($(wildcard ${OPT_INC}),)
	CXX   = g++
	ODIR  = .obj/build${D}
	SDIR  = .
	MKDIR = mkdir -p
	MV    = mv
endif

BASE_NAME = ExtendedShutterEx
NAME      = ${BASE_NAME}${D}
INC       = -I../../include -I/usr/include/flycapture
LIBS      = -L../../lib -lflycapture${D} ${FC2_DEPS}

UNAME=$(shell uname)
ifeq ($(UNAME),Darwin)
	# Apple Mac OS
	LIBS += -Wl,-rpath,../../lib
else ifeq ($(UNAME),Linux)
	# Linux
	LIBS += -Wl,-rpath-link=../../lib
else
	# Unsupported
endif

OUTDIR    = ${PROJECT_ROOT}/bin

_OBJ      = ${BASE_NAME}.o
SRC       = $(_OBJ:.o=.cpp)
OBJ       = $(patsubst %,$(ODIR)/%,$(_OBJ))

# Master rule
.PHONY: all
all: ${NAME}

# Output binary
${NAME}: ${OBJ}
	${CXX} ${OBJ} -o ${NAME} ${LIBS}
	-@${MV} ${NAME} ${OUTDIR}/${NAME}

# Intermediate object files
${OBJ}: ${ODIR}/%.o : ${SDIR}/%.cpp
	@${MKDIR} ${ODIR}
	${CXX} ${CXXFLAGS} ${LINUX_DEFINES} ${INC} -Wall -c $< -o $@

# Cleanup intermediate objects
.PHONY: clean_obj
clean_obj:
	rm -f ${OBJ}
	@echo "obj cleaned up!"

# Cleanup everything
.PHONY: clean
clean: clean_obj
	rm -f ${OUTDIR}/${NAME} ${OBJ}
	@echo "all cleaned up!"

