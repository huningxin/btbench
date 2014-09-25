CC = g++
CFLAGS = -O2 -I./ -I${BT_SRC_PATH} -DNDEBUG=1
LFLAGS = -lstdc++
BTLIBS = ${BT_LIB_PATH}/libBulletDynamics_gmake_x64_release.a ${BT_LIB_PATH}/libBulletCollision_gmake_x64_release.a ${BT_LIB_PATH}/libLinearMath_gmake_x64_release.a

OBJECTS = \
btbench.o

all: btbench

%.o: %.cc
	$(CC) $(CFLAGS) -o $@ -c $<

btbench: $(OBJECTS)
	$(CC) $(LFLAGS) -o $@ $(OBJECTS) $(BTLIBS)

clean:
	rm $(OBJECTS) btbench