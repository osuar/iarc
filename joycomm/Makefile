# Compiles TetMotorController.c test app

SYSTEM     = x86
COMMON_DIR = ../marsrover2011/Software/Common
OBJECTS    = joytest.o
PROJECT    = joytest

default: app

include $(COMMON_DIR)/Makefile.app

MotorController.o: ../MotorController.c
	$(CC) $(INCLUDES) $(CFLAGS) -c ../MotorController.c

TetsMotorController.o: joytest.c
	$(CC) $(INCLUDES) $(CFLAGS) -c joytest.c

app: comm serial joytest.o MotorController.o
	$(CC) $(LDFLAGS) $(OBJECTS) MotorController.o $(COMM_LIB)  $(SERIAL_LIB) $(LIBDIRS) $(LIBS) -o $(TARGET)

clean:
	rm -f joytest.o MotorController.o
	rm -f $(PROJECT).map $(PROJECT).elf $(PROJECT).hex $(PROJECT).eep
