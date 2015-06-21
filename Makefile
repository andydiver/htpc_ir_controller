MCU?=atmega328p
AVRDUDEMCU?=m328p

CC=/usr/bin/avr-gcc
CFLAGS=-Os -Wall -std=c99 -mcall-prologues -mmcu=$(MCU) -DF_CPU=16000000 
##CFLAGS=-Os -Wall -std=c99 -mcall-prologues -mmcu=$(MCU) -DF_CPU=8000000 

OBJ2HEX=/usr/bin/avr-objcopy
AVRDUDE=/usr/local/bin/avrdude
TARGET=ir_controller
SRCS=$(TARGET).c      
OBJECTS=$(TARGET).o  

$(TARGET).o: $(TARGET).c $(TARGET).h
	$(CC) $(CFLAGS) -c $< -o $@

all : $(TARGET) obj size

$(TARGET): $(OBJECTS) 
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS)

obj: $(TARGET) 
	$(OBJ2HEX) -R .eeprom -O ihex $(TARGET) $(TARGET).hex

install : all
	sudo gpio -g mode 22 out
	sudo gpio -g write 22 0
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -V -U flash:w:$(TARGET).hex
	sudo gpio -g write 22 1

noreset : all
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U flash:w:$(TARGET).hex

fuse :
	sudo gpio -g mode 22 out
	sudo gpio -g write 22 0
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U lfuse:w:0xef:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m  # xtal  SUT=1,0
##	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P /dev/spidev0.0 -c linuxspi -b 10000 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m  #       RC 8MHz
	sudo gpio -g write 22 1

size:
	du -b $(TARGET).hex 
	avr-size $(TARGET)

.PHONY: clean

clean :
	rm -f *.hex *.o $(TARGET)

