
# Change this to pick the correct programmer you're using
PROG = usbtiny

# Change this if you're not using a Tiny85
CHIP = attiny841

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude

CFLAGS = -Os -g -mmcu=$(CHIP) -std=c99 $(OPTS) -Wall -Wno-main -fno-tree-switch-conversion

DUDE_OPTS = -c $(PROG) -p $(CHIP)

all: ev_sim.hex

%.elf: %.o
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.hex: %.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

clean:
	rm -f *.o *.elf *.hex

fuse:
	$(AVRDUDE) $(DUDE_OPTS) -U lfuse:w:0xff:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

flash: ev_sim.hex
	$(AVRDUDE) $(DUDE_OPTS) -U flash:w:$^

init: fuse flash

