
# Change this to whatever AVR programmer you want to use.
PROGRAMMER = usbtiny

OUT=ev_sim

# Change this if you're not using a Tiny841
CHIP = attiny841

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude

CFLAGS = -Os -g -mmcu=$(CHIP) -std=c11 -Wall -Wno-main -fno-tree-switch-conversion

DUDE_OPTS = -c $(PROGRAMMER) -p $(CHIP)

all:	$(OUT).hex

%.o:	%.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.elf:	%.o
	$(CC) $(CFLAGS) -o $@ $^

%.hex:	%.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

clean:
	rm -f *.o *.elf *.hex

flash:	$(OUT).hex
	$(AVRDUDE) $(DUDE_OPTS) -U flash:w:$^

fuse:
	$(AVRDUDE) $(DUDE_OPTS) -U lfuse:w:0xff:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

init:	fuse flash

