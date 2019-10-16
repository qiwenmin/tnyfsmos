CC = sdcc
TARGET = blinky

BOARD ?= stc15w104

OUTPUT = build/$(BOARD)
SRC = $(wildcard *.c)

ifeq ($(BOARD),stc15w104)
	STCCODESIZE ?= 4089
	STCIRAMSIZE ?= 128
	CFLAGS ?= -mmcs51 --code-size $(STCCODESIZE) --iram-size $(STCIRAMSIZE) --xram-size 0 --std-sdcc99 --Werror -MD --disable-warning 190
	FLASHTOOL ?= stcgal
	FLASHFLAGS ?= -P stc15 -b 1200 -l 1200
endif

ifeq ($(BOARD),stm8s103f3)
	CFLAGS ?= -mstm8 --std-sdcc99 --Werror -MD
	FLASHTOOL ?= stm8flash
	FLASHFLAGS ?= -c stlinkv2 -p stm8s103f3 -w
endif

.PHONY: all clean flash

all: $(OUTPUT) $(OUTPUT)/$(TARGET).hex

$(OUTPUT):
	mkdir -p $@

$(OUTPUT)/$(TARGET).hex: $(OUTPUT)/$(TARGET).ihx
	packihx $^ > $@

$(OUTPUT)/$(TARGET).ihx: $(SRC:%.c=$(OUTPUT)/%.rel) $(OUTPUT)/tnyfsmos.rel
	$(CC) -o $@ $(CFLAGS) $^

$(OUTPUT)/tnyfsmos.rel:  ../../tnyfsmos.c
	$(CC) -c -o $@ $(CFLAGS) $<

$(OUTPUT)/%.rel: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

-include $(SRC:%.c=$(OUTPUT)/%.d) $(OUTPUT)/tnyfsmos.d

flash: $(OUTPUT)/$(TARGET).hex
	$(FLASHTOOL) $(FLASHFLAGS) $<

clean:
	rm -rf build
