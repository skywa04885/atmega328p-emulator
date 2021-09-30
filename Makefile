GCC ?= gcc

C_SOURCES += $(shell find . -name *.c)
OBJECTS += $(C_SOURCES:.c=.o)

MAIN_ELF		?= main.elf

GCC_C_ARGS		+= -I./inc
GCC_C_ARGS		+= -Wall -Werror
GCC_C_ARGS		+= -O3

%.o: %.c
	$(GCC) $(GCC_C_ARGS) -c $< -o $@

all: $(OBJECTS)
	$(GCC) $(GCC_LD_ARGS) $(OBJECTS) -o $(MAIN_ELF)
clean:
	rm -rf $(OBJECTS) $(MAIN_ELF)
