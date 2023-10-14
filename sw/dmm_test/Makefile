# ======================================================================
#  This Makefile builds an application *.ebf file for Aquila.
#
#  Chun-Jen Tsai, Dec/09/2019
# ======================================================================

CROSS = riscv32-unknown-elf
CCPATH = $(RISCV)/bin

CC = $(CCPATH)/$(CROSS)-gcc
LD = $(CCPATH)/$(CROSS)-ld
AR = $(CCPATH)/$(CROSS)-ar
OD = $(CCPATH)/$(CROSS)-objdump
SP = $(CCPATH)/$(CROSS)-strip

GCCVERSION = $(shell $(CC) --version | grep gcc | sed 's/^.* //g')

LIBC = ../elibc
LIBC_OBJS = $(LIBC)/crt0.o $(LIBC)/stdio.o $(LIBC)/stdlib.o $(LIBC)/string.o $(LIBC)/time.o $(LIBC)/uart.o

LIB_SWMM = sw_malloc
LIB_SWMM_OBJS = $(LIB_SWMM)/mallocr2.o

CCFLAGS = -Wall -O2 -I$(LIBC) -fno-builtin -march=rv32ima_zicsr_zifencei -mstrict-align -mabi=ilp32 -I$(LIB_SWMM)
LDFLAGS = -L. -Map=$*.map
LD_SOFT_FP = -L$(RISCV)/lib/gcc/riscv32-unknown-elf/$(GCCVERSION) -lgcc

OCFLAGS = -O binary
ODFLAGS = -d

PROJ = dmm_test
OBJS = $(PROJ).o

all:
	make $(PROJ).elf

clean:
	rm -f *.elf *.objdump *.map *.o *.a

%.o: %.c $(PROJ).ld
	$(CC) $(CCFLAGS) -c $< -o $@

libelibc.a : $(LIBC_OBJS) $(LIB_SWMM_OBJS)
	$(AR) r libelibc.a $(LIBC_OBJS) $(LIB_SWMM_OBJS)
	rm $^

%.elf: $(OBJS) libelibc.a
	$(LD) $(LDFLAGS) -T$*.ld $(OBJS) -lelibc $(LD_SOFT_FP) -o $@
	$(OD) $(ODFLAGS) $@ > $*.objdump
	$(SP) $@
	rm $^

