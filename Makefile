all:NX_flash.o
CC=arm_v4t_le-gcc
OPTION=-O2 -DMODULE -D__KERNEL__ -I/opt/IDU_Resource/linux-2.4.20_mvl31/include

SRC=NX_flash.c

$(SRC):NX_flash.h NX_record.h
	touch $@

%.o:%.c
	$(CC) $(OPTION) -c $?

clean:
	rm -f *.o

