obj-m += sllin.o
KPATH=/lib/modules/$(shell uname -r)/build
#KPATH=/mnt/data/_dokumenty_/_w_/_dce_can_/src/can-benchmark/kernel/build/shark/3.0.4
#KPATH=/mnt/data/_dokumenty_/_w_/_dce_can_/src/can-benchmark/kernel/build/shark/2.6.36

all:
	make -C ${KPATH} M=$(PWD) modules

clean:
	make -C ${KPATH} M=$(PWD) clean
