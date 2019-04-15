ifneq ($(KERNELRELEASE),)

obj-m += atsha204.o

else
	KERNELDIR ?= /home/xiao/work/sc60/out/target/product/msm8953_64/obj/KERNEL_OBJ/
	PWD ?= $(shell pwd)
	ARCH = arm64
	CROSS_COMPILE=/home/xiao/work/sc60/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-

.PHONY: modules clean

modules:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNELDIR) M=$(PWD) modules
	#这里还要给签名模块!
clean:
	@rm -rf *.o *.order *.symvers *.mod.* .*.o.cmd .*.mod.o.cmd .*.ko.cmd .tmp_versions *.ko

endif
