
DRIVER=spd5118
VERSION=0.1

obj-m = $(DRIVER).o

DKMS_FLAGS= -m $(DRIVER) -v $(VERSION)
DKMS_ROOT_PATH=/usr/src/$(DRIVER)-$(VERSION)

KERNEL_BUILD=/lib/modules/`uname -r`/build

all: modules

modules:
	@$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) $@

clean:
	@$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) $@

dkms:
	@mkdir $(DKMS_ROOT_PATH)
	@cp `pwd`/dkms.conf $(DKMS_ROOT_PATH)
	@cp `pwd`/Makefile $(DKMS_ROOT_PATH)
	@cp `pwd`/$(DRIVER).c $(DKMS_ROOT_PATH)
	@dkms add $(DKMS_FLAGS)
	@dkms build $(DKMS_FLAGS)
	@dkms install --force $(DKMS_FLAGS)
	@modprobe $(DRIVER)

dkms_clean:
	@dkms remove $(DKMS_FLAGS) --all
	@rm -rf $(DKMS_ROOT_PATH)
