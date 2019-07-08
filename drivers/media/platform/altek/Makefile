# check linux kernel version if excess 3.18.44
KVER = $(KERNELVERSION)
KMAJ = $(shell echo $(KVER) | \
sed -e 's/^\([0-9][0-9]*\)\.[0-9][0-9]*\.[0-9][0-9]*.*/\1/')
KMIN = $(shell echo $(KVER) | \
sed -e 's/^[0-9][0-9]*\.\([0-9][0-9]*\)\.[0-9][0-9]*.*/\1/')
KREV = $(shell echo $(KVER) | \
sed -e 's/^[0-9][0-9]*\.[0-9][0-9]*\.\([0-9][0-9]*\).*/\1/')

kver_ge = $(shell \
echo test | awk '{if($(KMAJ) < $(1)) {print 0} else { \
	if($(KMAJ) > $(1)) {print 1} else { \
	if($(KMIN) < $(2)) {print 0} else { \
	if($(KMIN) > $(2)) {print 1} else { \
	if($(KREV) < $(3)) {print 0} else { print 1 } \
}}}}}' \
)

ifeq ($(call kver_ge,3,18,45),1)
$(info great or equal than 3.18.45)
ccflags-y += -DNEW_ABS
else
#$(info less than 3.18.45)
endif

# check linux kernel version if excess 4.11
ifeq ($(call kver_ge,4,11,0),1)
$(info great or equal than 4.11.0)
ccflags-y += -DNEW_CLOCK_HEADER
endif

# get current Makefile path
makefile_path := $(lastword $(MAKEFILE_LIST))
makefile_folder := $(dir $(makefile_path))

# Note. $(shell) can't handle # character. it will cause "*** unterminated call to function 'shell': missing ')'.  Stop." error
EN_605_IOCTRL_INTF := $(shell grep "define EN_605_IOCTRL_INTF" $(makefile_folder)/include/miniisp_customer_define.h | cut -d' ' -f 3)
#$(info $(makefile_folder)/include/miniisp_customer_define.h)
#$(info $(EN_605_IOCTRL_INTF))

# add obj to linux kernel
obj-y += basic_setting_cmd.o
obj-y += bulk_data_cmd.o
obj-y += camera_profile_cmd.o
obj-y += miniisp_intf_i2c.o
obj-y += miniisp_intf_spi.o
obj-y += miniisp_isp.o
obj-y += ispctrl_if_master.o
obj-y += miniisp_customer_define.o
obj-y += miniisp_top.o
obj-y += miniisp_ctrl.o
obj-y += operation_cmd.o
obj-y += sys_manage_cmd.o
obj-y += miniisp_utility.o
obj-y += miniisp_debug_if.o
obj-y += miniisp_chip_base_define.o

ifeq ($(EN_605_IOCTRL_INTF),1)
$(info add miniisp_ctrl_intf.o!!)
obj-y += miniisp_ctrl_intf.o
else
$(info not add miniisp_ctrl_intf.o!!)
endif
