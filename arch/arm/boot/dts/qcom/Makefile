
dtb-$(CONFIG_ARCH_SDXPOORWILLS) += sdxpoorwills-rumi.dtb \
	sdxpoorwills-cdp.dtb \
	sdxpoorwills-mtp.dtb \
	sdxpoorwills-cdp-256.dtb \
	sdxpoorwills-mtp-256.dtb \
	sdxpoorwills-pcie-ep-cdp.dtb \
	sdxpoorwills-pcie-ep-mtp.dtb

targets += dtbs

include $(srctree)/arch/arm64/boot/dts/qcom/Makefile
$(obj)/%.dtb: $(src)/../../../../arm64/boot/dts/qcom/%.dts FORCE
	$(call if_changed_dep,dtc)

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
$(obj)/%.dtbo:$(src)/../../../../arm64/boot/dts/qcom/%.dts FORCE
	$(call if_changed_dep,dtc)
	$(call if_changed,dtbo_verify)

dtbs: $(addprefix $(obj)/,$(dtb-y)) $(addprefix $(obj)/,$(dtbo-y))
else
dtbs: $(addprefix $(obj)/,$(dtb-y))
endif
clean-files := *.dtb
