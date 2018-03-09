#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_EXP_CORR_ROM_SRCS = $(IP_BUILD_DIR)/exp_corr_rom/exp_corr_rom.xci

LIB_IP_CHANNELIZER_EXP_CORR_ROM_OUTS = $(addprefix $(IP_BUILD_DIR)/exp_corr_rom/, \
    exp_corr_rom.xci.out \
    synth/exp_corr_rom.vhd \
)

$(LIB_IP_CHANNELIZER_EXP_CORR_ROM_SRCS) $(LIB_IP_CHANNELIZER_EXP_CORR_ROM_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/exp_corr_rom/exp_corr_rom.xci
	$(call BUILD_VIVADO_IP,exp_corr_rom,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
