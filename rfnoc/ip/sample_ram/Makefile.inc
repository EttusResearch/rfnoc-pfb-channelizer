#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_SAMPLE_RAM_SRCS = $(IP_BUILD_DIR)/sample_ram/sample_ram.xci

LIB_IP_CHANNELIZER_SAMPLE_RAM_OUTS = $(addprefix $(IP_BUILD_DIR)/sample_ram/, \
    sample_ram.xci.out \
    synth/sample_ram.vhd \
)

$(LIB_IP_CHANNELIZER_SAMPLE_RAM_SRCS) $(LIB_IP_CHANNELIZER_SAMPLE_RAM_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/sample_ram/sample_ram.xci
	$(call BUILD_VIVADO_IP,sample_ram,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
