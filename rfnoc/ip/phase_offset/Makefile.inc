#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_PHASE_OFFSET_SRCS = $(IP_BUILD_DIR)/phase_offset/phase_offset.xci

LIB_IP_CHANNELIZER_PHASE_OFFSET_OUTS = $(addprefix $(IP_BUILD_DIR)/phase_offset/, \
    phase_offset.xci.out \
    synth/phase_offset.vhd \
)

$(LIB_IP_CHANNELIZER_PHASE_OFFSET_SRCS) $(LIB_IP_CHANNELIZER_PHASE_OFFSET_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/phase_offset/phase_offset.xci
	$(call BUILD_VIVADO_IP,phase_offset,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
