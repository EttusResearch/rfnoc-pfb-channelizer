#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_PFB_TAPS_SRCS = $(IP_BUILD_DIR)/pfb_taps/pfb_taps.xci

LIB_IP_CHANNELIZER_PFB_TAPS_OUTS = $(addprefix $(IP_BUILD_DIR)/pfb_taps/, \
pfb_taps.xci.out \
synth/pfb_taps.vhd \
)

$(LIB_IP_CHANNELIZER_PFB_TAPS_SRCS) $(LIB_IP_CHANNELIZER_PFB_TAPS_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/pfb_taps/pfb_taps.xci
	$(call BUILD_VIVADO_IP,pfb_taps,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
