#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_PFB_MAC_SRCS = $(IP_BUILD_DIR)/pfb_mac/pfb_mac.xci

LIB_IP_CHANNELIZER_PFB_MAC_OUTS = $(addprefix $(IP_BUILD_DIR)/pfb_mac/, \
    pfb_mac.xci.out \
    synth/pfb_mac.vhd \
)

$(LIB_IP_CHANNELIZER_PFB_MAC_SRCS) $(LIB_IP_CHANNELIZER_PFB_MAC_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/pfb_mac/pfb_mac.xci
	$(call BUILD_VIVADO_IP,pfb_mac,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
