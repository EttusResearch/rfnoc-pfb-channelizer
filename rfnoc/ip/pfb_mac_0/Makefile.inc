#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_PFB_MAC_0_SRCS = $(IP_BUILD_DIR)/pfb_mac_0/pfb_mac_0.xci

LIB_IP_CHANNELIZER_PFB_MAC_0_OUTS = $(addprefix $(IP_BUILD_DIR)/pfb_mac_0/, \
    pfb_mac_0.xci.out \
    synth/pfb_mac_0.vhd \
)

$(LIB_IP_CHANNELIZER_PFB_MAC_0_SRCS) $(LIB_IP_CHANNELIZER_PFB_MAC_0_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/pfb_mac_0/pfb_mac_0.xci
	$(call BUILD_VIVADO_IP,pfb_mac_0,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
