#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_HB_FIL_SRCS = $(IP_BUILD_DIR)/hb_fil/hb_fil.xci

LIB_IP_CHANNELIZER_HB_FIL_OUTS = $(addprefix $(IP_BUILD_DIR)/hb_fil/, \
    hb_fil.xci.out \
    synth/hb_fil.vhd \
)

$(LIB_IP_CHANNELIZER_HB_FIL_SRCS) $(LIB_IP_CHANNELIZER_HB_FIL_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/hb_fil/hb_fil.xci
	$(call BUILD_VIVADO_IP,hb_fil,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
