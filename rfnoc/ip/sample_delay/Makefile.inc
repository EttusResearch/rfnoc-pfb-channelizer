#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_SAMPLE_DELAY_SRCS = $(IP_BUILD_DIR)/sample_delay/sample_delay.xci

LIB_IP_CHANNELIZER_SAMPLE_DELAY_OUTS = $(addprefix $(IP_BUILD_DIR)/sample_delay/, \
    sample_delay.xci.out \
    synth/sample_delay.vhd \
)

$(LIB_IP_CHANNELIZER_SAMPLE_DELAY_SRCS) $(LIB_IP_CHANNELIZER_SAMPLE_DELAY_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/sample_delay/sample_delay.xci
	$(call BUILD_VIVADO_IP,sample_delay,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
