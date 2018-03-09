#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_EXP_AVERAGER_FILTER_SRCS = $(IP_BUILD_DIR)/exp_averager_filter/exp_averager_filter.xci

LIB_IP_CHANNELIZER_EXP_AVERAGER_FILTER_OUTS = $(addprefix $(IP_BUILD_DIR)/exp_averager_filter/, \
    exp_averager_filter.xci.out \
    synth/exp_averager_filter.vhd \
)

$(LIB_IP_CHANNELIZER_EXP_AVERAGER_FILTER_SRCS) $(LIB_IP_CHANNELIZER_EXP_AVERAGER_FILTER_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/exp_averager_filter/exp_averager_filter.xci
	$(call BUILD_VIVADO_IP,exp_averager_filter,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
