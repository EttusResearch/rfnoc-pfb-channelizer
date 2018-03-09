#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_EXP_CORR_MULT_SRCS = $(IP_BUILD_DIR)/exp_corr_mult/exp_corr_mult.xci

LIB_IP_CHANNELIZER_EXP_CORR_MULT_OUTS = $(addprefix $(IP_BUILD_DIR)/exp_corr_mult/, \
    exp_corr_mult.xci.out \
    synth/exp_corr_mult.vhd \
)

$(LIB_IP_CHANNELIZER_EXP_CORR_MULT_SRCS) $(LIB_IP_CHANNELIZER_EXP_CORR_MULT_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/exp_corr_mult/exp_corr_mult.xci
	$(call BUILD_VIVADO_IP,exp_corr_mult,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
