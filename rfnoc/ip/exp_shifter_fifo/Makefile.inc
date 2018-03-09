#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_EXP_SHIFTER_FIFO_SRCS = $(IP_BUILD_DIR)/exp_shifter_fifo/exp_shifter_fifo.xci

LIB_IP_CHANNELIZER_EXP_SHIFTER_FIFO_OUTS = $(addprefix $(IP_BUILD_DIR)/exp_shifter_fifo/, \
    exp_shifter_fifo.xci.out \
    synth/exp_shifter_fifo.vhd \
)

$(LIB_IP_CHANNELIZER_EXP_SHIFTER_FIFO_SRCS) $(LIB_IP_CHANNELIZER_EXP_SHIFTER_FIFO_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/exp_shifter_fifo/exp_shifter_fifo.xci
	$(call BUILD_VIVADO_IP,exp_shifter_fifo,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
