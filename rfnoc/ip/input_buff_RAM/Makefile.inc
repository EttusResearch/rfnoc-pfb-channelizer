#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_INPUT_BUFF_RAM_SRCS = $(IP_BUILD_DIR)/input_buff_RAM/input_buff_RAM.xci

LIB_IP_CHANNELIZER_INPUT_BUFF_RAM_OUTS = $(addprefix $(IP_BUILD_DIR)/input_buff_RAM/, \
    input_buff_RAM.xci.out \
    synth/input_buff_RAM.vhd \
)

$(LIB_IP_CHANNELIZER_INPUT_BUFF_RAM_SRCS) $(LIB_IP_CHANNELIZER_INPUT_BUFF_RAM_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/input_buff_RAM/input_buff_RAM.xci
	$(call BUILD_VIVADO_IP,input_buff_RAM,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
