#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_CIRC_BUFF_RAM_SRCS = $(IP_BUILD_DIR)/circ_buff_ram/circ_buff_ram.xci

LIB_IP_CHANNELIZER_CIRC_BUFF_RAM_OUTS = $(addprefix $(IP_BUILD_DIR)/circ_buff_ram/, \
    circ_buff_ram.xci.out \
    synth/circ_buff_ram.vhd \
)

$(LIB_IP_CHANNELIZER_CIRC_BUFF_RAM_SRCS) $(LIB_IP_CHANNELIZER_CIRC_BUFF_RAM_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/circ_buff_ram/circ_buff_ram.xci
	$(call BUILD_VIVADO_IP,circ_buff_ram,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
