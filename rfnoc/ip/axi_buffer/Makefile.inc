#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_AXI_BUFFER_SRCS = $(IP_BUILD_DIR)/axi_buffer/axi_buffer.xci

LIB_IP_CHANNELIZER_AXI_BUFFER_OUTS = $(addprefix $(IP_BUILD_DIR)/axi_buffer/, \
    axi_buffer.xci.out \
    synth/axi_buffer.vhd \
)

$(LIB_IP_CHANNELIZER_AXI_BUFFER_SRCS) $(LIB_IP_CHANNELIZER_AXI_BUFFER_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/axi_buffer/axi_buffer.xci
	$(call BUILD_VIVADO_IP,axi_buffer,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
