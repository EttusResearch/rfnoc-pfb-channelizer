#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_ip_builder.mak

LIB_IP_CHANNELIZER_XFFT_STREAM_VAR_SRCS = $(IP_BUILD_DIR)/xfft_stream_var/xfft_stream_var.xci

LIB_IP_CHANNELIZER_XFFT_STREAM_VAR_OUTS = $(addprefix $(IP_BUILD_DIR)/xfft_stream_var/, \
    xfft_stream_var.xci.out \
    synth/xfft_stream_var.vhd \
)

$(LIB_IP_CHANNELIZER_XFFT_STREAM_VAR_SRCS) $(LIB_IP_CHANNELIZER_XFFT_STREAM_VAR_OUTS) : $(RFNOC_PFB_CHANNELIZER_DIR)/ip/xfft_stream_var/xfft_stream_var.xci
	$(call BUILD_VIVADO_IP,xfft_stream_var,$(ARCH),$(PART_ID),$(RFNOC_PFB_CHANNELIZER_DIR)/ip,$(IP_BUILD_DIR),0)
