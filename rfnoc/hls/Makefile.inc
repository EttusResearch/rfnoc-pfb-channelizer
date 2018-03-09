#
# Copyright 2018 Ettus Research
#

include $(TOOLS_DIR)/make/viv_hls_ip_builder.mak

# Set the IP Directory
PFB_CHANNELIZER_HLS_DIR := $(OOT_DIR)/hls

#
# Include your HLS designs here
#

hls_ip: $(HLS_IP_BUILD_TARGETS)

.PHONY: hls_ip
