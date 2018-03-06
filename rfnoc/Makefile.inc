# Specify all relevant Makefiles to include

RFNOC_PFB_CHANNELIZER_DIR := $(OOT_DIR)
include $(abspath $(RFNOC_PFB_CHANNELIZER_DIR)/fpga-src/Makefile.inc)
include $(abspath $(RFNOC_PFB_CHANNELIZER_DIR)/hls/Makefile.inc)
include $(abspath $(RFNOC_PFB_CHANNELIZER_DIR)/ip/Makefile.inc)
