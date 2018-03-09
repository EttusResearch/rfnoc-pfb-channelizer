#
# Copyright 2016 Ettus Research
#

#-------------------------------------------------
# Top-of-Makefile
#-------------------------------------------------
# Define BASE_DIR to point to the "top" dir
BASE_DIR = $(FPGA_TOP_DIR)/usrp3/top
# Include viv_sim_preample after defining BASE_DIR
include $(BASE_DIR)/../tools/make/viv_sim_preamble.mak

ARCH = zynq
PART_ID = xc7z020/clg484/-3

#-------------------------------------------------
# IP Specific
#-------------------------------------------------
# If simulation contains IP, define the IP_DIR and point
# it to the base level IP directory
RFNOC_PFB_CHANNELIZER_DIR = $(abspath ../../)

$(info RFNOC_PFB_CHANNELIZER_DIR="$(RFNOC_PFB_CHANNELIZER_DIR)")
# Include makefiles and sources for all IP components
# *after* defining the RFNOC_PFB_CHANNELIZER_DIR
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/axi_buffer/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/circ_buff_ram/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/exp_shifter_fifo/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/input_buff_RAM/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/pfb_mac/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/pfb_mac_0/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/pfb_taps/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/phase_offset/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/hb_fil/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/sample_delay/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/sample_ram/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/xfft_stream_var/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/exp_averager_filter/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/exp_corr_rom/Makefile.inc
include $(RFNOC_PFB_CHANNELIZER_DIR)/ip/exp_corr_mult/Makefile.inc

DESIGN_SRCS += $(abspath \
$(LIB_IP_CHANNELIZER_AXI_BUFFER_SRCS) \
$(LIB_IP_CHANNELIZER_CIRC_BUFF_RAM_SRCS) \
$(LIB_IP_CHANNELIZER_EXP_SHIFTER_FIFO_SRCS) \
$(LIB_IP_CHANNELIZER_HB_FIL_SRCS) \
$(LIB_IP_CHANNELIZER_INPUT_BUFF_RAM_SRCS) \
$(LIB_IP_CHANNELIZER_PFB_MAC_SRCS) \
$(LIB_IP_CHANNELIZER_PFB_MAC_0_SRCS) \
$(LIB_IP_CHANNELIZER_PFB_TAPS_SRCS) \
$(LIB_IP_CHANNELIZER_PHASE_OFFSET_SRCS) \
$(LIB_IP_CHANNELIZER_SAMPLE_DELAY_SRCS) \
$(LIB_IP_CHANNELIZER_SAMPLE_RAM_SRCS) \
$(LIB_IP_CHANNELIZER_XFFT_STREAM_VAR_SRCS) \
$(LIB_IP_CHANNELIZER_EXP_AVERAGER_FILTER_SRCS) \
$(LIB_IP_CHANNELIZER_EXP_CORR_MULT_SRCS) \
$(LIB_IP_CHANNELIZER_EXP_CORR_ROM_SRCS))

#-------------------------------------------------
# Testbench Specific
#-------------------------------------------------
# Define only one toplevel module
SIM_TOP = noc_block_channelizer_tb

# Add test bench, user design under test, and
# additional user created files
SIM_SRCS = \
$(abspath noc_block_channelizer_tb.sv) \
$(abspath ../../fpga-src/channelizer_top.v) \
$(abspath ../../fpga-src/circ_buffer.v) \
$(abspath ../../fpga-src/count_cycle_iw36_cw11.v) \
$(abspath ../../fpga-src/count_items_iw36_cw11.v) \
$(abspath ../../fpga-src/exp_shifter.v) \
$(abspath ../../fpga-src/input_buffer.v) \
$(abspath ../../fpga-src/pfb_2x.v) \
$(abspath ../../fpga-src/noc_block_channelizer.v)

# MODELSIM_USER_DO = $(abspath wave.do)

#-------------------------------------------------
# Bottom-of-Makefile
#-------------------------------------------------
# Include all simulator specific makefiles here
# Each should define a unique target to simulate
# e.g. xsim, vsim, etc and a common "clean" target
include $(BASE_DIR)/../tools/make/viv_simulator.mak
