##################################################
# RFNoC Sources
##################################################

RFNOC_SRCS += $(addprefix $(RFNOC_PFB_CHANNELIZER_DIR)/fpga-src/, \
count_cycle_iw36_cw11.v \
count_items_iw36_cw11.v \
input_buffer.v \
pfb_2x.v \
circ_buffer.v \
exp_shifter.v \
channelizer_top.v \
noc_block_channelizer.v \
)
