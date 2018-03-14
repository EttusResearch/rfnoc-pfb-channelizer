//
// Copyright 2016-2018 Ettus Research, A National Instruments Company
//
// SPDX-License-Identifier: LGPL-3.0
//
// Module: count_cycle_iw36_cw11
// Description:
// Implement simple count / data alignment logic while optimizing pipelining.
// Used inside the input_buffer module to appropriately segment sample blocks.

module count_cycle_iw36_cw11
(
  input sync_reset,
  input clk,

  input [35:0] data_i,
  input [10:0] high_cnt,
  input valid_i,

  output [35:0] data_out,
  output [10:0] count,
  output valid_out
);

reg reset_cnt, next_reset_cnt;
wire [10:0] count_value;
wire count_valid;
wire [35:0] count_data;

reg [10:0] reset_flag1, reset_flag2;

reg valid_d1;
reg [35:0] data_d1;

assign valid_out = count_valid;
assign data_out = count_data;
assign count = count_value;

always @(posedge clk, posedge sync_reset)
begin
	if (sync_reset == 1'b1) begin
    reset_cnt <= 1'b1;
	end else begin
    reset_cnt <= next_reset_cnt;
	end
end

// delay process
always @(posedge clk)
begin
  valid_d1 <= valid_i;
  data_d1 <= data_i;
  reset_flag1 <= high_cnt - 11'd1;
  reset_flag2 <= high_cnt - 11'd2;
end

// write process.
always @*
begin
  if (reset_cnt == 1'b1) begin
    next_reset_cnt = 1'b0;
  end else if (valid_i == 1'b1) begin
    if ((count_value == reset_flag1 && valid_d1 == 1'b0) || (count_value == reset_flag2 && valid_d1 == 1'b1)) begin
      next_reset_cnt = 1'b1;
    end else begin
      next_reset_cnt = 1'b0;
    end
  end else begin
    next_reset_cnt = 1'b0;
  end
end

count_items_iw36_cw11 count_items
(
	.sync_reset(sync_reset),
	.clk(clk),
	.valid_i(valid_d1),
	.reset_cnt(reset_cnt),
	.data_i(data_d1),
	.valid_o(count_valid),
	.count_o(count_value),
	.data_o(count_data)
);

endmodule
