//
// Copyright 2016-2018 Ettus Research, A National Instruments Company
//
// SPDX-License-Identifier: LGPL-3.0
//
// Module: count_items_iw36_cw11
// Description:
// Simple module to count items used in count_cycle_iw36_cw11
module count_items_iw36_cw11
(
	input sync_reset,
	input clk,
	input valid_i,
	input reset_cnt,
	input [35:0] data_i,
	output valid_o,
	output [10:0] count_o,
	output [35:0] data_o
);

reg [10:0] cnt, next_cnt;
reg [10:0] cnt_d1, next_cnt_d1;

reg [35:0] out_d0, next_out_d0;
reg [35:0] out_d1, next_out_d1;

reg [1:0] init_cnt, next_init_cnt;
reg count_valid, next_count_valid;
reg first_flag, next_first_flag;
wire reset_cnt_s;

assign valid_o = count_valid;
assign data_o = out_d1;
assign count_o = cnt_d1;
assign reset_cnt_s = (first_flag == 1'b1 && valid_i == 1'b1)  ? 1'b1 : reset_cnt;

// do a reset
always @(posedge clk, posedge sync_reset)
begin
  if (sync_reset == 1'b1) begin
	  init_cnt <= 2'b01;
    count_valid <= 1'b0;
    first_flag <= 1'b1;
    out_d0 <= 0;
    out_d1 <= 0;
	  cnt <= 0;
	  cnt_d1 <= 0;
  end else begin
    out_d0 <= next_out_d0;
    out_d1 <= next_out_d1;
    init_cnt <= next_init_cnt;
    first_flag <= next_first_flag;
    count_valid <= next_count_valid;
	  cnt <= next_cnt;
	  cnt_d1 <= next_cnt_d1;
  end
end

always @*
begin
  next_out_d0 = out_d0;
  next_out_d1 = out_d1;
  next_init_cnt = init_cnt;
  next_count_valid = 1'b0;
  next_first_flag = first_flag;
	next_cnt = cnt;
	next_cnt_d1 = cnt_d1;
  if (valid_i == 1'b1) begin
    next_out_d0 = data_i;
    next_first_flag = 1'b0;
    next_out_d1 = out_d0;
	  next_cnt_d1 = cnt;
    if (init_cnt != 0) begin
      next_init_cnt = init_cnt - 1;
    end
	  if (reset_cnt_s == 1'b1) begin
		  next_cnt = 11'd0;
	  end else begin
		  next_cnt = cnt + 1;
	  end
    if (init_cnt == 0 && valid_i == 1'b1) begin
      next_count_valid = 1'b1;
	  end
	end
end

endmodule
