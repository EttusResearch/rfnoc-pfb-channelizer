//
// Copyright 2016-2018 Ettus Research, A National Instruments Company
//
// SPDX-License-Identifier: LGPL-3.0
//
// Module: exp_shifter
// Description:
//
// Since the fft is block floating point the apparent signal amplitude can be shifted
// in consecutive fft blocks.  The Exponent shifter, exp_shifter, implements a simple
// low pass filtering the shift signal and provides gain correction mechanism for mitigating
// the amplitude shifts caused by the fft module.  The module also provides buffering
// and flow control logic so that it can directly connected to the rest of the rfnoc
// infrastructure.

module exp_shifter
(
  // input [4:0] min_shift_val,
  input clk,
  input sync_reset,

  input [31:0] s_axis_data_tdata,
  input [23:0] s_axis_data_tuser,
  input s_axis_data_tvalid,
  output s_axis_data_tready,
  input [9:0] fft_size,

  input s_axis_data_tlast,
  input [7:0] s_axis_status_tdata,
  input s_axis_status_tvalid,
  output s_axis_status_tready,

  output [31:0] m_axis_tdata,
  output [23:0] m_axis_tuser,
  output m_axis_tvalid,
  output eob_tag,
  output m_axis_tlast,
  input m_axis_tready  // ready out
);

// status register BLK_EXP[4:0]
reg signed [5:0] sub_out;
reg signed [5:0] next_sub_out;
wire filter_tready;
wire [15:0] filter_tdata;

reg [23:0] tuser_d[0:6];
reg [31:0] tdata_d[0:2];
reg [6:0] tlast_d;
reg [6:0] tvalid_d;

reg [6:0] shift_val;
reg [15:0] i_val, q_val, next_i_val, next_q_val;

reg [9:0] fft_m1, fft_m2;
wire [8:0] mask;

wire filter_tvalid;
reg [15:0] filter_tdata_d0;
wire [10:0] axis_data_count;  // output wire [10 : 0] axis_data_count

reg [15:0] next_filter_tdata_d0;
reg [15:0] filter_ref, next_filter_ref;

reg [7:0] samp_cnt, next_samp_cnt;
reg [7:0] samp_cnt_out, next_samp_cnt_out;

wire [31:0] p_ival, p_qval;
wire [15:0] rom_val;

reg chan_0, next_chan_0;

wire [31:0] m_axis_tdata_s;
wire [23:0] m_axis_tuser_s;
wire m_axis_tvalid_s;
wire m_axis_tlast_s;
wire m_axis_tready_s;
wire take_data;
wire s_axis_data_tready_s;

wire fifo_tready;
wire axis_prog_full;

// channels are stored in 12:8 vector of the tuser input
assign s_axis_data_tready_s = !axis_prog_full & filter_tready;
assign s_axis_data_tready = s_axis_data_tready_s;
assign s_axis_status_tready = !axis_prog_full  & filter_tready;

// indicates a valid input transaction has occurred.
assign take_data = s_axis_data_tvalid & s_axis_data_tready_s;

assign m_axis_tdata = m_axis_tdata_s;
assign m_axis_tuser = m_axis_tuser_s;
assign m_axis_tvalid = m_axis_tvalid_s;
assign m_axis_tlast = m_axis_tlast_s;
assign m_axis_tready_s = m_axis_tready;
assign mask = fft_m1[7:0] & samp_cnt;
assign eob_tag = m_axis_tuser_s[23];

// Latency = 3.  This rom provides the correction term to smooth out the gain changes caused
// the shifting of the output of the FFT module.
exp_corr_rom corr_rom (
  .clka(clk),
  .addra(filter_ref[5:0]),
  .douta(rom_val)
);

// smoothing filter.  Used to effectively interpolate the shifts of the FFT module.
exp_averager_filter avg_filter (
  .aclk(clk),
  .s_axis_data_tvalid(s_axis_status_tvalid & s_axis_status_tready),
  .s_axis_data_tready(filter_tready),
  .s_axis_data_tdata({3'd0, s_axis_status_tdata[4:0]}),
  .m_axis_data_tvalid(filter_tvalid),
  .m_axis_data_tdata(filter_tdata)
);

// correction multipliers.
// Latency = 3
exp_corr_mult exp_corr_mult_I (
  .CLK(clk),
  .A(i_val),
  .B(rom_val),
  .P(p_ival)
);

// Latency = 3
exp_corr_mult exp_corr_mult_Q (
  .CLK(clk),
  .A(q_val),
  .B(rom_val),
  .P(p_qval)
);

// fifo is used primarily for output flow control.
exp_shifter_fifo exp_shifter_fifo (
  .s_aclk(clk),
  .s_aresetn(!sync_reset),
  .s_axis_tvalid(tvalid_d[6]),
  .s_axis_tready(fifo_tready),
  .s_axis_tdata({p_qval[30:15], p_ival[30:15]}),
  .s_axis_tlast(tlast_d[6]),
  .s_axis_tuser(tuser_d[6]),
  .m_axis_tvalid(m_axis_tvalid_s),
  .m_axis_tready(m_axis_tready_s),
  .m_axis_tdata(m_axis_tdata_s),
  .m_axis_tlast(m_axis_tlast_s),
  .m_axis_tuser(m_axis_tuser_s),
  .axis_data_count(axis_data_count),
  .axis_prog_full(axis_prog_full)
);

// reset / clock process.
always @(posedge clk, posedge sync_reset)
begin
	if (sync_reset == 1'b1) begin
    chan_0 <= 1'b0;
    filter_tdata_d0 <= 0;
    filter_ref <= 0;
    sub_out <= 0;
    samp_cnt <= 8'd255;
    samp_cnt_out <= 8'd255;
	end else begin
    chan_0 <= next_chan_0;
    filter_tdata_d0 <= next_filter_tdata_d0;
    filter_ref <= next_filter_ref;
    sub_out <= next_sub_out;
    samp_cnt <= next_samp_cnt;
    samp_cnt_out <= next_samp_cnt_out;
	end
end


// delay process / pipelining
integer i;
always @(posedge clk)
begin
  fft_m2 <= fft_size - 2;
  fft_m1 <= fft_size - 1;
  tvalid_d[0] <= take_data;
  for (i = 1; i < 7; i = i + 1) begin
    tvalid_d[i] <= tvalid_d[i-1];
  end
  tuser_d[0] <= {s_axis_data_tlast, s_axis_data_tuser[22:0]};
  for (i = 1; i < 7; i = i + 1) begin
    tuser_d[i] <= tuser_d[i-1];
  end
  tlast_d[0] <= s_axis_data_tlast;
  if (samp_cnt == 8'd255 && tvalid_d[0] == 1'b1) begin
    tlast_d[1] <= 1'b1;
  end else begin
    tlast_d[1] <= 1'b0;
  end
  for (i = 2; i < 7; i = i + 1) begin
    tlast_d[i] <= tlast_d[i-1];
  end
  tdata_d[0] <= s_axis_data_tdata;
  for (i=1; i < 3; i = i + 1) begin
    tdata_d[i] <= tdata_d[i-1];
  end
  i_val <= next_i_val;
  q_val <= next_q_val;
  shift_val <= sub_out - $signed(6'd3);
end

// tready logic.
always @*
begin
  next_chan_0 = chan_0;
  next_samp_cnt = samp_cnt;
  next_samp_cnt_out = samp_cnt_out;
  if (s_axis_data_tuser[8:0] == 9'd0 && take_data == 1'b1) begin
    next_chan_0 <= 1'b1;
  end else begin
    next_chan_0 <= 1'b0;
  end

  next_filter_tdata_d0 = filter_tdata_d0;
  if (filter_tvalid == 1'b1) begin
    next_filter_tdata_d0 = filter_tdata;
  end

  next_filter_ref = filter_ref;
  if (s_axis_data_tuser[8:0] == 9'd0 && take_data == 1'b1) begin
    next_filter_ref = filter_tdata_d0;
  end

  next_sub_out = sub_out;
  if (chan_0 == 1'b1) begin
    next_sub_out = $signed(tuser_d[0][20:16]) - $signed(filter_ref[10:6]);
  end

  // logic was used as internal debugging signals.
  if (take_data) begin
    // recovery logic.
    if (samp_cnt == 8'd255) begin
      next_samp_cnt = 0;
    // got off cut.
    end else if (mask != fft_m2[7:0] && s_axis_data_tlast == 1'b1) begin
      next_samp_cnt = 8'd255;
    end else begin
      next_samp_cnt = samp_cnt + 1;
    end
  end

  if (m_axis_tvalid_s == 1'b1 && m_axis_tready_s == 1'b1) begin
    if (m_axis_tlast_s == 1'b1) begin
      next_samp_cnt_out = 0;
    end else begin
      next_samp_cnt_out = samp_cnt_out + 1;
    end
  end
end

// output slicing.  Takes the filtered shift value and appropriately grabs the correct slice
// of the output signal.  Implemented as a bounded case statement.
always @*
begin
  case (shift_val)
    7'd0 :
    begin
      next_i_val = tdata_d[2][15:0];
      next_q_val = tdata_d[2][31:16];
    end
    7'd1:
    begin
      next_i_val = {tdata_d[2][14:0], 1'b0};
      next_q_val = {tdata_d[2][30:16], 1'b0};
    end
    7'd2:
    begin
      next_i_val = {tdata_d[2][13:0], 2'd0};
      next_q_val = {tdata_d[2][29:16], 2'd0};
    end
    7'd3:
    begin
      next_i_val = {tdata_d[2][12:0], 3'd0};
      next_q_val = {tdata_d[2][28:16], 3'd0};
    end
    7'd4:
    begin
      next_i_val = {tdata_d[2][11:0], 4'd0};
      next_q_val = {tdata_d[2][27:16], 4'd0};
    end
    7'd5:
    begin
      next_i_val = {tdata_d[2][10:0], 5'd0};
      next_q_val = {tdata_d[2][26:16], 5'd0};
    end
    7'd6:
    begin
      next_i_val = {tdata_d[2][9:0], 6'd0};
      next_q_val = {tdata_d[2][25:16], 6'd0};
    end
    7'd7:
    begin
      next_i_val = {tdata_d[2][9:0], 7'd0};
      next_q_val = {tdata_d[2][24:16], 7'd0};
    end
    // Maximum scaling gain = 2^7

    -7'd1:
    begin
      next_i_val = {{1{tdata_d[2][15]}}, tdata_d[2][15:1]};
      next_q_val = {{1{tdata_d[2][31]}}, tdata_d[2][31:17]};
    end
    -7'd2:
    begin
      next_i_val = {{2{tdata_d[2][15]}}, tdata_d[2][15:2]};
      next_q_val = {{2{tdata_d[2][31]}}, tdata_d[2][31:18]};
    end
    -7'd3:
    begin
      next_i_val = {{3{tdata_d[2][15]}}, tdata_d[2][15:3]};
      next_q_val = {{3{tdata_d[2][31]}}, tdata_d[2][31:19]};
    end
    -7'd4:
    begin
      next_i_val = {{4{tdata_d[2][15]}}, tdata_d[2][15:4]};
      next_q_val = {{4{tdata_d[2][31]}}, tdata_d[2][31:20]};
    end
    -7'd5:
    begin
      next_i_val = {{5{tdata_d[2][15]}}, tdata_d[2][15:5]};
      next_q_val = {{5{tdata_d[2][31]}}, tdata_d[2][31:21]};
    end
    -7'd6:
    begin
      next_i_val = {{6{tdata_d[2][15]}}, tdata_d[2][15:6]};
      next_q_val = {{6{tdata_d[2][31]}}, tdata_d[2][31:22]};
    end
    -7'd7:
    begin
      next_i_val = {{7{tdata_d[2][15]}}, tdata_d[2][15:7]};
      next_q_val = {{7{tdata_d[2][31]}}, tdata_d[2][31:23]};
    end
    -7'd8:
    begin
      next_i_val = {{8{tdata_d[2][15]}}, tdata_d[2][15:8]};
      next_q_val = {{8{tdata_d[2][31]}}, tdata_d[2][31:24]};
    end
    -7'd9:
    begin
      next_i_val = {{9{tdata_d[2][15]}}, tdata_d[2][15:9]};
      next_q_val = {{9{tdata_d[2][31]}}, tdata_d[2][31:25]};
    end
    -7'd10:
    begin
      next_i_val = {{10{tdata_d[2][15]}}, tdata_d[2][15:10]};
      next_q_val = {{10{tdata_d[2][31]}}, tdata_d[2][31:26]};
    end

    default :
    begin
      if (shift_val[6] == 1'b0) begin
        next_i_val = {tdata_d[2][9:0], 7'd0};
        next_q_val = {tdata_d[2][24:16], 7'd0};
      end else begin
        next_i_val = {{10{tdata_d[2][15]}}, tdata_d[2][15:10]};
        next_q_val = {{10{tdata_d[2][31]}}, tdata_d[2][31:26]};
      end
    end
  endcase
end

endmodule
