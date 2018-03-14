//
// Copyright 2016-2018 Ettus Research, A National Instruments Company
//
// SPDX-License-Identifier: LGPL-3.0
//
// Module: channelizer_top
// Description:
//
// Implements the M/2 Channelizer as specified in the
// "A Versatile Multichannel Filter Bank with Multiple Channel Bandwidths" paper.

module channelizer_top
(

  input sync_reset,
  input ce_clk,

  input [31:0] s_axis_reload_tdata,
  input s_axis_reload_tvalid,
  input s_axis_reload_tlast,
  output s_axis_reload_tready,

  // currently only supporting up to 512 bins.
  input [9:0] fft_size,

  input [31:0] s_axis_tdata,
  input s_axis_tvalid,
  output s_axis_tready,
  output eob_tag,

  output [15:0] m_axis_tuser,
  output [31:0] m_axis_tdata,
  output m_axis_tvalid,
  output m_axis_tlast,
  input m_axis_tready
);


wire [35:0] buffer_sig;
wire [35:0] pfb_sig;
wire [8:0] buffer_phase;
wire [8:0] pfb_phase;
wire buffer_valid, pfb_valid;

// real component is most significant word except for fft.
//****************************** infrastructure signals **********************//
reg async_reset, async_reset_d1;
wire [35:0] tdata_high_pad;
//****************************************************************************//

reg fft_config_tvalid;
reg next_fft_config_tvalid;
wire fft_tready;
wire fft_config_tready;

// circ buffer signals
wire [35:0] circ_buff_tdata;
wire circ_buff_tvalid;
wire circ_buff_tlast;
wire [31:0] circ_buff_trunc;

// axi buffer signals
wire axi_buffer_tvalid;
wire [31:0] axi_buffer_tdata;
wire axi_buffer_tlast;

wire hb_tvalid;
wire hb_tready;
wire hb_m_tready;
wire [95:0] hb_tdata;

wire axis_prog_full;

wire event_frame_started;
wire event_tlast_unexpected;
wire event_tlast_missing;
wire event_status_channel_halt;
wire event_data_in_channel_halt;
wire event_data_out_channel_halt;

wire in_buff_tready;

localparam S_CONFIG=0, S_IDLE=1;
reg config_state = S_CONFIG;
reg next_config_state = S_CONFIG;

wire [15:0] config_sig;
reg [4:0] nfft, next_nfft;

reg [9:0] fft_size_s;
reg [4:0] reset_cnt, next_reset_cnt;
reg reset_int, next_reset_int;

// channel value m_axis_fft_data_tuser[7:0]
// block exponent m_axis_fft_status_tuser[4:0]
assign tdata_high_pad = {hb_tdata[88:71], hb_tdata[40:23]};
assign hb_tready = in_buff_tready & !axis_prog_full;

assign circ_buff_trunc = {circ_buff_tdata[17:2], circ_buff_tdata[35:20]};

//---------- FFT signals  ----------------------------
// [20:16] blk_exp, [8:0] xk_index
wire [23:0] m_axis_fft_data_tuser;
wire m_axis_fft_data_tvalid;
wire [31:0] m_axis_fft_data_tdata;
wire m_axis_fft_data_tready;
wire m_axis_fft_data_tlast;

wire [7:0] m_axis_fft_status_tdata;
wire m_axis_fft_status_tvalid;
wire m_axis_fft_status_tready;

wire [11:0] axis_data_count;
//----------------------------------------------
//---------- Shift Signals ---------------------
wire [31:0] m_axis_shift_tdata;
wire [15:0] m_axis_shift_tuser;
wire m_axis_shift_tvalid;
wire m_axis_tlast;
wire m_axis_shift_tready;

assign config_sig = {11'd0, nfft};
assign s_axis_tready = hb_m_tready;  // !axis_prog_full & ;  // hb_m_tready & ;

//-----------------------------------------------
always @(posedge ce_clk)
begin
  async_reset <= !(sync_reset | reset_int);
  async_reset_d1 <= async_reset;
end

// ensures that reset pulse is wide enough for all blocks.
always @*
begin
  next_reset_cnt = reset_cnt;
  if (fft_size_s != fft_size) begin
    next_reset_cnt = 5'd04;
  end else if (reset_cnt != 0) begin
    next_reset_cnt = reset_cnt - 1;
  end
  if (reset_cnt != 0) begin
    next_reset_int = 1'b1;
  end else begin
    next_reset_int = 1'b0;
  end
end

// config process -- responsible for configuring the fft size the xfft_stream_var.  Note that the
// upper limit is currently 512.  This can be changed, but all sample and coefficient memories must be
// adjusted accordingly.
always @*
begin
  next_fft_config_tvalid = fft_config_tvalid;
  next_config_state = config_state;
  next_nfft = nfft;
  case (config_state)
    S_CONFIG:
    begin
      next_fft_config_tvalid = 1'b1;
      if (fft_config_tready == 1'b1 && fft_config_tvalid == 1'b1) begin
        next_config_state = S_IDLE;
      end
      if (fft_size == 10'd08) begin
        next_nfft = 5'b00011;
      end else if (fft_size == 10'd16) begin
        next_nfft = 5'b00100;
      end else if (fft_size == 10'd32) begin
        next_nfft = 5'b00101;
      end else if (fft_size == 10'd64) begin
        next_nfft = 5'b00110;
      end else if (fft_size == 10'd128) begin
        next_nfft = 5'b00111;
      end else if (fft_size == 10'd256) begin
        next_nfft = 5'b01000;
      end else if (fft_size == 10'd512) begin
        next_nfft = 5'b01001;
      end else begin
        next_nfft = 5'b01000;
      end

    end
    S_IDLE:
    begin
      if (async_reset == 1'b1 && async_reset_d1 == 1'b0) begin
        next_config_state = S_CONFIG;
      end else begin
        next_config_state = S_IDLE;
      end
      next_fft_config_tvalid = 1'b0;
    end
  endcase
end

// standard reset and clock process.
always @(posedge ce_clk, posedge sync_reset)
begin
	if (sync_reset == 1'b1) begin
    config_state <= S_IDLE;
    fft_config_tvalid <= 1'b0;
    nfft <= 5'b00111;  // default to 128
    fft_size_s <= 10'd128;
    reset_cnt <= 5'd31;
    reset_int <= 1'b1;
	end else begin
    config_state <= next_config_state;
    fft_config_tvalid <= next_fft_config_tvalid;
    nfft <= next_nfft;
    fft_size_s <= fft_size;
    reset_cnt <= next_reset_cnt;
    reset_int <= next_reset_int;
	end
end

// input half-band filter divides the streaming datarate by a factor of 2.  Coefficients are fixed,
// could be made to be configurable.
hb_fil hb_fil (
  .aclk(ce_clk),                              // input wire aclk
  .s_axis_data_tvalid(s_axis_tvalid),  // input wire s_axis_data_tvalid
  .s_axis_data_tready(hb_m_tready),  // output wire s_axis_data_tready
  .s_axis_data_tdata(s_axis_tdata),    // input wire [31 : 0] s_axis_data_tdata
  .m_axis_data_tvalid(hb_tvalid),  // output wire m_axis_data_tvalid
  .m_axis_data_tready(hb_tready),
  .m_axis_data_tdata(hb_tdata)    // output wire [95 : 0] m_axis_data_tdata
);

// The input buffer implemented in logic performs two functions. First, it provides the addressing
// mechanism so that input samples are written to two memory locations simultaneously.
// Second, it provides a ping-pong buffer interface so that input throttling is mitigated.
input_buffer input_buffer
(
  .sync_reset(reset_int),
  .clk(ce_clk),

  .s_tdata(tdata_high_pad),
  .s_tvalid(hb_tvalid & !axis_prog_full),  //tvalid_high),
  .s_tready(in_buff_tready),

  .fft_size(fft_size),

  .output_sig(buffer_sig),
  .phase(buffer_phase),
  .valid_out(buffer_valid) //output valid signal.
);


// Implements the M/2 PFB architecture referenced in the
// "A Versatile Multichannel Filter Bank with Multiple Channel Bandwidths" paper.
// This architecture has been mapped to the Xilinx architecture.
// This represents a fully pipelined design that maximizes the FMax potential of the design. It is important
// to understand that filter arms are loaded sequentially. This is referenced in the diagram by the
// incremental changes in the phase subscript through each subsequent delay register.
// The nth index is only updated once per revolution of the filter bank.
pfb_2x pfb_2x
(
  .sync_reset(reset_int),
  .clk(ce_clk),

  .phase(buffer_phase),
  .fft_size(fft_size),
  .input_sig(buffer_sig),
  .valid_i(buffer_valid),

  .s_axis_reload_tdata(s_axis_reload_tdata),
  .s_axis_reload_tlast(s_axis_reload_tlast),
  .s_axis_reload_tvalid(s_axis_reload_tvalid),
  .s_axis_reload_tready(s_axis_reload_tready),

  .output_sig(pfb_sig),
  .phase_out(pfb_phase),
  .valid_out(pfb_valid) //output valid signal.
);

// The circular buffer is similar to the input buffer
// in that the block is able to ping-pong between different memories.
// This mitigates the requirement to throttle the input data stream.
circ_buffer circ_buffer
(
   .sync_reset(reset_int),
   .clk(ce_clk),

   .phase(pfb_phase),
   .input_sig(pfb_sig),
   .valid_i(pfb_valid),

   .fft_size(fft_size),

   .m_axis_tdata(circ_buff_tdata),
   .m_axis_tvalid(circ_buff_tvalid),
   .m_axis_tlast(circ_buff_tlast)
);

// The primary purpose to this buffer is to provide a throttling condition.  The programmed full flag allows
// the input buffer to finish outputting the current block of sample before entering the throttling state.
// This greatly simplified the throttling logic contained in the input_buffer module.
axi_buffer axi_buffer (
  .s_aclk(ce_clk),
  .s_aresetn(async_reset),
  .s_axis_tvalid(circ_buff_tvalid),
  .s_axis_tready(),
  .s_axis_tdata(circ_buff_trunc),
  .s_axis_tlast(circ_buff_tlast),
  .m_axis_tvalid(axi_buffer_tvalid),
  .m_axis_tready(fft_tready),
  .m_axis_tdata(axi_buffer_tdata),
  .m_axis_tlast(axi_buffer_tlast),
  .axis_data_count(axis_data_count),
  .axis_prog_full(axis_prog_full)
);


// Implements the configurable IFFT block of the design.
// FFT uses opposite I/Q ordering on tdata bus.
xfft_stream_var xfft_stream_var (
  .aclk(ce_clk),
  .aresetn(async_reset),
  .s_axis_config_tdata(config_sig),  // inverse transform - page 45 Fast Fourier Transform v9.0
  .s_axis_config_tvalid(fft_config_tvalid),
  .s_axis_config_tready(fft_config_tready),
  .s_axis_data_tdata(axi_buffer_tdata),
  .s_axis_data_tvalid(axi_buffer_tvalid),
  .s_axis_data_tready(fft_tready),
  .s_axis_data_tlast(axi_buffer_tlast),
  .m_axis_data_tdata(m_axis_fft_data_tdata),
  .m_axis_data_tuser(m_axis_fft_data_tuser),
  .m_axis_data_tvalid(m_axis_fft_data_tvalid),
  .m_axis_data_tready(m_axis_fft_data_tready),
  .m_axis_data_tlast(m_axis_fft_data_tlast),
  .m_axis_status_tdata(m_axis_fft_status_tdata),
  .m_axis_status_tvalid(m_axis_fft_status_tvalid),
  .m_axis_status_tready(m_axis_fft_status_tready),
  .event_frame_started(event_frame_started),
  .event_tlast_unexpected(event_tlast_unexpected),
  .event_tlast_missing(event_tlast_missing),
  .event_status_channel_halt(event_status_channel_halt),
  .event_data_in_channel_halt(event_data_in_channel_halt),
  .event_data_out_channel_halt(event_data_out_channel_halt)
);

// Since the fft is block floating point the apparent signal amplitude can be shifted
// in consecutive fft blocks.  The Exponent shifter, exp_shifter, implements a simple
// low pass filtering the shift signal and provides gain correction mechanism for mitigating
// the amplitude shifts caused by the fft module.  The module also provides buffering
// and flow control logic so that it can directly connected to the rest of the rfnoc
// infrastructure.
exp_shifter exp_shifter (
  .clk(ce_clk),
  .sync_reset(reset_int),
  .fft_size(fft_size),

  .s_axis_data_tdata({m_axis_fft_data_tdata[15:0], m_axis_fft_data_tdata[31:16]}),
  .s_axis_data_tuser(m_axis_fft_data_tuser),
  .s_axis_data_tvalid(m_axis_fft_data_tvalid),
  .s_axis_data_tready(m_axis_fft_data_tready),
  .s_axis_data_tlast(m_axis_fft_data_tlast),

  .s_axis_status_tdata(m_axis_fft_status_tdata),
  .s_axis_status_tvalid(m_axis_fft_status_tvalid),
  .s_axis_status_tready(m_axis_fft_status_tready),
  .eob_tag(eob_tag),

  .m_axis_tdata(m_axis_tdata),
  .m_axis_tuser(m_axis_tuser),
  .m_axis_tvalid(m_axis_tvalid),
  .m_axis_tlast(m_axis_tlast),
  .m_axis_tready(m_axis_tready)
);
endmodule
