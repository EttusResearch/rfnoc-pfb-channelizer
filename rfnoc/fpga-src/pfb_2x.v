//
// Copyright 2016-2018 Ettus Research, A National Instruments Company
//
// SPDX-License-Identifier: LGPL-3.0
//
// Module: pfb_2x
// Description:
// Implements the M/2 PFB architecture referenced in the
// "A Versatile Multichannel Filter Bank with Multiple Channel Bandwidths" paper.
// This architecture has been mapped to the Xilinx architecture.
// This represents a fully pipelined design that maximizes the FMax potential
// of the design. It is important to understand that filter arms are loaded
// sequentially. This is referenced in the diagram by the incremental changes
// in the phase subscript through each subsequent delay register.
// The nth index is only updated once per revolution of the filter bank.
//
// It is best to refer to the attached document to understand the layout of the
// logic. This module currently implements 24 taps per phase.

module pfb_2x
(
  input sync_reset,
  input clk,

  input [8:0] phase,
  input [9:0] fft_size,
  input [35:0] input_sig,
  input valid_i,

  input [31:0] s_axis_reload_tdata,
  input s_axis_reload_tlast,
  input s_axis_reload_tvalid,
  output s_axis_reload_tready,

  output [35:0] output_sig,
  output [8:0] phase_out,
  output valid_out //output valid signal.
);

wire [24:0] taps[0:23];
wire [47:0] pcouti[0:22];
wire [47:0] pcoutq[0:22];
wire [47:0] pouti, poutq;

wire [35:0] delay[0:23];
wire [35:0] delay_sig;

reg [8:0] phase_d[0:29];
reg [7:0] valid_d;

reg [10:0] wr_addr_d[0:71];

reg [10:0] rd_addr, next_rd_addr;
reg [10:0] wr_addr, next_wr_addr;
reg [10:0] rd_addr_d[0:22];

reg [10:0] next_rd_addr_d[0:22];

reg [35:0] input_sig_d1, input_sig_d2, input_sig_d3;
reg [35:0] sig, next_sig;
reg [35:0] sig_d1, sig_d2, sig_d3;
reg [1:0] offset_cnt, next_offset_cnt;
reg [1:0] offset_cnt_prev, next_offset_cnt_prev;

reg s_axis_reload_tvalid_d1;
reg [13:0] taps_addr, next_taps_addr;

reg [24:0] taps_dina, next_taps_dina;
reg [8:0] rom_addr, next_rom_addr;
reg [24:0] rom_data, next_rom_data;
reg reload_tready;

reg new_coeffs, next_new_coeffs;
reg [23:0] taps_we, next_taps_we;

reg [9:0] fft_max;
reg [8:0] fft_max_slice;
reg [8:0] fft_half;

assign valid_out = valid_d[7];
assign output_sig = {pouti[40:23], poutq[40:23]};
assign phase_out = phase_d[29];
assign s_axis_reload_tready = reload_tready;


// logic implements the sample write address pipelining.
integer n;
always @(posedge clk)
begin
  fft_max <= fft_size - 1;
  fft_max_slice <= fft_max[8:0];
  fft_half <= fft_size >> 1;

  phase_d[0] <= rd_addr[8:0];
  phase_d[1] <= phase_d[0] & fft_max_slice;
  phase_d[2] <= phase_d[1];
  s_axis_reload_tvalid_d1 <= s_axis_reload_tvalid;

  input_sig_d1 <= input_sig;
  input_sig_d2 <= input_sig_d1;
  input_sig_d3 <= input_sig_d2;

  wr_addr_d[0] <= wr_addr;
  wr_addr_d[3] <= {~rd_addr[10], rd_addr[9], rd_addr[8:0]};
  wr_addr_d[6] <= {~rd_addr_d[0][10], rd_addr_d[0][9], rd_addr_d[0][8:0]};
  wr_addr_d[9] <= {~rd_addr_d[1][10], rd_addr_d[1][9], rd_addr_d[1][8:0]};
  wr_addr_d[12] <= {~rd_addr_d[2][10], rd_addr_d[2][9], rd_addr_d[2][8:0]};
  wr_addr_d[15] <= {~rd_addr_d[3][10], rd_addr_d[3][9], rd_addr_d[3][8:0]};
  wr_addr_d[18] <= {~rd_addr_d[4][10], rd_addr_d[4][9], rd_addr_d[4][8:0]};
  wr_addr_d[21] <= {~rd_addr_d[5][10], rd_addr_d[5][9], rd_addr_d[5][8:0]};
  wr_addr_d[24] <= {~rd_addr_d[6][10], rd_addr_d[6][9], rd_addr_d[6][8:0]};
  wr_addr_d[27] <= {~rd_addr_d[7][10], rd_addr_d[7][9], rd_addr_d[7][8:0]};
  wr_addr_d[30] <= {~rd_addr_d[8][10], rd_addr_d[8][9], rd_addr_d[8][8:0]};
  wr_addr_d[33] <= {~rd_addr_d[9][10], rd_addr_d[9][9], rd_addr_d[9][8:0]};
  wr_addr_d[36] <= {~rd_addr_d[10][10], rd_addr_d[10][9], rd_addr_d[10][8:0]};
  wr_addr_d[39] <= {~rd_addr_d[11][10], rd_addr_d[11][9], rd_addr_d[11][8:0]};
  wr_addr_d[42] <= {~rd_addr_d[12][10], rd_addr_d[12][9], rd_addr_d[12][8:0]};
  wr_addr_d[45] <= {~rd_addr_d[13][10], rd_addr_d[13][9], rd_addr_d[13][8:0]};
  wr_addr_d[48] <= {~rd_addr_d[14][10], rd_addr_d[14][9], rd_addr_d[14][8:0]};
  wr_addr_d[51] <= {~rd_addr_d[15][10], rd_addr_d[15][9], rd_addr_d[15][8:0]};
  wr_addr_d[54] <= {~rd_addr_d[16][10], rd_addr_d[16][9], rd_addr_d[16][8:0]};
  wr_addr_d[57] <= {~rd_addr_d[17][10], rd_addr_d[17][9], rd_addr_d[17][8:0]};
  wr_addr_d[60] <= {~rd_addr_d[18][10], rd_addr_d[18][9], rd_addr_d[18][8:0]};
  wr_addr_d[63] <= {~rd_addr_d[19][10], rd_addr_d[19][9], rd_addr_d[19][8:0]};
  wr_addr_d[66] <= {~rd_addr_d[20][10], rd_addr_d[20][9], rd_addr_d[20][8:0]};
  wr_addr_d[69] <= {~rd_addr_d[21][10], rd_addr_d[21][9], rd_addr_d[21][8:0]};

  wr_addr_d[1] <= wr_addr_d[0];
  wr_addr_d[2] <= wr_addr_d[1];
  wr_addr_d[4] <= wr_addr_d[3];
  wr_addr_d[5] <= wr_addr_d[4];
  wr_addr_d[7] <= wr_addr_d[6];
  wr_addr_d[8] <= wr_addr_d[7];
  wr_addr_d[10] <= wr_addr_d[9];
  wr_addr_d[11] <= wr_addr_d[10];
  wr_addr_d[13] <= wr_addr_d[12];
  wr_addr_d[14] <= wr_addr_d[13];
  wr_addr_d[16] <= wr_addr_d[15];
  wr_addr_d[17] <= wr_addr_d[16];
  wr_addr_d[19] <= wr_addr_d[18];
  wr_addr_d[20] <= wr_addr_d[19];
  wr_addr_d[22] <= wr_addr_d[21];
  wr_addr_d[23] <= wr_addr_d[22];
  wr_addr_d[25] <= wr_addr_d[24];
  wr_addr_d[26] <= wr_addr_d[25];
  wr_addr_d[28] <= wr_addr_d[27];
  wr_addr_d[29] <= wr_addr_d[28];
  wr_addr_d[31] <= wr_addr_d[30];
  wr_addr_d[32] <= wr_addr_d[31];
  wr_addr_d[34] <= wr_addr_d[33];
  wr_addr_d[35] <= wr_addr_d[34];
  wr_addr_d[37] <= wr_addr_d[36];
  wr_addr_d[38] <= wr_addr_d[37];
  wr_addr_d[40] <= wr_addr_d[39];
  wr_addr_d[41] <= wr_addr_d[40];
  wr_addr_d[43] <= wr_addr_d[42];
  wr_addr_d[44] <= wr_addr_d[43];
  wr_addr_d[46] <= wr_addr_d[45];
  wr_addr_d[47] <= wr_addr_d[46];
  wr_addr_d[49] <= wr_addr_d[48];
  wr_addr_d[50] <= wr_addr_d[49];
  wr_addr_d[52] <= wr_addr_d[51];
  wr_addr_d[53] <= wr_addr_d[52];
  wr_addr_d[55] <= wr_addr_d[54];
  wr_addr_d[56] <= wr_addr_d[55];
  wr_addr_d[58] <= wr_addr_d[57];
  wr_addr_d[59] <= wr_addr_d[58];
  wr_addr_d[61] <= wr_addr_d[60];
  wr_addr_d[62] <= wr_addr_d[61];
  wr_addr_d[64] <= wr_addr_d[63];
  wr_addr_d[65] <= wr_addr_d[64];
  wr_addr_d[67] <= wr_addr_d[66];
  wr_addr_d[68] <= wr_addr_d[67];
  wr_addr_d[70] <= wr_addr_d[69];
  wr_addr_d[71] <= wr_addr_d[70];
  sig_d1 <= sig;
  sig_d2 <= sig_d1;
  sig_d3 <= sig_d2;
  reload_tready <= 1'b1;
end


// clock and reset process.
integer m;
always @(posedge clk, posedge sync_reset)
begin
	if (sync_reset == 1'b1) begin
    offset_cnt <= 1;  // this ensures that the first read / write is to offset 0.
    offset_cnt_prev <= 0;
    sig <= 0;
    valid_d <= 0;
    for (m=0; m<23; m=m+1) begin
      rd_addr_d[m] <= 0;
    end
    new_coeffs <= 1'b1;
    taps_addr <= 0;
    rom_addr <= 0;
    rom_data <= 0;
    taps_we <= 0;
    taps_dina <= 0;
    rd_addr <= 0;
    wr_addr <= 0;
	end else begin
    offset_cnt <= next_offset_cnt;
    offset_cnt_prev <= next_offset_cnt_prev;
    sig <= next_sig;
    valid_d <= {valid_d[6:0], valid_i};
    phase_d[0] <= phase;
    for (m=0; m<23; m=m+1) begin
      rd_addr_d[m] <= next_rd_addr_d[m];
    end
    new_coeffs <= next_new_coeffs;
    taps_addr <= next_taps_addr;
    rom_addr <= next_rom_addr;
    rom_data <= next_rom_data;
    taps_we <= next_taps_we;
    taps_dina <= next_taps_dina;
    rd_addr <= next_rd_addr;
    wr_addr <= next_wr_addr;
	end
end

always @(posedge clk)
begin
  if (valid_d[6] == 1'b1) begin
    phase_d[3] <= phase_d[2];
    phase_d[4] <= phase_d[3];
    phase_d[5] <= phase_d[4];
    phase_d[6] <= phase_d[5];
    phase_d[7] <= phase_d[6];
    phase_d[8] <= phase_d[7];
    phase_d[9] <= phase_d[8];
    phase_d[10] <= phase_d[9];
    phase_d[11] <= phase_d[10];
    phase_d[12] <= phase_d[11];
    phase_d[13] <= phase_d[12];
    phase_d[14] <= phase_d[13];
    phase_d[15] <= phase_d[14];
    phase_d[16] <= phase_d[15];
    phase_d[17] <= phase_d[16];
    phase_d[18] <= phase_d[17];
    phase_d[19] <= phase_d[18];
    phase_d[20] <= phase_d[19];
    phase_d[21] <= phase_d[20];
    phase_d[22] <= phase_d[21];
    phase_d[23] <= phase_d[22];
    phase_d[24] <= phase_d[23];
    phase_d[25] <= phase_d[24];
    phase_d[26] <= phase_d[25];
    phase_d[27] <= phase_d[26];
    phase_d[28] <= phase_d[27];
    phase_d[29] <= phase_d[28];
  end
end

// reload process
always @*
begin
  next_taps_addr = taps_addr;
  next_taps_dina = taps_dina;
  next_new_coeffs = new_coeffs;

  next_rom_addr = taps_addr[8:0];
  next_rom_data = taps_dina;
  if (s_axis_reload_tvalid == 1'b1) begin
    next_taps_dina = s_axis_reload_tdata[24:0];
    if (new_coeffs == 1'b1) begin
      next_taps_addr = 0;
      next_new_coeffs = 1'b0;
    end else begin
      next_taps_addr = taps_addr + 1;
      if (s_axis_reload_tlast == 1'b1) begin
        next_new_coeffs = 1'b1;
      end
    end
  end
  // implements the write address pointer for tap updates.
  if (s_axis_reload_tvalid_d1 == 1'b1) begin
    case (taps_addr[13:9])
      5'd0: next_taps_we = 24'd1;
      5'd1: next_taps_we = 24'd2;
      5'd2: next_taps_we = 24'd4;
      5'd3: next_taps_we = 24'd8;
      5'd4: next_taps_we = 24'd16;
      5'd5: next_taps_we = 24'd32;
      5'd6: next_taps_we = 24'd64;
      5'd7: next_taps_we = 24'd128;
      5'd8: next_taps_we = 24'd256;
      5'd9: next_taps_we = 24'd512;
      5'd10: next_taps_we = 24'd1024;
      5'd11: next_taps_we = 24'd2048;
      5'd12: next_taps_we = 24'd4096;
      5'd13: next_taps_we = 24'd8192;
      5'd14: next_taps_we = 24'd16384;
      5'd15: next_taps_we = 24'd32768;
      5'd16: next_taps_we = 24'd65536;
      5'd17: next_taps_we = 24'd131072;
      5'd18: next_taps_we = 24'd262144;
      5'd19: next_taps_we = 24'd524288;
      5'd20: next_taps_we = 24'd1048576;
      5'd21: next_taps_we = 24'd2097152;
      5'd22: next_taps_we = 24'd4194304;
      5'd23: next_taps_we = 24'd8388608;
      default: next_taps_we = 24'd0;
    endcase
  end else begin
    next_taps_we = 24'd0;
  end
end

// read and write address update logic.
always @*
begin
  next_offset_cnt = offset_cnt;
  next_offset_cnt_prev = offset_cnt_prev;
  next_rd_addr = rd_addr;
  next_wr_addr = wr_addr;
  // increment offset count once per cycle through the PFB arms.
  if (valid_d[2] == 1'b1) begin
    if (phase_d[2] == 9'd0) begin
      next_offset_cnt_prev = offset_cnt;
      next_offset_cnt = offset_cnt + 1;
      next_wr_addr = {offset_cnt + 1, phase_d[2]};
      next_rd_addr = {offset_cnt, phase_d[2]};
    end else begin
      next_rd_addr = {offset_cnt_prev, phase_d[2]};
      next_wr_addr = {offset_cnt, phase_d[2]};
    end
  end

  if (valid_d[2] == 1'b1) begin
    if ((phase_d[2] & fft_half) != 8'd0) begin
      next_sig = delay_sig;
    end else begin
      next_sig = input_sig_d3;
    end
  end else begin
    next_sig = sig;
  end

  // shift through old values.
  if (valid_d[2] == 1'b1) begin
    next_rd_addr_d[0] = rd_addr;
    for (n=1; n<23; n=n+1) begin
      next_rd_addr_d[n] <= rd_addr_d[n-1];
    end
  end else begin
    for (n=0; n<23; n=n+1) begin
      next_rd_addr_d[n] <= rd_addr_d[n];
    end
  end
end

// 3 cycle latency.
sample_delay sample_delay (
  .clka(clk),    // input wire clka
  .wea(valid_d[0]),      // input wire [0 : 0] wea
  .addra(phase_d[0][8:0]),  // input wire [11 : 0] addra
  .dina(input_sig_d1),    // input wire [35 : 0] dina
  .clkb(clk),    // input wire clkb
  .addrb(phase[8:0]),  // input wire [11 : 0] addrb
  .doutb(delay_sig)  // output wire [35 : 0] doutb
);

// 3 cycle latency
sample_ram sample_ram_0 (
  .clka(clk),    // input wire clka
  .wea(valid_d[6]),      // input wire [0 : 0] wea
  .addra(wr_addr_d[2][10:0]),  //
  .dina(sig_d3),    // input wire [35 : 0] dina
  .clkb(clk),    // input wire clkb
  .addrb(rd_addr[10:0]),  // input wire [10 : 0] addrb
  .doutb(delay[0])  // output wire [35 : 0] doutb
);

genvar i;
generate
  for (i=1; i<24; i=i+1) begin : TAP_DELAY
    sample_ram sample_ram_inst (
    .clka(clk),    // input wire clka
    .wea(valid_d[6]),      // input wire [0 : 0] wea
    .addra(wr_addr_d[i*3+2][10:0]),  // input wire [13 : 0] addra
    .dina(delay[i-1]),    // input wire [35 : 0] dina
    .clkb(clk),    // input wire clkb
    .addrb(rd_addr_d[i-1][10:0]),  // input wire [13 : 0] addrb
    .doutb(delay[i])  // output wire [35 : 0] doutb
    );
  end
endgenerate

// Coefficent memories
// latency = 3.
pfb_taps pfb_taps_0 (
  .clka(clk),
  .wea(taps_we[0]),
  .addra(rom_addr[8:0]),
  .dina(rom_data),
  .clkb(clk),    // input wire clka
  .addrb(rd_addr[8:0]),  // input wire [11 : 0] addra
  .doutb(taps[0])  // output wire [24 : 0] douta
);

genvar nn;
generate
  for (nn=1; nn<24; nn=nn+1) begin : COEFFS
    pfb_taps pfb_taps_n
    (
      .clka(clk),
      .wea(taps_we[nn]),
      .addra(rom_addr[8:0]),
      .dina(rom_data),
      .clkb(clk),    // input wire clka
      .addrb(rd_addr_d[nn-1][8:0]),  // input wire [7 : 0] addra
      .doutb(taps[nn])  // output wire [24 : 0] douta
    );
  end
endgenerate


// PFB MAC blocks
pfb_mac_0 pfb_mac_i_start (
  .CLK(clk),      // input wire CLK
  .CE(valid_d[6]),
  .A(taps[0]),          // input wire [24 : 0] A
  .B(delay[0][35:18]),          // input wire [17 : 0] B
  .PCOUT(pcouti[0]),  // output wire [47 : 0] PCOUT
  .P()          // output wire [42 : 0] P
);

// Latency = 6
pfb_mac_0 pfb_mac_q_start (
  .CLK(clk),      // input wire CLK
  .CE(valid_d[6]),
  .A(taps[0]),          // input wire [24 : 0] A
  .B(delay[0][17:0]),          // input wire [17 : 0] B
  .PCOUT(pcoutq[0]),  // output wire [47 : 0] PCOUT
  .P()          // output wire [42 : 0] P
);

genvar j;
generate
  for (j=1; j<23; j=j+1) begin : MAC
    pfb_mac pfb_mac_i
    (
      .CLK(clk),      // input wire CLK
      .CE(valid_d[6]),        // input wire CE
      .PCIN(pcouti[j-1]),    // input wire [47 : 0] PCIN
      .A(taps[j]),          // input wire [24 : 0] A
      .B(delay[j][35:18]),          // input wire [17 : 0] B
      .PCOUT(pcouti[j]),  // output wire [47 : 0] PCOUT
      .P()          // output wire [47 : 0] P
    );

    pfb_mac pfb_mac_q
    (
      .CLK(clk),      // input wire CLK
      .CE(valid_d[6]),        // input wire CE
      .PCIN(pcoutq[j-1]),    // input wire [47 : 0] PCIN
      .A(taps[j]),          // input wire [24 : 0] A
      .B(delay[j][17:0]),          // input wire [17 : 0] B
      .PCOUT(pcoutq[j]),  // output wire [47 : 0] PCOUT
      .P()          // output wire [47 : 0] P
    );
  end
endgenerate

pfb_mac pfb_mac_i_23 (
  .CLK(clk),      // input wire CLK
  .CE(valid_d[6]),
  .PCIN(pcouti[22]),    // input wire [47 : 0] PCIN
  .A(taps[23]),          // input wire [24 : 0] A
  .B(delay[23][35:18]),          // input wire [17 : 0] B
  .PCOUT(),  // output wire [47 : 0] PCOUT
  .P(pouti)          // output wire [42 : 0] P
);

pfb_mac pfb_mac_q_23 (
  .CLK(clk),      // input wire CLK
  .CE(valid_d[6]),
  .PCIN(pcoutq[22]),    // input wire [47 : 0] PCIN
  .A(taps[23]),          // input wire [24 : 0] A
  .B(delay[23][17:0]),          // input wire [17 : 0] B
  .PCOUT(),  // output wire [47 : 0] PCOUT
  .P(poutq)          // output wire [42 : 0] P
);

endmodule
