`timescale 1ns/1ps
`define NS_PER_TICK 1
`define NUM_TEST_CASES 4

`define SEEK_SET 0
`define SEEK_CUR 1
`define SEEK_END 2

`include "sim_exec_report.vh"
`include "sim_clks_rsts.vh"
`include "sim_rfnoc_lib.svh"

module noc_block_channelizer_tb();
  `TEST_BENCH_INIT("noc_block_channelizer",`NUM_TEST_CASES,`NS_PER_TICK);
  localparam BUS_CLK_PERIOD = $ceil(20);
  localparam CE_CLK_PERIOD  = $ceil(20);
  localparam NUM_CE         = 1;  // Number of Computation Engines / User RFNoC blocks to simulate
  localparam NUM_STREAMS    = 1;  // Number of test bench streams
  `RFNOC_SIM_INIT(NUM_CE, NUM_STREAMS, BUS_CLK_PERIOD, CE_CLK_PERIOD);
  `RFNOC_ADD_BLOCK(noc_block_channelizer, 0);

  localparam SPP = 16; // Samples per packet

  /********************************************************
  ** Verification
  ********************************************************/
  initial begin : tb_main
    string s;
    logic [31:0] random_word;
    logic [63:0] readback;
    int num_taps;
    int in_file;
    int fd_int;
    int fd_coeffs;
    int file_size;
    int temp;
    int offset;
    int position = 0;
    cvita_payload_t send_payload, recv_payload;
    cvita_metadata_t md;


    /********************************************************
    ** Test 1 -- Reset
    ********************************************************/
    `TEST_CASE_START("Wait for Reset");
    while (bus_rst) @(posedge bus_clk);
    while (ce_rst) @(posedge ce_clk);
    `TEST_CASE_DONE(~bus_rst & ~ce_rst);

    /********************************************************
    ** Test 2 -- Check for correct NoC IDs
    ********************************************************/
    `TEST_CASE_START("Check NoC ID");
    // Read NOC IDs
    tb_streamer.read_reg(sid_noc_block_channelizer, RB_NOC_ID, readback);
    $display("Read Frame Detect NOC ID: %16x", readback);
    `ASSERT_ERROR(readback == noc_block_channelizer.NOC_ID, "Incorrect NOC ID");
    `TEST_CASE_DONE(1);

    /********************************************************
    ** Test 3 -- Connect RFNoC blocks
    ********************************************************/
    `TEST_CASE_START("Connect RFNoC blocks");
    `RFNOC_CONNECT(noc_block_tb,noc_block_channelizer,SC16,SPP);
    `RFNOC_CONNECT(noc_block_channelizer,noc_block_tb,SC16,SPP);
    `TEST_CASE_DONE(1);

    /********************************************************
    ** Test 4 -- Load Coefficients
    ********************************************************/
    `TEST_CASE_START("Load Coefficients");
    fd_coeffs = $fopen("/home/cuervo/git/rfnoc-pfb-channelizer/rfnoc/testbenches/noc_block_channelizer_tb/M_32_taps.bin", "r");
    //fd_coeffs = $fopen("M_32_taps.bin", "r");
    $display("current file position = %d", $ftell(fd_coeffs));
    temp = $fseek(fd_coeffs, 0, `SEEK_END);
    file_size = $ftell(fd_coeffs) >> 2;  // this is in bytes / 4.
    offset = $fseek(fd_coeffs, position, `SEEK_SET);
    $display("Taps - Current file position = %d", $ftell(fd_coeffs));
    $display("Taps : FileDescr = %d", fd_coeffs);
    $display("Taps File Size : %d", file_size);
    // seek back to the beginning of the file.
    offset = $fseek(fd_coeffs, 0, `SEEK_SET);
    $display("Taps - Current file position = %d", $ftell(fd_coeffs));
    // tb_streamer.read_reg(sid_noc_block_channelizer, RB_NOC_ID, readback);
    // $display("Read Frame Detect NOC ID: %16x", readback);

    /* Set filter coefficients via reload bus */
    begin
        // set fft_size to 256
        int num_read;
        reg [31:0] data32;
        reg [31:0] memory;
        int file_idx;
        // data32 = 32'd256;
        data32 = 32'd16;
        tb_streamer.write_reg(sid_noc_block_channelizer, noc_block_channelizer.SR_FFT_SIZE, data32);
        for (int i=0; i<file_size; i++) begin //file_size
              file_idx = $ftell(fd_coeffs);
              num_read = $fread(memory, fd_coeffs, file_idx, 1);
              data32 = {memory[7:0], memory[15:8], memory[23:16], memory[31:24]};
              if (i == file_size-1) begin
                tb_streamer.write_reg(sid_noc_block_channelizer, noc_block_channelizer.SR_RELOAD_LAST, data32);
              end else begin
                tb_streamer.write_reg(sid_noc_block_channelizer, noc_block_channelizer.SR_RELOAD, data32);
              end
        end
    end

    /********************************************************
    ** Test 5 -- Test Channelizer
    ********************************************************/
    // Sending an impulse will readback the FIR filter coefficients
    `TEST_CASE_START("Test Channelizer");
    fd_int = $fopen("sig_store_256.bin", "r");

    $display("current file position = %d", $ftell(fd_int));
    temp = $fseek(fd_int, 0, `SEEK_END);
    file_size = $ftell(fd_int) >> 2;  // ftell is in bytes
    offset = $fseek(fd_int, position, `SEEK_SET);
    $display("Reader - current file position = %d", $ftell(fd_int));
    $display("Reader : FileDescr = %d",fd_int);
    $display("File Size : %d", file_size);
    // seek back to the beginning of the file.
    offset = $fseek(fd_int, 0, `SEEK_SET);
    $display("Reader - current file position = %d", $ftell(fd_int));
    tb_streamer.read_reg(sid_noc_block_channelizer, RB_NOC_ID, readback);
    $display("Read Frame Detect NOC ID: %16x", readback);

    // tb_streamer.write_reg(sid_noc_block_channelizer, noc_block_channelizer.SR_FFT_SIZE, 256);
    fork
    begin
        int r;
        int eob;
        int num_read;
        reg [31:0] data32;
        logic [63:0] data;
        reg [31:0] memory;
        int frame_end;
        int file_idx;
        for (int i=0; i<file_size; i++) begin
              if ((i % SPP-1) == 0 && i != 0) begin
                frame_end = 1;
              end else begin
                frame_end = 0;
              end
              file_idx = $ftell(fd_int);
              num_read = $fread(memory, fd_int, file_idx, 1);
              data32 = {memory[7:0], memory[15:8], memory[23:16], memory[31:24]};
              tb_streamer.push_word({data32[15:0], data32[31:16]}, frame_end);
              #5ns;
        end
    end
    begin
      cvita_metadata_t md;
      cvita_payload_t recv_payload;
      md.eob = 0;
      // make sure the
      //while (~md.eob) tb_streamer.recv(recv_payload,md);
      while(1) begin
        while (~md.eob) tb_streamer.recv(recv_payload,md);
        // tb_streamer.recv(recv_payload,md);
        // $display("Reading Data");
        md.eob = 0;
        #20000ns;
      end

    end
    join
    `TEST_CASE_DONE(1);
    `TEST_BENCH_DONE;

  end
endmodule
