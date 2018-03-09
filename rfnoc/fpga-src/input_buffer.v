/*****************************************************************************/
//
//
// The muxing shown provides the capability for the input buffer to be reading one dual port ram while
// the other is being written. The fixed counters provide the appropriate addressing for both reading
// and writing RAMs. Note that input data rate must be 1/2 the clock rate to maintain unthrottled performance
/*****************************************************************************/

module input_buffer
(
    input sync_reset,
    input clk,

    input [35:0] s_tdata,
    input s_tvalid,
    output s_tready,

    input [9:0] fft_size,

    output [35:0] output_sig,
    output [8:0] phase,
    output valid_out
);


wire [10:0] count_value;
reg [10:0] count_value_d1;
reg [10:0] count_offset;
wire count_valid;

wire [35:0] count_data;

reg we_0, next_we_0;
reg we_1, next_we_1;

reg we_side, next_we_side;
reg rd_side, next_rd_side;
reg [10:0] addra;

wire [35:0] buffer_0, buffer_1;

reg [35:0] count_data_d1;
reg [35:0] count_data_d2;
reg count_valid_d1;

reg [7:0] roll_over_offset;

localparam S_IDLE=0, S_READ=1;
reg state, next_state;

reg [3:0] rd_en_d;
reg rd_en, next_rd_en;

reg [8:0] read_addr, next_read_addr;
reg [8:0] read_addr_d1, read_addr_d2, read_addr_d3, read_addr_d4;
reg [8:0] phase_s, next_phase;

reg tvalid, next_tvalid;

reg [3:0] rd_side_d;
wire [7:0] raddr;
reg [35:0] data, next_data;

reg [9:0] fft_max, fft_max_m1, fft_max_m2, fft_max_m3;
reg [8:0] fft_max_slice;
reg [7:0] roll_over;
reg [7:0] roll_over_m1;

reg tready, next_tready;

wire take_data;
wire read_window;

reg write_done;
reg take_data_d1;
reg next_write_done;
wire write_d;

assign take_data = s_tvalid & s_tready;
// this signal indicates where throttling should be released if engaged.  The address range is a reflection
// of the internal pipelining.
assign read_window = (read_addr == fft_max_m3 || read_addr == fft_max_m2 || read_addr == fft_max_m1 || read_addr == fft_max) ? 1'b1 : 1'b0;
assign s_tready = tready;
// provides truncated read address.  Note that the read address will cycle twice per each cycle of the
// write address.
assign raddr = read_addr_d1[7:0] & roll_over;
assign write_d = (count_valid_d1 == 1'b1 && count_value_d1 == roll_over) ? 1'b1 : 1'b0;
assign phase = phase_s;
assign valid_out = tvalid;
assign output_sig = data;

// clock and reset proces.
always @(posedge clk, posedge sync_reset)
begin
	if (sync_reset == 1'b1) begin
        we_side <= 1'b0;
        rd_side <= 1'b0;
        we_0 <= 1'b0;
        we_1 <= 1'b0;
        state <= S_IDLE;
        rd_en <= 1'b0;
        rd_en_d <= 0;
        data <= 0;
        tvalid <= 1'b0;
        phase_s <= 0;
        read_addr <= 0;
        tready <= 1'b1;
        write_done <= 1'b0;
	end else begin
        we_side <= next_we_side;
        rd_side <= next_rd_side;
        we_0 <= next_we_0;
        we_1 <= next_we_1;
        state <= next_state;
        rd_en <= next_rd_en;
        rd_en_d <= {rd_en_d[2:0], rd_en};
        data <= next_data;
        tvalid <= next_tvalid;
        phase_s <= next_phase;
        read_addr <= next_read_addr;
        tready <= next_tready;
        write_done <= next_write_done;
	end
end


// delay process  -- pipelining of counters and read addresses.  Also pipelines the address offsets used
// by the read_window signal.
always @(posedge clk)
begin
    addra <= count_offset;
    count_data_d1 <= count_data;
    count_data_d2 <= count_data_d1;
    count_valid_d1 <= count_valid;
    count_value_d1 <= count_value;
    count_offset <= roll_over - count_value;
    read_addr_d1 <= read_addr;
    read_addr_d2 <= read_addr_d1;
    read_addr_d3 <= read_addr_d2;
    read_addr_d4 <= read_addr_d3;
    fft_max <= fft_size - 1;
    fft_max_slice <= fft_max[8:0];
    roll_over <= fft_max_slice[8:1];
    rd_side_d <= {rd_side_d[2:0], rd_side};
    roll_over_offset <= roll_over - 2;
    roll_over_m1 <= roll_over - 1;
    take_data_d1 <= take_data;
    fft_max_m3 <= fft_max - 3;
    fft_max_m2 <= fft_max - 2;
    fft_max_m1 <= fft_max - 1;
end

// write process.  Keeps track of current buffer being written.
always @*
begin
    next_we_0 = 1'b0;
    next_we_1 = 1'b0;
    next_we_side = we_side;
    if (count_valid_d1 == 1'b1) begin
        if (we_side == 1'b0) begin
            next_we_0 = 1'b1;
        end else begin
            next_we_1 = 1'b1;
        end
        if (count_offset == 0) begin
            next_we_side = ~we_side;
        end
    end
end

// tready process
always @*
begin
    next_tready = tready;
    if (state == S_IDLE) begin
        next_tready = 1'b1;
    end else begin
        if (read_window == 1'b1) begin
            next_tready = 1'b1;
        end else if (count_value == roll_over_offset && take_data == 1'b1 && take_data_d1 == 1'b1) begin
            next_tready = 1'b0;
        end else if (count_value == roll_over_m1 && take_data == 1'b1) begin
            next_tready = 1'b0;
        end
    end
end

// read process.  Controls the read pointers of the RAMs.  Pushes data out of the input buffers
// buffers.
always @*
begin
    next_state = state;
    next_rd_en = rd_en;
    next_rd_side = rd_side;
    next_read_addr = read_addr;
    next_write_done = write_done;
    case (state)
        S_IDLE:
        begin
            if (write_d == 1'b1) begin
                next_rd_en = 1'b1;
                next_read_addr = 0;
                next_state = S_READ;
                next_rd_side = we_side;
                next_write_done = 1'b0;
            end
        end
        S_READ:
        begin
            if (read_addr == fft_max_slice) begin
                if (write_d == 1'b1 || write_done == 1'b1) begin
                    next_rd_en = 1'b1;
                    next_read_addr = 0;
                    next_state = S_READ;
                    next_rd_side = ~rd_side;
                    next_write_done = 1'b0;
                end else begin
                    next_state = S_IDLE;
                    next_rd_en = 1'b0;
                end
            end else begin
                next_read_addr = read_addr + 1;
                next_rd_en = 1'b1;
                if (write_d == 1'b1) begin
                    next_write_done = 1'b1;
                end
            end
        end
        default :
        begin
        end
    endcase
end

// output pipelining
always @*
begin
    next_data = data;
    next_phase = phase_s;
    next_tvalid = rd_en_d[3];  //  & rd_valid;
    if (rd_en_d[3] == 1'b1) begin
        // reading from top setp of buffers
        next_phase = read_addr_d4;
        if (rd_side_d[3] == 1'b0) begin
            next_data = buffer_0;
        end else begin
            next_data = buffer_1;
        end
    end
end

// Block used for keeping track of input sample count.  Performs roll-over at specifed point.
// it has a storage length of 2.
// Latency = 3
count_cycle_iw36_cw11 count_cycle_iw36_cw11
(
	.sync_reset(sync_reset),
	.clk(clk),
	.valid_i(s_tvalid & tready),
	.high_cnt({3'b000, roll_over}),
	.data_i(s_tdata),
	.valid_out(count_valid),
	.count(count_value),
	.data_out(count_data)
);

// Latency = 3
// RAMs used for sample storage.  Note that they are read out 2 twice each time they are written.
input_buff_RAM buff_0 (
  .clka(clk),    // input wire clka
  .wea(we_0),      // input wire [0 : 0] wea
  .addra(addra[7:0]),  // input wire [10 : 0] addra
  .dina(count_data_d2),    // input wire [35 : 0] dina
  .clkb(clk),    // input wire clkb
  .addrb(raddr[7:0]),  // input wire [10 : 0] addrb
  .doutb(buffer_0)  // output wire [35 : 0] doutb
);

// Latency = 3
input_buff_RAM buff_1 (
  .clka(clk),    // input wire clka
  .wea(we_1),      // input wire [0 : 0] wea
  .addra(addra[7:0]),  // input wire [10 : 0] addra
  .dina(count_data_d2),    // input wire [35 : 0] dina
  .clkb(clk),    // input wire clkb
  .addrb(raddr[7:0]),  // input wire [10 : 0] addrb
  .doutb(buffer_1)  // output wire [35 : 0] doutb
);


endmodule
