/*****************************************************************************/
// The circular buffer implements the circular shifting functionality as
// specified in "A Versatile Multichannel Filter Bank with Multiple Channel Bandwidths" paper.
// It is implemented using multiple sample memores that allow the block to ping-pong between different memories.
// This mitigates the requirement to throttle the input data stream.
//
/*****************************************************************************/

module circ_buffer
(
    input sync_reset,
    input clk,

    input [8:0] phase,
    input [9:0] fft_size,
    input [35:0] input_sig,
    input valid_i,

    output [35:0] m_axis_tdata,
    output m_axis_tvalid,
    output m_axis_tlast
);

reg mux_switch, next_mux_switch;

// Phase shift register.  Reference Figure 7.
reg [8:0] phase_d[0:4];
reg [35:0] input_d[0:4];
reg [35:0] output_s, next_output_s;

wire [35:0] ram0, ram1;

reg [11:0] rd_cnt_val, next_rd_cnt_val;

// Buffer read state machine.  Used to read buffers in single burst, once a buffer is filled.
localparam S_IDLE=0, S_READ=1;
reg state, next_state;

reg [3:0] rd_en_d;
reg [3:0] rd_side_d;
// reg rd_valid, next_rd_valid;
// used to determine the current buffer being read from.
reg rd_side, next_rd_side;

// read enable signal; used to determine if the data on the ouput of a RAM is valid.
// it is fed into a shift register that follows the logic delays of the entire block.
reg rd_en, next_rd_en;
reg [6:0] valid_d;

// independent write enables to each RAM
reg next_we_0, we_0;
reg next_we_1, we_1;

reg tvalid, next_tvalid;

reg [35:0] data, next_data;
reg [8:0] rd_cnt_d[0:3];

reg tlast, next_tlast;

reg last_side, next_last_side;
reg write_end, next_write_end;

reg [9:0] phase_end; //, phase_end_m1, phase_end_m2;
wire [8:0] phase_end_s; // , phase_end_m1s, phase_end_m2s;

reg [8:0] raddr, next_raddr;

wire [8:0] half_cnt;
wire write_end_logic;

// output interface
assign m_axis_tvalid = tvalid;
assign m_axis_tdata = data;
assign m_axis_tlast = tlast;
assign write_end_logic = ((phase == phase_end_s) && (valid_i == 1'b1)) ? 1'b1 : 1'b0;
assign phase_end_s = phase_end[8:0];


assign half_cnt = fft_size[9:1];

// clock and reset process
always @(posedge clk, posedge sync_reset)
begin
	if (sync_reset == 1'b1) begin
        mux_switch <= 1'b0;
        valid_d <= 0;
        tlast <= 1'b0;
        rd_en <= 1'b0;
        data <= 0;
        rd_side <= 1'b0;
        rd_en_d <= 0;
        state <= S_IDLE;
        tvalid <= 1'b0;
        write_end <= 1'b0;
        last_side <= 1'b0;
        rd_cnt_val <= 0;
        raddr <= 0;
	end else begin
        mux_switch <= next_mux_switch;
        valid_d <= {valid_d[6:0], valid_i};
        tlast <= next_tlast;
        rd_en <= next_rd_en;
        data <= next_data;
        rd_side <= next_rd_side;
        rd_en_d <= {rd_en_d[2:0], rd_en};
        state <= next_state;
        tvalid <= next_tvalid;
        write_end <= next_write_end;
        last_side <= next_last_side;
        rd_cnt_val <= next_rd_cnt_val;
        raddr <= next_raddr;
	end
end


// delay process
integer i;
always @(posedge clk)
begin
    we_0 <= next_we_0;
    we_1 <= next_we_1;

    phase_end <= fft_size - 10'd1;
    output_s <= next_output_s;
    phase_d[0] <= phase;
    for (i=1; i<5; i=i+1) begin
        phase_d[i] <= phase_d[i-1];
    end
    input_d[0] <= input_sig;
    for (i=1; i<5; i=i+1) begin
        input_d[i] <= input_d[i-1];
    end

    rd_side_d <= {rd_side_d[2:0], rd_side};
    rd_cnt_d[0] <= rd_cnt_val;
    for (i=1;i<4;i=i+1) begin
        rd_cnt_d[i] <= rd_cnt_d[i-1];
    end
end


// write process.  Keeps track of current buffer being written.
always @*
begin
    next_we_0 = 1'b0;
    next_we_1 = 1'b0;
    if ((phase_d[3] == phase_end_s) && (valid_d[3] == 1'b1)) begin
        next_mux_switch = !mux_switch;
    end else begin
        next_mux_switch = mux_switch;
    end
    if (valid_d[3] == 1'b1) begin
        next_we_0 = !mux_switch;
        next_we_1 = mux_switch;
    end

end

// read process.  Controls the read pointers of the RAMs.  Pushes data out of the circular shift_val
// buffers
always @*
begin
    next_state = state;
    next_rd_en = rd_en;
    next_rd_side = rd_side;
    next_write_end = write_end;
    next_last_side = last_side;
    next_rd_cnt_val = rd_cnt_val;
    case (state)
        S_IDLE:
        begin
            // currently writing the last value.
            if (write_end_logic == 1'b1) begin // phase_d[1] == 8'd255 && (we_0 == 1'b1 || we_1 == 1'b1)) begin
                next_rd_en = 1'b1;
                next_state = S_READ;
                next_write_end = 1'b0;
                next_rd_cnt_val = 0;
                next_rd_side = mux_switch;
            end
        end
        S_READ:
        begin
            if (rd_cnt_val == phase_end) begin
                if (write_end == 1'b1 || write_end_logic == 1'b1) begin
                    next_rd_en = 1'b1;
                    next_rd_cnt_val = 0;
                    next_state = S_READ;
                    next_write_end = 1'b0;
                    if (write_end_logic == 1'b1) begin
                        next_rd_side = mux_switch;
                    end else begin
                        next_rd_side = last_side;
                    end
                end else begin
                    next_state = S_IDLE;
                    next_rd_en = 1'b0;
                end
            end else begin
                next_rd_cnt_val = rd_cnt_val + 1;
                if (write_end_logic == 1'b1) begin
                    next_write_end = 1'b1;
                    next_last_side = mux_switch;
                end
            end
        end
    endcase
end

// output pipelining
always @*
begin
    // next_rd_valid = 1'b0;
    next_data = data;
    next_tlast = 1'b0;
    next_tvalid = rd_en_d[3]; //  & rd_en_d[6];
    // output of dual port ram is valid.
    if (rd_en_d[3] == 1'b1) begin
        // reading from top setp of buffers
        if (rd_side_d[3] == 1'b0) begin
            next_data = ram0;
        end else begin
            next_data = ram1;
        end
        if (rd_cnt_d[3] == phase_end) begin
            next_tlast = 1'b1;
        end
    end
    if (rd_side == 1'b1) begin
        next_raddr = (half_cnt + rd_cnt_val) & phase_end_s;
    end else begin
        next_raddr = rd_cnt_val;
    end
end

// Ping-pong memories.
// Latency = 3
circ_buff_ram circ_buff_0 (
  .clka(clk),    // input wire clka
  .wea(we_0),      // input wire [0 : 0] wea
  .addra(phase_d[4][8:0]),  // input wire [11 : 0] addra
  .dina(input_d[4]),    // input wire [35 : 0] dina
  .clkb(clk),    // input wire clkb
  .addrb(raddr[8:0]),  // input wire [7 : 0] addrb
  .doutb(ram0)  // output wire [35 : 0] doutb
);

// Latency = 3
circ_buff_ram circ_buff_1 (
  .clka(clk),    // input wire clka
  .wea(we_1),      // input wire [0 : 0] wea
  .addra(phase_d[4][8:0]),  // input wire [11 : 0] addra
  .dina(input_d[4]),    // input wire [35 : 0] dina
  .clkb(clk),    // input wire clkb
  .addrb(raddr[8:0]),  // input wire [7 : 0] addrb
  .doutb(ram1)  // output wire [35 : 0] doutb
);


endmodule
