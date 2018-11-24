module MyDesign #(
        parameter OUTPUT_LENGTH  = 8,
        parameter MAX_MESSAGE_LENGTH = 55,
        parameter NUMBER_OF_Ks = 64,
        parameter NUMBER_OF_Hs = 8,
        parameter SYMBOL_WIDTH =8)
    (
    input wire [ $clog2(MAX_MESSAGE_LENGTH):0] xxx__dut__msg_length ,
    output wire dut__xxx__finish     ,
    input wire xxx__dut__go         ,
    output [ $clog2(MAX_MESSAGE_LENGTH)-1:0] dut__msg__address    ,
    input [7:0] msg__dut__data       ,
    output wire dut__msg__enable     ,
    output wire dut__msg__write      ,

    output wire [5:0] dut__kmem__address   ,
    input  wire [31:0] kmem__dut__data      ,
    output wire dut__kmem__enable    ,
    output wire dut__kmem__write     ,

    output wire [2:0] dut__hmem__address   ,
    input  wire [31:0] hmem__dut__data      ,
    output wire dut__hmem__enable    ,
    output wire dut__hmem__write     ,

    output wire [2:0] dut__dom__address,
    output  wire [31:0] dut__dom__data,
    output wire dut__dom__enable,
    output wire dut__dom__write,

    input wire reset,
    input wire clk
    );


wire [2:0] m_states;
wire m_finish;
wire [31:0] M_1_out [15:0];
wire h_finish;
wire [31:0] saved_Hs [7:0];
wire k_finish;
wire [31:0] saved_Ks [63:0];

//big controller wires
wire    msram_con_finish;
wire    hsram_con_finish;
wire    ksram_con_finish;
wire    wcon_con_finish;
wire    comp_con_finish;
wire    push_con_finish;
wire    con_msram_start;
wire    con_hsram_start;
wire    con_ksram_start;
wire    con_wcon_start;
wire    con_comp_start;
wire    con_push_start;

//w_control wires
wire  [5:0]   count_current_evens;
wire  [5:0]   count_minus15_evens;
wire  [5:0]   count_minus2_evens;
wire  [5:0]   count_minus16_evens;
wire  [5:0]   count_minus7_evens;
wire  [5:0]   count_current_odds;
wire  [5:0]   count_minus15_odds;
wire  [5:0]   count_minus2_odds;
wire  [5:0]   count_minus16_odds;
wire  [5:0]   count_minus7_odds;
wire  [2:0]   w_states;

//w_datapath wires
wire  [31:0]  W_final  [63:0];
//compression_control wires
wire  [2:0]   compression_states;
wire  [5:0]   compression_counter;
//compression datapath wires
wire  [31:0]  final_H [7:0];

sram_push #(.SRAM_SIZE(8))
    hash_to_mem(
    .clock(clk),
    .reset(reset),
    .control(con_push_start),
    .H_in(final_H),
    .write(dut__dom__write),
    .enable(dut__dom__enable),
    .address_output(dut__dom__address),
    .finish(push_con_finish),
    .dut_dom_data(dut__dom__data)
);

w_control
 wcon (
    .clock(clk),
    .reset(reset),
    .start(con_wcon_start),
    .count_current_evens(count_current_evens),
    .count_minus15_evens(count_minus15_evens),
    .count_minus2_evens(count_minus2_evens),
    .count_minus16_evens(count_minus16_evens),
    .count_minus7_evens(count_minus7_evens),
    .count_current_odds(count_current_odds),
    .count_minus15_odds(count_minus15_odds),
    .count_minus2_odds(count_minus2_odds),
    .count_minus16_odds(count_minus16_odds),
    .count_minus7_odds(count_minus7_odds),
    .output_control(w_states),
    .finish(wcon_con_finish)
);

w_datapath
  w_data(
    .clock(clk),
    .reset(reset),
    .fsm_control(w_states),
    .fsm_count_current_evens(count_current_evens),
    .fsm_count_minus15_evens(count_minus15_evens),
    .fsm_count_minus2_evens(count_minus2_evens),
    .fsm_count_minus16_evens(count_minus16_evens),
    .fsm_count_minus7_evens(count_minus7_evens),
    .fsm_count_current_odds(count_current_odds),
    .fsm_count_minus15_odds(count_minus15_odds),
    .fsm_count_minus2_odds(count_minus2_odds),
    .fsm_count_minus16_odds(count_minus16_odds),
    .fsm_count_minus7_odds(count_minus7_odds),
    .M_1_to_W(M_1_out),
    .W_final(W_final)
);

compression_control
    comp_control(
    .clock(clk),
    .reset(reset),
    .start(con_comp_start),
    .control(compression_states),
    .fsm_counter(compression_counter),
    .finish(comp_con_finish)
);

compression_datapath
    comp_data(
    .reset(reset),
    .clock(clk),
    .control(compression_states),
    .iteration_count(compression_counter),
    .W_input(W_final),
    .K_input(saved_Ks),
    .H_input(saved_Hs),
    .H_output(final_H)
);

big_controller
    master_control(
    .clock(clk),
    .reset(reset),
    .go(xxx__dut__go),
    .msram_con_finish(msram_con_finish),
    .hsram_con_finish(hsram_con_finish),
    .ksram_con_finish(ksram_con_finish),
    .wcon_con_finish(wcon_con_finish),
    .comp_con_finish(comp_con_finish),
    .push_con_finish(push_con_finish),
    .con_msram_start(con_msram_start),
    .con_hsram_start(con_hsram_start),
    .con_ksram_start(con_ksram_start),
    .con_wcon_start(con_wcon_start),
    .con_comp_start(con_comp_start),
    .con_push_start(con_push_start),
    .dut_xxx_finish(dut__xxx__finish)
);

 sram_pull #(.SRAM_SIZE ( 64))
            k_store (
            .clock (clk),
            .reset (reset),
            .control (con_ksram_start),
            .msg_dut_data (kmem__dut__data),
            .write (dut__kmem__write),
            .enable (dut__kmem__enable),
            .address_output (dut__kmem__address),
            .finish (ksram_con_finish),
            .sram_saved (saved_Ks)
            );

  sram_pull2 #(.SRAM_SIZE (8))
            h_store (
            .clock (clk),
            .reset (reset),
            .control (con_hsram_start),
            .msg_dut_data (hmem__dut__data),
            .write (dut__hmem__write),
            .enable (dut__hmem__enable),
            .address_output (dut__hmem__address),
            .finish (hsram_con_finish),
            .sram_saved (saved_Hs)
            );

    M_build_datapath #(.MAX_MESSAGE_LENGTH (MAX_MESSAGE_LENGTH))
                    M_data (
                        .clock (clk),
                        .reset (reset),
                        .control (m_states),
                        .xxx__dut__msg__length(xxx__dut__msg_length),
                        .msg__dut__data(msg__dut__data),
                        .M_1 (M_1_out)
                    );
    M_build_control #(.MAX_MESSAGE_LENGTH (MAX_MESSAGE_LENGTH))
                M_control_path (
                    .clock (clk),
                    .reset (reset),
                    .go     (con_msram_start),
                    .xxx__dut__msg__length(xxx__dut__msg_length),
                    .control (m_states),
                    .enable (dut__msg__enable),
                    .write (dut__msg__write),
                    .address (dut__msg__address),
                    .finish (msram_con_finish)
                );
endmodule
module sram_push #(
            parameter SRAM_SIZE  = 8)
(
    input wire              clock,
    input wire              reset,
    input wire              control,
    input wire [31:0]       H_in [7:0],
    output reg              write,
    output reg              enable,
    output reg [2:0]        address_output,
    output reg              finish,
    output reg [31:0]       dut_dom_data
);

typedef enum reg [1:0] {
                SRAM_WAIT               = 2'b00,
                SRAM_CAPTURE            = 2'b01,
                SRAM_START_STORING      = 2'b10,
                SRAM_FINISH             = 2'b11} fsm_state_enum;
fsm_state_enum sram_fsm_state;
fsm_state_enum sram_fsm_next_state;

reg [31:0] H_capture[7:0];
reg internal_enable;
reg [2:0] address;

wire [31:0] data_output;

always @ (posedge clock) begin
    if (reset) begin
        dut_dom_data <= 'x;
    end
    else
        dut_dom_data <= data_output;
end

assign data_output = H_capture[address];

always @ (posedge clock) begin
    if (reset) begin
        address_output <= 'x;
    end
    else
        address_output <= address;
end

always @ (posedge clock) begin
        if (reset) begin
            H_capture[0] <= 0;
            H_capture[1] <= 0;
            H_capture[2] <= 0;
            H_capture[3] <= 0;
            H_capture[4] <= 0;
            H_capture[5] <= 0;
            H_capture[6] <= 0;
            H_capture[7] <= 0;
        end
        else
        casex(sram_fsm_state)
            SRAM_CAPTURE: begin
                H_capture[0] <= H_in[0];
                H_capture[1] <= H_in[1];
                H_capture[2] <= H_in[2];
                H_capture[3] <= H_in[3];
                H_capture[4] <= H_in[4];
                H_capture[5] <= H_in[5];
                H_capture[6] <= H_in[6];
                H_capture[7] <= H_in[7];
            end
            default: begin
                H_capture[0] <= H_capture[0];
                H_capture[1] <= H_capture[1];
                H_capture[2] <= H_capture[2];
                H_capture[3] <= H_capture[3];
                H_capture[4] <= H_capture[4];
                H_capture[5] <= H_capture[5];
                H_capture[6] <= H_capture[6];
                H_capture[7] <= H_capture[7];
            end
        endcase
end

//next state logic
always @ (*) begin
    casex(sram_fsm_state)
        SRAM_WAIT: begin
            if (control)
                sram_fsm_next_state = SRAM_CAPTURE;
            else
                sram_fsm_next_state = SRAM_WAIT;
        end
        SRAM_CAPTURE: begin
            sram_fsm_next_state = SRAM_START_STORING;
        end
        SRAM_START_STORING: begin
            if(address == (SRAM_SIZE-1))
                sram_fsm_next_state = SRAM_FINISH;
            else
                sram_fsm_next_state = SRAM_START_STORING;
        end
        SRAM_FINISH:
            sram_fsm_next_state = SRAM_WAIT;
        default:
            sram_fsm_next_state = SRAM_WAIT;
    endcase
end

always @ (posedge clock) begin
    if (reset) begin
        sram_fsm_state <= SRAM_WAIT;
        enable <= 0;
    end
    else begin
        sram_fsm_state <= sram_fsm_next_state;
        enable <= internal_enable;
    end
end

always @ (posedge clock) begin

    casex(sram_fsm_state)
        SRAM_WAIT: begin
            address<= 0;
	    write <= 0;
	    internal_enable <= 0;
	    finish <= 0;
        end
        SRAM_CAPTURE: begin
            address<= 0;
            internal_enable <= 1;
	    write <= 0;
	    finish <= 0;
        end
        SRAM_START_STORING: begin
            internal_enable <= 1;
            write <= 1;
            address <= address+1;
	    finish <= 0;
        end
        SRAM_FINISH: begin
            address <= 0;
            finish <= 1;
	    write <= 0;
	    internal_enable <= 0;
        end /*
        default : begin
            address <= 0;
	    write <= 0;
	    internal_enable <= 0;
	    finish <= 0;
        end */
    endcase
end


endmodule
module sram_pull #(
            parameter SRAM_SIZE  = 64)
(
    input wire              clock,
    input wire              reset,
    input wire              control,
    input wire [31:0]       msg_dut_data,
    output reg              write,
    output reg              enable,
    output reg [$clog2(SRAM_SIZE)-1:0]        address_output,
    output reg              finish,
    output reg [31:0]       sram_saved [SRAM_SIZE-1:0]
);

typedef enum reg [2:0] {
                SRAM_WAIT               = 3'b000,
                SRAM_FIRST_REQUEST      = 3'b001,
            //    SRAM_DELAY              = 3'b101,
                SRAM_START_STORING      = 3'b010,
                SRAM_FINAL_STORE        = 3'b011,
                SRAM_FINISH             = 3'b100} fsm_state_enum;
fsm_state_enum sram_fsm_state;
fsm_state_enum sram_fsm_next_state;

reg internal_enable;
reg [7:0] address_request;
reg [7:0] data_counter;
generate
for (genvar gvi = 0; gvi<SRAM_SIZE; gvi=gvi+1) begin : sram_saved_data
    always @ (posedge clock) begin
        if (reset) begin
            sram_saved[gvi] <= 0;
        end
        else if ((data_counter == gvi) && internal_enable) begin
            sram_saved[gvi] <= msg_dut_data;
        end
        else
            sram_saved[gvi] <= sram_saved[gvi];
    end
end
endgenerate

//next state logic
always @ (*) begin
    casex(sram_fsm_state)
        SRAM_WAIT: begin
            if (control)
                sram_fsm_next_state = SRAM_FIRST_REQUEST;
            else
                sram_fsm_next_state = SRAM_WAIT;
        end
        SRAM_FIRST_REQUEST: begin
            sram_fsm_next_state = SRAM_START_STORING;
        end
        SRAM_START_STORING: begin
            if(address_request == (SRAM_SIZE-1))
                sram_fsm_next_state = SRAM_FINAL_STORE;
            else
                sram_fsm_next_state = SRAM_START_STORING;
        end
        SRAM_FINAL_STORE: begin
            sram_fsm_next_state = SRAM_FINISH;
        end
        SRAM_FINISH:
            sram_fsm_next_state = SRAM_WAIT;
        default:
            sram_fsm_next_state = SRAM_WAIT;
    endcase
end

always @ (posedge clock) begin
    if (reset) begin
        sram_fsm_state <= SRAM_WAIT;
    end
    else begin
        sram_fsm_state <= sram_fsm_next_state;
        address_output <= address_request;
    end
end

always @ (posedge clock) begin
    write <= 0;
    enable <= 0;
    finish <= 0;
    internal_enable <= 0;
    if (reset) begin
        data_counter <= 0;
        address_request <=0;
    end
    else
    casex(sram_fsm_state)
        SRAM_WAIT: begin
            address_request <= 0;
            data_counter <= 0;
        end
        SRAM_FIRST_REQUEST: begin
            address_request <= address_request +1;
            enable <= 1;
            data_counter <= 0;
            internal_enable <= 1;
        end 
        SRAM_START_STORING: begin
            enable <= 1;
            data_counter <= data_counter +1;
            address_request <= address_request +1;
            internal_enable <= 1;
        end
        SRAM_FINAL_STORE: begin
            data_counter <= data_counter + 1;
            address_request <= 0;
            internal_enable <= 1;
        end
        SRAM_FINISH: begin
            address_request <= 0;
            data_counter <= 0;
            finish <= 1;
        end
        default : begin
            address_request <= 0;
            data_counter <= 0;
        end
    endcase
end


endmodule

module sram_pull2 #(
            parameter SRAM_SIZE  = 8)
(
    input wire              clock,
    input wire              reset,
    input wire              control,
    input wire [31:0]       msg_dut_data,
    output reg              write,
    output reg              enable,
    output reg [$clog2(SRAM_SIZE)-1:0]        address_output,
    output reg              finish,
    output reg [31:0]       sram_saved [SRAM_SIZE-1:0]
);

typedef enum reg [2:0] {
                SRAM_WAIT               = 3'b000,
                SRAM_FIRST_REQUEST      = 3'b001,
            //    SRAM_DELAY              = 3'b101,
                SRAM_START_STORING      = 3'b010,
                SRAM_FINAL_STORE        = 3'b011,
                SRAM_FINISH             = 3'b100} fsm_state_enum;
fsm_state_enum sram_fsm_state;
fsm_state_enum sram_fsm_next_state;

reg internal_enable;
reg [7:0] address_request;
reg [7:0] data_counter;
generate
for (genvar gvi = 0; gvi<SRAM_SIZE; gvi=gvi+1) begin : sram_saved_data
    always @ (posedge clock) begin
        if (reset) begin
            sram_saved[gvi] <= 0;
        end
        else if ((data_counter == gvi) && internal_enable) begin
            sram_saved[gvi] <= msg_dut_data;
        end
        else
            sram_saved[gvi] <= sram_saved[gvi];
    end
end
endgenerate

//next state logic
always @ (*) begin
    casex(sram_fsm_state)
        SRAM_WAIT: begin
            if (control)
                sram_fsm_next_state = SRAM_FIRST_REQUEST;
            else
                sram_fsm_next_state = SRAM_WAIT;
        end
        SRAM_FIRST_REQUEST: begin
            sram_fsm_next_state = SRAM_START_STORING;
        end
        SRAM_START_STORING: begin
            if(address_request == (SRAM_SIZE-1))
                sram_fsm_next_state = SRAM_FINAL_STORE;
            else
                sram_fsm_next_state = SRAM_START_STORING;
        end
        SRAM_FINAL_STORE: begin
            sram_fsm_next_state = SRAM_FINISH;
        end
        SRAM_FINISH:
            sram_fsm_next_state = SRAM_WAIT;
        default:
            sram_fsm_next_state = SRAM_WAIT;
    endcase
end

always @ (posedge clock) begin
    if (reset) begin
        sram_fsm_state <= SRAM_WAIT;
    end
    else begin
        sram_fsm_state <= sram_fsm_next_state;
        address_output <= address_request;
    end
end

always @ (posedge clock) begin
    write <= 0;
    enable <= 0;
    finish <= 0;
    internal_enable <= 0;
    if (reset) begin
        data_counter <= 0;
        address_request <=0;
    end
    else
    casex(sram_fsm_state)
        SRAM_WAIT: begin
            address_request <= 0;
            data_counter <= 0;
        end
        SRAM_FIRST_REQUEST: begin
            address_request <= address_request +1;
            enable <= 1;
            data_counter <= 0;
            internal_enable <= 1;
        end 
        SRAM_START_STORING: begin
            enable <= 1;
            data_counter <= data_counter +1;
            address_request <= address_request +1;
            internal_enable <= 1;
        end
        SRAM_FINAL_STORE: begin
            data_counter <= data_counter + 1;
            address_request <= 0;
            internal_enable <= 1;
        end
        SRAM_FINISH: begin
            address_request <= 0;
            data_counter <= 0;
            finish <= 1;
        end
        default : begin
            address_request <= 0;
            data_counter <= 0;
        end
    endcase
end


endmodule
module M_build_control #(
                  parameter MAX_MESSAGE_LENGTH  = 55)

(
    input wire                                      clock,
    input wire                                      reset,
    input wire                                      go,
    input wire  [ $clog2(MAX_MESSAGE_LENGTH):0]     xxx__dut__msg__length ,
    output reg  [2:0]                               control,
    output reg                                      enable,
    output reg                                      write,
    output reg  [5:0]                               address,
    output reg                                      finish
);

reg [$clog2(MAX_MESSAGE_LENGTH):0] msg_length;
reg [$clog2(MAX_MESSAGE_LENGTH):0] state_counter;
reg go_capture;

typedef enum reg [2:0] {
                M_WAIT      = 3'b000,
                M_PREP      = 3'b001,
                M_ADD       = 3'b010,
                M_FINAL     = 3'b011,
                M_DONE      = 3'b100} fsm_state_enum;
fsm_state_enum M_fsm_state;
fsm_state_enum M_fsm_next_state;

//register the input for go
always @ (posedge clock) begin
    if (reset)
        go_capture <= 0;
    else
        go_capture <= go; //should only be high for one clock like this
end

always @ (posedge clock) begin
    if (reset) begin
        M_fsm_state <= M_WAIT;
        control <= M_WAIT;
    end
    else begin
        M_fsm_state <= M_fsm_next_state;
        control <= M_fsm_next_state; //redundant?
    end
end
//FSM for the M module
//doesn't go back to a wait state so
//if the counter stays at 0 done will always be high

always @ (*) begin
    casex(M_fsm_state)
        M_WAIT: begin
            if(go_capture)
                M_fsm_next_state = M_PREP;
            else
                M_fsm_next_state = M_WAIT;
        end
        M_PREP: begin
            M_fsm_next_state = M_ADD;
        end
        M_ADD: begin
            if (state_counter == 0)
                M_fsm_next_state = M_FINAL;
            else
                M_fsm_next_state = M_ADD;
        end
	M_FINAL: begin
            M_fsm_next_state = M_DONE;
	end
        M_DONE: begin //maybe want to output with this state a finish signal
            M_fsm_next_state = M_WAIT;
        end
        default: M_fsm_next_state = M_WAIT;
    endcase
end

always @ (posedge clock) begin

    finish <= 0;
    write <= 0;
    enable <= 0;
    address <= 'x;
    msg_length <= xxx__dut__msg__length;

    if (reset) begin
        state_counter <= 0;
        msg_length <=0;
    end
    else
    casex(M_fsm_state)
        M_WAIT: begin
            if(go_capture) begin
                state_counter <= msg_length -1;
            end
            else begin
                state_counter <= 0;
            end
        end
        M_PREP: begin
            state_counter<= state_counter -1;
            enable <= 1;
            address <= state_counter;
        end
        M_ADD: begin
            enable <= 1;
            address <= state_counter;
            state_counter<= state_counter -1;
        end
    	M_FINAL:
	    state_counter <= state_counter;
        M_DONE: begin
            state_counter <= state_counter;
            finish <= 1;
        end
        default:
            state_counter <= state_counter;
    endcase
end



endmodule
module M_build_datapath #(
                  parameter MAX_MESSAGE_LENGTH  = 55)

(
    input   wire                                    clock,
    input   wire                                    reset,
    input   wire  [2:0]                             control,
    input   wire  [ $clog2(MAX_MESSAGE_LENGTH):0]   xxx__dut__msg__length ,
    input   wire  [7:0]                             msg__dut__data     ,  // read each letter
    output  reg   [31:0]                            M_1 [15:0]
    );

reg [511:0]     M_block;
reg [511:0]     message_prepare;
reg [447:0]     new_message;
wire [31:0]      M_intermediate [15:0];

generate
for (genvar gvi =0; gvi<16; gvi=gvi+1) begin : M1_blocks
    always @ (posedge clock) begin
        if (reset)
            M_1 [gvi] <= 0;
        else
            M_1 [gvi] <= M_intermediate [gvi];
    end
end
endgenerate

assign M_intermediate[0] = M_block [511:480];
assign M_intermediate[1] = M_block [479:448];
assign M_intermediate[2] = M_block [447:416];
assign M_intermediate[3] = M_block [415:382];
assign M_intermediate[4] = M_block [383:352];
assign M_intermediate[5] = M_block [351:320];
assign M_intermediate[6] = M_block [319:288];
assign M_intermediate[7] = M_block [287:256];
assign M_intermediate[8] = M_block [255:224];
assign M_intermediate[9] = M_block [223:192];
assign M_intermediate[10] = M_block [191:160];
assign M_intermediate[11] = M_block [159:128];
assign M_intermediate[12] = M_block [127:96];
assign M_intermediate[13] = M_block [95:64];
assign M_intermediate[14] = M_block [63:32];
assign M_intermediate[15] = M_block [31:0];

always @ (posedge clock) begin
    if (reset) begin
        M_block <= 0;
        //reset M_1 in a generate block that also handles the final copy
    end
    else

    casex (control)
    3'b000: M_block <= M_block; //do nothing
    3'b001: begin //prepare the message
       M_block <= message_prepare;
    end
    3'b01x: begin
        M_block [511:64] <= new_message;
    end
    default: M_block <= M_block;
    endcase
end
//combinational logic for initializing the M_block
//should probably be the gated input from control
always @ (*) begin
    if (control == 3'b001) begin
        message_prepare = 0;
        message_prepare [511] = 1;
        message_prepare [63:0] = xxx__dut__msg__length << 3; //not using a latched value
    end
    else
        message_prepare = 0;
end

always @ (*) begin
    if (control == 3'b010) begin
        new_message  = M_block [511:64] >> 8;
        new_message [447:440] = msg__dut__data;
    end
    else if (control == 3'b011) begin
        new_message  = M_block [511:64] >> 8;
        new_message [447:440] = msg__dut__data;
    end
    else
        new_message = M_block [511:64];
end
endmodule
module  compression_datapath(
    input wire          reset,
    input wire          clock,
    input wire  [2:0]   control,
    input wire  [5:0]   iteration_count,
    input wire  [31:0]  W_input  [63:0],
    input wire  [31:0]  K_input  [63:0],
    input wire  [31:0]  H_input  [7:0],
    output reg  [31:0]  H_output [7:0]
);


reg  [31:0]  a,b,c,d,e,f,g,h;
reg  [31:0]  H_original  [7:0];

wire [31:0]   a_next;
wire [31:0]   b_next;
wire [31:0]   c_next;
wire [31:0]   d_next;
wire [31:0]   e_next;
wire [31:0]   f_next;
wire [31:0]   g_next;
wire [31:0]   h_next;
parameter    COMPRESSION_WAIT      = 3'b000;
parameter    COMPRESSION_CAPTURE   = 3'b001;
parameter    COMPRESSION_PROCESS   = 3'b010;
parameter    COMPRESSION_TRANSFER  = 3'b011;
parameter    COMPRESSION_FINAL     = 3'b100;

always @ (posedge clock) begin
    if(reset) begin
        H_original[0] <= 0;
        H_original[1] <= 0;
        H_original[2] <= 0;
        H_original[3] <= 0;
        H_original[4] <= 0;
        H_original[5] <= 0;
        H_original[6] <= 0;
        H_original[7] <= 0;
    end
else
    casex(control)
    COMPRESSION_CAPTURE: begin
        H_original[0] <= H_input[0];
        H_original[1] <= H_input[1];
        H_original[2] <= H_input[2];
        H_original[3] <= H_input[3];
        H_original[4] <= H_input[4];
        H_original[5] <= H_input[5];
        H_original[6] <= H_input[6];
        H_original[7] <= H_input[7];
    end
    default:  begin
        H_original[0] <= H_original[0];
        H_original[1] <= H_original[1];
        H_original[2] <= H_original[2];
        H_original[3] <= H_original[3];
        H_original[4] <= H_original[4];
        H_original[5] <= H_original[5];
        H_original[6] <= H_original[6];
        H_original[7] <= H_original[7];
    end 
    endcase
end

always @ (posedge clock) begin
    if(reset) begin
        H_output[0]  <= 0;
        H_output[1]  <= 0;
        H_output[2]  <= 0;
        H_output[3]  <= 0;
        H_output[4]  <= 0;
        H_output[5]  <= 0;
        H_output[6]  <= 0;
        H_output[7]  <= 0;
    end
    else
    casex(control)
    COMPRESSION_TRANSFER: begin
        H_output[0]  <= H_original[0] + a;
        H_output[1]  <= H_original[1] + b;
        H_output[2]  <= H_original[2] + c;
        H_output[3]  <= H_original[3] + d;
        H_output[4]  <= H_original[4] + e;
        H_output[5]  <= H_original[5] + f;
        H_output[6]  <= H_original[6] + g;
        H_output[7]  <= H_original[7] + h;
    end
    default: begin
        H_output[0]  <= H_output[0];
        H_output[1]  <= H_output[1];
        H_output[2]  <= H_output[2];
        H_output[3]  <= H_output[3];
        H_output[4]  <= H_output[4];
        H_output[5]  <= H_output[5];
        H_output[6]  <= H_output[6];
        H_output[7]  <= H_output[7];
    end
    endcase
end

always @ (posedge clock) begin
    if(reset) begin
        a <= 0;
        b <= 0;
        c <= 0;
        d <= 0;
        e <= 0;
        f <= 0;
        g <= 0;
        h <= 0;
    end
    else
    casex(control)
    COMPRESSION_CAPTURE: begin
        a <= H_input[0];
        b <= H_input[1];
        c <= H_input[2];
        d <= H_input[3];
        e <= H_input[4];
        f <= H_input[5];
        g <= H_input[6];
        h <= H_input[7];
    end
    COMPRESSION_PROCESS: begin
        a <= a_next;
        b <= b_next;
        c <= c_next;
        d <= d_next;
        e <= e_next;
        f <= f_next;
        g <= g_next;
        h <= h_next;
    end
    COMPRESSION_TRANSFER: begin
        a <= a;
        b <= b;
        c <= c;
        d <= d;
        e <= e;
        f <= f;
        g <= g;
        h <= h;
    end
    default: begin
        a <= a;
        b <= b;
        c <= c;
        d <= d;
        e <= e;
        f <= f;
        g <= g;
        h <= h;
    end
    endcase
end

//combinational logic for a-h
reg [31:0]   right_rotate6;
reg [31:0]   right_rotate11;
reg [31:0]   right_rotate25;

reg [31:0]   right_rotate2;
reg [31:0]   right_rotate13;
reg [31:0]   right_rotate22;

always @ (*) begin
    right_rotate6 = {e,e} >> 6;
    right_rotate11 = {e,e} >> 11;
    right_rotate25 = {e,e} >> 25;

    right_rotate2 = {a,a} >> 2;
    right_rotate13 = {a,a} >> 13;
    right_rotate22 = {a,a} >> 22;
end

wire [31:0]   S1    ;
wire [31:0]   S0    ;
wire [31:0]   maj   ;
wire [31:0]   ch    ;
wire [31:0]   wk_sum;
wire [31:0]   temp1 ;
wire [31:0]   temp2 ;

assign S1       = (right_rotate6 ^ right_rotate11) ^ right_rotate25;
assign S0       = (right_rotate2 ^ right_rotate13) ^ right_rotate22;
assign maj      = (a & b) ^ (a & c) ^ (b & c);
assign ch       = (e & f) ^ ((~e) & g);
assign wk_sum   = K_input[iteration_count] +  W_input[iteration_count];
assign temp1    = wk_sum + (S1 + ch) + h;
assign temp2    = maj + S0;

assign a_next   = temp1 + temp2;
assign b_next   = a;
assign c_next   = b;
assign d_next   = c;
assign e_next   = d + temp1;
assign f_next   = e;
assign g_next   = f;
assign h_next   = g;

endmodule
module compression_control
(
    input wire          clock,
    input wire          reset,
    input wire          start,
    output reg  [2:0]   control,
    output reg  [5:0]   fsm_counter,
    output reg         finish
);

typedef enum reg [2:0] {
    COMPRESSION_WAIT      = 3'b000,
    COMPRESSION_CAPTURE   = 3'b001,
    COMPRESSION_PROCESS   = 3'b010,
    COMPRESSION_TRANSFER  = 3'b011,
    COMPRESSION_FINAL     = 3'b100} compression_state_enum;

compression_state_enum c_state;
compression_state_enum c_state_next;
//
//not sure why this is here so maybe I meant to latch this out
//reg [5:0] fsm_counter;

always @(posedge clock) begin
    if (reset) begin
        control <= COMPRESSION_WAIT;
        c_state <= COMPRESSION_WAIT;
    end
    else begin
        control <= c_state_next;
        c_state <= c_state_next;
    end
end

always @(*) begin
    finish = 0;
    casex(c_state)
    COMPRESSION_WAIT: begin
        if(start)
            c_state_next  = COMPRESSION_CAPTURE;
        else
            c_state_next = COMPRESSION_WAIT;
    end
    COMPRESSION_CAPTURE: begin
        c_state_next = COMPRESSION_PROCESS;
    end
    COMPRESSION_PROCESS: begin
        if(fsm_counter == 63)
            c_state_next = COMPRESSION_TRANSFER;
        else
            c_state_next = COMPRESSION_PROCESS;
    end
    COMPRESSION_TRANSFER: begin
        c_state_next = COMPRESSION_FINAL;
    end
    COMPRESSION_FINAL: begin
        c_state_next = COMPRESSION_WAIT;
        finish = 1;
    end
    default:
        c_state_next = COMPRESSION_WAIT;
    endcase
end

always @(posedge clock) begin
    if(reset)
        fsm_counter <= 0;
    else
    casex(c_state)
    COMPRESSION_CAPTURE: begin
        fsm_counter <= 0;
    end
    COMPRESSION_PROCESS: begin
        fsm_counter <= fsm_counter + 1;
    end
    default:
        fsm_counter <= fsm_counter;
    endcase
end

endmodule
module big_controller
(
    input   wire    clock,
    input   wire    reset,
    input   wire    go,
    input   wire    msram_con_finish,
    input   wire    hsram_con_finish,
    input   wire    ksram_con_finish,
    input   wire    wcon_con_finish,
    input   wire    comp_con_finish,
    input   wire    push_con_finish,
    output  reg     con_msram_start,
    output  reg     con_hsram_start,
    output  reg     con_ksram_start,
    output  reg     con_wcon_start,
    output  reg     con_comp_start,
    output  reg     con_push_start,
    output  reg     dut_xxx_finish
);

typedef enum reg [3:0] {
    WAIT        = 4'b0000,
    LOAD_SRAMS  = 4'b0001,
    LOAD_SRAMS2  = 4'b1001,
    W_OPERATION = 4'b0010,
    W_OPERATION2 = 4'b0011,
    COMPRESSION = 4'b0100,
    COMPRESSION2 = 4'b0101,
    PUSH_SRAM   = 4'b0110,
    PUSH_SRAM2   = 4'b111,
    FINISH      = 4'b1000} b_fsm_state_enum;

b_fsm_state_enum B_fsm_state;
b_fsm_state_enum B_fsm_state_next;

reg go_capture;

reg w_complete, srams_loaded,compression_complete, push_complete;

always @(posedge clock) begin
    if (reset) begin
        B_fsm_state <= WAIT;
        go_capture <= 0;
        w_complete <= 0;
        srams_loaded <= 0;
        compression_complete <= 0;
        push_complete <= 0;
    end
    else begin
        go_capture <= go;
        B_fsm_state <= B_fsm_state_next;
        w_complete <= wcon_con_finish;
        srams_loaded <= ksram_con_finish;
        compression_complete <= comp_con_finish;
        push_complete <= push_con_finish;
    end
end

always @(*) begin
    casex(B_fsm_state)
        WAIT: begin
            if (go_capture)
                B_fsm_state_next = LOAD_SRAMS;
            else
                B_fsm_state_next = WAIT;
        end
        LOAD_SRAMS: begin
                B_fsm_state_next = LOAD_SRAMS2;
        end
        LOAD_SRAMS2: begin
            if  (srams_loaded)
                B_fsm_state_next = W_OPERATION;
            else
                B_fsm_state_next = LOAD_SRAMS2;
        end
        W_OPERATION: begin
                B_fsm_state_next = W_OPERATION2;
        end
        W_OPERATION2: begin
            if  (w_complete)
                B_fsm_state_next = COMPRESSION;
            else
                B_fsm_state_next = W_OPERATION2;
        end
        COMPRESSION: begin
              B_fsm_state_next = COMPRESSION2;
        end
        COMPRESSION2: begin
            if(compression_complete)
                B_fsm_state_next = PUSH_SRAM;
            else
                B_fsm_state_next = COMPRESSION2;
        end
        PUSH_SRAM: begin
                B_fsm_state_next = PUSH_SRAM2;
        end
        PUSH_SRAM2: begin
            if (push_complete)
                B_fsm_state_next = FINISH;
            else
                B_fsm_state_next = PUSH_SRAM2;
        end
        FINISH: begin
             B_fsm_state_next = WAIT;
/*            if(go_capture)
                B_fsm_state_next = LOAD_SRAMS;
            else
                B_fsm_state_next = FINISH;
*/
        end
        default:
            B_fsm_state_next = WAIT;
    endcase
end

always @ (posedge clock) begin
    if (reset) begin
        con_msram_start <= 0;
        con_hsram_start <= 0;
        con_ksram_start <= 0;
        con_wcon_start  <= 0;
        con_comp_start <= 0;
        con_push_start  <= 0;
        dut_xxx_finish   <= 0;
    end
    else
        casex(B_fsm_state)
            WAIT: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 0;
                con_comp_start <= 0;
                con_push_start  <= 0;
                dut_xxx_finish   <= 0;
            end
            LOAD_SRAMS: begin
                con_msram_start <= 1;
                con_hsram_start <= 1;
                con_ksram_start <= 1;
                con_wcon_start  <= 0;
                con_comp_start <= 0;
                con_push_start  <= 0;
                dut_xxx_finish   <= 0;
            end
            LOAD_SRAMS2: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 0;
                con_comp_start <= 0;
                con_push_start  <= 0;
                dut_xxx_finish   <= 0;
            end
            W_OPERATION: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 1;
                con_comp_start <= 0;
                con_push_start  <= 0;
                dut_xxx_finish   <= 0;
            end
            W_OPERATION2: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 0;
                con_comp_start <= 0;
                con_push_start  <= 0;
                dut_xxx_finish   <= 0;
            end
            COMPRESSION: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 0;
                con_comp_start  <= 1;
                con_push_start  <= 0;
                dut_xxx_finish   <= 0;
            end
            COMPRESSION2: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 0;
                con_comp_start  <= 0;
                con_push_start  <= 0;
                dut_xxx_finish   <= 0;
            end
            PUSH_SRAM: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 0;
                con_comp_start  <= 0;
                con_push_start  <= 1;
                dut_xxx_finish   <= 0;
            end
            PUSH_SRAM2: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 0;
                con_comp_start  <= 0;
                con_push_start  <= 0;
                dut_xxx_finish   <= 0;
            end
            FINISH: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 0;
                con_comp_start  <= 0;
                con_push_start  <= 0;
                dut_xxx_finish   <= 1;
            end
            default: begin
                con_msram_start <= 0;
                con_hsram_start <= 0;
                con_ksram_start <= 0;
                con_wcon_start  <= 0;
                con_comp_start  <= 0;
                con_push_start  <= 0;
                dut_xxx_finish   <= 0;
            end
        endcase
end




endmodule
module w_control
(
    input wire          clock,
    input wire          reset,
    input wire          start,
    output reg  [5:0]   count_current_evens,
    output reg  [5:0]   count_minus15_evens,
    output reg  [5:0]   count_minus2_evens,
    output reg  [5:0]   count_minus16_evens,
    output reg  [5:0]   count_minus7_evens,
    output reg  [5:0]   count_current_odds,
    output reg  [5:0]   count_minus15_odds,
    output reg  [5:0]   count_minus2_odds,
    output reg  [5:0]   count_minus16_odds,
    output reg  [5:0]   count_minus7_odds,
    output reg  [2:0]   output_control,
    output reg          finish
);

typedef enum reg [2:0] {
    W_WAIT      = 3'b000,
    W_CAPTURE   = 3'b001,
    W_FIRST16   = 3'b010,
    W_PROCESS   = 3'b011,
    W_FINAL     = 3'b100} w_fsm_state_enum;

w_fsm_state_enum W_fsm_state;
w_fsm_state_enum W_fsm_state_next;

always @(posedge clock) begin
    if (reset) begin
        W_fsm_state <= W_WAIT;
        output_control <= W_WAIT;
    end
    else begin
        W_fsm_state <= W_fsm_state_next;
        output_control <= W_fsm_state_next;
    end
end

always @(*) begin
    finish = 0;
    casex(W_fsm_state)
    W_WAIT: begin
        if(start)
            W_fsm_state_next = W_CAPTURE;
        else
            W_fsm_state_next = W_WAIT;
    end
    W_CAPTURE: begin
        W_fsm_state_next = W_FIRST16;
    end
    W_FIRST16: begin
        W_fsm_state_next = W_PROCESS;
    end
    W_PROCESS: begin
        //needs a conditional based on a counter
        if(count_current_odds ==63)
            W_fsm_state_next = W_FINAL;
        else
            W_fsm_state_next = W_PROCESS;
    end
    W_FINAL: begin
        finish = 1;
        W_fsm_state_next = W_WAIT;
    end
    default:
        W_fsm_state_next = W_WAIT;
    endcase
end

always @ (posedge clock) begin
/*    if(reset) begin
        count_current_evens <= 0;
        count_minus15_evens <= 0;
        count_minus2_evens <= 0;
        count_minus16_evens <= 0;
        count_minus7_evens <= 0;
        count_current_odds <= 0;
        count_minus15_odds <= 0;
        count_minus2_odds <= 0;
        count_minus16_odds <= 0;
        count_minus7_odds <= 0;
    end
    else */
    casex(W_fsm_state)
    W_WAIT: begin
        count_current_evens <= 0;
        count_minus15_evens <= 0;
        count_minus2_evens <= 0;
        count_minus16_evens <= 0;
        count_minus7_evens <= 0;
        count_current_odds <= 0;
        count_minus15_odds <= 0;
        count_minus2_odds <= 0;
        count_minus16_odds <= 0;
        count_minus7_odds <= 0;
    end
    W_CAPTURE: begin
        count_current_evens <= 0;
        count_minus15_evens <= 0;
        count_minus2_evens <= 0;
        count_minus16_evens <= 0;
        count_minus7_evens <= 0;
        count_current_odds <= 0;
        count_minus15_odds <= 0;
        count_minus2_odds <= 0;
        count_minus16_odds <= 0;
        count_minus7_odds <= 0;
    end
    W_FIRST16: begin
        count_current_evens <= 16;
        count_minus15_evens <= 1;
        count_minus2_evens <= 14;
        count_minus16_evens <= 0;
        count_minus7_evens <= 9;
        count_current_odds <= 17;
        count_minus15_odds <= 2;
        count_minus2_odds <= 15;
        count_minus16_odds <= 1;
        count_minus7_odds <= 10;
    end
    W_PROCESS: begin
        count_current_evens <=count_current_evens+ 1;
        count_minus15_evens <=count_minus15_evens+ 1;
        count_minus2_evens <= count_minus2_evens +1;
        count_minus16_evens <=count_minus16_evens +1;
        count_minus7_evens <= count_minus7_evens +1;
        count_current_odds <= count_current_odds +1;
        count_minus15_odds <= count_minus15_odds +1;
        count_minus2_odds <= count_minus2_odds +1;
        count_minus16_odds <= count_minus16_odds + 1;
        count_minus7_odds <= count_minus7_odds +1;
    end
    W_FINAL: begin
        count_current_evens <= 0;
        count_minus15_evens <= 0;
        count_minus2_evens <= 0;
        count_minus16_evens <= 0;
        count_minus7_evens <= 0;
        count_current_odds <= 0;
        count_minus15_odds <= 0;
        count_minus2_odds <= 0;
        count_minus16_odds <= 0;
        count_minus7_odds <= 0;
    end
    default: begin
        count_current_evens <= 0;
        count_minus15_evens <= 0;
        count_minus2_evens <= 0;
        count_minus16_evens <= 0;
        count_minus7_evens <= 0;
        count_current_odds <= 0;
        count_minus15_odds <= 0;
        count_minus2_odds <= 0;
        count_minus16_odds <= 0;
        count_minus7_odds <= 0;
    end
    endcase

end

endmodule
module w_datapath
(
    input wire          clock,
    input wire          reset,
    input wire  [2:0]   fsm_control, //this might just need 2 bits
    input wire  [5:0]   fsm_count_current_evens,
    input wire  [5:0]   fsm_count_minus15_evens,
    input wire  [5:0]   fsm_count_minus2_evens,
    input wire  [5:0]   fsm_count_minus16_evens,
    input wire  [5:0]   fsm_count_minus7_evens,
    input wire  [5:0]   fsm_count_current_odds,
    input wire  [5:0]   fsm_count_minus15_odds,
    input wire  [5:0]   fsm_count_minus2_odds,
    input wire  [5:0]   fsm_count_minus16_odds,
    input wire  [5:0]   fsm_count_minus7_odds,
    input wire  [31:0]  M_1_to_W [15:0],
    output reg  [31:0]  W_final  [63:0]
);

reg [31:0] W_intermediate [63:0];
reg [31:0] W_in [15:0];
reg [31:0] final_sum_evens;
reg [31:0] final_sum_odds;

wire [31:0] W_first16 [63:0];


generate
for (genvar gvi2=0; gvi2<16; gvi2=gvi2+1) begin : W_capture
    always @ (posedge clock) begin
        if (reset)
            W_in[gvi2] <= 0;
        else begin
            if(fsm_control == 3'b001)
                W_in[gvi2] <= M_1_to_W[gvi2];
            else
                W_in[gvi2] <= W_in[gvi2];
        end
    end
end
endgenerate

generate
for (genvar gvi=0; gvi<64; gvi=gvi+1) begin : W_output
    always @ (posedge clock) begin
        if (reset)
            W_final[gvi] <= 0; //or x?
        else
            W_final[gvi] <= W_intermediate[gvi];
    end
end
endgenerate

//write enable for W_intermediate
reg [63:0] write_enable_evens;

always @(*) begin
    casex(fsm_count_current_evens)
    6'd0:
        write_enable_evens = 64'h0000000000000000;
    6'd1:
        write_enable_evens = 64'h0000000000000000;
    6'd2:
        write_enable_evens = 64'h0000000000000000;
    6'd3:
        write_enable_evens = 64'h0000000000000000;
    6'd4:
        write_enable_evens = 64'h0000000000000000;
    6'd5:
        write_enable_evens = 64'h0000000000000000;
    6'd6:
        write_enable_evens = 64'h0000000000000000;
    6'd7:
        write_enable_evens = 64'h0000000000000000;
    6'd8:
        write_enable_evens = 64'h0000000000000000;
    6'd9:
        write_enable_evens = 64'h0000000000000000;
    6'd10:
        write_enable_evens = 64'h0000000000000000;
    6'd11:
        write_enable_evens = 64'h0000000000000000;
    6'd12:
        write_enable_evens = 64'h0000000000000000;
    6'd13:
        write_enable_evens = 64'h0000000000000000;
    6'd14:
        write_enable_evens = 64'h0000000000000000;
    6'd15:
        write_enable_evens = 64'h0000000000000000;
    6'd16:
        write_enable_evens = 64'h0000000000010000;
    6'd17:
        write_enable_evens = 64'h0000000000020000;
    6'd18:
        write_enable_evens = 64'h0000000000040000;
    6'd19:
        write_enable_evens = 64'h0000000000080000;
    6'd20:
        write_enable_evens = 64'h0000000000100000;
    6'd21:
        write_enable_evens = 64'h0000000000200000;
    6'd22:
        write_enable_evens = 64'h0000000000400000;
    6'd23:
        write_enable_evens = 64'h0000000000800000;
    6'd24:
        write_enable_evens = 64'h0000000001000000;
    6'd25:
        write_enable_evens = 64'h0000000002000000;
    6'd26:
        write_enable_evens = 64'h0000000004000000;
    6'd27:
        write_enable_evens = 64'h0000000008000000;
    6'd28:
        write_enable_evens = 64'h0000000010000000;
    6'd29:
        write_enable_evens = 64'h0000000020000000;
    6'd30:
        write_enable_evens = 64'h0000000040000000;
    6'd31:
        write_enable_evens = 64'h0000000080000000;
    6'd32:
        write_enable_evens = 64'h0000000100000000;
    6'd33:
        write_enable_evens = 64'h0000000200000000;
    6'd34:
        write_enable_evens = 64'h0000000400000000;
    6'd35:
        write_enable_evens = 64'h0000000800000000;
    6'd36:
        write_enable_evens = 64'h0000001000000000;
    6'd37:
        write_enable_evens = 64'h0000002000000000;
    6'd38:
        write_enable_evens = 64'h0000004000000000;
    6'd39:
        write_enable_evens = 64'h0000008000000000;
    6'd40:
        write_enable_evens = 64'h0000010000000000;
    6'd41:
        write_enable_evens = 64'h0000020000000000;
    6'd42:
        write_enable_evens = 64'h0000040000000000;
    6'd43:
        write_enable_evens = 64'h0000080000000000;
    6'd44:
        write_enable_evens = 64'h0000100000000000;
    6'd45:
        write_enable_evens = 64'h0000200000000000;
    6'd46:
        write_enable_evens = 64'h0000400000000000;
    6'd47:
        write_enable_evens = 64'h0000800000000000;
    6'd48:
        write_enable_evens = 64'h0001000000000000;
    6'd49:
        write_enable_evens = 64'h0002000000000000;
    6'd50:
        write_enable_evens = 64'h0004000000000000;
    6'd51:
        write_enable_evens = 64'h0008000000000000;
    6'd52:
        write_enable_evens = 64'h0010000000000000;
    6'd53:
        write_enable_evens = 64'h0020000000000000;
    6'd54:
        write_enable_evens = 64'h0040000000000000;
    6'd55:
        write_enable_evens = 64'h0080000000000000;
    6'd56:
        write_enable_evens = 64'h0100000000000000;
    6'd57:
        write_enable_evens = 64'h0200000000000000;
    6'd58:
        write_enable_evens = 64'h0400000000000000;
    6'd59:
        write_enable_evens = 64'h0800000000000000;
    6'd60:
        write_enable_evens = 64'h1000000000000000;
    6'd61:
        write_enable_evens = 64'h2000000000000000;
    6'd62:
        write_enable_evens = 64'h4000000000000000;
    6'd63:
        write_enable_evens = 64'h8000000000000000;
    endcase
end
//This generate block handles the even array elements
//including the loopback due to the dependency on w-2
generate
for (genvar gvi3=0; gvi3<64; gvi3=gvi3+2) begin : W_process_evens
    always @ (posedge clock) begin
        if (reset)
            W_intermediate[gvi3] <= 'x; //or x?
        else
        casex(fsm_control)
        3'b000: begin                       //do nothing, W is being captured
            W_intermediate[gvi3] <= W_intermediate[gvi3];
        end
        3'b010: begin   //assign the first 16 values of W
            W_intermediate[gvi3] <= W_first16[gvi3];
        end
        3'b011: begin
            if(write_enable_evens[gvi3])
                W_intermediate[gvi3] <= final_sum_evens;
            else
                W_intermediate[gvi3] <= W_intermediate[gvi3];
        end
        default:
            W_intermediate[gvi3] <= W_intermediate[gvi3];
        endcase
    end
end
endgenerate

reg [63:0] write_enable_odds;

always @(*) begin
    casex(fsm_count_current_odds)
    6'd0:
       write_enable_odds = 64'h0000000000000000;
    6'd1:
       write_enable_odds = 64'h0000000000000000;
    6'd2:
       write_enable_odds = 64'h0000000000000000;
    6'd3:
       write_enable_odds = 64'h0000000000000000;
    6'd4:
       write_enable_odds = 64'h0000000000000000;
    6'd5:
       write_enable_odds = 64'h0000000000000000;
    6'd6:
       write_enable_odds = 64'h0000000000000000;
    6'd7:
       write_enable_odds = 64'h0000000000000000;
    6'd8:
       write_enable_odds = 64'h0000000000000000;
    6'd9:
       write_enable_odds = 64'h0000000000000000;
    6'd10:
       write_enable_odds = 64'h0000000000000000;
    6'd11:
       write_enable_odds = 64'h0000000000000000;
    6'd12:
       write_enable_odds = 64'h0000000000000000;
    6'd13:
       write_enable_odds = 64'h0000000000000000;
    6'd14:
       write_enable_odds = 64'h0000000000000000;
    6'd15:
       write_enable_odds = 64'h0000000000000000;
    6'd16:
       write_enable_odds = 64'h0000000000010000;
    6'd17:
       write_enable_odds = 64'h0000000000020000;
    6'd18:
       write_enable_odds = 64'h0000000000040000;
    6'd19:
       write_enable_odds = 64'h0000000000080000;
    6'd20:
       write_enable_odds = 64'h0000000000100000;
    6'd21:
       write_enable_odds = 64'h0000000000200000;
    6'd22:
       write_enable_odds = 64'h0000000000400000;
    6'd23:
       write_enable_odds = 64'h0000000000800000;
    6'd24:
       write_enable_odds = 64'h0000000001000000;
    6'd25:
       write_enable_odds = 64'h0000000002000000;
    6'd26:
       write_enable_odds = 64'h0000000004000000;
    6'd27:
       write_enable_odds = 64'h0000000008000000;
    6'd28:
       write_enable_odds = 64'h0000000010000000;
    6'd29:
       write_enable_odds = 64'h0000000020000000;
    6'd30:
       write_enable_odds = 64'h0000000040000000;
    6'd31:
       write_enable_odds = 64'h0000000080000000;
    6'd32:
       write_enable_odds = 64'h0000000100000000;
    6'd33:
       write_enable_odds = 64'h0000000200000000;
    6'd34:
       write_enable_odds = 64'h0000000400000000;
    6'd35:
       write_enable_odds = 64'h0000000800000000;
    6'd36:
       write_enable_odds = 64'h0000001000000000;
    6'd37:
       write_enable_odds = 64'h0000002000000000;
    6'd38:
       write_enable_odds = 64'h0000004000000000;
    6'd39:
       write_enable_odds = 64'h0000008000000000;
    6'd40:
       write_enable_odds = 64'h0000010000000000;
    6'd41:
       write_enable_odds = 64'h0000020000000000;
    6'd42:
       write_enable_odds = 64'h0000040000000000;
    6'd43:
       write_enable_odds = 64'h0000080000000000;
    6'd44:
       write_enable_odds = 64'h0000100000000000;
    6'd45:
       write_enable_odds = 64'h0000200000000000;
    6'd46:
       write_enable_odds = 64'h0000400000000000;
    6'd47:
       write_enable_odds = 64'h0000800000000000;
    6'd48:
       write_enable_odds = 64'h0001000000000000;
    6'd49:
       write_enable_odds = 64'h0002000000000000;
    6'd50:
       write_enable_odds = 64'h0004000000000000;
    6'd51:
       write_enable_odds = 64'h0008000000000000;
    6'd52:
       write_enable_odds = 64'h0010000000000000;
    6'd53:
       write_enable_odds = 64'h0020000000000000;
    6'd54:
       write_enable_odds = 64'h0040000000000000;
    6'd55:
       write_enable_odds = 64'h0080000000000000;
    6'd56:
       write_enable_odds = 64'h0100000000000000;
    6'd57:
       write_enable_odds = 64'h0200000000000000;
    6'd58:
       write_enable_odds = 64'h0400000000000000;
    6'd59:
       write_enable_odds = 64'h0800000000000000;
    6'd60:
       write_enable_odds = 64'h1000000000000000;
    6'd61:
       write_enable_odds = 64'h2000000000000000;
    6'd62:
       write_enable_odds = 64'h4000000000000000;
    6'd63:
       write_enable_odds = 64'h8000000000000000;
    endcase
end
//This generate block handles the even array elements
//including the loopback due to the dependency on w-2
generate
for (genvar gvi4=1; gvi4<64; gvi4=gvi4+2) begin : W_process_odds
    always @ (posedge clock) begin
        if (reset)
            W_intermediate[gvi4] <= 0; //or x?
        else
        casex(fsm_control)
        3'b000: begin                       //do nothing, W is being captured
            W_intermediate[gvi4] <= W_intermediate[gvi4];
        end
        3'b010: begin   //assign the first 16 values of W
            W_intermediate[gvi4] <= W_first16[gvi4];
        end
        3'b011: begin
            if(write_enable_odds[gvi4])
                W_intermediate[gvi4] <= final_sum_odds;
            else
                W_intermediate[gvi4] <= W_intermediate[gvi4];
        end
        default:
            W_intermediate[gvi4] <= W_intermediate[gvi4];
        endcase
    end
end
endgenerate


//combinational logic in the critical path
reg [31:0] minus15_evens_selected_mem;
reg [31:0] minus2_evens_selected_mem;
reg [31:0] minus16_evens_selected_mem;
reg [31:0] minus7_evens_selected_mem;
reg [31:0] minus15_odds_selected_mem;
reg [31:0] minus2_odds_selected_mem;
reg [31:0] minus16_odds_selected_mem;
reg [31:0] minus7_odds_selected_mem;

reg [31:0] right_rotate_7_evens;
reg [31:0] right_rotate_7_odds;
reg [31:0] right_rotate_18_evens;
reg [31:0] right_rotate_18_odds;
reg [31:0] right_shift_3_evens;
reg [31:0] right_shift_3_odds;

reg [31:0] right_rotate_17_evens;
reg [31:0] right_rotate_17_odds;
reg [31:0] right_rotate_19_evens;
reg [31:0] right_rotate_19_odds;
reg [31:0] right_shift_10_evens;
reg [31:0] right_shift_10_odds;

reg [31:0] xor_minus_15_evens;
reg [31:0] xor_minus_15_odds;

reg [31:0] xor_minus_2_evens;
reg [31:0] xor_minus_2_odds;

reg [31:0] minus16_plus_minus7_evens;
reg [31:0] minus16_plus_minus7_odds;


always @(*) begin
    minus15_evens_selected_mem = W_intermediate[fsm_count_minus15_evens];
    minus2_evens_selected_mem = W_intermediate[fsm_count_minus2_evens];
    minus16_evens_selected_mem = W_intermediate[fsm_count_minus16_evens];
    minus7_evens_selected_mem = W_intermediate[fsm_count_minus7_evens];
    minus15_odds_selected_mem = W_intermediate[fsm_count_minus15_odds];
    minus2_odds_selected_mem = W_intermediate[fsm_count_minus2_odds];
    minus16_odds_selected_mem = W_intermediate[fsm_count_minus16_odds];
    minus7_odds_selected_mem = W_intermediate[fsm_count_minus7_odds];
//following logic corresponds to w-15 for both the odd and even engines
    right_rotate_7_evens = {minus15_evens_selected_mem, minus15_evens_selected_mem} >> 7;
    right_rotate_7_odds = {minus15_odds_selected_mem, minus15_odds_selected_mem} >> 7;
    right_rotate_18_evens = {minus15_evens_selected_mem, minus15_evens_selected_mem} >>18;
    right_rotate_18_odds = {minus15_odds_selected_mem, minus15_odds_selected_mem} >>18 ;
    right_shift_3_evens = minus15_evens_selected_mem >>3;
    right_shift_3_odds = minus15_odds_selected_mem >>3;

    xor_minus_15_evens = (right_rotate_7_evens ^ right_rotate_18_evens) ^ right_shift_3_evens;
    xor_minus_15_odds = (right_rotate_7_odds ^ right_rotate_18_odds) ^ right_shift_3_odds;

//following logic corresponds to w-12 for both the odd and even paths
    right_rotate_17_evens = {minus2_evens_selected_mem, minus2_evens_selected_mem} >> 17;
    right_rotate_17_odds = {minus2_odds_selected_mem, minus2_odds_selected_mem} >> 17;
    right_rotate_19_evens = {minus2_evens_selected_mem, minus2_evens_selected_mem} >> 19;
    right_rotate_19_odds = {minus2_odds_selected_mem, minus2_odds_selected_mem} >> 19;
    right_shift_10_evens = minus2_evens_selected_mem >>10;
    right_shift_10_odds = minus2_odds_selected_mem >>10;

    xor_minus_2_evens = (right_rotate_17_evens ^ right_rotate_19_evens) ^ right_shift_10_evens;
    xor_minus_2_odds = (right_rotate_17_odds ^ right_rotate_19_odds) ^ right_shift_10_odds;

//following logic adds w-16 and w-7
    minus16_plus_minus7_evens = minus16_evens_selected_mem  + minus7_evens_selected_mem;
    minus16_plus_minus7_odds = minus16_odds_selected_mem  + minus7_odds_selected_mem;

//following logic completes the w conversion for both odd and evens
    final_sum_evens = (xor_minus_15_evens + xor_minus_2_evens) + minus16_plus_minus7_evens;
    final_sum_odds = (xor_minus_15_odds + xor_minus_2_odds) + minus16_plus_minus7_odds;
end
//The following assigns are for assigning the first 16 values of W
assign W_first16[0] = W_in[0];
assign W_first16[1] = W_in[1];
assign W_first16[2] = W_in[2];
assign W_first16[3] = W_in[3];
assign W_first16[4] = W_in[4];
assign W_first16[5] = W_in[5];
assign W_first16[6] = W_in[6];
assign W_first16[7] = W_in[7];
assign W_first16[8] = W_in[8];
assign W_first16[9] = W_in[9];
assign W_first16[10] = W_in[10];
assign W_first16[11] = W_in[11];
assign W_first16[12] = W_in[12];
assign W_first16[13] = W_in[13];
assign W_first16[14] = W_in[14];
assign W_first16[15] = W_in[15];
assign W_first16[16] = 0;
assign W_first16[17] = 0;
assign W_first16[18] = 0;
assign W_first16[19] = 0;
assign W_first16[20] = 0;
assign W_first16[21] = 0;
assign W_first16[22] = 0;
assign W_first16[23] = 0;
assign W_first16[24] = 0;
assign W_first16[25] = 0;
assign W_first16[26] = 0;
assign W_first16[27] = 0;
assign W_first16[28] = 0;
assign W_first16[29] = 0;
assign W_first16[30] = 0;
assign W_first16[31] = 0;
assign W_first16[32] = 0;
assign W_first16[33] = 0;
assign W_first16[34] = 0;
assign W_first16[34] = 0;
assign W_first16[35] = 0;
assign W_first16[36] = 0;
assign W_first16[37] = 0;
assign W_first16[38] = 0;
assign W_first16[39] = 0;
assign W_first16[40] = 0;
assign W_first16[41] = 0;
assign W_first16[42] = 0;
assign W_first16[43] = 0;
assign W_first16[44] = 0;
assign W_first16[44] = 0;
assign W_first16[45] = 0;
assign W_first16[46] = 0;
assign W_first16[47] = 0;
assign W_first16[48] = 0;
assign W_first16[49] = 0;
assign W_first16[50] = 0;
assign W_first16[51] = 0;
assign W_first16[52] = 0;
assign W_first16[53] = 0;
assign W_first16[54] = 0;
assign W_first16[54] = 0;
assign W_first16[55] = 0;
assign W_first16[56] = 0;
assign W_first16[57] = 0;
assign W_first16[58] = 0;
assign W_first16[59] = 0;
assign W_first16[60] = 0;
assign W_first16[61] = 0;
assign W_first16[62] = 0;
assign W_first16[63] = 0;

endmodule
