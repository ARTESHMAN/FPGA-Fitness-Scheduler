//////////////////////////////////////////////////////////////////////////////////
//
// Personalized Project: Fitness Training Scheduler
// Student ID: [40331427]
// Description: An FPGA-based system to calculate and time a series of exercises.
//
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

`define SYS_CLK 50_000_000

module karno_map_logic(
    input [2:0] weight,
    input [1:0] calories,
    output [7:0] out
);
    assign out[7] = (~weight[2] & ~weight[1] & calories[1]) | (~weight[2] & ~weight[0] & calories[1]) |
    (~weight[2] & calories[1] & calories[0]) | (~weight[1] & ~weight[0] & calories[1] & calories[0]);
    assign out[6] = (~weight[2] & ~weight[1] & calories[0]) | (~weight[2] & ~calories[1] & calories[0]) |
    (weight[1] & weight[0] & calories[1] & ~calories[0]) | (~weight[1] & ~weight[0] & ~calories[1] & calories[0]) |
    (weight[2] & calories[1] & ~calories[0]) | (weight[2] & weight[1] & calories[1]) | (~weight[1] & weight[0] & calories[1] & calories[0]);
    assign out[5] = (~weight[2] & ~calories[1] & ~calories[0]) | (~weight[2] & weight[1] & weight[0] & ~calories[0]) |
    (~weight[1] & ~weight[0] & ~calories[0]) | (weight[2] & weight[0] & calories[0]) | (weight[2] & weight[1] & calories[0]) |
    (~weight[2] & ~weight[1] & ~calories[1]) | (~weight[2] & ~weight[0] & calories[1] & calories[0]);
    assign out[4] = (~weight[2] & ~weight[1] & ~weight[0]) | (~weight[2] & weight[1] & weight[0] & calories[1]) |
    (weight[2] & ~weight[1] & weight[0]) | (weight[2] & weight[1] & ~weight[0] & ~calories[0]) | (~weight[2] & ~weight[1] & ~calories[0]) |
    (~weight[2] & ~weight[0] & ~calories[1] & calories[0]) | (weight[2] & weight[1] & ~calories[1]);
    assign out[3] = (~weight[2] & ~weight[1] & ~weight[0] & ~calories[1]) | (~weight[1] & weight[0] & calories[1] & calories[0]) |
    (weight[1] & ~weight[0] & calories[1] & calories[0]) | (~weight[2] & weight[1] & weight[0] & ~calories[1] & calories[0]) |
    (weight[2] & ~weight[1] & weight[0]) | (weight[2] & weight[0] & ~calories[0]) | (weight[1] & ~weight[0] & ~calories[1] & ~calories[0]);
    assign out[2] = (~weight[2] & ~weight[1] & ~weight[0] & ~calories[0]) | (~weight[1] & weight[0] & ~calories[1] & calories[0]) |
    (~weight[2] & ~weight[1] & calories[1] & ~calories[0]) | (weight[1] & ~weight[0] & ~calories[1] & calories[0]) |
    (~weight[2] & weight[1] & weight[0] & ~calories[1] & ~calories[0]) | (weight[1] & weight[0] & calories[1] & calories[0]) |
    (weight[2] & ~weight[1] & weight[0] & ~calories[1]) | (~weight[1] & ~weight[0] & calories[1] & ~calories[0]) |
    (weight[2] & ~weight[0] & calories[1] & calories[0]);
    assign out[1] = (~weight[1] & weight[0] & ~calories[0]) | (~weight[2] & weight[1] & calories[0]) | 
    (weight[2] & ~weight[0] & ~calories[1] & calories[0]) | (weight[1] & ~calories[1] & calories[0]) |
    (~weight[2] & weight[1] & ~calories[1]) | (weight[2] & weight[1] & ~weight[0] & ~calories[0]) |
    (weight[2] & weight[0] & calories[1] & ~calories[0]);
    assign out[0] = (~weight[2] & weight[1] & weight[0] & ~calories[1] & calories[0]) |
    (weight[2] & ~weight[0] & ~calories[1]) | (weight[2] & ~weight[0] & calories[0]) | (~weight[2] & weight[1] & ~weight[0] & ~calories[0]) |
    (~weight[2] & weight[1] & ~weight[0] & calories[1]) | (~weight[2] & weight[1] & calories[1] & ~calories[0]) |
    (weight[2] & weight[1] & weight[0] & ~calories[0]);
endmodule


module half_adder(
    input a, b,
    output s, c
);
    assign s = a ^ b;
    assign c = a & b;
endmodule

module full_adder(
    input a, b, cin,
    output s, cout
);
    wire s1, c1, c2;
    half_adder ha1( .a(a), .b(b), .s(s1), .c(c1) );
    half_adder ha2( .a(s1), .b(cin), .s(s), .c(c2) );
    assign cout = c1 | c2;
endmodule

module adder8_rca (
    input [7:0] A,
    input [7:0] B,
    input C0,
    output [8:0] S
);
    wire [7:0] C; 
    full_adder fa0 ( .a(A[0]), .b(B[0]), .cin(C0),   .s(S[0]), .cout(C[0]) );
    full_adder fa1 ( .a(A[1]), .b(B[1]), .cin(C[0]), .s(S[1]), .cout(C[1]) );
    full_adder fa2 ( .a(A[2]), .b(B[2]), .cin(C[1]), .s(S[2]), .cout(C[2]) );
    full_adder fa3 ( .a(A[3]), .b(B[3]), .cin(C[2]), .s(S[3]), .cout(C[3]) );
    full_adder fa4 ( .a(A[4]), .b(B[4]), .cin(C[3]), .s(S[4]), .cout(C[4]) );
    full_adder fa5 ( .a(A[5]), .b(B[5]), .cin(C[4]), .s(S[5]), .cout(C[5]) );
    full_adder fa6 ( .a(A[6]), .b(B[6]), .cin(C[5]), .s(S[6]), .cout(C[6]) );
    full_adder fa7 ( .a(A[7]), .b(B[7]), .cin(C[6]), .s(S[7]), .cout(C[7]) );
    assign S[8] = C[7];
endmodule

module gender_apply(
    input gender,
    input [7:0] in_data,
    output [8:0] out_num
);
    wire [8:0] ext = {1'b0, in_data};
    wire [8:0] shifted_sum;
    adder8_rca sum1(.A(in_data), .B(in_data >> 3), .C0(1'b0), .S(shifted_sum));
    assign out_num = gender ? shifted_sum : ext;
endmodule

module met_divider(
    input [8:0] data_in,
    input [1:0] MET,
    output [8:0] out_time
);
    wire [8:0] a = data_in;
    wire [8:0] b = data_in >> 1;
    wire [8:0] c = data_in >> 2;
    wire [8:0] d = data_in >> 3;
    assign out_time = (~MET[1] & ~MET[0]) ? a :
                      (~MET[1] &  MET[0]) ? b :
                      ( MET[1] & ~MET[0]) ? c :
                                             d;
endmodule

module calc_total(
    input [2:0] weight_in,
    input [1:0] calories_in,
    input [1:0] MET_in,
    input gender_in,
    output [8:0] final_out
);
    wire [7:0] stage1;
    wire [8:0] stage2;
    karno_map_logic fv(.weight(weight_in), .calories(calories_in), .out(stage1));
    gender_apply ga(.gender(gender_in), .in_data(stage1), .out_num(stage2));
    met_divider md(.data_in(stage2), .MET(MET_in), .out_time(final_out));
endmodule

module bin2bcd(
    input [8:0] binary,
    output reg [3:0] hundreds,
    output reg [3:0] tens,
    output reg [3:0] ones
);
    integer i;
    reg [11:0] bcd;
    always @(*) begin
        bcd = 0;
        for (i = 8; i >= 0; i = i - 1) begin
            if (bcd[3:0] >= 5) bcd = bcd + 12'd3;
            if (bcd[7:4] >= 5) bcd = bcd + 12'd48;
            if (bcd[11:8] >= 5) bcd = bcd + 12'd768;
            bcd = {bcd[10:0], binary[i]};
        end
        hundreds = bcd[11:8];
        tens = bcd[7:4];
        ones = bcd[3:0];
    end
endmodule

module clk_en_gen #(
    parameter DIVIDER = `SYS_CLK
)(
    input clk,
    input reset_n,
    output reg clk_en
);
    reg [31:0] ctr;
    always @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            ctr <= 0;
            clk_en <= 0;
        end else begin
            if (ctr == DIVIDER - 1) begin
                ctr <= 0;
                clk_en <= 1;
            end else begin
                ctr <= ctr + 1;
                clk_en <= 0;
            end
        end
    end
endmodule

module DebounceLevel #(parameter integer STABLE_COUNT=250000)(
    input clk, rst, din, output reg dout
);
    reg [31:0] cnt; reg stable;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt<=0;
            stable<=1'b1;
            dout<=1'b1;
        end else begin
            if (din==stable)
                cnt<=0;
            else if (cnt==STABLE_COUNT) begin
                stable<=din;
                dout<=din;
                cnt<=0;
            end
            else
                cnt<=cnt+1;
        end
    end
endmodule

module ButtonCond #(parameter ACTIVE_LOW=1, parameter integer STABLE_COUNT=250000)(
    input  clk,
    input rst,
    input btn_in,
    output reg press
);
    wire lvl_raw;
    DebounceLevel #(.STABLE_COUNT(STABLE_COUNT))
      udb (.clk(clk), .rst(rst), .din(btn_in), .dout(lvl_raw));

    wire lvl_norm = ACTIVE_LOW ? ~lvl_raw : lvl_raw;
    reg  prev;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            prev<=1'b0;
            press<=1'b0;
        end else begin
            press <= (lvl_norm & ~prev);
            prev <= lvl_norm;
        end
    end
endmodule

module fsm_core_logic(
    input clk,
    input reset_n,
    input start_edge,
    input skip_edge,
    input reset_edge,
    input clk_en_rising,
    input [8:0] total_count_in,
    output reg [1:0] state = 2'b00,
    output reg [5:0] timer = 6'b101101,
    output reg [7:0] curr_idx = 0,
    output reg buz_pulse = 1'b0,
    output reg buz_mode = 0
);
    localparam IDLE = 2'b00, EXERCISE = 2'b01, REST = 2'b10;
    reg [8:0] active_count = 0;
    always @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            state        <= IDLE;
            timer        <= 6'b101101;
            curr_idx     <= 0;
            active_count <= 0;
            buz_pulse    <= 1'b1;
            buz_mode     <= 1'b0;
        end else begin
            buz_pulse <= 1'b0;
            if (reset_edge) begin 
                state <= IDLE;
            end else begin
                case (state)
                    IDLE: begin
                        timer        <= 6'b101101;
                        curr_idx     <= 0;
                        active_count <= 0;
                        buz_mode <= 1'b0;
                        if (start_edge && total_count_in > 0) begin
                            active_count <= total_count_in;
                            state        <= EXERCISE;
                        end
                    end

                    EXERCISE: begin
                        if (skip_edge) begin
                            if (active_count > 0 && curr_idx == active_count - 1) begin
                                state     <= IDLE;
                                buz_pulse <= 1'b1;
                                buz_mode  <= 1'b1; 
                            end else if (active_count > 0) begin
                                curr_idx  <= curr_idx + 1;
                                state     <= EXERCISE;
                                timer     <= 6'b101101;
                                buz_pulse <= 1'b1;
                            end
                        end else if (clk_en_rising) begin
                            if (timer == 1) begin
                                state     <= REST;
                                timer     <= 6'b001111;
                                buz_pulse <= 1'b1;
                            end else if (timer != 0) begin
                                timer <= timer - 1;
                            end
                        end
                    end

                    REST: begin
                        if (skip_edge) begin
                            if (active_count > 0 && curr_idx == active_count - 1) begin
                                state     <= IDLE;
                                buz_pulse <= 1'b1;
                                buz_mode  <= 1'b1;
                            end else if (active_count > 0) begin
                                curr_idx  <= curr_idx + 1;
                                state     <= EXERCISE;
                                timer     <= 6'b101101;
                                buz_pulse <= 1'b1;
                            end
                        end else if (clk_en_rising) begin
                            if (timer == 1) begin
                                if (curr_idx == active_count - 1) begin
                                    state     <= IDLE;
                                    buz_pulse <= 1'b1;
                                    buz_mode  <= 1'b1;
                                end else begin
                                    curr_idx  <= curr_idx + 1;
                                    state     <= EXERCISE;
                                    timer     <= 6'b101101;
                                    buz_pulse <= 1'b1;
                                end
                            end else if (timer != 0) begin
                                timer <= timer - 1;
                            end
                        end
                    end
                endcase
            end
        end
    end
endmodule

module workout_fsm(
    input start_btn,
    input skip_btn,
    input reset_btn,
    input clk,
    input [8:0] total_count_in,
    output [1:0] state,
    output [5:0] timer,
    output [7:0] curr_idx,
    output buz_pulse,
    output buz_mode
);
    wire start_edge, skip_edge, reset_edge;
    wire clk_out;
    localparam integer DB_COUNT = 250000;
    ButtonCond #(.ACTIVE_LOW(1'b1), .STABLE_COUNT(DB_COUNT))
      bc_start (.clk(clk), .rst(~reset_btn), .btn_in(start_btn), .press(start_edge));
    ButtonCond #(.ACTIVE_LOW(1'b1), .STABLE_COUNT(DB_COUNT))
      bc_skip  (.clk(clk), .rst(~reset_btn), .btn_in(skip_btn),  .press(skip_edge));
    ButtonCond #(.ACTIVE_LOW(1'b1), .STABLE_COUNT(DB_COUNT))
      bc_reset (.clk(clk), .rst(~reset_btn), .btn_in(reset_btn), .press(reset_edge));
    clk_en_gen #(.DIVIDER(`SYS_CLK)) clkgen (
        .clk(clk),
        .reset_n(reset_btn),
        .clk_en(clk_out)
    );
    fsm_core_logic fcl (
        .clk(clk),
        .reset_n(reset_btn),
        .start_edge(start_edge),
        .skip_edge(skip_edge),
        .reset_edge(reset_edge),
        .clk_en_rising(clk_out),
        .total_count_in(total_count_in),
        .state(state),
        .timer(timer),
        .curr_idx(curr_idx),
        .buz_pulse(buz_pulse),
        .buz_mode(buz_mode)
    );
endmodule

module seg_driver(
    input [3:0] digit,
    output reg [7:0] seg_out
);
    always @(*) begin
        case (digit)
            4'd0: seg_out = 8'b00111111;
            4'd1: seg_out = 8'b00000110;
            4'd2: seg_out = 8'b01011011;
            4'd3: seg_out = 8'b01001111;
            4'd4: seg_out = 8'b01100110;
            4'd5: seg_out = 8'b01101101;
            4'd6: seg_out = 8'b01111101;
            4'd7: seg_out = 8'b00000111;
            4'd8: seg_out = 8'b01111111;
            4'd9: seg_out = 8'b01101111;
            default: seg_out = 8'b00000000;
        endcase
    end
endmodule

module seg_multiplexer(
    input clk,
    input reset_n,
    input [8:0] show_value,
    input [5:0] timer,
    output reg [4:0] sel = 5'b00001,
    output reg [3:0] digit
);
    reg [18:0] counter = 0;
    wire clk_tick = (counter == 100000 - 1);
    wire [3:0] t_h, t_t, t_o;
    bin2bcd timer_bcd(.binary({3'b0, timer}), .hundreds(t_h), .tens(t_t), .ones(t_o));
    wire [3:0] v_h, v_t, v_o;
    bin2bcd value_bcd(.binary(show_value), .hundreds(v_h), .tens(v_t), .ones(v_o));
    always @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            sel <= 5'b00001;
            digit <= 4'd0;
            counter <= 0;
        end else if (clk_tick) begin
            counter <= 0;
            case (sel)
                5'b00001: begin sel <= 5'b00010; digit <= t_t; end
                5'b00010: begin sel <= 5'b00100; digit <= v_o; end
                5'b00100: begin sel <= 5'b01000; digit <= v_t; end
                5'b01000: begin sel <= 5'b00001; digit <= t_o; end
                default:  begin sel <= 5'b00001; digit <= 4'd0; end
            endcase
        end else begin
            counter <= counter + 1;
        end
    end
endmodule

module BuzzerControllerModified(
    input clk,
    input rst, 
    input shortBeepTrig,
    input longBeepTrig,
    output buzzer
);

    localparam [25:0] SHORT_CYC = 26'd2500000; 
    localparam [25:0] PAUSE_CYC = 26'd2500000;  
    localparam [25:0] LONG_CYC  = 26'd30000000; 
    localparam [15:0] SHORT_DIV = 16'd24999;  
    localparam [15:0] LONG_DIV  = 16'd12499;   
    localparam S_IDLE      = 3'd0, S_BEEP1 = 3'd1, S_PAUSE1 = 3'd2, S_BEEP2 = 3'd3;
    localparam S_PAUSE2    = 3'd4, S_BEEP3 = 3'd5, S_LONG_BEEP = 3'd6;
    reg [2:0] state = S_IDLE;
    reg [25:0] cntDur;
    reg [15:0] divCnt;
    reg sq;
    wire buzzer_on;
    wire [15:0] divSel;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            cntDur <= 0;
        end else begin
            case(state)
                S_IDLE: begin
                    if (longBeepTrig) begin state <= S_LONG_BEEP; cntDur <= LONG_CYC;
                    end else if (shortBeepTrig) begin state <= S_BEEP1; cntDur <= SHORT_CYC; end
                end
                S_BEEP1: begin if (cntDur != 0) cntDur <= cntDur - 1; else begin state <= S_PAUSE1; cntDur <= PAUSE_CYC; end end
                S_PAUSE1: begin if (cntDur != 0) cntDur <= cntDur - 1; else begin state <= S_BEEP2; cntDur <= SHORT_CYC; end end
                S_BEEP2: begin if (cntDur != 0) cntDur <= cntDur - 1; else begin state <= S_PAUSE2; cntDur <= PAUSE_CYC; end end
                S_PAUSE2: begin if (cntDur != 0) cntDur <= cntDur - 1; else begin state <= S_BEEP3; cntDur <= SHORT_CYC; end end
                S_BEEP3: begin if (cntDur != 0) cntDur <= cntDur - 1; else state <= S_IDLE; end
                S_LONG_BEEP: begin if (cntDur != 0) cntDur <= cntDur - 1; else state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end
    assign buzzer_on = (state == S_BEEP1) || (state == S_BEEP2) || (state == S_BEEP3) || (state == S_LONG_BEEP);
    assign divSel = (state == S_LONG_BEEP) ? LONG_DIV : SHORT_DIV;
    always @(posedge clk or posedge rst) begin
        if(rst) begin divCnt <= 0; sq <= 0;
        end else begin
            if (!buzzer_on) begin divCnt <= 0; sq <= 0;
            end else if (divCnt == divSel) begin divCnt <= 0; sq <= ~sq;
            end else begin divCnt <= divCnt + 1; end
        end
    end
    assign buzzer = sq;
endmodule

module lcd_display(
    input clk,
    input rst_n,
    input workout_finished, 
    input [7:0] work_in,
    input [5:0] timer,
    output reg RS,
    output reg RW,
    output reg E,
    output reg [7:0] db
);
    localparam S_INIT = 4'd0, S_INIT_WAIT = 4'd1, S_REF_HOME = 4'd2, S_REF_L1 = 4'd3;
    localparam S_REF_L2_ADDR = 4'd4, S_REF_L2 = 4'd5, S_DONE_ADDR1 = 4'd6, S_DONE_WRITE = 4'd7;
    localparam S_DONE_ADDR2 = 4'd8, S_DONE_CLEAR = 4'd9, S_DONE_FINAL = 4'd10;

    reg [3:0] state = S_INIT;
    localparam CLK_DIV_1MS = `SYS_CLK / 1000;
    reg [15:0] ms_counter = 0;
    reg tick_ms = 0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin ms_counter <= 0; tick_ms <= 0;
        end else begin
            if(ms_counter == CLK_DIV_1MS - 1) begin ms_counter <= 0; tick_ms <= 1;
            end else begin ms_counter <= ms_counter + 1; tick_ms <= 0; end
        end
    end

    reg [7:0] timer_ms = 0;
    reg [4:0] step = 0;
    wire [3:0] timer_h, timer_t, timer_o;
    bin2bcd timer_bcd(.binary({3'b0, timer}), .hundreds(timer_h), .tens(timer_t), .ones(timer_o));
    wire [3:0] work_in_h, work_in_t, work_in_o;
    bin2bcd work_in_bcd(.binary({1'b0, work_in}), .hundreds(work_in_h), .tens(work_in_t), .ones(work_in_o));

    function [7:0] get_ex_char;
        input [3:0] ex_idx;
        input [4:0] p;
        reg [127:0] str;
        begin
            case (ex_idx)
                4'd0: str = "WALL SIT       "; 4'd1: str = "SQUAT          "; 4'd2: str = "PUSH UP        ";
                4'd3: str = "LUNGES R       "; 4'd4: str = "LUNGES L       "; 4'd5: str = "TRICEP DIPS    ";
                4'd6: str = "PLANK LADDER   "; 4'd7: str = "MOUNTAIN CLIMBER"; 4'd8: str = "TRICEP DIPS    ";
                4'd9: str = "BURPEES        "; default: str = "WORKOUT         ";
            endcase
            if (p < 16) get_ex_char = str[ (15-p)*8 +: 8 ]; else get_ex_char = " ";
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state <= S_INIT; timer_ms <= 0; step <= 0; E <= 0;
        end else begin
            E <= (ms_counter < (CLK_DIV_1MS/2));
            if(tick_ms) begin
                case(state)
                    S_INIT: begin
                        if(timer_ms < 50) timer_ms <= timer_ms + 1;
                        else begin timer_ms <= 0; state <= S_INIT_WAIT; end
                    end
                    S_INIT_WAIT: begin
                        RS <= 0; RW <= 0;
                        case(step)
                            0: begin db <= 8'h38; if(timer_ms >= 5) begin step <= step + 1; timer_ms <= 0; end end
                            1: begin db <= 8'h38; if(timer_ms >= 1) begin step <= step + 1; timer_ms <= 0; end end
                            2: begin db <= 8'h0C; if(timer_ms >= 1) begin step <= step + 1; timer_ms <= 0; end end
                            3: begin db <= 8'h01; if(timer_ms >= 2) begin step <= step + 1; timer_ms <= 0; end end
                            4: begin db <= 8'h06; if(timer_ms >= 1) begin step <= 0; state <= S_REF_HOME; timer_ms <= 0; end end
                        endcase
                        timer_ms <= timer_ms + 1;
                    end
                    S_REF_HOME: begin
                        if (workout_finished) begin state <= S_DONE_ADDR1; timer_ms <= 0;
                        end else begin
                            RS <= 0; RW <= 0; db <= 8'h02;
                            if(timer_ms >= 2) begin timer_ms <= 0; state <= S_REF_L1; end
                            timer_ms <= timer_ms + 1;
                        end
                    end
                    S_REF_L1: begin
                        RS <= 1; RW <= 0;
                        case(step)
                            0: db <= "T"; 1: db <= "i"; 2: db <= "m"; 3: db <= "e"; 4: db <= ":"; 5: db <= " ";
                            6: db <= timer_t+'h30; 7: db <= timer_o+'h30; 8: db <= "s"; default: db <= " ";
                        endcase
                        step <= step + 1;
                        if(step == 15) begin step <= 0; state <= S_REF_L2_ADDR; end
                    end
                    S_REF_L2_ADDR: begin
                        RS <= 0; RW <= 0; db <= 8'hC0;
                        if(timer_ms >= 1) begin timer_ms <= 0; step <= 0; state <= S_REF_L2; end
                        timer_ms <= timer_ms + 1;
                    end
                    S_REF_L2: begin
                        RS <= 1; RW <= 0;
                        if(step < 16) begin db <= get_ex_char(work_in_o, step); step <= step + 1;
                        end else begin step <= 0; state <= S_REF_HOME; end
                    end
                    S_DONE_ADDR1: begin
                        RS <= 0; RW <= 0; db <= 8'h80; 
                        if(timer_ms >= 2) begin timer_ms <= 0; step <= 0; state <= S_DONE_WRITE; end
                        timer_ms <= timer_ms + 1;
                    end
                    S_DONE_WRITE: begin
                        RS <= 1; RW <= 0;
                        case(step)
                            0: db <= "D"; 1: db <= "o"; 2: db <= "n"; 3: db <= "e"; 4: db <= "!"; default: db <= " ";
                        endcase
                        step <= step + 1;
                        if(step == 15) begin step <= 0; state <= S_DONE_ADDR2; end
                    end
                    S_DONE_ADDR2: begin
                        RS <= 0; RW <= 0; db <= 8'hC0; 
                        if(timer_ms >= 2) begin timer_ms <= 0; step <= 0; state <= S_DONE_CLEAR; end
                        timer_ms <= timer_ms + 1;
                    end
                    S_DONE_CLEAR: begin
                        RS <= 1; RW <= 0; db <= " ";
                        step <= step + 1;
                        if (step == 15) begin step <= 0; state <= S_DONE_FINAL; end
                    end
                    S_DONE_FINAL: begin end
                    default: state <= S_INIT;
                endcase
            end
        end
    end
endmodule
module mainmodule (
    input start,
    input skip,
    input reset, 
    input clk,
    input [2:0] weight,
    input [1:0] calories,
    input [1:0] MET,
    input gender,
    output [4:0] seg_sel,
    output [7:0] seg_data,
    output buzzer_out,
    output LCD_RS,
    output LCD_RW,
    output LCD_E,
    output [7:0] LCD_DB
);
    wire [1:0] state_w;
    wire [5:0] timer_w;
    wire [7:0] curr_idx_w;
    wire act_buz_w;
    wire buz_mode_w;

    wire [8:0] total_workouts;
    calc_total calc1(
        .weight_in(weight), .calories_in(calories),
        .MET_in(MET), .gender_in(gender), .final_out(total_workouts)
    );

    workout_fsm f1(
        .start_btn(start), .skip_btn(skip), .reset_btn(reset), .clk(clk),
        .total_count_in(total_workouts), .state(state_w), .timer(timer_w),
        .curr_idx(curr_idx_w), .buz_pulse(act_buz_w), .buz_mode(buz_mode_w)
    );

    wire short_beep, long_beep;
    assign short_beep = act_buz_w & ~buz_mode_w;
    assign long_beep = act_buz_w & buz_mode_w;

    BuzzerControllerModified b1(
        .clk(clk), .rst(~reset), 
        .shortBeepTrig(short_beep), .longBeepTrig(long_beep), .buzzer(buzzer_out)
    );

    reg workout_finished_latch = 1'b0;
    always @(posedge clk or negedge reset) begin
        if(!reset)
            workout_finished_latch <= 1'b0;
        else if(long_beep) 
            workout_finished_latch <= 1'b1;
    end

    wire [8:0] to_show;
    assign to_show = (state_w == 2'b00) ? total_workouts : {1'b0, curr_idx_w};
    wire [3:0] seg_digit;
    seg_multiplexer s1_real(
        .clk(clk), .reset_n(reset),
        .show_value(to_show), .timer(timer_w),
        .sel(seg_sel), .digit(seg_digit)
    );
    seg_driver s2(.digit(seg_digit), .seg_out(seg_data));

    lcd_display lcd1(
        .clk(clk), .rst_n(reset),
        .workout_finished(workout_finished_latch), 
        .work_in(curr_idx_w), .timer(timer_w),
        .RS(LCD_RS), .RW(LCD_RW), .E(LCD_E), .db(LCD_DB)
    );
endmodule

