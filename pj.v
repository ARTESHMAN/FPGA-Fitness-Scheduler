//////////////////////////////////////////////////////////////////////////////////
//
// Personalized Project: Fitness Training Scheduler
// Student ID: [40331427]
// Description: An FPGA-based system to calculate and time a series of exercises.
//
//////////////////////////////////////////////////////////////////////////////////


//================================================================================
// Module 1: ClockGenerator
// Generates the required clock signals from the main 50MHz system clock.
// Outputs:
//   - clk_1s_tick: A 1Hz clock for second-based counters.
//   - clk_display_refresh: A clock for refreshing the 7-segment display.
//   - clk_buzzer_tone: A clock for generating the buzzer's audio frequency.
//================================================================================
module ClockGenerator(
    input wire sys_clk_50mhz,
    input wire global_reset,
    output reg clk_1s_tick,
    output reg clk_display_refresh,
    output reg clk_buzzer_tone
);

    // Counter for 1Hz clock generation (50,000,000 / 2 = 25,000,000)
    reg [24:0] counter_for_1s;
    localparam MAX_COUNT_1S = 25'd24_999_999;

    // Counter for display refresh clock (~488 Hz)
    reg [16:0] counter_for_display;
    localparam MAX_COUNT_DISPLAY = 17'd51_228;

    // Counter for audio tone generation (~1 kHz)
    reg [14:0] counter_for_audio;
    localparam MAX_COUNT_AUDIO = 15'd24_999;

    always @(posedge sys_clk_50mhz or posedge global_reset) begin
        if (global_reset) begin
            counter_for_1s <= 0;
            clk_1s_tick <= 0;
            counter_for_display <= 0;
            clk_display_refresh <= 0;
            counter_for_audio <= 0;
            clk_buzzer_tone <= 0;
        end else begin
            // Logic for the 1Hz tick
            if (counter_for_1s == MAX_COUNT_1S) begin
                counter_for_1s <= 0;
                clk_1s_tick <= ~clk_1s_tick;
            end else begin
                counter_for_1s <= counter_for_1s + 1;
            end

            // Logic for the display refresh clock
            if (counter_for_display == MAX_COUNT_DISPLAY) begin
                counter_for_display <= 0;
                clk_display_refresh <= ~clk_display_refresh;
            end else begin
                counter_for_display <= counter_for_display + 1;
            end

            // Logic for the buzzer audio tone
            if (counter_for_audio == MAX_COUNT_AUDIO) begin
                counter_for_audio <= 0;
                clk_buzzer_tone <= ~clk_buzzer_tone;
            end else begin
                counter_for_audio <= counter_for_audio + 1;
            end
        end
    end

endmodule


//================================================================================
// Module 2: ButtonDebouncer
// Removes mechanical bounce from push-button inputs to provide a clean signal.
//================================================================================
module ButtonDebouncer(
    input wire clk,
    input wire button_in_noisy,
    output wire button_out_clean
);

    reg [2:0] debounce_shift_reg;

    always @(posedge clk) begin
        debounce_shift_reg <= {debounce_shift_reg[1:0], button_in_noisy};
    end

    // Output is high only when the last 3 samples are high
    assign button_out_clean = &debounce_shift_reg;

endmodule


//================================================================================
// Module 3: WorkoutTimeCalculator
// Combinational logic to calculate the total number of workout minutes (T).
// Formula: T = ((Calories * 60) / Weight) * GenderFactor * (1/MET)
// Implemented without using '*' or '/' operators, as per project constraints.
//================================================================================
module WorkoutTimeCalculator(
    input wire [2:0] weight_code,
    input wire [1:0] calories_code,
    input wire [1:0] met_code,
    input wire gender_code,
    output wire [7:0] total_workouts_out
);

    wire [4:0] lookup_addr = {weight_code, calories_code};
    reg [8:0] term1_result; // Result of (Cal*60)/W. Using 9 bits for precision.
    reg [11:0] term2_result; // Result after applying gender factor. 12 bits to prevent overflow.
    reg [7:0] final_workout_count;

    // Part 1: Calculate (Calories * 60) / Weight using a Look-Up Table (LUT)
    always @(*) begin
        case (lookup_addr)
            // Weight = 50kg (000)
            5'b000_00: term1_result = 9'd60;
            5'b000_01: term1_result = 9'd120;
            5'b000_10: term1_result = 9'd180;
            5'b000_11: term1_result = 9'd240;
            // Weight = 60kg (001)
            5'b001_00: term1_result = 9'd50;
            5'b001_01: term1_result = 9'd100;
            5'b001_10: term1_result = 9'd150;
            5'b001_11: term1_result = 9'd200;
            // Weight = 70kg (010)
            5'b010_00: term1_result = 9'd42;
            5'b010_01: term1_result = 9'd85;
            5'b010_10: term1_result = 9'd128;
            5'b010_11: term1_result = 9'd171;
            // Weight = 80kg (011)
            5'b011_00: term1_result = 9'd37;
            5'b011_01: term1_result = 9'd75;
            5'b011_10: term1_result = 9'd112;
            5'b011_11: term1_result = 9'd150;
            // Weight = 90kg (100)
            5'b100_00: term1_result = 9'd33;
            5'b100_01: term1_result = 9'd66;
            5'b100_10: term1_result = 9'd100;
            5'b100_11: term1_result = 9'd133;
            // Weight = 100kg (101)
            5'b101_00: term1_result = 9'd30;
            5'b101_01: term1_result = 9'd60;
            5'b101_10: term1_result = 9'd90;
            5'b101_11: term1_result = 9'd120;
            // Weight = 110kg (110)
            5'b110_00: term1_result = 9'd27;
            5'b110_01: term1_result = 9'd54;
            5'b110_10: term1_result = 9'd81;
            5'b110_11: term1_result = 9'd109;
            // Weight = 120kg (111)
            5'b111_00: term1_result = 9'd25;
            5'b111_01: term1_result = 9'd50;
            5'b111_10: term1_result = 9'd75;
            5'b111_11: term1_result = 9'd100;
            default: term1_result = 9'd0;
        endcase
    end

    // Part 2: Apply Gender Factor (1 for Male, 1.125 for Female)
    // G=1.125 is implemented as: value + value/8
    // value/8 is achieved by shifting right by 3 positions.
    always @(*) begin
        if (gender_code == 1'b0) begin // Male
            term2_result = term1_result;
        end else begin // Female
            term2_result = term1_result + (term1_result >>> 3); // Logical shift for unsigned
        end
    end

    // Part 3: Apply MET Factor (Divide by MET value)
    // Since MET values are powers of 2, division is done via right shifting.
    always @(*) begin
        case (met_code)
            2'b00: final_workout_count = term2_result;        // MET=1 (shift by 0)
            2'b01: final_workout_count = term2_result >>> 1;  // MET=2 (divide by 2)
            2'b10: final_workout_count = term2_result >>> 2;  // MET=4 (divide by 4)
            2'b11: final_workout_count = term2_result >>> 3;  // MET=8 (divide by 8)
            default: final_workout_count = 8'd0;
        endcase
    end

    assign total_workouts_out = final_workout_count;

endmodule

//================================================================================
// Module 4: ExerciseFSM
// The main state machine that controls the workout/rest timing sequence.
//================================================================================
module ExerciseFSM(
    input wire clk_1s_tick,
    input wire reset_signal,
    input wire start_signal,
    input wire skip_signal,
    input wire [7:0] total_workouts_in,

    output reg [7:0] current_workout_num_out,
    output reg [5:0] remaining_time_out,
    output reg trigger_beep_out,
    output reg trigger_finish_beep_out,
    output reg session_complete_flag
);

    // State definitions for the FSM
    localparam STATE_IDLE = 2'b00;
    localparam STATE_EXERCISE = 2'b01;
    localparam STATE_REST = 2'b10;
    localparam STATE_FINISHED = 2'b11;

    reg [1:0] current_state, next_state;

    // Internal counters and registers
    reg [7:0] total_workouts_reg;
    reg [7:0] exercise_counter;
    reg [5:0] countdown_timer;

    // Edge detection for single-pulse button presses
    reg start_prev_state, skip_prev_state;
    wire start_pressed_edge = start_signal && !start_prev_state;
    wire skip_pressed_edge = skip_signal && !skip_prev_state;

    always @(posedge clk_1s_tick or posedge reset_signal) begin
        if(reset_signal) begin
           start_prev_state <= 1'b0;
           skip_prev_state <= 1'b0;
        end else begin
           start_prev_state <= start_signal;
           skip_prev_state <= skip_signal;
        end
    end

    // State register logic
    always @(posedge clk_1s_tick or posedge reset_signal) begin
        if (reset_signal) begin
            current_state <= STATE_IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // Combinational logic for next state and outputs
    always @(*) begin
        // Default output values
        next_state = current_state;
        current_workout_num_out = exercise_counter;
        remaining_time_out = countdown_timer;
        trigger_beep_out = 1'b0;
        trigger_finish_beep_out = 1'b0;
        session_complete_flag = 1'b0;

        case (current_state)
            STATE_IDLE: begin
                current_workout_num_out = 0;
                remaining_time_out = 0;
                if (start_pressed_edge && total_workouts_in > 0) begin
                    next_state = STATE_EXERCISE;
                end
            end
            STATE_EXERCISE: begin
                if (countdown_timer == 0 || skip_pressed_edge) begin
                    next_state = STATE_REST;
                end
            end
            STATE_REST: begin
                if (countdown_timer == 0) begin
                    trigger_beep_out = 1'b1; // Short beep at the end of rest
                    if (exercise_counter >= total_workouts_reg) begin
                        next_state = STATE_FINISHED;
                    end else begin
                        next_state = STATE_EXERCISE;
                    end
                end
            end
            STATE_FINISHED: begin
                session_complete_flag = 1'b1;
                trigger_finish_beep_out = 1'b1; // Longer beep for session end
                next_state = STATE_FINISHED; // Remain here until reset
            end
        endcase
    end

    // Sequential logic for internal counters
    always @(posedge clk_1s_tick or posedge reset_signal) begin
        if (reset_signal) begin
            exercise_counter <= 0;
            countdown_timer <= 0;
            total_workouts_reg <= 0;
        end else begin
            // Using next_state to avoid a 1-clock-cycle delay in counter logic
            if (next_state == STATE_IDLE) begin
                exercise_counter <= 0;
                countdown_timer <= 0;
                total_workouts_reg <= total_workouts_in; // Latch the total workouts
            end
            else if (next_state == STATE_EXERCISE && current_state != STATE_EXERCISE) begin
                // Entering EXERCISE state for the first time
                exercise_counter <= exercise_counter + 1;
                countdown_timer <= 45;
            end
            else if (next_state == STATE_REST && current_state != STATE_REST) begin
                // Entering REST state for the first time
                countdown_timer <= 15;
            end
            else if (countdown_timer > 0) begin
                // Decrement timer in WORK or REST states
                countdown_timer <= countdown_timer - 1;
            end
        end
    end

endmodule


//================================================================================
// Module 5: BCDto7SegmentDecoder
// Converts a 4-bit BCD digit to 7-segment display signals (common cathode).
//================================================================================
module BCDto7SegmentDecoder(
    input wire [3:0] bcd_digit_in,
    output reg [6:0] segment_data_out // {g,f,e,d,c,b,a}
);
    always @(*) begin
        case(bcd_digit_in)
            4'd0: segment_data_out = 7'b0111111; // "0"
            4'd1: segment_data_out = 7'b0000110; // "1"
            4'd2: segment_data_out = 7'b1011011; // "2"
            4'd3: segment_data_out = 7'b1001111; // "3"
            4'd4: segment_data_out = 7'b1100110; // "4"
            4'd5: segment_data_out = 7'b1101101; // "5"
            4'd6: segment_data_out = 7'b1111101; // "6"
            4'd7: segment_data_out = 7'b0000111; // "7"
            4'd8: segment_data_out = 7'b1111111; // "8"
            4'd9: segment_data_out = 7'b1101111; // "9"
            default: segment_data_out = 7'b0000000; // Off
        endcase
    end
endmodule


//================================================================================
// Module 6: DisplayMultiplexer
// Drives a 4-digit 7-segment display using time-division multiplexing.
//================================================================================
module DisplayMultiplexer(
    input wire clk_refresh,
    input wire reset_signal,
    input wire [7:0] value1, // e.g., workout number
    input wire [5:0] value2, // e.g., remaining time
    output reg [3:0] anode_select_out,
    output wire [6:0] segment_data_out
);

    reg [1:0] active_digit_selector;
    reg [3:0] bcd_to_decode;

    // Counter to cycle through the 4 digits
    always @(posedge clk_refresh or posedge reset_signal) begin
        if (reset_signal)
            active_digit_selector <= 2'b00;
        else
            active_digit_selector <= active_digit_selector + 1;
    end

    // MUX to select the data for the currently active digit
    always @(*) begin
        case (active_digit_selector)
            2'b00: begin // Digit 1 (Time - Units)
                anode_select_out = 4'b1110; // Activate right-most digit
                bcd_to_decode = value2 % 10;
            end
            2'b01: begin // Digit 2 (Time - Tens)
                anode_select_out = 4'b1101; // Activate second digit
                bcd_to_decode = value2 / 10;
            end
            2'b10: begin // Digit 3 (Workout# - Units)
                anode_select_out = 4'b1011; // Activate third digit
                bcd_to_decode = value1 % 10;
            end
            2'b11: begin // Digit 4 (Workout# - Tens)
                anode_select_out = 4'b0111; // Activate left-most digit
                bcd_to_decode = value1 / 10;
            end
            default: begin
                anode_select_out = 4'b1111; // All off
                bcd_to_decode = 4'b0000;
            end
        endcase
    end

    // Instantiate the BCD to 7-segment decoder
    BCDto7SegmentDecoder digit_decoder (
        .bcd_digit_in(bcd_to_decode),
        .segment_data_out(segment_data_out)
    );

endmodule

//================================================================================
// Module 7: BuzzerController
// Generates an audible tone when enabled by the FSM.
//================================================================================
module BuzzerController(
    input wire audio_freq_clk,
    input wire beep_short_enable,
    input wire beep_long_enable,
    output reg buzzer_pin_out
);
    reg [9:0] beep_timer; // Counter for beep duration

    always @(posedge audio_freq_clk) begin
        if (beep_short_enable) begin
            // For a short beep, e.g., 200ms
            if (beep_timer < 200) begin
                buzzer_pin_out <= ~buzzer_pin_out; // Generate square wave
                beep_timer <= beep_timer + 1;
            end else begin
                buzzer_pin_out <= 1'b0;
            end
        end else if (beep_long_enable) begin
            // For a long beep, e.g., 1000ms (1 second)
            if (beep_timer < 1000) begin
                buzzer_pin_out <= ~buzzer_pin_out;
                beep_timer <= beep_timer + 1;
            end else begin
                buzzer_pin_out <= 1'b0;
            end
        end else begin
            buzzer_pin_out <= 1'b0;
            beep_timer <= 0; // Reset duration counter
        end
    end
endmodule


//================================================================================
// TOP LEVEL MODULE: FitnessSystemTop
// Integrates all sub-modules and connects them to the FPGA's physical pins.
//================================================================================
module FitnessSystemTop(
    input wire main_clk_50mhz,
    input wire physical_reset_btn,

    // User inputs from switches
    input wire [2:0] weight_sw,
    input wire [1:0] calories_sw,
    input wire [1:0] met_sw,
    input wire gender_sw,

    // Control buttons
    input wire start_pb,
    input wire skip_pb,

    // Physical outputs
    output wire [6:0] seven_seg_pins,
    output wire [3:0] seven_seg_anodes,
    output wire buzzer_pin
);

    // Internal signals connecting the modules
    wire clk_1hz, clk_for_display, clk_for_audio;
    wire reset_clean, start_clean, skip_clean;
    wire [7:0] calculated_total_workouts;
    wire [7:0] active_workout_number;
    wire [5:0] seconds_remaining;
    wire short_beep_trigger, long_beep_trigger, is_finished_flag;


    // 1. Instantiate Clock Generator
    ClockGenerator clk_gen_unit (
        .sys_clk_50mhz(main_clk_50mhz),
        .global_reset(physical_reset_btn),
        .clk_1s_tick(clk_1hz),
        .clk_display_refresh(clk_for_display),
        .clk_buzzer_tone(clk_for_audio)
    );

    // 2. Instantiate Debouncers for push buttons
    ButtonDebouncer reset_debouncer (
        .clk(clk_for_display), .button_in_noisy(physical_reset_btn), .button_out_clean(reset_clean)
    );
    ButtonDebouncer start_debouncer (
        .clk(clk_for_display), .button_in_noisy(start_pb), .button_out_clean(start_clean)
    );
    ButtonDebouncer skip_debouncer (
        .clk(clk_for_display), .button_in_noisy(skip_pb), .button_out_clean(skip_clean)
    );

    // 3. Instantiate Workout Time Calculator
    WorkoutTimeCalculator calc_unit (
        .weight_code(weight_sw),
        .calories_code(calories_sw),
        .met_code(met_sw),
        .gender_code(gender_sw),
        .total_workouts_out(calculated_total_workouts)
    );

    // 4. Instantiate Main Exercise FSM
    ExerciseFSM fsm_controller (
        .clk_1s_tick(clk_1hz),
        .reset_signal(reset_clean),
        .start_signal(start_clean),
        .skip_signal(skip_clean),
        .total_workouts_in(calculated_total_workouts),
        .current_workout_num_out(active_workout_number),
        .remaining_time_out(seconds_remaining),
        .trigger_beep_out(short_beep_trigger),
        .trigger_finish_beep_out(long_beep_trigger),
        .session_complete_flag(is_finished_flag)
    );

    // 5. Instantiate Display Multiplexer
    // Display total calculated workouts in idle state, otherwise show live data.
    wire [7:0] display_val1 = (active_workout_number == 0) ? calculated_total_workouts : active_workout_number;
    wire [5:0] display_val2 = (active_workout_number == 0) ? 0 : seconds_remaining;

    DisplayMultiplexer display_driver (
        .clk_refresh(clk_for_display),
        .reset_signal(reset_clean),
        .value1(display_val1),
        .value2(display_val2),
        .anode_select_out(seven_seg_anodes),
        .segment_data_out(seven_seg_pins)
    );
    
    // 6. Instantiate Buzzer Controller
    BuzzerController buzzer_unit (
        .audio_freq_clk(clk_for_audio),
        .beep_short_enable(short_beep_trigger),
        .beep_long_enable(long_beep_trigger),
        .buzzer_pin_out(buzzer_pin)
    );

endmodule