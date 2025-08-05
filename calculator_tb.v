`timescale 1ns / 1ps
`include "pj.v"
module WorkoutTimeCalculator_TB;

    // Testbench inputs
    reg [7:0] test_vector_in;
    reg [2:0] w_code;
    reg [1:0] cal_code;
    reg [1:0] m_code;
    reg g_code;

    // DUT (Device Under Test) output
    wire [7:0] workout_minutes_out;

    // File I/O and helper variables
    integer file_in, file_out, status_code;
    integer weight_val, calories_val, met_val;
    real gender_factor;

    // Instantiate the Device Under Test
    WorkoutTimeCalculator dut (
        .weight_code(w_code),
        .calories_code(cal_code),
        .met_code(m_code),
        .gender_code(g_code),
        .total_workouts_out(workout_minutes_out)
    );

    initial begin
        // Open input and output files
        file_in = $fopen("inputs.txt", "r");
        file_out = $fopen("outputs.txt", "w");

        if (file_in == 0 || file_out == 0) begin
            $display("ERROR: Testbench could not open I/O files.");
            $finish;
        end
        
        $fdisplay(file_out, "--- Workout Calculator Simulation Results ---");

        // Loop to read each line from the input file
        while (!$feof(file_in)) begin
            status_code = $fscanf(file_in, "%b\n", test_vector_in);

            // Assign bits from the test vector to the DUT inputs
            {w_code, cal_code, m_code, g_code} = test_vector_in;

            // Allow time for combinational logic to settle
            #10;

            // Convert input codes to human-readable values for logging
            case(w_code)
                3'b000: weight_val = 50;
                3'b001: weight_val = 60;
                3'b010: weight_val = 70;
                3'b011: weight_val = 80;
                3'b100: weight_val = 90;
                3'b101: weight_val = 100;
                3'b110: weight_val = 110;
                3'b111: weight_val = 120;
                default: weight_val = 0;
            endcase

            case(cal_code)
                2'b00: calories_val = 50;
                2'b01: calories_val = 100;
                2'b10: calories_val = 150;
                2'b11: calories_val = 200;
                default: calories_val = 0;
            endcase

            case(m_code)
                2'b00: met_val = 1;
                2'b01: met_val = 2;
                2'b10: met_val = 4;
                2'b11: met_val = 8;
                default: met_val = 0;
            endcase
            
            gender_factor = (g_code == 1'b0) ? 1.0 : 1.125;

            // Write formatted results to the output file
            $fdisplay(file_out, "Vector: %b -> W:%0d, Cal:%0d, MET:%0d, G:%.3f | RESULT: %0d minutes",
                test_vector_in, weight_val, calories_val, met_val, gender_factor, workout_minutes_out);
        end

        // Clean up and finish simulation
        $fclose(file_in);
        $fclose(file_out);
        $display("Simulation complete. Please check 'outputs.txt'.");
        $finish;
    end

endmodule