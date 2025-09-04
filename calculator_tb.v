`include "mainmodule.v"
`timescale 1ns / 1ps

module testbench_calculator;

    integer fin, fout;
    
    reg [7:0] data_in;
    
    reg [2:0] weight_in;
    reg [1:0] calories_in;
    reg [1:0] MET_in;
    reg gender_in;
    
    wire [8:0] final_out;

    calc_total uut (
        .weight_in(weight_in),
        .calories_in(calories_in),
        .MET_in(MET_in),
        .gender_in(gender_in),
        .final_out(final_out)
    );

    initial begin
        fin = $fopen("input.txt", "r");
        fout = $fopen("output.txt", "w");
        
        if (fin == 0) begin
            $display("Error: Could not open inputs.txt.");
            $finish;
        end
        if (fout == 0) begin
             $display("Error: Could not open output.txt.");
             $finish;
        end
        while ($fscanf(fin, "%b\n", data_in) == 1) begin
            weight_in   = data_in[7:5];
            calories_in = data_in[4:3];
            MET_in      = data_in[2:1];
            gender_in   = data_in[0];

            #10;

            write_formatted_output(fout, data_in, final_out);
        end

        $fclose(fin);
        $fclose(fout);
        
        $finish;
    end

    task write_formatted_output;
        input integer file_handle;
        input [7:0] inputs;
        input [8:0] t_output;

        integer w_val, cal_val, met_val;
        reg [2:0] w_bin;
        reg [1:0] cal_bin;
        reg [1:0] met_bin;
        reg g_bin;

        begin
            w_bin = inputs[7:5];
            cal_bin = inputs[4:3];
            met_bin = inputs[2:1];
            g_bin = inputs[0];

            case(w_bin)
                3'b000: w_val = 50;
                3'b001: w_val = 60;
                3'b010: w_val = 70;
                3'b011: w_val = 80;
                3'b100: w_val = 90;
                3'b101: w_val = 100;
                3'b110: w_val = 110;
                3'b111: w_val = 120;
                default: w_val = 0;
            endcase

            case(cal_bin)
                2'b00: cal_val = 50;
                2'b01: cal_val = 100;
                2'b10: cal_val = 150;
                2'b11: cal_val = 200;
                default: cal_val = 0;
            endcase

            case(met_bin)
                2'b00: met_val = 1;
                2'b01: met_val = 2;
                2'b10: met_val = 4;
                2'b11: met_val = 8;
                default: met_val = 0;
            endcase
            
            if (g_bin == 1) begin
                $fdisplay(file_handle, "Inputs: W=%0d, Cal=%0d, MET=%0d, G=1.125 \t=> outputs: T=%0dmin", 
                                w_val, cal_val, met_val, t_output);
            end else begin
                $fdisplay(file_handle, "Inputs: W=%0d, Cal=%0d, MET=%0d, G=1 \t\t=> outputs: T=%0dmin", 
                                w_val, cal_val, met_val, t_output);
            end
        end
    endtask

endmodule
