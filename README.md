# FPGA-Based Fitness Scheduler

A digital system designed in Verilog for FPGAs to create and manage a personalized workout schedule. This project was developed for the Digital Logic Laboratory course at Amirkabir University of Technology.

The system calculates the required number of one-minute workout sessions based on user inputs (weight, target calories, exercise intensity, and gender) and then manages the timing for each exercise and rest period.

---

## Key Features

- **Custom Workout Calculation:** A combinational circuit calculates the total workout duration based on a specific formula.
- **Automated Timing:** A Finite State Machine (FSM) manages the sequence of exercises and rest periods.
- **Workout and Rest Cycles:** Each session consists of a 45-second exercise period followed by a 15-second rest period.
- **User Controls:** Supports `Start`, `Skip`, and `Reset` functionalities to control the workout flow.
- **Real-Time Display:** Shows the current workout number and remaining time on a 4-digit 7-segment display.
- **Audio Feedback:** Uses a buzzer to signal the end of each rest period and the end of the entire session.
- **Debounced Inputs:** Implements debouncers for physical push-buttons to ensure clean signal processing.

---

## Module Descriptions

The project is designed with a modular approach for clarity and scalability:

- **`FitnessSystemTop`**: The top-level module that integrates all sub-modules.
- **`ClockGenerator`**: Generates the necessary clock frequencies (1Hz, display refresh, buzzer tone) from the main 50MHz clock.
- **`ButtonDebouncer`**: Cleans up noisy signals from physical push-buttons.
- **`WorkoutTimeCalculator`**: The combinational logic unit that calculates the total number of workouts.
- **`ExerciseFSM`**: The core state machine that controls the timing and flow of the session.
- **`DisplayMultiplexer`**: Manages the 4-digit 7-segment display using time-division multiplexing.
- **`BCDto7SegmentDecoder`**: Decodes BCD values into signals for the 7-segment display.
- **`BuzzerController`**: Drives the buzzer to produce audible beeps.

---

## How to Simulate

1.  **Prerequisites:** You need a Verilog simulator like Icarus Verilog or Xilinx Vivado.
2.  **Files:** Ensure `project_custom.v`, `testbench_custom.v`, and `inputs.txt` are in the same directory.
3.  **Compile and Run:** Use the following commands if you are using Icarus Verilog:
    ```bash
    # Compile both the design and the testbench
    iverilog -o workout_test project_custom.v testbench_custom.v

    # Run the simulation
    vvp workout_test
    ```
4.  **Check Output:** The simulation results will be written to the `outputs.txt` file.

---

*This repository is for educational purposes as part of a university project. The fitness and calorie calculation formulas are simplified and not intended for medical use.*
