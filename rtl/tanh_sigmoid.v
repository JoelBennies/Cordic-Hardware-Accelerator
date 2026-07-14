module joint_top_module(
    input clk,
    input reset,
    input signed [15:0] z_in,
    input div_start,
    
    output signed [15:0] tanh_out,
    output signed [15:0] sigmoid_out,

    output ready_tanh,
    output ready_sig,
    output dbz_error_tanh,
    output dbz_error_sig
);

    // PREPROCESSING: Zero-Gate parallel wire routing split
    wire signed [15:0] z_track_tanh = z_in;        
    wire signed [15:0] z_track_sig  = z_in >>> 1;  // Arithmetic shift for z/2

    // Internal wire to hold the raw intermediate tanh(z/2) quotient
    wire signed [15:0] raw_sig_quotient;

    // CHANNEL A INSTANTIATION: Pure Tanh Processing Path
    twice_module TRACK_TANH (
        .clk(clk),
        .reset(reset),
        .z_in(z_track_tanh),
        .div_start(div_start),
        .x_out(), // Unused at the top-level pins
        .y_out(),
        .z_out(),
        .tanh(tanh_out), // Directly drives the top-level Tanh pin
        .ready(ready_tanh),
        .dbz_error(dbz_error_tanh)
    );

    // CHANNEL B INSTANTIATION: Sigmoid Processing Path
    twice_module TRACK_SIGMOID (
        .clk(clk),
        .reset(reset),
        .z_in(z_track_sig),
        .div_start(div_start),
        .x_out(),
        .y_out(),
        .z_out(),
        .tanh(raw_sig_quotient), // Catches raw quotient ratio output
        .ready(ready_sig),
        .dbz_error(dbz_error_sig)
    );

    // POST-PROCESSING: Formula transformation layer
    assign sigmoid_out = (raw_sig_quotient + 16'sd16384) >>> 1;

endmodule




module twice_module(
    input clk,
    input reset,
    input signed [15:0] z_in,
    input div_start,
    
    output signed [15:0] x_out,
    output signed [15:0] y_out,
    output signed [15:0] z_out,
    output signed [15:0] tanh,

    output ready,
    output dbz_error
);

    wire signed [15:0] x_pipe [0:9];
    wire signed [15:0] y_pipe [0:9];
    wire signed [15:0] z_pipe [0:9];

    wire [15:0] div_remainder;
    wire [15:0] quotient;

    assign x_pipe[0] = 16'd19781; 
    assign y_pipe[0] = 16'd0;
    assign z_pipe[0] = z_in;

    genvar i;
    generate
        combined_logic #(
            .shift_idx(1), 
            .alpha_1(16'd4185), 
            .alpha_2(16'd9000)
        ) CL ( 
            .clk(clk), 
            .x_in(x_pipe[0]), 
            .y_in(y_pipe[0]), 
            .z_in(z_pipe[0]), 
            .x_out(x_pipe[1]), 
            .y_out(y_pipe[1]), 
            .z_out(z_pipe[1])
        );

        for (i=0; i<8; i = i+1) begin: cordic_pipeline_stages

            localparam [3:0] stage_shift = (i == 0) ? (i + 4'd1) :
                                           (i == 6) ? (i - 4'd1) :
                                           i ;

            localparam [15:0] stage_alpha1 = (stage_shift == 4'd1) ? 16'd4185 :
                                             (stage_shift == 4'd2) ? 16'd1025 :
                                             (stage_shift == 4'd3) ? 16'd256 :
                                             (stage_shift == 4'd4) ? 16'd64 :
                                             (stage_shift == 4'd5) ? 16'd16 :
                                             (stage_shift == 4'd6) ? 16'd4 : 
                                             (stage_shift == 4'd7) ? 16'd1 :
                                                                      16'd1;

            localparam [15:0] stage_alpha2 = (stage_shift == 4'd1) ? 16'd9000 :
                                             (stage_shift == 4'd2) ? 16'd2059 :
                                             (stage_shift == 4'd3) ? 16'd512 :
                                             (stage_shift == 4'd4) ? 16'd128 :
                                             (stage_shift == 4'd5) ? 16'd32 :
                                             (stage_shift == 4'd6) ? 16'd8 : 
                                             (stage_shift == 4'd7) ? 16'd2 :
                                                                      16'd1;
            combined_logic #(
                .shift_idx(stage_shift), 
                .alpha_1(stage_alpha1), 
                .alpha_2(stage_alpha2)
            ) CL ( 
                .clk(clk), 
                .x_in(x_pipe[i+1]), 
                .y_in(y_pipe[i+1]), 
                .z_in(z_pipe[i+1]), 
                .x_out(x_pipe[i+2]), 
                .y_out(y_pipe[i+2]), 
                .z_out(z_pipe[i+2])
            );
        end
    endgenerate

    division_module DM(
        .clk(clk),
        .reset(reset),
        .start(div_start),
        .dividend(y_pipe[8]),
        .divisor(x_pipe[8]),
        .remainder(div_remainder),
        .quotient(quotient),
        .ready(ready),
        .dbz_error(dbz_error)
    );

    assign x_out = x_pipe[8]; 
    assign y_out = y_pipe[8];
    assign z_out = z_pipe[8];
    assign tanh = quotient;

endmodule

module division_module(
    input  wire clk,
    input  wire reset,
    input  wire start,
    input  wire [15:0] dividend,   // Q14 unsigned
    input  wire [15:0] divisor,    // Q14 unsigned
    output reg [15:0] quotient,    // Q14 unsigned result
    output reg [15:0] remainder,   // raw remainder of the scaled division
    output reg ready,
    output reg dbz_error     
);

    localparam IDLE      = 2'b00;
    localparam CALCULATE = 2'b01;
    localparam DONE      = 2'b10;

    localparam DATA_W    = 16;
    localparam FRAC_BITS = 14;                  // Q14
    localparam NUM_W     = 30;   // 30-bit shifted numerator

    reg [1:0]  state, next_state;
    reg [15:0] m_reg;        // divisor
    reg [4:0]  counter;            // needs to count up to 30 -> 5 bits is enough
    reg [16:0]      a_reg;     // 17-bit accumulator (unchanged width)
    reg [29:0]     q_reg;     // 30-bit quotient/numerator shift register

    // shift {A,Q} left first, THEN subtract M from the shifted A
    wire [DATA_W:0] shifted_a  = {a_reg[DATA_W-1:0], q_reg[NUM_W-1]};
    wire [DATA_W:0] sub_result = shifted_a - {1'b0, m_reg};

    always @(posedge clk or posedge reset) begin
        if (reset) state <= IDLE;
        else       state <= next_state;
    end

    always @(*) begin
        case (state)
            IDLE:      next_state = (start && divisor != 16'b0) ? CALCULATE : IDLE;
            CALCULATE: next_state = (counter == 5'd0) ? DONE : CALCULATE;
            DONE:      next_state = IDLE;
            default:   next_state = IDLE;
        endcase
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            counter   <= 5'd0;
            m_reg     <= 16'b0;
            a_reg     <= {(DATA_W+1){1'b0}};
            q_reg     <= {NUM_W{1'b0}};
            quotient  <= 16'b0;
            remainder <= 16'b0;
            ready     <= 1'b0;
            dbz_error <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    ready <= 1'b0;
                    if (start) begin
                        if (divisor == 16'b0) begin
                            dbz_error <= 1'b1;
                        end else begin
                            dbz_error <= 1'b0;
                            counter   <= NUM_W;                     // 30 iterations now
                            m_reg     <= divisor;
                            a_reg     <= {(DATA_W+1){1'b0}};
                            q_reg     <= {dividend, {FRAC_BITS{1'b0}}}; // dividend << 14
                        end
                    end
                end

                CALCULATE: begin
                    if (sub_result[DATA_W] == 1'b1) begin
                        // went negative -> restore
                        a_reg <= shifted_a;
                        q_reg <= {q_reg[NUM_W-2:0], 1'b0};
                    end else begin
                        a_reg <= sub_result;
                        q_reg <= {q_reg[NUM_W-2:0], 1'b1};
                    end
                    counter <= counter - 5'd1;
                end

                DONE: begin
                    ready     <= 1'b1;
                    quotient  <= (q_reg[DATA_W-2:0] >> 1);  // lower 16 bits = Q14 result
                    remainder <= a_reg[DATA_W-1:0];
                end
            endcase
        end
    end
endmodule

module combined_logic #(
    parameter [3:0]  shift_idx = 4'd1,   
    parameter [15:0] alpha_1   = 16'd0, 
    parameter [15:0] alpha_2   = 16'd0
)(
    input clk,

    input signed [15:0] x_in,
    input signed [15:0] y_in,
    input signed [15:0] z_in,

    output wire signed [15:0] x_out,
    output wire signed [15:0] y_out,
    output wire signed [15:0] z_out
);

    wire signed [2:0]  sigma;
    wire signed [15:0] alpha;

    cordic_decision_logic #(
        .shift_idx(shift_idx)
    ) DL (
        .z_in(z_in),
        .alpha_1(alpha_1),
        .alpha_2(alpha_2), 
        .sigma(sigma),
        .alpha(alpha)
    );

    cordic_hyperbolic_step #(
        .shift_idx(shift_idx)
    ) HS (
        .clk(clk),
        .x_in(x_in),
        .y_in(y_in),
        .z_in(z_in),
        .sigma(sigma),
        .alpha(alpha),
        .x_out(x_out),
        .y_out(y_out),
        .z_out(z_out)
    );
endmodule


module cordic_decision_logic #(
    parameter [3:0] shift_idx = 4'd1
)(
    input  signed [15:0] z_in,               
    
    input  signed [15:0] alpha_1,         
    input  signed [15:0] alpha_2,         

    output reg signed [2:0]  sigma,     
    output reg signed [15:0] alpha        
);

    wire signed [15:0] thresh_0_5;
    wire signed [15:0] thresh_1_5;

    assign thresh_0_5 = 16'sh2000 >>> (shift_idx << 1);
    assign thresh_1_5 = 16'sh6000 >>> (shift_idx << 1);

    wire signed [15:0] minus_thresh_0_5 = -thresh_0_5;
    wire signed [15:0] minus_thresh_1_5 = -thresh_1_5;

    always @(*) begin
        if (z_in > thresh_1_5) begin
            sigma = 3'sd2;
            alpha = alpha_2;
        end 
        else if (z_in > thresh_0_5) begin 
            sigma = 3'sd1;
            alpha = alpha_1;
        end 
        else if (z_in >= minus_thresh_0_5) begin 
            sigma = 3'sd0;
            alpha = 16'sd0;
        end 
        else if (z_in >= minus_thresh_1_5) begin 
            sigma = -3'sd1;
            alpha = -alpha_1;
        end 
        else begin 
            sigma = -3'sd2;
            alpha = -alpha_2;
        end
    end
endmodule

module cordic_hyperbolic_step #(
    parameter [3:0] shift_idx = 4'd1
)(
    input signed [15:0] x_in,
    input signed [15:0] y_in,
    input signed [15:0] z_in,
 
    input signed [2:0]  sigma,       // Direction factor σ_i from {-2, -1, 0, 1, 2}
    input signed [15:0] alpha,       // Target angle step from LUT: atanh(sigma * 4^-i)
    input clk,
    output reg signed [15:0] x_out,
    output reg signed [15:0] y_out,
    output reg signed [15:0] z_out
);
    wire [3:0] base_shift = shift_idx << 1;

    always @(posedge clk) begin        
        case (sigma)
            3'sd2: begin
                x_out <= x_in + (y_in >>> base_shift) + (y_in >>> base_shift);
                y_out <= y_in + (x_in >>> base_shift) + (x_in >>> base_shift);
            end
            
            3'sd1: begin
                x_out <= x_in + (y_in >>> base_shift);
                y_out <= y_in + (x_in >>> base_shift);
            end
            
            3'sd0: begin
                x_out <= x_in;
                y_out <= y_in;
            end
            
            -3'sd1: begin  
                x_out <= x_in - (y_in >>> base_shift);
                y_out <= y_in - (x_in >>> base_shift);
            end
            
            -3'sd2: begin  
                x_out <= x_in - (y_in >>> base_shift) - (y_in >>> base_shift);
                y_out <= y_in - (x_in >>> base_shift) - (x_in >>> base_shift);
            end
            
            default: begin
                x_out <= x_in;
                y_out <= y_in;
            end
        endcase
        
        z_out <= z_in - alpha; 
    end
endmodule
