module sigmoid(
    input clk,
    input reset,
    input signed [7:0] z_in,
    input div_start,
    
    output [7:0] sigmoid,
    output [7:0] x_out,

    output ready,
    output dbz_error
);

    wire signed [7:0] x_pipe [0:7];
    wire signed [7:0] y_pipe [0:7];
    wire signed [7:0] z_pipe [0:7];
    wire [8:0] sigmoid_sum;
    wire round_bit;

    wire [7:0] div_remainder;
    wire [7:0] quotient;

    assign x_pipe[0] = 8'd77; 
    assign y_pipe[0] = 8'd0;
    assign z_pipe[0] = (z_in >> 1);

    genvar i;
    generate
        combined_logic #(
            .shift_idx(1), 
            .alpha_1(8'd16), 
            .alpha_2(8'd35)
        ) CL ( 
            .clk(clk), 
            .x_in(x_pipe[0]), 
            .y_in(y_pipe[0]), 
            .z_in(z_pipe[0]), 
            .x_out(x_pipe[1]), 
            .y_out(y_pipe[1]), 
            .z_out(z_pipe[1])
        );

        for (i=0; i<6; i = i+1) begin: cordic_pipeline_stages

            localparam [3:0] stage_shift = (i == 0) ? (i + 4'd2) :
                                            (i == 4) ? (i - 4'd2) :
                                            (i == 5) ? (i - 4'd4) :
                                           i ;

            localparam [7:0] stage_alpha1 = (stage_shift == 4'd1) ? 8'd16 :
                                             (stage_shift == 4'd2) ? 8'd4 :
                                             (stage_shift == 4'd3) ? 8'd1 :
                                             (stage_shift == 4'd4) ? 8'd0 :
                                                                      8'd0;

            localparam [7:0] stage_alpha2 = (stage_shift == 4'd1) ? 8'd35 :
                                             (stage_shift == 4'd2) ? 8'd8 :
                                             (stage_shift == 4'd3) ? 8'd2 :
                                             (stage_shift == 4'd4) ? 8'd1 :
                                                                      8'd1;
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
        .dividend(y_pipe[7]),
        .divisor(x_pipe[7]),
        .remainder(div_remainder),
        .quotient(quotient),
        .ready(ready),
        .dbz_error(dbz_error)
    );

    assign x_out = x_pipe[7];

    assign sigmoid_sum = {1'b0, 8'd64} + {1'b0, quotient};
    assign round_bit = sigmoid_sum[0];

    assign sigmoid = ((sigmoid_sum + round_bit)>>1);

endmodule

module division_module(
    input  wire clk,
    input  wire reset,
    input  wire start,
    input  wire [7:0] dividend,   // Q6 unsigned (e.g. Q2.6)
    input  wire [7:0] divisor,    // Q6 unsigned
    output reg  [7:0] quotient,   // Q6 unsigned result
    output reg  [7:0] remainder,  // raw remainder of the scaled division
    output reg  ready,
    output reg  dbz_error
);

    localparam IDLE      = 2'b00;
    localparam CALCULATE = 2'b01;
    localparam DONE      = 2'b10;

    localparam DATA_W    = 8;
    localparam FRAC_BITS = 6;                   // Q6
    localparam NUM_W     = DATA_W + FRAC_BITS;  // 14-bit shifted numerator
    localparam A_W       = DATA_W + 1;          // 9-bit accumulator

    reg [1:0]  state, next_state;
    reg [DATA_W-1:0] m_reg;         // divisor
    reg [3:0]  counter;             // counts up to NUM_W (14) -> 4 bits enough
    reg [A_W-1:0]    a_reg;         // 9-bit accumulator
    reg [NUM_W-1:0]  q_reg;         // 14-bit quotient/numerator shift register

    // shift {A,Q} left first, THEN subtract M from the shifted A
    wire [A_W-1:0] shifted_a  = {a_reg[DATA_W-1:0], q_reg[NUM_W-1]};
    wire [A_W-1:0] sub_result = shifted_a - {1'b0, m_reg};

    always @(posedge clk or posedge reset) begin
        if (reset) state <= IDLE;
        else       state <= next_state;
    end

    always @(*) begin
        case (state)
            IDLE:      next_state = (start && divisor != 8'b0) ? CALCULATE : IDLE;
            CALCULATE: next_state = (counter == 4'd0) ? DONE : CALCULATE;
            DONE:      next_state = IDLE;
            default:   next_state = IDLE;
        endcase
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            counter   <= 4'd0;
            m_reg     <= {DATA_W{1'b0}};
            a_reg     <= {A_W{1'b0}};
            q_reg     <= {NUM_W{1'b0}};
            quotient  <= 8'b0;
            remainder <= 8'b0;
            ready     <= 1'b0;
            dbz_error <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    ready <= 1'b0;
                    if (start) begin
                        if (divisor == 8'b0) begin
                            dbz_error <= 1'b1;
                        end else begin
                            dbz_error <= 1'b0;
                            counter   <= NUM_W;
                            m_reg     <= divisor;
                            a_reg     <= {A_W{1'b0}};
                            q_reg     <= {dividend, {FRAC_BITS{1'b0}}};  // dividend << 6
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
                    counter <= counter - 4'd1;
                end

                DONE: begin
                    ready     <= 1'b1;
                    quotient  <= (q_reg[DATA_W-1:0] >> 1); 
                    remainder <= a_reg[DATA_W-1:0];
                end
            endcase
        end
    end
endmodule

module combined_logic #(
    parameter [3:0]  shift_idx = 4'd1,   
    parameter [7:0] alpha_1   = 8'd0, 
    parameter [7:0] alpha_2   = 8'd0
)(
    input clk,

    input signed [7:0] x_in,
    input signed [7:0] y_in,
    input signed [7:0] z_in,

    output wire signed [7:0] x_out,
    output wire signed [7:0] y_out,
    output wire signed [7:0] z_out
);

    wire signed [2:0]  sigma;
    wire signed [7:0] alpha;

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
    input  signed [7:0] z_in,               
    
    input  signed [7:0] alpha_1,         
    input  signed [7:0] alpha_2,         

    output reg signed [2:0]  sigma,     
    output reg signed [7:0] alpha        
);

    wire signed [7:0] thresh_0_5;
    wire signed [7:0] thresh_1_5;

    assign thresh_0_5 = 8'sh20 >>> (shift_idx << 1);
    assign thresh_1_5 = 8'sh60 >>> (shift_idx << 1);

    wire signed [7:0] minus_thresh_0_5 = -thresh_0_5;
    wire signed [7:0] minus_thresh_1_5 = -thresh_1_5;

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
    input signed [7:0] x_in,
    input signed [7:0] y_in,
    input signed [7:0] z_in,
 
    input signed [2:0]  sigma,       // Direction factor σ_i from {-2, -1, 0, 1, 2}
    input signed [7:0] alpha,       // Target angle step from LUT: atanh(sigma * 4^-i)
    input clk,
    output reg signed [7:0] x_out,
    output reg signed [7:0] y_out,
    output reg signed [7:0] z_out
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
