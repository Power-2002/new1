`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/15 16:40:36
// Design Name: 
// Module Name: dwc_unit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module dwc_unit#(
    parameter K = 3,
    parameter DATA_W = 8,
    parameter PROD_W = 16,
    parameter PSUM_W = 18
)(
    input  wire clk,
    input  wire rst_n,
    input  wire in_valid,
    
    // Six rows of input feature map
    input  wire signed [DATA_W-1:0] buffer0, 
    input  wire signed [DATA_W-1:0] buffer1, 
    input  wire signed [DATA_W-1:0] buffer2, 
    input  wire signed [DATA_W-1:0] buffer3, 
    input  wire signed [DATA_W-1:0] buffer4, 
    input  wire signed [DATA_W-1:0] buffer5,      

    // Weights (Packed: {w2, w1, w0})
    input  wire [K*DATA_W-1:0] w_col0,
    input  wire [K*DATA_W-1:0] w_col1,
    input  wire [K*DATA_W-1:0] w_col2,

    // Outputs
    output wire signed [31:0] out_sum0,
    output wire signed [31:0] out_sum1,
    output wire signed [31:0] out_sum2,
    output wire signed [31:0] out_sum3,
    
    output reg out_valid0,
    output reg out_valid1,
    output reg out_valid2,
    output reg out_valid3
);

    // ============================================================
    // 1. Weight Unpacking
    // ============================================================
    wire signed [DATA_W-1:0] w0_0 = w_col0[7:0];
    wire signed [DATA_W-1:0] w0_1 = w_col0[15:8];  wire signed [DATA_W-1:0] w0_2 = w_col0[23:16];
    wire signed [DATA_W-1:0] w1_0 = w_col1[7:0];
    wire signed [DATA_W-1:0] w1_1 = w_col1[15:8];  wire signed [DATA_W-1:0] w1_2 = w_col1[23:16];
    wire signed [DATA_W-1:0] w2_0 = w_col2[7:0];
    wire signed [DATA_W-1:0] w2_1 = w_col2[15:8];  wire signed [DATA_W-1:0] w2_2 = w_col2[23:16];

    // ============================================================
    // 2. Shared Shift Registers (Line Buffer Slice)
    // ============================================================
    reg signed [DATA_W-1:0] b0_d1, b0_d2;
    reg signed [DATA_W-1:0] b1_d1, b1_d2;
    reg signed [DATA_W-1:0] b2_d1, b2_d2;
    reg signed [DATA_W-1:0] b3_d1, b3_d2;
    reg signed [DATA_W-1:0] b4_d1, b4_d2;
    reg signed [DATA_W-1:0] b5_d1, b5_d2;
    
    reg v_d1, v_d2, v_d3; // 增加一级 v_d3 用于新的流水线阶段

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b0_d1 <= 0; b0_d2 <= 0;
            b1_d1 <= 0; b1_d2 <= 0;
            b2_d1 <= 0; b2_d2 <= 0;
            b3_d1 <= 0; b3_d2 <= 0;
            b4_d1 <= 0; b4_d2 <= 0;
            b5_d1 <= 0; b5_d2 <= 0;
            v_d1  <= 1'b0;
            v_d2  <= 1'b0;
            v_d3  <= 1'b0;
        end else begin
            if (in_valid) begin
                b0_d1 <= buffer0; b0_d2 <= b0_d1;
                b1_d1 <= buffer1; b1_d2 <= b1_d1;
                b2_d1 <= buffer2; b2_d2 <= b2_d1;
                b3_d1 <= buffer3; b3_d2 <= b3_d1;
                b4_d1 <= buffer4; b4_d2 <= b4_d1;
                b5_d1 <= buffer5; b5_d2 <= b5_d1;
                v_d1 <= 1'b1;
            end else begin
                v_d1 <= 1'b0;
            end
            // Valid pipeline shifts continuously
            v_d2 <= v_d1;
            v_d3 <= v_d2;
        end
    end

    // Packing Function
    function signed [26:0] pack;
        input signed [7:0] high;
        input signed [7:0] low;
        begin
            pack = {high, 11'd0, low};
        end
    endfunction

    // ============================================================
    // 3. Pipelined MAC (Fix for DSP Inference)
    // ============================================================
    
    // Stage 1 Registers: Stores products immediately after multiplication
    // 使用 (* use_dsp = "yes" *) 建议综合器将这些寄存器吸收到DSP的MREG中
    (* use_dsp = "yes" *) reg signed [47:0] p01_0, p01_1, p01_2, p01_3, p01_4, p01_5, p01_6, p01_7, p01_8;
    (* use_dsp = "yes" *) reg signed [47:0] p23_0, p23_1, p23_2, p23_3, p23_4, p23_5, p23_6, p23_7, p23_8;

    // Stage 2 Registers: Accumulators
    (* use_dsp = "no" *) reg signed [47:0] acc_pack_01;
    (* use_dsp = "no" *) reg signed [47:0] acc_pack_23;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_pack_01 <= 0;
            acc_pack_23 <= 0;
            
            p01_0 <= 0; p01_1 <= 0; p01_2 <= 0;
            p01_3 <= 0; p01_4 <= 0; p01_5 <= 0;
            p01_6 <= 0; p01_7 <= 0; p01_8 <= 0;
            
            p23_0 <= 0; p23_1 <= 0; p23_2 <= 0;
            p23_3 <= 0; p23_4 <= 0; p23_5 <= 0;
            p23_6 <= 0; p23_7 <= 0; p23_8 <= 0;

            out_valid0 <= 0; out_valid1 <= 0; out_valid2 <= 0; out_valid3 <= 0;
        end else begin
            
            // --- Stage 1: Calculate Products (Pipeline Level 1) ---
            // Triggered on v_d2, results ready for v_d3
            if (v_d2) begin
                // Row 0 & 1
                p01_0 <= pack(buffer1, buffer0) * w2_0;
                p01_1 <= pack(buffer2, buffer1) * w2_1;
                p01_2 <= pack(buffer3, buffer2) * w2_2;
                p01_3 <= pack(b1_d1,   b0_d1)   * w1_0;
                p01_4 <= pack(b2_d1,   b1_d1)   * w1_1;
                p01_5 <= pack(b3_d1,   b2_d1)   * w1_2;
                p01_6 <= pack(b1_d2,   b0_d2)   * w0_0;
                p01_7 <= pack(b2_d2,   b1_d2)   * w0_1;
                p01_8 <= pack(b3_d2,   b2_d2)   * w0_2;

                // Row 2 & 3
                p23_0 <= pack(buffer3, buffer2) * w2_0;
                p23_1 <= pack(buffer4, buffer3) * w2_1;
                p23_2 <= pack(buffer5, buffer4) * w2_2;
                p23_3 <= pack(b3_d1,   b2_d1)   * w1_0;
                p23_4 <= pack(b4_d1,   b3_d1)   * w1_1;
                p23_5 <= pack(b5_d1,   b4_d1)   * w1_2;
                p23_6 <= pack(b3_d2,   b2_d2)   * w0_0;
                p23_7 <= pack(b4_d2,   b3_d2)   * w0_1;
                p23_8 <= pack(b5_d2,   b4_d2)   * w0_2;
            end

            // --- Stage 2: Summation (Pipeline Level 2) ---
            // Triggered on v_d3 (which aligns with pXX registers being valid)
            if (v_d3) begin
                acc_pack_01 <= p01_0 + p01_1 + p01_2 + p01_3 + p01_4 + p01_5 + p01_6 + p01_7 + p01_8;
                acc_pack_23 <= p23_0 + p23_1 + p23_2 + p23_3 + p23_4 + p23_5 + p23_6 + p23_7 + p23_8;
                
                out_valid0 <= 1'b1; out_valid1 <= 1'b1;
                out_valid2 <= 1'b1; out_valid3 <= 1'b1;
            end else begin
                out_valid0 <= 1'b0; out_valid1 <= 1'b0;
                out_valid2 <= 1'b0; out_valid3 <= 1'b0;
            end
        end
    end

    // Outputs
    assign out_sum0 = {{13{acc_pack_01[18]}}, acc_pack_01[18:0]};
    assign out_sum1 = {{3{acc_pack_01[47]}},  acc_pack_01[47:19]};
    assign out_sum2 = {{13{acc_pack_23[18]}}, acc_pack_23[18:0]};
    assign out_sum3 = {{3{acc_pack_23[47]}},  acc_pack_23[47:19]};

endmodule