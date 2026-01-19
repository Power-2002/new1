`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/15 16:33:02
// Design Name: 
// Module Name: requantize16
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
//////////////////////////////////////////////////////////////////////////////////`timescale 1ns / 1ps
`timescale 1ns / 1ps

module requantize16#(
  parameter integer LANES           = 16,
  parameter integer ACC_BITS        = 32,
  parameter integer OUT_BITS        = 8,
  parameter integer SHIFT           = 12   
)(
  input  wire                      CLK,
  input  wire                      RESET, 
  input  wire                      en,
  input  wire [LANES*ACC_BITS-1:0] in_acc,
  input  wire [LANES*32-1:0]       bias_in,
  input  wire signed [15:0]        cfg_mult_scalar,
  input  wire        [5:0]         cfg_shift_scalar, 
  input  wire                      cfg_symmetric,
  input  wire signed [7:0]         cfg_zp_out,
  output reg  [LANES*OUT_BITS-1:0] out_q,
  output reg                       out_valid
);

  // --- 修复 1: 使用常量函数计算 CONST_ROUND_VAL，规避负移位报错 ---
  function [47:0] get_round_val;
    input integer s;
    begin
        if (s <= 0) get_round_val = 48'd0;
        else        get_round_val = 48'd1 << (s - 1);
    end
  endfunction

  localparam signed [47:0] CONST_ROUND_VAL = get_round_val(SHIFT);

  reg signed [31:0] common_zp_val;
  reg               common_en_d1, common_en_d2;

  always @(posedge CLK) begin
    if (!RESET) begin
      common_zp_val <= 0;
      common_en_d1  <= 0;
      common_en_d2  <= 0;
      out_valid     <= 0;
    end else begin
      if (en) begin
        // --- 修复 2: Verilog-2000 不支持在复制位中使用变量，手动展开符号位扩展 ---
        common_zp_val <= cfg_symmetric ? 32'sd0 : {{24{cfg_zp_out[7]}}, cfg_zp_out}; 
      end
      common_en_d1 <= en;
      common_en_d2 <= common_en_d1;
      out_valid    <= common_en_d2;
    end
  end

  function [7:0] sat_s8;
    input signed [31:0] x;
    begin
      if (x > 32'sd127)       sat_s8 = 8'd127;
      else if (x < -32'sd128) sat_s8 = 8'd128; // 修正有符号数值表示
      else                    sat_s8 = x[7:0];
    end
  endfunction

  genvar gi;
  generate
    for (gi=0; gi<LANES; gi=gi+1) begin : G
      wire signed [ACC_BITS-1:0] acc_full  = in_acc [gi*ACC_BITS + ACC_BITS-1 : gi*ACC_BITS];
      wire signed [31:0]         bias_full = bias_in[gi*32 + 31 : gi*32];

      wire signed [25:0] dsp_in_a = acc_full[31:6];
      wire signed [25:0] dsp_in_d = bias_full[31:6];

      (* use_dsp = "yes" *)
      reg signed [47:0] prod_rounded;
      always @(posedge CLK) begin
        if (!RESET) begin
          prod_rounded <= 48'sd0;
        end else if (en) begin
          // --- 修复 3: 确保所有操作数显式声明为 signed ---
          prod_rounded <= ($signed(dsp_in_a) + $signed(dsp_in_d)) * $signed(cfg_mult_scalar) 
                          + $signed(CONST_ROUND_VAL);
        end
      end

      reg signed [31:0] shift_res;
      always @(posedge CLK) begin
        if (!RESET) begin
          shift_res <= 0;
        end else if (common_en_d1) begin
          // --- 修复 4: 使用算术右移确保符号位保留 ---
          shift_res <= (prod_rounded >>> SHIFT) + common_zp_val;
        end
      end

      always @(posedge CLK) begin
        if (!RESET) begin
          out_q[gi*OUT_BITS +: OUT_BITS] <= 0;
        end else if (common_en_d2) begin
          out_q[gi*OUT_BITS +: OUT_BITS] <= sat_s8(shift_res);
        end
      end
    end
  endgenerate

endmodule