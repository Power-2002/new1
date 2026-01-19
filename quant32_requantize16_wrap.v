`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/13 12:21:12
// Design Name: 
// Module Name: quant32_requantize16_wrap
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

module quant32_requantize16_wrap (
  input  wire               CLK,
  input  wire               RESETn,

  input  wire               en,
  input  wire [1023:0]      in_acc,
  input  wire [1023:0]      bias_vec,

  input  wire signed [15:0] quant_M,
  input  wire [5:0]         quant_s,
  input  wire signed [7:0]  quant_zp,

  output wire [255:0]       out_q,
  output wire               out_valid
);

  requantize16 #(.LANES(32)) u_q (
    .CLK(CLK),
    .RESET(RESETn),
    .en(en),
    .in_acc(in_acc),
    .bias_in(bias_vec),
    .cfg_mult_scalar(quant_M),
   // .cfg_shift_scalar(quant_s),
    .cfg_symmetric(1'b0),
    .cfg_zp_out(quant_zp),
    .out_q(out_q),
    .out_valid(out_valid)
  );

endmodule