`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/13 12:20:43
// Design Name: 
// Module Name: mac32_accum_core
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
module mac32_accum_core #(
  parameter integer LANES = 32,
  parameter integer ACC_W = 32
)(
  input  wire                   CLK,
  input  wire                   RESETn,

  input  wire                   acc_clear,
  input  wire                   acc_load_en,
  input  wire [LANES*ACC_W-1:0] acc_load_data,

  input  wire                   step_en,
  input  wire signed [7:0]      act_k,
  input  wire [LANES*8-1:0]     w_vec,

  output reg  [LANES*ACC_W-1:0] acc_out
);

  integer i;

  reg  signed [7:0]  w_s;
  reg  signed [15:0] prod;
  reg  signed [ACC_W-1:0] acc_s;

  (* use_dsp = "yes" *)
  always @(posedge CLK) begin
    if (!RESETn) begin
      acc_out <= {LANES*ACC_W{1'b0}};
    end else begin
      if (acc_clear) begin
        acc_out <= {LANES*ACC_W{1'b0}};
      end else if (acc_load_en) begin
        acc_out <= acc_load_data;
      end else if (step_en) begin
        for (i=0; i<LANES; i=i+1) begin
          w_s  = $signed(w_vec[i*8 +: 8]);

          prod = $signed(act_k) * $signed(w_s);

          acc_s = $signed(acc_out[i*ACC_W +: ACC_W]);

          acc_out[i*ACC_W +: ACC_W] <= acc_s + {{(ACC_W-16){prod[15]}}, prod};
        end
      end
    end
  end

endmodule
