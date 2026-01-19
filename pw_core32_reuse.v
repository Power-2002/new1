`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/13 14:57:07
// Design Name: 
// Module Name: pw_core32_reuse
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
`include "mobilenet_defines.vh"

module pw_core32_reuse #(
  parameter integer ADDR_W = 16,
  parameter integer PIXEL_TILE_SIZE = 128
)(
  input  wire                 CLK,
  input  wire                 RESETn,

  // psum tile buffer control
  input  wire                 psum_do_read,
  input  wire [7:0]           t_in_tile,
  output wire                 psum_rd_valid,
  output wire [1023:0]        psum_rd_data,

  input  wire                 psum_do_write,
  input  wire [1023:0]        psum_wr_data,

  // MAC control
  input  wire                 mac_clear_pulse,
  input  wire                 mac_load_pulse,
  input  wire [1023:0]        mac_load_data,
  input  wire                 mac_step_en,
  input  wire signed [7:0]    mac_act_k,
  input  wire [5:0]           mac_k,

  // expose acc_out for scheduler to write psum (synth-safe)
  output wire [1023:0]        acc_out,

  // weight loader stream
  input  wire                 wt_load_start,
  output wire                 wt_load_done,

  input  wire                 wt_bank_commit,
  input  wire                 weight_valid,
  input  wire [127:0]         weight_data,
  input  wire                 weight_done,

  // quant config + bias
  input  wire signed [15:0]   quant_M,
  input  wire [5:0]           quant_s,
  input  wire signed [7:0]    quant_zp,
  input  wire [1023:0]        bias_vec,

  // quant trigger and output
  input  wire                 q_en,
  output wire [255:0]         q_out,
  output wire                 q_valid
);

  // weight tile buffer
  wire [255:0] wt_w_vec;
  pw_weight_tile_buffer_ws u_wbuf (
    .clk       (CLK),
    .rst_n     (RESETn),
    .load_start(wt_load_start),
    .load_done (wt_load_done),
    .bank_commit(wt_bank_commit),
    .w_valid   (weight_valid),
    .w_data    (weight_data),
    .w_done    (weight_done),
    .rd_k      (mac_k),
    .w_vec     (wt_w_vec)
  );

  // psum tile buffer
  localparam integer TDEPTH = PIXEL_TILE_SIZE;
  localparam integer TAW    = $clog2(TDEPTH);

  reg                  psum_rd_en_r;
  reg  [TAW-1:0]        psum_rd_addr_r;
  wire [1023:0]         psum_rd_data_i;
  wire                 psum_rd_valid_i;

  reg                  psum_wr_en_r;
  reg  [TAW-1:0]        psum_wr_addr_r;
  reg  [1023:0]         psum_wr_data_r;

  always @(*) begin
    psum_rd_en_r   = psum_do_read;
    psum_rd_addr_r = t_in_tile[TAW-1:0];

    psum_wr_en_r   = psum_do_write;
    psum_wr_addr_r = t_in_tile[TAW-1:0];
    psum_wr_data_r = psum_wr_data;
  end

  pw_psum_tile_buffer #(
    .DEPTH(TDEPTH),
    .LANES(32),
    .ACC_W(32)
  ) u_psum (
    .clk     (CLK),
    .rst_n   (RESETn),
    .rd_en   (psum_rd_en_r),
    .rd_addr (psum_rd_addr_r),
    .rd_data (psum_rd_data_i),
    .rd_valid(psum_rd_valid_i),
    .wr_en   (psum_wr_en_r),
    .wr_addr (psum_wr_addr_r),
    .wr_data (psum_wr_data_r)
  );

  assign psum_rd_valid = psum_rd_valid_i;
  assign psum_rd_data  = psum_rd_data_i;

  // MAC core (reuse)
  mac32_accum_core u_mac (
    .CLK(CLK),
    .RESETn(RESETn),
    .acc_clear(mac_clear_pulse),
    .acc_load_en(mac_load_pulse),
    .acc_load_data(mac_load_data),
    .step_en(mac_step_en),
    .act_k(mac_act_k),
    .w_vec(wt_w_vec),
    .acc_out(acc_out)
  );

  // Quant (reuse)
  quant32_requantize16_wrap u_quant (
    .CLK(CLK),
    .RESETn(RESETn),
    .en(q_en),
    .in_acc(acc_out),
    .bias_vec(bias_vec),
    .quant_M(quant_M),
    .quant_s(quant_s),
    .quant_zp(quant_zp),
    .out_q(q_out),
    .out_valid(q_valid)
  );

endmodule
