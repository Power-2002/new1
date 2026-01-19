`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/13 10:14:53
// Design Name: 
// Module Name: pw_psum_tile_buffer
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
module pw_psum_tile_buffer #(
  parameter integer DEPTH = 128,
  parameter integer LANES = 32,
  parameter integer ACC_W = 32
)(
  input  wire                 clk,
  input  wire                 rst_n,

  input  wire                 rd_en,
  input  wire [$clog2(DEPTH)-1:0] rd_addr,
  output reg  [LANES*ACC_W-1:0]   rd_data,
  output reg                  rd_valid,

  input  wire                 wr_en,
  input  wire [$clog2(DEPTH)-1:0] wr_addr,
  input  wire [LANES*ACC_W-1:0]   wr_data
);

  localparam DATA_W = LANES * ACC_W;

  // 定义存储器
  reg [DATA_W-1:0] mem [0:DEPTH-1];

  // 定义控制寄存器
  reg rd_en_d1;
  reg [$clog2(DEPTH)-1:0] rd_addr_d1;


  always @(posedge clk) begin
      // 写操作
      if (wr_en) begin
          mem[wr_addr] <= wr_data;
      end

      if (rd_en_d1) begin
          rd_data <= mem[rd_addr_d1];
      end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_en_d1   <= 1'b0;
      rd_addr_d1 <= {$clog2(DEPTH){1'b0}};
      rd_valid   <= 1'b0;
    end else begin
      // 这里的逻辑保持不变，产生读地址和读使能的延迟
      rd_en_d1   <= rd_en;
      rd_addr_d1 <= rd_addr;
      
      // 输出 valid 信号跟随流水线
      rd_valid   <= rd_en_d1;
    end
  end

endmodule
