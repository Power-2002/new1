`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/13 12:21:45
// Design Name: 
// Module Name: writeback32
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
module writeback32 #(
  parameter integer ADDR_W = 16
)(
  input  wire               CLK,
  input  wire               RESETn,

  input  wire               in_valid,
  input  wire [255:0]       in_q,
  input  wire [ADDR_W-1:0]  base_addr,   // low half address; high half at base_addr+1

  output reg                wr_en,
  output reg  [ADDR_W-1:0]  wr_addr,
  output reg  [127:0]       wr_data,

  output reg                busy,
  output reg                done_pulse
);

  localparam [1:0] S_IDLE=2'd0, S_W0=2'd1, S_W1=2'd2;
  reg [1:0] state;

  reg [255:0] qbuf;
  reg [ADDR_W-1:0] abase;

  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn) begin
      wr_en <= 1'b0;
      wr_addr <= 0;
      wr_data <= 0;
      busy <= 1'b0;
      done_pulse <= 1'b0;
      state <= S_IDLE;
      qbuf <= 0;
      abase <= 0;
    end else begin
      wr_en <= 1'b0;
      done_pulse <= 1'b0;

      case (state)
        S_IDLE: begin
          busy <= 1'b0;
          if (in_valid) begin
            qbuf <= in_q;
            abase <= base_addr;
            busy <= 1'b1;
            state <= S_W0;
          end
        end

        S_W0: begin
          busy <= 1'b1;
          wr_en   <= 1'b1;
          wr_addr <= abase;
          wr_data <= qbuf[127:0];
          state <= S_W1;
        end

        S_W1: begin
          busy <= 1'b1;
          wr_en   <= 1'b1;
          wr_addr <= abase + 1'b1;
          wr_data <= qbuf[255:128];
          done_pulse <= 1'b1;
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule