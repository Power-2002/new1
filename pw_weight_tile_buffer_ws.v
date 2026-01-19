`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/13 10:13:29
// Design Name: 
// Module Name: pw_weight_tile_buffer_ws
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
module pw_weight_tile_buffer_ws #(
  parameter integer LANES = 32,
  parameter integer KT    = 32
)(
  input  wire         clk,
  input  wire         rst_n,

  input  wire         load_start,
  output reg          load_done,

  // commit: switch active bank to the bank that was most recently loaded (inactive bank)
  input  wire         bank_commit,

  input  wire         w_valid,
  input  wire [127:0] w_data,
  input  wire         w_done,

  input  wire [5:0]   rd_k,
  output wire [LANES*8-1:0] w_vec
);

  // Two banks
  reg signed [7:0] W0 [0:LANES-1][0:KT-1];
  reg signed [7:0] W1 [0:LANES-1][0:KT-1];

  // active bank selector (0->W0, 1->W1)
  reg active_bank;

  // which bank we are currently loading into
  reg load_bank;

  reg [5:0] lane;
  reg [5:0] kbase;
  reg       loading;

  integer i;

  // output vector for given k (always from active bank)
  genvar gi;
  generate
    for (gi=0; gi<LANES; gi=gi+1) begin : GEN_WV
      wire signed [7:0] w0 = W0[gi][rd_k[4:0]];
      wire signed [7:0] w1 = W1[gi][rd_k[4:0]];
      assign w_vec[gi*8 +: 8] = (active_bank) ? w1 : w0;
    end
  endgenerate

  // unpack 16 bytes per beat
  wire signed [7:0] b [0:15];
  generate
    for (gi=0; gi<16; gi=gi+1) begin : GEN_B
      assign b[gi] = w_data[gi*8 +: 8];
    end
  endgenerate

  // synchronous reset (helps RAM inference / keeps style consistent with the rest)
  always @(posedge clk) begin
    if (!rst_n) begin
      loading     <= 1'b0;
      load_done   <= 1'b0;
      lane        <= 6'd0;
      kbase       <= 6'd0;
      active_bank <= 1'b0;
      load_bank   <= 1'b1; // inactive initially
    end else begin
      // one-cycle pulse by default
      load_done <= 1'b0;

      // switch active bank when scheduler commits
      if (bank_commit) begin
        active_bank <= load_bank;
      end

      // start loading into inactive bank
      if (load_start && !loading) begin
        loading   <= 1'b1;
        lane      <= 6'd0;
        kbase     <= 6'd0;
        load_bank <= ~active_bank; // write into inactive bank
      end

      if (loading && w_valid) begin
        // Fill 16 consecutive cin values for current lane (cin-fast packing)
        for (i=0; i<16; i=i+1) begin
          if (lane < LANES) begin
            if (load_bank) begin
              W1[lane][(kbase + i) & 6'h1F] <= b[i];
            end else begin
              W0[lane][(kbase + i) & 6'h1F] <= b[i];
            end
          end
        end

        // advance within lane: 0->16->0 and lane++
        if (kbase == 6'd0) begin
          kbase <= 6'd16;
        end else begin
          kbase <= 6'd0;
          lane  <= lane + 6'd1;
        end
      end

      if (loading && w_done) begin
        loading   <= 1'b0;
        load_done <= 1'b1; // pulse
      end
    end
  end

endmodule
