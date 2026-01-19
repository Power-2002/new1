`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/16 11:12:20
// Design Name: 
// Module Name: conv1_featmem_from_window
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
`timescale 1ns / 1ps

module conv1_featmem_from_window #(
  parameter integer ADDR_W = 16
)(
  input  wire                 CLK,
  input  wire                 RESETn,
  input  wire                 enable,

  input  wire                 feat_rd_en,
  input  wire [ADDR_W-1:0]    feat_rd_addr,
  output reg  [127:0]         feat_rd_data,
  output reg                  feat_rd_valid,

  output reg                  win_req,
  input  wire                 win_valid,
  input  wire [215:0]         win_flat,
  input  wire                 frame_done
);

  // cache for current px
  reg [255:0] act_vec;
  reg [ADDR_W-2:0] act_px;
  reg act_valid;

  // pending request
  reg pending;
  reg [ADDR_W-1:0] pend_addr;

  wire [ADDR_W-2:0] req_px   = pend_addr[ADDR_W-1:1];
  wire              req_half = pend_addr[0];

  // response hold (IMPORTANT!)
  reg  resp_hold;
  reg [127:0] resp_data_hold;

  // debug
  reg [31:0] dbg_ctr;

  // FSM
  localparam S_IDLE     = 2'd0;
  localparam S_REQ_WIN  = 2'd1;
  localparam S_WAIT_WIN = 2'd2;
  localparam S_RESP     = 2'd3;
  reg [1:0] st;

  integer i;

  task pack_window_to_vec;
    input  [215:0] wf;
    output [255:0] v;
    reg    [255:0] tmp;
    begin
      tmp = 256'd0;
      for (i = 0; i < 27; i = i + 1) begin
        tmp[i*8 +: 8] = wf[i*8 +: 8];
      end
      v = tmp; // bytes 27..31 are 0
    end
  endtask

  always @(posedge CLK) begin
    if (!RESETn) begin
      feat_rd_data   <= 128'd0;
      feat_rd_valid  <= 1'b0;
      win_req        <= 1'b0;

      act_vec        <= 256'd0;
      act_px         <= { (ADDR_W-1){1'b0} };
      act_valid      <= 1'b0;

      pending        <= 1'b0;
      pend_addr      <= {ADDR_W{1'b0}};

      resp_hold      <= 1'b0;
      resp_data_hold <= 128'd0;

      st             <= S_IDLE;
      dbg_ctr        <= 32'd0;
    end else begin
      // default
      win_req <= 1'b0;

      // ---- VALID HOLD LOGIC (key fix) ----
      // keep feat_rd_valid asserted while resp_hold=1
      feat_rd_valid <= resp_hold;
      if (resp_hold) begin
        feat_rd_data <= resp_data_hold;
      end

      if (!enable) begin
        pending   <= 1'b0;
        resp_hold <= 1'b0;
        st        <= S_IDLE;
        dbg_ctr   <= 32'd0;
      end else begin
        // Clear the held response only when next request comes in
        // (so pw_scheduler cannot miss the valid level)
        if (feat_rd_en) begin
          resp_hold <= 1'b0;
        end

        // latch request (one outstanding)
        if (feat_rd_en && !pending) begin
          pending   <= 1'b1;
          pend_addr <= feat_rd_addr;
        end

        case (st)
          S_IDLE: begin
            if (pending) begin
              if (act_valid && (act_px == req_px)) begin
                st <= S_RESP;
              end else begin
                st <= S_REQ_WIN;
              end
            end
          end

          S_REQ_WIN: begin
            // HOLD win_req until win_valid (fetcher only samples in its IDLE)
            win_req <= 1'b1;
            st      <= S_WAIT_WIN;
          end

          S_WAIT_WIN: begin
            win_req <= 1'b1;
            if (win_valid) begin
              pack_window_to_vec(win_flat, act_vec);
              act_px    <= req_px;
              act_valid <= 1'b1;
              st        <= S_RESP;
            end
          end

          S_RESP: begin
            // Prepare response data and HOLD valid until next feat_rd_en
            resp_data_hold <= req_half ? act_vec[255:128] : act_vec[127:0];
            resp_hold      <= 1'b1;

            pending <= 1'b0;
            st      <= S_IDLE;

          end

          default: st <= S_IDLE;
        endcase

        if (frame_done) begin
          act_valid <= 1'b0;
        end
      end
    end
  end

endmodule


