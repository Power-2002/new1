`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/13 10:19:21
// Design Name: 
// Module Name: pw_scheduler_ws_full
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

module pw_scheduler_ws_full #(
  parameter integer ADDR_W = 16,
  parameter integer PIXEL_TILE_SIZE = 128,
  parameter integer ACT_FIFO_DEPTH  = 16,
  parameter integer DBG_PRINT_MOD   = 20000
)(
  input  wire                 CLK,
  input  wire                 RESETn,
  input  wire                 start,
  output reg                  done,

  input  wire [10:0]          cin,
  input  wire [10:0]          cout,
  input  wire [7:0]           img_w,
  input  wire [7:0]           img_h,
  input  wire [ADDR_W-1:0]    w_base_in,

  input  wire signed [15:0]   quant_M,
  input  wire [5:0]           quant_s,
  input  wire signed [7:0]    quant_zp,

  input  wire [1023:0]        bias_vec,

  // weight channel
  output reg                  weight_req,
  input  wire                 weight_grant,
  output reg  [ADDR_W-1:0]    weight_base,
  output reg  [16:0]          weight_count,
  input  wire                 weight_valid,
  input  wire [127:0]         weight_data,
  input  wire                 weight_done,

  // feature read (exclusive during L2)
  output reg                  feat_rd_en,
  output reg  [15:0]          feat_rd_addr,
  input  wire [127:0]         feat_rd_data,
  input  wire                 feat_rd_valid,

  // feature write (1 lane, 128b)  -> driven ONLY by writeback module
  output wire                 feat_wr_en,
  output wire [15:0]          feat_wr_addr,
  output wire [127:0]         feat_wr_data
);

  // -------------------------
  // derived sizes
  // -------------------------
  wire [15:0] total_px = img_w * img_h;
  wire [10:0] cin_tiles  = (cin  + 11'd31) >> 5;
  wire [10:0] cout_tiles = (cout + 11'd31) >> 5;
  wire [15:0] words_per_px_out = (cout + 11'd15) >> 4; // ceil(cout/16)

  function automatic [15:0] in_addr(input [15:0] px, input [10:0] cin_t, input half);
    begin
      in_addr = px * (cin_tiles * 2) + (cin_t * 2) + half;
    end
  endfunction

  function automatic [15:0] out_base_addr(input [15:0] px, input [10:0] cout_t);
    begin
      out_base_addr = px * words_per_px_out + (cout_t * 2);
    end
  endfunction

  function automatic [ADDR_W-1:0] wtile_base(input [10:0] cout_t, input [10:0] cin_t);
    reg [31:0] idx;
    begin
      idx = (cout_t * cin_tiles) + cin_t;
      wtile_base = w_base_in + (idx * 32'd64);
    end
  endfunction

  // -------------------------
  // weight tile buffer (single)
  // -------------------------
  reg  wt_load_start;
  wire wt_load_done;
  reg  [5:0] wt_rd_k;
  wire [255:0] wt_w_vec;

  pw_weight_tile_buffer_ws u_wbuf (
    .clk       (CLK),
    .rst_n     (RESETn),
    .load_start(wt_load_start),
    .load_done (wt_load_done),
    .w_valid   (weight_valid),
    .w_data    (weight_data),
    .w_done    (weight_done),
    .rd_k      (wt_rd_k),
    .w_vec     (wt_w_vec)
  );

  // -------------------------
  // psum tile buffer (store 128 pixels partial sums)
  // -------------------------
  localparam integer TDEPTH = PIXEL_TILE_SIZE;
  localparam integer TAW    = $clog2(TDEPTH);

  reg                   psum_rd_en;
  reg  [TAW-1:0]         psum_rd_addr;
  wire [1023:0]          psum_rd_data;
  wire                  psum_rd_valid;

  reg                   psum_wr_en;
  reg  [TAW-1:0]         psum_wr_addr;
  reg  [1023:0]          psum_wr_data;

  pw_psum_tile_buffer #(
    .DEPTH(TDEPTH),
    .LANES(32),
    .ACC_W(32)
  ) u_psum (
    .clk     (CLK),
    .rst_n   (RESETn),
    .rd_en   (psum_rd_en),
    .rd_addr (psum_rd_addr),
    .rd_data (psum_rd_data),
    .rd_valid(psum_rd_valid),
    .wr_en   (psum_wr_en),
    .wr_addr (psum_wr_addr),
    .wr_data (psum_wr_data)
  );

  // -------------------------
  // Shared compute core (reuse)
  // -------------------------
  reg        acc_clear, acc_load_en, step_en;
  reg [1023:0] acc_load_data;
  reg signed [7:0] mac_act_k;
  wire [1023:0] acc_out;

  mac32_accum_core u_mac (
    .CLK(CLK),
    .RESETn(RESETn),
    .acc_clear(acc_clear),
    .acc_load_en(acc_load_en),
    .acc_load_data(acc_load_data),
    .step_en(step_en),
    .act_k(mac_act_k),
    .w_vec(wt_w_vec),
    .acc_out(acc_out)
  );

  // -------------------------
  // Shared quant (reuse)
  // -------------------------
  reg  q_en;
  wire [255:0] q_out;
  wire         q_valid;

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

  // -------------------------
  // addr_fifo to align q_en -> q_valid base_addr
  // -------------------------
  localparam integer ADDR_FIFO_DEPTH = 4;
  localparam integer ADDR_FIFO_AW    = $clog2(ADDR_FIFO_DEPTH);

  reg [15:0] addr_fifo [0:ADDR_FIFO_DEPTH-1];
  reg [ADDR_FIFO_AW-1:0] addr_wptr, addr_rptr;
  reg [ADDR_FIFO_AW:0]   addr_count;

  wire addr_full  = (addr_count == ADDR_FIFO_DEPTH);
  wire addr_empty = (addr_count == 0);
  wire [15:0] addr_head = addr_fifo[addr_rptr];

  // We push base_addr when q_en is issued (same cycle).
  // We pop base_addr when q_valid arrives.
  reg addr_push;
  reg [15:0] addr_push_data;
  reg addr_pop;

  wire do_addr_push = addr_push && !addr_full;
  wire do_addr_pop  = addr_pop  && !addr_empty;

  // -------------------------
  // Writeback queue (depth=2) + shared writeback
  // -------------------------
  reg [1:0] qf_count;
  reg [255:0] qf_data0, qf_data1;
  reg [15:0]  qf_addr0, qf_addr1; // base addr (low half)

  wire qf_full  = (qf_count == 2);
  wire qf_empty = (qf_count == 0);

  // push on q_valid if space AND addr_fifo has an entry
  wire qf_push = q_valid && !qf_full && !addr_empty;

  // writeback module pop handshake
  reg        wb_in_valid;
  reg [255:0] wb_in_q;
  reg [15:0]  wb_base_addr;

  wire wb_busy, wb_done_pulse;
  wire wb_wr_en;
  wire [15:0]  wb_wr_addr;
  wire [127:0] wb_wr_data;

  writeback32 #(.ADDR_W(16)) u_wb (
    .CLK(CLK),
    .RESETn(RESETn),
    .in_valid(wb_in_valid),
    .in_q(wb_in_q),
    .base_addr(wb_base_addr),
    .wr_en(wb_wr_en),
    .wr_addr(wb_wr_addr),
    .wr_data(wb_wr_data),
    .busy(wb_busy),
    .done_pulse(wb_done_pulse)
  );

  assign feat_wr_en   = wb_wr_en;
  assign feat_wr_addr = wb_wr_addr;
  assign feat_wr_data = wb_wr_data;

  // -------------------------
  // Streaming ACT FIFO
  // -------------------------
  localparam integer FIFO_AW = $clog2(ACT_FIFO_DEPTH);

  reg [15:0]  act_fifo_px   [0:ACT_FIFO_DEPTH-1];
  reg [255:0] act_fifo_vec  [0:ACT_FIFO_DEPTH-1];

  reg [FIFO_AW-1:0] fifo_wptr, fifo_rptr;
  reg [FIFO_AW:0]   fifo_count;

  wire fifo_full  = (fifo_count == ACT_FIFO_DEPTH);
  wire fifo_empty = (fifo_count == 0);

  wire [15:0]  fifo_px_head  = act_fifo_px[fifo_rptr];
  wire [255:0] fifo_vec_head = act_fifo_vec[fifo_rptr];

  reg fifo_push;
  reg [15:0]  fifo_push_px;
  reg [255:0] fifo_push_vec;

  reg fifo_pop;
  reg fifo_clear;

  wire do_push = fifo_push && !fifo_full;
  wire do_pop  = fifo_pop  && !fifo_empty;

  // prefetch engine
  reg        pf_active;
  reg [15:0] px_prefetch;
  reg [127:0] pf_low128;

  localparam [2:0] PF_IDLE  = 3'd0;
  localparam [2:0] PF_RD0   = 3'd1;
  localparam [2:0] PF_WAIT0 = 3'd2;
  localparam [2:0] PF_RD1   = 3'd3;
  localparam [2:0] PF_WAIT1 = 3'd4;
  reg [2:0] pf_state;

  reg        pf_restart;
  reg [15:0] pf_restart_px;

  // -------------------------
  // local regs
  // -------------------------
  reg [10:0] cout_t;
  reg [10:0] cin_t;
  reg [15:0] px_tile_base;
  reg [7:0]  t_in_tile;
  reg [15:0] px;
  reg [5:0]  k;

  reg [4:0]  state;

  wire [15:0] tile_rem = total_px - px_tile_base;
  wire [7:0]  tile_len = (tile_rem >= PIXEL_TILE_SIZE) ? PIXEL_TILE_SIZE[7:0] : tile_rem[7:0];

  wire is_first_cin = (cin_t == 0);
  wire is_last_cin  = (cin_t == (cin_tiles - 1));

  function automatic signed [7:0] act_k(input [255:0] v, input [5:0] kk);
    begin
      act_k = v[kk*8 +: 8];
    end
  endfunction

  // -------------------------
  // FSM encoding
  // -------------------------
  localparam [4:0] S_IDLE      = 5'd0;
  localparam [4:0] S_WREQ      = 5'd1;
  localparam [4:0] S_WLOAD     = 5'd2;
  localparam [4:0] S_TILE_INIT = 5'd3;
  localparam [4:0] S_PSUM_RD   = 5'd4;
  localparam [4:0] S_PSUM_WAIT = 5'd5;
  localparam [4:0] S_WAIT_ACT  = 5'd6;
  localparam [4:0] S_MAC       = 5'd7;
  localparam [4:0] S_PSUM_WR   = 5'd8;
  localparam [4:0] S_Q_EN      = 5'd9;
  localparam [4:0] S_NEXT_PX   = 5'd10;
  localparam [4:0] S_NEXT_PXT  = 5'd11;
  localparam [4:0] S_NEXT_CT   = 5'd12;
  localparam [4:0] S_NEXT_COT  = 5'd13;
  localparam [4:0] S_DONE      = 5'd14;

  // -------------------------
  // Main sequential process
  // -------------------------
  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn) begin
      done <= 1'b0;

      weight_req   <= 1'b0;
      weight_base  <= {ADDR_W{1'b0}};
      weight_count <= 17'd0;

      feat_rd_en   <= 1'b0;
      feat_rd_addr <= 16'd0;

      wt_load_start <= 1'b0;

      psum_rd_en   <= 1'b0;
      psum_rd_addr <= {TAW{1'b0}};
      psum_wr_en   <= 1'b0;
      psum_wr_addr <= {TAW{1'b0}};
      psum_wr_data <= 1024'd0;

      // mac/quant control
      acc_clear <= 1'b0;
      acc_load_en <= 1'b0;
      acc_load_data <= 0;
      step_en <= 1'b0;
      mac_act_k <= 0;
      q_en <= 1'b0;

      // act fifo
      fifo_wptr  <= {FIFO_AW{1'b0}};
      fifo_rptr  <= {FIFO_AW{1'b0}};
      fifo_count <= 0;
      fifo_push  <= 1'b0;
      fifo_push_px <= 16'd0;
      fifo_push_vec<= 256'd0;
      fifo_pop   <= 1'b0;
      fifo_clear <= 1'b0;

      // prefetch
      pf_active <= 1'b0;
      px_prefetch <= 16'd0;
      pf_low128 <= 128'd0;
      pf_state <= PF_IDLE;
      pf_restart <= 1'b0;
      pf_restart_px <= 16'd0;

      // compute regs
      cout_t <= 0;
      cin_t  <= 0;
      px_tile_base <= 0;
      t_in_tile <= 0;
      px <= 0;
      k  <= 0;
      wt_rd_k <= 0;
      state <= S_IDLE;

      // addr_fifo
      addr_wptr <= 0;
      addr_rptr <= 0;
      addr_count <= 0;
      addr_push <= 1'b0;
      addr_push_data <= 0;
      addr_pop <= 1'b0;

      // q_fifo
      qf_count <= 0;
      qf_data0 <= 0; qf_data1 <= 0;
      qf_addr0 <= 0; qf_addr1 <= 0;

      // wb launch
      wb_in_valid <= 1'b0;
      wb_in_q <= 0;
      wb_base_addr <= 0;

    end else begin
      // defaults
      done <= 1'b0;

      weight_req    <= 1'b0;
      wt_load_start <= 1'b0;

      feat_rd_en <= 1'b0;

      psum_rd_en <= 1'b0;
      psum_wr_en <= 1'b0;

      // mac/quant defaults
      acc_clear <= 1'b0;
      acc_load_en <= 1'b0;
      step_en <= 1'b0;
      q_en <= 1'b0;

      // fifo defaults
      fifo_push  <= 1'b0;
      fifo_pop   <= 1'b0;
      fifo_clear <= 1'b0;

      // prefetch restart defaults
      pf_restart <= 1'b0;
      pf_restart_px <= 16'd0;

      // addr_fifo defaults
      addr_push <= 1'b0;
      addr_push_data <= 16'd0;
      addr_pop <= 1'b0;

      // wb launch default
      wb_in_valid <= 1'b0;

      // -------------------------
      // Prefetch restart action
      // -------------------------
      if (pf_restart) begin
        px_prefetch <= pf_restart_px;
        pf_state    <= PF_IDLE;
      end

      // -------------------------
      // Prefetch engine
      // -------------------------
      if (pf_active) begin
        case (pf_state)
          PF_IDLE: begin
            if (!fifo_full && (px_prefetch < (px_tile_base + tile_len))) begin
              pf_state <= PF_RD0;
            end
          end

          PF_RD0: begin
            feat_rd_en   <= 1'b1;
            feat_rd_addr <= in_addr(px_prefetch, cin_t, 1'b0);
            pf_state <= PF_WAIT0;
          end

          PF_WAIT0: begin
            if (feat_rd_valid) begin
              pf_low128 <= feat_rd_data;
              pf_state <= PF_RD1;
            end
          end

          PF_RD1: begin
            feat_rd_en   <= 1'b1;
            feat_rd_addr <= in_addr(px_prefetch, cin_t, 1'b1);
            pf_state <= PF_WAIT1;
          end

          PF_WAIT1: begin
            if (feat_rd_valid) begin
              fifo_push     <= 1'b1;
              fifo_push_px  <= px_prefetch;
              fifo_push_vec <= {feat_rd_data, pf_low128};
              px_prefetch   <= px_prefetch + 1'b1;
              pf_state      <= PF_IDLE;
            end
          end
          default: pf_state <= PF_IDLE;
        endcase
      end

      // -------------------------
      // Main compute FSM
      // -------------------------
      wt_rd_k <= k;

      case (state)
        S_IDLE: begin
          if (start) begin
            cout_t <= 0;
            cin_t  <= 0;
            px_tile_base <= 0;
            state <= S_WREQ;
          end
        end

        S_WREQ: begin
          weight_req   <= 1'b1;
          weight_base  <= wtile_base(cout_t, cin_t);
          weight_count <= 17'd64;
          if (weight_grant) begin
            wt_load_start <= 1'b1;
            state <= S_WLOAD;
          end
        end

        S_WLOAD: begin
          if (wt_load_done) begin
            px_tile_base <= 0;
            state <= S_TILE_INIT;
          end
        end

        S_TILE_INIT: begin
          t_in_tile <= 0;

          fifo_clear <= 1'b1;
          pf_active <= 1'b1;
          pf_restart <= 1'b1;
          pf_restart_px <= px_tile_base;

          state <= S_PSUM_RD;
        end

        S_PSUM_RD: begin
          if (t_in_tile < tile_len) begin
            px <= px_tile_base + t_in_tile;

            if (is_first_cin) begin
              acc_clear <= 1'b1;
              state <= S_WAIT_ACT;
            end else begin
              psum_rd_en   <= 1'b1;
              psum_rd_addr <= t_in_tile[TAW-1:0];
              state <= S_PSUM_WAIT;
            end
          end else begin
            state <= S_NEXT_PXT;
          end
        end

        S_PSUM_WAIT: begin
          if (psum_rd_valid) begin
            acc_load_en   <= 1'b1;
            acc_load_data <= psum_rd_data;
            state <= S_WAIT_ACT;
          end
        end

        S_WAIT_ACT: begin
          if (!fifo_empty) begin
            if (fifo_px_head == px) begin
              k <= 0;
              state <= S_MAC;
            end else if (fifo_px_head < px) begin
              fifo_pop <= 1'b1;
            end else begin
              fifo_clear <= 1'b1;
              pf_active <= 1'b1;
              pf_restart <= 1'b1;
              pf_restart_px <= px;
            end
          end
        end

        S_MAC: begin
          wt_rd_k <= k;

          mac_act_k <= act_k(fifo_vec_head, k);
          step_en   <= 1'b1;

          if (k == 6'd31) begin
            fifo_pop <= 1'b1;

            if (!is_last_cin) state <= S_PSUM_WR;
            else              state <= S_Q_EN;
          end else begin
            k <= k + 1'b1;
          end
        end

        S_PSUM_WR: begin
          psum_wr_data <= acc_out;
          psum_wr_en   <= 1'b1;
          psum_wr_addr <= t_in_tile[TAW-1:0];
          state <= S_NEXT_PX;
        end

        S_Q_EN: begin
          // For correctness with overlap, enforce:
          // 1) addr_fifo must have space (push base_addr here)
          // 2) q_fifo must have space (eventual q_valid will push data)
          if (!addr_full && !qf_full) begin
            // launch quant
            q_en <= 1'b1;

            // push base_addr aligned with THIS q_en
            addr_push <= 1'b1;
            addr_push_data <= out_base_addr(px, cout_t);

            state <= S_NEXT_PX;
          end
        end

        S_NEXT_PX: begin
          t_in_tile <= t_in_tile + 1'b1;
          state <= S_PSUM_RD;
        end

        S_NEXT_PXT: begin
          pf_active <= 1'b0;

          if (px_tile_base + PIXEL_TILE_SIZE < total_px) begin
            px_tile_base <= px_tile_base + PIXEL_TILE_SIZE;
            state <= S_TILE_INIT;
          end else begin
            state <= S_NEXT_CT;
          end
        end

        S_NEXT_CT: begin
          if (cin_t + 1 < cin_tiles) begin
            cin_t <= cin_t + 1'b1;
            state <= S_WREQ;
          end else begin
            cin_t <= 0;
            state <= S_NEXT_COT;
          end
        end

        S_NEXT_COT: begin
          if (cout_t + 1 < cout_tiles) begin
            cout_t <= cout_t + 1'b1;
            state <= S_WREQ;
          end else begin
            state <= S_DONE;
          end
        end

        S_DONE: begin
          // wait until all queued writes drained
          if (qf_empty && addr_empty && !wb_busy) begin
            done <= 1'b1;
            state <= S_IDLE;
          end
        end

        default: state <= S_IDLE;
      endcase

      // -------------------------
      // addr_fifo apply
      // -------------------------
      if (do_addr_push) begin
        addr_fifo[addr_wptr] <= addr_push_data;
        addr_wptr <= addr_wptr + 1'b1;
      end
      if (do_addr_pop) begin
        addr_rptr <= addr_rptr + 1'b1;
      end
      case ({do_addr_push, do_addr_pop})
        2'b10: addr_count <= addr_count + 1'b1;
        2'b01: addr_count <= addr_count - 1'b1;
        default: addr_count <= addr_count;
      endcase

      // -------------------------
      // On q_valid: pair with addr_fifo head and push into q_fifo
      // -------------------------
      // Pop addr when we actually push q into q_fifo (ensures 1:1 pairing)
      if (qf_push) begin
        addr_pop <= 1'b1;

        if (qf_count == 0) begin
          qf_data0 <= q_out;
          qf_addr0 <= addr_head;
          qf_count <= 1;
        end else if (qf_count == 1) begin
          qf_data1 <= q_out;
          qf_addr1 <= addr_head;
          qf_count <= 2;
        end
      end

      // -------------------------
      // Launch writeback when idle and q_fifo not empty
      // -------------------------
      if (!wb_busy && !qf_empty) begin
        wb_in_valid  <= 1'b1;
        wb_in_q      <= qf_data0;
        wb_base_addr <= qf_addr0;

        // pop q_fifo (shift)
        if (qf_count == 1) begin
          qf_count <= 0;
        end else begin
          qf_data0 <= qf_data1;
          qf_addr0 <= qf_addr1;
          qf_count <= 1;
        end
      end

      // -------------------------
      // ACT FIFO apply (single place updates wptr/rptr/count)
      // -------------------------
      if (fifo_clear) begin
        fifo_wptr  <= 0;
        fifo_rptr  <= 0;
        fifo_count <= 0;
      end else begin
        if (do_push) begin
          act_fifo_px[fifo_wptr]  <= fifo_push_px;
          act_fifo_vec[fifo_wptr] <= fifo_push_vec;
          fifo_wptr <= fifo_wptr + 1'b1;
        end

        if (do_pop) begin
          fifo_rptr <= fifo_rptr + 1'b1;
        end

        case ({do_push, do_pop})
          2'b10: fifo_count <= fifo_count + 1'b1;
          2'b01: fifo_count <= fifo_count - 1'b1;
          default: fifo_count <= fifo_count;
        endcase
      end
    end
  end

`ifndef SYNTHESIS
  integer cyc;
  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn) begin
      cyc <= 0;
    end else begin
      cyc <= cyc + 1;
      if ((cyc % DBG_PRINT_MOD) == 0) begin
        $display("[%0t][cyc=%0d] PW state=%0d cout_t=%0d cin_t=%0d px_tile_base=%0d t=%0d px=%0d k=%0d | wt_load_done=%b | pf_active=%b pf_state=%0d px_prefetch=%0d | fifo_cnt=%0d full=%b empty=%b head_px=%0d | addr_cnt=%0d addr_full=%b | qf_cnt=%0d qf_full=%b | wb_busy=%b",
          $time, cyc, state, cout_t, cin_t, px_tile_base, t_in_tile, px, k,
          wt_load_done,
          pf_active, pf_state, px_prefetch,
          fifo_count, fifo_full, fifo_empty, (fifo_empty ? 16'hFFFF : fifo_px_head),
          addr_count, addr_full,
          qf_count, qf_full,
          wb_busy);
      end
    end
  end
`endif

endmodule