`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/13 14:57:58
// Design Name: 
// Module Name: pw_scheduler_ws_4full
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
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/13 14:57:58
// Design Name: 
// Module Name: pw_scheduler_ws_4full
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
module pw_scheduler_ws_4full #(
  parameter integer ADDR_W = 16,
  parameter integer PIXEL_TILE_SIZE = 128,
  parameter integer ACT_FIFO_DEPTH  = 16
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

  // shared weight channel (serialized)
  output reg                  weight_req,
  input  wire                 weight_grant,
  output reg  [ADDR_W-1:0]    weight_base,
  output reg  [16:0]          weight_count,
  input  wire                 weight_valid,
  input  wire [127:0]         weight_data,
  input  wire                 weight_done,

  // feature read
  output reg                  feat_rd_en,
  output reg  [15:0]          feat_rd_addr,
  input  wire [127:0]         feat_rd_data,
  input  wire                 feat_rd_valid,

  // feature write (4 lanes)
  output wire [3:0]           feat_wr_en,
  output wire [63:0]          feat_wr_local_addr_vec,
  output wire [511:0]         feat_wr_data_vec
);

  // -------------------------
  // derived sizes / addressing
  // -------------------------
  wire [15:0] total_px = img_w * img_h;
  wire [10:0] cin_tiles  = (cin  + 11'd31) >> 5;
  wire [10:0] cout_tiles = (cout + 11'd31) >> 5;
  wire [15:0] words_per_px_out = (cout + 11'd15) >> 4;

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

  function automatic signed [7:0] act_k(input [255:0] v, input [5:0] kk);
    begin
      act_k = v[kk*8 +: 8];
    end
  endfunction

  // -------------------------
  // state encoding
  // -------------------------
  localparam [4:0] S_IDLE      = 5'd0;
  localparam [4:0] S_WL_REQ    = 5'd1;
  localparam [4:0] S_WL_WAIT   = 5'd2;
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
  localparam [4:0] S_NEXT_CG   = 5'd13;
  localparam [4:0] S_DONE      = 5'd14;

  reg [4:0] state;

  // -------------------------
  // PERF counters (Verilog-2001 safe)
  // -------------------------
  reg [31:0] perf_total;
  reg [31:0] perf_wload;
  reg [31:0] perf_psum_wait;
  reg [31:0] perf_wait_act;
  reg [31:0] perf_mac;
  reg [31:0] perf_qstall;
  reg [31:0] perf_wb_busy;
  reg [31:0] perf_fifo_empty;

  reg q_can_issue;
  integer pi;
  integer ii;

  // -------------------------
  // act fifo + prefetch
  // -------------------------
  localparam integer FIFO_AW = $clog2(ACT_FIFO_DEPTH);

  reg [15:0]  act_fifo_px   [0:ACT_FIFO_DEPTH-1];
  (* ram_style = "block" *) reg [255:0] act_fifo_vec  [0:ACT_FIFO_DEPTH-1];
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
  // indices
  // -------------------------
  reg [10:0] cout_grp; // 0,4,8...
  reg [10:0] cin_t;
  reg [15:0] px_tile_base;
  reg [7:0]  t_in_tile;
  reg [15:0] px;
  reg [5:0]  k;

  wire [15:0] tile_rem = total_px - px_tile_base;
  wire [7:0]  tile_len = (tile_rem >= PIXEL_TILE_SIZE) ? PIXEL_TILE_SIZE[7:0] : tile_rem[7:0];

  wire is_first_cin = (cin_t == 0);
  wire is_last_cin  = (cin_t == (cin_tiles - 1));

  wire [10:0] core_cout_t [0:3];
  wire [3:0]  core_valid;
  assign core_cout_t[0] = cout_grp + 0;
  assign core_cout_t[1] = cout_grp + 1;
  assign core_cout_t[2] = cout_grp + 2;
  assign core_cout_t[3] = cout_grp + 3;

  assign core_valid[0] = (core_cout_t[0] < cout_tiles);
  assign core_valid[1] = (core_cout_t[1] < cout_tiles);
  assign core_valid[2] = (core_cout_t[2] < cout_tiles);
  assign core_valid[3] = (core_cout_t[3] < cout_tiles);

  // -------------------------
  // 4 cores
  // -------------------------
  reg  [3:0] core_psum_do_read;
  reg  [3:0] core_psum_do_write;

  wire [3:0] core_psum_rd_valid;
  wire [4*1024-1:0] core_psum_rd_data_flat;
  reg  [4*1024-1:0] core_psum_wr_data_flat;

  reg        mac_clear_pulse;
  reg        mac_load_pulse;
  reg        mac_step_en;
  reg signed [7:0] mac_act_k;
  reg  [5:0] mac_k;

  reg  [4*1024-1:0] mac_load_data_flat;
  wire [4*1024-1:0] core_acc_out_flat;

  reg  [3:0] core_q_en;
  wire [3:0] core_q_valid;
  wire [4*256-1:0] core_q_out_flat;

  reg  [3:0] core_wt_load_start;
  wire [3:0] core_wt_load_done;

  genvar gi;
  generate
    for (gi=0; gi<4; gi=gi+1) begin : GEN_CORE
      pw_core32_reuse #(
        .ADDR_W(ADDR_W),
        .PIXEL_TILE_SIZE(PIXEL_TILE_SIZE)
      ) u_core (
        .CLK(CLK),
        .RESETn(RESETn),

        .psum_do_read(core_psum_do_read[gi]),
        .t_in_tile(t_in_tile),
        .psum_rd_valid(core_psum_rd_valid[gi]),
        .psum_rd_data(core_psum_rd_data_flat[gi*1024 +: 1024]),

        .psum_do_write(core_psum_do_write[gi]),
        .psum_wr_data(core_psum_wr_data_flat[gi*1024 +: 1024]),

        .mac_clear_pulse(mac_clear_pulse),
        .mac_load_pulse(mac_load_pulse),
        .mac_load_data(mac_load_data_flat[gi*1024 +: 1024]),
        .mac_step_en(mac_step_en),
        .mac_act_k(mac_act_k),
        .mac_k(mac_k),

        .acc_out(core_acc_out_flat[gi*1024 +: 1024]),

        .wt_load_start(core_wt_load_start[gi]),
        .wt_load_done(core_wt_load_done[gi]),
        .weight_valid(weight_valid),
        .weight_data(weight_data),
        .weight_done(weight_done),

        .quant_M(quant_M),
        .quant_s(quant_s),
        .quant_zp(quant_zp),
        .bias_vec(bias_vec),

        .q_en(core_q_en[gi]),
        .q_out(core_q_out_flat[gi*256 +: 256]),
        .q_valid(core_q_valid[gi])
      );
    end
  endgenerate

  // -------------------------
  // per-core addr_fifo + q_fifo + writeback (lane=core)
  // -------------------------
  localparam integer AF_DEPTH = 4;
  localparam integer AF_AW    = $clog2(AF_DEPTH);

  reg [15:0] af_mem [0:3][0:AF_DEPTH-1];
  reg [AF_AW-1:0] af_wptr [0:3];
  reg [AF_AW-1:0] af_rptr [0:3];
  reg [AF_AW:0]   af_cnt  [0:3];

  wire [3:0] af_empty;
  wire [3:0] af_full;
  wire [15:0] af_head [0:3];

  generate
    for (gi=0; gi<4; gi=gi+1) begin : GEN_AF
      assign af_empty[gi] = (af_cnt[gi] == 0);
      assign af_full[gi]  = (af_cnt[gi] == AF_DEPTH);
      assign af_head[gi]  = af_mem[gi][af_rptr[gi]];
    end
  endgenerate

  reg [1:0] qf_cnt [0:3];
  reg [255:0] qf_q0 [0:3], qf_q1 [0:3];
  reg [15:0]  qf_a0 [0:3], qf_a1 [0:3];

  wire [3:0] qf_full;
  generate
    for (gi=0; gi<4; gi=gi+1) begin : GEN_QFF
      assign qf_full[gi] = (qf_cnt[gi] == 2);
    end
  endgenerate

  wire [3:0] wb_wr_en;
  wire [4*16-1:0] wb_wr_addr_vec;
  wire [4*128-1:0] wb_wr_data_vec;
  wire [3:0] wb_busy;
  wire [3:0] wb_done_pulse;

  // When a per-core writeback engine can accept a new 256b vector, it will
  // "take" one entry from that core's q_fifo. IMPORTANT: q_fifo pop/shift
  // MUST be performed in the single main always block to avoid multi-driven
  // regs (qf_cnt/qf_q0/qf_q1/qf_a0/qf_a1).
  wire [3:0] wb_take;

  genvar g_take;
  generate
    for (g_take=0; g_take<4; g_take=g_take+1) begin : GEN_WBTAKE
      assign wb_take[g_take] = (!wb_busy[g_take]) && (qf_cnt[g_take] != 0);
    end
  endgenerate

  genvar wi;
  generate
    for (wi=0; wi<4; wi=wi+1) begin : GEN_WB
      reg        wb_in_valid;
      reg [255:0] wb_in_q;
      reg [15:0]  wb_base_addr;

      writeback32 #(.ADDR_W(16)) u_wb (
        .CLK(CLK),
        .RESETn(RESETn),
        .in_valid(wb_in_valid),
        .in_q(wb_in_q),
        .base_addr(wb_base_addr),
        .wr_en(wb_wr_en[wi]),
        .wr_addr(wb_wr_addr_vec[wi*16 +: 16]),
        .wr_data(wb_wr_data_vec[wi*128 +: 128]),
        .busy(wb_busy[wi]),
        .done_pulse(wb_done_pulse[wi])
      );

      // NOTE: Do NOT update q_fifo (qf_*) here. Those regs are updated in the
      // single main sequential always block to avoid multi-driven nets.
      // Here we only present one entry to the writeback engine when it can take.
      always @(posedge CLK) begin
        if (!RESETn) begin
          wb_in_valid   <= 1'b0;
          wb_in_q       <= 256'd0;
          wb_base_addr  <= 16'd0;
        end else begin
          wb_in_valid <= 1'b0;
          if (wb_take[wi]) begin
            wb_in_valid  <= 1'b1;
            wb_in_q      <= qf_q0[wi];
            wb_base_addr <= qf_a0[wi];
          end
        end
      end
    end
  endgenerate

  assign feat_wr_en = wb_wr_en;
  assign feat_wr_local_addr_vec = wb_wr_addr_vec;
  assign feat_wr_data_vec = wb_wr_data_vec;

  // -------------------------
  // weight load serialization
  // -------------------------
  reg [1:0] wl_core_sel;

  // -------------------------
  // main sequential
  // -------------------------
  always @(posedge CLK) begin
    if (!RESETn) begin
      done <= 1'b0;

      weight_req <= 1'b0;
      weight_base <= 0;
      weight_count <= 0;

      feat_rd_en <= 1'b0;
      feat_rd_addr <= 0;

      fifo_wptr <= 0; fifo_rptr <= 0; fifo_count <= 0;
      fifo_push <= 1'b0; fifo_pop <= 1'b0; fifo_clear <= 1'b0;

      pf_active <= 1'b0;
      px_prefetch <= 0;
      pf_low128 <= 0;
      pf_state <= PF_IDLE;
      pf_restart <= 1'b0;
      pf_restart_px <= 0;

      cout_grp <= 0;
      cin_t <= 0;
      px_tile_base <= 0;
      t_in_tile <= 0;
      px <= 0;
      k <= 0;

      core_psum_do_read <= 0;
      core_psum_do_write <= 0;
      core_psum_wr_data_flat <= 0;

      mac_clear_pulse <= 1'b0;
      mac_load_pulse <= 1'b0;
      mac_step_en <= 1'b0;
      mac_act_k <= 0;
      mac_k <= 0;
      mac_load_data_flat <= 0;

      core_q_en <= 0;
      core_wt_load_start <= 0;

      wl_core_sel <= 0;
      state <= S_IDLE;

      for (ii=0; ii<4; ii=ii+1) begin
        af_wptr[ii] <= 0; af_rptr[ii] <= 0; af_cnt[ii] <= 0;
        qf_cnt[ii] <= 0; qf_q0[ii] <= 0; qf_q1[ii] <= 0; qf_a0[ii] <= 0; qf_a1[ii] <= 0;
      end

      // PERF reset
      perf_total      <= 0;
      perf_wload      <= 0;
      perf_psum_wait  <= 0;
      perf_wait_act   <= 0;
      perf_mac        <= 0;
      perf_qstall     <= 0;
      perf_wb_busy    <= 0;
      perf_fifo_empty <= 0;

      q_can_issue <= 1'b0;
    end else begin
      // defaults
      done <= 1'b0;

      weight_req <= 1'b0;
      core_wt_load_start <= 4'b0000;

      feat_rd_en <= 1'b0;

      fifo_push <= 1'b0;
      fifo_pop  <= 1'b0;
      fifo_clear<= 1'b0;

      pf_restart <= 1'b0;

      core_psum_do_read <= 4'b0000;
      core_psum_do_write<= 4'b0000;

      mac_clear_pulse <= 1'b0;
      mac_load_pulse  <= 1'b0;
      mac_step_en     <= 1'b0;
      mac_k           <= k;

      core_q_en <= 4'b0000;

      // -------------------------
      // PERF accumulate
      // -------------------------
      if (state != S_IDLE) begin
        perf_total <= perf_total + 1;

        if (state == S_WL_REQ || state == S_WL_WAIT) perf_wload <= perf_wload + 1;
        if (state == S_PSUM_WAIT) perf_psum_wait <= perf_psum_wait + 1;
        if (state == S_WAIT_ACT) begin
          perf_wait_act <= perf_wait_act + 1;
          if (fifo_empty) perf_fifo_empty <= perf_fifo_empty + 1;
        end
        if (state == S_MAC) perf_mac <= perf_mac + 1;

        if (wb_busy != 4'b0000) perf_wb_busy <= perf_wb_busy + 1;
      end

      // -------------------------
      // Prefetch restart
      // -------------------------
      if (pf_restart) begin
        px_prefetch <= pf_restart_px;
        pf_state <= PF_IDLE;
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
              fifo_push <= 1'b1;
              fifo_push_px <= px_prefetch;
              fifo_push_vec <= {feat_rd_data, pf_low128};
              px_prefetch <= px_prefetch + 1'b1;
              pf_state <= PF_IDLE;
            end
          end
          default: pf_state <= PF_IDLE;
        endcase
      end

      // -------------------------
      // addr_fifo + q_fifo capture
      // -------------------------
      for (ii=0; ii<4; ii=ii+1) begin
        // -------------------------
        // addr_fifo capture (per core)
        // -------------------------
        if (core_q_en[ii] && core_valid[ii] && !af_full[ii]) begin
          af_mem[ii][af_wptr[ii]] <= out_base_addr(px, core_cout_t[ii]);
          af_wptr[ii] <= af_wptr[ii] + 1'b1;
          af_cnt[ii]  <= af_cnt[ii]  + 1'b1;
        end

        // -------------------------
        // q_fifo push from core quant output
        // Allow push even in same cycle as wb_take pop (because pop frees space)
        // -------------------------
        if (core_q_valid[ii] && core_valid[ii] && !af_empty[ii]) begin
          // determine available slots AFTER an optional pop
          // (if wb_take and qf_cnt==2 => effective cnt=1; if wb_take and qf_cnt==1 => effective cnt=0)
          if (wb_take[ii]) begin
            // after pop, there is always at least one free slot
            if (qf_cnt[ii] == 2) begin
              // pop one (shift) and push into q1
              qf_q0[ii]  <= qf_q1[ii];
              qf_a0[ii]  <= qf_a1[ii];
              // after pop, effective cnt=1, so new entry goes to q1
              qf_q1[ii]  <= core_q_out_flat[ii*256 +: 256];
              qf_a1[ii]  <= af_head[ii];
              qf_cnt[ii] <= 2;
            end else begin
              // after pop, effective cnt=0, so new entry goes to q0
              qf_q0[ii]  <= core_q_out_flat[ii*256 +: 256];
              qf_a0[ii]  <= af_head[ii];
              qf_cnt[ii] <= 1;
            end
            af_rptr[ii] <= af_rptr[ii] + 1'b1;
            af_cnt[ii]  <= af_cnt[ii]  - 1'b1;
          end else begin
            // no pop: push only if there is space
            if (qf_cnt[ii] != 2) begin
              if (qf_cnt[ii] == 0) begin
                qf_q0[ii]  <= core_q_out_flat[ii*256 +: 256];
                qf_a0[ii]  <= af_head[ii];
                qf_cnt[ii] <= 1;
              end else begin
                qf_q1[ii]  <= core_q_out_flat[ii*256 +: 256];
                qf_a1[ii]  <= af_head[ii];
                qf_cnt[ii] <= 2;
              end
              af_rptr[ii] <= af_rptr[ii] + 1'b1;
              af_cnt[ii]  <= af_cnt[ii]  - 1'b1;
            end
          end
        end else if (wb_take[ii]) begin
          // pop only (no new q push this cycle)
          if (qf_cnt[ii] == 2) begin
            qf_q0[ii]  <= qf_q1[ii];
            qf_a0[ii]  <= qf_a1[ii];
            qf_cnt[ii] <= 1;
          end else begin
            qf_cnt[ii] <= 0;
          end
        end
      end

      // -------------------------
      // Main FSM
      // -------------------------
      case (state)
        S_IDLE: begin
          if (start) begin
            cout_grp <= 0;
            cin_t <= 0;
            px_tile_base <= 0;
            wl_core_sel <= 0;
            state <= S_WL_REQ;

            // reset PERF per-layer run
            perf_total      <= 0;
            perf_wload      <= 0;
            perf_psum_wait  <= 0;
            perf_wait_act   <= 0;
            perf_mac        <= 0;
            perf_qstall     <= 0;
            perf_wb_busy    <= 0;
            perf_fifo_empty <= 0;
          end
        end

        S_WL_REQ: begin
          if (core_valid[wl_core_sel]) begin
            weight_req   <= 1'b1;
            weight_base  <= wtile_base(cout_grp + wl_core_sel, cin_t);
            weight_count <= 17'd64;
            if (weight_grant) begin
              core_wt_load_start[wl_core_sel] <= 1'b1;
              state <= S_WL_WAIT;
            end
          end else begin
            if (wl_core_sel == 3) begin
              state <= S_TILE_INIT;
            end else begin
              wl_core_sel <= wl_core_sel + 1'b1;
            end
          end
        end

        S_WL_WAIT: begin
          if (core_wt_load_done[wl_core_sel]) begin
            if (wl_core_sel == 3) begin
              state <= S_TILE_INIT;
            end else begin
              wl_core_sel <= wl_core_sel + 1'b1;
              state <= S_WL_REQ;
            end
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
              mac_clear_pulse <= 1'b1;
              state <= S_WAIT_ACT;
            end else begin
              core_psum_do_read <= core_valid;
              state <= S_PSUM_WAIT;
            end
          end else begin
            state <= S_NEXT_PXT;
          end
        end

        S_PSUM_WAIT: begin
          if ( ((~core_valid) | core_psum_rd_valid) == 4'hF ) begin
            mac_load_pulse <= 1'b1;
            for (ii=0; ii<4; ii=ii+1) begin
              mac_load_data_flat[ii*1024 +: 1024] <= core_psum_rd_data_flat[ii*1024 +: 1024];
            end
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
          mac_k <= k;
          mac_act_k <= act_k(fifo_vec_head, k);
          mac_step_en <= 1'b1;

          if (k == 6'd31) begin
            fifo_pop <= 1'b1;
            if (!is_last_cin) state <= S_PSUM_WR;
            else              state <= S_Q_EN;
          end else begin
            k <= k + 1'b1;
          end
        end

        S_PSUM_WR: begin
          core_psum_do_write <= core_valid;
          for (ii=0; ii<4; ii=ii+1) begin
            core_psum_wr_data_flat[ii*1024 +: 1024] <= core_acc_out_flat[ii*1024 +: 1024];
          end
          state <= S_NEXT_PX;
        end

        S_Q_EN: begin
          q_can_issue = 1'b1;
          for (pi=0; pi<4; pi=pi+1) begin
            if (core_valid[pi]) begin
              if (af_full[pi]) q_can_issue = 1'b0;
              if (qf_full[pi]) q_can_issue = 1'b0;
            end
          end

          if (!q_can_issue) begin
            perf_qstall <= perf_qstall + 1;
          end

          if (q_can_issue) begin
            core_q_en <= core_valid;
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
            wl_core_sel <= 0;
            state <= S_WL_REQ;
          end else begin
            cin_t <= 0;
            state <= S_NEXT_CG;
          end
        end

        S_NEXT_CG: begin
          if (cout_grp + 4 < cout_tiles) begin
            cout_grp <= cout_grp + 4;
            wl_core_sel <= 0;
            state <= S_WL_REQ;
          end else begin
            state <= S_DONE;
          end
        end

        S_DONE: begin
          if ( (af_cnt[0]==0)&&(af_cnt[1]==0)&&(af_cnt[2]==0)&&(af_cnt[3]==0) &&
               (qf_cnt[0]==0)&&(qf_cnt[1]==0)&&(qf_cnt[2]==0)&&(qf_cnt[3]==0) &&
               (wb_busy==4'b0000) ) begin
`ifndef SYNTHESIS
            $display("[PWx4 PERF] total=%0d mac=%0d util=%0d.%0d%%  wload=%0d psum_wait=%0d wait_act=%0d qstall=%0d wb_busy=%0d fifo_empty=%0d other=%0d",
                     perf_total, perf_mac, (perf_total? (perf_mac*100)/perf_total : 0), (perf_total? ((perf_mac*10000)/perf_total)%100 : 0),
                     perf_wload, perf_psum_wait, perf_wait_act, perf_qstall, perf_wb_busy, perf_fifo_empty,
                     (perf_total - (perf_mac + perf_wload + perf_psum_wait + perf_wait_act + perf_qstall)) );
`endif
            done <= 1'b1;
            state <= S_IDLE;
          end
        end

        default: state <= S_IDLE;
      endcase

      // -------------------------
      // ACT FIFO apply
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

endmodule
