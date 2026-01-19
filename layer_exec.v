`timescale 1ns / 1ps
`include "mobilenet_defines.vh"
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/15 16:29:20
// Design Name: 
// Module Name: layer_exec
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
`include "mobilenet_defines.vh"
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/15 16:29:20
// Design Name: 
// Module Name: layer_exec
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
module layer_exec #(
  parameter PE_ROWS = `PE_ROWS,
  parameter PE_COLS = `PE_COLS,
  parameter integer ADDR_W = 16,
  parameter integer FAST_SIM_EN            = 0,
  parameter integer FAST_PW_COUT_SUBSAMPLE = 8,
  parameter integer FAST_PW_PX_SUBSAMPLE   = 8,
  parameter integer PW_WRITE_LIMIT_EN      = 1,
  parameter integer PW_WRITE_LIMIT_WORDS   = 2048,
  parameter integer USE_PW32x16            = 1
)(
  input  wire CLK,
  input  wire RESETn,
  input  wire start,
  output reg  done,

  input  wire [5:0] layer_id,
  input  wire [2:0] layer_type,
  input  wire [10:0] cin,
  input  wire [10:0] cout,
  input  wire [7:0]  img_w,
  input  wire [7:0]  img_h,
  input  wire [1:0]  stride,

  input  wire [ADDR_W-1:0] w_base,
  input  wire [11:0]       b_base,

  input  wire signed [15:0] quant_M,
  input  wire        [5:0]  quant_s,
  input  wire signed [7:0]  quant_zp,

  // DW channel
  output wire               dw_req,
  output wire [ADDR_W-1:0]  dw_base,
  output wire [16:0]        dw_count,
  input  wire               dw_grant,
  input  wire               dw_valid,
  input  wire [127:0]       dw_data,
  input  wire               dw_done,

  // PW / CONV channel
  output wire               pw_req,
  output wire [ADDR_W-1:0]  pw_base,
  output wire [16:0]        pw_count,
  input  wire               pw_grant,
  input  wire               pw_valid,
  input  wire [127:0]       pw_data,
  input  wire               pw_done,

  // Bias ROM
  input  wire [1023:0]       bias_vec,
  input  wire               bias_valid,
  output reg  [6:0]         bias_block_idx,
  output reg                bias_rd_en,

  // Feature buffer
  output wire [3:0]         feat_wr_en,
  output wire [4*16-1:0]    feat_wr_local_addr_vec,
  output wire [4*128-1:0]   feat_wr_data_vec,

  output wire               feat_rd_en,
  output wire [15:0]        feat_rd_local_addr,
  input  wire [127:0]       feat_rd_data,
  input  wire               feat_rd_valid,

  // FC out
  output wire               fc_out_valid,
  output wire [10:0]        fc_out_class_idx,
  output wire signed [7:0]  fc_out_logit,

  output reg feat_bank_wr_sel,
  output reg feat_bank_rd_sel
);

  // ============================================================
  // 1) Local FSM
  // ============================================================
  localparam LE_IDLE    = 4'd0;
  localparam LE_RUN_L0  = 4'd1;
  localparam LE_RUN_L1D = 4'd2;
  localparam LE_RUN_L2P = 4'd3;
  localparam LE_RUN_AP  = 4'd4;
  localparam LE_RUN_FC  = 4'd5;
  localparam LE_BYPASS  = 4'd6;
  
  reg [3:0] le_state, le_state_n;

  reg  l0_start;  wire l0_done;
  wire l1_done;
  reg  l2_start;  wire l2_done;
  reg  ap_start;  wire ap_done;
  reg  fc_start;  wire fc_done;


  // ============================================================
  // PE / layer type
  // ============================================================
  localparam [2:0] TYPE_CONV = 3'd0;
  localparam [2:0] TYPE_DW   = 3'd1;
  localparam [2:0] TYPE_PW   = 3'd2;
  localparam [2:0] TYPE_AP   = 3'd3;
  localparam [2:0] TYPE_FC   = 3'd4;

  localparam integer DW_UNIT = 16;

  wire is_dw_layer = (layer_type == TYPE_DW);
  wire is_ap_layer = (layer_type == TYPE_AP);
  wire is_fc_layer = (layer_type == TYPE_FC);

  wire [6:0] dw_num_blocks  = (cin + DW_UNIT - 1) / DW_UNIT;
  reg  [6:0] dw_block_idx;
  wire       dw_last_block  = (dw_block_idx == (dw_num_blocks - 1));
  wire       dw_stride2     = (stride == 2'd2);

  reg        dw_block_started;
  wire       dw_block_start;





  wire [6:0] fc_bias_block_idx;

  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn) begin
      le_state <= LE_IDLE;
      feat_bank_wr_sel <= 1'b0;
      feat_bank_rd_sel <= 1'b1;
    end else begin
      le_state <= le_state_n;

      if (
        (le_state == LE_RUN_L0  && l0_done) ||
        (le_state == LE_RUN_L1D && l1_done) ||
        (le_state == LE_RUN_L2P && l2_done) ||
        (le_state == LE_RUN_FC  && fc_done)
      ) begin
        feat_bank_wr_sel <= ~feat_bank_wr_sel;
        feat_bank_rd_sel <= ~feat_bank_rd_sel;
      end
    end
  end

  always @(*) begin
    le_state_n = le_state;
    done       = 1'b0;

    l0_start   = 1'b0;
    l2_start   = 1'b0;
    ap_start   = 1'b0;
    fc_start   = 1'b0;

    bias_rd_en     = 1'b0;
    bias_block_idx = dw_block_idx;

    case (le_state)
      LE_IDLE: begin
        if (start) begin
          case (layer_type)
            TYPE_CONV: begin
              if (layer_id == 6'd0) begin
                le_state_n = LE_RUN_L0;
                l0_start   = 1'b1;
              end else begin
                le_state_n = LE_BYPASS;
              end
            end
            TYPE_DW: begin
              le_state_n = LE_RUN_L1D;
            end
            TYPE_PW: begin
              le_state_n = LE_RUN_L2P;
              l2_start   = 1'b1;
            end
            TYPE_AP: begin
              le_state_n = LE_RUN_AP;
              ap_start   = 1'b1;
            end
            TYPE_FC: begin
              le_state_n = LE_RUN_FC;
              fc_start   = 1'b1;
            end
            default: begin
              le_state_n = LE_BYPASS;
            end
          endcase
        end
      end

      LE_RUN_L0:  begin if (l0_done) begin le_state_n = LE_IDLE; done = 1'b1; end end
      LE_RUN_L1D: begin
        bias_rd_en     = 1'b1;
        bias_block_idx = dw_block_idx;
        if (l1_done) begin le_state_n = LE_IDLE; done = 1'b1; end
      end
      LE_RUN_L2P: begin if (l2_done) begin le_state_n = LE_IDLE; done = 1'b1; end end
      LE_RUN_AP:  begin if (ap_done) begin le_state_n = LE_IDLE; done = 1'b1; end end
      LE_RUN_FC:  begin
        bias_rd_en     = 1'b1;
        bias_block_idx = fc_bias_block_idx;
        if (fc_done) begin le_state_n = LE_IDLE; done = 1'b1; end
      end

      LE_BYPASS: begin done = 1'b1; le_state_n = LE_IDLE; end
      default:  begin le_state_n = LE_IDLE; end
    endcase
  end

  // ============================================================
  // 2) Feature buffer mux
  // ============================================================
  wire       l1_feat_rd_en;   wire [15:0] l1_feat_rd_addr;
  wire       l2_feat_rd_en;   wire [15:0] l2_feat_rd_addr;
  wire       ap_feat_rd_en;   wire [15:0] ap_feat_rd_addr;
  wire       fc_feat_rd_en;   wire [15:0] fc_feat_rd_addr;

  // L0 now uses 4-lane writeback (lane0/1 active)
  wire [3:0]       l0_feat_wr_en_vec;
  wire [4*16-1:0]  l0_feat_wr_addr_vec;
  wire [4*128-1:0] l0_feat_wr_data_vec;

  wire [3:0] l1_feat_wr_en_vec; wire [4*16-1:0] l1_feat_wr_addr_vec; wire [4*128-1:0] l1_feat_wr_data_vec;

  // PW x4 writeback (4 lanes)
  wire [3:0]       l2_feat_wr_en_vec;
  wire [4*16-1:0]  l2_feat_wr_addr_vec;
  wire [4*128-1:0] l2_feat_wr_data_vec;

  wire       ap_feat_wr_en;   wire [15:0] ap_feat_wr_addr; wire [127:0] ap_feat_wr_data;
  wire       fc_feat_wr_en;   wire [15:0] fc_feat_wr_addr; wire [127:0] fc_feat_wr_data;

  assign feat_wr_en =
    (le_state == LE_RUN_L0)  ? l0_feat_wr_en_vec :
    (le_state == LE_RUN_L1D) ? l1_feat_wr_en_vec :
    (le_state == LE_RUN_L2P) ? l2_feat_wr_en_vec :
    (le_state == LE_RUN_AP)  ? {3'b000, ap_feat_wr_en} :
    (le_state == LE_RUN_FC)  ? {3'b000, fc_feat_wr_en} :
                               4'b0000;

  assign feat_wr_local_addr_vec =
    (le_state == LE_RUN_L0)  ? l0_feat_wr_addr_vec :
    (le_state == LE_RUN_L1D) ? l1_feat_wr_addr_vec :
    (le_state == LE_RUN_L2P) ? l2_feat_wr_addr_vec :
    (le_state == LE_RUN_AP)  ? {48'd0, ap_feat_wr_addr} :
    (le_state == LE_RUN_FC)  ? {48'd0, fc_feat_wr_addr} :
                               64'd0;

  assign feat_wr_data_vec =
    (le_state == LE_RUN_L0)  ? l0_feat_wr_data_vec :
    (le_state == LE_RUN_L1D) ? l1_feat_wr_data_vec :
    (le_state == LE_RUN_L2P) ? l2_feat_wr_data_vec :
    (le_state == LE_RUN_AP)  ? {384'd0, ap_feat_wr_data} :
    (le_state == LE_RUN_FC)  ? {384'd0, fc_feat_wr_data} :
                               512'd0;

  assign feat_rd_en =
    (le_state == LE_RUN_L1D) ? l1_feat_rd_en :
    (le_state == LE_RUN_L2P) ? l2_feat_rd_en :
    (le_state == LE_RUN_AP)  ? ap_feat_rd_en :
    (le_state == LE_RUN_FC)  ? fc_feat_rd_en : 1'b0;

  assign feat_rd_local_addr =
    (le_state == LE_RUN_L1D) ? l1_feat_rd_addr :
    (le_state == LE_RUN_L2P) ? l2_feat_rd_addr :
    (le_state == LE_RUN_AP)  ? ap_feat_rd_addr :
    (le_state == LE_RUN_FC)  ? fc_feat_rd_addr : 16'd0;

  // ============================================================
  // 3) PW channel mux (L0 / L2 / FC)
  // ============================================================
  wire        l0_pw_req, l2_pw_req, fc_pw_req;
  wire [ADDR_W-1:0] l0_pw_base, l2_pw_base, fc_pw_base;
  wire [16:0]       l0_pw_count, l2_pw_count, fc_pw_count;

  assign pw_req =
    (le_state == LE_RUN_L0)  ? l0_pw_req :
    (le_state == LE_RUN_L2P) ? l2_pw_req :
    (le_state == LE_RUN_FC)  ? fc_pw_req : 1'b0;

  assign pw_base =
    (le_state == LE_RUN_L0)  ? l0_pw_base :
    (le_state == LE_RUN_L2P) ? l2_pw_base :
    (le_state == LE_RUN_FC)  ? fc_pw_base : w_base;

  assign pw_count =
    (le_state == LE_RUN_L0)  ? l0_pw_count :
    (le_state == LE_RUN_L2P) ? l2_pw_count :
    (le_state == LE_RUN_FC)  ? fc_pw_count : 17'd0;

  // ============================================================
  // 4) L0: 3x3 CONV (CORRECT: 27-step MAC32+quant, 4-lane writeback)
  // ============================================================
  wire        l0_win_req, l0_win_valid;
  wire [215:0] l0_win_flat;
  wire [15:0]  l0_win_x, l0_win_y;
  wire        l0_frame_done;

  window_fetcher_pull_3x3x3_str2_int8 u_l0_fetch (
    .CLK        (CLK),
    .RESET      (RESETn),
    .start_frame(l0_start),
    .win_req    (l0_win_req),
    .win_valid  (l0_win_valid),
    .win_flat   (l0_win_flat),
    .win_x      (l0_win_x),
    .win_y      (l0_win_y),
    .frame_done (l0_frame_done)
  );

  // -------------------------
  // L0: Conv1 reuse PW engine
  // -------------------------
  // Conv1 provides 27B/window; PW engine expects 32B/pixel packed as two 128b reads.
  // This adapter pads to 32B and answers pw_scheduler feat_rd requests.
  wire [127:0] l0_pw_feat_rd_data;
  wire         l0_pw_feat_rd_valid;

  conv1_featmem_from_window u_l0_featmem (
    .CLK         (CLK),
    .RESETn      (RESETn),
    .enable      (le_state == LE_RUN_L0),

    .feat_rd_en    (l2_feat_rd_en),
    .feat_rd_addr  (l2_feat_rd_addr),
    .feat_rd_data  (l0_pw_feat_rd_data),
    .feat_rd_valid (l0_pw_feat_rd_valid),

    .win_req    (l0_win_req),
    .win_valid  (l0_win_valid),
    .win_flat   (l0_win_flat),
    .frame_done (l0_frame_done)
  );

  // mux pw_scheduler read-data source: L0 uses adapter, PW layers use real feature SRAM
  wire [127:0] pw_sched_feat_rd_data  = (le_state == LE_RUN_L0) ? l0_pw_feat_rd_data  : feat_rd_data;
  wire         pw_sched_feat_rd_valid = (le_state == LE_RUN_L0) ? l0_pw_feat_rd_valid : feat_rd_valid;

  // Conv1 is executed as a special PW case with cin padded to 32, cout=32, out size=112x112.
  wire [10:0] pw_cin_mux  = (le_state == LE_RUN_L0) ? 11'd32  : cin;
  wire [10:0] pw_cout_mux = (le_state == LE_RUN_L0) ? 11'd32  : cout;
  wire [7:0]  pw_img_w_mux = (le_state == LE_RUN_L0) ? 8'd112 : img_w;
  wire [7:0]  pw_img_h_mux = (le_state == LE_RUN_L0) ? 8'd112 : img_h;

  // IMPORTANT:
  // l0_start/l2_start are asserted in LE_IDLE when we *enter* the run state.
  // In that cycle le_state is still LE_IDLE, so we must NOT mux start by le_state.
  // Use OR so the PW scheduler reliably sees the start pulse for both L0 and PW layers.
  wire pw_start_mux;
  assign pw_start_mux = l0_start | l2_start;
  wire pw_done_mux;
  // NOTE: Conv1 now reuses PW engine (no dedicated conv1 MAC/quant).

  // ============================================================
  // 5) PW (x4 cores, synth-safe)
  // ============================================================
  // PW scheduler done is shared by L0 (Conv1) and all PW layers

  pw_scheduler_ws_4full #(
    .ADDR_W(ADDR_W),
    .PIXEL_TILE_SIZE(128),
    .ACT_FIFO_DEPTH(16)
  ) u_l2_sched_ws_x4 (
    .CLK       (CLK),
    .RESETn    (RESETn),
    .start     (pw_start_mux),
    .done      (pw_done_mux),

    .cin       (pw_cin_mux),
    .cout      (pw_cout_mux),
    .img_w     (pw_img_w_mux),
    .img_h     (pw_img_h_mux),
    .w_base_in (w_base),

    .quant_M   (quant_M),
    .quant_s   (quant_s),
    .quant_zp  (quant_zp),
    .bias_vec  (bias_vec),

    .weight_req   (l2_pw_req),
    .weight_grant (pw_grant),
    .weight_base  (l2_pw_base),
    .weight_count (l2_pw_count),
    .weight_valid (pw_valid),
    .weight_data  (pw_data),
    .weight_done  (pw_done),

    .feat_rd_en    (l2_feat_rd_en),
    .feat_rd_addr  (l2_feat_rd_addr),
    .feat_rd_data  (pw_sched_feat_rd_data),
    .feat_rd_valid (pw_sched_feat_rd_valid),

    .feat_wr_en            (l2_feat_wr_en_vec),
    .feat_wr_local_addr_vec(l2_feat_wr_addr_vec),
    .feat_wr_data_vec      (l2_feat_wr_data_vec)
  );
  assign l2_done = pw_done_mux;
  assign l0_done = pw_done_mux;

  // L0 reuses PW loader/writeback signals
  assign l0_pw_req   = l2_pw_req;
  assign l0_pw_base  = l2_pw_base;
  assign l0_pw_count = l2_pw_count;

  assign l0_feat_wr_en_vec    = l2_feat_wr_en_vec;
  assign l0_feat_wr_addr_vec  = l2_feat_wr_addr_vec;
  assign l0_feat_wr_data_vec  = l2_feat_wr_data_vec;
  // ============================================================
  // 6) L1: DW 3x3   [UNCHANGED BELOW]
  // ============================================================
  // ---- everything below is your original code ----

  wire l1_cache_load_start, l1_cache_load_done;
  reg  l1_weight_loaded;

  wire        l1_buffer_ready, l1_prefetch_done, l1_prefetch_busy;
  wire        l1_read_enable;
  wire [6:0]  l1_read_addr_internal;
  wire [767:0] l1_buffer_out;
  wire        l1_scanner_done, l1_scanner_busy;

  reg  l1_scanner_done_d1;
  wire l1_scanner_done_rise = l1_scanner_done & ~l1_scanner_done_d1;

  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn) begin
      l1_weight_loaded <= 1'b0;
    end else begin
      if (l1_scanner_done_rise && ! dw_last_block) begin
        l1_weight_loaded <= 1'b0;
      end else if (l1_cache_load_done) begin
        l1_weight_loaded <= 1'b1;
      end else if (le_state != LE_RUN_L1D) begin
        l1_weight_loaded <= 1'b0;
      end
    end
  end

  assign l1_cache_load_start = (le_state == LE_RUN_L1D) && !l1_weight_loaded;

  wire [ADDR_W-1:0] dw_w_base_blk = w_base + (dw_block_idx * 11'd9);

  wire        l1_w_valid;
  wire [3:0]  l1_w_idx;
  wire [127:0] l1_w_data;

  dw_weight_cache #(
    .ADDR_W(ADDR_W)
  ) u_l1_dw_cache (
    .clk           (CLK),
    .rst_n         (RESETn),
    .load_start    (l1_cache_load_start),
    .base_addr     (dw_w_base_blk[ADDR_W-1:0]),
    .load_done     (l1_cache_load_done),

    .ldr_req       (dw_req),
    .ldr_grant     (dw_grant),
    .ldr_base_addr (dw_base),
    .ldr_count     (dw_count),
    .ldr_valid     (dw_valid),
    .ldr_data      (dw_data),
    .ldr_done_sig  (dw_done),

    .w_valid       (l1_w_valid),
    .w_idx         (l1_w_idx),
    .w_data        (l1_w_data)
  );

  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn)
      l1_scanner_done_d1 <= 1'b0;
    else
      l1_scanner_done_d1 <= l1_scanner_done;
  end

  assign dw_block_start =
    (le_state == LE_RUN_L1D) &&
    l1_weight_loaded &&
    bias_valid &&
    ! dw_block_started;

  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn) begin
      dw_block_started <= 1'b0;
    end else begin
      if (le_state != LE_RUN_L1D)
        dw_block_started <= 1'b0;
      else if (l1_scanner_done_rise)
        dw_block_started <= 1'b0;
      else if (dw_block_start)
        dw_block_started <= 1'b1;
    end
  end

  wire [7:0] dw_out_w = dw_stride2 ? ((img_w + 8'd1) >> 1) : img_w;
  wire [7:0] dw_out_h = dw_stride2 ? ((img_h + 8'd1) >> 1) : img_h;

  wire [$clog2(112)-1:0] l1_current_tile_row;
  wire l1_tile_start;

  wire [7:0] scanner_input_w = img_w + 8'd2;
  wire [7:0] scanner_input_h = img_h;

  simple_column_scanner_pipeline #(
    .OUT_W(112),
    .OUT_H(112),
    .TILE_H(6),
    .K(3),
    .PADDING(1)
  ) u_l1_scanner (
    .clk              (CLK),
    .rst_n            (RESETn),
    .start            (dw_block_start),
    .cfg_w            (scanner_input_w),
    .cfg_h            (img_h),
    .stride2_en       (dw_stride2),
    .current_tile_row (l1_current_tile_row),
    .tile_start       (l1_tile_start),
    .buffer_ready     (l1_buffer_ready),
    .read_enable      (l1_read_enable),
    .read_addr        (l1_read_addr_internal),
    .busy             (l1_scanner_busy),
    .done             (l1_scanner_done),
    .current_col      ()
  );

  wire [14:0] l1_rd_addr_raw;
  wire [31:0] l1_rd_addr_scaled = (l1_rd_addr_raw * dw_num_blocks) + dw_block_idx;
  assign l1_feat_rd_addr = l1_rd_addr_scaled[15:0];

  wire l1_prefetch_enable = (le_state == LE_RUN_L1D);

  prefetch_double_buffer u_l1_prefetch (
    .clk           (CLK),
    .rst_n         (RESETn),
    .prefetch_start(l1_tile_start),
    .tile_row      (l1_current_tile_row),
    .prefetch_enable(l1_prefetch_enable),
    .cfg_w(img_w),
    .cfg_h(img_h),
    .mem_en        (l1_feat_rd_en),
    .mem_addr      (l1_rd_addr_raw),
    .mem_dout      (feat_rd_data),
    .mem_valid     (feat_rd_valid),
    .read_enable   (l1_read_enable),
    .read_addr     (l1_read_addr_internal),
    .buffer_out    (l1_buffer_out),
    .buffer_ready  (l1_buffer_ready),
    .prefetch_busy (l1_prefetch_busy),
    .prefetch_done (l1_prefetch_done)
  );

  wire l1_read_fire = (l1_read_enable && l1_buffer_ready);
  reg  l1_read_fire_d1;

  always @(posedge CLK or negedge RESETn) begin
    if(!RESETn) l1_read_fire_d1 <= 1'b0;
    else        l1_read_fire_d1 <= l1_read_fire;
  end

  wire [767:0] l1_column_data;
  wire         l1_column_valid;

  column_passthrough u_l1_col_pass (
    .clk             (CLK),
    .rst_n           (RESETn),
    .column_data_in  (l1_buffer_out),
    .column_valid    (l1_read_fire_d1),
    .column_data_out (l1_column_data),
    .out_valid       (l1_column_valid)
  );

  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn) begin
      dw_block_idx <= 7'd0;
    end else begin
      if (le_state == LE_IDLE && start && is_dw_layer) begin
        dw_block_idx <= 7'd0;
      end else if (le_state == LE_RUN_L1D && l1_scanner_done_rise) begin
        if (!dw_last_block)
          dw_block_idx <= dw_block_idx + 7'd1;
      end
    end
  end

  reg [3:0] wstream_cnt;
  reg       weights_ready;

  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn) begin
      wstream_cnt   <= 0;
      weights_ready <= 0;
    end else begin
      if (dw_block_start) begin
        wstream_cnt   <= 0;
        weights_ready <= 0;
      end else if (l1_w_valid && !weights_ready) begin
        if (wstream_cnt == 8)
          weights_ready <= 1;
        else
          wstream_cnt <= wstream_cnt + 1;
      end
    end
  end

  wire [2047:0] l1_dwc_out_sums;
  wire [63:0]   l1_dwc_out_valids;

  dwc_pu u_l1_dwc (
    .clk        (CLK),
    .rst_n      (RESETn),
    .in_valid   (l1_column_valid && l1_weight_loaded),
    .column_data(l1_column_data),
    .w_valid    (l1_w_valid),
    .w_idx      (l1_w_idx),
    .w_data     (l1_w_data),
    .out_sums   (l1_dwc_out_sums),
    .out_valids (l1_dwc_out_valids)
  );

  reg [6:0] l1_current_col_d1, l1_current_col_d2, l1_current_col_d3;
  reg [6:0] l1_tile_row_d1, l1_tile_row_d2, l1_tile_row_d3;

  always @(posedge CLK) begin
    l1_current_col_d1 <= l1_read_addr_internal;
    l1_current_col_d2 <= l1_current_col_d1;
    l1_current_col_d3 <= l1_current_col_d2;

    l1_tile_row_d1 <= l1_current_tile_row;
    l1_tile_row_d2 <= l1_tile_row_d1;
    l1_tile_row_d3 <= l1_tile_row_d2;
  end

  wire [6:0] l1_current_col = l1_current_col_d3;
  wire [6:0] l1_aligned_tile_row = l1_tile_row_d3;

  wire signed [8:0] l1_col_unpadded = {2'b00, l1_current_col} - 9'sd1;
  wire signed [8:0] l1_row_unpadded = {2'b00, l1_aligned_tile_row};

  wire signed [8:0] l1_output_col_signed = dw_stride2 ? (l1_col_unpadded >>> 1) : l1_col_unpadded;
  wire signed [8:0] l1_output_row_signed = dw_stride2 ? (l1_row_unpadded >>> 1) : l1_row_unpadded;

  wire [7:0] l1_output_col = l1_output_col_signed[7:0];
  wire [7:0] l1_output_row = l1_output_row_signed[7:0];

  wire [7:0] expected_out_w = dw_stride2 ? ((img_w + 8'd1) >> 1) : img_w;
  wire [7:0] expected_out_h = dw_stride2 ? ((img_h + 8'd1) >> 1) : img_h;

  wire l1_output_nonneg = (l1_output_col_signed[8] == 1'b0) &&
                          (l1_output_row_signed[8] == 1'b0);

  wire l1_output_in_range = (l1_output_col < expected_out_w) &&
                            (l1_output_row < expected_out_h);

  wire l1_on_stride_grid = ! dw_stride2 ||
                           ((l1_col_unpadded[0] == 1'b0) &&
                            (l1_row_unpadded[0] == 1'b0));

  wire l1_input_reasonable = (l1_col_unpadded >= -9'sd1) &&
                             (l1_col_unpadded <= {1'b0, img_w}) &&
                             (l1_row_unpadded >= 9'sd0) &&
                             (l1_row_unpadded < {1'b0, img_h});

  wire l1_coord_valid = l1_output_nonneg &&
                        l1_output_in_range &&
                        l1_on_stride_grid &&
                        l1_input_reasonable;

  wire [63:0] l1_filtered_valids = l1_coord_valid ? l1_dwc_out_valids : 64'd0;

  wire [6:0] q_tile_row  = l1_output_row[6:0];
  wire [6:0] q_col_idx   = l1_output_col[6:0];
  wire [5:0] q_blk_idx   = dw_block_idx[5:0];

  wire        dw_wr_en0, dw_wr_en1, dw_wr_en2, dw_wr_en3;
  wire [15:0] dw_wr_addr0, dw_wr_addr1, dw_wr_addr2, dw_wr_addr3;
  wire [127:0] dw_wr_data0, dw_wr_data1, dw_wr_data2, dw_wr_data3;

  quant_l1_stream_4channel #(
    .UNIT_NUM(16),
    .OUT_W_MAX(112),
    .OUT_H_MAX(112),
    .BLOCKS_MAX(128),
    .ACC_W(32),
    .OUT_BITS(8)
  ) u_dw_quant_stream (
    .clk(CLK),
    .rst_n(RESETn),

    .cfg_out_w(dw_out_w[6:0]),
    .cfg_out_h(dw_out_h[6:0]),
    .cfg_blocks(dw_num_blocks),

    .dwc_sums(l1_dwc_out_sums),
    .dwc_valids(l1_filtered_valids),
    .tile_row(q_tile_row),
    .col_index(q_col_idx),
    .block_idx(q_blk_idx),

    .bias_vec(bias_vec),
    .cfg_mult_scalar(quant_M),
    .cfg_shift_scalar(quant_s),
    .cfg_symmetric(1'b0),
    .cfg_zp_out(quant_zp),

    .wr_en0(dw_wr_en0), .wr_addr0(dw_wr_addr0), .wr_data0(dw_wr_data0),
    .wr_en1(dw_wr_en1), .wr_addr1(dw_wr_addr1), .wr_data1(dw_wr_data1),
    .wr_en2(dw_wr_en2), .wr_addr2(dw_wr_addr2), .wr_data2(dw_wr_data2),
    .wr_en3(dw_wr_en3), .wr_addr3(dw_wr_addr3), .wr_data3(dw_wr_data3)
  );

  assign l1_feat_wr_en_vec   = {dw_wr_en3,   dw_wr_en2,   dw_wr_en1,   dw_wr_en0};
  assign l1_feat_wr_addr_vec = {dw_wr_addr3, dw_wr_addr2, dw_wr_addr1, dw_wr_addr0};
  assign l1_feat_wr_data_vec = {dw_wr_data3, dw_wr_data2, dw_wr_data1, dw_wr_data0};

  assign l1_done = (le_state == LE_RUN_L1D) && l1_scanner_done_rise && dw_last_block;

  // ============================================================
  // 9) AP
  // ============================================================
  global_avg_pool #(
    .CHANNELS  (1024),
    .POOL_SIZE (7),
    .DATA_W    (8),
    .ACC_W     (32),
    .LANES     (16)
  ) u_gap (
    .clk               (CLK),
    .rst_n             (RESETn),
    .start             (ap_start),
    .feat_rd_en        (ap_feat_rd_en),
    .feat_rd_local_addr(ap_feat_rd_addr),
    .feat_rd_data      (feat_rd_data),
    .feat_rd_valid     (feat_rd_valid),
    .feat_wr_en        (ap_feat_wr_en),
    .feat_wr_local_addr(ap_feat_wr_addr),
    .feat_wr_data      (ap_feat_wr_data),
    .done              (ap_done)
  );

  // ============================================================
  // 10) FC
  // ============================================================
  fc_layer #(
    .IN_FEATURES (1024),
    .OUT_CLASSES (1000),
    .DATA_W      (8),
    .ACC_W       (32),
    .LANES       (16),
    .ADDR_W      (ADDR_W)
  ) u_fc (
    .clk               (CLK),
    .rst_n             (RESETn),
    .start             (fc_start),
    .w_base            (w_base),
    .b_base            (b_base),
    .quant_M           (quant_M),
    .quant_s           (quant_s),
    .quant_zp          (quant_zp),

    .weight_req        (fc_pw_req),
    .weight_base       (fc_pw_base),
    .weight_count      (fc_pw_count),
    .weight_grant      (pw_grant),
    .weight_valid      (pw_valid),
    .weight_data       (pw_data),
    .weight_done       (pw_done),

    .bias_vec          (bias_vec),
    .bias_valid        (bias_valid),
    .bias_block_idx    (fc_bias_block_idx),
    .bias_rd_en        (),

    .feat_rd_en        (fc_feat_rd_en),
    .feat_rd_local_addr(fc_feat_rd_addr),
    .feat_rd_data      (feat_rd_data),
    .feat_rd_valid     (feat_rd_valid),

    .out_valid         (fc_out_valid),
    .out_class_idx     (fc_out_class_idx),
    .out_logit         (fc_out_logit),
    .done              (fc_done)
  );

  assign fc_feat_wr_en   = 1'b0;
  assign fc_feat_wr_addr = 16'd0;
  assign fc_feat_wr_data = 128'd0;
  
    // ============================================================
  // Debug (simulation-only)
  // ============================================================
  // These prints help locate deadlocks when reusing PW engine for L0.
  // Safe for synthesis (wrapped by `ifndef SYNTHESIS).
  reg [31:0] dbg_l0_watch;
  always @(posedge CLK or negedge RESETn) begin
    if (!RESETn) begin
      dbg_l0_watch <= 32'd0;
    end else begin
`ifndef SYNTHESIS
      if (l0_start) begin
        $display("[%0t ns][layer_exec] L0 start pulse seen (le_state=%0d)", $time, le_state);
      end
      if (le_state == LE_RUN_L0 && !l0_done) begin
        dbg_l0_watch <= dbg_l0_watch + 1'b1;
        if (dbg_l0_watch == 32'd50000) begin
          $display("[%0t ns][layer_exec][L0 WATCH] still running. pw_req=%b pw_grant=%b pw_valid=%b pw_done=%b  feat_rd_en(from pw)=%b addr=%0d  win_req=%b win_valid=%b", 
                   $time, pw_req, pw_grant, pw_valid, pw_done, l2_feat_rd_en, l2_feat_rd_addr, l0_win_req, l0_win_valid);
          dbg_l0_watch <= 32'd0;
        end
      end else begin
        dbg_l0_watch <= 32'd0;
      end
`endif
    end
  end
endmodule