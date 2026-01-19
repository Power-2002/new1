`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/15 16:38:35
// Design Name: 
// Module Name: prefetch_double_buffer
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
module prefetch_double_buffer #(
  parameter integer OUT_W  = 112,
  parameter integer OUT_H  = 112,
  parameter integer TILE_H = 6,
  parameter integer DATA_W = 8,
  parameter integer LANES  = 16
)(
  input  wire clk,
  input  wire rst_n,
  input  wire prefetch_start,
  input  wire [$clog2(OUT_H)-1:0] tile_row,

  // Memory interface
  output reg  mem_en,
  output reg  [$clog2(OUT_W*OUT_H*(32/LANES))-1:0] mem_addr,
  input  wire [LANES*DATA_W-1:0] mem_dout,
  input  wire mem_valid,
  
  input wire [7:0] cfg_w,
  input wire [7:0] cfg_h,
  input  wire prefetch_enable,

  // Consumer interface
  input  wire read_enable,
  input  wire [$clog2(OUT_W)-1:0] read_addr,
  output reg  [TILE_H*DATA_W*LANES-1:0] buffer_out,
  output reg  buffer_ready,
  output wire prefetch_busy,
  output wire prefetch_done
);
  localparam integer WORD_W   = DATA_W * LANES;
  localparam integer PADDING  = 1;
  localparam integer PADDED_W = OUT_W + 2 * PADDING;
  localparam integer ADDR_W   = $clog2(OUT_W*OUT_H*(32/LANES));
  localparam integer BUF_AW   = $clog2(PADDED_W);
  localparam integer ROW_W    = $clog2(TILE_H);
  localparam integer MAX_OUTSTANDING = 64;
  
  wire [BUF_AW-1:0] padded_w_dyn  = cfg_w + 8'd2;
  wire [15:0]       total_words_dyn = TILE_H * padded_w_dyn;

  // ============================================================
  // 内部状态寄存器
  // ============================================================
  reg active_read_buf;
  reg active_write_buf;
  reg bufA_valid, bufB_valid;
  reg write_active;
  reg prefetch_done_reg;
  
  reg [BUF_AW-1:0] req_colp;
  reg [ROW_W-1:0]  req_rowi;
  reg [$clog2(OUT_H)-1:0] base_tile_row;
  reg [15:0] issue_count;
  reg [15:0] commit_count;
  reg [7:0]  rd_outstanding;

  assign prefetch_busy = write_active;
  assign prefetch_done = prefetch_done_reg;

  // ============================================================
  // [关键优化 1] 使用 XPM FIFO 替代寄存器数组
  // ============================================================
  localparam integer TAG_W = 1 + 1 + ROW_W + BUF_AW;
  localparam integer FIFO_DEPTH = 512;
  
  // --- Tag FIFO Signals ---
  wire [TAG_W-1:0] tag_din, tag_dout;
  wire tag_wr_en, tag_rd_en;
  wire tag_full, tag_empty;
  
  // --- Data FIFO Signals ---
  wire [WORD_W-1:0] data_din, data_dout;
  wire data_wr_en, data_rd_en;
  wire data_full, data_empty;

  // 1. Tag FIFO: 放入 BRAM 以节省 LUT
  xpm_fifo_sync #(
    .FIFO_MEMORY_TYPE("auto"),  // 让工具自动选择 (Block RAM 或 Distributed RAM)
    .FIFO_READ_LATENCY(0),      // 0延迟 = FWFT模式的前提
    .FIFO_WRITE_DEPTH(FIFO_DEPTH),
    .READ_MODE("fwft"),         // First-Word-Fall-Through (简化读逻辑)
    .WRITE_DATA_WIDTH(TAG_W),
    .READ_DATA_WIDTH(TAG_W),
    .USE_ADV_FEATURES("0000")
  ) u_tag_fifo (
    .din(tag_din), .wr_en(tag_wr_en), .full(tag_full),
    .dout(tag_dout), .rd_en(tag_rd_en), .empty(tag_empty),
    .rst(!rst_n), .wr_clk(clk), .sleep(1'b0), .injectdbiterr(1'b0), .injectsbiterr(1'b0),
    .overflow(), .underflow(), .prog_full(), .prog_empty(), .wr_ack(), .data_valid()
  );

  // 2. Data FIFO: 必须放入 Block RAM (节省大量 LUT)
  xpm_fifo_sync #(
    .FIFO_MEMORY_TYPE("block"), // 强制使用 Block RAM
    .FIFO_READ_LATENCY(0),
    .FIFO_WRITE_DEPTH(FIFO_DEPTH),
    .READ_MODE("fwft"),         // FWFT 模式
    .WRITE_DATA_WIDTH(WORD_W),
    .READ_DATA_WIDTH(WORD_W),
    .USE_ADV_FEATURES("0000")
  ) u_data_fifo (
    .din(data_din), .wr_en(data_wr_en), .full(data_full),
    .dout(data_dout), .rd_en(data_rd_en), .empty(data_empty),
    .rst(!rst_n), .wr_clk(clk), .sleep(1'b0), .injectdbiterr(1'b0), .injectsbiterr(1'b0),
    .overflow(), .underflow(), .prog_full(), .prog_empty(), .wr_ack(), .data_valid()
  );

  // ============================================================
  // [关键优化 2] 使用 DSP 进行地址计算
  // ============================================================
  wire [$clog2(OUT_H)-1:0] abs_row = base_tile_row + req_rowi;
  wire [BUF_AW-1:0] col_ext = req_colp - 1'b1;
  
  // 显式声明使用 DSP48，避免使用 LUT 搭建乘法器
  (* use_dsp = "yes" *) wire [31:0] dsp_calc_res;
  assign dsp_calc_res = abs_row * cfg_w + col_ext;
  
  wire [ADDR_W-1:0] calc_addr_wire = dsp_calc_res[ADDR_W-1:0];

  // Logic wires
  wire is_pad_col = (req_colp == {BUF_AW{1'b0}}) || (req_colp == (padded_w_dyn - 1'b1));
  wire is_row_oob = (abs_row >= cfg_h[$clog2(OUT_H)-1:0]);
  wire is_zero_req = is_pad_col || is_row_oob;

  // ============================================================
  // Issue Logic (适配 XPM FIFO 接口)
  // ============================================================
  wire can_issue = write_active && (issue_count < total_words_dyn) && !tag_full;
  wire outstanding_ok = (rd_outstanding < MAX_OUTSTANDING);
  wire mem_issue = can_issue && !is_zero_req && outstanding_ok && !data_full;

  // Tag FIFO 数据准备
  assign tag_din = is_zero_req ? 
                   {active_write_buf, 1'b1, req_rowi, req_colp} : 
                   {active_write_buf, 1'b0, req_rowi, req_colp};
  
  assign tag_wr_en = (can_issue && is_zero_req) || mem_issue;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      write_active          <= 1'b0;
      prefetch_done_reg     <= 1'b0;
      active_write_buf      <= 1'b0;
      bufA_valid            <= 1'b0;
      bufB_valid            <= 1'b0;
      base_tile_row         <= 0;
      req_colp              <= 0;
      req_rowi              <= 0;
      issue_count           <= 0;
      rd_outstanding        <= 0;
      mem_en                <= 1'b0;
      mem_addr              <= 0;
    end else begin
      mem_en                <= 1'b0;
      prefetch_done_reg     <= 1'b0;

      // Outstanding tracking
      case ({mem_issue, mem_valid && prefetch_enable})
        2'b10: rd_outstanding <= rd_outstanding + 1'b1;
        2'b01: rd_outstanding <= (rd_outstanding > 0) ? (rd_outstanding - 1'b1) : 8'd0;
      endcase

      // Start prefetch
      if (prefetch_start && !write_active) begin
        write_active   <= 1'b1;
        base_tile_row  <= tile_row;
        req_colp       <= 0;
        req_rowi       <= 0;
        issue_count    <= 0;
        rd_outstanding <= 0;
        
        // Ping-Pong 逻辑
        if (! bufA_valid) begin
          active_write_buf <= 1'b0;
          bufA_valid       <= 1'b0;
        end else if (!bufB_valid) begin
          active_write_buf <= 1'b1;
          bufB_valid       <= 1'b0;
        end else begin
          active_write_buf <= ~active_read_buf;
        end
      end

      // Issue logic
      if ((can_issue && is_zero_req) || mem_issue) begin
          issue_count <= issue_count + 1;
          
          if (mem_issue) begin
            mem_en   <= 1'b1;
            mem_addr <= calc_addr_wire; // 使用 DSP 计算结果
          end

          if (issue_count + 1 < total_words_dyn) begin
            if (req_rowi < TILE_H-1) 
              req_rowi <= req_rowi + 1;
            else begin 
              req_rowi <= 0;
              req_colp <= req_colp + 1; 
            end
          end
      end

      // Commit Done
      if (write_active && (commit_count == total_words_dyn)) begin
        write_active      <= 1'b0;
        prefetch_done_reg <= 1'b1;
        if (active_write_buf == 1'b0) bufA_valid <= 1'b1;
        else bufB_valid <= 1'b1;
        active_write_buf <= ~active_write_buf;
      end
    end
  end

  // ============================================================
  // Data FIFO Write
  // ============================================================
  assign data_din   = mem_dout;
  assign data_wr_en = mem_valid && !data_full && prefetch_enable;

  // ============================================================
  // Writer / Commit Logic (适配 FWFT 模式)
  // ============================================================
  // 由于使用了 FWFT 模式，FIFO 输出端口 (dout) 上永远是最新的有效数据
  // 我们不需要先读出再使用，而是"使用完再确认(Pop)"
  
  wire tag_is_zero = tag_dout[TAG_W-2];
  wire tag_buf_sel = tag_dout[TAG_W-1];
  wire [ROW_W-1:0] tag_rowi = tag_dout[TAG_W-3 -: ROW_W];
  wire [BUF_AW-1:0] tag_colp = tag_dout[BUF_AW-1:0];
  
  wire can_commit = write_active && !tag_empty && (tag_is_zero || !data_empty);
  
  reg wr_fire;
  reg wr_buf_sel;
  reg [ROW_W-1:0] wr_rowi;
  reg [BUF_AW-1:0] wr_colp;
  reg [WORD_W-1:0] wr_data;

  // 读使能逻辑 (Pop)
  assign tag_rd_en  = can_commit; // 提交时弹出 Tag
  assign data_rd_en = can_commit && !tag_is_zero; // 非零填充时弹出数据

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_fire    <= 1'b0;
      wr_buf_sel <= 1'b0;
      wr_rowi    <= 0;
      wr_colp    <= 0;
      wr_data    <= 0;
      commit_count <= 0;
    end else begin
      wr_fire <= 1'b0;
      if (can_commit) begin
        wr_fire    <= 1'b1;
        wr_buf_sel <= tag_buf_sel;
        wr_rowi    <= tag_rowi;
        wr_colp    <= tag_colp;
        // 直接使用 FIFO 输出数据
        wr_data    <= tag_is_zero ? {WORD_W{1'b0}} : data_dout; 
        
        commit_count <= commit_count + 1;
      end
      
      if (! write_active)
        commit_count <= 0;
    end
  end

  // ============================================================
  // [保留] 经过优化的 12 URAM 方案
  // ============================================================
  wire ram_en_read = (read_enable && buffer_ready && (read_addr[BUF_AW-1:0] < padded_w_dyn));
  wire [WORD_W-1:0] row_dout [0:TILE_H-1];
  
  genvar r;
  generate
    for (r = 0; r < TILE_H; r = r + 1) begin : GEN_ROWS
      wire write_en = wr_fire && (wr_rowi == r);
      wire [BUF_AW:0] write_addr_merged = {wr_buf_sel, wr_colp};
      wire [BUF_AW:0] read_addr_merged  = {active_read_buf, read_addr[BUF_AW-1:0]};

      // 保持您之前修改后的 URAM 参数
      xpm_memory_tdpram #(
        .ADDR_WIDTH_A(BUF_AW + 1),
        .ADDR_WIDTH_B(BUF_AW + 1),
        .MEMORY_SIZE(2 * WORD_W * PADDED_W),
        .MEMORY_PRIMITIVE("ultra"), 
        .WRITE_DATA_WIDTH_A(WORD_W),
        .READ_DATA_WIDTH_B(WORD_W),
        .READ_LATENCY_B(1),
        .WRITE_MODE_A("no_change"),
        .WRITE_MODE_B("no_change")
      ) u_buf_merged (
        .clka(clk), .ena(write_en), .wea(1'b1), .addra(write_addr_merged), .dina(wr_data), .douta(),
        .clkb(clk), .enb(ram_en_read), .web(1'b0), .addrb(read_addr_merged), .dinb({WORD_W{1'b0}}), .doutb(row_dout[r]),
        .sleep(1'b0), .rsta(1'b0), .rstb(1'b0), .regcea(1'b1), .regceb(1'b1),
        .injectdbiterra(1'b0), .injectsbiterra(1'b0), .injectdbiterrb(1'b0), .injectsbiterrb(1'b0), .dbiterrb(), .sbiterrb()
      );
    end
  endgenerate

  // ============================================================
  // Output Logic
  // ============================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      buffer_ready <= 1'b0;
      active_read_buf <= 1'b0;
      buffer_out <= {TILE_H*WORD_W{1'b0}};
    end else begin
      buffer_ready <= (active_read_buf == 1'b0) ? bufA_valid : bufB_valid;
      if (prefetch_done_reg) begin
        if (bufA_valid && !bufB_valid) active_read_buf <= 1'b0;
        else if (bufB_valid && !bufA_valid) active_read_buf <= 1'b1;
        else if (bufA_valid && bufB_valid) active_read_buf <= active_write_buf;
      end
      
      if (ram_en_read) begin
        buffer_out <= {row_dout[5], row_dout[4], row_dout[3], row_dout[2], row_dout[1], row_dout[0]};
      end
    end
  end

endmodule