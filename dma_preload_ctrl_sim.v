`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/24 14:51:51
// Design Name: 
// Module Name: dma_preload_ctrl_sim
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
module dma_preload_ctrl_sim #(
    parameter ADDR_W = 16,
    parameter integer BUF_ADDR_W = 16,
    parameter DATA_W = 128,
    // 确保指向已清洗的文件
    parameter MEM_FILE = "D:/NoC/mycode/mobilenet_acc3/data/weights/all_weights_128b_fixed.mem"
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                preload_req,
    input  wire [ADDR_W-1:0]   preload_base,
    input  wire [16:0]         preload_count,
    output reg                 preload_done,

    output reg                 dma_wr_en,
    output reg [BUF_ADDR_W-1:0] dma_wr_addr,
    output reg [DATA_W-1:0]    dma_wr_data
);

`ifndef SYNTHESIS
    localparam SIM_MEM_BITS = 20; 
    reg [DATA_W-1:0] ddr_mem [0:(1 << SIM_MEM_BITS)-1];
    
    initial begin
        $display("\n=======================================================");
        $display("[DMA_SIM] Loading from: %s", MEM_FILE);
        $readmemh(MEM_FILE, ddr_mem);
        $display("[DMA_SIM] VERIFY ddr_mem[0] = %h", ddr_mem[0]);
        $display("=======================================================\n");
    end
`endif

    reg [ADDR_W-1:0] preload_base_clean;
    integer i;
    always @(*) begin
        preload_base_clean = 0;
        for (i = 0; i < ADDR_W; i = i + 1) begin
            if (preload_base[i] === 1'b1) 
                preload_base_clean[i] = 1'b1;
            else 
                preload_base_clean[i] = 1'b0; // 将 Z 和 X 视为 0
        end
    end
    // =========================================================================

    reg [1:0] state;
    localparam S_IDLE    = 2'd0;
    localparam S_PRELOAD = 2'd1;
    localparam S_DONE    = 2'd2;

    reg [16:0]       count;
    reg [ADDR_W-1:0] ddr_addr;
    reg [BUF_ADDR_W-1:0] buf_addr;
    reg loaded_once;

    wire [16:0] eff_count = (preload_count == 0) ? 17'd1 : preload_count;
    
    // 显式索引，只取低20位防止数组越界
    wire [19:0] mem_read_index = ddr_addr[19:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            preload_done <= 1'b0;
            dma_wr_en    <= 1'b0;
            dma_wr_addr  <= {BUF_ADDR_W{1'b0}};
            dma_wr_data  <= {DATA_W{1'b0}};
            count        <= 17'd0;
            ddr_addr     <= {ADDR_W{1'b0}};
            buf_addr     <= {BUF_ADDR_W{1'b0}};
            loaded_once  <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    preload_done <= 1'b0;
                    dma_wr_en    <= 1'b0;
                    if (preload_req) begin
                        if (loaded_once) begin
                            preload_done <= 1'b1;
                            state        <= S_DONE;
                        end else begin
                            state    <= S_PRELOAD;
                            count    <= 17'd0;
                            buf_addr <= {BUF_ADDR_W{1'b0}};
                            
                            // 使用清洗后的地址，彻底根除 Z/X
                            ddr_addr <= preload_base_clean;
                             
                            $display("[DMA_SIM] Start Preload. RawBase=%h CleanedBase=%h", preload_base, preload_base_clean);
                        end
                    end
                end

                S_PRELOAD: begin
                    dma_wr_en   <= 1'b1;
                    dma_wr_addr <= buf_addr;
                    
`ifndef SYNTHESIS
    dma_wr_data <= ddr_mem[mem_read_index];
`else
    dma_wr_data <= { (DATA_W/16) { { (16-BUF_ADDR_W){1'b0} }, dma_wr_addr } };
`endif

                    if (count < eff_count - 1) begin
                        count    <= count + 1;
                        buf_addr <= buf_addr + 1;
                        ddr_addr <= ddr_addr + 1; // 现在这里是安全的，因为 ddr_addr 是纯净的 0/1
                    end else begin
                        dma_wr_en    <= 1'b0;
                        preload_done <= 1'b1;
                        loaded_once  <= 1'b1;
                        state        <= S_DONE;
                    end
                end

                S_DONE: begin
                    dma_wr_en <= 1'b0;
                    if (!preload_req) begin
                        preload_done <= 1'b0;
                        state <= S_IDLE;
                    end
                end
            endcase
        end
    end
    

endmodule