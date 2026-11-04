//=============================================================================
// ram_init.v — 上电依次清零各状态 RAM（v2 §4.2 core）
//
// T6 需清零对象（其余状态 RAM 因"计数寄存器门控 + 写先于读"免清）：
//   sel=1 发送 VL 表（1024×32b，en/bag 等配置位必须默认 0）
//   sel=2 stats 帧计数   128×32b
//   sel=3 stats 字节计数 128×32b
//   sel=4 stats 抖动最大  128×32b
// T7 增（接收侧状态/表，en/seen/valid 必须默认 0）：
//   sel=5 IC 状态        8192×9b（{seen,psn}×{net,vl}）
//   sel=6 RM 状态        4096×33b（{valid,rsn,t_last}）
//   sel=7 接收 VL 表     4096×32b ×2（W0/W1 两块并行，属主内部同喂）
// 写数据恒 0；各 RAM 属主按 ri_sel 匹配后把 ri_we/ri_addr 多选进自己的写口
// （多选在 afdx_es_core 核级完成）。init_done=1 前配置写入不应发生（tb 先查
// STATUS.0，管理面由 615A 流程保证）。
//=============================================================================

module ram_init #(
    parameter integer TABLE_WORDS = 1024,   // 发送 VL 表字数
    parameter integer STATS_WORDS = 128,    // stats 每RAM字数
    parameter integer IC_WORDS    = 8192,   // IC 状态字数（2 网 × 4096）
    parameter integer RX_WORDS    = 4096    // RM 状态 / 接收表字数
)(
    input  wire        clk,
    input  wire        reset,
    output wire        init_busy,
    output wire        init_done,
    output reg  [2:0]  ri_sel,              // 0=空闲 1=表 2/3/4=stats 5=IC 6=RM 7=接收表
    output reg  [15:0] ri_addr,
    output reg         ri_we
);

    localparam [2:0] S_TBL = 3'd1, S_SF = 3'd2, S_SB = 3'd3, S_SJ = 3'd4,
                     S_IC = 3'd5, S_RM = 3'd6, S_RXT = 3'd7, S_DONE = 3'd0;

    reg [2:0]  state;
    reg [15:0] cnt;

    assign init_busy = (state != S_DONE);
    assign init_done = (state == S_DONE);

    always @(posedge clk) begin
        if (reset) begin
            state  <= S_TBL;                 // 复位释放即开始清表
            cnt    <= 16'd0;
            ri_sel <= S_TBL;
            ri_addr <= 16'd0;
            ri_we  <= 1'b1;
        end else begin
            case (state)
                S_TBL: begin
                    ri_sel <= S_TBL;
                    ri_we  <= 1'b1;
                    ri_addr <= cnt[9:0];
                    if (cnt == TABLE_WORDS[15:0] - 16'd1) begin
                        state <= S_SF;
                        cnt   <= 16'd0;
                    end else
                        cnt <= cnt + 16'd1;
                end
                S_SF, S_SB, S_SJ: begin
                    ri_sel <= state;
                    ri_we  <= 1'b1;
                    ri_addr <= cnt[6:0];
                    if (cnt == STATS_WORDS[15:0] - 16'd1) begin
                        cnt <= 16'd0;
                        if (state == S_SJ) begin
                            state <= S_IC;           // stats 三 RAM 后接 IC
                        end else
                            state <= state + 3'd1;
                    end else
                        cnt <= cnt + 16'd1;
                end
                S_IC: begin                          // 8192×9b（IC 状态）
                    ri_sel <= S_IC;
                    ri_we  <= 1'b1;
                    ri_addr <= cnt[12:0];
                    if (cnt == IC_WORDS[15:0] - 16'd1) begin
                        state <= S_RM;
                        cnt   <= 16'd0;
                    end else
                        cnt <= cnt + 16'd1;
                end
                S_RM: begin                          // 4096×33b（RM 状态）
                    ri_sel <= S_RM;
                    ri_we  <= 1'b1;
                    ri_addr <= cnt[11:0];
                    if (cnt == RX_WORDS[15:0] - 16'd1) begin
                        state <= S_RXT;
                        cnt   <= 16'd0;
                    end else
                        cnt <= cnt + 16'd1;
                end
                S_RXT: begin                         // 4096×32b ×2（接收表并行清）
                    ri_sel <= S_RXT;
                    ri_we  <= 1'b1;
                    ri_addr <= cnt[11:0];
                    if (cnt == RX_WORDS[15:0] - 16'd1) begin
                        state <= S_DONE;
                        ri_we <= 1'b0;               // 末字已在本拍写入，收口
                    end else
                        cnt <= cnt + 16'd1;
                end
                default: begin                       // S_DONE
                    ri_we  <= 1'b0;
                    ri_sel <= 3'd0;
                end
            endcase
        end
    end

endmodule
