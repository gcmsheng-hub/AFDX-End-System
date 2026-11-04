//=============================================================================
// mac_rx.v — 接收 MAC（v2 §4.1/4.2 mac_rx；T7 纯仿真 RTL 版，T8 换 eLinx MAC IP）
//
// 职责（v2 §5.5 前两项）：SFD 同步（容忍前导长度）、逐字节入环形帧缓冲 +
// 在线 CRC32、帧长检查（64~1518）、FCS 校验、推出描述符 {slot, len, ts}。
//   - 前导 + SFD：IDLE 态在 RX_DV 期寻找 0xD5，找到即进 BODY（前导字节不入缓冲）
//   - 帧体（含 FCS 4B 全部计入 len）：写入 2 槽环形缓冲（每槽 2048B）；
//     CRC 覆盖全部帧体字节（含 FCS），RX_DV 下降沿查魔数残差 0xDEBB20E3
//     （数据+FCS 小端回灌后寄存器恒为该值——zlib.crc32 交叉验证，v2 §5.1 口径）
//   - 坏帧（FCS 错 / 长度越界 / RX_ER）直接丢弃并计数；槽位回滚复用
//   - 两槽全忙（描述符未及消费）→ 整帧旁路丢弃计 overflow
// 描述符：sync_fifo FWFT，{slot[0], len[10:0], ts_ns[63:0]}（ts = SFD 拍 time_ns）。
// 槽位归还：引擎处理完调 ret_valid/ret_slot 释放（写/读两端经 2 槽解耦）。
// 读口：rd_addr（{slot,off} 12b）→ rd_data 1 拍（rx_path 顺序读帧体）。
//=============================================================================

module mac_rx #(
    parameter [1:0] NET_ID = 2'b01          // 01=A 网 10=B 网（观测/元数据标注）
)(
    input  wire        clk,
    input  wire        reset,

    input  wire [63:0] time_ns,             // 时基（SFD 拍打戳，hrx_meta 用）

    // GMII 接收侧（T7：tb 层级驱动；T8：eLinx MAC IP GMII）
    input  wire [7:0]  rxd,
    input  wire        rx_dv,
    input  wire        rx_er,

    // 帧缓冲读口（引擎用）
    input  wire [11:0] rd_addr,             // {slot, off[10:0]}
    output wire [7:0]  rd_data,

    // 描述符出队（FWFT：desc_vld=1 时 desc_* 为队首，desc_re 当拍弹出）
    output wire        desc_vld,
    output wire        desc_slot,
    output wire [10:0] desc_len,            // 完整帧长（含 FCS，64~1518）
    output wire [63:0] desc_ts,
    input  wire        desc_re,

    // 槽位归还（引擎处理完释放；单拍，同拍至多归还一槽）
    input  wire        ret_valid,
    input  wire        ret_slot,

    // 事件（stats 单拍脉冲）
    output wire        ev_frame,            // MAC 收到一好帧
    output wire        ev_fcserr,
    output wire        ev_lenerr,
    output wire        ev_overflow
);

    localparam [7:0]  SFD      = 8'hD5;
    localparam [31:0] CRC_MAGI = 32'hDEBB20E3;   // 数据+FCS 回灌残差（见头注）

    // 接收 FSM 寄存器（先声明——帧缓冲/CRC 实例引用）
    localparam [2:0] S_IDLE = 3'd0, S_BODY = 3'd1, S_CHK = 3'd2, S_DROP = 3'd3;
    reg [2:0]  state;
    reg        wr_slot;                       // 当前写槽（不变式：恒空闲）
    reg [10:0] wr_off;                        // 帧内写偏移（含 FCS = 完整帧长）
    reg [63:0] f_ts;                          // SFD 拍时基
    reg [1:0]  slot_busy;                     // bit0/1 = 槽 0/1 有帧待消费
    reg        ev_frame_r, ev_fcserr_r, ev_lenerr_r, ev_ovf_r;

    // 写使能（组合）：S_BODY 态且本拍有字节——字节当拍写入 + CRC 当拍吸入
    // （寄存一拍会丢首字节、把帧尾写成垃圾——T7-001 流水 tb 抓出并修复）
    wire wr_we = (state == S_BODY) && rx_dv && !rx_er;

    // 帧缓冲：2 槽 × 2048B（帧 ≤1518）拼 4KB BRAM（地址高位 = 槽号）
    wire [11:0] wr_addr = {wr_slot, wr_off};
    wire [7:0]  mem_rdata;

    sdp_ram #(.WIDTH(8), .DEPTH(4096), .AW(12)) u_buf (
        .clk(clk), .we(wr_we), .waddr(wr_addr), .wdata(rxd),
        .raddr(rd_addr), .rdata(mem_rdata));

    assign rd_data = mem_rdata;

    // 在线 CRC（覆盖全部帧体字节含 FCS；下降沿查魔数残差判 FCS）
    // init 用 SFD 检出拍的组合条件（提前一拍）——若与首字节同拍 init，
    // init 优先会吃掉 byte0（T7-001 流水 tb 抓出并修复）
    wire [31:0] crc_r;
    wire crc_init = (state == S_IDLE) && rx_dv && (rxd == SFD)
                 && !(wr_slot ? slot_busy[1] : slot_busy[0]);
    crc32_d8 u_crc (
        .clk(clk), .reset(reset),
        .init(crc_init),
        .valid(wr_we),
        .data(rxd),
        .crc(crc_r), .fcs());

    //-------------------------------------------------------------------------
    // 描述符 FIFO：{slot, len[10:0], ts_ns[63:0]} 76b × 4
    //-------------------------------------------------------------------------
    reg        df_we;
    reg [75:0] df_wdata;
    wire [75:0] df_rdata;
    wire       df_full, df_empty;
    wire       df_re = desc_re;

    sync_fifo #(.DW(76), .DEPTH(4), .AW(2)) u_desc (
        .clk(clk), .reset(reset),
        .we(df_we), .wdata(df_wdata), .re(df_re),
        .rdata(df_rdata), .full(df_full), .empty(df_empty),
        .count(), .almost_full());

    assign {desc_slot, desc_len, desc_ts} = df_rdata;
    assign desc_vld = !df_empty;

    //-------------------------------------------------------------------------
    // 接收 FSM
    //-------------------------------------------------------------------------
    assign ev_frame    = ev_frame_r;
    assign ev_fcserr   = ev_fcserr_r;
    assign ev_lenerr   = ev_lenerr_r;
    assign ev_overflow = ev_ovf_r;

    always @(posedge clk) begin
        if (reset) begin
            state <= S_IDLE;
            wr_slot <= 1'b0; wr_off <= 11'd0;
            f_ts <= 64'd0;
            slot_busy <= 2'b00;
            ev_frame_r <= 1'b0; ev_fcserr_r <= 1'b0;
            ev_lenerr_r <= 1'b0; ev_ovf_r <= 1'b0;
            df_we <= 1'b0; df_wdata <= 76'd0;
        end else begin
            ev_frame_r <= 1'b0; ev_fcserr_r <= 1'b0;
            ev_lenerr_r <= 1'b0; ev_ovf_r <= 1'b0;
            df_we   <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (rx_dv && !rx_er && (rxd == SFD)) begin
                        if (wr_slot ? slot_busy[1] : slot_busy[0]) begin
                            ev_ovf_r <= 1'b1;     // 两槽全忙：整帧旁路
                            state    <= S_DROP;
                        end else begin
                            wr_off  <= 11'd0;     // SFD 检出 → 帧体
                            f_ts    <= time_ns;
                            state   <= S_BODY;
                        end
                    end
                    // 其余 = 前导字节，忽略（容忍前导长度差异）
                end

                S_BODY: begin
                    if (rx_er) begin              // 传输错误：弃帧（不计 FCS 桶）
                        state <= S_IDLE;
                    end else if (!rx_dv) begin    // RX_DV 下降沿：判 FCS/长度
                        state <= S_CHK;
                    end else begin
                        wr_off <= wr_off + 11'd1; // 本拍字节已组合写入（含 FCS）
                        if (wr_off == 11'd1535) begin   // 越界防护（>1518 必坏）
                            ev_lenerr_r <= 1'b1;
                            state <= S_DROP;
                        end
                    end
                end

                S_CHK: begin                      // wr_off = 完整帧长
                    if (rx_dv) begin              // 异常：DV 又起，重同步
                        state <= S_IDLE;
                    end else if (wr_off < 11'd64 || wr_off > 11'd1518) begin
                        ev_lenerr_r <= 1'b1;      // runt / 超长帧
                        state <= S_IDLE;          // 槽未占，无需归还
                    end else if (crc_r != CRC_MAGI) begin
                        ev_fcserr_r <= 1'b1;      // FCS 错
                        state <= S_IDLE;
                    end else begin
                        slot_busy[wr_slot] <= 1'b1;   // 占槽推描述符
                        df_we    <= 1'b1;
                        df_wdata <= {wr_slot, wr_off, f_ts};
                        wr_slot  <= ~wr_slot;         // 下一帧写另一槽
                        ev_frame_r <= 1'b1;
                        state    <= S_IDLE;
                    end
                end

                S_DROP: begin                     // 旁路丢弃：吃掉余下字节
                    if (!rx_dv)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            // 槽位归还（引擎释放）
            if (ret_valid)
                slot_busy[ret_slot] <= 1'b0;
        end
    end

endmodule
