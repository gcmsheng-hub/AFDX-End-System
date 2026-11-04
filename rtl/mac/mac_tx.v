//=============================================================================
// mac_tx.v — 发送 MAC（v2 §4.2 mac；v2 §5.1 CRC 要点）
// 职责：前导码/SFD 插入、FCS 计算+追加（低字节先发）、IFG、帧边界门控。
// 输入为组帧器的 s_stream 帧体（目的 MAC..SN，无 FCS）；IF_ID 字节（帧体第 11
// 字节）按本网替换后参与 CRC——A/B 两网 FCS 随之不同（doc/01 §1 源 MAC 行）。
// 两个实例（A/B）由调度器同拍启动、锁步消费同一 s_stream（副本同时发出，
// 满足 P7-24 冗余副本 0.5ms 时限）。
// 吸收 FIFO（8B）：start 后组帧器即流式送数，mac 前导 8 拍期间由 FIFO 吸收，
// BODY 期 1B/拍弹出——前导与帧体严丝合缝（无拖尾/空洞）。
// CRC32：反射 0xEDB88320，init/fin 0xFFFFFFFF，与 zlib.crc32 一致（v2 §5.1）。
//=============================================================================

module mac_tx #(
    parameter [2:0] IF_ID       = 3'b001,   // A 网 001 / B 网 010（doc/01 §1）
    parameter [7:0] IF_ID_BYTE  = 8'h20     // {IF_ID, 5'b0}：A=0x20 / B=0x40
)(
    input  wire        clk,
    input  wire        reset,

    input  wire        start,            // 单拍（调度器与对网同拍发出）
    input  wire [10:0] body_len,         // 帧体字节数（不含 FCS，来自组帧器）

    // s_stream（v2 §4.3 ④，与组帧器直连）
    input  wire        s_valid,
    output wire        s_ready,          // = FIFO 未满（start 后即可流式）
    input  wire        s_sop,            // 未强制使用（字节计数为主）
    input  wire        s_eop,
    input  wire [7:0]  s_data,

    // GMII 发送侧（8bit @125MHz；T8 经 mac_port_if 对接 eLinx MAC IP）
    output reg  [7:0]  gmii_d,
    output reg         gmii_en,          // 前导+SFD+帧体+FCS 期间为 1
    output wire        done              // 单拍：FCS+IFG 完毕
);

    // 前导/SFD
    localparam [7:0] PREAMBLE = 8'h55;
    localparam [7:0] SFD      = 8'hD5;
    localparam [3:0] IFG_LEN  = 4'd12;    // 帧间隔 ≥ 96ns = 12 字节拍

    localparam [2:0] S_IDLE = 3'd0, S_PRE = 3'd1, S_BODY = 3'd2,
                     S_FCSW = 3'd3, S_FCS = 3'd4, S_IFG = 3'd5;

    reg [2:0]  state;
    reg [10:0] bcnt;                      // 帧体字节计数
    reg [2:0]  pre_cnt;                   // 前导计数（0..6 = 55，7 = SFD）
    reg [1:0]  fcs_cnt;
    reg [3:0]  ifg_cnt;
    reg [31:0] fcs_r;
    wire [31:0] fcs_w;

    //-------------------------------------------------------------------------
    // 吸收 FIFO：start 后组帧器流式写入，BODY 期弹出
    //-------------------------------------------------------------------------
    wire       f_we;                      // 组合：组帧器有效且未满
    wire       f_re;                      // 组合：BODY 拍弹出（FWFT 对齐）
    wire [7:0] f_rdata;
    wire       f_full, f_empty;
    wire [4:0] f_cnt;
    reg        started;                   // start 后置位（s_ready 门控）

    sync_fifo #(.DW(8), .DEPTH(16), .AW(4), .AF_LEVEL(14)) u_skid (
        .clk(clk), .reset(reset),
        .we(f_we), .wdata(s_data), .re(f_re),
        .rdata(f_rdata), .full(f_full), .empty(f_empty),
        .count(f_cnt), .almost_full()
    );

    // started 后才收流（组帧器 S_RDYW 等此信号）；深度 16 覆盖前导 8 拍的积压
    assign s_ready = started && !f_full;
    assign f_we    = s_valid && s_ready;
    assign f_re    = (state == S_BODY) && !f_empty;

    // 帧体字节按网替换：第 11 字节（源 MAC 第 6 字节）= IF_ID_BYTE
    wire [7:0] body_byte = (bcnt == 11'd11) ? IF_ID_BYTE : f_rdata;

    crc32_d8 u_crc (
        .clk(clk), .reset(reset),
        .init(start),                        // start 拍重置 CRC
        .valid(f_re),                        // 弹出拍 = CRC 吸入拍
        .data(body_byte),
        .crc(), .fcs(fcs_w)
    );

    // done：最后一个 TX_EN 无效拍（ifg_cnt==IFG_LEN）——FCS 末字节已完整发出
    assign done = (state == S_IFG) && (ifg_cnt == IFG_LEN);

    always @(posedge clk) begin
        if (reset) begin
            state <= S_IDLE; gmii_d <= 8'h0; gmii_en <= 1'b0;
            bcnt <= 0; pre_cnt <= 0; fcs_cnt <= 0; ifg_cnt <= 0; fcs_r <= 0;
            started <= 1'b0;
        end else begin
            if (start)
                started <= 1'b1;
            case (state)
                S_IDLE: begin
                    gmii_en <= 1'b0;
                    if (start) begin
                        pre_cnt <= 3'd0;
                        bcnt    <= 11'd0;
                        state   <= S_PRE;
                    end
                end
                S_PRE: begin                             // 7×55 + SFD
                    gmii_en <= 1'b1;
                    gmii_d  <= (pre_cnt == 3'd7) ? SFD : PREAMBLE;
                    pre_cnt <= pre_cnt + 3'd1;
                    if (pre_cnt == 3'd7) begin
                        bcnt   <= 11'd0;
                        state  <= S_BODY;
                    end
                end
                S_BODY: begin                            // 1B/拍弹出（f_re 组合）
                    if (!f_empty) begin
                        gmii_d  <= body_byte;
                        bcnt    <= bcnt + 11'd1;
                        if (bcnt == body_len - 11'd1) begin
                            fcs_cnt <= 2'd0;
                            state   <= S_FCSW;           // CRC 末拍已在本沿推进
                        end
                    end
                end
                S_FCSW: begin                            // CRC 末字节已入寄存器
                    fcs_r  <= fcs_w;                     // = ~crc（低字节先发）
                    gmii_d <= fcs_w[7:0];
                    fcs_cnt<= 2'd1;
                    state  <= S_FCS;
                end
                S_FCS: begin                             // 4B FCS，低字节先发
                    gmii_d <= fcs_cnt[1] ? (fcs_cnt[0] ? fcs_r[31:24] : fcs_r[23:16])
                                         : (fcs_cnt[0] ? fcs_r[15:8] : fcs_r[7:0]);
                    if (fcs_cnt == 2'd3) begin
                        // REV-002-01：末 FCS 字节在本沿写出，转 S_IFG 后仍以
                        // TX_EN 有效驱动一个完整周期（gmii_en 此处不撤）
                        ifg_cnt <= 4'd0;
                        state   <= S_IFG;
                    end else
                        fcs_cnt <= fcs_cnt + 2'd1;
                end
                S_IFG: begin                             // 帧间隔 ≥ 12 字节拍
                    if (ifg_cnt == 4'd0)
                        gmii_en <= 1'b0;                 // 末 FCS 字节周期结束再撤 TX_EN
                    if (ifg_cnt == IFG_LEN) begin        // TX_EN 无效拍数 = IFG_LEN
                        state   <= S_IDLE;
                        started <= 1'b0;                 // 回 IDLE 停止收流
                    end else
                        ifg_cnt <= ifg_cnt + 4'd1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
