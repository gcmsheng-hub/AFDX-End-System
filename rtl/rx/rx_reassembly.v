//=============================================================================
// rx_reassembly.v — IP 分片重组（v2 §4.2 rx_reassembly；doc/06 TC-FRG-01~06）
//
// 上下文：2 个并行消息上下文，键 =（VL, IP ID）。每上下文 8KB BRAM 载荷缓冲
// + 1024b 块位图（8B/块，与片偏移粒度一致）+ {total, got, have_total,
// maxend, tail, tend}。引擎在 RM 接受分片后经 f_* 通道写入；单帧报文
// （MF=0 且 off=0）不走本模块。
//
// 写入协议（一次一片，引擎串行）：
//   f_alloc（任一分片前单拍，f_vl/f_ipid 有效）→ f_alloc_ok 组合回显：
//     同键忙上下文已在座 → 本次 alloc 为无操作（乱序中/尾片先到复用上下文，
//     VERDICT-010 §1-01）；无匹配且有空座 → 占新座（total 未知，c_have=0）；
//     无匹配且无空座（f_alloc_ok=0）→ 引擎整帧丢弃计 ctxerr。
//   f_start 单拍（f_off8/f_len/f_mf/f_udplen）→ 随后 f_len 拍 f_valid 字节流
//   → 片尾（W_END）完整性判定（VERDICT-010 §1-02 接受不变量：所有片 ⊆
//   [0,total)、两两不重叠、长度和 == total、唯一尾片终点 == total ⟹
//   [0,total) 全覆盖；违反任一即 f_aborted 并整上下文释放）：
//     位图相交（重叠）         → abort
//     首片到达（total 确定拍） → total ∉ [1, MAX_MSG] / 本片越 total /
//                                先到片 maxend 回溯越 total / 尾片终点回溯
//                                不符 → abort
//     total 已知后片终点 > total → abort
//     第二个 MF=0 片           → abort
//     尾片终点 != total        → abort
//     全部通过且 got==total 且已见首尾片 → f_done 单拍（等 r_start 读出）
//     不齐备不 abort（后续片可补齐，TC-FRG-04 乱序容忍），补不齐由重组
//     定时器回收（TC-FRG-05）。
//   f_start 无匹配上下文（VL/IP ID 不符或未 alloc）→ f_aborted（防御）。
//   位图置位于片尾一拍完成（掩码 = off8 起 ceil(len/8) 个块）；got 按片累加。
//
// 超时：忙且未齐备的上下文按 µs 计时（片尾续期），≥ TIMEOUT 拍数释放并
// f_timeout 单拍（doc/06 TC-FRG-05：10ms 定时器，缺片不交付残片）。
//
// 读出：r_start/r_ctx → total 拍 r_valid/r_data 流（RAM 1 拍延迟已对齐），
// 末字节 r_last 拍引擎同拍 r_ack → 释放上下文（冻结握手：valid 高不撤销）。
//=============================================================================

module rx_reassembly #(
    parameter [13:0] MAX_MSG_W  = 14'd8192,   // 单条消息上限（= 核 MAX_MSG）
    parameter [13:0] TIMEOUT_W  = 14'd10000   // 重组定时器（µs，doc/06 TC-FRG-05）
)(
    input  wire        clk,
    input  wire        reset,
    input  wire        tick_us,

    // ---- 片写入通道 ----
    input  wire        f_alloc,          // 单拍：新消息占座
    input  wire [11:0] f_vl,
    input  wire [15:0] f_ipid,
    output wire        f_alloc_ok,       // 组合：有空座（当拍有效）
    input  wire        f_start,          // 单拍：开始一片（参数当拍有效）
    input  wire [9:0]  f_off8,           // 片偏移（8B 单位）
    input  wire [10:0] f_len,            // 本片载荷字节数
    input  wire        f_mf,             // 1=后续还有片
    input  wire [15:0] f_udplen,         // off8==0 时有效：UDP Length（total=udplen-8）
    input  wire [7:0]  f_data,
    input  wire        f_valid,          // 载荷字节有效（共 f_len 拍）
    output reg         f_busy,           // 1=片写入进行中（f_start 受理后置位）
    output reg         f_done,           // 单拍：消息齐备
    output reg  [1:0]  f_done_ctx,
    output reg         f_aborted,        // 单拍：异常终止（无座由 f_alloc_ok 表达）
    output reg  [1:0]  f_abort_ctx,
    output reg         f_timeout,        // 单拍：重组超时丢弃
    output reg  [1:0]  f_to_ctx,

    // ---- 齐备消息读出 ----
    input  wire        r_start,
    input  wire [1:0]  r_ctx,
    output reg  [7:0]  r_data,
    output reg         r_valid,
    output reg         r_last,
    input  wire        r_ack,            // 引擎在 r_last 拍同拍确认 → 释放上下文
    output wire [13:0] r_total           // 读出长度（r_start 拍锁存输出）
);

    integer i;

    //-------------------------------------------------------------------------
    // 上下文状态
    //-------------------------------------------------------------------------
    reg          busy  [0:1];
    reg          done  [0:1];               // 齐备待读出
    reg [11:0]   c_vl  [0:1];
    reg [15:0]   c_ipid[0:1];
    reg [13:0]   c_total[0:1];
    reg          c_have[0:1];               // total 已知（首片已到）
    reg [13:0]   c_got [0:1];
    reg          c_tail[0:1];               // 尾片（MF=0）已到
    reg [13:0]   c_tend[0:1];               // 尾片字节终点（回溯校验用）
    reg [13:0]   c_maxend[0:1];             // total 未知期先到片的最大终点
    reg [1023:0] c_bmp [0:1];               // 8B 块占用位图
    reg [13:0]   c_tim [0:1];               // µs 定时器（片尾续期）

    //-------------------------------------------------------------------------
    // 片覆盖块掩码：块号 [off8, off8+ceil(len/8)-1] 置 1。索引上界由
    // MAX_MSG 界检查保证：off8*8+len ≤ 8192 ⇔ off8+ceil(len/8) ≤ 1024。
    // 实现为纯移位掩码（eLinx yosys 32 位对 1024 次循环展开函数 OOM，
    // T8 综合实测；语义与逐位置位等价：nblk 个 1 左移 off8）。
    //-------------------------------------------------------------------------
    function [1023:0] frag_mask;
        input [9:0]  off8;
        input [10:0] len;
        begin
            frag_mask = ((1024'b1 << ((len + 11'd7) >> 3)) - 1024'b1) << off8;
        end
    endfunction

    //-------------------------------------------------------------------------
    // 写通道 FSM
    //-------------------------------------------------------------------------
    localparam [1:0] W_IDLE = 2'd0, W_DATA = 2'd1, W_END = 2'd2;
    reg [1:0]  wstate;
    reg [1:0]  wctx;                      // 当前写上下文号
    reg [10:0] wk;                        // 片内字节计数
    reg [12:0] wbase;                     // 片字节基址（off8*8）
    reg [10:0] wlen;
    reg        wmf;
    reg [15:0] wudplen;

    //-------------------------------------------------------------------------
    // 片尾完整性判决（W_END 用；VERDICT-010 §1-02 接受不变量：
    //   所有片 ⊆ [0,total) ∧ 两两不重叠 ∧ 长度和 == total ∧ 唯一尾片终点
    //   == total ⟹ [0,total) 连续覆盖。位图块级不重叠 ⟹ 字节级不重叠，
    //   故 got==total + 各片不越 total ⟹ 全覆盖，空洞消息不可能完成交付）
    //-------------------------------------------------------------------------
    wire [13:0] f_end   = {1'b0, wbase} + {3'b0, wlen};  // 本片字节终点
    wire [15:0] total_w = wudplen - 16'd8;               // 首片声明总载荷
    wire        w_first = (wbase == 13'd0);              // 本片为首片（off=0）
    // 重叠检测归约分片化（VERDICT-013 首选改法）：原一式的 1024 位全宽
    // `!= 1024'b0` 归约在 eLinx elaboration 期单核自旋挂（RPT-T8-005 单变量
    // 矩阵实证），拆为 16 片 64 位片内或归约再总或；16 片位区间两两不相交，
    // 逐位等价：任一片有交即重叠。frag_mask 调用与其余逻辑不动。
    wire [1023:0] ovlp_bits = c_bmp[wctx] & frag_mask(wbase[12:3], wlen);
    wire        end_ovlp = (|ovlp_bits[63:0])    | (|ovlp_bits[127:64])
                         | (|ovlp_bits[191:128]) | (|ovlp_bits[255:192])
                         | (|ovlp_bits[319:256]) | (|ovlp_bits[383:320])
                         | (|ovlp_bits[447:384]) | (|ovlp_bits[511:448])
                         | (|ovlp_bits[575:512]) | (|ovlp_bits[639:576])
                         | (|ovlp_bits[703:640]) | (|ovlp_bits[767:704])
                         | (|ovlp_bits[831:768]) | (|ovlp_bits[895:832])
                         | (|ovlp_bits[959:896]) | (|ovlp_bits[1023:960]);
    // 首片到达（total 确定拍）回溯校验：total 合法域 + 先到片/尾片终点复核
    wire        tot_bad  = w_first && !c_have[wctx]
                         && ((wudplen < 16'd9)                        // total≤0
                             || (total_w > {2'b00, MAX_MSG_W})        // 超 8KB
                             || ({5'b0, wlen} > total_w)              // 本片越 total
                             || (c_maxend[wctx] > total_w[13:0])      // 先到片回溯
                             || (c_tail[wctx] && (c_tend[wctx] != total_w[13:0])));
    wire        over_bad  = !w_first && c_have[wctx] && (f_end > c_total[wctx]);
    wire        tail2_bad = !wmf && c_tail[wctx];        // 第二尾片（金标准只许一个）
    wire        tend_bad  = !wmf && c_have[wctx] && (f_end != c_total[wctx]);
    wire        fend_bad  = end_ovlp || tot_bad || over_bad || tail2_bad || tend_bad;

    // f_alloc_ok：有空座 **或** 同键忙上下文已在座（VERDICT-010 §1-01：任一分片
    // 皆先 f_alloc——已建座消息的后续片在双座全忙时必须照常受理，引擎不得以
    // "无空座"为由丢弃；键互异由"同键不重复占座"分支保证）
    wire asm_key0 = busy[0] && !done[0] && c_vl[0] == f_vl && c_ipid[0] == f_ipid;
    wire asm_key1 = busy[1] && !done[1] && c_vl[1] == f_vl && c_ipid[1] == f_ipid;
    assign f_alloc_ok = !(busy[0] && busy[1]) || asm_key0 || asm_key1;

    // 载荷缓冲：两上下文各 8KB 拼一块 16KB BRAM（地址高位 = 上下文号）
    wire [13:0] waddr = (wctx == 2'd0 ? 14'd0 : 14'd8192) + {1'b0, wbase} + {3'b0, wk};
    wire [13:0] raddr;
    wire [7:0]  rdata_ram;

    sdp_ram #(.WIDTH(8), .DEPTH(16384), .AW(14)) u_buf (
        .clk(clk),
        .we((wstate == W_DATA) && f_valid), .waddr(waddr), .wdata(f_data),
        .raddr(raddr), .rdata(rdata_ram));

    //-------------------------------------------------------------------------
    // 读出通道声明（自下方读出块前上移至此）：写通道 always 块尾的 R 引擎
    // 释放分支（消 busy/done 多驱动，见块内注释）引用 rstate/R_RUN/R_WAIT/
    // rctx_q，Verilog 要求声明先于引用（iverilog -g2001 声明后用拒编、
    // eLinx elaboration 自旋挂，T8 第 6 轮模块二分实证）。只动声明位置：
    // localparam/reg 声明属细化期，连续 assign 与语句顺序无关，逐拍等价。
    //-------------------------------------------------------------------------
    localparam [1:0] R_IDLE = 2'd0, R_RUN = 2'd1, R_WAIT = 2'd2;
    reg [1:0]  rstate;
    reg [1:0]  rctx_q;
    reg [13:0] r_k;                       // 已发地址数
    reg [13:0] r_e;                       // 已呈现字节数
    reg [13:0] r_total_q;

    assign raddr   = (rctx_q == 2'd0 ? 14'd0 : 14'd8192) + {1'b0, r_k};
    assign r_total = r_total_q;

    always @(posedge clk) begin
        if (reset) begin
            wstate <= W_IDLE; f_busy <= 1'b0;
            f_done <= 1'b0; f_aborted <= 1'b0; f_timeout <= 1'b0;
            f_done_ctx <= 2'd0; f_abort_ctx <= 2'd0; f_to_ctx <= 2'd0;
            wctx <= 2'd0; wk <= 11'd0; wbase <= 13'd0;
            wlen <= 11'd0; wmf <= 1'b0; wudplen <= 16'd0;
            for (i = 0; i < 2; i = i + 1) begin
                busy[i] <= 1'b0; done[i] <= 1'b0; c_have[i] <= 1'b0;
                c_total[i] <= 14'd0; c_got[i] <= 14'd0;
                c_vl[i] <= 12'd0; c_ipid[i] <= 16'd0; c_tail[i] <= 1'b0;
                c_bmp[i] <= 1024'b0; c_tim[i] <= 14'd0;
                c_maxend[i] <= 14'd0; c_tend[i] <= 14'd0;
            end
        end else begin
            f_done <= 1'b0; f_aborted <= 1'b0; f_timeout <= 1'b0;   // 单拍默认

            // 超时扫描（写入进行中不计——该上下文定时器在片尾续期）
            if (!f_busy) begin
                if (busy[0] && !done[0] && tick_us) begin
                    c_tim[0] <= c_tim[0] + 14'd1;
                    if (c_tim[0] >= TIMEOUT_W) begin
                        busy[0] <= 1'b0; done[0] <= 1'b0;
                        f_timeout <= 1'b1; f_to_ctx <= 2'd0;
                    end
                end
                if (busy[1] && !done[1] && tick_us) begin
                    c_tim[1] <= c_tim[1] + 14'd1;
                    if (c_tim[1] >= TIMEOUT_W) begin
                        busy[1] <= 1'b0; done[1] <= 1'b0;
                        f_timeout <= 1'b1; f_to_ctx <= 2'd1;
                    end
                end
            end

            // 新消息占座 / 既有上下文复用（VERDICT-010 §1-01：任一首到分片皆可
            // 建上下文——同键忙上下文已在座则本次 alloc 为无操作，偏移非 0 的
            // 先到片同样能建座；空座优先 0 号）
            if (f_alloc && f_alloc_ok) begin
                if ((busy[0] && !done[0] && c_vl[0] == f_vl && c_ipid[0] == f_ipid)
                    || (busy[1] && !done[1] && c_vl[1] == f_vl && c_ipid[1] == f_ipid)) begin
                    ;                                    // 同键已在座：不重复占座
                end else if (!busy[0]) begin
                    busy[0] <= 1'b1; done[0] <= 1'b0; c_have[0] <= 1'b0;
                    c_got[0] <= 14'd0; c_tail[0] <= 1'b0;
                    c_vl[0] <= f_vl; c_ipid[0] <= f_ipid;
                    c_bmp[0] <= 1024'b0; c_tim[0] <= 14'd0;
                    c_maxend[0] <= 14'd0; c_tend[0] <= 14'd0;
                end else begin
                    busy[1] <= 1'b1; done[1] <= 1'b0; c_have[1] <= 1'b0;
                    c_got[1] <= 14'd0; c_tail[1] <= 1'b0;
                    c_vl[1] <= f_vl; c_ipid[1] <= f_ipid;
                    c_bmp[1] <= 1024'b0; c_tim[1] <= 14'd0;
                    c_maxend[1] <= 14'd0; c_tend[1] <= 14'd0;
                end
            end

            case (wstate)
                W_IDLE: begin
                    f_busy <= 1'b0;
                    if (f_start) begin
                        if (!(busy[0] && !done[0] && c_vl[0] == f_vl && c_ipid[0] == f_ipid)
                            && !(busy[1] && !done[1] && c_vl[1] == f_vl && c_ipid[1] == f_ipid)) begin
                            f_aborted   <= 1'b1;         // 无匹配（未 alloc / 键不符）
                            f_abort_ctx <= 2'd0;
                        end else if (({1'b0, f_off8, 3'b000} + {3'b0, f_len}) > MAX_MSG_W) begin
                            f_aborted   <= 1'b1;         // 越界（防御）
                            f_abort_ctx <= (busy[0] && !done[0] && c_vl[0] == f_vl
                                            && c_ipid[0] == f_ipid) ? 2'd0 : 2'd1;
                        end else begin
                            wctx   <= (busy[0] && !done[0] && c_vl[0] == f_vl && c_ipid[0] == f_ipid)
                                    ? 2'd0 : 2'd1;
                            wbase  <= {1'b0, f_off8, 3'b000};  // off8*8
                            wlen   <= f_len;
                            wmf    <= f_mf;
                            wudplen<= f_udplen;
                            wk     <= 11'd0;
                            wstate <= W_DATA;
                            f_busy <= 1'b1;
                        end
                    end
                end

                W_DATA: begin
                    if (f_valid) begin
                        wk <= wk + 11'd1;
                        if (wk + 11'd1 == wlen)
                            wstate <= W_END;
                    end
                end

                W_END: begin
                    f_busy <= 1'b0;
                    wstate <= W_IDLE;
                    if (fend_bad) begin
                        // 完整性违例（重叠/越界/回溯不符/双尾片/尾片终点不符）：
                        // 整上下文释放，零交付（Luna 原例口径，VERDICT-010 §1-02）
                        busy[wctx] <= 1'b0; done[wctx] <= 1'b0;
                        f_aborted   <= 1'b1;
                        f_abort_ctx <= wctx;
                    end else begin
                        // 记账：got += len；位图置位；首片补 total；尾片打标；续期
                        c_got[wctx] <= c_got[wctx] + {3'b0, wlen};
                        c_bmp[wctx] <= c_bmp[wctx] | frag_mask(wbase[12:3], wlen);
                        c_tim[wctx] <= 14'd0;
                        if (w_first) begin                   // 首片（含乱序后到）
                            c_total[wctx] <= total_w[13:0];  //   total = udplen-8
                            c_have[wctx]  <= 1'b1;
                        end else if (!c_have[wctx]) begin    // total 未知期先到片
                            if (f_end > c_maxend[wctx])      //   记录最大终点，
                                c_maxend[wctx] <= f_end;     //   首片到达时回溯校验
                        end
                        if (!wmf) begin                      // 尾片：打标 + 记终点
                            c_tail[wctx] <= 1'b1;
                            c_tend[wctx] <= f_end;
                        end
                        // 齐备判定（每片到达后统一判，乱序首片后到同样触发）：
                        //   total 已知 + 尾片已到 + got == total（配合片终点越界
                        //   检查与位图不重叠 ⟹ [0,total) 全覆盖）。不齐备不 abort
                        //   （后续片可补齐，TC-FRG-04 乱序容忍），补不齐由重组
                        //   定时器回收（TC-FRG-05，frag_timeout 计数）。
                        if ((c_have[wctx] || w_first)
                            && (c_tail[wctx] || !wmf)
                            && (c_got[wctx] + {3'b0, wlen}
                                == (w_first ? total_w[13:0] : c_total[wctx]))) begin
                            done[wctx]  <= 1'b1;
                            f_done      <= 1'b1;
                            f_done_ctx  <= wctx;
                        end
                    end
                end

                default: wstate <= W_IDLE;
            endcase

            //---- R 引擎上下文释放（迁自下方读出块）：busy/done 只允许单一
            //---- 过程驱动（ehiway_map 多驱动拒绝，iverilog 双驱动仿真侥幸）。
            //---- 释放条件逐拍等价（rstate/r_valid/r_last/rctx_q/r_ack 均为
            //---- 前值可读信号）；同拍 W 侧不会写同一上下文：alloc 需 !busy、
            //---- timeout 需 !done、completion 需 !done，均与"读出中 done=1"
            //---- 状态互斥，无同元素覆盖冲突，时序不变。
            if (rstate == R_RUN && r_valid && r_last && r_ack) begin
                busy[rctx_q] <= 1'b0; done[rctx_q] <= 1'b0;
            end else if (rstate == R_WAIT && r_ack) begin
                busy[rctx_q] <= 1'b0; done[rctx_q] <= 1'b0;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 读出通道：r_start → 发 total 个读地址（流水 1 拍）→ r_valid/r_data 对齐；
    // 末字节拍引擎同拍 r_ack → 释放上下文。（本通道 localparam/reg/assign
    // 声明已上移至写通道块之前——声明先于 W 块内释放分支的引用，见该处注释。）
    //-------------------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            rstate <= R_IDLE; rctx_q <= 2'd0;
            r_k <= 14'd0; r_e <= 14'd0; r_total_q <= 14'd0;
            r_valid <= 1'b0; r_last <= 1'b0; r_data <= 8'h0;
        end else begin
            r_valid <= 1'b0;
            r_last  <= 1'b0;
            case (rstate)
                R_IDLE: begin
                    if (r_start) begin
                        rctx_q    <= r_ctx;
                        r_total_q <= c_total[r_ctx];
                        r_k       <= 14'd0;
                        r_e       <= 14'd0;
                        rstate    <= R_RUN;
                    end
                end
                R_RUN: begin
                    if (r_k < r_total_q)               // 地址流水（领先数据 1 拍）
                        r_k <= r_k + 14'd1;
                    if (r_e < r_total_q && r_k > r_e) begin
                        r_valid <= 1'b1;
                        r_data  <= rdata_ram;          // r_k-1 = r_e 号字节
                        r_last  <= (r_e == r_total_q - 14'd1);
                        r_e     <= r_e + 14'd1;
                    end
                    if (r_valid && r_last && r_ack) begin
                        // 末字节同拍确认（引擎组合给出；正常路径）
                        // busy/done 释放已迁入上方 W 块统一驱动（消多驱动）
                        rstate <= R_IDLE;
                    end else if (r_e == r_total_q) begin
                        rstate <= R_WAIT;              // 兜底：错过同拍确认
                    end
                end
                R_WAIT: begin
                    if (r_ack) begin
                        // busy/done 释放已迁入上方 W 块统一驱动（消多驱动）
                        rstate <= R_IDLE;
                    end
                end
                default: rstate <= R_IDLE;
            endcase
        end
    end

endmodule
