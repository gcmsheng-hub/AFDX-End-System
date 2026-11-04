//=============================================================================
// rx_redundancy.v — RM 冗余管理（P7-25~26 §3.2.6.2.2；v2 §5.4；doc/07 §4.4）
//
// 仅实现 MODE=1 环形距离窗（VERDICT-003 A 组：MODE=0 弃用）。每 VL 维护
// {valid, rsn[7:0], t_last[23:0]}（µs），RAM 4096×33b。
// 判决（rm_en 使能时；rm_en=0 由调用方旁路，A/B 帧全交付）：
//   !valid                 → accept（复位初态首帧，TC-SNRM-11）
//   sn == 0                → 复位旁路保留（VERDICT-009 §1 增补）+ SkewMax 窗内
//                            副本去重：RSN==0（上一接受帧即 SN=0，t_last 即其
//                            交付时刻）且 elapsed <= skew → dup 丢弃（A/B 双网
//                            副本只交付一次）；否则视为发送端再次复位，accept
//                            并重置 RSN=0（TC-SNRM-12 / FIX3 用例 rx5）
//   elapsed > skew         → accept（SkewMax 超窗无条件接受并重同步，TC-SNRM-09）
//   d == 0                 → dup（重复副本，First-Win 丢弃，TC-SNRM-06/08）
//   1 <= d <= RM_WIN(127)  → accept（递增新帧）
//   其余（128..255）       → old（旧帧 / 严重乱序丢弃；含 RSN=0 复位同步态
//                            时 sn=255 → d=255 记 old，VERDICT-010 §1-04 澄清）
// 距离公式（doc/07 §4.4 冻结 RTL 式）：
//   d = (sn >= rsn) ? (sn - rsn) : (sn - rsn + 255)   （8b 无符号）
//   —— 与模 255 映射式 d=((sn-1)-(rsn-1)+255)%255 在 sn,rsn∈[0,255]、且 sn==0
//   走旁路的前提下等价（rsn=0 时 sn=255 得 d=255 → 按 old 丢弃；穷举对拍口径）。
// elapsed = (us_now - t_last) mod 2^24（24b 回绕，v2 §5.4 原口径）。
// 接受时写回：{valid=1, rsn=sn, t_last=us_now}；丢弃不更新（t_last 不动）。
//
// 时序：t0 rm_start 锁存并发读地址 → t1 RAM 出旧值、组合判决、rm_vld 单拍
//       输出，接受时同拍写回。共 2 拍/次。输入 {rm_vl, rm_sn, rm_skew} 须保持
//       至 rm_vld 拍（串行引擎保证）。
// 上电清零：ram_init sel=6（4096×33b 全 0 → valid=0 全初态）。
//=============================================================================

module rx_redundancy #(
    parameter integer RX_VL_NUM = 4096,
    parameter [7:0]   RM_WIN     = 8'd127
)(
    input  wire        clk,
    input  wire        reset,
    input  wire        tick_us,

    input  wire        ri_we,             // ram_init sel=6 清零（4096 深）
    input  wire [11:0] ri_addr,

    input  wire        rm_start,          // 单拍：启动一次判决
    input  wire [11:0] rm_vl,             // VL 索引（vlid[11:0]）
    input  wire [7:0]  rm_sn,             // 当前帧 SN（IC 通过后的帧）
    input  wire [15:0] rm_skew,           // SkewMax（µs，VL 表 W1，保持至 rm_vld 拍）
    output wire        rm_vld,            // 单拍：判决有效（rm_start 后 1 拍）
    output wire        rm_accept,         // 1=接受转发 0=丢弃
    output wire [7:0]  rm_rsn_old,        // 观测：判决时读到的旧 RSN（tb 对拍用）
    output wire [23:0] rm_us              // 观测：当前 µs 计数
);

    // 24b µs 自由计数（16.7s 回绕；模 2^24 运算口径见 v2 §5.4）
    reg [23:0] us_cnt;
    always @(posedge clk) begin
        if (reset)        us_cnt <= 24'd0;
        else if (tick_us) us_cnt <= us_cnt + 24'd1;
    end
    assign rm_us = us_cnt;

    // 判决有效脉冲（t0 启动 → t1 输出；RAM 写口 we 引用，先声明）
    reg vld_r;
    wire [32:0] rdata;
    always @(posedge clk) begin
        if (reset) vld_r <= 1'b0;
        else       vld_r <= rm_start;
    end

    // 判决组合（t1 拍基于 RAM 旧值出结果）
    wire        valid = rdata[32];
    wire [7:0]  rsn   = rdata[31:24];
    wire [23:0] tlast = rdata[23:0];
    wire [23:0] elap  = us_cnt - tlast;                  // mod 2^24 回绕
    wire [7:0]  d     = (rm_sn >= rsn) ? (rm_sn - rsn) : (rm_sn - rsn + 8'd255);

    reg accept_c;
    always @(*) begin
        if (!valid)
            accept_c = 1'b1;                                // 复位初态首帧
        else if (rm_sn == 8'd0)
            // SN=0 复位旁路 + 窗内副本去重（VERDICT-009 §1）：RSN==0 说明上一
            // 接受帧即 SN=0（t_last 即其交付时刻）；elapsed <= skew 判副本 dup，
            // 超窗判发送端再次复位照常接受。正常帧 SN 恒非 0，RSN==0 只可能
            // 来自 SN=0 旁路，无需额外状态字。
            accept_c = !((rsn == 8'd0) && (elap <= {8'h00, rm_skew}));
        else if (elap > {8'h00, rm_skew})
            accept_c = 1'b1;                                // SkewMax 超窗强制接受
        else if (d == 8'd0)
            accept_c = 1'b0;                                // 重复副本
        else
            accept_c = (d <= RM_WIN);                       // 递增新帧 / 旧帧
    end

    //-------------------------------------------------------------------------
    // 状态 RAM：{valid, rsn[7:0], t_last[23:0]} × vl
    //-------------------------------------------------------------------------
    wire [11:0] raddr = rm_vl;
    wire [11:0] waddr = ri_we ? ri_addr : raddr;
    wire        we    = ri_we || (vld_r && accept_c);    // 接受拍写回
    wire [32:0] wdata = ri_we ? 33'h0 : {1'b1, rm_sn, us_cnt};

    sdp_ram #(.WIDTH(33), .DEPTH(RX_VL_NUM), .AW(12)) u_ram (
        .clk(clk), .we(we), .waddr(waddr), .wdata(wdata),
        .raddr(waddr), .rdata(rdata));

    assign rm_vld     = vld_r;
    assign rm_accept  = accept_c;
    assign rm_rsn_old = rsn;

endmodule
