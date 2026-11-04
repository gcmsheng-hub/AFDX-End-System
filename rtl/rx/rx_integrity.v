//=============================================================================
// rx_integrity.v — IC 完整性检查（P7-24 §3.2.6.2.1；v2 §5.3；doc/01 §4.3）
//
// 状态：按（VL × 网络）独立维护 {seen, psn[7:0]}，RAM 8192×9b（addr={net,vl}）。
// 判决（ic_en 使能时；ic_en=0 由调用方旁路）：
//   !seen 或 sn==0        → accept（初态 / 发送端复位标志 SN=0）
//   sn == nxt(psn)        → accept
//   sn == nxt(nxt(psn))   → accept_lost（窗口第二格 [PSN+2]，中间丢一帧）
//   其余                  → drop（大跳号 / 旧帧）
//   nxt(x) = (x==255 || x==0) ? 1 : x+1（255 回绕 1；0 只在复位首帧出现）
// PSN 强行更新语义（doc/01 §4.3）：不论接受与否 psn<=sn、seen<=1（重新同步）。
//
// 时序：t0 ic_start 拍锁存 {vl,net,sn} 并发 RAM 读地址 → t1 RAM 出旧值、组合
//       判决、ic_vld 单拍输出，同拍写口写回 {seen=1, sn}（sdp_ram 非阻塞：
//       读旧写新同地址无冒险；串行引擎保证同一 (vl,net) 不背靠背判决）。
// 上电清零：ram_init sel=5（8192×9b 全 0 → seen=0 全初态）。
//=============================================================================

module rx_integrity #(
    parameter integer RX_VL_NUM = 4096
)(
    input  wire        clk,
    input  wire        reset,

    input  wire        ri_we,             // ram_init sel=5 清零（8192 深）
    input  wire [12:0] ri_addr,

    input  wire        ic_start,          // 单拍：启动一次判决
    input  wire [11:0] ic_vl,             // VL 索引（vlid[11:0]）
    input  wire        ic_net,            // 0=A 网 1=B 网
    input  wire [7:0]  ic_sn,             // 当前帧 SN（帧尾）
    output wire        ic_vld,            // 单拍：判决有效（ic_start 后 1 拍）
    output wire [1:0]  ic_verdict,        // 0=drop 1=accept 2=accept_lost
    output wire [7:0]  ic_psn_old         // 观测：判决时读到的旧 PSN（tb 对拍用）
);

    //-------------------------------------------------------------------------
    // 判决有效脉冲（先声明——RAM 写口 we 引用）
    //-------------------------------------------------------------------------
    reg vld_r;
    always @(posedge clk) begin
        if (reset) vld_r <= 1'b0;
        else       vld_r <= ic_start;    // t0 启动 → t1 判决有效
    end

    //-------------------------------------------------------------------------
    // 状态 RAM：{seen, psn[7:0]} × {net, vl}
    //-------------------------------------------------------------------------
    wire [12:0] raddr = {ic_net, ic_vl};
    wire [12:0] waddr = ri_we ? {1'b0, ri_addr} : raddr;
    wire        we    = ri_we || vld_r;
    wire [8:0]  wdata = ri_we ? 9'h000 : {1'b1, ic_sn};   // seen=1, psn=sn
    wire [8:0]  rdata;

    sdp_ram #(.WIDTH(9), .DEPTH(2 * RX_VL_NUM), .AW(13)) u_ram (
        .clk(clk), .we(we), .waddr(waddr), .wdata(wdata),
        .raddr(waddr), .rdata(rdata));

    //-------------------------------------------------------------------------
    // 判决（t1 拍：rdata = ic_start 拍地址的旧值）
    //-------------------------------------------------------------------------
    wire       seen = rdata[8];
    wire [7:0] psn  = rdata[7:0];

    function [7:0] nxt;                   // SN 增量（255→1、0→1，其余 +1）
        input [7:0] x;
        begin
            nxt = (x == 8'd255 || x == 8'd0) ? 8'd1 : x + 8'd1;
        end
    endfunction

    reg [1:0] verd;
    always @(*) begin
        if (!seen || ic_sn == 8'd0)
            verd = 2'd1;                                   // accept
        else if (ic_sn == nxt(psn))
            verd = 2'd1;                                   // accept
        else if (ic_sn == nxt(nxt(psn)))
            verd = 2'd2;                                   // accept_lost
        else
            verd = 2'd0;                                   // drop
    end

    assign ic_vld     = vld_r;
    assign ic_verdict = verd;
    assign ic_psn_old = psn;

endmodule
