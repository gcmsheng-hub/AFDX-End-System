//=============================================================================
// tx_sn_gen.v — 128 个 SN 计数器（v2 §4.2 tx；doc/01 §2.1 发送端 SN）
// 复位后首帧 SN=0（SN_FIRST，复位标记帧）；之后 1..255 循环，255→1 回绕
// （0 不再出现）。A/B 两份副本同 SN——每帧只读一次、递增一次（调度器每帧
// 发起一次读）。
//=============================================================================

module tx_sn_gen #(
    parameter integer TX_VL_NUM = 128,
    parameter [7:0]  SN_FIRST   = 8'd0,
    parameter [7:0]  SN_MAX     = 8'd255
)(
    input  wire       clk,
    input  wire       reset,
    input  wire       sn_re,           // 单拍：为本帧取 SN
    input  wire [6:0] sn_vl,
    output reg        sn_vld,          // 1 拍后有效
    output reg [7:0]  sn_val
);

    reg [7:0] sn_r [0:TX_VL_NUM-1];

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < TX_VL_NUM; i = i + 1)
                sn_r[i] <= SN_FIRST;
            sn_vld <= 1'b0;
            sn_val <= 8'h00;
        end else begin
            sn_vld <= sn_re;
            if (sn_re) begin
                sn_val     <= sn_r[sn_vl];
                // 0→1，1..254→+1，255→1（0 只在复位后首帧出现）
                sn_r[sn_vl] <= (sn_r[sn_vl] >= SN_MAX) ? 8'd1 : (sn_r[sn_vl] + 8'd1);
            end
        end
    end

endmodule
