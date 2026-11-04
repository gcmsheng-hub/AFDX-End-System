//=============================================================================
// csum16.v — 16 位反码求和累加器（v2 §4.2 lib 基础件）
// 按字节输入：clr 拍清零；valid 字节按出现次序交替作为 16 位字的高/低字节
// （第 1 个字节 = 高字节，即网络字节序流的自然累加）；进位回卷（end-around）。
// csum 输出 = ~sum（最后奇数字节需调用方补一个 0x00 再出结果——IP 头恒 20B 偶数，无此问题）。
// 用途：T7 接收侧 IP/UDP 头校验（发送侧由 tx_frame_builder 按寄存器直算）。
//=============================================================================

module csum16 (
    input  wire        clk,
    input  wire        reset,
    input  wire        clr,      // 单拍：清累加器，开始新计算
    input  wire        valid,
    input  wire [7:0]  data,
    output wire [15:0] csum      // = ~sum（随时可读；读前确保已喂完）
);

    reg  [15:0] sum;
    reg         hi;              // 1 = 下一个字节是 16 位字的高字节

    wire [16:0] addend = hi ? {data, 8'b0} : {8'b0, data};
    wire [16:0] acc    = {1'b0, sum} + addend;

    always @(posedge clk) begin
        if (reset || clr) begin
            sum <= 16'h0000;
            hi  <= 1'b1;         // 复位后第一个字节视为高字节
        end else if (valid) begin
            // end-around carry：17 位进位折回加到低 16 位
            sum <= acc[15:0] + {15'b0, acc[16]};
            hi  <= ~hi;
        end
    end

    assign csum = ~sum;

endmodule
