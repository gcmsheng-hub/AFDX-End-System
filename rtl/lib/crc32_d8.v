//=============================================================================
// crc32_d8.v — 每拍 8 位并行 CRC32（v2 §4.2 lib 基础件）
// 反射多项式 0xEDB88320（即 0x04C11DB7 的位反序），初值/终值异或 0xFFFFFFFF。
// 用法：init 拍清寄存器；随后 valid 数据按字节流喂入（1B/拍）；
//       fcs = 寄存器取反 —— 与 Python zlib.crc32 一致（v2 §5.1 CRC 要点），
//       发送侧按 fcs 低字节先发（doc/01 §1 FCS 行）。
// 自测锚点：crc32("123456789") = 0xCBF43926。
//=============================================================================

module crc32_d8 (
    input  wire       clk,
    input  wire       reset,     // 同步高有效（整体复位到全 1 初值）
    input  wire       init,      // 单拍：开始新帧计算（crc <= 0xFFFFFFFF）
    input  wire       valid,     // 当拍 data 有效
    input  wire [7:0] data,
    output wire [31:0] crc,      // 内部寄存器值（未取反）
    output wire [31:0] fcs       // 最终值 = ~crc（低字节先发）
);

    // 单字节推进：x = crc ^ data；LSB 优先迭代 8 次
    function [31:0] crc32_next;
        input [31:0] c;
        input [7:0]  d;
        integer k;
        reg [31:0] x;
        begin
            x = c ^ {24'b0, d};
            for (k = 0; k < 8; k = k + 1)
                x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
            crc32_next = x;
        end
    endfunction

    reg [31:0] crc_r;

    always @(posedge clk) begin
        if (reset)
            crc_r <= 32'hFFFFFFFF;
        else if (init)
            crc_r <= 32'hFFFFFFFF;
        else if (valid)
            crc_r <= crc32_next(crc_r, data);
    end

    assign crc = crc_r;
    assign fcs = ~crc_r;

endmodule
