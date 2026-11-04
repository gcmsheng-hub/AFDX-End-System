//=============================================================================
// timebase.v — 64 位 ns 时间（v2 §4.2 core）
// 每拍 +NS_PER_CLK（125 MHz → 8 ns）；端到端时延/抖动打点用（doc/02 §6 测量点）。
//=============================================================================

module timebase #(
    parameter integer NS_PER_CLK = 8     // 125 MHz → 8 ns/拍
)(
    input  wire        clk,
    input  wire        reset,
    output reg  [63:0] time_ns
);

    always @(posedge clk) begin
        if (reset)
            time_ns <= 64'd0;
        else
            time_ns <= time_ns + NS_PER_CLK[5:0];
    end

endmodule
