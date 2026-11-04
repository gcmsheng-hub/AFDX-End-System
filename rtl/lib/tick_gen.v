//=============================================================================
// tick_gen.v — 精确 1 µs tick（v2 §4.2 lib 基础件，相位累加器 / Bresenham）
// 每拍累加 NS_PER_CLK（125 MHz → 8 ns），累加满 TICK_NS（1000 ns）出一个单拍
// tick。125 MHz 下 1000/8=125 整除，tick 严格每 125 拍一次、无累积误差；
// NS_PER_CLK 非 8 时同样无长期漂移（余数保留在累加器）。
//=============================================================================

module tick_gen #(
    parameter integer NS_PER_CLK = 8,    // 每时钟周期对应的 ns（125 MHz → 8）
    parameter integer TICK_NS    = 1000  // tick 周期（ns），1 µs
)(
    input  wire clk,
    input  wire reset,
    output reg  tick                        // 单拍脉冲
);

    // 累加器位宽覆盖 TICK_NS 即可（TICK_NS < 65536）
    reg [15:0] acc;
    wire [16:0] nxt = {1'b0, acc} + NS_PER_CLK;

    always @(posedge clk) begin
        if (reset) begin
            acc  <= 16'h0000;
            tick <= 1'b0;
        end else begin
            tick <= 1'b0;
            if (nxt >= TICK_NS) begin
                tick <= 1'b1;
                acc  <= nxt[15:0] - TICK_NS[15:0];   // 保留余数
            end else begin
                acc  <= nxt[15:0];
            end
        end
    end

endmodule
