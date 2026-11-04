//=============================================================================
// 练习工程：LED 闪烁（第一次跑通 eLinx 用）
// 命名按竞赛规范：端口/信号小写，参数大写
//=============================================================================

module led_blink (
    input  wire clk,       // 时钟输入（假定 50 MHz）
    input  wire reset,     // 同步复位，高有效
    output reg  led        // LED 输出，计数满周期翻转
);

    parameter [25:0] CNT_MAX = 26'd50_000_000;    // 1s 计数值（50MHz），仿真时可覆写小值

    reg [25:0] cnt;        // 分频计数器

    always @(posedge clk) begin
        if (reset) begin
            cnt <= 26'd0;
            led <= 1'b0;
        end else if (cnt == CNT_MAX - 1'b1) begin
            cnt <= 26'd0;
            led <= ~led;                       // 计满翻转，LED 以 2*1s 周期闪烁
        end else begin
            cnt <= cnt + 1'b1;
        end
    end

endmodule
