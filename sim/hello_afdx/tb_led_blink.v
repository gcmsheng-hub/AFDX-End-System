//=============================================================================
// 练习工程 testbench：LED 闪烁仿真
// 用小 CNT_MAX（1000 个时钟翻转一次），几毫秒内即可看到多次翻转
//=============================================================================

`timescale 1ns/1ps

module tb_led_blink;

    reg  clk;
    reg  reset;
    wire led;

    integer flip_cnt = 0;          // led 翻转次数统计

    // 50 MHz 时钟，周期 20ns
    initial clk = 1'b0;
    always #10 clk = ~clk;

    // 统计翻转次数（验证设计活着）
    always @(posedge led or negedge led)
        flip_cnt = flip_cnt + 1;

    // 复位后仿真 2ms 结束
    initial begin
        reset = 1'b1;
        #100;
        reset = 1'b0;
        #2_000_000;
        $display("[TB] done: led=%b, flip_cnt=%0d (期望>0)", led, flip_cnt);
        if (flip_cnt == 0)
            $display("[TB] FAIL: led 从未翻转");
        else
            $display("[TB] PASS");
        $finish;
    end

    // 例化被测模块，参数覆写为小计数（1000 个时钟 = 20us 翻转一次）
    led_blink #(
        .CNT_MAX (1000)
    ) u_blink (
        .clk   (clk),
        .reset (reset),
        .led   (led)
    );

endmodule
