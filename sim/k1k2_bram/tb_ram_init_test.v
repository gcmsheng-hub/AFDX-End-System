//=============================================================================
// K2 测试 testbench：验证 $readmemh 初始化内容正确
// 初始化规则（由 gen_init_hex.py 生成，tb 用同一公式自算期望值）：mem[i]=(i*7+11)&0xFF
//=============================================================================

`timescale 1ns/1ps

module tb_ram_init_test;

    reg         clk;
    reg         we;
    reg  [11:0] waddr;
    reg  [7:0]  wdata;
    reg  [11:0] raddr;
    wire [7:0]  rdata;

    integer errors = 0;

    initial clk = 1'b0;
    always #10 clk = ~clk;   // 50 MHz

    // 期望值公式与 gen_init_hex.py 一致
    function [7:0] expect_val(input [11:0] i);
        expect_val = (i * 7 + 11) & 8'hFF;
    endfunction

    // 读任务：发地址，下一拍取数并比对
    task check_read(input [11:0] addr);
        begin
            @(negedge clk);
            raddr = addr; we = 1'b0;
            @(negedge clk);
            if (rdata !== expect_val(addr)) begin
                $display("[TB] FAIL: mem[%0d]=%02x 期望 %02x", addr, rdata, expect_val(addr));
                errors = errors + 1;
            end else
                $display("[TB] OK: mem[%0d]=%02x", addr, rdata);
        end
    endtask

    initial begin
        we = 0; waddr = 0; wdata = 0; raddr = 0;
        #50;

        // 1) 验证初始内容（K2 核心）：含首地址、末地址、跨页地址
        check_read(12'd0);
        check_read(12'd1);
        check_read(12'd100);
        check_read(12'd2048);
        check_read(12'd4095);

        // 2) 写后读回（写 0x5A 到地址 5，改期望基准后核对）
        @(negedge clk);
        waddr = 12'd5; wdata = 8'h5A; we = 1'b1;
        @(negedge clk);
        we = 1'b0;
        @(negedge clk);
        raddr = 12'd5;
        @(negedge clk);
        if (rdata !== 8'h5A) begin
            $display("[TB] FAIL: 写后读 mem[5]=%02x 期望 5A", rdata);
            errors = errors + 1;
        end else
            $display("[TB] OK: 写后读 mem[5]=5A");

        if (errors == 0)
            $display("[TB] PASS: readmemh 初始化与读写全部正确");
        else
            $display("[TB] FAIL: %0d 处错误", errors);
        $finish;
    end

    ram_init_test u_ram (
        .clk   (clk),
        .we    (we),
        .waddr (waddr),
        .wdata (wdata),
        .raddr (raddr),
        .rdata (rdata)
    );

endmodule
