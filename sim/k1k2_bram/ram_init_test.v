//=============================================================================
// K1/K2 排雷测试：BRAM 推断 + $readmemh 初始化
// K1: 4096x8 同步 RAM，按 FPGA 综合 BRAM 推断友好风格书写（同步读、寄存输出）
//     在 eLinx 中综合后查看报告是否推断为 Block RAM（而非 LUT RAM/触发器阵列）
// K2: 复位后用 $readmemh 从 ram_init.hex 初始化——正是接收路径 LastSN 表的用法
//=============================================================================

module ram_init_test (
    input  wire        clk,
    input  wire        we,          // 写使能
    input  wire [11:0] waddr,       // 写地址（4096 深）
    input  wire [7:0]  wdata,       // 写数据
    input  wire [11:0] raddr,       // 读地址
    output reg  [7:0]  rdata        // 读数据（同步读，寄存输出）
);

    parameter INIT_FILE = "ram_init.hex";

    // 4096 x 8 位 RAM：同步读 → 可推断 BRAM
    reg [7:0] mem [0:4095];

    initial begin
        $readmemh(INIT_FILE, mem);   // K2：上电初始化
    end

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
        rdata <= mem[raddr];         // 同步读
    end

endmodule
