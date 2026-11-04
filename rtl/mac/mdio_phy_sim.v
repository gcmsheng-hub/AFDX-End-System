//=============================================================================
// mdio_phy_sim.v — 仿真用虚拟 PHY（MDIO Clause 22 从机，最小实现）
// 目的：Tri_Mode_Ethernet_Mac 的 MDIO 主机复位后自动读 PHY 状态寄存器定
// speed/link（用户指南 §3.4.11.6）；纯仿真无外接 PHY，本模块在 mdio 线上
// 应答读操作，返回 BMCR/BMSR 预置值（1000M + 链路有效），保证 IP 内部
// speed=2'b10、link=1。真实硬件时去除本模块、mdio/mdc 接外 PHY。
// 协议：前导 1 任意 → ST(2b)=01 → OP(2b)：读=10 → PHYADDR(5) → REGADDR(5)
// → TA(2b)=Z0 → 数据(16b, MSB 先)由从机驱动。
// 可综合（纯寄存器/移位逻辑）；综合进网表体积可忽略。
//=============================================================================

module mdio_phy_sim #(
    parameter [15:0] BMCR_VAL = 16'h0140,   // Reg0：0.4Gbit/s(bit6)+全双工(bit8)
    parameter [15:0] BMSR_VAL = 16'h0024    // Reg1：AN 完成(bit5)+链路有效(bit2)
)(
    input  wire       mdc,
    inout  wire       mdio
);
    // 寄存器文件（读应答用；写忽略）
    reg [15:0] regs [0:1];
    initial begin
        regs[0] = BMCR_VAL;
        regs[1] = BMSR_VAL;
    end

    // 串行采样（mdc 上升沿）
    reg [4:0]  bitcnt;                    // 帧内位计数
    reg [1:0]  st;                        // ST 域
    reg [1:0]  op;                        // OP 域
    reg [4:0]  phy_addr, reg_addr;
    reg        ta_seen;
    reg [3:0]  dat_cnt;                   // 数据位计数（16b MSB 先）
    reg        op_read;                   // 本帧为读

    wire mdio_in = mdio;                  // 采样当前线电平（主机驱动期）

    // 从机驱动期（TA 第 2 拍起 16 拍）
    reg       drv_en;
    reg [15:0] drv_shr;
    assign mdio = drv_en ? drv_shr[15] : 1'bz;

    always @(posedge mdc) begin
        if (!ta_seen) begin
            // 域移位捕获（ST→OP→ADDR→REGADDR）
            st       <= {st[0],   mdio_in};
            op       <= {op[0],   st[1]};       // st 老值移入 op（流水对齐见下）
            phy_addr <= {phy_addr[3:0], op[1]};
            reg_addr <= {reg_addr[3:0], phy_addr[4]};
            bitcnt   <= bitcnt + 5'd1;
            // 位对齐说明：串行流依次 ST0 ST1 OP0 OP1 A4..A0 R4..R0，
            // 各寄存器错位捕获相邻旧位，bitcnt==13 时 reg_addr 恰收齐。
            if (bitcnt == 5'd13) begin
                ta_seen <= 1'b1;
                dat_cnt <= 4'd0;
                drv_shr <= regs[reg_addr[0]];  // 仅低 2 位寄存器（0/1）
                op_read <= (op[0] == 1'b0);    // OP=10（老值序）→ 读
                drv_en  <= (op[0] == 1'b0) && (phy_addr == 5'd0)
                                          && (reg_addr[4:1] == 4'd0);
            end
        end else begin
            if (!op_read) begin
                ta_seen <= 1'b0;               // 写帧：应答窗口结束即回监听
                drv_en  <= 1'b0;
            end else if (dat_cnt == 4'd15) begin
                ta_seen <= 1'b0;               // 16 位数据送毕
                drv_en  <= 1'b0;
            end else begin
                drv_shr <= {drv_shr[14:0], 1'b0};
                dat_cnt <= dat_cnt + 4'd1;
            end
        end
    end
endmodule
