//=============================================================================
// rr_arbiter.v — N 路循环仲裁（v2 §4.2 lib 基础件，N=4/128 两处使用）
// 从上次获准索引的下一个位置开始轮转（上次获准者本次优先级最低）。
// gnt/gidx 纯组合输出；update=1 且 gnt=1 的当拍锁存指针（调度器在 grant
// 被接受的那拍给 update 单拍脉冲）。N 必须为 2 的幂。
//=============================================================================

module rr_arbiter #(
    parameter integer N  = 4,    // 请求数（2 的幂）
    parameter integer IW = 2     // 索引位宽，N ≤ 2^IW
)(
    input  wire         clk,
    input  wire         reset,
    input  wire [N-1:0] req,
    input  wire         update,  // 单拍：接受当前 grant，推进轮转指针
    output wire         gnt,     // = |req
    output wire [IW-1:0] gidx
);

    reg [IW-1:0] last;           // 上次获准索引；0 为复位初值

    localparam [IW-1:0] MASK = N - 1;    // N=2^IW；N=1 时 MASK=0，g 恒 0

    integer k;
    integer idx;
    reg [IW-1:0] g;
    reg found;

    // 环形优先扫描：k=1..N，idx = (last + k) mod N（N 为 2 的幂，用掩码实现）
    always @(*) begin
        g = {IW{1'b0}};
        found = 1'b0;
        for (k = 1; k <= N; k = k + 1) begin
            idx = last + k;
            if (!found && req[idx & MASK]) begin
                g = idx[IW-1:0] & MASK;
                found = 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (reset)
            last <= {IW{1'b0}};
        else if (update && gnt)
            last <= g;
    end

    assign gnt  = |req;
    assign gidx = g;

endmodule
