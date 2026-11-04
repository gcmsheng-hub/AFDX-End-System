//=============================================================================
// host_arb.v — 多主机发送客户端包级 RR（v2 §4.2 core；接口 v2 §4.3 ②）
// 包级仲裁：sop 获准后锁定该客户端直到 eop 传完（包内不切换、不打断）；
// 包间由 rr_arbiter 轮转。T6 只接 1 个客户端（应用口，信封→htx 转换在核级），
// NUM_HOST=1 时为纯透传；app_emu/管理面客户端随后续任务接入。
//=============================================================================

module host_arb #(
    parameter integer NUM_HOST = 1,     // 客户端数（2 的幂）
    parameter integer IW       = 1      // 索引位宽
)(
    input  wire              clk,
    input  wire              reset,

    // N 路客户端（扁平拼接，客户端 i 占位 [i] / [8i+:8] / [7i+:7]）
    input  wire [NUM_HOST-1:0] h_valid,
    input  wire [NUM_HOST-1:0] h_sop,
    input  wire [NUM_HOST-1:0] h_eop,
    input  wire [8*NUM_HOST-1:0] h_data,
    input  wire [7*NUM_HOST-1:0] h_vl,
    output wire [NUM_HOST-1:0] h_ready,

    // 汇聚输出（v2 §4.3 ②）
    output wire              o_valid,
    output wire              o_sop,
    output wire              o_eop,
    output wire [7:0]        o_data,
    output wire [6:0]        o_vl,
    input  wire              o_ready
);

    wire       rr_gnt;
    wire [IW-1:0] rr_idx;
    rr_arbiter #(.N(NUM_HOST), .IW(IW)) u_rr (
        .clk(clk), .reset(reset),
        .req(h_valid), .update(o_valid && o_ready && o_sop),
        .gnt(rr_gnt), .gidx(rr_idx)
    );

    reg        pkt_hold;                 // 包内锁定
    reg [IW-1:0] pkt_idx;
    wire [IW-1:0] cur = pkt_hold ? pkt_idx : rr_idx;

    assign o_valid = h_valid[cur];
    assign o_sop   = h_sop[cur];
    assign o_eop   = h_eop[cur];
    assign o_data  = h_data[cur*8 +: 8];
    assign o_vl    = h_vl[cur*7 +: 7];

    // h_ready 译码：整数比较（genvar 位选在部分工具下产生 Z，禁用）
    reg [NUM_HOST-1:0] hrdy_c;
    integer hj;
    always @(*) begin
        for (hj = 0; hj < NUM_HOST; hj = hj + 1)
            hrdy_c[hj] = (hj == cur) && o_ready;
    end
    assign h_ready = hrdy_c;

    always @(posedge clk) begin
        if (reset) begin
            pkt_hold <= 1'b0;
            pkt_idx  <= {IW{1'b0}};
        end else begin
            if (o_valid && o_ready && o_sop) begin
                pkt_hold <= !o_eop;      // 单字节包不锁定
                pkt_idx  <= cur;
            end else if (o_valid && o_ready && o_eop)
                pkt_hold <= 1'b0;
        end
    end

endmodule
