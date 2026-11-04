//=============================================================================
// sync_fifo.v — 同步 FIFO（v2 §4.2 lib 基础件）
// 带 count / almost_full；FWFT（rdata 恒指向队首，re 当拍弹出当前 rdata）。
// 满（full）时写被忽略、空（empty）时读被忽略——调用方先看 full/empty。
// 深度必须为 2 的幂（环形指针自然回绕）。
//=============================================================================

module sync_fifo #(
    parameter integer DW       = 8,
    parameter integer DEPTH    = 16,        // 2 的幂
    parameter integer AW       = 4,         // 指针位宽 = log2(DEPTH)
    parameter integer AF_LEVEL = 14        // almost_full 阈值（count ≥ AF_LEVEL）
)(
    input  wire        clk,
    input  wire        reset,             // 同步高有效
    input  wire        we,
    input  wire [DW-1:0] wdata,
    input  wire        re,
    output wire [DW-1:0] rdata,           // FWFT：队首值
    output wire        full,
    output wire        empty,
    output wire [AW:0] count,
    output wire        almost_full
);

    reg [DW-1:0] mem [0:DEPTH-1];
    reg  [AW-1:0] wp, rp;
    reg  [AW:0]   cnt;

    wire do_w = we    && !full;
    wire do_r = re    && !empty;

    assign full        = (cnt == DEPTH[AW:0]);
    assign empty       = (cnt == 0);
    assign count       = cnt;
    assign almost_full = (cnt >= AF_LEVEL[AW:0]);
    assign rdata       = mem[rp];

    always @(posedge clk) begin
        if (reset) begin
            wp <= {AW{1'b0}};
            rp <= {AW{1'b0}};
            cnt <= {(AW+1){1'b0}};
        end else begin
            if (do_w) begin
                mem[wp] <= wdata;
                wp <= wp + 1'b1;
            end
            if (do_r)
                rp <= rp + 1'b1;
            case ({do_w, do_r})
                2'b10:   cnt <= cnt + 1'b1;
                2'b01:   cnt <= cnt - 1'b1;
                default: ;                       // 同读同写 count 不变
            endcase
        end
    end

endmodule
