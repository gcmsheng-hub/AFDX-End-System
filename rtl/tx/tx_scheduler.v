//=============================================================================
// tx_scheduler.v — RR 选 VL、生成发送任务（v2 §4.2 tx；v2 §5.6 调度）
// 流程：RR 从 req 中选 VL → task_start（组帧器读表/描述符/SN）→ frame_ready →
// 按 net_mask 同拍启动 A/B 两网 mac（锁步消费组帧器的 s_stream，副本同时发出，
// A/B SFD 时差 = 0，严格满足 TC-SNRM-10 的 0.5ms 时限）→ 双 mac 完成（含各自
// FCS/IFG）→ task_done（BAG 重启、统计打点）。
// req 由核级合成：regulator.eligible & buf_mgr.q_nonempty & g_tx_en。
// RR 指针在 task_start 被接受当拍推进（rr_arbiter.update）。
//=============================================================================

module tx_scheduler #(
    parameter integer TX_VL_NUM = 128
)(
    input  wire         clk,
    input  wire         reset,

    input  wire [TX_VL_NUM-1:0] req,

    // 组帧器握手
    output reg          task_start,       // 单拍
    output reg  [6:0]   task_vl,
    input  wire         bld_busy,
    input  wire         frame_ready,      // 单拍
    input  wire [1:0]   fr_mask,          // bit0=A bit1=B
    input  wire [13:0]  fr_paylen,        // 本帧 UDP 载荷字节数（统计用）

    // mac 握手
    output reg          macA_start,       // 单拍
    input  wire         macA_done,        // 单拍（FCS+IFG 完）
    output reg          macB_start,
    input  wire         macB_done,

    // 任务完成（单拍）
    output reg          task_done,

    // 统计打点（与 task_done 同拍）
    output reg          st_frame,
    output reg  [6:0]   st_vl,
    output reg  [13:0]  st_paylen
);

    localparam [2:0] S_IDLE = 3'd0, S_BUILD = 3'd1, S_MACS = 3'd2,
                     S_WAITM = 3'd3, S_DONE = 3'd4;

    reg [2:0]  state;
    reg [6:0]  vl_r;
    reg [1:0]  mask_r;
    reg [13:0] paylen_r;

    wire       rr_gnt;
    wire [6:0] rr_idx;
    rr_arbiter #(.N(TX_VL_NUM), .IW(7)) u_rr (
        .clk(clk), .reset(reset),
        .req(req), .update(task_start), .gnt(rr_gnt), .gidx(rr_idx)
    );

    always @(posedge clk) begin
        if (reset) begin
            state <= S_IDLE;
            task_start <= 1'b0; task_vl <= 7'd0;
            macA_start <= 1'b0; macB_start <= 1'b0;
            task_done <= 1'b0; st_frame <= 1'b0; st_vl <= 7'd0; st_paylen <= 14'd0;
            vl_r <= 7'd0; mask_r <= 2'd0; paylen_r <= 14'd0;
        end else begin
            // 单拍脉冲默认
            task_start <= 1'b0; macA_start <= 1'b0; macB_start <= 1'b0;
            task_done  <= 1'b0; st_frame    <= 1'b0;

            case (state)
                S_IDLE:
                    if (rr_gnt && !bld_busy) begin
                        task_start <= 1'b1;           // 接受 grant，RR 指针推进
                        task_vl    <= rr_idx;
                        vl_r       <= rr_idx;
                        state      <= S_BUILD;
                    end
                S_BUILD:
                    if (frame_ready) begin
                        mask_r   <= fr_mask;
                        paylen_r <= fr_paylen;
                        state    <= S_MACS;
                    end
                S_MACS: begin
                    // 双网同拍启动（锁步消费 s_stream；单网 VL 只启使能网）
                    macA_start <= mask_r[0];
                    macB_start <= mask_r[1];
                    state      <= S_WAITM;
                end
                S_WAITM:
                    if ((!mask_r[0] || macA_done) && (!mask_r[1] || macB_done))
                        state <= S_DONE;
                S_DONE: begin
                    task_done <= 1'b1;
                    st_frame  <= 1'b1;
                    st_vl     <= vl_r;
                    st_paylen <= paylen_r;
                    state     <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
