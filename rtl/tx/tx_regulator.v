//=============================================================================
// tx_regulator.v — BAG 整形（v2 §4.2 tx；v2 §5.6 发送整形）
// 每 VL 一个 17 位 µs 上计数器：cnt ≥ reload 即 BAG 到期（bag_ok）。
//   - W0 写（{en[31], res, bag_ms[7:0]}）：装 en/reload，cnt←reload（配置后立即
//     到期，首发不等待；phase 由 W4 单独装）
//   - W4 写（phase_us[15:0]）：cnt←phase（延迟首发，供离线调度错开各 VL）
//   - tick_us：cnt 未到 reload 时 +1（到齐保持）
//   - 帧发出（fs_pulse）：cnt←0，重新计一个 BAG（相邻帧起始间隔 ≥ BAG）
// eligible[i] = en & bag_ok（队列非空与全局使能由调度侧合并）。
// 预算：128×(17+17+1)b ≈ 4.5Kb FF（doc/02 §8.2 增补注记）。
//=============================================================================

module tx_regulator #(
    parameter integer TX_VL_NUM = 128,
    parameter integer IW = 7              // VL 索引位宽（fs_vl 比较）
)(
    input  wire        clk,
    input  wire        reset,
    input  wire        tick_us,          // 1 µs 单拍（tick_gen）

    // cfg 写广播（W0 / W4）
    input  wire        cw_we,
    input  wire [6:0]  cw_idx,
    input  wire [2:0]  cw_wsel,
    input  wire [31:0] cw_wdata,

    // 帧已发出（调度器任务完成，单拍）
    input  wire        fs_pulse,
    input  wire [6:0]  fs_vl,

    output wire [TX_VL_NUM-1:0] eligible
);

    reg [16:0] reload_r [0:TX_VL_NUM-1];
    reg [16:0] cnt_r    [0:TX_VL_NUM-1];
    reg        en_r     [0:TX_VL_NUM-1];

    wire        w0      = cw_we && (cw_wsel == 3'd0);
    wire        w4      = cw_we && (cw_wsel == 3'd4);
    wire [16:0] bag_us  = cw_wdata[7:0] * 13'd1000;    // BAG ms → µs（≤128000）

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < TX_VL_NUM; i = i + 1) begin
                reload_r[i] <= 17'd0;
                cnt_r[i]    <= 17'd0;
                en_r[i]     <= 1'b0;
            end
        end else begin
            if (w0) begin
                en_r[cw_idx]     <= cw_wdata[31];
                reload_r[cw_idx] <= bag_us;
                cnt_r[cw_idx]    <= bag_us + 17'd1;       // 配置即到期（严格 > 比较）
            end else if (w4) begin
                // 相位装载封顶（REV-002-06 / FIX2 #6，doc/02 §8.2 增补第 8 条）：
                // phase_us 合法域 [0, BAG)，超出封顶为 reload——cnt 初值不大于
                // reload，严格 > 判定下至多延迟 1 tick 到期
                cnt_r[cw_idx]    <= ({17'b0, cw_wdata[15:0]} > reload_r[cw_idx])
                                  ? reload_r[cw_idx]
                                  : {1'b0, cw_wdata[15:0]};
            end else begin
                // tick 计数（饱和于 reload+1：严格 > 的到期点。计满 reload+1 个
                // tick 补偿自由跑 tick 的相位损失（重装后 1000 tick 实际历时
                // 可能少至 BAG−124 拍），保证相邻帧 SFD 间隔恒 ≥ BAG——T6-006 实证）
                for (i = 0; i < TX_VL_NUM; i = i + 1)
                    if (tick_us && (cnt_r[i] <= reload_r[i]))
                        cnt_r[i] <= cnt_r[i] + 17'd1;
            end
            if (fs_pulse)
                cnt_r[fs_vl] <= 17'd0;                    // 帧后重新计一个 BAG
        end
    end

    // fs 当拍组合压制：task_done 与 BAG 重置同拍，若不压制，调度器在该拍仍看
    // 到旧 elig=1（cnt 尚未清零的旧值）而立即重授同一 VL，造成同 VL 两帧间隔
    // 远小于 BAG 的违规（S4 单测抓到）。
    // 严格 > ：自由跑 tick 的相位使"重装后 1000 个 tick"实际历时可能少至
    // BAG−124 拍（tick 相位损失），若用 >= 会让相邻帧 SFD 间隔最小到
    // BAG−1016 拍（doc/06 TC-VL 系列按 ≥BAG 判定会误报）；+1 tick 裕量后
    // 间隔恒 ≥ BAG（最多多 1µs，在抖动指标内）。
    genvar gi;
    generate
        for (gi = 0; gi < TX_VL_NUM; gi = gi + 1) begin: g_elig
            assign eligible[gi] = en_r[gi] && (cnt_r[gi] > reload_r[gi])
                              && !(fs_pulse && (fs_vl == gi[IW-1:0]));
        end
    endgenerate

endmodule
