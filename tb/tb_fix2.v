//=============================================================================
// tb_fix2.v — FIX2/REV-002-04 定向用例：FIFO 满与 ready 同拍恢复竞态
//
// 场景（AFDX_top 全链实例化）：
//   1) VL3 预入队 4 条 1B 消息（tx_en 关）→ q_cnt[3]=4，信封 B 的 sop 被拒
//      （room=0），吸收 FIFO 此前为空
//   2) 开闸：VL3 消息 1 立即发出；信封 B（VLID 3，24B 载荷）自帧起始 +K 拍起
//      连续注入——前 8 个载荷字节恰填满 8B 吸收 FIFO（sk_full），buf_mgr 尚未
//      收过任何字节（presented=0）
//   3) 竞态拍：消息 1 末帧弹出→释放行走结束（room 恢复）与"count==8 且新载荷
//      字节到来"同拍 → sk_re 首次下送与满丢判定同拍：
//        旧代码 abort_pend<=presented(0) → 不回收 → enq_active 卡死 + 1 槽泄漏，
//        后续消息 SOP 永被拒（本用例失败）；
//        新代码 abort_pend<=presented || (sk_re && !sk_eop) → htx_abort 回收。
//   4) 断言：race_hit（竞态拍确实发生）+ 回收后 enq_active=0/无槽泄漏 + 下一条
//      消息 C 正常入队（q_cnt 3→4）并在队列排空后发帧（VLID=3）。
// K 扫描（每 trial 全复位重跑，命中即停）：帧完成流水 D = frag_last − 帧起始
//   为确定值，竞态要求 B 首载荷字节位于 D−7 拍附近；K∈[50,58]（实测 walk_busy 于帧起始 +61 拍、恢复 +63 拍）。
//=============================================================================

`timescale 1ns/1ps

module tb_fix2;

    parameter CLK_HALF = 4;
    integer errors;

    reg clk, reset;
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    task chk;
        input cond;
        input [511:0] name;
        begin
            if (cond !== 1'b1) begin
                $display("[FIX2-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // DUT：AFDX_top（cfg 走顶层引脚 VERDICT-015；GMII 层级只读观测）
    //-------------------------------------------------------------------------
    reg        tx_valid;
    reg  [7:0] tx_data;
    wire [7:0] rx_data;
    wire       rx_valid, tx_ready;
    reg        rx_ready;
    reg  [19:0] cfg_addr;
    reg  [31:0] cfg_wdata;
    reg         cfg_we, cfg_re;
    wire [31:0] cfg_rdata;
    wire        cfg_rvalid;

    AFDX_top u_dut (
        .clk(clk), .reset(reset),
        .tx_data(tx_data), .rx_data(rx_data),
        .tx_valid(tx_valid), .rx_valid(rx_valid),
        .tx_ready(tx_ready), .rx_ready(rx_ready),
        .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .cfg_we(cfg_we), .cfg_re(cfg_re),
        .cfg_rdata(cfg_rdata), .cfg_rvalid(cfg_rvalid)
    );

    // 竞态拍观测：吸收 FIFO 满 && 下游当拍弹出 && 新载荷字节 && 尚未进入丢弃态
    wire race_now = (u_dut.u_core.u_skid.count == 4'd8)
                 && u_dut.u_core.sk_re
                 && tx_valid
                 && !u_dut.u_core.dropping;

    //-------------------------------------------------------------------------
    // cfg 写 / 信封驱动
    //-------------------------------------------------------------------------
    task cfg_write;
        input [19:0] a;
        input [31:0] d;
        begin
            @(negedge clk);
            cfg_addr  = a;
            cfg_wdata = d;
            cfg_we    = 1'b1;
            @(negedge clk);
            cfg_we    = 1'b0;
        end
    endtask

    // 信封驱动：VLID 2B + plen 载荷连续流（valid 全程拉高）
    task drive_env;
        input [7:0] vl_lo;
        input integer plen;
        integer k;
        begin
            @(negedge clk);
            tx_valid = 1'b1; tx_data = 8'h00;    // VLID 高
            @(negedge clk);
            tx_data = vl_lo;                     // VLID 低
            for (k = 0; k < plen; k = k + 1) begin
                @(negedge clk);
                tx_data = 8'hA0 ^ k[7:0];
            end
            @(negedge clk);
            tx_valid = 1'b0;
        end
    endtask

    //-------------------------------------------------------------------------
    // GMII A 网抓帧（仅 en 有效拍采样）
    //-------------------------------------------------------------------------
    reg [7:0] capA [0:4095];
    integer capA_n, capA_frames, capA_base;
    integer capA_len [0:63];                  // 各帧字节数（含 8B 前导）
    reg      capA_in;

    // 帧序号 f 的起始偏移（capA_len 前缀和）
    function integer capA_base_of;
        input integer f;
        integer j, s;
        begin
            s = 0;
            for (j = 0; j < f; j = j + 1)
                s = s + capA_len[j];
            capA_base_of = s;
        end
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            capA_in = 1'b0;
        end else begin
            if (u_dut.u_core.gmii_a_en && !capA_in)
                capA_in = 1'b1;              // 阻塞赋值：上升首拍即采样
            if (capA_in && u_dut.u_core.gmii_a_en) begin
                capA[capA_n] = u_dut.u_core.gmii_a_d;
                capA_n = capA_n + 1;
            end
            if (!u_dut.u_core.gmii_a_en && capA_in) begin
                capA_in = 1'b0;
                capA_len[capA_frames] = capA_n - capA_base;   // 含 8B 前导
                capA_base = capA_n;
                capA_frames = capA_frames + 1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 单次 trial：全复位重配 → 预入队 4 条 → 开闸 → B 信封偏移 K 注入 → 观测
    //-------------------------------------------------------------------------
    integer guard, i0, i1;
    reg race_hit;
    reg [31:0] s_cyc;
    reg        d_seen;
    reg [31:0] cyc = 32'd0;                  // 周期计数（调试/对齐观测）
    always @(posedge clk) cyc <= cyc + 32'd1;


    wire dbg_trace = (cyc > 32'd1400) && (cyc < 32'd1560);
    always @(posedge clk) if (dbg_trace)
        $display("[FIX2-CYC] %0d txv=%b pv=%b peop=%b skwe=%b skv=%b htxv=%b sop=%b eop=%b vl=%02x rdy=%b dx=%b enqact=%b lenok=%b disc=%b mtot=%0d cvl=%0d smax3=%h evenq=%b evl=%b qcnt=%0d",
                 cyc, tx_valid, u_dut.u_core.p_valid,
                 u_dut.u_core.p_eop, u_dut.u_core.u_skid.we === 1'b1,
                 u_dut.u_core.sk_vld, u_dut.u_core.htx_valid,
                 u_dut.u_core.htx_sop, u_dut.u_core.htx_eop,
                 u_dut.u_core.htx_vl, u_dut.u_core.htx_ready_w,
                 u_dut.u_core.u_bm.do_xfer, u_dut.u_core.u_bm.enq_active,
                 u_dut.u_core.u_bm.len_ok, u_dut.u_core.u_bm.enq_discard,
                 u_dut.u_core.u_bm.msg_total, u_dut.u_core.u_bm.cur_vl,
                 u_dut.u_core.u_bm.smax_r[3], u_dut.u_core.u_bm.ev_enq,
                 u_dut.u_core.u_bm.ev_qfull, u_dut.u_core.u_bm.q_cnt[3]);

    task trial;
        input integer koff;
        output hit;
        reg ok;
        begin
            hit = 0; ok = 1;
            reset = 1'b1; tx_valid = 1'b0; tx_data = 8'h00;
            capA_frames = 0; capA_n = 0; capA_base = 0; capA_in = 0; race_hit = 0; d_seen = 0;
            repeat (4) @(posedge clk);
            @(negedge clk);
            reset = 1'b0;
            guard = 0;
            while (u_dut.u_core.init_done !== 1'b1 && guard < 50000) begin
                @(posedge clk); #1; guard = guard + 1;
            end
            if (u_dut.u_core.init_done !== 1'b1) begin
                $display("[FIX2-TB] trial K=%0d: init timeout", koff);
                errors = errors + 1;
                ok = 0;
            end
            if (ok) begin
                // VL3：vlid=3, bag=1ms, smax=1518, mask=A|B
                cfg_write(20'h01000 + (3 << 3) + 1, 32'h0000FFFF);
                cfg_write(20'h01000 + (3 << 3) + 2, 32'h00000003);
                cfg_write(20'h01000 + (3 << 3) + 3, 32'h00000003);
                cfg_write(20'h01000 + (3 << 3) + 5, 32'h00001389);
                cfg_write(20'h01000 + (3 << 3) + 6, 32'h0000138A);
                cfg_write(20'h01000 + (3 << 3) + 0, 32'h80000001);
                cfg_write(20'h00001, 32'h00000A17);
                cfg_write(20'h00002, 32'h0A000205);
                // 预入队 4 条 1B 消息（tx_en 关，队列填满）；信封流水有 2-3 拍
                // 滞后，轮询 q_cnt 到 4 为止
                repeat (4) drive_env(8'h03, 1);
                guard = 0;
                while (u_dut.u_core.u_bm.q_cnt[3] !== 3'd4 && guard < 200) begin
                    @(posedge clk); #1; guard = guard + 1;
                end
                if (u_dut.u_core.u_bm.q_cnt[3] !== 3'd4) begin
                    $display("[FIX2-TB] trial K=%0d: pre-queue q_cnt=%0d exp 4",
                             koff, u_dut.u_core.u_bm.q_cnt[3]);
                    errors = errors + 1;
                    ok = 0;
                end
            end
            if (ok) begin
                // 开闸，等消息 1 的帧起始（en 上升拍 = K 的原点）
                cfg_write(20'h00003, 32'h1);
                guard = 0;
                while (u_dut.u_core.gmii_a_en !== 1'b1 && guard < 50000) begin
                    @(posedge clk); #1; guard = guard + 1;
                end
                if (u_dut.u_core.gmii_a_en !== 1'b1) begin
                    $display("[FIX2-TB] trial K=%0d: no frame start", koff);
                    errors = errors + 1;
                    ok = 0;
                end
                s_cyc = cyc;
            end
            if (ok) begin
                // B 信封与竞态观测并行（观测须覆盖注入窗口）
                fork
                    begin
                        @(negedge clk);
                        repeat (koff - 2) @(posedge clk);
                        drive_env(8'h03, 24);
                    end
                    begin
                        guard = 0;
                        while (guard < 300 && !race_hit) begin
                            @(posedge clk); #1;
                            if (race_now) race_hit = 1;
                            guard = guard + 1;
                        end
                    end
                join
                repeat (200) @(posedge clk);
                hit = race_hit;
                if (race_hit) begin
                    $display("[FIX2-TB] race hit at K=%0d", koff);
                    // 回收清洁：enq_active 清零、无槽泄漏（32 − 4 预入队 + 1 释放）
                    chk(u_dut.u_core.u_bm.enq_active === 1'b0,
                        "race: enq_active cleared after abort");
                    chk(u_dut.u_core.u_bm.free_cnt === 6'd29,
                        "race: no slot leak (32-4+1)");
                    // 下一条消息 C 正常入队（q_cnt 3→4；旧代码 sop 被卡死的
                    // enq_active 拒收，q_cnt 停在 3）
                    drive_env(8'h03, 1);
                    guard = 0;
                    while (u_dut.u_core.u_bm.q_cnt[3] !== 3'd4 && guard < 200) begin
                        @(posedge clk); #1; guard = guard + 1;
                    end
                    chk(u_dut.u_core.u_bm.q_cnt[3] === 3'd4,
                        "race: next message C accepted (q_cnt 3->4)");
                    chk(u_dut.u_core.u_bm.free_cnt === 6'd28,
                        "race: C occupies 1 slot (32-4)");
                    // C 在 msg2..4 之后发帧（BAG 1ms ×4），帧 VLID=3、长度 64
                    guard = 0;
                    while (capA_frames < 5 && guard < 1_500_000) begin
                        @(posedge clk); #1; guard = guard + 1;
                    end
                    chk(capA_frames >= 5, "race: C frame captured (5th)");
                    if (capA_frames >= 5) begin
                        if ({capA[capA_base_of(4) + 12], capA[capA_base_of(4) + 13]}
                                !== 16'h0003) begin
                            $display("[FIX2-TB] FAIL: frame4 VLID=%02x%02x exp 0003",
                                     capA[capA_base_of(4) + 12],
                                     capA[capA_base_of(4) + 13]);
                            errors = errors + 1;
                        end
                        if (capA_len[4] !== 72) begin   // 64B 帧 + 8B 前导
                            $display("[FIX2-TB] FAIL: frame4 len %0d exp 72",
                                     capA_len[4]);
                            errors = errors + 1;
                        end
                    end
                end
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 主流程：K 扫描
    //-------------------------------------------------------------------------
    integer koff, done;
    reg hit;
    initial begin : main
        errors = 0; done = 0;
        for (koff = 50; koff <= 58 && !done; koff = koff + 1) begin
            trial(koff, hit);
            if (hit)
                done = 1;
        end
        chk(done === 1'b1,
            "same-cycle FIFO-full + ready-recovery race exercised (K sweep)");
        if (errors == 0)
            $display("[FIX2-TB] PASS");
        else
            $display("[FIX2-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("[FIX2-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
