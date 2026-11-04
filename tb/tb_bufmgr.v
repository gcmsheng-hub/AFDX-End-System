//=============================================================================
// tb_bufmgr.v — buf_mgr 单元测试（自检式）
// 覆盖：单帧消息入队/描述符/按链逐字节回读/弹出释放与 slot 复用；
//       跨 slot 消息（2049B）边界回读 + 分片推进 fidx；IP ID 按消息/VL 独立；
//       队列深度 4 拒收（ev_qfull 去重）+ 弹 1 条后续入队（D2，FIX2 回归）；
//       Smax 单帧/分片规则拒收（ev_lenerr）；
//       MAX_MSG 超长吞模式（8193B → 拒收且无 slot 泄漏）；8192B 恰好接受；
//       释放行走期间 sop 被拒；
//       FIX2 定向（REV-002-02/03）：H 同 VL 同拍入队+末帧弹出（q_cnt 净 0）、
//       I eop 拒收与末帧弹出同拍、J abort 与末帧弹出同拍（两请求均不丢、
//       free 槽回 32）、M 行走忙碌期请求队列承接。
// 观测：free_cnt/q_cnt/enq_active 经层级引用（u_dut.free_cnt / u_dut.q_cnt /
//       u_dut.enq_active）。
//=============================================================================

`timescale 1ns/1ps

module tb_bufmgr;

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
                $display("[BM-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // DUT
    //-------------------------------------------------------------------------
    reg         htx_valid, htx_sop, htx_eop;
    reg  [7:0]  htx_data;
    reg  [6:0]  htx_vl;
    reg         htx_abort;        // 半截入队回收脉冲（FIX2 用例 H/I/J/M）
    wire        htx_ready;
    wire [127:0] q_nonempty;

    reg         desc_req;
    reg  [6:0]  desc_vl;
    wire        desc_vld;
    wire [5:0]  desc_slot;
    wire [13:0] desc_len;
    wire [15:0] desc_ipid;
    wire [3:0]  desc_fidx;

    reg         frag_upd, frag_last;
    reg  [6:0]  frag_vl;
    reg  [5:0]  frag_slot0;
    reg  [13:0] frag_len;

    reg  [16:0] pre_addr;
    wire [7:0]  pre_data;

    reg  [5:0]  nx_ra;
    wire [5:0]  nx_rd;

    reg         cw_we;
    reg  [6:0]  cw_idx;
    reg  [2:0]  cw_wsel;
    reg  [31:0] cw_wdata;

    wire ev_enq, ev_qfull, ev_lenerr, ev_frame;

    buf_mgr u_dut (
        .clk(clk), .reset(reset),
        .htx_valid(htx_valid), .htx_ready(htx_ready),
        .htx_sop(htx_sop), .htx_eop(htx_eop),
        .htx_data(htx_data), .htx_vl(htx_vl),
        .htx_abort(htx_abort),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .q_nonempty(q_nonempty),
        .desc_req(desc_req), .desc_vl(desc_vl),
        .desc_vld(desc_vld), .desc_slot(desc_slot), .desc_len(desc_len),
        .desc_ipid(desc_ipid), .desc_fidx(desc_fidx),
        .frag_upd(frag_upd), .frag_last(frag_last), .frag_vl(frag_vl),
        .frag_slot0(frag_slot0), .frag_len(frag_len),
        .pre_addr(pre_addr), .pre_data(pre_data),
        .nx_ra(nx_ra), .nx_rd(nx_rd),
        .ev_enq(ev_enq), .ev_qfull(ev_qfull),
        .ev_lenerr(ev_lenerr), .ev_frame(ev_frame)
    );

    //-------------------------------------------------------------------------
    // 激励/采样工具
    //-------------------------------------------------------------------------
    integer k, i;
    reg     g_rdy_sop;        // sop 拍 htx_ready 采样
    reg     g_rejected;       // sop 被拒
    reg     g_ev_enq, g_ev_lenerr, g_ev_qfull, g_ev_frame;
    reg [5:0]  g_slot;
    reg [13:0] g_len;
    reg [5:0]  h_slot1;                   // FIX2 用例暂存（链头/链长）
    reg [13:0] h_len1;
    reg [15:0] g_ipid;
    reg [3:0]  g_fidx;
    reg [5:0]  cur_slot;
    reg [10:0] off;
    reg [15:0] ipid_exp;

    function [7:0] pat_byte;              // 载荷模式
        input integer sel;
        input integer idx;
        begin
            if (sel == 0)      pat_byte = (idx * 7) & 8'hFF;
            else if (sel == 1) pat_byte = (idx * 13 + 7) & 8'hFF;
            else               pat_byte = 8'hA5;
        end
    endfunction

    // 发一条消息：sop 拍采样 ready；被拒则保持一拍后放弃（模拟上层丢弃），
    // 否则整条送完。事件脉冲在收尾负沿采样进 g_ev_*（此时仍在脉冲周期内）。
    task send_msg;
        input [6:0]  vl;
        input integer len;
        input integer pat;
        integer kb;                          // 局部循环变量（勿覆盖调用方的 k）
        begin
            g_rejected = 0;
            for (kb = 0; kb < len; kb = kb + 1) begin
                @(negedge clk);
                htx_valid = 1'b1;
                htx_sop   = (kb == 0);
                htx_eop   = (kb == len - 1);
                htx_vl    = vl;
                htx_data  = pat_byte(pat, kb);
                #1;                          // 等组合稳定再采样（避开 delta 环）
                if (kb == 0) begin
                    g_rdy_sop  = htx_ready;
                    g_rejected = !htx_ready;
                end
                if (g_rejected) begin        // 保持一拍（让 ev_qfull 出脉冲）
                    @(posedge clk); #1;
                    g_ev_qfull = ev_qfull;
                    @(negedge clk);
                    htx_valid = 1'b0; htx_sop = 1'b0; htx_eop = 1'b0;
                    kb = 1 << 30;            // 终止本 task 的 for（V2001 无 break）
                end
            end
            if (!g_rejected) begin
                @(negedge clk);              // 收尾负沿：事件脉冲仍在
                htx_valid = 1'b0; htx_sop = 1'b0; htx_eop = 1'b0;
                g_ev_enq    = ev_enq;
                g_ev_lenerr = ev_lenerr;
            end
        end
    endtask

    // 读描述符：req 单拍，轮询 desc_vld（2 拍后与 rdata 同拍有效）
    task read_desc;
        input [6:0] vl;
        integer guard;
        begin
            @(negedge clk);
            desc_req = 1'b1; desc_vl = vl;
            @(negedge clk);
            desc_req = 1'b0;
            guard = 0;
            while (guard < 10) begin
                @(posedge clk); #1;
                if (desc_vld === 1'b1) begin
                    g_slot = desc_slot; g_len = desc_len;
                    g_ipid = desc_ipid; g_fidx = desc_fidx;
                    guard = 99;
                end else
                    guard = guard + 1;
            end
            chk(guard === 99, "desc_vld observed within 10 cycles");
        end
    endtask

    // 沿链逐字节回读并与模式比对
    task check_chain;
        input integer len;
        input integer pat;
        begin
            cur_slot = g_slot;
            nx_ra = cur_slot;
            for (i = 0; i < len; i = i + 1) begin
                off = i;             // 整数截断入 11b（V2001 不允许整数位选）
                @(negedge clk);
                pre_addr = {cur_slot, off};
                @(posedge clk); #1;
                if (pre_data !== pat_byte(pat, i)) begin
                    $display("[BM-TB] FAIL: chain byte %0d got %02x exp %02x",
                             i, pre_data, pat_byte(pat, i));
                    errors = errors + 1;
                end
                if ((off == 11'd2047) && (i != len - 1)) begin
                    cur_slot = nx_rd;           // 跨 slot：跟链（nx_ra 已指向当前槽）
                end else if (off != 11'd2047) begin
                    nx_ra = cur_slot;           // 保持指向当前槽备跨接
                end
            end
        end
    endtask

    // 帧完成脉冲（负沿→负沿）
    task frag_done;
        input last;
        input [6:0] vl;
        input [5:0] s0;
        input [13:0] l;
        begin
            @(negedge clk);
            frag_upd = 1'b1; frag_last = last; frag_vl = vl;
            frag_slot0 = s0; frag_len = l;
            @(negedge clk);
            frag_upd = 1'b0; frag_last = 1'b0;
            g_ev_frame = ev_frame;           // 收尾负沿采样（脉冲仍在）
        end
    endtask

    // 等释放行走结束（层级引用观测 free_cnt 回到期望值）
    task wait_free;
        input [5:0] expect_cnt;
        integer guard;
        begin
            guard = 0;
            while (u_dut.free_cnt !== expect_cnt && guard < 100) begin
                @(posedge clk);
                guard = guard + 1;
            end
            #1;
            chk(u_dut.free_cnt === expect_cnt, "free_cnt restored after walk");
        end
    endtask

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    // 发送消息前 len 字节（sop 在第 0 字节，无 eop 不收尾）——构造半截入队
    task send_msg_head;
        input [6:0]  vl;
        input integer len;
        input integer pat;
        integer kb;
        begin
            for (kb = 0; kb < len; kb = kb + 1) begin
                @(negedge clk);
                htx_valid = 1'b1;
                htx_sop   = (kb == 0);
                htx_eop   = 1'b0;
                htx_vl    = vl;
                htx_data  = pat_byte(pat, kb);
            end
            @(negedge clk);
            htx_valid = 1'b0; htx_sop = 1'b0;
        end
    endtask

    initial begin : main
        errors = 0; reset = 1'b1;
        htx_valid = 0; htx_sop = 0; htx_eop = 0; htx_data = 0; htx_vl = 0;
        htx_abort = 1'b0;
        desc_req = 0; desc_vl = 0;
        frag_upd = 0; frag_last = 0; frag_vl = 0; frag_slot0 = 0; frag_len = 0;
        pre_addr = 0; nx_ra = 0;
        cw_we = 0; cw_idx = 0; cw_wsel = 0; cw_wdata = 0;
        g_ev_qfull = 0; g_ev_enq = 0; g_ev_lenerr = 0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        repeat (2) @(posedge clk);

        //---------------------------------------------------------------
        // A. 100B 单帧消息：入队 → 描述符 → 回读 → 弹出释放 → slot 复用
        //---------------------------------------------------------------
        send_msg(7'd1, 100, 0);
        chk(g_rejected === 1'b0, "A: 100B msg sop accepted");
        chk(g_ev_enq === 1'b1, "A: ev_enq pulse");
        chk(q_nonempty[1] === 1'b1, "A: q_nonempty[1]");
        chk(u_dut.free_cnt === 6'd31, "A: free_cnt 32->31");
        read_desc(7'd1);
        chk(g_len === 14'd100, "A: desc len==100");
        chk(g_ipid === 16'd1, "A: desc ipid==1 (first msg)");
        chk(g_fidx === 4'd0, "A: desc fidx==0");
        check_chain(100, 0);
        g_ev_frame = 0;
        frag_done(1'b1, 7'd1, g_slot, g_len);
        chk(g_ev_frame === 1'b1, "A: ev_frame on last frag");
        chk(q_nonempty[1] === 1'b0, "A: queue empty after pop");
        wait_free(6'd32);

        // 复用：第二条消息 ipid 递增
        send_msg(7'd1, 100, 2);
        read_desc(7'd1);
        chk(g_ipid === 16'd2, "A2: ipid increments per message");
        chk(g_len === 14'd100, "A2: desc len==100");
        frag_done(1'b1, 7'd1, g_slot, g_len);
        wait_free(6'd32);

        //---------------------------------------------------------------
        // B. 2049B 跨 slot 消息：回读跨边界 + fidx 推进
        //---------------------------------------------------------------
        send_msg(7'd1, 2049, 1);
        chk(g_ev_enq === 1'b1, "B: ev_enq");
        read_desc(7'd1);
        chk(g_len === 14'd2049, "B: desc len==2049");
        chk(g_ipid === 16'd3, "B: ipid==3");
        check_chain(2049, 1);
        frag_done(1'b0, 7'd1, g_slot, g_len);       // 片 0 完成
        read_desc(7'd1);
        chk(g_fidx === 4'd1, "B: fidx advanced to 1");
        frag_done(1'b1, 7'd1, g_slot, g_len);       // 片 1（最后）完成
        wait_free(6'd32);
        chk(q_nonempty[1] === 1'b0, "B: popped after last frag");

        //---------------------------------------------------------------
        // C. 3000B（3 片）到 VL2：推进两片后弹出；IP ID 按 VL 独立（从 1 起）
        //---------------------------------------------------------------
        send_msg(7'd2, 3000, 1);
        read_desc(7'd2);
        chk(g_len === 14'd3000, "C: desc len==3000");
        chk(g_ipid === 16'd1, "C: VL2 ipid starts at 1 (per-VL counter)");
        frag_done(1'b0, 7'd2, g_slot, g_len);
        frag_done(1'b0, 7'd2, g_slot, g_len);
        read_desc(7'd2);
        chk(g_fidx === 4'd2, "C: fidx==2 after two non-last frags");
        frag_done(1'b1, 7'd2, g_slot, g_len);
        wait_free(6'd32);

        //---------------------------------------------------------------
        // D. 队列深度 4：VL3 连入 4 条 10B（ipid 1..4），第 5 条 sop 拒；
        //    D2（FIX2 回归）：弹 1 条后续入队成功、计数回 4，再全部弹出
        //---------------------------------------------------------------
        for (k = 0; k < 4; k = k + 1)
            send_msg(7'd3, 10, 2);
        chk(u_dut.q_cnt[3] === 3'd4, "D: q_cnt[3]==4");
        g_ev_qfull = 0;
        send_msg(7'd3, 10, 2);                      // 第 5 条：sop 应被拒
        chk(g_rejected === 1'b1, "D: 5th msg rejected at sop");
        chk(g_ev_qfull === 1'b1, "D: ev_qfull pulse");
        @(posedge clk); #1;
        chk(ev_qfull === 1'b0, "D: ev_qfull single-cycle");
        // D2: 弹出队首 1 条（释放 1 槽）→ 继续入队应成功
        read_desc(7'd3);
        frag_done(1'b1, 7'd3, g_slot, g_len);
        wait_free(6'd29);
        chk(u_dut.q_cnt[3] === 3'd3, "D2: q_cnt 4->3 after one pop");
        send_msg(7'd3, 10, 2);
        chk(g_rejected === 1'b0, "D2: enqueue after pop accepted");
        chk(g_ev_enq === 1'b1, "D2: ev_enq after pop");
        chk(u_dut.q_cnt[3] === 3'd4, "D2: q_cnt back to 4");
        for (k = 0; k < 4; k = k + 1) begin
            read_desc(7'd3);
            ipid_exp = k + 2;                       // D2 已弹走 ipid1，余 2..5
            chk(g_ipid === ipid_exp, "D: VL3 ipid sequence");
            frag_done(1'b1, 7'd3, g_slot, g_len);
        end
        wait_free(6'd32);
        chk(u_dut.q_cnt[3] === 3'd0, "D: q_cnt[3]==0 after drain");

        //---------------------------------------------------------------
        // E. Smax（应用载荷层级）：VL4 配 smax=100
        //---------------------------------------------------------------
        @(negedge clk);
        cw_we = 1; cw_idx = 7'd4; cw_wsel = 3'd1; cw_wdata = 32'd100;
        @(negedge clk);
        cw_we = 0;
        // E1: 101B > 100 → eop 拒收（sop 只查深度/槽位，不查 Smax）
        send_msg(7'd4, 101, 2);
        chk(g_rejected === 1'b0, "E1: 101B sop accepted (check at eop)");
        chk(g_ev_lenerr === 1'b1, "E1: ev_lenerr on 101B vs smax=100");
        chk(q_nonempty[4] === 1'b0, "E1: rejected msg not queued");
        wait_free(6'd32);
        // E2: 2000B 分片消息需 smax ≥ 1464 → 拒收
        send_msg(7'd4, 2000, 2);
        chk(g_ev_lenerr === 1'b1, "E2: 2000B rejected (smax<1464)");
        wait_free(6'd32);
        // E3: 100B == smax → 接受；且拒收不消耗 IP ID
        send_msg(7'd4, 100, 2);
        read_desc(7'd4);
        chk(g_ipid === 16'd1, "E3: ipid==1 (rejects do not consume ipid)");
        chk(g_len === 14'd100, "E3: 100B accepted");
        frag_done(1'b1, 7'd4, g_slot, g_len);
        wait_free(6'd32);

        //---------------------------------------------------------------
        // F. MAX_MSG：8193B → 超长吞模式拒收且 32 槽全回收；8192B 恰好接受
        //---------------------------------------------------------------
        send_msg(7'd1, 8193, 1);
        chk(g_ev_lenerr === 1'b1, "F1: 8193B rejected (MAX_MSG, discard mode)");
        wait_free(6'd32);
        send_msg(7'd1, 8192, 1);
        chk(g_ev_enq === 1'b1, "F2: 8192B accepted");
        read_desc(7'd1);
        chk(g_len === 14'd8192, "F2: desc len==8192");
        chk(u_dut.free_cnt === 6'd28, "F2: 4 slots in use");
        check_chain(8192, 1);
        // 分片推进/弹出（分片数由组帧器决定；此处验证游标与释放）
        frag_done(1'b0, 7'd1, g_slot, g_len);
        frag_done(1'b0, 7'd1, g_slot, g_len);
        frag_done(1'b0, 7'd1, g_slot, g_len);
        read_desc(7'd1);
        chk(g_fidx === 4'd3, "F2: fidx==3");
        frag_done(1'b1, 7'd1, g_slot, g_len);
        wait_free(6'd32);

        //---------------------------------------------------------------
        // G. 释放行走期间 sop 被拒（free_busy 门控）
        //---------------------------------------------------------------
        send_msg(7'd1, 100, 2);
        read_desc(7'd1);
        // frag_done 返回在 negedge A+8：posedge A+4 已置 walk_busy，A+12 才释放
        frag_done(1'b1, 7'd1, g_slot, g_len);
        htx_valid = 1'b1; htx_sop = 1'b1; htx_eop = 1'b0;
        htx_vl = 7'd5; htx_data = 8'h00;
        #1;
        chk(htx_ready === 1'b0, "G: htx_ready low during walk");
        @(negedge clk);
        htx_valid = 1'b0; htx_sop = 1'b0;
        wait_free(6'd32);
        htx_valid = 1'b1; htx_sop = 1'b1; htx_eop = 1'b0;
        htx_vl = 7'd5; htx_data = 8'h00;
        #1;
        chk(htx_ready === 1'b1, "G: room restored after walk");
        @(negedge clk);
        htx_valid = 1'b0; htx_sop = 1'b0;
        // 探测拍 sop 已被接受（VL5 半截消息占 1 槽）——htx_abort 回收后再继续，
        // 否则 enq_active 卡死会污染后续用例
        htx_abort = 1'b1;
        @(negedge clk);
        htx_abort = 1'b0;
        wait_free(6'd32);
        @(posedge clk); #1;
        chk(u_dut.enq_active === 1'b0, "G: probe message reclaimed by abort");

        //---------------------------------------------------------------
        // H.（FIX2/REV-002-02）同 VL 同拍入队(+1)与末帧弹出(-1)：q_cnt 净变化 0
        //    旧实现两处非阻塞赋值相减覆盖加，会把非空队列记成空——调度停发
        //---------------------------------------------------------------
        send_msg(7'd6, 100, 0);                     // msg1 → q_cnt=1
        read_desc(7'd6);
        h_slot1 = g_slot; h_len1 = g_len;
        send_msg(7'd6, 100, 0);                     // msg2 → q_cnt=2
        for (i = 0; i < 99; i = i + 1) begin        // msg3 前 99 字节（无 eop）
            @(negedge clk);
            htx_valid = 1'b1; htx_sop = (i == 0); htx_eop = 1'b0;
            htx_vl = 7'd6; htx_data = pat_byte(0, i);
        end
        @(negedge clk);                             // 对齐拍：msg3 末字节 + msg1 弹出
        htx_valid = 1'b1; htx_sop = 1'b0; htx_eop = 1'b1;
        htx_vl = 7'd6; htx_data = pat_byte(0, 99);
        frag_upd = 1'b1; frag_last = 1'b1; frag_vl = 7'd6;
        frag_slot0 = h_slot1; frag_len = h_len1;
        @(negedge clk);
        htx_valid = 1'b0; htx_eop = 1'b0; frag_upd = 1'b0; frag_last = 1'b0;
        g_ev_enq = ev_enq; g_ev_frame = ev_frame;   // 收尾负沿采样（脉冲仍在）
        chk(g_ev_enq === 1'b1 && g_ev_frame === 1'b1,
            "H: enq+frame pulses same cycle");
        chk(u_dut.q_cnt[6] === 3'd2, "H: q_cnt net 0 (2 -> 2)");
        wait_free(6'd30);
        read_desc(7'd6);                            // 弹出 msg2、msg3 收尾
        frag_done(1'b1, 7'd6, g_slot, g_len);
        read_desc(7'd6);
        chk(g_len === 14'd100, "H: msg3 descriptor intact");
        frag_done(1'b1, 7'd6, g_slot, g_len);
        wait_free(6'd32);
        chk(u_dut.q_cnt[6] === 3'd0 && q_nonempty[6] === 1'b0, "H: VL6 drained");

        //---------------------------------------------------------------
        // I.（FIX2/REV-002-03）eop 拒收释放与另一 VL 末帧弹出同拍：
        //    两个释放请求同拍入队，一个都不得丢（旧实现单选丢一个 → 槽泄漏）
        //---------------------------------------------------------------
        send_msg(7'd7, 100, 0);                     // VL7 msg（frag 侧）
        read_desc(7'd7);
        h_slot1 = g_slot; h_len1 = g_len;
        @(negedge clk);                             // VL9 配 smax=100（拒收侧）
        cw_we = 1; cw_idx = 7'd9; cw_wsel = 3'd1; cw_wdata = 32'd100;
        @(negedge clk);
        cw_we = 0;
        for (i = 0; i < 199; i = i + 1) begin       // VL9 200B 消息前 199 字节
            @(negedge clk);
            htx_valid = 1'b1; htx_sop = (i == 0); htx_eop = 1'b0;
            htx_vl = 7'd9; htx_data = pat_byte(2, i);
        end
        @(negedge clk);                             // 对齐拍：VL9 eop 拒收 + VL7 弹出
        htx_valid = 1'b1; htx_sop = 1'b0; htx_eop = 1'b1;
        htx_vl = 7'd9; htx_data = pat_byte(2, 199);
        frag_upd = 1'b1; frag_last = 1'b1; frag_vl = 7'd7;
        frag_slot0 = h_slot1; frag_len = h_len1;
        @(negedge clk);
        htx_valid = 1'b0; htx_eop = 1'b0; frag_upd = 1'b0; frag_last = 1'b0;
        g_ev_lenerr = ev_lenerr; g_ev_frame = ev_frame;
        chk(g_ev_lenerr === 1'b1 && g_ev_frame === 1'b1,
            "I: lenerr+frame pulses same cycle");
        chk(u_dut.q_cnt[9] === 3'd0, "I: rejected msg not queued");
        chk(u_dut.q_cnt[7] === 3'd0, "I: VL7 popped");
        wait_free(6'd32);                           // 两条链都必须走完（32 槽全回）

        //---------------------------------------------------------------
        // J.（FIX2/REV-002-03）abort 半截回收与末帧弹出同拍：两请求不丢
        //---------------------------------------------------------------
        send_msg(7'd10, 100, 0);                    // VL10 msg（frag 侧）
        read_desc(7'd10);
        h_slot1 = g_slot; h_len1 = g_len;
        send_msg_head(7'd11, 6, 2);                 // VL11 半截消息（sop+5B，占 1 槽）
        @(posedge clk); #1;
        chk(u_dut.enq_active === 1'b1, "J: half message enq_active");
        @(negedge clk);                             // 对齐拍：abort + VL10 末帧弹出
        htx_abort = 1'b1;
        frag_upd = 1'b1; frag_last = 1'b1; frag_vl = 7'd10;
        frag_slot0 = h_slot1; frag_len = h_len1;
        @(negedge clk);
        htx_abort = 1'b0; frag_upd = 1'b0; frag_last = 1'b0;
        @(posedge clk); #1;
        chk(u_dut.enq_active === 1'b0, "J: enq_active cleared by abort");
        wait_free(6'd32);
        chk(u_dut.q_cnt[10] === 3'd0 && u_dut.q_cnt[11] === 3'd0,
            "J: both VLs empty");

        //---------------------------------------------------------------
        // M.（FIX2/REV-002-03）行走忙碌期第二个释放请求：请求队列承接不覆盖
        //    （池全空 + 最低位优先分配 → msg1=槽0、msg2=槽1，若分配器变更此
        //    断言会显式失败）
        //---------------------------------------------------------------
        send_msg(7'd12, 100, 0);                    // msg1（1 槽链）
        read_desc(7'd12);
        h_slot1 = g_slot; h_len1 = g_len;
        chk(h_slot1 === 6'd0, "M: lowest-first alloc msg1 slot0==0");
        send_msg(7'd12, 100, 0);                    // msg2（槽 1）
        frag_done(1'b1, 7'd12, h_slot1, h_len1);    // 请求 1：行走启动
        frag_done(1'b1, 7'd12, h_slot1 + 6'd1, h_len1); // 请求 2：行走中 → 入队
        wait_free(6'd32);                           // 两条链都必须回满 32
        chk(u_dut.q_cnt[12] === 3'd0, "M: VL12 drained");

        //---------------------------------------------------------------
        // 结论
        //---------------------------------------------------------------
        if (errors == 0)
            $display("[BM-TB] PASS");
        else
            $display("[BM-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("[BM-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
