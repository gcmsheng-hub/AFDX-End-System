//=============================================================================
// tb_perf.v — T8 性能/边界专项 tb（GEM-003 §二 11 条待测清单，doc/06 口径）
// 结构：AFDX_top 全链 + tb 层级回环（同 tb_afdx_rx RTL 模式）+ 用例文件驱动。
// 用例文件（scripts/gen_perf_cases.py 产出，sim/tb_out/perf_<case>/）：
//   perf_cfg.txt    "T idx bag_ms smax vlid mask"（发送表，sport/dport 固定）
//                   "R idx vlid ic rm mask smax skew"（接收表）
//   perf_inject.txt "P <pace_div>"  全局注入口径：1B/pace_div 拍（消息内 1B/拍
//                                   连发，消息间空档 = pace_div×len − len 拍）
//                   "M <max_ms>"    交付等待上限（ms）
//                   "I <vlid> <len> <count>"  注入条目（len=信封总长含 VLID 2B）
//   注入模型：条目按声明序轮转（round-robin），直至各条目 count 用尽。
// 测量输出（[PERF-TB] 行，scripts/check_perf.py 消费判定）：
//   frame net=<A|B> idx sfd_cyc len vlid   （GMII 帧：len=EN 高电平字节−8 前导）
//   msg   idx vlid len first_cyc           （应用口交付信封，len 含 VLID 2B）
//   lat   vlid=<v> seq=<k> cyc=<交付首拍−注入首拍>（按 VL 内序配对）
//   cnt   qfull= ovf= txlenerr= rxlenerr= rxfcs= fragto= ctxerr= msgbad=
//   inj   total_bytes=<n>；delivered <n> msgs (exp <n>)；TB-PASS/FAIL
// 载荷模式：字节 k = (k*7 + vlid[7:0]) & 0xFF（收发同式自校验）。
//=============================================================================

`timescale 1ns/1ps

module tb_perf;

    parameter CLK_HALF = 4;                // 125 MHz
    reg clk, reset;
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    integer errors;
    initial errors = 0;

    localparam integer WD_NS = 300_000_000;   // 看门狗 300ms 仿真时间

    // tick_us / time_ns（同 tb_rxpipe 口径）
    reg tick_us;
    reg [6:0] tdiv;
    reg [63:0] time_ns;
    always @(posedge clk) begin
        if (reset) begin
            tdiv <= 7'd0; tick_us <= 1'b0; time_ns <= 64'd0;
        end else begin
            tick_us  <= 1'b0;
            time_ns  <= time_ns + 64'd8;
            tdiv     <= tdiv + 7'd1;
            if (tdiv == 7'd124) begin
                tick_us <= 1'b1;
                tdiv    <= 7'd0;
            end
        end
    end

    //-------------------------------------------------------------------------
    // DUT：AFDX_top 全链（cfg 走顶层引脚 VERDICT-015；GMII 层级只读观测）
    //-------------------------------------------------------------------------
    reg  [7:0]  tx_data;
    wire [7:0]  rx_data;
    reg         tx_valid;
    wire        rx_valid;
    wire        tx_ready;
    reg         rx_ready;
    reg  [19:0] cfg_addr;
    reg  [31:0] cfg_wdata;
    reg         cfg_we, cfg_re;
    wire [31:0] cfg_rdata;
    wire        cfg_rvalid;

    AFDX_top u_dut (
        .clk(clk), .reset(reset),
        .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .rx_data(rx_data), .rx_valid(rx_valid), .rx_ready(rx_ready),
        .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .cfg_we(cfg_we), .cfg_re(cfg_re),
        .cfg_rdata(cfg_rdata), .cfg_rvalid(cfg_rvalid)
    );

    //-------------------------------------------------------------------------
    // 回环电缆（tb 层级驱动，同 tb_afdx_rx RTL 模式；IP 集成模式不编译）
    //-------------------------------------------------------------------------
`ifndef AFDX_MAC_LOOPBACK
    always @(posedge clk) begin
        if (reset) begin
            u_dut.u_core.gmii_rx_a_d  <= 8'h0;
            u_dut.u_core.gmii_rx_a_dv <= 1'b0;
            u_dut.u_core.gmii_rx_a_er <= 1'b0;
            u_dut.u_core.gmii_rx_b_d  <= 8'h0;
            u_dut.u_core.gmii_rx_b_dv <= 1'b0;
            u_dut.u_core.gmii_rx_b_er <= 1'b0;
        end else begin
            u_dut.u_core.gmii_rx_a_d  <= u_dut.u_core.gmii_a_d;
            u_dut.u_core.gmii_rx_a_dv <= u_dut.u_core.gmii_a_en;
            u_dut.u_core.gmii_rx_a_er <= 1'b0;
            u_dut.u_core.gmii_rx_b_d  <= u_dut.u_core.gmii_b_d;
            u_dut.u_core.gmii_rx_b_dv <= u_dut.u_core.gmii_b_en;
            u_dut.u_core.gmii_rx_b_er <= 1'b0;
        end
    end
`endif

    //-------------------------------------------------------------------------
    // GMII 捕获（A/B 各一 always；帧长 = EN 高电平字节 − 8 前导）
    //-------------------------------------------------------------------------
    integer fr_a_n, fr_b_n;
    reg [63:0] fr_sfdA [0:8191];
    reg [63:0] fr_sfdB [0:8191];
    integer    fr_lenA [0:8191];
    integer    fr_lenB [0:8191];
    integer    fr_vlA  [0:8191];
    integer    fr_vlB  [0:8191];
    reg [7:0]  caA [0:2047];
    reg [7:0]  caB [0:2047];
    integer    ca_nA, ca_nB;
    reg        ca_inA, ca_inB;
    integer    ca_iA, ca_iB;
    reg [63:0] ca_sA, ca_sB;

    always @(posedge clk) begin
        if (reset) begin
            ca_inA <= 1'b0; ca_iA <= 0;
        end else if (u_dut.u_core.gmii_a_en && !ca_inA) begin
            ca_inA <= 1'b1; ca_iA <= 1; ca_sA <= $time / 8;
            caA[0] <= u_dut.u_core.gmii_a_d;       // 首字节同拍存入
        end else if (u_dut.u_core.gmii_a_en) begin
            if (ca_iA < 2048) begin
                caA[ca_iA] <= u_dut.u_core.gmii_a_d;
                ca_iA <= ca_iA + 1;
            end
        end else if (ca_inA) begin
            ca_inA <= 1'b0;
            fr_sfdA[fr_a_n] <= ca_sA;
            fr_lenA[fr_a_n] <= ca_iA - 8;
            fr_vlA[fr_a_n]  <= {caA[12], caA[13]};   // 前导 8 + dstMAC[4:5]
            fr_a_n <= fr_a_n + 1;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            ca_inB <= 1'b0; ca_iB <= 0;
        end else if (u_dut.u_core.gmii_b_en && !ca_inB) begin
            ca_inB <= 1'b1; ca_iB <= 1; ca_sB <= $time / 8;
            caB[0] <= u_dut.u_core.gmii_b_d;
        end else if (u_dut.u_core.gmii_b_en) begin
            if (ca_iB < 2048) begin
                caB[ca_iB] <= u_dut.u_core.gmii_b_d;
                ca_iB <= ca_iB + 1;
            end
        end else if (ca_inB) begin
            ca_inB <= 1'b0;
            fr_sfdB[fr_b_n] <= ca_sB;
            fr_lenB[fr_b_n] <= ca_iB - 8;
            fr_vlB[fr_b_n]  <= {caB[12], caB[13]};
            fr_b_n <= fr_b_n + 1;
        end
    end

    //-------------------------------------------------------------------------
    // 应用口交付捕获（信封自校验）
    //-------------------------------------------------------------------------
    integer msg_n;
    integer msg_lenx  [0:8191];
    integer msg_vlx   [0:8191];
    reg [63:0] msg_cx [0:8191];
    integer msg_bad;
    reg        rx_in;
    integer    rx_i;
    reg [15:0] cur_vl;
    reg [63:0] rx_c0;

    always @(posedge clk) begin
        if (reset) begin
            rx_in <= 1'b0; rx_i <= 0; msg_bad <= 0;
        end else if (rx_valid) begin
            if (!rx_in) begin
                rx_in <= 1'b1; rx_i <= 1; rx_c0 <= $time / 8;
                cur_vl[15:8] <= rx_data;
            end else begin
                rx_i <= rx_i + 1;
                if (rx_i == 1)
                    cur_vl[7:0] <= rx_data;
                if (rx_i >= 2) begin
                    if (rx_data !== (((rx_i - 2) * 7 + {8'h00, cur_vl[7:0]}) & 8'hFF)) begin
                        $display("[PERF-TB] FAIL: msg byte mismatch idx=%0d k=%0d d=%02x vl=%04x",
                                 msg_n, rx_i - 2, rx_data, cur_vl);
                        msg_bad <= msg_bad + 1;
                    end
                end
            end
        end else if (rx_in) begin
            rx_in <= 1'b0;
            msg_lenx[msg_n] <= rx_i;
            msg_vlx[msg_n]  <= cur_vl;
            msg_cx[msg_n]   <= rx_c0;
            msg_n <= msg_n + 1;
        end
    end

    //-------------------------------------------------------------------------
    // 事件计数（层级引用 core 事件线）
    //-------------------------------------------------------------------------
    integer cnt_qfull, cnt_txlen, cnt_rxovf, cnt_rxlen, cnt_rxfcs, cnt_fragto, cnt_ctx;
    integer cnt_vlunk, cnt_neterr, cnt_macerr, cnt_iperr, cnt_icerr;
    integer cnt_frame, cnt_rmdup, cnt_rmold;
    always @(posedge clk) begin
        if (!reset) begin
            if (u_dut.u_core.ev_qfull)     cnt_qfull  <= cnt_qfull  + 1;
            if (u_dut.u_core.ev_lenerr)    cnt_txlen  <= cnt_txlen  + 1;
            if (u_dut.u_core.rx_ev_ovf)    cnt_rxovf  <= cnt_rxovf  + 1;
            if (u_dut.u_core.rx_ev_len)    cnt_rxlen  <= cnt_rxlen  + 1;
            if (u_dut.u_core.rx_ev_fcs)    cnt_rxfcs  <= cnt_rxfcs  + 1;
            if (u_dut.u_core.rx_ev_fragto) cnt_fragto <= cnt_fragto + 1;
            if (u_dut.u_core.rx_ev_ctx)    cnt_ctx    <= cnt_ctx    + 1;
            if (u_dut.u_core.rx_ev_vlunk)  cnt_vlunk  <= cnt_vlunk  + 1;
            if (u_dut.u_core.rx_ev_net)    cnt_neterr <= cnt_neterr + 1;
            if (u_dut.u_core.rx_ev_mac)    cnt_macerr <= cnt_macerr + 1;
            if (u_dut.u_core.rx_ev_ip)     cnt_iperr  <= cnt_iperr  + 1;
            if (u_dut.u_core.rx_ev_ic)     cnt_icerr  <= cnt_icerr  + 1;
            if (u_dut.u_core.rx_ev_frame)  cnt_frame  <= cnt_frame  + 1;
            if (u_dut.u_core.rx_ev_rmdup)  cnt_rmdup  <= cnt_rmdup  + 1;
            if (u_dut.u_core.rx_ev_rmold)  cnt_rmold  <= cnt_rmold  + 1;
        end
    end

    //-------------------------------------------------------------------------
    // 注入器：全局 1B/pace_div 拍口径（消息内 1B/拍，消息间空档补足）
    //-------------------------------------------------------------------------
    integer pace_div;
    integer max_ms;
    integer inj_vlid  [0:255];
    integer inj_len   [0:255];
    integer inj_count [0:255];
    integer inj_done  [0:255];
    integer n_inj;
    // S 行调度模式（绝对时刻逐条注入；多 VL 异构节奏用）
    integer sch_cyc  [0:8191];
    integer sch_vlid [0:8191];
    integer sch_len  [0:8191];
    integer n_sched;
    integer inj_bytes;
    integer inj_msg_cyc [0:8191];
    integer inj_msg_vlid [0:8191];
    integer n_injmsg;

    // 调度注入：S 行按绝对时刻逐条（文件按时间序给出，顺序等待到点即发）
    task inject_sched;
        integer e, k;
        reg [7:0] b;
        begin
            inj_bytes = 0; n_injmsg = 0;
            for (e = 0; e < n_sched; e = e + 1) begin
                while ($time / 8 < sch_cyc[e])
                    @(posedge clk);
                inj_msg_cyc[n_injmsg]  = $time / 8;
                inj_msg_vlid[n_injmsg] = sch_vlid[e];
                n_injmsg = n_injmsg + 1;
                @(negedge clk);
                tx_valid <= 1'b1;
                tx_data  <= sch_vlid[e][15:8];
                inj_bytes = inj_bytes + 1;
                @(negedge clk);
                tx_data  <= sch_vlid[e][7:0];
                inj_bytes = inj_bytes + 1;
                for (k = 0; k < sch_len[e] - 2; k = k + 1) begin
                    b = ((k * 7) + sch_vlid[e][7:0]) & 8'hFF;
                    @(negedge clk);
                    tx_data <= b;
                    inj_bytes = inj_bytes + 1;
                end
                @(negedge clk);
                tx_valid <= 1'b0;
            end
        end
    endtask

    task inject_all;
        integer e, k, i, gap;
        reg [7:0] b;
        reg all_done;
        begin
            inj_bytes = 0; n_injmsg = 0;
            all_done = 1'b0;
            while (!all_done) begin
                for (i = 0; i < n_inj; i = i + 1) begin
                    if (inj_done[i] < inj_count[i]) begin
                        // 发送一条消息（tx_valid 连续覆盖整条信封）
                        inj_msg_cyc[n_injmsg]  = $time / 8;
                        inj_msg_vlid[n_injmsg] = inj_vlid[i];
                        n_injmsg = n_injmsg + 1;
                        @(negedge clk);
                        tx_valid <= 1'b1;
                        tx_data  <= inj_vlid[i][15:8];
                        inj_bytes = inj_bytes + 1;
                        @(negedge clk);
                        tx_data  <= inj_vlid[i][7:0];
                        inj_bytes = inj_bytes + 1;
                        for (k = 0; k < inj_len[i] - 2; k = k + 1) begin
                            b = ((k * 7) + inj_vlid[i][7:0]) & 8'hFF;
                            @(negedge clk);
                            tx_data <= b;
                            inj_bytes = inj_bytes + 1;
                        end
                        @(negedge clk);
                        tx_valid <= 1'b0;
                        inj_done[i] = inj_done[i] + 1;
                        // 消息间空档（全局 1B/pace_div 拍口径）
                        gap = pace_div * inj_len[i] - inj_len[i];
                        for (e = 0; e < gap; e = e + 1)
                            @(negedge clk);
                    end
                end
                // 收敛判定
                all_done = 1'b1;
                for (i = 0; i < n_inj; i = i + 1)
                    if (inj_done[i] < inj_count[i])
                        all_done = 1'b0;
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    reg [1023:0] casedir;
    integer fd, c, a1, a2, a3, a4, a5, a6, a7;
    reg [1023:0] tok;
    reg [1023:0] linebuf;
    integer rlen;
    integer i, m, wait_guard;
    integer exp_msgs;
    reg [63:0] t_end;
    integer seqv [0:255];

    initial begin : main
        tx_data = 0; tx_valid = 0; rx_ready = 1'b1;
        cfg_addr = 0; cfg_wdata = 0; cfg_we = 0; cfg_re = 0;
        errors = 0; msg_n = 0; msg_bad = 0;
        fr_a_n = 0; fr_b_n = 0;
        cnt_qfull = 0; cnt_txlen = 0; cnt_rxovf = 0; cnt_rxlen = 0;
        cnt_rxfcs = 0; cnt_fragto = 0; cnt_ctx = 0;
        cnt_vlunk = 0; cnt_neterr = 0; cnt_macerr = 0; cnt_iperr = 0; cnt_icerr = 0;
        cnt_frame = 0; cnt_rmdup = 0; cnt_rmold = 0;
        n_inj = 0; exp_msgs = 0; pace_div = 1; max_ms = 20; n_sched = 0;

        if (!$value$plusargs("CASE_DIR=%s", casedir))
            casedir = "sim/tb_out/perf_bnd04";

        reset = 1'b1;
        repeat (8) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        wait_guard = 0;
        while (u_dut.u_core.init_done !== 1'b1 && wait_guard < 100000) begin
            @(posedge clk); #1; wait_guard = wait_guard + 1;
        end
        if (u_dut.u_core.init_done !== 1'b1) begin
            $display("[PERF-TB] FAIL: init done timeout");
            errors = errors + 1;
        end

        // ---- 配置 ----
        fd = $fopen({casedir, "/perf_cfg.txt"}, "r");
        if (fd == 0) begin
            $display("[PERF-TB] FAIL: perf_cfg.txt opened");
            errors = errors + 1;
        end else begin
            // $fgets+$sscanf 逐行解析（iverilog $fscanf 部分匹配返回值不可靠）
            rlen = $fgets(linebuf, fd);
            while (rlen != 0) begin
                if ($sscanf(linebuf, "T %d %d %d %d %d %d",
                            a1, a2, a3, a4, a5, a6) >= 5) begin
                    // T idx bag_ms smax vlid mask [phase_us]（phase 缺省 0）
                    // W0 先写、W4 最后写（doc/02 §8.2-8：W0 装载会改写 cnt，
                    // 相位必须在 W0 之后装载才生效）
                    cfg_write(20'h01000 + a1 * 8 + 0,
                              32'h8000_0000 | {24'h0, a2[7:0]});
                    cfg_write(20'h01000 + a1 * 8 + 1, {16'h0, a3[15:0]});
                    cfg_write(20'h01000 + a1 * 8 + 2, {16'h0, a4[15:0]});
                    cfg_write(20'h01000 + a1 * 8 + 3, {30'h0, a5[1:0]});
                    cfg_write(20'h01000 + a1 * 8 + 5, 32'h00001389);
                    cfg_write(20'h01000 + a1 * 8 + 6, 32'h0000138A);
                    cfg_write(20'h01000 + a1 * 8 + 4, {16'h0, a6[15:0]});
                end else if ($sscanf(linebuf, "R %d %d %d %d %d %d %d",
                                     a1, a2, a3, a4, a5, a6, a7) == 7) begin
                    // R idx vlid ic rm mask smax skew
                    cfg_write(20'h10000 + a1 * 2,
                              {11'b0, a5[1:0], a4[0], a3[0], 1'b1, a2[15:0]});
                    cfg_write(20'h10000 + a1 * 2 + 1,
                              {5'b0, a6[10:0], a7[15:0]});
                end
                rlen = $fgets(linebuf, fd);
            end
            $fclose(fd);
        end
        // ---- 注入条目 ----
        fd = $fopen({casedir, "/perf_inject.txt"}, "r");
        if (fd == 0) begin
            $display("[PERF-TB] FAIL: perf_inject.txt opened");
            errors = errors + 1;
        end else begin
            c = $fscanf(fd, "%s %d %d %d\n", tok, a1, a2, a3);
            // P/M 行 1 数（c==2）、I 行 3 数（c==4）——按返回数分支
            while (c >= 2) begin
                if (tok == "P" && c == 2)
                    pace_div = a1;
                else if (tok == "M" && c == 2)
                    max_ms = a1;
                else if (tok == "I" && c == 4) begin
                    inj_vlid[n_inj] = a1; inj_len[n_inj] = a2;
                    inj_count[n_inj] = a3; inj_done[n_inj] = 0;
                    exp_msgs = exp_msgs + a3;
                    n_inj = n_inj + 1;
                    $display("[PERF-TB] inj entry vlid=%04x len=%0d cnt=%0d",
                             a1, a2, a3);
                end
                else if (tok == "S" && c == 4) begin
                    // S <start_cyc> <vlid> <len>：绝对时刻调度注入
                    sch_cyc[n_sched]  = a1;
                    sch_vlid[n_sched] = a2;
                    sch_len[n_sched]  = a3;
                    n_sched = n_sched + 1;
                    exp_msgs = exp_msgs + 1;
                end
                c = $fscanf(fd, "%s %d %d %d\n", tok, a1, a2, a3);
            end
            $fclose(fd);
        end
        cfg_write(20'h00001, 32'h00000A17);
        cfg_write(20'h00002, 32'h0A000205);
        cfg_write(20'h00003, 32'h1);           // CTRL.tx_en 开闸
        repeat (8) @(posedge clk);

        // ---- 注入 + 交付等待（固定时长：max_ms 毫秒）----
        if (n_sched > 0)
            inject_sched;
        else
            inject_all;
        t_end = $time + max_ms * 64'd1000000;   // ms→ns（$time 单位 ns）
        wait_guard = 0;
        while ($time < t_end && msg_n < exp_msgs
               && wait_guard < 100000000) begin
            @(posedge clk); #1; wait_guard = wait_guard + 1;
        end
        repeat (20000) @(posedge clk);         // 尾部排空

        // ---- 测量输出 ----
        $display("[PERF-TB] inj total_bytes=%0d", inj_bytes);
        for (i = 0; i < fr_a_n; i = i + 1)
            $display("[PERF-TB] frame net=A idx=%0d sfd_cyc=%0d len=%0d vlid=%04x",
                     i, fr_sfdA[i], fr_lenA[i], fr_vlA[i]);
        for (i = 0; i < fr_b_n; i = i + 1)
            $display("[PERF-TB] frame net=B idx=%0d sfd_cyc=%0d len=%0d vlid=%04x",
                     i, fr_sfdB[i], fr_lenB[i], fr_vlB[i]);
        for (i = 0; i < 256; i = i + 1) seqv[i] = 0;
        for (m = 0; m < msg_n; m = m + 1) begin
            $display("[PERF-TB] lat vlid=%04x seq=%0d cyc=%0d",
                     msg_vlx[m], seqv[msg_vlx[m] & 255],
                     msg_cx[m] - inj_msg_cyc[m]);
            seqv[msg_vlx[m] & 255] = seqv[msg_vlx[m] & 255] + 1;
        end
        for (m = 0; m < msg_n; m = m + 1)
            $display("[PERF-TB] msg idx=%0d vlid=%04x len=%0d first_cyc=%0d",
                     m, msg_vlx[m], msg_lenx[m], msg_cx[m]);
        $display("[PERF-TB] cnt qfull=%0d ovf=%0d txlenerr=%0d rxlenerr=%0d rxfcs=%0d fragto=%0d ctxerr=%0d msgbad=%0d vlunk=%0d neterr=%0d macerr=%0d iperr=%0d icerr=%0d",
                 cnt_qfull, cnt_rxovf, cnt_txlen, cnt_rxlen, cnt_rxfcs,
                 cnt_fragto, cnt_ctx, msg_bad,
                 cnt_vlunk, cnt_neterr, cnt_macerr, cnt_iperr, cnt_icerr);
        $display("[PERF-TB] rx frames=%0d rmdup=%0d rmold=%0d", cnt_frame, cnt_rmdup, cnt_rmold);
        $display("[PERF-TB] delivered %0d msgs (exp %0d)", msg_n, exp_msgs);

        if (errors == 0 && msg_bad == 0 && msg_n == exp_msgs)
            $display("[PERF-TB] TB-PASS");
        else
            $display("[PERF-TB] FAIL: errors=%0d msgbad=%0d delivered=%0d exp=%0d",
                     errors, msg_bad, msg_n, exp_msgs);
        $finish;
    end

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

    initial begin
        #WD_NS;
        $display("[PERF-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
