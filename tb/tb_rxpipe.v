//=============================================================================
// tb_rxpipe.v — T7 接收通路流水级 tb（mac_rx×2 → parser/lookup/IC/RM/重组
//               → hrx → 应用口交付，全链对拍）
// 激励：gen_rx_cases.py 产出 rx_gmii_{A,B}.hex（B/I/W 行流，两网独立回放）；
//       rx_cfg.txt（接收 VL 表）；rx_expect.hex/lens（期望交付消息流）；
//       rx_stats.txt（期望事件计数）。
// 判定：交付消息逐条逐字节比对 + 事件计数比对 + 孤帧时延实测（首字节入→
//       交付首字节，帧起始拍打点）。
//=============================================================================

`timescale 1ns/1ps

module tb_rxpipe;

    parameter CLK_HALF = 4;                // 125 MHz
    reg clk, reset;
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    integer errors;
    initial errors = 0;

    // µs tick（125 拍）与 ns 时基
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
    // DUT：rx_path 全链
    //-------------------------------------------------------------------------
    reg [7:0]  rxd_a, rxd_b;
    reg        dv_a, dv_b, er_a, er_b;
    reg        cfg_we;                         // 接收表 cfg 写（tb 驱动）
    reg [10:0] cfg_idx;
    reg        cfg_wsel;
    reg [31:0] cfg_wdata;
    wire       ev_rxframe, ev_fcserr, ev_lenerr, ev_macerr, ev_vlunk, ev_neterr;
    wire       ev_iperr, ev_icerr, ev_rmdup, ev_rmold, ev_fragto, ev_ovf;
    wire       ev_ctxerr, ev_rxmsg;
    wire [13:0] ev_rxbytes;
    wire [7:0] rx_data;
    wire       rx_valid;

    rx_path #(.RX_VL_NUM(4096)) u_rx (
        .clk(clk), .reset(reset), .tick_us(tick_us), .time_ns(time_ns),
        .rxd_a(rxd_a), .dv_a(dv_a), .er_a(er_a),
        .rxd_b(rxd_b), .dv_b(dv_b), .er_b(er_b),
        .rxcfg_we(cfg_we), .rxcfg_idx(cfg_idx), .rxcfg_wsel(cfg_wsel),
        .rxcfg_wdata(cfg_wdata),
        .rxcfg_re(1'b0), .rxcfg_ridx(11'd0), .rxcfg_rsel(1'b0), .rxcfg_rdata(),
        .ri_we5(1'b0), .ri_addr5(13'd0),
        .ri_we6(1'b0), .ri_addr6(12'd0),
        .ri_we7(1'b0), .ri_addr7(12'd0),
        .ev_rxframe(ev_rxframe), .ev_fcserr(ev_fcserr), .ev_lenerr(ev_lenerr),
        .ev_macerr(ev_macerr), .ev_vlunk(ev_vlunk), .ev_neterr(ev_neterr),
        .ev_iperr(ev_iperr), .ev_icerr(ev_icerr), .ev_rmdup(ev_rmdup),
        .ev_rmold(ev_rmold), .ev_fragto(ev_fragto), .ev_ovf(ev_ovf),
        .ev_ctxerr(ev_ctxerr), .ev_rxmsg(ev_rxmsg), .ev_rxbytes(ev_rxbytes),
        .rx_data(rx_data), .rx_valid(rx_valid));

    //-------------------------------------------------------------------------
    // 事件计数（与 rx_stats.txt 比对）
    //-------------------------------------------------------------------------
    integer cnt_rxframe, cnt_fcs, cnt_len, cnt_mac, cnt_vlunk, cnt_net;
    integer cnt_ip, cnt_ic, cnt_rmdup, cnt_rmold, cnt_fragto, cnt_ovf;
    integer cnt_ctx, cnt_msg;
    reg [63:0] payload_bytes;
    // 交付字节/元组 FIFO 占用峰值（REV-003-05 满态可达性实测证据）
    integer peak_bf, peak_mf;

    always @(posedge clk) begin
        if (!reset) begin
            if (ev_rxframe) cnt_rxframe <= cnt_rxframe + 1;
            if (ev_fcserr)  cnt_fcs     <= cnt_fcs + 1;
            if (ev_lenerr)  cnt_len     <= cnt_len + 1;
            if (ev_macerr)  cnt_mac     <= cnt_mac + 1;
            if (ev_vlunk)   cnt_vlunk   <= cnt_vlunk + 1;
            if (ev_neterr)  cnt_net     <= cnt_net + 1;
            if (ev_iperr)   cnt_ip      <= cnt_ip + 1;
            if (ev_icerr)   cnt_ic      <= cnt_ic + 1;
            if (ev_rmdup)   cnt_rmdup   <= cnt_rmdup + 1;
            if (ev_rmold)   cnt_rmold   <= cnt_rmold + 1;
            if (ev_fragto)  cnt_fragto  <= cnt_fragto + 1;
            if (ev_ovf)     cnt_ovf     <= cnt_ovf + 1;
            if (ev_ctxerr)  cnt_ctx     <= cnt_ctx + 1;
            if (ev_rxmsg) begin
                cnt_msg       <= cnt_msg + 1;
                payload_bytes <= payload_bytes + {50'd0, ev_rxbytes};
            end
            if (u_rx.u_bf.count > peak_bf) peak_bf <= u_rx.u_bf.count;
            if (u_rx.u_mf.count > peak_mf) peak_mf <= u_rx.u_mf.count;
        end
    end

    //-------------------------------------------------------------------------
    // GMII 激励回放（两网独立进程；B/I/W 行流，读完保持空闲）
    //-------------------------------------------------------------------------
    reg        drv_go;
    reg        drvA_done, drvB_done;
    reg [63:0] first_frame_cyc;                // 孤帧首字节拍（时延基准）
    reg        first_frame_seen;
    reg [1023:0] casedir;

    // 两网驱动各自独立任务（Verilog task 局部静态——并发调用会互踩，
    // 故按网拆成两个单调用任务）
    task drv_net_a;
        reg [1023:0] fname;
        integer fd, c, n, w;
        reg [1023:0] tok;
        reg [31:0] v;
        begin
            fname = {casedir, "/rx_gmii_A.hex"};
            fd = $fopen(fname, "r");
            if (fd == 0) begin
                $display("[RXPIPE-TB] FAIL: cannot open %0s", fname);
                errors = errors + 1;
            end else begin
                c = $fscanf(fd, "%s\n", tok);
                while (c == 1) begin
                    @(posedge clk); #1;        // 每行 1 拍
                    if (tok == "B") begin
                        c = $fscanf(fd, "%h\n", v);
                        rxd_a = v[7:0]; dv_a = 1'b1;
                        if (!first_frame_seen) begin
                            first_frame_cyc = $time / 8;
                            first_frame_seen = 1'b1;
                        end
                    end else if (tok == "I") begin
                        c = $fscanf(fd, "%h\n", v);
                        dv_a = 1'b0;
                    end else if (tok == "W") begin
                        c = $fscanf(fd, "%d\n", n);
                        dv_a = 1'b0;
                        w = 0;
                        while (w < n) begin
                            @(posedge clk); #1;
                            w = w + 1;
                        end
                    end else begin
                        $display("[RXPIPE-TB] FAIL: bad gmii-A token %0s", tok);
                        errors = errors + 1;
                        c = 0;
                    end
                    if (c > 0)
                        c = $fscanf(fd, "%s\n", tok);
                end
                @(posedge clk); #1;        // 末字节采样窗保持一拍再落 dv
                dv_a = 1'b0; drvA_done = 1'b1;
                $fclose(fd);
            end
        end
    endtask

    task drv_net_b;
        reg [1023:0] fname;
        integer fd, c, n, w;
        reg [1023:0] tok;
        reg [31:0] v;
        begin
            fname = {casedir, "/rx_gmii_B.hex"};
            fd = $fopen(fname, "r");
            if (fd == 0) begin
                $display("[RXPIPE-TB] FAIL: cannot open %0s", fname);
                errors = errors + 1;
            end else begin
                c = $fscanf(fd, "%s\n", tok);
                while (c == 1) begin
                    @(posedge clk); #1;
                    if (tok == "B") begin
                        c = $fscanf(fd, "%h\n", v);
                        rxd_b = v[7:0]; dv_b = 1'b1;
                    end else if (tok == "I") begin
                        c = $fscanf(fd, "%h\n", v);
                        dv_b = 1'b0;
                    end else if (tok == "W") begin
                        c = $fscanf(fd, "%d\n", n);
                        dv_b = 1'b0;
                        w = 0;
                        while (w < n) begin
                            @(posedge clk); #1;
                            w = w + 1;
                        end
                    end else begin
                        $display("[RXPIPE-TB] FAIL: bad gmii-B token %0s", tok);
                        errors = errors + 1;
                        c = 0;
                    end
                    if (c > 0)
                        c = $fscanf(fd, "%s\n", tok);
                end
                @(posedge clk); #1;        // 末字节采样窗保持一拍再落 dv
                dv_b = 1'b0; drvB_done = 1'b1;
                $fclose(fd);
            end
        end
    endtask

    // 两网并行回放（initial 块对 + go 门控）
    initial begin
        wait (drv_go === 1'b1);
        drv_net_a;
    end
    initial begin
        wait (drv_go === 1'b1);
        drv_net_b;
    end

    //-------------------------------------------------------------------------
    // 交付捕获（信封：rx_valid 高电平段收集字节 + 首字节拍）
    //-------------------------------------------------------------------------
    reg [7:0]  cap [0:262143];
    integer    cap_n;
    integer    cap_len [0:4095];
    integer    cap_msgs;
    reg [63:0] cap_first_cyc [0:4095];
    reg        cap_in;
    integer    cap_i;

    reg dbg_on;
    reg dbg_lut;
    reg dbg_ctx;
    always @(posedge clk) begin
        if (!reset && u_rx.rm_vld)
            $display("[DBG-RM] t=%0t sn=%0d rsn=%0d acc=%b net=%b",
                     $time, u_rx.eng_sn, u_rx.rm_rsn_old, u_rx.rm_accept,
                     u_rx.eng_net_a);
        if (!reset && (u_rx.f_done || u_rx.f_aborted || u_rx.f_alloc))
            $display("[DBG-ASM] t=%0t %s alloc_ok=%b done_ctx=%0d abctx=%0d",
                     $time, u_rx.f_done ? "DONE" : (u_rx.f_aborted ? "ABRT" : "ALOC"),
                     u_rx.f_alloc_ok, u_rx.f_done_ctx, u_rx.f_abort_ctx);
    end

    always @(posedge clk) begin
        if (!reset) begin
            if (ev_rxframe) $display("[DBG-EV] t=%0t rxframe", $time);
            if (ev_fcserr)  $display("[DBG-EV] t=%0t fcserr", $time);
            if (ev_lenerr)  $display("[DBG-EV] t=%0t lenerr", $time);
            if (ev_vlunk)   $display("[DBG-EV] t=%0t vlunk", $time);
            if (ev_iperr)   $display("[DBG-EV] t=%0t iperr", $time);
            if (ev_rmdup)   $display("[DBG-EV] t=%0t rmdup", $time);
            if (ev_rmold)   $display("[DBG-EV] t=%0t rmold", $time);
            if (ev_ovf)     $display("[DBG-EV] t=%0t ovf", $time);
            if (ev_rxmsg)   $display("[DBG-EV] t=%0t rxmsg %0d", $time, ev_rxbytes);
        end
    end

    always @(posedge clk) begin
        if (!reset) begin
            if (u_rx.estate == 4'd6 && !dbg_on) begin
                dbg_on <= 1'b1;
                $display("[DBG] t=%0t E_CPY enter: plen=%0d p_lo=%0d frag=%b off=%0d mf=%b ulen=%0d",
                         $time, u_rx.eng_plen, u_rx.p_lo, u_rx.eng_frag_q,
                         u_rx.hp_off, u_rx.hp_mf, u_rx.hp_ulen);
            end
            if (dbg_on && u_rx.estate != 4'd6)
                dbg_on <= 1'b0;
            if (u_rx.estate == 4'd2 && u_rx.lk_vld && !dbg_lut) begin
                dbg_lut <= 1'b1;
                $display("[DBG-LUT] t=%0t vlid=%04x hit=%b mask=%b smax=%0d ic=%b rm=%b okmac=%b oket=%b okip=%b okudp=%b",
                         $time, u_rx.hp_vlid, u_rx.lk_hit, u_rx.lk_mask,
                         u_rx.lk_smax, u_rx.lk_ic_en, u_rx.lk_rm_en,
                         u_rx.hp_ok_mac, u_rx.hp_ok_etype, u_rx.hp_ok_ip,
                         u_rx.hp_ok_udp);
            end
            if (u_rx.estate != 4'd2)
                dbg_lut <= 1'b0;
            if (u_rx.estate == 4'd8 && !dbg_ctx) begin
                dbg_ctx <= 1'b1;
                $display("[DBG-CTX] t=%0t E_CTXW enter total=%0d", $time, u_rx.r_total);
            end
            if (u_rx.estate != 4'd8)
                dbg_ctx <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            cap_in <= 1'b0; cap_i <= 0;
        end else begin
            if (rx_valid) begin
                cap[cap_n] <= rx_data;
                cap_n      <= cap_n + 1;
                if (!cap_in) begin
                    cap_in <= 1'b1;
                    cap_i  <= 1;                     // 首字节计入
                    cap_first_cyc[cap_msgs] <= $time / 8;
                end else begin
                    cap_i <= cap_i + 1;
                end
            end
            if (!rx_valid && cap_in) begin
                cap_in  <= 1'b0;
                cap_len[cap_msgs] <= cap_i;
                cap_msgs <= cap_msgs + 1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    integer fd, c, n, k, m, n_exp, e_len, e_vlid, base, nxt;
    integer s_val, s_exp, s_errs;
    reg [31:0] v;
    reg [1023:0] tok, fname;
    reg [63:0] lat0;
    reg [1023:0] evname;
    integer msg_len_tbl [0:4095];          // 期望消息长度表（读 expect.lens 填充）
    integer ii;                            // 状态 RAM 清零循环

    task chk;
        input cond;
        input [255:0] name;
        begin
            if (cond !== 1'b1) begin
                $display("[RXPIPE-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    initial begin : main
        rxd_a = 0; dv_a = 0; er_a = 0;
        rxd_b = 0; dv_b = 0; er_b = 0;
        cfg_we = 0; cfg_idx = 0; cfg_wsel = 0; cfg_wdata = 0;
        drv_go = 0; drvA_done = 0; drvB_done = 0;
        first_frame_seen = 0; first_frame_cyc = 0;
        cap_n = 0; cap_msgs = 0; cap_in = 0; cap_i = 0;
        dbg_on = 0; dbg_lut = 0; dbg_ctx = 0;
        cnt_rxframe = 0; cnt_fcs = 0; cnt_len = 0; cnt_mac = 0; cnt_vlunk = 0;
        cnt_net = 0; cnt_ip = 0; cnt_ic = 0; cnt_rmdup = 0; cnt_rmold = 0;
        cnt_fragto = 0; cnt_ovf = 0; cnt_ctx = 0; cnt_msg = 0;
        payload_bytes = 0; peak_bf = 0; peak_mf = 0;
        if (!$value$plusargs("CASE_DIR=%s", casedir))
            casedir = "sim/tb_out/rx1";
        reset = 1'b1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        repeat (4) @(posedge clk);
        $display("[RXPIPE-TB] stage: reset released"); $fflush();
        // 本 tb 绕过 core（ram_init 不在链上）——IC/RM 状态 RAM 与接收表手工清零
        for (ii = 0; ii < 8192; ii = ii + 1)
            u_rx.u_ic.u_ram.mem[ii] = 9'h000;
        for (ii = 0; ii < 4096; ii = ii + 1)
            u_rx.u_rm.u_ram.mem[ii] = 33'h0;
        for (ii = 0; ii < 4096; ii = ii + 1) begin
            u_rx.u_lut.u_w0.mem[ii] = 32'h0;
            u_rx.u_lut.u_w1.mem[ii] = 32'h0;
        end

        // ---- 接收 VL 表配置（cfg 写广播直连端口）----
        fname = {casedir, "/rx_cfg.txt"};
        fd = $fopen(fname, "r");
        chk(fd != 0, "rx_cfg.txt opened");
        if (fd != 0) begin
            c = $fscanf(fd, "%d %d %d %d %d %d %d\n",
                        n, e_vlid, k, m, s_val, s_exp, s_errs);
            while (c == 7) begin
                // W0：{mask, rm_en, ic_en, en, tag}
                cfg_wdata = {11'b0, s_val[1:0], m[0], k[0], 1'b1, e_vlid[15:0]};
                cfg_idx   = n[10:0];
                cfg_wsel  = 1'b0;
                cfg_we    = 1'b1;
                @(posedge clk); #1;
                // W1：{smax, skew}
                cfg_wdata = {5'b0, s_exp[10:0], 16'd0} | {16'd0, s_errs[15:0]};
                cfg_wsel  = 1'b1;
                @(posedge clk); #1;
                cfg_we    = 1'b0;
                c = $fscanf(fd, "%d %d %d %d %d %d %d\n",
                            n, e_vlid, k, m, s_val, s_exp, s_errs);
            end
            $fclose(fd);
        end
        repeat (4) @(posedge clk);
        $display("[RXPIPE-TB] stage: cfg done"); $fflush();

        // ---- 激励回放 ----
        drv_go = 1'b1;
        $display("[RXPIPE-TB] stage: driving..."); $fflush();
        wait (drvA_done === 1'b1 && drvB_done === 1'b1);
        $display("[RXPIPE-TB] stage: drivers done"); $fflush();
        // 交付/重组收尾：8192B 读出 ≈65536 拍 + 重组超时回收 10ms=1.25M 拍 + 裕量
        //（rx6 ⑥b 残座须等 TIMEOUT_W 到期出 fragto 计数）
        repeat (1400000) @(posedge clk);
        $display("[RXPIPE-TB] fifo peak occupancy: bf=%0d/16384 mf=%0d/16",
                 peak_bf, peak_mf);

        // ---- 交付比对 ----
        fname = {casedir, "/rx_expect.lens"};
        fd = $fopen(fname, "r");
        chk(fd != 0, "rx_expect.lens opened");
        n_exp = 0;
        c = $fscanf(fd, "%d\n", e_len);
        while (c == 1) begin
            msg_len_tbl[n_exp] = e_len;
            n_exp = n_exp + 1;
            c = $fscanf(fd, "%d\n", e_len);
        end
        $fclose(fd);
        chk(cap_msgs == n_exp, "delivered message count");
        $display("[RXPIPE-TB] delivered %0d msgs (exp %0d), payload %0d bytes",
                 cap_msgs, n_exp, payload_bytes);
        if (cap_msgs == n_exp) begin
            base = 0; nxt = 0;
            fname = {casedir, "/rx_expect.hex"};
            fd = $fopen(fname, "r");
            for (m = 0; m < n_exp; m = m + 1) begin
                // VLID 2B + 载荷逐字节
                for (k = 0; k < msg_len_tbl[m]; k = k + 1) begin
                    c = $fscanf(fd, "%h\n", v);
                    if (c == 1 && cap[base + k] !== v[7:0]) begin
                        $display("[RXPIPE-TB] FAIL: msg %0d byte %0d = %02x exp %02x",
                                 m, k, cap[base + k], v[7:0]);
                        errors = errors + 1;
                        k = msg_len_tbl[m];    // V2001 无 break
                    end
                end
                base = base + msg_len_tbl[m];
            end
            $fclose(fd);
        end

        // ---- 事件计数比对 ----
        s_errs = 0;
        fname = {casedir, "/rx_stats.txt"};
        fd = $fopen(fname, "r");
        chk(fd != 0, "rx_stats.txt opened");
        c = $fscanf(fd, "%s %d\n", evname, s_exp);
        while (c == 2) begin
            if (evname == "rxframe")      s_val = cnt_rxframe;
            else if (evname == "fcs")     s_val = cnt_fcs;
            else if (evname == "len")     s_val = cnt_len;
            else if (evname == "mac")     s_val = cnt_mac;
            else if (evname == "vlunk")   s_val = cnt_vlunk;
            else if (evname == "net")     s_val = cnt_net;
            else if (evname == "ip")      s_val = cnt_ip;
            else if (evname == "ic")      s_val = cnt_ic;
            else if (evname == "rmdup")   s_val = cnt_rmdup;
            else if (evname == "rmold")   s_val = cnt_rmold;
            else if (evname == "fragto")  s_val = cnt_fragto;
            else if (evname == "ovf")     s_val = cnt_ovf;
            else if (evname == "ctx")     s_val = cnt_ctx;
            else if (evname == "msg")     s_val = cnt_msg;
            else begin
                $display("[RXPIPE-TB] FAIL: unknown stats key %0s", evname);
                s_val = -1; errors = errors + 1;
            end
            if (s_val !== s_exp) begin
                $display("[RXPIPE-TB] FAIL: stats %0s = %0d exp %0d",
                         evname, s_val, s_exp);
                s_errs = s_errs + 1;
            end
            c = $fscanf(fd, "%s %d\n", evname, s_exp);
        end
        $fclose(fd);
        chk(s_errs == 0, "stats counters");

        // ---- 时延实测（孤帧：首帧首字节 → 首交付首字节）----
        if (cap_msgs > 0 && first_frame_seen) begin
            lat0 = cap_first_cyc[0] - first_frame_cyc;
            $display("[RXPIPE-TB] latency first-frame-in -> first-msg-out = %0d cycles = %0d ns",
                     lat0, lat0 * 8);
            chk(lat0 < 125000, "latency < 1ms (rx-path, first frame)");
        end

        if (errors == 0)
            $display("[RXPIPE-TB] PASS");
        else
            $display("[RXPIPE-TB] FAIL: %0d errors", errors);
        $finish;
    end

    // 期望消息长度表（读 expect.lens 时填充）

    initial begin
        #2_000_000_000;                        // 2s 看门狗（rx3 含 ms 级 W）
        $display("[RXPIPE-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
