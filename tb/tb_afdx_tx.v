//=============================================================================
// tb_afdx_tx.v — T6-006 端到端 tb：AFDX_top 应用口信封驱动（冻结顶层语义）
// 链路：tb 驱动应用口（tx_data/tx_valid 信封，VLID 2B 前缀）→ AFDX_top →
//       afdx_es_core（信封解析→host_arb→buf_mgr→regulator/scheduler→
//       sn_gen+tx_frame_builder→s_stream→mac_tx×2 锁步→mac_port_if）→
//       GMII 观测（tb 层级引用 u_dut.u_core.gmii_* 抓帧）。
// cfg：顶层 cfg_* 引脚驱动（VERDICT-015 交付口径；层级引用赋值后门已清除）。
// 判定：tb 自检（帧数/前导/SFD）+ 落盘 frame_out_{A,B}.hex/lens 与
//       frame_times_{A,B}.txt → scripts/check_output.py 与 afdx_ref 金标准
//       逐字节比对 + FCS/IP 校验和独立核算 + BAG/0.5ms 时序检查。
// 激励与配置来自 scripts/gen_stim.py tx1（msg_tx.hex / msg_lens.txt /
//       vl_cfg.txt / golden_{A,B}.hex / frame_{A,B}.lens / case.json）。
// 运行模式（FIX2 回归；模式来自 case 目录 tb_mode.txt，gen_stim 产出，
//           "+TX_EARLY" / "+SN_WRAP" 参数可覆盖）：
//   （缺省）先注入后开闸——tx1 原口径；
//   TX_EARLY   tx_en 开启状态下交错注入（REV-002-02 回归，tx2 用例）；
//   SN_WRAP    逐帧注入长序列（SN 255→1 回绕）+ 结束后复位首帧重发检查（txsn）；
//   LENERR_EXP=<n>   期望 stats g_lenerr 计数（Smax 拒收用例，txsmax）；
//   所有模式收尾断言 slot 池 free_cnt==32（资源回收）。
// GMII 抓帧仅采样 TX_EN 有效拍（REV-002-01：en 无效拍不构成有效传输）。
//=============================================================================

`timescale 1ns/1ps

module tb_afdx_tx;

    parameter CLK_HALF = 4;
    // errors 由 `include "afdx_if.v" 声明（公共任务集作用域）

    reg clk, reset;
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    //-------------------------------------------------------------------------
    // 模块间连线（先声明后使用；afdx_if 在声明后 include）
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    wire [6:0]  bld_rd_idx;
    wire [2:0]  bld_rd_wsel;
    wire [31:0] bld_rd_data;
    wire        bld_desc_req, bld_desc_vld;
    wire [6:0]  bld_desc_vl;
    wire [10:0] fr_body_len;
    wire [13:0] fr_paylen;
    wire [1:0]  fr_mask;
    wire        frame_ready, bld_busy;
    wire        task_start, task_done, st_frame;
    wire [6:0]  task_vl, st_vl;
    wire [13:0] st_paylen;
    wire        macA_start, macB_start, macA_done, macB_done;
    wire        macA_sready, macB_sready;
    wire        s_valid, s_sop, s_eop;
    wire [7:0]  s_data;
    wire        sn_re_w, sn_vld_w;
    wire [6:0]  sn_vl_w;
    wire [7:0]  sn_val_w;

    // 应用口（AFDX_top 模板端口，drive_app_msg 直接驱动）
    reg         tx_valid;
    reg  [7:0]  tx_data;
    wire [7:0]  rx_data;
    wire        rx_valid;
    wire        tx_ready;
    reg         rx_ready;
    // cfg 主机配置接口（VERDICT-015 交付口径：顶层引脚驱动，层级引用后门已清除）
    reg  [19:0] cfg_addr;
    reg  [31:0] cfg_wdata;
    reg         cfg_we, cfg_re;
    wire [31:0] cfg_rdata;
    wire        cfg_rvalid;
    integer     tx_n, gold_n, ml_n, fl_n;   // 加载任务输出（afdx_if 约定）
    integer     i0;
    reg [1023:0] casedir;
    reg [2047:0] path;

    `include "afdx_if.v"

    task chk;
        input cond;
        input [511:0] name;
        begin
            if (cond !== 1'b1) begin
                $display("[E2E-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask


    //-------------------------------------------------------------------------
    // DUT：AFDX_top（应用口直连；cfg/GMII 层级引用）
    //-------------------------------------------------------------------------
    reg        clk_r_unused;
    // 应用口（模板端口，正常连接）
    //   tx_data/tx_valid/tx_ready/rx_data/rx_valid/rx_ready 在下方实例化连接

    // cfg 总线主机侧（顶层 cfg_* 引脚，VERDICT-015；只读观测仍走层级引用）
    //   访问路径：本 tb 的 cfg_addr / cfg_wdata / cfg_we / cfg_re 寄存器→引脚
    //   读回：    cfg_rdata / cfg_rvalid（AFDX_top 出引脚）
    // GMII 观测：u_dut.u_core.gmii_a_d / gmii_a_en / gmii_b_d / gmii_b_en
    // init 完成：u_dut.u_core.init_done

    AFDX_top u_dut (
        .clk      (clk),
        .reset    (reset),
        .tx_data  (tx_data),
        .rx_data  (rx_data),
        .tx_valid (tx_valid),
        .rx_valid (rx_valid),
        .tx_ready (tx_ready),
        .rx_ready (rx_ready),
        .cfg_addr (cfg_addr),
        .cfg_wdata(cfg_wdata),
        .cfg_we   (cfg_we),
        .cfg_re   (cfg_re),
        .cfg_rdata(cfg_rdata),
        .cfg_rvalid(cfg_rvalid)
    );

    // 应用口宿主寄存器（afdx_if 的 drive_app_msg 使用）
    // （tx_valid/tx_data 已在 afdx_if 声明区之前的宿主信号段声明）

    // cfg 读（顶层 cfg 引脚驱动，VERDICT-015；读 cfg_rdata/rvalid）
    task cfg_read;
        input [19:0] a;
        output [31:0] rd;
        begin
            @(negedge clk);
            cfg_addr = a;
            cfg_re   = 1'b1;
            @(posedge clk); #1;
            rd = cfg_rdata;
            @(negedge clk);
            cfg_re   = 1'b0;
        end
    endtask

    // 抓帧观测线（层级引用别名，cap_sample 直用）
    wire [7:0] gmii_a_d  = u_dut.u_core.gmii_a_d;
    wire       gmii_a_en = u_dut.u_core.gmii_a_en;
    wire [7:0] gmii_b_d  = u_dut.u_core.gmii_b_d;
    wire       gmii_b_en = u_dut.u_core.gmii_b_en;

    //-------------------------------------------------------------------------
    // GMII 抓帧（A/B 各自缓存；校验前导/SFD；记录起始周期）
    //-------------------------------------------------------------------------
    reg [7:0]  capA [0:65535];
    reg [7:0]  capB [0:65535];
    integer    capA_n, capB_n;
    integer    capA_frames, capB_frames;
    integer    capA_len [0:511];
    integer    capB_len [0:511];
    integer    capA_base, capB_base;    // 帧起始时的累计字节数（帧长=差值）
    reg [31:0] capA_cyc [0:511];
    reg [31:0] capB_cyc [0:511];
    integer    capA_idx, capB_idx;      // 帧内字节序号（含前导 8B）
    reg        capA_in, capB_in;
    reg [31:0] cyc_cnt;

    // 单网采样（g=0 A / 1 B）——仅在 TX_EN 有效拍采样帧字节（REV-002-01：
    // en 无效拍不构成 GMII 有效传输，不得收入帧缓冲）
    task cap_sample;
        input       g;
        input [7:0] d;
        input       en;
        begin
            if (g == 0) begin
                if (en && !capA_in) begin
                    capA_in  = 1;
                    capA_idx = 0;
                    capA_cyc[capA_frames] = cyc_cnt;
                end
                if (capA_in && en) begin
                    if (capA_idx < 8) begin
                        if (capA_idx < 7) begin
                            if (d !== 8'h55) begin
                                $display("[E2E-TB] FAIL: A 前导@%0d = %02x",
                                         capA_idx, d);
                                errors = errors + 1;
                            end
                        end else if (d !== 8'hD5) begin
                            $display("[E2E-TB] FAIL: A SFD = %02x", d);
                            errors = errors + 1;
                        end
                    end else begin
                        capA[capA_n] = d;
                        capA_n = capA_n + 1;
                    end
                    capA_idx = capA_idx + 1;
                end
                if (!en && capA_in) begin
                    capA_in = 0;
                    capA_len[capA_frames] = capA_n - capA_base;
                    capA_base = capA_n;
                    capA_frames = capA_frames + 1;
                end
            end else begin
                if (en && !capB_in) begin
                    capB_in  = 1;
                    capB_idx = 0;
                    capB_cyc[capB_frames] = cyc_cnt;
                end
                if (capB_in && en) begin
                    if (capB_idx < 8) begin
                        if (capB_idx < 7) begin
                            if (d !== 8'h55) begin
                                $display("[E2E-TB] FAIL: B 前导@%0d = %02x",
                                         capB_idx, d);
                                errors = errors + 1;
                            end
                        end else if (d !== 8'hD5) begin
                            $display("[E2E-TB] FAIL: B SFD = %02x", d);
                            errors = errors + 1;
                        end
                    end else begin
                        capB[capB_n] = d;
                        capB_n = capB_n + 1;
                    end
                    capB_idx = capB_idx + 1;
                end
                if (!en && capB_in) begin
                    capB_in = 0;
                    capB_len[capB_frames] = capB_n - capB_base;
                    capB_base = capB_n;
                    capB_frames = capB_frames + 1;
                end
            end
        end
    endtask

    always @(posedge clk) begin
        if (reset) begin
            cyc_cnt <= 0;
            capA_in <= 0; capB_in <= 0;
        end else begin
            cyc_cnt <= cyc_cnt + 1;
            cap_sample(0, gmii_a_d, gmii_a_en);
            cap_sample(1, gmii_b_d, gmii_b_en);
        end
    end

    //-------------------------------------------------------------------------
    // 配置/激励任务
    //-------------------------------------------------------------------------
    integer guard, m, n, vl_idx;
    integer first_idx;                        // vl_cfg 首行表索引（回读校验用）
    reg [31:0] first_vlid;
    reg [31:0] w_en, w_bag, w_smax, w_vlid, w_mask, w_phase, w_sport, w_dport;
    reg [31:0] rd_lenerr;                 // stats g_lenerr 回读（LENERR_EXP 断言）
    integer cfg_fd;
    reg mode_early;      // +TX_EARLY：tx_en 开启状态下交错注入（FIX2/REV-002-02）
    reg mode_sn;         // +SN_WRAP：逐帧注入 + 结束后复位首帧重发检查（FIX2 回归）
    integer lenerr_exp;  // +LENERR_EXP=<n>：期望 ev_lenerr 计数（Smax 拒收用例）

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

    // vl_cfg.txt → VL 表（复位后重配亦复用；列：idx en bag_ms smax vlid mask phase sport dport）
    task load_vl_cfg;
        begin
            path = {casedir, "/vl_cfg.txt"};
            cfg_fd = $fopen(path, "r");
            chk(cfg_fd != 0, "vl_cfg.txt opened");
            first_idx = -1; first_vlid = 0;
            if (cfg_fd != 0) begin
                while ($fscanf(cfg_fd, "%d %d %d %d %d %d %d %d %d",
                               vl_idx, w_en, w_bag, w_smax, w_vlid, w_mask,
                               w_phase, w_sport, w_dport) == 9) begin
                    if (first_idx < 0) begin
                        first_idx = vl_idx; first_vlid = w_vlid;
                    end
                    // W4(相位) 先写，W0(en/bag) 最后写——W0 装载"配置即到期"
                    cfg_write(20'h01000 + (vl_idx << 3) + 4, {16'h0, w_phase[15:0]});
                    cfg_write(20'h01000 + (vl_idx << 3) + 1, {16'h0, w_smax[15:0]});
                    cfg_write(20'h01000 + (vl_idx << 3) + 2, {16'h0, w_vlid[15:0]});
                    cfg_write(20'h01000 + (vl_idx << 3) + 3, {30'h0, w_mask[1:0]});
                    cfg_write(20'h01000 + (vl_idx << 3) + 5, {16'h0, w_sport[15:0]});
                    cfg_write(20'h01000 + (vl_idx << 3) + 6, {16'h0, w_dport[15:0]});
                    cfg_write(20'h01000 + (vl_idx << 3) + 0,
                              (w_en != 0 ? 32'h8000_0000 : 32'h0) | {24'h0, w_bag[7:0]});
                end
                $fclose(cfg_fd);
            end
        end
    endtask

    // 复位后重配（SN_WRAP 复位首帧检查用）：init 等待 + VL 表 + UID/SRC_IP
    task reinit_and_cfg;
        begin
            guard = 0;
            while (u_dut.u_core.init_done !== 1'b1 && guard < 50000) begin
                @(posedge clk); #1; guard = guard + 1;
            end
            chk(u_dut.u_core.init_done === 1'b1, "reinit: init done");
            load_vl_cfg;
            cfg_write(20'h00001, 32'h00000A17);      // UID（与 gen_stim 一致）
            cfg_write(20'h00002, 32'h0A000205);      // SRC_IP = 10.0.2.5
            cfg_write(20'h00003, 32'h1);             // CTRL.tx_en（复位清零须重开）
        end
    endtask


    //-------------------------------------------------------------------------
    // 落盘（hex 每行一字节；lens 每行一整数；times 每行一起始周期）
    //-------------------------------------------------------------------------
    task dump_net;
        input g;
        integer fd, i;
        reg [1023:0] f_hex, f_len, f_tim;
        begin
            if (g == 0) begin
                f_hex = {casedir, "/frame_out_A.hex"};
                f_len = {casedir, "/frame_out_A.lens"};
                f_tim = {casedir, "/frame_times_A.txt"};
                fd = $fopen(f_hex, "w");
                for (i = 0; i < capA_n; i = i + 1)
                    $fdisplay(fd, "%h", capA[i]);
                $fclose(fd);
                fd = $fopen(f_len, "w");
                for (i = 0; i < capA_frames; i = i + 1)
                    $fdisplay(fd, "%0d", capA_len[i]);
                $fclose(fd);
                fd = $fopen(f_tim, "w");
                for (i = 0; i < capA_frames; i = i + 1)
                    $fdisplay(fd, "%0d", capA_cyc[i]);
                $fclose(fd);
            end else begin
                f_hex = {casedir, "/frame_out_B.hex"};
                f_len = {casedir, "/frame_out_B.lens"};
                f_tim = {casedir, "/frame_times_B.txt"};
                fd = $fopen(f_hex, "w");
                for (i = 0; i < capB_n; i = i + 1)
                    $fdisplay(fd, "%h", capB[i]);
                $fclose(fd);
                fd = $fopen(f_len, "w");
                for (i = 0; i < capB_frames; i = i + 1)
                    $fdisplay(fd, "%0d", capB_len[i]);
                $fclose(fd);
                fd = $fopen(f_tim, "w");
                for (i = 0; i < capB_frames; i = i + 1)
                    $fdisplay(fd, "%0d", capB_cyc[i]);
                $fclose(fd);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    integer exp_frames;
    integer done_wait;

    initial begin : main
        errors = 0; reset = 1'b1;
        tx_valid = 0; tx_data = 0; rx_ready = 1'b1;
        cfg_addr = 0; cfg_wdata = 0; cfg_we = 0; cfg_re = 0;
        capA_n = 0; capB_n = 0; capA_frames = 0; capB_frames = 0;
        capA_idx = 0; capB_idx = 0; capA_in = 0; capB_in = 0;
        capA_base = 0; capB_base = 0;
        if (!$value$plusargs("CASE_DIR=%s", casedir))
            casedir = "sim/tb_out/tx1";
        // 运行模式来自 case 目录 tb_mode.txt（gen_stim 产出；"+" 参数仍可覆盖）
        //   行1：TX_EARLY / SN_WRAP / "-"；行2：LENERR_EXP 期望值 / "-"
        mode_early = $test$plusargs("TX_EARLY");
        mode_sn    = $test$plusargs("SN_WRAP");
        lenerr_exp = -1;
        begin : rd_mode
            integer mfd, mc;
            reg [1023:0] mstr;
            mfd = $fopen({casedir, "/tb_mode.txt"}, "r");
            if (mfd != 0) begin
                mc = $fscanf(mfd, "%s\n", mstr);
                if (mc == 1 && mstr == "TX_EARLY") mode_early = 1'b1;
                if (mc == 1 && mstr == "SN_WRAP")  mode_sn    = 1'b1;
                mc = $fscanf(mfd, "%d\n", lenerr_exp);
                if (mc != 1)
                    lenerr_exp = -1;
                $fclose(mfd);
            end
            $display("[E2E-TB] mode: early=%b sn=%b lenerr_exp=%0d",
                     mode_early, mode_sn, lenerr_exp);
        end

        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        guard = 0;
        while (u_dut.u_core.init_done !== 1'b1 && guard < 50000) begin
            @(posedge clk); #1; guard = guard + 1;
        end
        chk(u_dut.u_core.init_done === 1'b1, "init done");

        // 1) 加载激励/金标准
        path = {casedir, "/msg_tx.hex"};
        load_msg_file(path, tx_n);
        path = {casedir, "/msg_lens.txt"};
        load_msg_lens(path, ml_n);
        path = {casedir, "/golden_A.hex"};
        load_gold_file(path, gold_n);
        path = {casedir, "/frame_A.lens"};
        load_gold_lens(path, fl_n);
        exp_frames = fl_n;
        $display("[E2E-TB] loaded: msgs=%0d frames=%0d tx_bytes=%0d",
                 ml_n, exp_frames, tx_n);
        chk(ml_n > 0 && fl_n > 0, "case files non-empty");

        // 2) vl_cfg.txt → VL 表（列：idx en bag_ms smax vlid mask phase sport dport）
        load_vl_cfg;
        cfg_write(20'h00001, 32'h00000A17);      // UID（与 gen_stim 一致）
        cfg_write(20'h00002, 32'h0A000205);      // SRC_IP = 10.0.2.5
        // CTRL 暂不开：先入队后开闸（TX_EARLY 模式相反，见下）
        begin : cfg_verify
            reg [31:0] rd;
            cfg_read(20'h00001, rd);
            if (rd !== 32'h00000A17) begin
                $display("[E2E-TB] FAIL: cfg readback UID=%08x", rd);
                errors = errors + 1;
            end
            cfg_read(20'h01000 + (first_idx << 3) + 2, rd);
            if (rd !== first_vlid) begin
                $display("[E2E-TB] FAIL: cfg readback VL%0d.W2=%08x exp %08x",
                         first_idx, rd, first_vlid);
                errors = errors + 1;
            end
            $display("[E2E-TB] DBG cfg ok: uid/w2 readback match (VL%0d); reg en=%b reload=%0d",
                     first_idx, u_dut.u_core.u_reg.en_r[first_idx],
                     u_dut.u_core.u_reg.reload_r[first_idx]);
        end

        // 2b) TX_EARLY：发送开启状态下交错注入（FIX2/REV-002-02 回归口径）；
        //     SN_WRAP：逐帧注入需发送同步，同样先开闸
        if (mode_early || mode_sn)
            cfg_write(20'h00003, 32'h1);

        // 3) 全部消息经应用口注入（信封：VLID 2B + 载荷，drive_app_msg）
        //    SN_WRAP 模式逐帧注入：等上一条消息的帧发出再注入下一条（单 VL
        //    1 帧/消息，队列不积压——260 条 > 队列深度 4，必须与发送同步）
        n = 0;
        m = 0;
        while (m < ml_n) begin
            drive_app_msg(n, msg_len[m]);
            n = n + msg_len[m];
            m = m + 1;
            if (mode_sn && m < ml_n) begin
                guard = 0;
                while (capA_frames < m && guard < 400_000) begin
                    @(posedge clk);
                    guard = guard + 1;
                end
                chk(capA_frames >= m, "sn: frame out before next inject");
            end
        end
        chk(m == ml_n, "all messages injected");
        $display("[E2E-TB] injected %0d messages", m);

        // 4) 开闸（默认模式：先入队后开闸）+ 等两网帧数均达期望
        if (!mode_early && !mode_sn) begin
            repeat (50) @(posedge clk);
            cfg_write(20'h00003, 32'h1);
        end
        guard = 0;
        done_wait = exp_frames;   // 每网帧数（非两网合计）
        while ((capA_frames < done_wait || capB_frames < done_wait)
               && guard < 500_000_000) begin
            @(posedge clk);
            guard = guard + 1;
        end
        chk(capA_frames >= done_wait && capB_frames >= done_wait,
            "all frames captured on both nets");
        $display("[E2E-TB] captured A=%0d B=%0d (exp %0d/net)",
                 capA_frames, capB_frames, exp_frames);
        repeat (2000) @(posedge clk);            // 末帧释放/统计收尾

        // 4b) 资源回收断言：slot 池全部释放（FIX2 各用例通用口径）
        chk(u_dut.u_core.u_bm.free_cnt === 6'd32, "slot pool fully released (32)");
        if (lenerr_exp >= 0) begin
            cfg_read(20'h203C4, rd_lenerr);       // stats 全局 g_lenerr（0x203C4）
            chk(rd_lenerr === lenerr_exp,
                "lenerr count matches rejected messages");
        end

        // 5) 落盘（case 目录）
        dump_net(0);
        dump_net(1);
        $display("[E2E-TB] outputs written to case dir");

        // 6) SN_WRAP：复位首帧重发检查（FIX2 回归——复位后首帧 SN=0）
        if (mode_sn) begin
            @(negedge clk);
            reset = 1'b1;
            repeat (8) @(posedge clk);
            @(negedge clk);
            reset = 1'b0;
            // 捕获状态清零（此后抓到的首帧即复位后首帧）
            capA_n = 0; capB_n = 0; capA_frames = 0; capB_frames = 0;
            capA_idx = 0; capB_idx = 0; capA_in = 0; capB_in = 0;
            capA_base = 0; capB_base = 0;
            reinit_and_cfg;
            drive_app_msg(0, msg_len[0]);        // 重发与主流程第 1 条相同的消息
            guard = 0;
            while ((capA_frames < 1 || capB_frames < 1) && guard < 400_000) begin
                @(posedge clk);
                guard = guard + 1;
            end
            chk(capA_frames >= 1 && capB_frames >= 1, "sn reset: frame captured");
            repeat (2000) @(posedge clk);
            // A 网复位帧与 golden 帧 0 逐字节比对（SN=0、IP ID=1、同载荷）
            if (capA_len[0] !== gold_len[0]) begin
                $display("[E2E-TB] FAIL: sn reset frame len %0d exp %0d",
                         capA_len[0], gold_len[0]);
                errors = errors + 1;
            end else begin
                for (i0 = 0; i0 < gold_len[0]; i0 = i0 + 1) begin
                    if (capA[i0] !== gold_mem[i0]) begin
                        $display("[E2E-TB] FAIL: sn reset frame byte %0d = %02x exp %02x",
                                 i0, capA[i0], gold_mem[i0]);
                        errors = errors + 1;
                        i0 = gold_len[0];        // V2001 无 break
                    end
                end
            end
            chk(capB_len[0] === capA_len[0], "sn reset: B frame length matches");
            chk(u_dut.u_core.u_bm.free_cnt === 6'd32,
                "sn reset: slot pool fully released");
            $display("[E2E-TB] sn reset-check done (frame len %0d, SN=0 re-sent)",
                     capA_len[0]);
        end

        if (errors == 0)
            $display("[E2E-TB] PASS");
        else
            $display("[E2E-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #500_000_000;                            // 500ms 仿真时间（SN_WRAP 长跑 ~270ms）
        $display("[E2E-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
