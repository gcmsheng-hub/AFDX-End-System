//=============================================================================
// tb_afdx_rx.v — T7 端到端回环 tb：AFDX_top 发送 → GMII 抓帧回灌接收 → 应用口
//
// 链路：tb 驱动 tx_data/tx_valid（信封）→ AFDX_top 发送链 → GMII 观测口
//       （u_dut.u_core.gmii_a_d/en）→ tb 回灌 u_dut.u_core.gmii_rx_a_d/dv
//       （延迟 1 拍的回环电缆）→ 接收链（mac_rx×2→parser→lookup→IC→RM→
//       重组→hrx）→ rx_data/rx_valid 应用口交付。
// 判定：交付按 VL 分组与期望序列比对（SN=0 复位标记帧按 VERDICT-009 §1 增补
//       口径：RM 旁路保留 + SkewMax 窗内副本去重——双网副本只交付一次，
//       RPT-FIX3 报备）；端到端时延实测
//       （消息首 tx_data 拍 → 交付首 rx_data 拍；单帧消息 < 2ms 官方口径，
//       跨 BAG 分片消息按 doc/02 §6 不受此约束）。
// 激励：tb 内置定值（3 VL：单帧边界 + 2/3 片分片；同 VL 消息间隔 ≥BAG，
//       排队时延归 TC-PERF 类）。
//=============================================================================

`timescale 1ns/1ps

module tb_afdx_rx;

    parameter CLK_HALF = 4;                // 125 MHz
    reg clk, reset;
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    integer errors;
    initial errors = 0;

    task chk;
        input cond;
        input [255:0] name;
        begin
            if (cond !== 1'b1) begin
                $display("[E2E-RX-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // DUT：AFDX_top（cfg 层级引用；GMII 抓帧 + 回灌）
    //-------------------------------------------------------------------------
    reg         tx_valid;
    reg  [7:0]  tx_data;
    wire [7:0]  rx_data;
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
        .tx_data(tx_data), .rx_data(rx_data),
        .tx_valid(tx_valid), .rx_valid(rx_valid),
        .tx_ready(tx_ready), .rx_ready(rx_ready),
        .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .cfg_we(cfg_we), .cfg_re(cfg_re),
        .cfg_rdata(cfg_rdata), .cfg_rvalid(cfg_rvalid)
    );

    wire [7:0] gmii_a_d  = u_dut.u_core.gmii_a_d;
    wire       gmii_a_en = u_dut.u_core.gmii_a_en;
    wire [7:0] gmii_b_d  = u_dut.u_core.gmii_b_d;
    wire       gmii_b_en = u_dut.u_core.gmii_b_en;

    reg loopback_on;                   // 二分诊断：0=断开回环
    reg skip_rx_cfg;                   // 二分诊断：1=跳过接收表配置
    // 回环电缆：GMII 观测 → 接收侧（延迟 1 拍）。
    // `ifdef AFDX_MAC_LOOPBACK`（T8 IP 集成模式）：回环在 mac_ip_top 片内、
    // core 的 gmii_rx_* 由核内驱动，本 tb 不得重复驱动（避免多驱动 X 冲突）。
`ifndef AFDX_MAC_LOOPBACK
    always @(posedge clk) begin
        if (reset) begin
            u_dut.u_core.gmii_rx_a_d  <= 8'h0;
            u_dut.u_core.gmii_rx_a_dv <= 1'b0;
            u_dut.u_core.gmii_rx_a_er <= 1'b0;
            u_dut.u_core.gmii_rx_b_d  <= 8'h0;
            u_dut.u_core.gmii_rx_b_dv <= 1'b0;
            u_dut.u_core.gmii_rx_b_er <= 1'b0;
        end else if (loopback_on) begin
            u_dut.u_core.gmii_rx_a_d  <= gmii_a_d;
            u_dut.u_core.gmii_rx_a_dv <= gmii_a_en;
            u_dut.u_core.gmii_rx_a_er <= 1'b0;
            u_dut.u_core.gmii_rx_b_d  <= gmii_b_d;
            u_dut.u_core.gmii_rx_b_dv <= gmii_b_en;
            u_dut.u_core.gmii_rx_b_er <= 1'b0;
        end
    end
`endif

    //-------------------------------------------------------------------------
    // cfg 写（顶层 cfg_* 引脚，VERDICT-015 交付口径）
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

    integer guard;

    // 发送 VL 表 + 接收 VL 表 + 全局（3 VL：idx 10/11/12 = 0x000A/0B/0C）
    task do_config;
        integer i;
        reg [15:0] vlid_q;
        begin
            for (i = 0; i < 3; i = i + 1) begin
                vlid_q = 16'h000A + i;
                // 发送表：W4 相位/W1 smax/W2 vlid/W3 mask/W5 sport/W6 dport/W0 en+bag
                cfg_write(20'h01000 + (10 + i) * 8 + 4, 32'h0);
                cfg_write(20'h01000 + (10 + i) * 8 + 1, 32'h000005EE);   // smax=1518
                cfg_write(20'h01000 + (10 + i) * 8 + 2, 32'h0000000A + i);
                cfg_write(20'h01000 + (10 + i) * 8 + 3, 32'h3);
                cfg_write(20'h01000 + (10 + i) * 8 + 5, 32'h00001389);   // 5001
                cfg_write(20'h01000 + (10 + i) * 8 + 6, 32'h0000138A);   // 5002
                cfg_write(20'h01000 + (10 + i) * 8 + 0,
                          (i == 2) ? 32'h8000_0002 : 32'h8000_0001);     // BAG 1/1/2ms
                if (!skip_rx_cfg) begin
                    // 接收表 W0：{mask=3, rm=1, ic=1, en=1, tag}
                    cfg_write(20'h10000 + (10 + i) * 2,
                              {11'b0, 2'b11, 1'b1, 1'b1, 1'b1, vlid_q});
                    // 接收表 W1：{smax=1518, skew=5000µs}
                    cfg_write(20'h10000 + (10 + i) * 2 + 1,
                              {5'b0, 11'd1518, 16'd5000});
                end
            end
            cfg_write(20'h00001, 32'h00000A17);  // UID
            cfg_write(20'h00002, 32'h0A000205);  // SRC_IP
        end
    endtask

    //-------------------------------------------------------------------------
    // 消息注入（信封；载荷 = (k*7 + vl*3 + 1) & 0xFF 确定性模式）
    //-------------------------------------------------------------------------
    integer   capA_frames;
    reg       capA_in;
    integer   msg_vl [0:63];               // 每条消息的 VL 组号（注入序）
    integer   msg_len8 [0:63];
    integer   msg_frag [0:63];             // 片数（1=单帧；时延约束豁免用）
    reg [63:0] tx_first_cyc [0:63];
    integer   n_msgs;
    integer   frames_before;

    task inject_msg;
        input integer vl;                  // 0/1/2
        input integer plen;
        input integer nfrag;
        integer k;
        begin
            frames_before = capA_frames;
            @(negedge clk);
            tx_first_cyc[n_msgs] = $time / 8;
            msg_vl[n_msgs] = vl;
            msg_len8[n_msgs] = plen;
            msg_frag[n_msgs] = nfrag;
            tx_valid <= 1'b1;
            tx_data  <= 8'h00;                       // VLID 高（P0 采样）
            @(posedge clk);
            tx_data  <= 8'h0A + vl;                  // VLID 低（P1 采样）
            @(posedge clk);                          // P1：补拍，防 VLID 低被覆盖
            for (k = 0; k < plen; k = k + 1) begin
                tx_data <= (k * 7 + vl * 3 + 1) & 8'hFF;
                @(posedge clk);
            end
            tx_valid <= 1'b0;
            n_msgs = n_msgs + 1;
            // 同 VL 间隔 ≥BAG：等本消息首帧发出（末帧等待交给下一条）
            guard = 0;
            while (capA_frames < frames_before + 1 && guard < 2600000) begin
                @(posedge clk); #1;
                guard = guard + 1;
            end
            chk(guard < 2600000, "frame out within BAG wait");
            repeat (200) @(posedge clk);
        end
    endtask

    // 调试心跳：tx 侧关键状态（临时；每 100k 拍）
    integer dbg_heart;
    always @(posedge clk) begin
        if (!reset) begin
            dbg_heart = dbg_heart + 1;
            if (dbg_heart >= 17840 && dbg_heart <= 18060)
                $display("[DBG-CYC] h=%0d t=%0t valid=%b data=%02x eidx=%0d pv=%b peop=%b skwe=%b skfull=%b htv=%b htrdy=%b qcnt=%0d",
                         dbg_heart, $time, u_dut.u_core.tx_valid,
                         u_dut.u_core.tx_data, u_dut.u_core.env_idx,
                         u_dut.u_core.p_valid, u_dut.u_core.p_eop,
                         u_dut.u_core.sk_we, u_dut.u_core.sk_full,
                         u_dut.u_core.htx_valid_w, u_dut.u_core.htx_ready_w,
                         u_dut.u_core.u_bm.q_cnt[10]);
            if (dbg_heart % 10000 == 0)
                $display("[DBG-TX] t=%0t frames=%0d qcnt10=%0d free=%0d walk=%b enqact=%b dreq=%b dvld=%b tstart=%b bldbusy=%b",
                         $time, capA_frames, u_dut.u_core.u_bm.q_cnt[10],
                         u_dut.u_core.u_bm.free_cnt, u_dut.u_core.u_bm.walk_busy,
                         u_dut.u_core.u_bm.enq_active, u_dut.u_core.bld_desc_req,
                         u_dut.u_core.bld_desc_vld, u_dut.u_core.task_start,
                         u_dut.u_core.bld_busy);
        end
    end
    //-------------------------------------------------------------------------
    // GMII A 帧计数（注入同步用）
    //-------------------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            capA_frames <= 0; capA_in <= 1'b0;
        end else begin
            if (gmii_a_en && !capA_in) capA_in <= 1'b1;
            if (!gmii_a_en && capA_in) begin
                capA_in <= 1'b0;
                capA_frames <= capA_frames + 1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 交付捕获（信封；按 VL 分组）
    //-------------------------------------------------------------------------
    reg [7:0] cap [0:65535];
    integer   cap_n, cap_msgs, cap_i;
    reg       cap_in;
    integer   cap_len [0:255];
    integer   cap_vl [0:255];              // 按交付序的 VL 组号
    reg [63:0] cap_first_cyc [0:255];
    integer   cap_frag [0:255];            // 对应注入消息的片数
    reg [63:0] lat_min, lat_max, lat_sum;
    integer    lat_n;

    always @(posedge clk) begin
        if (reset) begin
            cap_in <= 1'b0; cap_i <= 0;
        end else begin
            if (rx_valid) begin
                cap[cap_n] <= rx_data;
                cap_n      <= cap_n + 1;
                if (!cap_in) begin
                    cap_in <= 1'b1;
                    cap_i  <= 1;                   // 首字节计入
                    cap_first_cyc[cap_msgs] <= $time / 8;
                end else begin
                    cap_i <= cap_i + 1;
                end
            end
            if (!rx_valid && cap_in) begin
                cap_in  <= 1'b0;
                cap_len[cap_msgs] <= cap_i;
                cap_vl[cap_msgs] <= (cap[cap_n - cap_i + 1] - 8'h0A);  // VLID 低字节 → 组号
                cap_msgs <= cap_msgs + 1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    integer i, k, m, base;
    // 期望（按 VL 分组的交付载荷长度序列）：首消息（SN=0）单份
    // （FIX3 去重：RM 旁路保留 + SkewMax 窗内副本去重，VERDICT-009 §1）
    integer exp_seq [0:2][0:15];
    integer exp_n [0:2];
    integer exp_idx [0:2];
    integer exp_frag [0:2][0:15];
    integer expect_total;
    reg [63:0] e2e_lat;
    reg [63:0] lat0_cyc;
    reg [63:0] tx_cyc_of;
    integer    seq_i;

    initial begin : main
        tx_valid = 0; tx_data = 0; rx_ready = 1'b1;
        cfg_addr = 0; cfg_wdata = 0; cfg_we = 0; cfg_re = 0;
        cap_n = 0; cap_msgs = 0; cap_i = 0; cap_in = 0;
        capA_frames = 0; n_msgs = 0; dbg_heart = 0;
        loopback_on = 1'b1;                // 回环电缆
        skip_rx_cfg = 1'b0;                // 接收表配置
        lat_min = 64'hFFFFFFFFFFFFFFFF; lat_max = 0; lat_sum = 0; lat_n = 0;
        // VL A（0x000A）：100B(SN0 单份) 1B 1471B 2000B(2 片)
        exp_n[0] = 4;
        exp_seq[0][0] = 100; exp_frag[0][0] = 1;
        exp_seq[0][1] = 1;   exp_frag[0][1] = 1;
        exp_seq[0][2] = 1471; exp_frag[0][2] = 1;
        exp_seq[0][3] = 2000; exp_frag[0][3] = 2;
        // VL B（0x000B）：17B(SN0 单份) 3000B(3 片)
        exp_n[1] = 2;
        exp_seq[1][0] = 17; exp_frag[1][0] = 1;
        exp_seq[1][1] = 3000; exp_frag[1][1] = 3;
        // VL C（0x000C）：500B（SN=0 单份）
        exp_n[2] = 1;
        exp_seq[2][0] = 500; exp_frag[2][0] = 1;
        exp_idx[0] = 0; exp_idx[1] = 0; exp_idx[2] = 0;
        expect_total = 7;

        reset = 1'b1;
        repeat (8) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        guard = 0;
        while (u_dut.u_core.init_done !== 1'b1 && guard < 100000) begin
            @(posedge clk); #1; guard = guard + 1;
        end
        chk(u_dut.u_core.init_done === 1'b1, "init done");
        do_config;
        repeat (4) @(posedge clk);
        cfg_write(20'h00003, 32'h1);         // CTRL.tx_en 开闸
        repeat (4) @(posedge clk);

        // 注入（同 VL 消息间隔 ≥BAG；跨 VL 交错模拟并发）
        lat0_cyc = $time / 8;
        inject_msg(0, 100, 1);               // VL A：100B（SN=0，双份交付）
        inject_msg(1, 17, 1);                // VL B：17B（SN=0，双份交付）
        inject_msg(0, 1, 1);                 // VL A：1B
        inject_msg(2, 500, 1);               // VL C：500B
        inject_msg(0, 1471, 1);              // VL A：1471B 单帧
        inject_msg(0, 2000, 2);              // VL A：2000B（2 片，BAG 间隔）
        inject_msg(1, 3000, 3);              // VL B：3000B（3 片）

        // 等全部交付（7 信封 + 重组读出）
        guard = 0;
        while (cap_msgs < expect_total && guard < 12000000) begin
            @(posedge clk); #1; guard = guard + 1;
        end
        chk(cap_msgs == expect_total, "delivered envelope count");
        repeat (50000) @(posedge clk);

        $display("[E2E-RX-TB] delivered %0d envelopes, %0d bytes", cap_msgs, cap_n);
        for (m = 0; m < cap_msgs; m = m + 1)
            $display("[E2E-RX-TB]   seq %0d: VL%0d len %0d", m, cap_vl[m], cap_len[m]);

        // ---- 逐消息比对：VL 分组序 + 长度 + VLID + 载荷模式 ----
        base = 0;
        for (m = 0; m < cap_msgs; m = m + 1) begin
            i = cap_vl[m];
            seq_i = 0;
            for (k = 0; k < m; k = k + 1)
                if (cap_vl[k] == i) seq_i = seq_i + 1;
            if (i < 0 || i > 2 || seq_i >= exp_n[i]) begin
                $display("[E2E-RX-TB] FAIL: msg %0d VL 组 %0d 序 %0d 超出期望",
                         m, i, seq_i);
                errors = errors + 1;
            end else if (cap_len[m] !== exp_seq[i][seq_i] + 2) begin
                $display("[E2E-RX-TB] FAIL: msg %0d (VL%0d 序%0d) len %0d exp %0d",
                         m, i, seq_i, cap_len[m], exp_seq[i][seq_i] + 2);
                errors = errors + 1;
            end else begin
                if (cap[base] !== 8'h00) begin
                    $display("[E2E-RX-TB] FAIL: msg %0d VLID hi %02x", m, cap[base]);
                    errors = errors + 1;
                end
                if (cap[base + 1] !== 8'h0A + i) begin
                    $display("[E2E-RX-TB] FAIL: msg %0d VLID lo %02x exp %02x",
                             m, cap[base + 1], 8'h0A + i);
                    errors = errors + 1;
                end
                for (k = 2; k < cap_len[m]; k = k + 1) begin
                    if (cap[base + k] !== (((k - 2) * 7 + i * 3 + 1) & 8'hFF)) begin
                        $display("[E2E-RX-TB] FAIL: msg %0d byte %0d = %02x exp %02x",
                                 m, k, cap[base + k],
                                 (((k - 2) * 7 + i * 3 + 1) & 8'hFF));
                        errors = errors + 1;
                        k = cap_len[m];      // V2001 无 break
                    end
                end
            end
            base = base + cap_len[m];
        end
        chk(base == cap_n, "byte accounting");

        // ---- 端到端时延实测（交付 ↔ 注入对齐：VL 内序号 → 注入条目）----
        for (m = 0; m < cap_msgs; m = m + 1) begin
            i = cap_vl[m];
            seq_i = 0;
            for (k = 0; k < m; k = k + 1)
                if (cap_vl[k] == i) seq_i = seq_i + 1;
            // VL 内交付序 → 注入条目（A：0,2,4,5；B：1,6；C：3——FIX3 去重后
            // SN=0 消息单份交付，交付序与注入序一一对应）
            case (i)
                0: tx_cyc_of = tx_first_cyc[(seq_i == 0) ? 0 :
                                            (seq_i == 1) ? 2 :
                                            (seq_i == 2) ? 4 : 5];
                1: tx_cyc_of = tx_first_cyc[(seq_i == 0) ? 1 : 6];
                default: tx_cyc_of = tx_first_cyc[3];   // VL C = 注入 3
            endcase
            e2e_lat = cap_first_cyc[m] - tx_cyc_of;
            lat_sum = lat_sum + e2e_lat;
            lat_n = lat_n + 1;
            if (e2e_lat < lat_min) lat_min = e2e_lat;
            if (e2e_lat > lat_max) lat_max = e2e_lat;
            // 单帧消息硬线 <2ms；跨 BAG 分片消息按 doc/02 §6 豁免（仅告警）
            if (exp_frag[i][seq_i] == 1 && e2e_lat >= 250000) begin
                $display("[E2E-RX-TB] FAIL: msg %0d 单帧时延 %0d >= 2ms", m, e2e_lat);
                errors = errors + 1;
            end else if (e2e_lat >= 250000) begin
                $display("[E2E-RX-TB] INFO: msg %0d 跨 BAG 分片总时延 %0d 拍（豁免）",
                         m, e2e_lat);
            end
        end
        if (lat_n > 0)
            $display("[E2E-RX-TB] latency: min=%0d max=%0d avg=%0d cycles (n=%0d)",
                     lat_min, lat_max, lat_sum / lat_n, lat_n);
        $display("[E2E-RX-TB] e2e first-byte latency: %0d cycles",
                 cap_first_cyc[0] - lat0_cyc);

        if (errors == 0)
            $display("[E2E-RX-TB] PASS");
        else
            $display("[E2E-RX-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #120_000_000;                      // 120ms 看门狗（仿真 ≈15ms 足够）
        $display("[E2E-RX-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
