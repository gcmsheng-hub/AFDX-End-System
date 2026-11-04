//=============================================================================
// tb_rxicrm.v — IC/RM 单元穷举对拍（afdx_ref.ic_step/rm_step 全矩阵向量）
//   IC：psn(256) × seen(2) × sn(256) = 131072 向量，逐个注入状态（tb 层级
//       写 RAM）→ 判决 + 写回核对。
//   RM：rsn(256) × sn(256) × 3 组 (elapsed,skew) + 初态 256 = 196864 向量，
//       tick_us 恒 0（us_cnt=0，t_last = (2^24-elapsed) mod 2^24）。
// 向量由 scripts/gen_rx_vec.py 生成（afdx_ref 单步参考，自测已穷举自校）。
//=============================================================================

`timescale 1ns/1ps

module tb_rxicrm;

    reg clk, reset;
    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer errors;
    initial errors = 0;

    //-------------------------------------------------------------------------
    // IC 实例
    //-------------------------------------------------------------------------
    reg        ic_start;
    reg [11:0] ic_vl;
    reg        ic_net;
    reg [7:0]  ic_sn;
    wire       ic_vld;
    wire [1:0] ic_verdict;
    wire [7:0] ic_psn_old;

    rx_integrity #(.RX_VL_NUM(4096)) u_ic (
        .clk(clk), .reset(reset),
        .ri_we(1'b0), .ri_addr(13'd0),
        .ic_start(ic_start), .ic_vl(ic_vl), .ic_net(ic_net), .ic_sn(ic_sn),
        .ic_vld(ic_vld), .ic_verdict(ic_verdict), .ic_psn_old(ic_psn_old));

    //-------------------------------------------------------------------------
    // RM 实例（tick_us 恒 0 → us_cnt=0）
    //-------------------------------------------------------------------------
    reg        rm_start;
    reg [11:0] rm_vl;
    reg [7:0]  rm_sn;
    reg [15:0] rm_skew;
    wire       rm_vld, rm_accept;
    wire [7:0] rm_rsn_old;
    wire [23:0] rm_us;

    rx_redundancy #(.RX_VL_NUM(4096)) u_rm (
        .clk(clk), .reset(reset), .tick_us(1'b0),
        .ri_we(1'b0), .ri_addr(12'd0),
        .rm_start(rm_start), .rm_vl(rm_vl), .rm_sn(rm_sn), .rm_skew(rm_skew),
        .rm_vld(rm_vld), .rm_accept(rm_accept), .rm_rsn_old(rm_rsn_old),
        .rm_us(rm_us));

    //-------------------------------------------------------------------------
    // 向量回放
    //-------------------------------------------------------------------------
    integer fd, n_vec, errs_ic, errs_rm, c;
    integer v_a, v_b, v_c, v_d, v_e;
    reg [8:0]  st9;
    reg [32:0] st33;

    task chk;
        input cond;
        input [255:0] name;
        begin
            if (cond !== 1'b1) begin
                $display("[RXICRM-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    // IC 单向量：状态注入 → 判决 → 写回核对
    task do_ic;
        input integer v_psn;
        input integer v_seen;
        input integer v_sn;
        input integer v_verd;
        begin
            @(negedge clk);
            u_ic.u_ram.mem[13'd0] = {v_seen[0], v_psn[7:0]};   // 状态注入（addr={net=0,vl=0}）
            ic_vl   = 12'd0;
            ic_net  = 1'b0;
            ic_sn   = v_sn[7:0];
            ic_start= 1'b1;
            @(posedge clk); #1;                 // t1：判决有效
            chk(ic_vld === 1'b1, "ic vld");
            if (ic_verdict !== v_verd[1:0]) begin
                $display("[RXICRM-TB] FAIL ic: psn=%0d seen=%0d sn=%0d verdict=%0d exp %0d",
                         v_psn, v_seen, v_sn, ic_verdict, v_verd);
                errs_ic = errs_ic + 1;
            end
            @(negedge clk);
            ic_start = 1'b0;
            @(posedge clk); #1;                 // t2：写回完成
            st9 = u_ic.u_ram.mem[13'd0];
            if (st9 !== {1'b1, v_sn[7:0]}) begin
                $display("[RXICRM-TB] FAIL ic wb: psn=%0d seen=%0d sn=%0d mem=%b",
                         v_psn, v_seen, v_sn, st9);
                errs_ic = errs_ic + 1;
            end
        end
    endtask

    // RM 单向量：状态注入 → 判决 →（接受时）写回核对
    task do_rm;
        input integer v_rsn;
        input integer v_sn;
        input integer v_el;
        input integer v_sk;
        input integer v_verd;
        reg [23:0] t_last;
        begin
            t_last = 24'd0 - v_el[23:0];       // us_cnt=0 → elapsed=el（模 2^24）
            @(negedge clk);
            u_rm.u_ram.mem[12'd0] = {1'b1, v_rsn[7:0], t_last};
            rm_vl   = 12'd0;
            rm_sn   = v_sn[7:0];
            rm_skew = v_sk[15:0];
            rm_start= 1'b1;
            @(posedge clk); #1;                 // t1：判决有效
            chk(rm_vld === 1'b1, "rm vld");
            if (rm_accept !== v_verd[0]) begin
                $display("[RXICRM-TB] FAIL rm: rsn=%0d sn=%0d el=%0d sk=%0d acc=%b exp %0d",
                         v_rsn, v_sn, v_el, v_sk, rm_accept, v_verd);
                errs_rm = errs_rm + 1;
            end
            @(negedge clk);
            rm_start = 1'b0;
            @(posedge clk); #1;                 // t2：写回完成（仅接受）
            st33 = u_rm.u_ram.mem[12'd0];
            if (v_verd[0]) begin
                if (st33 !== {1'b1, v_sn[7:0], 24'd0}) begin
                    $display("[RXICRM-TB] FAIL rm wb: rsn=%0d sn=%0d el=%0d mem=%b",
                             v_rsn, v_sn, v_el, st33);
                    errs_rm = errs_rm + 1;
                end
            end
        end
    endtask

    // 初态（valid=0）单向量
    task do_rm_init;
        input integer v_sn;
        begin
            @(negedge clk);
            u_rm.u_ram.mem[12'd0] = 33'd0;
            rm_vl   = 12'd0;
            rm_sn   = v_sn[7:0];
            rm_skew = 16'd5000;
            rm_start= 1'b1;
            @(posedge clk); #1;
            if (rm_accept !== 1'b1) begin
                $display("[RXICRM-TB] FAIL rm init: sn=%0d acc=%b", v_sn, rm_accept);
                errs_rm = errs_rm + 1;
            end
            @(negedge clk);
            rm_start = 1'b0;
            @(posedge clk); #1;
        end
    endtask

    reg [1023:0] vecdir;

    initial begin : main
        ic_start = 0; ic_vl = 0; ic_net = 0; ic_sn = 0;
        rm_start = 0; rm_vl = 0; rm_sn = 0; rm_skew = 0;
        if (!$value$plusargs("VEC_DIR=%s", vecdir))
            vecdir = "sim/tb_out/rxvec";
        reset = 1'b1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        repeat (2) @(posedge clk);

        // ---- IC 穷举 ----
        n_vec = 0; errs_ic = 0;
        fd = $fopen({vecdir, "/ic_vec.txt"}, "r");
        chk(fd != 0, "ic_vec.txt opened");
        c = $fscanf(fd, "%d %d %d %d\n", v_a, v_b, v_c, v_d);
        while (c == 4) begin
            do_ic(v_a, v_b, v_c, v_d);
            n_vec = n_vec + 1;
            c = $fscanf(fd, "%d %d %d %d\n", v_a, v_b, v_c, v_d);
        end
        $fclose(fd);
        $display("[RXICRM-TB] ic vectors=%0d errs=%0d", n_vec, errs_ic);

        // ---- RM 穷举 ----
        n_vec = 0; errs_rm = 0;
        fd = $fopen({vecdir, "/rm_vec.txt"}, "r");
        chk(fd != 0, "rm_vec.txt opened");
        c = $fscanf(fd, "%d %d %d %d %d\n", v_a, v_b, v_c, v_d, v_e);
        while (c == 5) begin
            if (n_vec >= 196608)
                do_rm_init(v_b);                 // 末 256 行为初态向量（gen 序确定）
            else
                do_rm(v_a, v_b, v_c, v_d, v_e);
            n_vec = n_vec + 1;
            c = $fscanf(fd, "%d %d %d %d %d\n", v_a, v_b, v_c, v_d, v_e);
        end
        $fclose(fd);
        $display("[RXICRM-TB] rm vectors=%0d errs=%0d", n_vec, errs_rm);

        // ---- 显式边界向量（VERDICT-010 §1-04 澄清 + VERDICT-009 §1 SN=0 去重）----
        // (sn,rsn)=(255,0)：RSN=0 为复位同步态、在模 255 公式定义域之外，
        // 按 RTL 现行分类记 old（丢弃），与参考模型口径一致
        do_rm(0, 255, 0, 5000, 0);
        $display("[RXICRM-TB] boundary (255,0) el=0 -> old(drop) checked");
        // SN=0 去重三态：窗内副本 dup / 超窗再次复位 accept / RSN!=0 旁路 accept
        do_rm(0, 0, 0, 5000, 0);
        $display("[RXICRM-TB] boundary sn=0 rsn=0 el=0<=skew -> dup checked");
        do_rm(0, 0, 5001, 5000, 1);
        $display("[RXICRM-TB] boundary sn=0 rsn=0 el>skew -> accept checked");
        do_rm(5, 0, 0, 5000, 1);
        $display("[RXICRM-TB] boundary sn=0 rsn=5 -> bypass accept checked");

        if (errors == 0 && errs_ic == 0 && errs_rm == 0)
            $display("[RXICRM-TB] PASS");
        else
            $display("[RXICRM-TB] FAIL: errors=%0d ic=%0d rm=%0d",
                     errors, errs_ic, errs_rm);
        $finish;
    end

    initial begin
        #400_000_000;
        $display("[RXICRM-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
