//=============================================================================
// tb_rxparser.v — rx_parser 单元对拍（afdx_ref/gen_rx_vec 解析向量）
//   12 帧：正常单帧/最小帧/首片/中片/尾片 + etype/proto/csum/ver/ulen/
//   dst MAC 六类异常变体 + 跨 VLID 帧。逐帧喂 42B 头部流，比对 4 个 ok
//   标志与 8 个提取字段（向量期望由 gen_rx_vec.manual_exp 独立重算）。
//=============================================================================

`timescale 1ns/1ps

module tb_rxparser;

    reg clk, reset;
    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer errors;
    initial errors = 0;

    reg        hp_start, hp_valid;
    reg [7:0]  hp_data;
    wire       hp_done;
    wire       hp_ok_mac, hp_ok_etype, hp_ok_ip, hp_ok_udp;
    wire [15:0] hp_vlid, hp_ipid, hp_sport, hp_dport, hp_ulen, hp_iptot;
    wire [31:0] hp_sip;
    wire        hp_mf;
    wire [12:0] hp_off;

    rx_parser u_par (
        .clk(clk), .reset(reset),
        .hp_start(hp_start), .hp_valid(hp_valid), .hp_data(hp_data),
        .hp_done(hp_done),
        .hp_ok_mac(hp_ok_mac), .hp_ok_etype(hp_ok_etype),
        .hp_ok_ip(hp_ok_ip), .hp_ok_udp(hp_ok_udp),
        .hp_vlid(hp_vlid), .hp_sip(hp_sip), .hp_ipid(hp_ipid),
        .hp_iptot(hp_iptot), .hp_mf(hp_mf), .hp_off(hp_off),
        .hp_sport(hp_sport), .hp_dport(hp_dport), .hp_ulen(hp_ulen));

    task chk;
        input cond;
        input [255:0] name;
        begin
            if (cond !== 1'b1) begin
                $display("[RXP-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    reg [1023:0] vecdir;
    reg [7:0]  hdr [0:41];
    integer fd, c, k, nf, nerrs;
    integer e_mac, e_et, e_ip, e_udp, e_vlid, e_iptot, e_ipid;
    integer e_mf, e_off, e_sport, e_dport, e_ulen;
    reg [1023:0] fname;

    task do_frame;
        input integer idx;
        integer i;
        begin
            // 读 42B 头部
            fname = {vecdir, "/", "f", "  "};
            // （iverilog 无 sprintf；逐帧文件名直接拼）
            case (idx)
                0: fname = {vecdir, "/f0.hdr"};
                1: fname = {vecdir, "/f1.hdr"};
                2: fname = {vecdir, "/f2.hdr"};
                3: fname = {vecdir, "/f3.hdr"};
                4: fname = {vecdir, "/f4.hdr"};
                5: fname = {vecdir, "/f5.hdr"};
                6: fname = {vecdir, "/f6.hdr"};
                7: fname = {vecdir, "/f7.hdr"};
                8: fname = {vecdir, "/f8.hdr"};
                9: fname = {vecdir, "/f9.hdr"};
                10: fname = {vecdir, "/f10.hdr"};
                default: fname = {vecdir, "/f11.hdr"};
            endcase
            fd = $fopen(fname, "r");
            chk(fd != 0, "hdr file opened");
            for (i = 0; i < 42; i = i + 1) begin
                c = $fscanf(fd, "%h\n", hdr[i]);
                chk(c == 1, "hdr byte read");
            end
            $fclose(fd);

            // 喂流：start 单拍 + 42 拍顺序字节
            @(negedge clk);
            hp_start = 1'b1; hp_valid = 1'b0;
            @(negedge clk);
            hp_start = 1'b0;
            for (i = 0; i < 42; i = i + 1) begin
                hp_valid = 1'b1;
                hp_data  = hdr[i];
                @(negedge clk);
            end
            hp_valid = 1'b0;
            #1;                                 // hp_done 已在本周期置位（字节 41
                                                // 处理沿之后），当拍采样
            chk(hp_done === 1'b1, "hp_done pulse");

            // 读期望
            case (idx)
                0: fname = {vecdir, "/f0.exp"};
                1: fname = {vecdir, "/f1.exp"};
                2: fname = {vecdir, "/f2.exp"};
                3: fname = {vecdir, "/f3.exp"};
                4: fname = {vecdir, "/f4.exp"};
                5: fname = {vecdir, "/f5.exp"};
                6: fname = {vecdir, "/f6.exp"};
                7: fname = {vecdir, "/f7.exp"};
                8: fname = {vecdir, "/f8.exp"};
                9: fname = {vecdir, "/f9.exp"};
                10: fname = {vecdir, "/f10.exp"};
                default: fname = {vecdir, "/f11.exp"};
            endcase
            fd = $fopen(fname, "r");
            chk(fd != 0, "exp file opened");
            c = $fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
                        e_mac, e_et, e_ip, e_udp, e_vlid, e_iptot, e_ipid,
                        e_mf, e_off, e_sport, e_dport, e_ulen);
            chk(c == 12, "exp fields read");
            $fclose(fd);

            nerrs = 0;
            if (hp_ok_mac   !== e_mac[0])  begin $display("[RXP-TB] f%0d ok_mac %b exp %0d", idx, hp_ok_mac, e_mac); nerrs = nerrs + 1; end
            if (hp_ok_etype !== e_et[0])   begin $display("[RXP-TB] f%0d ok_etype %b exp %0d", idx, hp_ok_etype, e_et); nerrs = nerrs + 1; end
            if (hp_ok_ip    !== e_ip[0])   begin $display("[RXP-TB] f%0d ok_ip %b exp %0d", idx, hp_ok_ip, e_ip); nerrs = nerrs + 1; end
            if (hp_ok_udp   !== e_udp[0])  begin $display("[RXP-TB] f%0d ok_udp %b exp %0d", idx, hp_ok_udp, e_udp); nerrs = nerrs + 1; end
            if (hp_vlid     !== e_vlid[15:0]) begin $display("[RXP-TB] f%0d vlid %04x exp %04x", idx, hp_vlid, e_vlid); nerrs = nerrs + 1; end
            if (hp_iptot    !== e_iptot[15:0]) begin $display("[RXP-TB] f%0d iptot %04x exp %04x", idx, hp_iptot, e_iptot); nerrs = nerrs + 1; end
            if (hp_ipid     !== e_ipid[15:0]) begin $display("[RXP-TB] f%0d ipid %04x exp %04x", idx, hp_ipid, e_ipid); nerrs = nerrs + 1; end
            if (hp_mf       !== e_mf[0])   begin $display("[RXP-TB] f%0d mf %b exp %0d", idx, hp_mf, e_mf); nerrs = nerrs + 1; end
            if (hp_off      !== e_off[12:0]) begin $display("[RXP-TB] f%0d off %0d exp %0d", idx, hp_off, e_off); nerrs = nerrs + 1; end
            if (hp_sport    !== e_sport[15:0]) begin $display("[RXP-TB] f%0d sport %04x exp %04x", idx, hp_sport, e_sport); nerrs = nerrs + 1; end
            if (hp_dport    !== e_dport[15:0]) begin $display("[RXP-TB] f%0d dport %04x exp %04x", idx, hp_dport, e_dport); nerrs = nerrs + 1; end
            if (hp_ulen     !== e_ulen[15:0]) begin $display("[RXP-TB] f%0d ulen %04x exp %04x", idx, hp_ulen, e_ulen); nerrs = nerrs + 1; end
            errors = errors + nerrs;
            $display("[RXP-TB] f%0d %s (%0d field errs)", idx,
                     (nerrs == 0) ? "ok" : "MISMATCH", nerrs);
        end
    endtask

    initial begin : main
        hp_start = 0; hp_valid = 0; hp_data = 0;
        if (!$value$plusargs("VEC_DIR=%s", vecdir))
            vecdir = "sim/tb_out/rxvec";
        reset = 1'b1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        repeat (2) @(posedge clk);

        nf = 12;
        for (k = 0; k < nf; k = k + 1)
            do_frame(k);

        if (errors == 0)
            $display("[RXP-TB] PASS");
        else
            $display("[RXP-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #10_000_000;
        $display("[RXP-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
