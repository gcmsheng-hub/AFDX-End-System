//=============================================================================
// tb_rxasm.v — rx_reassembly 单元对拍（afdx_ref.reassemble 场景向量）
//   8 场景：在序 2/3/6 片（TC-FRG-01/02/03）、乱序（TC-FRG-04）、缺中片
//   （abort）、缺尾片（timeout，TIMEOUT_W 参数缩至 20 tick 加速）、重叠
//   （abort）、缺首片（abort）；另有 tb 直驱的"上下文占满 f_alloc_ok=0"检查。
//   向量：gen_rx_vec.py 产出 asm_vec.txt（SCEN / F off8 len mf udplen + len 行
//   hex 字节 / E done + len 行期望 hex | abort | timeout）。
//=============================================================================

`timescale 1ns/1ps

module tb_rxasm;

    reg clk, reset;
    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer errors;
    initial errors = 0;

    // tick_us 每 10 拍一个（TIMEOUT_W=20 → 200 拍 ≈ 2µs 仿真超时）
    reg tick_us;
    reg [3:0] tick_div;
    always @(posedge clk) begin
        if (reset) begin
            tick_div <= 4'd0; tick_us <= 1'b0;
        end else begin
            tick_us  <= 1'b0;
            tick_div <= tick_div + 4'd1;
            if (tick_div == 4'd9) begin
                tick_us  <= 1'b1;
                tick_div <= 4'd0;
            end
        end
    end

    reg        f_alloc;
    reg [11:0] f_vl;
    reg [15:0] f_ipid;
    wire       f_alloc_ok;
    reg        f_start;
    reg [9:0]  f_off8;
    reg [10:0] f_len;
    reg        f_mf;
    reg [15:0] f_udplen;
    reg [7:0]  f_data;
    reg        f_valid;
    wire       f_busy, f_done, f_aborted, f_timeout;
    wire [1:0] f_done_ctx, f_abort_ctx, f_to_ctx;
    reg        r_start;
    reg [1:0]  r_ctx;
    wire [7:0] r_data;
    wire       r_valid, r_last;
    wire       r_ack = r_valid && r_last;
    wire [13:0] r_total;

    rx_reassembly #(.MAX_MSG_W(14'd8192), .TIMEOUT_W(14'd20)) u_asm (
        .clk(clk), .reset(reset), .tick_us(tick_us),
        .f_alloc(f_alloc), .f_vl(f_vl), .f_ipid(f_ipid), .f_alloc_ok(f_alloc_ok),
        .f_start(f_start), .f_off8(f_off8), .f_len(f_len), .f_mf(f_mf),
        .f_udplen(f_udplen), .f_data(f_data), .f_valid(f_valid),
        .f_busy(f_busy), .f_done(f_done), .f_done_ctx(f_done_ctx),
        .f_aborted(f_aborted), .f_abort_ctx(f_abort_ctx),
        .f_timeout(f_timeout), .f_to_ctx(f_to_ctx),
        .r_start(r_start), .r_ctx(r_ctx), .r_data(r_data),
        .r_valid(r_valid), .r_last(r_last), .r_ack(r_ack), .r_total(r_total));

    task chk;
        input cond;
        input [255:0] name;
        begin
            if (cond !== 1'b1) begin
                $display("[RXASM-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    reg [1023:0] vecdir;
    integer fd, c, n, k, nscen;
    integer gd, ga, gt;
    reg [1023:0] tok;
    integer off8, flen, fmf, ful;
    reg [7:0] expb [0:16383];
    integer expn;
    reg [255:0] expect;

    // 写入一片：f_start + len 拍字节（字节来自文件逐行）
    task send_frag;
        input integer v_off;
        input integer v_len;
        input integer v_mf;
        input integer v_ul;
        integer i;
        reg [31:0] v;
        begin
            @(negedge clk);
            f_off8  = v_off[9:0];
            f_len   = v_len[10:0];
            f_mf    = v_mf[0];
            f_udplen= v_ul[15:0];
            f_start = 1'b1;
            @(negedge clk);
            f_start = 1'b0;
            for (i = 0; i < v_len; i = i + 1) begin
                c = $fscanf(fd, "%h\n", v);
                chk(c == 1, "frag byte read");
                f_data  = v[7:0];
                f_valid = 1'b1;
                @(negedge clk);
            end
            f_valid = 1'b0;
        end
    endtask

    // 事件锁存（f_done/f_aborted/f_timeout 为单拍脉冲；多片段场景中脉冲可能
    // 落在后续片字节流送期间，wait_event 整片送完后才轮询——须锁存到读清）
    reg evt_done, evt_abort, evt_to;
    always @(posedge clk) begin
        if (reset) begin
            evt_done <= 1'b0; evt_abort <= 1'b0; evt_to <= 1'b0;
        end else begin
            if (f_done)    evt_done  <= 1'b1;
            if (f_aborted) evt_abort <= 1'b1;
            if (f_timeout) evt_to    <= 1'b1;
        end
    end

    // 等待事件（读锁存并在返回前清零，场景间不串扰）
    task wait_event;
        output integer got_done;
        output integer got_abort;
        output integer got_to;
        integer guard;
        begin
            got_done = 0; got_abort = 0; got_to = 0;
            guard = 0;
            while (guard < 5000 && !evt_done && !evt_abort && !evt_to) begin
                @(posedge clk); #1;
                guard = guard + 1;
            end
            got_done  = evt_done;
            got_abort = evt_abort;
            got_to    = evt_to;
            evt_done = 1'b0; evt_abort = 1'b0; evt_to = 1'b0;
        end
    endtask

    // 读出齐备消息并与期望比对（按 r_last 终止；事后核对 r_total）
    task readout;
        integer i, guard;
        reg last_seen;
        reg [31:0] v;
        begin
            @(negedge clk);
            r_start = 1'b1;
            r_ctx   = f_done_ctx;
            @(negedge clk);
            r_start = 1'b0;
            i = 0;
            last_seen = 1'b0;
            guard = 0;
            while (!last_seen && guard < 100000) begin
                @(posedge clk); #1;
                if (r_valid) begin
                    c = $fscanf(fd, "%h\n", v);
                    if (c != 1) begin
                        $display("[RXASM-TB] FAIL: exp byte read @%0d", i);
                        errors = errors + 1;
                    end else if (r_data !== v[7:0]) begin
                        $display("[RXASM-TB] FAIL: byte %0d = %02x exp %02x",
                                 i, r_data, v[7:0]);
                        errors = errors + 1;
                    end
                    i = i + 1;
                    if (r_last)
                        last_seen = 1'b1;
                end
                guard = guard + 1;
            end
            if (!last_seen) begin
                $display("[RXASM-TB] FAIL: readout no r_last");
                errors = errors + 1;
            end else if (i !== r_total) begin
                $display("[RXASM-TB] FAIL: readout %0d bytes != r_total %0d",
                         i, r_total);
                errors = errors + 1;
            end
        end
    endtask

    initial begin : main
        f_alloc = 0; f_vl = 0; f_ipid = 0; f_start = 0;
        f_off8 = 0; f_len = 0; f_mf = 0; f_udplen = 0; f_data = 0; f_valid = 0;
        r_start = 0; r_ctx = 0;
        if (!$value$plusargs("VEC_DIR=%s", vecdir))
            vecdir = "sim/tb_out/rxvec";
        reset = 1'b1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        repeat (2) @(posedge clk);

        fd = $fopen({vecdir, "/asm_vec.txt"}, "r");
        chk(fd != 0, "asm_vec.txt opened");
        nscen = 0;
        c = $fscanf(fd, "%s\n", tok);
        while (c == 1) begin
            if (tok == "SCEN") begin
                // 新场景
                c = $fscanf(fd, "%s\n", tok);        // 场景名
                nscen = nscen + 1;
                // 本场景第一片前占座
                @(negedge clk);
                f_vl   = 12'd1;
                f_ipid = 16'd5;
                f_alloc= 1'b1;
                @(negedge clk);
                f_alloc= 1'b0;
                chk(f_alloc_ok === 1'b1, "alloc ok");
                // 场景内：F 行（片）与 E 行（期望）
                c = $fscanf(fd, "%s\n", tok);
                while (c == 1 && tok == "F") begin
                    c = $fscanf(fd, "%d %d %d %d\n", off8, flen, fmf, ful);
                    chk(c == 4, "frag params");
                    send_frag(off8, flen, fmf, ful);
                    c = $fscanf(fd, "%s\n", tok);
                end
                // 期望（E 行 = "E" + 关键字两 token）
                chk(tok == "E", "E line marker");
                c = $fscanf(fd, "%s\n", tok);
                chk(c == 1, "E keyword");
                expect = tok;
                if (tok == "done") begin
                    wait_event(gd, ga, gt);
                    chk(gd == 1, "done event");
                    if (gd == 1)
                        readout;                 // 按 r_last 终止，事后对 r_total
                end else if (tok == "abort") begin
                    wait_event(gd, ga, gt);
                    chk(ga == 1, "abort event");
                end else begin                     // timeout
                    wait_event(gd, ga, gt);
                    chk(gt == 1, "timeout event");
                end
                $display("[RXASM-TB] scen %0d (%0s) done", nscen, tok);
                repeat (20) @(posedge clk);
                c = $fscanf(fd, "%s\n", tok);
            end else begin
                $display("[RXASM-TB] FAIL: unexpected token '%0s' after scen %0d", tok, nscen);
                errors = errors + 1;
                c = 0;
            end
        end
        $fclose(fd);

        // tb 直驱：两上下文占满 → f_alloc_ok=0
        $display("[RXASM-TB] DBG busy=%b%b done=%b%b", u_asm.busy[1],
                 u_asm.busy[0], u_asm.done[1], u_asm.done[0]);
        @(negedge clk);
        f_vl = 12'd2; f_ipid = 16'd6; f_alloc = 1'b1;
        #1;
        chk(f_alloc_ok === 1'b1, "ctx a alloc ok");   // 脉冲当拍采样
        @(negedge clk);
        f_alloc = 1'b0;
        @(negedge clk);
        f_vl = 12'd3; f_ipid = 16'd7; f_alloc = 1'b1;
        #1;
        chk(f_alloc_ok === 1'b1, "ctx b alloc ok");
        @(negedge clk);
        f_alloc = 1'b0;
        @(negedge clk);
        f_vl = 12'd4; f_ipid = 16'd8; f_alloc = 1'b1;
        #1;
        chk(f_alloc_ok === 1'b0, "ctx full: alloc not ok");
        @(negedge clk);
        f_alloc = 1'b0;
        // 清理：不释放也无妨（仿真收尾）
        repeat (10) @(posedge clk);

        if (errors == 0)
            $display("[RXASM-TB] PASS (%0d scenarios)", nscen);
        else
            $display("[RXASM-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("[RXASM-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
