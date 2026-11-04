//=============================================================================
// afdx_if.v — tb 公共任务集（`` `include`` 于 tb 模块内部使用，非独立编译单元）
// 文件式协同仿真的 Verilog 侧标准件：加载激励/期望文件、驱动应用口、
// 金标准帧头检查。Verilog-2001 兼容（eLinx vlog01compat / iverilog 双跑）。
// 约定：数据文件每行一字节 hex；lens 文件每行一个十进制整数。
//=============================================================================

    //-------------------------------------------------------------------------
    // 公共存储（tb 模块作用域）
    //-------------------------------------------------------------------------
    reg [7:0]  msg_mem  [0:262143];    // tx 激励字节流（VLID 2B + 载荷，消息连续排列）
    reg [31:0] msg_len  [0:4095];      // 每条消息字节数（ISS-15：256→4096）
    reg [7:0]  gold_mem [0:262143];    // 金标准帧字节流（单网）
    reg [31:0] gold_len [0:4095];      // 每条帧字节数（ISS-15：256→4096）
    integer    n_msgs, n_frames;
    integer    errors;                 // tb 错误计数

    //-------------------------------------------------------------------------
    // 读 "每行一字节 hex" 文件 → msg_mem，n = 总字节数
    //-------------------------------------------------------------------------
    task load_msg_file(input [1023:0] fname, output integer n);
        integer fd, c; reg [31:0] v;
        begin
            n = 0;
            fd = $fopen(fname, "r");
            if (fd == 0) begin
                $display("[TB] FAIL: cannot open %0s", fname);
                errors = errors + 1;
            end else begin
                c = $fscanf(fd, "%h", v);
                while (c == 1) begin
                    msg_mem[n] = v[7:0];
                    n = n + 1;
                    c = $fscanf(fd, "%h", v);
                end
                $fclose(fd);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 读 "每行一字节 hex" 文件 → gold_mem
    //-------------------------------------------------------------------------
    task load_gold_file(input [1023:0] fname, output integer n);
        integer fd, c; reg [31:0] v;
        begin
            n = 0;
            fd = $fopen(fname, "r");
            if (fd == 0) begin
                $display("[TB] FAIL: cannot open %0s", fname);
                errors = errors + 1;
            end else begin
                c = $fscanf(fd, "%h", v);
                while (c == 1) begin
                    gold_mem[n] = v[7:0];
                    n = n + 1;
                    c = $fscanf(fd, "%h", v);
                end
                $fclose(fd);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 读 lens 文件（每行一个十进制整数）→ msg_len[0..n-1]，返回条数
    // （V2001 task 端口不支持解包数组，故拆成 msg/gold 两个专用 task）
    //-------------------------------------------------------------------------
    task load_msg_lens(input [1023:0] fname, output integer n);
        integer fd, c, v;
        begin
            n = 0;
            fd = $fopen(fname, "r");
            if (fd == 0) begin
                $display("[TB] FAIL: cannot open %0s", fname);
                errors = errors + 1;
            end else begin
                c = $fscanf(fd, "%d", v);
                while (c == 1) begin
                    msg_len[n] = v;
                    n = n + 1;
                    c = $fscanf(fd, "%d", v);
                end
                $fclose(fd);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 读 lens 文件 → gold_len[0..n-1]，返回条数
    //-------------------------------------------------------------------------
    task load_gold_lens(input [1023:0] fname, output integer n);
        integer fd, c, v;
        begin
            n = 0;
            fd = $fopen(fname, "r");
            if (fd == 0) begin
                $display("[TB] FAIL: cannot open %0s", fname);
                errors = errors + 1;
            end else begin
                c = $fscanf(fd, "%d", v);
                while (c == 1) begin
                    gold_len[n] = v;
                    n = n + 1;
                    c = $fscanf(fd, "%d", v);
                end
                $fclose(fd);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 驱动一条应用消息：tx_valid 拉高持续 len 拍（每拍 1 字节），下降沿收尾
    // （doc/05：valid 上升沿→下降沿为一条消息；前 2 字节为 VLID）
    //-------------------------------------------------------------------------
    task drive_app_msg(input integer base, input integer len);
        integer k;
        begin
            @(posedge clk);
            tx_valid <= 1'b1;
            for (k = 0; k < len; k = k + 1) begin
                tx_data <= msg_mem[base + k];
                @(posedge clk);
            end
            tx_valid <= 1'b0;
        end
    endtask

    //-------------------------------------------------------------------------
    // 金标准帧头检查：目的MAC(常量+VLID)/源MAC(02 00 00 UID + IF_ID)/类型0x0800
    //-------------------------------------------------------------------------
    task check_frame_header(input integer base, input [15:0] vlid,
                            input [7:0] uid_hi, input [7:0] uid_lo,
                            input [7:0] if_id, input integer len);
        begin
            if (gold_mem[base + 0] !== 8'h03 || gold_mem[base + 1] !== 8'h00 ||
                gold_mem[base + 2] !== 8'h00 || gold_mem[base + 3] !== 8'h00) begin
                $display("[TB] FAIL: frame@%0d dst MAC 常量错误", base);
                errors = errors + 1;
            end
            if ({gold_mem[base + 4], gold_mem[base + 5]} !== vlid) begin
                $display("[TB] FAIL: frame@%0d dst MAC VLID=%02x%02x 期望 %04x",
                         base, gold_mem[base + 4], gold_mem[base + 5], vlid);
                errors = errors + 1;
            end
            if (gold_mem[base + 6] !== 8'h02 || gold_mem[base + 7] !== 8'h00 ||
                gold_mem[base + 8] !== 8'h00) begin
                $display("[TB] FAIL: frame@%0d src MAC 常量错误", base);
                errors = errors + 1;
            end
            if (gold_mem[base + 9] !== uid_hi || gold_mem[base + 10] !== uid_lo ||
                gold_mem[base + 11] !== if_id) begin
                $display("[TB] FAIL: frame@%0d src MAC UID/IF_ID 错误（got %02x %02x %02x）",
                         base, gold_mem[base + 9], gold_mem[base + 10], gold_mem[base + 11]);
                errors = errors + 1;
            end
            if (gold_mem[base + 12] !== 8'h08 || gold_mem[base + 13] !== 8'h00) begin
                $display("[TB] FAIL: frame@%0d Ethertype 非 0x0800", base);
                errors = errors + 1;
            end
            if (len < 64 || len > 1518) begin
                $display("[TB] FAIL: frame@%0d 长度 %0d 越界 [64,1518]", base, len);
                errors = errors + 1;
            end
        end
    endtask
