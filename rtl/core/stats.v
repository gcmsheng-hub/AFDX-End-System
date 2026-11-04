//=============================================================================
// stats.v — 全局/每 VL 计数器与发送抖动记录（v2 §4.2 core；doc/02 §6 测量点）
//   - 每 VL RAM×3：tx_frames / tx_payload_bytes / tx_jitter_max（32b，RMW）
//   - t_elig[VL]（24b µs）：eligible 上升沿打点（≈ max(BAG 到期, 入队) 时刻）
//   - 帧完成：jit = now_us − t_elig[vl]，RMW 更新三 RAM + 全局计数
//   - 全局计数：g_tx_frames / g_tx_bytes / g_msgs / g_qfull / g_lenerr
// cfg 读（stats_re 单拍，1 拍后 stats_rdata 有效）：
//   addr[9:6]!=0xF：{vl=addr[9:3], sel=addr[2:1]}，sel 0=帧 1=字节 2=抖动 3=0
//     （字节地址 = 0x20000 + vl*8 + sel*2）
//   addr[9:6]==0xF：全局，addr[5:0]：
//     0x00 g_frames 0x01 g_bytes 0x02 g_msgs 0x03 g_qfull 0x04 g_lenerr（发送）
//     0x05 g_rx_frames 0x06 g_rx_fcserr 0x07 g_rx_lenerr 0x08 g_rx_macerr
//     0x09 g_rx_vlunk 0x0A g_rx_neterr 0x0B g_rx_iperr 0x0C g_ic_err
//     0x0D g_rm_dup 0x0E g_rm_old 0x0F g_frag_to
//     0x10 g_rx_msgs 0x11 g_rx_payload 0x12 g_rx_ovf 0x13 g_rx_ctxerr（T7）
//     （字节地址 = 0x203C0 + n）
// RAM 读口 cfg 优先；RMW 读在 st_frame 当拍切地址（下一拍捕获，与 cfg 回读不冲突）。
// 上电清零：ram_init ri_sel=2/3/4。us_cnt 24b µs（16.7s 回绕，测试时长内无影响）。
//=============================================================================

module stats #(
    parameter integer TX_VL_NUM = 128
)(
    input  wire        clk,
    input  wire        reset,
    input  wire        tick_us,          // 1 µs 单拍

    input  wire [TX_VL_NUM-1:0] elig,    // 合成 eligible（检上升沿记 t_elig）
    input  wire        st_frame,         // 单拍：一帧发出
    input  wire [6:0]  st_vl,
    input  wire [13:0] st_paylen,
    input  wire        ev_enq,           // 单拍：消息入队（buf_mgr）
    input  wire        ev_qfull,
    input  wire        ev_lenerr,

    // 接收侧事件（T7；单拍脉冲束，rx_path 汇聚）
    input  wire        ev_rxframe,       // MAC 收到一好帧
    input  wire        ev_rxfcs,         // FCS 错
    input  wire        ev_rxlen,         // runt/超长/smax/pad
    input  wire        ev_rxmac,         // 目的 MAC 常量错
    input  wire        ev_rxvlunk,       // VL 查表未命中
    input  wire        ev_rxnet,         // net_mask 不含本网
    input  wire        ev_rxip,          // etype/ver/ihl/proto/csum/ulen
    input  wire        ev_rxic,          // IC 超窗丢弃
    input  wire        ev_rmdup,         // RM 重复副本
    input  wire        ev_rmold,         // RM 旧帧
    input  wire        ev_fragto,        // 重组超时
    input  wire        ev_rxovf,         // 帧缓冲满丢弃
    input  wire        ev_rxctx,         // 重组上下文异常
    input  wire        ev_rxmsg,         // 应用口交付一条消息
    input  wire [13:0] ev_rxbytes,       // 该消息载荷字节数（ev_rxmsg 拍有效）

    // cfg 读（1 拍后 stats_rdata 有效，与 cfg_bus 契约对齐）
    input  wire        stats_re,
    input  wire [9:0]  stats_raddr,
    output wire [31:0] stats_rdata,

    // 上电清零（ram_init）
    input  wire        ri_we,
    input  wire [2:0]  ri_sel,           // 2=帧 3=字节 4=抖动
    input  wire [15:0] ri_addr
);

    reg  [23:0] us_cnt;
    always @(posedge clk) begin
        if (reset)        us_cnt <= 24'd0;
        else if (tick_us) us_cnt <= us_cnt + 24'd1;
    end

    //-------------------------------------------------------------------------
    // t_elig 打点（eligible 上升沿）
    //-------------------------------------------------------------------------
    reg [23:0]    t_elig [0:TX_VL_NUM-1];
    reg [TX_VL_NUM-1:0] elig_d;
    wire [TX_VL_NUM-1:0] elig_rise = elig & ~elig_d;

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            elig_d <= {TX_VL_NUM{1'b0}};
            for (i = 0; i < TX_VL_NUM; i = i + 1)
                t_elig[i] <= 24'd0;
        end else begin
            elig_d <= elig;
            for (i = 0; i < TX_VL_NUM; i = i + 1)
                if (elig_rise[i])
                    t_elig[i] <= us_cnt;
        end
    end

    //-------------------------------------------------------------------------
    // 帧完成锁存 + RMW（读 1 拍 + 组合写回）
    //-------------------------------------------------------------------------
    wire [31:0] ram_f_rdata, ram_b_rdata, ram_j_rdata;

    localparam [1:0] S_IDLE = 2'd0, S_RD = 2'd1, S_WR = 2'd2;
    reg [1:0]  rmw_state;
    reg        pend;                        // 待统计帧（stats_re 撞车时延后）
    reg [6:0]  pend_vl;
    reg [13:0] pend_paylen;
    reg [31:0] f_r, b_r, j_r;

    wire cfg_rd = stats_re && (stats_raddr[9:6] != 4'hF);

    always @(posedge clk) begin
        if (reset) begin
            rmw_state <= S_IDLE; pend <= 1'b0;
            pend_vl <= 7'd0; pend_paylen <= 14'd0;
            f_r <= 32'd0; b_r <= 32'd0; j_r <= 32'd0;
        end else begin
            if (st_frame) begin              // 锁存（同拍 stats_re 则顺延）
                pend        <= 1'b1;
                pend_vl     <= st_vl;
                pend_paylen <= st_paylen;
            end
            case (rmw_state)
                S_IDLE:
                    if (pend && !stats_re) begin
                        rmw_state <= S_RD;   // 读地址本拍已切（见 rd_addr）
                        pend      <= 1'b0;
                    end
                S_RD: begin                  // rdata = mem[pend_vl]（上拍地址）
                    f_r <= ram_f_rdata;
                    b_r <= ram_b_rdata;
                    j_r <= ram_j_rdata;
                    rmw_state <= S_WR;
                end
                S_WR:                        // 写口组合有效（见 do_wr）
                    rmw_state <= S_IDLE;
                default: rmw_state <= S_IDLE;
            endcase
        end
    end

    // RMW 读地址恒 pend_vl：启动拍（IDLE 且 pend 且无 cfg 读）RAM 捕获，S_RD 拍取数。
    // 同一时刻只有一条消息在途（组帧器串行），3 拍 RMW 窗口内不会有新 st_frame 改写
    // pend_vl——写入地址稳定性由此保证。
    wire [6:0] rd_addr = pend_vl;

    // 写口（组合）：RMW 写 / 上电清零（sel 2/3/4 → 帧/字节/抖动）
    wire       do_wr = (rmw_state == S_WR);
    wire       ini_f = ri_we && (ri_sel == 3'd2);
    wire       ini_b = ri_we && (ri_sel == 3'd3);
    wire       ini_j = ri_we && (ri_sel == 3'd4);
    wire [6:0] wa    = (ini_f || ini_b || ini_j) ? ri_addr[6:0] : pend_vl;
    wire [23:0] jit  = us_cnt - t_elig[pend_vl];
    wire [31:0] wd_f = f_r + 32'd1;
    wire [31:0] wd_b = b_r + {18'b0, pend_paylen};
    wire [31:0] wd_j = (j_r < {8'b0, jit}) ? {8'b0, jit} : j_r;

    // cfg 读地址取 addr[9:3] = VL 索引（布局：{vl[6:0], sel[1:1], rsvd[0]}）
    sdp_ram #(.WIDTH(32), .DEPTH(TX_VL_NUM), .AW(7)) u_ram_f (
        .clk(clk), .we(do_wr || ini_f),
        .waddr(wa), .wdata(ini_f ? 32'h0 : wd_f),
        .raddr(cfg_rd ? stats_raddr[9:3] : rd_addr), .rdata(ram_f_rdata));
    sdp_ram #(.WIDTH(32), .DEPTH(TX_VL_NUM), .AW(7)) u_ram_b (
        .clk(clk), .we(do_wr || ini_b),
        .waddr(wa), .wdata(ini_b ? 32'h0 : wd_b),
        .raddr(cfg_rd ? stats_raddr[9:3] : rd_addr), .rdata(ram_b_rdata));
    sdp_ram #(.WIDTH(32), .DEPTH(TX_VL_NUM), .AW(7)) u_ram_j (
        .clk(clk), .we(do_wr || ini_j),
        .waddr(wa), .wdata(ini_j ? 32'h0 : wd_j),
        .raddr(cfg_rd ? stats_raddr[9:3] : rd_addr), .rdata(ram_j_rdata));

    //-------------------------------------------------------------------------
    // 全局计数（发送 0x00~0x04；接收 0x05~0x13，T7）
    //-------------------------------------------------------------------------
    reg [31:0] g_frames, g_bytes, g_msgs, g_qfull, g_lenerr;
    reg [31:0] g_rx_frames, g_rx_fcs, g_rx_len, g_rx_mac, g_rx_vlunk;
    reg [31:0] g_rx_net, g_rx_ip, g_ic_err, g_rm_dup, g_rm_old, g_frag_to;
    reg [31:0] g_rx_msgs, g_rx_pay, g_rx_ovf, g_rx_ctx;
    always @(posedge clk) begin
        if (reset) begin
            g_frames <= 0; g_bytes <= 0; g_msgs <= 0; g_qfull <= 0; g_lenerr <= 0;
            g_rx_frames <= 0; g_rx_fcs <= 0; g_rx_len <= 0; g_rx_mac <= 0;
            g_rx_vlunk <= 0; g_rx_net <= 0; g_rx_ip <= 0; g_ic_err <= 0;
            g_rm_dup <= 0; g_rm_old <= 0; g_frag_to <= 0;
            g_rx_msgs <= 0; g_rx_pay <= 0; g_rx_ovf <= 0; g_rx_ctx <= 0;
        end else begin
            if (st_frame) begin
                g_frames <= g_frames + 32'd1;
                g_bytes  <= g_bytes + {18'b0, st_paylen};
            end
            if (ev_enq)    g_msgs   <= g_msgs   + 32'd1;
            if (ev_qfull)  g_qfull  <= g_qfull  + 32'd1;
            if (ev_lenerr) g_lenerr <= g_lenerr + 32'd1;
            // 接收侧（T7）
            if (ev_rxframe) g_rx_frames <= g_rx_frames + 32'd1;
            if (ev_rxfcs)   g_rx_fcs    <= g_rx_fcs    + 32'd1;
            if (ev_rxlen)   g_rx_len    <= g_rx_len    + 32'd1;
            if (ev_rxmac)   g_rx_mac    <= g_rx_mac    + 32'd1;
            if (ev_rxvlunk) g_rx_vlunk  <= g_rx_vlunk  + 32'd1;
            if (ev_rxnet)   g_rx_net    <= g_rx_net    + 32'd1;
            if (ev_rxip)    g_rx_ip     <= g_rx_ip     + 32'd1;
            if (ev_rxic)    g_ic_err    <= g_ic_err    + 32'd1;
            if (ev_rmdup)   g_rm_dup    <= g_rm_dup    + 32'd1;
            if (ev_rmold)   g_rm_old    <= g_rm_old    + 32'd1;
            if (ev_fragto)  g_frag_to   <= g_frag_to   + 32'd1;
            if (ev_rxovf)   g_rx_ovf    <= g_rx_ovf    + 32'd1;
            if (ev_rxctx)   g_rx_ctx    <= g_rx_ctx    + 32'd1;
            if (ev_rxmsg) begin
                g_rx_msgs <= g_rx_msgs + 32'd1;
                g_rx_pay  <= g_rx_pay  + {18'b0, ev_rxbytes};
            end
        end
    end

    //-------------------------------------------------------------------------
    // cfg 读数据（1 拍：地址在 re 拍锁存，下一拍与 RAM 读出对齐）
    //-------------------------------------------------------------------------
    reg [9:0] rd_q;                        // 锁存的读地址（多路选择用）
    always @(posedge clk) begin
        if (reset)      rd_q <= 10'h0;
        else if (stats_re) rd_q <= stats_raddr;   // RAM 读口仍用当拍地址（上拍已采样）
    end

    reg [31:0] stats_mux;
    always @(*) begin
        if (rd_q[9:6] == 4'hF) begin
            case (rd_q[5:0])
                6'h00:   stats_mux = g_frames;
                6'h01:   stats_mux = g_bytes;
                6'h02:   stats_mux = g_msgs;
                6'h03:   stats_mux = g_qfull;
                6'h04:   stats_mux = g_lenerr;
                6'h05:   stats_mux = g_rx_frames;
                6'h06:   stats_mux = g_rx_fcs;
                6'h07:   stats_mux = g_rx_len;
                6'h08:   stats_mux = g_rx_mac;
                6'h09:   stats_mux = g_rx_vlunk;
                6'h0A:   stats_mux = g_rx_net;
                6'h0B:   stats_mux = g_rx_ip;
                6'h0C:   stats_mux = g_ic_err;
                6'h0D:   stats_mux = g_rm_dup;
                6'h0E:   stats_mux = g_rm_old;
                6'h0F:   stats_mux = g_frag_to;
                6'h10:   stats_mux = g_rx_msgs;
                6'h11:   stats_mux = g_rx_pay;
                6'h12:   stats_mux = g_rx_ovf;
                6'h13:   stats_mux = g_rx_ctx;
                default: stats_mux = 32'h0;
            endcase
        end else begin
            case (rd_q[2:1])
                2'd0:    stats_mux = ram_f_rdata;
                2'd1:    stats_mux = ram_b_rdata;
                2'd2:    stats_mux = ram_j_rdata;
                default: stats_mux = 32'h0;
            endcase
        end
    end
    assign stats_rdata = stats_mux;

endmodule
