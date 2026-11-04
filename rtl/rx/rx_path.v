//=============================================================================
// rx_path.v — 接收通路顶层（v2 §4.2 rx 全链；doc/07 §2.3 七级过滤与重组）
//
//   mac_rx×2（A/B GMII → 2 槽帧缓冲 + 描述符）
//     → 串行引擎（两网描述符轮替选帧）：
//        rx_parser（流式头部检查，帧偏移 0..41；vlid 在字节 5 数据拍即出）
//        rx_vl_lookup（4096 接收表，头部读期间重叠查表，kr_re 于 hcnt=7 拍）
//        rx_integrity（IC，按 VL×网；PSN 不论判决均更新）
//        rx_redundancy（RM，MODE=1 距离窗，First-Win；SN=0 旁路 + SkewMax 窗内
//                      副本去重，VERDICT-009 §1）
//        单帧 → 交付字节 FIFO；分片 → rx_reassembly 上下文（任一分片皆先
//        f_alloc 建座/复座，乱序中/尾片先到可建上下文，VERDICT-010 §1-01；
//        MF=1 分片载荷长度 8B 对齐在 E_LUT 拒绝，REV-003-03）
//        （上下文齐备 → 读出 → 交付字节 FIFO）
//     → hrx（冻结接口 v2 §4.3 ③：sop/eop/port/meta）→ 应用口信封格式化
//       （rx_data = VLID 2B 大端 + 载荷，与 tx 注入信封对称，doc/02 §3）
//
// 引擎节拍：帧头 43 拍 + SN/IC/RM ≈ 8 拍 + 载荷搬运 N 拍 ≈ N+51 < N+67
// （线上一帧最少拍数 = 帧 47B 开销 + 前导/IFG 20B），满线速背靠背不积压。
// 交付字节 FIFO 16384B：单条消息 ≤8192B 全量驻留 + 跨消息数拍裕量。
// hrx_port 恒 0：T7 不做 64 口 CAM 分发（v2 §4.7 rx_port_dispatch 属 P1，
// VERDICT-007 §4 未列、doc/06 接收类无对应用例；RPT-T7-001 报备）。
//=============================================================================

module rx_path #(
    parameter integer RX_VL_NUM = 4096
)(
    input  wire        clk,
    input  wire        reset,
    input  wire        tick_us,
    input  wire [63:0] time_ns,

    // ---- GMII 接收（T7：核内 reg 由 tb 层级驱动；T8：MAC IP）----
    input  wire [7:0]  rxd_a,
    input  wire        dv_a,
    input  wire        er_a,
    input  wire [7:0]  rxd_b,
    input  wire        dv_b,
    input  wire        er_b,

    // ---- cfg 接收 VL 表（0x10000 区，cfg_bus 广播/读回）----
    input  wire        rxcfg_we,
    input  wire [10:0] rxcfg_idx,
    input  wire        rxcfg_wsel,
    input  wire [31:0] rxcfg_wdata,
    input  wire        rxcfg_re,
    input  wire [10:0] rxcfg_ridx,
    input  wire        rxcfg_rsel,
    output wire [31:0] rxcfg_rdata,

    // ---- 上电清零（ram_init sel=5/6/7）----
    input  wire        ri_we5,
    input  wire [12:0] ri_addr5,
    input  wire        ri_we6,
    input  wire [11:0] ri_addr6,
    input  wire        ri_we7,
    input  wire [11:0] ri_addr7,

    // ---- stats 事件（单拍脉冲）----
    output wire        ev_rxframe,
    output wire        ev_fcserr,
    output wire        ev_lenerr,      // runt/超长/smax/pad 口径
    output wire        ev_macerr,
    output wire        ev_vlunk,
    output wire        ev_neterr,
    output wire        ev_iperr,       // etype/ver/ihl/proto/csum/ulen
    output wire        ev_icerr,
    output wire        ev_rmdup,
    output wire        ev_rmold,
    output wire        ev_fragto,
    output wire        ev_ovf,
    output wire        ev_ctxerr,      // 重组异常（无座/无匹配/重叠/短尾）
    output wire        ev_rxmsg,       // 应用口交付一条消息
    output wire [13:0] ev_rxbytes,     // 该消息载荷字节数（ev_rxmsg 拍有效）

    // ---- 应用接收口（信封：VLID 2B 大端 + 载荷）----
    output wire [7:0]  rx_data,
    output wire        rx_valid
);

    //=========================================================================
    // 引擎寄存器（先声明——mac_rx/parser/lookup/IC/RM/重组实例引用）
    //=========================================================================
    localparam [3:0] E_IDLE = 4'd0, E_HDR = 4'd1, E_LUT = 4'd2, E_SN = 4'd3,
                     E_IC  = 4'd4, E_RM  = 4'd5, E_CPY = 4'd6, E_FW  = 4'd7,
                     E_CTXW= 4'd8, E_FREE= 4'd9;
    reg [3:0]  estate;
    reg        eng_net_a;                    // 当前帧来源网（1=A）
    reg        eng_slot;                     // 当前帧槽号
    reg [10:0] eng_len;                      // 完整帧长（含 FCS）
    reg [63:0] eng_ts;                       // SFD 时基（hrx_meta 用）
    reg [5:0]  hcnt;                         // 头部读节拍
    reg [10:0] r_off;                        // 环缓冲读偏移
    reg [12:0] p_lo;                         // 载荷起始帧偏移
    reg [13:0] eng_plen;                     // 载荷字节数 N
    reg        eng_frag_q;                   // 分片标志
    reg        eng_first_q;                  // 首片（off==0）
    reg [13:0] c_issue, c_done;              // 拷贝流水计数
    reg [13:0] c_ctx;                        // 重组读出计数
    reg        cstream;                      // 地址流水进行中
    reg        fa_pend;                      // alloc 已受理，待发 f_start
    reg        ic_wait, rm_wait;             // IC/RM 启动脉冲已发
    reg        sn_wait;                      // SN 读两拍等待（地址发出次拍数据到）
    reg        dva_re, dvb_re, ret_a, ret_b;
    reg        hp_start, hp_valid, kr_re;    // 引擎驱动单拍（parser/lookup）
    reg        ic_start, rm_start;           // 引擎驱动单拍（IC/RM）
    reg        f_alloc, f_start, r_start;    // 引擎驱动单拍（重组）
    reg  [7:0] eng_sn;                       // 帧尾 SN
    reg  [1:0] f_done_ctx_q;                 // 齐备上下文号
    wire        bf_we;
    wire [7:0]  bf_wdata;
    // 查表结果锁存（kr_vld 单拍脉冲在帧头读早期——引擎 E_LUT 到来前已过，
    // 必须粘滞锁存；新帧头部启动拍清除）
    reg        lk_vld, lk_hit, lk_ic_en, lk_rm_en;
    reg [1:0]  lk_mask;
    reg [10:0] lk_smax;
    reg [15:0] lk_skew;
    reg        mf_we;
    reg [143:0] mf_wdata;
    reg [15:0] sport_q;                    // off==0 帧的 UDP sport（分片交付元组
                                           // 用：尾片 34..39 为数据，不可取当拍值）
    reg        ev_macerr_r, ev_vlunk_r, ev_neterr_r, ev_iperr_r;
    reg        ev_icerr_r, ev_rmdup_r, ev_rmold_r, ev_ctxerr_r;
    reg        ev_rxmsg_r, ev_lenerr_e_r;
    reg [13:0] ev_rxbytes_r;

    //=========================================================================
    // mac_rx ×2（读口/描述符/归还按当前网多路）
    //=========================================================================
    wire        dva_vld, dvb_vld;
    wire        dva_slot, dvb_slot;
    wire [10:0] dva_len,  dvb_len;
    wire [63:0] dva_ts,   dvb_ts;
    wire        ev_frame_a, ev_frame_b;
    wire        ev_fcs_a,   ev_fcs_b;
    wire        ev_len_a,   ev_len_b;
    wire        ev_ovf_a,   ev_ovf_b;
    wire [11:0] eng_raddr;                 // 引擎读地址（当前网）
    wire [7:0]  rda_data, rdb_data;

    mac_rx #(.NET_ID(2'b01)) u_mrx_a (
        .clk(clk), .reset(reset), .time_ns(time_ns),
        .rxd(rxd_a), .rx_dv(dv_a), .rx_er(er_a),
        .rd_addr(eng_net_a ? eng_raddr : 12'h0), .rd_data(rda_data),
        .desc_vld(dva_vld), .desc_slot(dva_slot), .desc_len(dva_len),
        .desc_ts(dva_ts), .desc_re(dva_re),
        .ret_valid(ret_a), .ret_slot(eng_slot),
        .ev_frame(ev_frame_a), .ev_fcserr(ev_fcs_a), .ev_lenerr(ev_len_a),
        .ev_overflow(ev_ovf_a));

    mac_rx #(.NET_ID(2'b10)) u_mrx_b (
        .clk(clk), .reset(reset), .time_ns(time_ns),
        .rxd(rxd_b), .rx_dv(dv_b), .rx_er(er_b),
        .rd_addr(!eng_net_a ? eng_raddr : 12'h0), .rd_data(rdb_data),
        .desc_vld(dvb_vld), .desc_slot(dvb_slot), .desc_len(dvb_len),
        .desc_ts(dvb_ts), .desc_re(dvb_re),
        .ret_valid(ret_b), .ret_slot(eng_slot),
        .ev_frame(ev_frame_b), .ev_fcserr(ev_fcs_b), .ev_lenerr(ev_len_b),
        .ev_overflow(ev_ovf_b));

    wire [7:0] eng_rdata = eng_net_a ? rda_data : rdb_data;

    //=========================================================================
    // 解析 / 查表 / IC / RM / 重组
    //=========================================================================
    wire        hp_done;
    wire        hp_ok_mac, hp_ok_etype, hp_ok_ip, hp_ok_udp;
    wire [15:0] hp_vlid;
    wire [31:0] hp_sip;
    wire [15:0] hp_ipid, hp_sport, hp_dport, hp_ulen, hp_iptot;
    wire        hp_mf;
    wire [12:0] hp_off;
    wire [7:0]  hp_data = eng_rdata;

    rx_parser u_par (
        .clk(clk), .reset(reset),
        .hp_start(hp_start), .hp_valid(hp_valid), .hp_data(hp_data),
        .hp_done(hp_done),
        .hp_ok_mac(hp_ok_mac), .hp_ok_etype(hp_ok_etype),
        .hp_ok_ip(hp_ok_ip), .hp_ok_udp(hp_ok_udp),
        .hp_vlid(hp_vlid), .hp_sip(hp_sip), .hp_ipid(hp_ipid),
        .hp_iptot(hp_iptot), .hp_mf(hp_mf), .hp_off(hp_off),
        .hp_sport(hp_sport), .hp_dport(hp_dport), .hp_ulen(hp_ulen));

    wire        kr_vld, kr_hit, kr_ic_en, kr_rm_en;
    wire [1:0]  kr_mask;
    wire [10:0] kr_smax;
    wire [15:0] kr_skew;

    rx_vl_lookup #(.RX_VL_NUM(RX_VL_NUM)) u_lut (
        .clk(clk), .reset(reset),
        .rxcfg_we(rxcfg_we), .rxcfg_idx(rxcfg_idx), .rxcfg_wsel(rxcfg_wsel),
        .rxcfg_wdata(rxcfg_wdata),
        .rxcfg_re(rxcfg_re), .rxcfg_ridx(rxcfg_ridx), .rxcfg_rsel(rxcfg_rsel),
        .rxcfg_rdata(rxcfg_rdata),
        .ri_we(ri_we7), .ri_addr(ri_addr7),
        .kr_re(kr_re), .kr_vlid_in(hp_vlid),
        .kr_vld(kr_vld), .kr_hit(kr_hit), .kr_ic_en(kr_ic_en),
        .kr_rm_en(kr_rm_en), .kr_mask(kr_mask), .kr_smax(kr_smax),
        .kr_skew(kr_skew));

    wire        ic_vld;
    wire [1:0]  ic_verdict;

    rx_integrity #(.RX_VL_NUM(RX_VL_NUM)) u_ic (
        .clk(clk), .reset(reset),
        .ri_we(ri_we5), .ri_addr(ri_addr5),
        .ic_start(ic_start), .ic_vl(hp_vlid[11:0]), .ic_net(!eng_net_a),
        .ic_sn(eng_sn),
        .ic_vld(ic_vld), .ic_verdict(ic_verdict), .ic_psn_old());

    wire        rm_vld, rm_accept;
    wire [7:0]  rm_rsn_old;
    wire [23:0] rm_us;

    rx_redundancy #(.RX_VL_NUM(RX_VL_NUM)) u_rm (
        .clk(clk), .reset(reset), .tick_us(tick_us),
        .ri_we(ri_we6), .ri_addr(ri_addr6),
        .rm_start(rm_start), .rm_vl(hp_vlid[11:0]), .rm_sn(eng_sn),
        .rm_skew(lk_skew),
        .rm_vld(rm_vld), .rm_accept(rm_accept), .rm_rsn_old(rm_rsn_old),
        .rm_us(rm_us));

    // dup/old 计数区分：d == 0 → 重复（与 RM 判决式同口径）
    wire [7:0] d_chk = (eng_sn >= rm_rsn_old) ? (eng_sn - rm_rsn_old)
                                             : (eng_sn - rm_rsn_old + 8'd255);

    wire        f_alloc_ok, f_busy, f_done, f_aborted, f_timeout;
    wire [1:0]  f_done_ctx, f_abort_ctx, f_to_ctx;
    wire        r_valid, r_last;
    wire [7:0]  r_data;
    wire [13:0] r_total;
    wire        copy_valid;                // 载荷字节呈现拍（→ f_valid）

    rx_reassembly #(.MAX_MSG_W(14'd8192), .TIMEOUT_W(14'd10000)) u_asm (
        .clk(clk), .reset(reset), .tick_us(tick_us),
        .f_alloc(f_alloc), .f_vl(hp_vlid[11:0]), .f_ipid(hp_ipid),
        .f_alloc_ok(f_alloc_ok),
        .f_start(f_start), .f_off8(hp_off[9:0]), .f_len(eng_plen[10:0]),
        .f_mf(hp_mf), .f_udplen(hp_ulen),
        .f_data(eng_rdata), .f_valid(copy_valid && eng_frag_q),
        .f_busy(f_busy), .f_done(f_done), .f_done_ctx(f_done_ctx),
        .f_aborted(f_aborted), .f_abort_ctx(f_abort_ctx),
        .f_timeout(f_timeout), .f_to_ctx(f_to_ctx),
        .r_start(r_start), .r_ctx(f_done_ctx_q), .r_data(r_data),
        .r_valid(r_valid), .r_last(r_last), .r_ack(r_valid && r_last),
        .r_total(r_total));

    //=========================================================================
    // 交付字节 FIFO + 元组 FIFO
    //=========================================================================
    wire [7:0]  bf_rdata;
    wire        bf_empty;
    wire        bf_re;

    sync_fifo #(.DW(8), .DEPTH(16384), .AW(14)) u_bf (
        .clk(clk), .reset(reset),
        .we(bf_we), .wdata(bf_wdata), .re(bf_re),
        .rdata(bf_rdata), .full(), .empty(bf_empty),
        .count(), .almost_full());

    // 元组：{vlid[15:0], len[13:0], sip[31:0], sport[15:0], ts[63:0], net[1:0]}
    wire [143:0] mf_rdata;
    wire        mf_empty;
    wire        mf_re;

    sync_fifo #(.DW(144), .DEPTH(16), .AW(4)) u_mf (
        .clk(clk), .reset(reset),
        .we(mf_we), .wdata(mf_wdata), .re(mf_re),
        .rdata(mf_rdata), .full(), .empty(mf_empty),
        .count(), .almost_full());

    wire [15:0] m_vlid  = mf_rdata[143:128];
    wire [13:0] m_len   = mf_rdata[127:114];
    wire [31:0] m_sip   = mf_rdata[113:82];
    wire [15:0] m_sport = mf_rdata[81:66];
    wire [63:0] m_ts    = mf_rdata[65:2];
    wire [1:0]  m_net   = mf_rdata[1:0];

    //=========================================================================
    // hrx 源（冻结接口 v2 §4.3 ③）：字节/元组 FIFO → 流 + sop/eop/port/meta
    //=========================================================================
    reg  [13:0] hpos;                      // 本消息剩余载荷字节数（0=无消息在呈）
    wire        hrx_arm   = (hpos == 14'd0) && !mf_empty && !bf_empty;
    wire        hrx_valid = (hpos != 14'd0) && !bf_empty;
    wire        hrx_sop   = hrx_valid && (hpos == {2'b00, m_len});
    wire        hrx_eop   = hrx_valid && (hpos == 14'd1);
    wire [7:0]  hrx_data  = bf_rdata;
    wire        hrx_ready;               // 格式化器给出（F_B 态）
    wire        hrx_xfer  = hrx_valid && hrx_ready;
    wire [5:0]  hrx_port  = 6'd0;        // T7：无端口 CAM（头注报备）
    wire [141:0] hrx_meta = {m_sip, m_sport, m_ts, m_vlid, m_net};  // sop 拍有效

    assign bf_re = hrx_xfer;
    assign mf_re = hrx_xfer && hrx_eop;    // 末载荷字节拍弹元组

    always @(posedge clk) begin
        if (reset)
            hpos <= 14'd0;
        else if (hrx_arm)
            hpos <= {2'b00, m_len};
        else if (hrx_xfer)
            hpos <= hpos - 14'd1;
    end

    //=========================================================================
    // 应用口格式化：meta（VLID 2B 大端）+ hrx 载荷 → rx_data/rx_valid 信封
    //=========================================================================
    localparam [2:0] F_IDLE = 3'd0, F_VL2 = 3'd1, F_B = 3'd2, F_LAST = 3'd3,
                     F_GAP = 3'd4;
    reg [2:0]  fstate;
    reg [15:0] f_vlid;
    reg [7:0]  rx_data_r;
    reg        rx_valid_r;

    assign rx_data  = rx_data_r;
    assign rx_valid = rx_valid_r;
    assign hrx_ready = (fstate == F_B);    // 仅载荷消费态收 hrx

    always @(posedge clk) begin
        if (reset) begin
            fstate <= F_IDLE;
            f_vlid <= 16'h0;
            rx_data_r <= 8'h0; rx_valid_r <= 1'b0;
        end else begin
            case (fstate)
                F_IDLE: begin
                    rx_valid_r <= 1'b0;
                    if (!mf_empty && !bf_empty) begin
                        f_vlid     <= m_vlid;
                        rx_valid_r <= 1'b1;
                        rx_data_r  <= m_vlid[15:8];  // 次拍：VLID 高字节
                        fstate     <= F_VL2;
                    end
                end
                F_VL2: begin
                    rx_data_r <= f_vlid[7:0];        // 次拍：VLID 低字节
                    fstate    <= F_B;
                end
                F_B: begin
                    if (hrx_valid) begin
                        rx_data_r <= hrx_data;       // 次拍起：载荷逐字节
                        if (hrx_eop)
                            fstate <= F_LAST;        // 末字节次拍上线后再收尾
                    end
                end
                F_LAST: begin
                    rx_valid_r <= 1'b0;              // 末字节本拍已呈现
                    fstate     <= F_GAP;
                end
                F_GAP: fstate <= F_IDLE;             // 消息间 ≥1 拍 rx_valid=0
                default: fstate <= F_IDLE;
            endcase
        end
    end

    //=========================================================================
    // 串行引擎
    //=========================================================================
    assign eng_raddr = {eng_slot, r_off};

    // 载荷拷贝流水：字节 j 地址于第 j+1 拍发出（RAM 同沿采样）、第 j+2 拍
    // 数据到——呈现条件 c_issue >= c_done+2（c_issue 计已发地址数）。
    // 末地址多发一发（c_issue <= plen，读到 SN/FCS 区无害）以保证末字节呈现。
    wire copy_run     = (estate == E_CPY) && cstream;
    assign copy_valid = copy_run && (c_done < eng_plen)
                      && (c_issue >= c_done + 14'd2);
    // 交付 FIFO 写通道：单帧拷贝直写 / 重组读出转写（连续赋值，单驱动）
    assign bf_we    = (estate == E_CTXW) ? r_valid : (copy_valid && !eng_frag_q);
    assign bf_wdata = (estate == E_CTXW) ? r_data  : eng_rdata;

    // E_LUT 判决辅助（hp_* 与 eng_len 均已稳定）
    wire [12:0] ip_end = 13'd14 + {9'b0, hp_iptot};      // IP 区尾帧偏移
    wire [10:0] sn_off = eng_len - 11'd5;                // SN 帧偏移（不含 FCS）
    wire        pad_ok = ({1'b0, ip_end} <= {3'b0, sn_off})
                       && (({3'b0, sn_off} - {1'b0, ip_end}) <= 13'd16);
    wire [13:0] plen_c  = (hp_off != 13'd0) ? ({3'b000, hp_iptot} - 14'd20)   // 中/尾片
                        : (hp_mf             ? ({3'b000, hp_iptot} - 14'd28)   // 首片（含 UDP 头）
                                             : ({2'b00, hp_ulen} - 14'd8));    // 单帧：ulen-8
    // REV-F-01（冻结终审）：分片边界算术改 17 位不回绕——{hp_off,3'b000}+plen_c
    // 全宽表达偏移字节地址（65528+8192<2^17）再与 8192 比较；旧式 14 位乘加在
    // hp_off>=2048（8B 单位）时回绕别名到低偏移，越界分片绕过 MAX_MSG_W 校验
    // 进入重组。f_off8 传参 hp_off[9:0] 不动（越界分片已在此拒收，不入重组）。
    wire [16:0] frag_end = {hp_off, 3'b000} + {3'b000, plen_c};
    wire        plen_ok = (plen_c >= 14'd1) && (plen_c <= 14'd8192)
                        && (frag_end <= 17'd8192);
    // MF=1 分片载荷长度须 8B 对齐（RFC 791 §3.2，REV-003-03/FIX3；尾片 MF=0 豁免）
    wire        mf_align_fail = hp_mf && (plen_c[2:0] != 3'b000);
    wire        len_fail  = !pad_ok || !plen_ok || mf_align_fail;
    wire        smax_fail = (lk_smax != 11'd0) && (plen_c > {3'b0, lk_smax});

    // stats 事件汇聚
    assign ev_rxframe = ev_frame_a | ev_frame_b;
    assign ev_fcserr  = ev_fcs_a   | ev_fcs_b;
    assign ev_lenerr  = ev_len_a   | ev_len_b | ev_lenerr_e_r;
    assign ev_ovf     = ev_ovf_a   | ev_ovf_b;
    assign ev_macerr  = ev_macerr_r;
    assign ev_vlunk   = ev_vlunk_r;
    assign ev_neterr  = ev_neterr_r;
    assign ev_iperr   = ev_iperr_r;
    assign ev_icerr   = ev_icerr_r;
    assign ev_rmdup   = ev_rmdup_r;
    assign ev_rmold   = ev_rmold_r;
    assign ev_fragto  = f_timeout;
    assign ev_ctxerr  = ev_ctxerr_r;
    assign ev_rxmsg   = ev_rxmsg_r;
    assign ev_rxbytes = ev_rxbytes_r;

    always @(posedge clk) begin
        if (reset) begin
            estate <= E_IDLE;
            eng_net_a <= 1'b0; eng_slot <= 1'b0; eng_len <= 11'd0; eng_ts <= 64'd0;
            hcnt <= 6'd0; r_off <= 11'd0; eng_sn <= 8'd0;
            p_lo <= 13'd0; eng_plen <= 14'd0;
            eng_frag_q <= 1'b0; eng_first_q <= 1'b0;
            c_issue <= 14'd0; c_done <= 14'd0; c_ctx <= 14'd0;
            cstream <= 1'b0; fa_pend <= 1'b0;
            ic_wait <= 1'b0; rm_wait <= 1'b0;
            dva_re <= 1'b0; dvb_re <= 1'b0; ret_a <= 1'b0; ret_b <= 1'b0;
            hp_start <= 1'b0; hp_valid <= 1'b0; kr_re <= 1'b0;
            ic_start <= 1'b0; rm_start <= 1'b0;
            f_alloc <= 1'b0; f_start <= 1'b0; r_start <= 1'b0;
            f_done_ctx_q <= 2'd0;
            lk_vld <= 1'b0; lk_hit <= 1'b0; lk_ic_en <= 1'b0; lk_rm_en <= 1'b0;
            lk_mask <= 2'b00; lk_smax <= 11'd0; lk_skew <= 16'd0;
            sn_wait <= 1'b0;
            mf_we <= 1'b0; mf_wdata <= 144'd0; sport_q <= 16'd0;
            ev_macerr_r <= 1'b0; ev_vlunk_r <= 1'b0; ev_neterr_r <= 1'b0;
            ev_iperr_r <= 1'b0; ev_icerr_r <= 1'b0; ev_rmdup_r <= 1'b0;
            ev_rmold_r <= 1'b0; ev_ctxerr_r <= 1'b0; ev_rxmsg_r <= 1'b0;
            ev_lenerr_e_r <= 1'b0; ev_rxbytes_r <= 14'd0;
        end else begin
            // 单拍默认
            dva_re <= 1'b0; dvb_re <= 1'b0; ret_a <= 1'b0; ret_b <= 1'b0;
            hp_start <= 1'b0; hp_valid <= 1'b0; kr_re <= 1'b0;
            ic_start <= 1'b0; rm_start <= 1'b0;
            f_alloc <= 1'b0; f_start <= 1'b0; r_start <= 1'b0;
            mf_we <= 1'b0;
            ev_macerr_r <= 1'b0; ev_vlunk_r <= 1'b0; ev_neterr_r <= 1'b0;
            ev_iperr_r <= 1'b0; ev_icerr_r <= 1'b0; ev_rmdup_r <= 1'b0;
            ev_rmold_r <= 1'b0; ev_ctxerr_r <= 1'b0; ev_rxmsg_r <= 1'b0;
            ev_lenerr_e_r <= 1'b0;

            // 查表结果锁存：kr_vld 拍抓取；新帧启动拍清有效
            if (kr_vld) begin
                lk_vld   <= 1'b1;
                lk_hit   <= kr_hit;
                lk_ic_en <= kr_ic_en;
                lk_rm_en <= kr_rm_en;
                lk_mask  <= kr_mask;
                lk_smax  <= kr_smax;
                lk_skew  <= kr_skew;
            end else if ((estate == E_HDR) && (hcnt == 6'd0)) begin
                lk_vld <= 1'b0;
            end

            case (estate)
                // ---- 选帧：两网描述符轮替（上轮 A 则本轮 B 优先）----
                E_IDLE: begin
                    if (dvb_vld && (!dva_vld || eng_net_a)) begin
                        eng_net_a <= 1'b0;
                        eng_slot  <= dvb_slot;
                        eng_len   <= dvb_len;
                        eng_ts    <= dvb_ts;
                        dvb_re    <= 1'b1;
                        hcnt      <= 6'd0;
                        estate    <= E_HDR;
                    end else if (dva_vld) begin
                        eng_net_a <= 1'b1;
                        eng_slot  <= dva_slot;
                        eng_len   <= dva_len;
                        eng_ts    <= dva_ts;
                        dva_re    <= 1'b1;
                        hcnt      <= 6'd0;
                        estate    <= E_HDR;
                    end
                end

                // ---- 头部 42 字节流：r_off=hcnt 发地址，hcnt+1 拍出数据 ----
                E_HDR: begin
                    r_off <= {1'b0, hcnt};
                    if (hcnt <= 6'd42)
                        hcnt <= hcnt + 6'd1;
                    if (hcnt == 6'd0)
                        hp_start <= 1'b1;            // 入口拍启动解析器
                    if (hcnt >= 6'd1 && hcnt <= 6'd42)
                        hp_valid <= 1'b1;            // 字节 hcnt-1 本拍呈现
                    if (hcnt == 6'd7)
                        kr_re <= 1'b1;               // vlid 已稳定（字节 5 数据拍寄存）
                    if (hp_done)
                        estate <= E_LUT;             // 校验结果 + 字段已稳定
                end

                // ---- 查表结果 + 全量判决（v2 §5.5 优先序）----
                E_LUT: begin
                    if (lk_vld) begin
                        if (!hp_ok_mac) begin
                            ev_macerr_r <= 1'b1;
                            estate <= E_FREE;
                        end else if (!lk_hit) begin
                            ev_vlunk_r <= 1'b1;
                            estate <= E_FREE;
                        end else if (!(eng_net_a ? lk_mask[0] : lk_mask[1])) begin
                            ev_neterr_r <= 1'b1;
                            estate <= E_FREE;
                        end else if (!(hp_ok_etype && hp_ok_ip)) begin
                            ev_iperr_r <= 1'b1;
                            estate <= E_FREE;
                        end else if ((hp_off == 13'd0) && !hp_ok_udp) begin
                            ev_iperr_r <= 1'b1;      // off==0：UDP Length 口径
                            estate <= E_FREE;
                        end else if (len_fail) begin
                            ev_lenerr_e_r <= 1'b1;
                            estate <= E_FREE;
                        end else if (smax_fail) begin
                            ev_lenerr_e_r <= 1'b1;   // Smax（载荷层级，doc/02 §8.2-3）
                            estate <= E_FREE;
                        end else begin
                            p_lo       <= (hp_off == 13'd0) ? 13'd42 : 13'd34;
                            eng_plen   <= plen_c;
                            eng_frag_q <= hp_mf || (hp_off != 13'd0);
                            eng_first_q<= (hp_off == 13'd0);
                            if (hp_off == 13'd0)
                                sport_q <= hp_sport;   // off==0 帧 sport（交付元组）
                            r_off      <= {1'b0, sn_off};  // 发 SN 读地址
                            sn_wait    <= 1'b0;
                            estate     <= E_SN;
                        end
                    end
                end

                // ---- SN（帧尾）----
                E_SN: begin
                    // r_off 在 E_LUT 末拍发出：本态第 1 拍 RAM 采样，第 2 拍数据到
                    if (!sn_wait) begin
                        sn_wait <= 1'b1;
                    end else begin
                        eng_sn  <= eng_rdata;        // SN 帧尾字节
                        sn_wait <= 1'b0;
                        c_issue <= 14'd0;
                        c_done  <= 14'd0;
                        cstream <= 1'b0;
                        fa_pend <= 1'b0;
                        ic_wait <= 1'b0;
                        rm_wait <= 1'b0;
                        estate  <= lk_ic_en ? E_IC : (lk_rm_en ? E_RM : E_CPY);
                    end
                end

                // ---- IC（按 VL×网；PSN 不论判决均更新）----
                E_IC: begin
                    if (!ic_wait) begin
                        ic_start <= 1'b1;
                        ic_wait  <= 1'b1;
                    end else if (ic_vld) begin
                        ic_wait <= 1'b0;
                        if (ic_verdict == 2'd0) begin
                            ev_icerr_r <= 1'b1;
                            estate <= E_FREE;
                        end else begin
                            estate <= lk_rm_en ? E_RM : E_CPY;
                        end
                    end
                end

                // ---- RM（MODE=1 距离窗；First-Win）----
                E_RM: begin
                    if (!rm_wait) begin
                        rm_start <= 1'b1;
                        rm_wait  <= 1'b1;
                    end else if (rm_vld) begin
                        rm_wait <= 1'b0;
                        if (rm_accept) begin
                            estate <= E_CPY;
                        end else begin
                            if (d_chk == 8'd0)
                                ev_rmdup_r <= 1'b1;  // 重复副本
                            else
                                ev_rmold_r <= 1'b1;  // 旧帧 / 严重乱序
                            estate <= E_FREE;
                        end
                    end
                end

                // ---- 载荷搬运：单帧→交付 FIFO；分片→重组上下文 ----
                E_CPY: begin
                    if (!cstream && !fa_pend) begin
                        // 启动拍：任一分片先 f_alloc（同键已在座为无操作、无座则
                        // 占新座——乱序中/尾片先到也能建上下文，VERDICT-010 §1-01；
                        // 次拍 f_start）
                        if (eng_frag_q) begin
                            f_alloc <= 1'b1;
                            if (!f_alloc_ok) begin
                                ev_ctxerr_r <= 1'b1;
                                estate      <= E_FREE;
                            end else begin
                                fa_pend <= 1'b1;     // 次拍发 f_start
                            end
                        end else begin
                            cstream <= 1'b1;
                            r_off   <= p_lo[10:0];   // 单帧：直接开流
                            c_issue <= 14'd1;
                        end
                    end else if (fa_pend) begin
                        // alloc 次拍：发 f_start + 首地址
                        f_start <= 1'b1;
                        fa_pend <= 1'b0;
                        cstream <= 1'b1;
                        r_off   <= p_lo[10:0];
                        c_issue <= 14'd1;
                    end else begin
                        // 地址流水 + 数据呈现（copy_valid 组合）
                        if (c_issue <= eng_plen) begin
                            r_off   <= p_lo[10:0] + c_issue[10:0];
                            c_issue <= c_issue + 14'd1;
                        end
                        if (copy_valid)
                            c_done <= c_done + 14'd1;
                        // 完成判定：末字节呈现拍（copy_valid 且 c_done=plen-1）
                        if (copy_valid && (c_done == eng_plen - 14'd1)) begin
                            cstream <= 1'b0;
                            if (!eng_frag_q) begin
                                mf_we        <= 1'b1;
                                mf_wdata     <= {hp_vlid, eng_plen, hp_sip, hp_sport,
                                                 eng_ts, eng_net_a ? 2'b01 : 2'b10};
                                ev_rxmsg_r   <= 1'b1;
                                ev_rxbytes_r <= eng_plen;
                                estate       <= E_FREE;
                            end else begin
                                estate <= E_FW;      // 等重组片尾判定
                            end
                        end
                    end
                end

                // ---- 重组片尾判定等待 ----
                // mf=1 的非末片：f_busy 落下且无 done/abort 脉冲 = 正常（上下文
                // 留存等后续片）→ 直接释放槽位回 idle；f_done → 读出交付；
                // f_aborted（重叠/短尾/无匹配）→ 计 ctxerr 丢弃。
                E_FW: begin
                    if (!f_busy) begin
                        if (f_done) begin
                            f_done_ctx_q <= f_done_ctx;
                            c_ctx        <= 14'd0;
                            r_start      <= 1'b1;
                            estate       <= E_CTXW;
                        end else if (f_aborted) begin
                            ev_ctxerr_r <= 1'b1;
                            estate      <= E_FREE;
                        end else begin
                            ret_a   <= eng_net_a;    // 非末片：帧处理完毕
                            ret_b   <= !eng_net_a;
                            estate  <= E_IDLE;
                        end
                    end
                end

                // ---- 齐备消息读出 → 交付 FIFO ----
                E_CTXW: begin
                    if (r_valid) begin
                        if (r_last) begin
                            // 分片消息交付元组：sport 取首片（off==0）帧锁存值
                            // （完成帧为中/尾片时 34..39 是载荷数据，hp_sport 无效）
                            mf_we        <= 1'b1;
                            mf_wdata     <= {hp_vlid, c_ctx + 14'd1, hp_sip, sport_q,
                                             eng_ts, eng_net_a ? 2'b01 : 2'b10};
                            ev_rxmsg_r   <= 1'b1;
                            ev_rxbytes_r <= c_ctx + 14'd1;
                            estate       <= E_FREE;  // r_ack 已随 r_last 发出
                        end else begin
                            c_ctx <= c_ctx + 14'd1;
                        end
                    end
                end

                // ---- 释放槽位 ----
                E_FREE: begin
                    ret_a   <= eng_net_a;
                    ret_b   <= !eng_net_a;
                    estate  <= E_IDLE;
                end

                default: estate <= E_IDLE;
            endcase
        end
    end

endmodule
