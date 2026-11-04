//=============================================================================
// buf_mgr.v — 发送缓冲管理器（v2 §4.2 lib，★4 发送/接收共用关键路径的发送侧）
//
// 职责：htx 流入队（slot 池存储 + per-VL 描述符环）+ 组帧侧出队（描述符读、
//       载荷走线读、分片推进/释放）。
//
// 存储结构（预算 v2 §4.4：slot 池 32×2KB、描述符 128 队列×4 深×40b）：
//   - slot 池：TX_SLOTS×SLOT_BYTES RAM（口 A 入队写 / 口 B 组帧读）；
//   - 空闲链：32b 位图 + 计数（复位全 1，无需 RAM 初始化）；
//   - slot 链：next_ptr 寄存器阵列（组合读，跨 slot 连续载荷/释放行走）；
//   - 描述符：512×40b 双口 RAM（口 A 入队写，口 B 组帧读），队首/队尾/计数为
//     寄存器（复位清零 → 未写条目永不被读，RAM 无需上电清零）；
//   - 分片游标 fidx / 消息级 IP ID 计数器：寄存器阵列。
//
// 一条消息 = 1 个描述符（占 1 个队列深度），分片在出队侧逐 BAG 逐帧生成
// （doc/01 §3：p 片消息耗时 ≤ p×BAG+jitter+tech；TC-PERF-06 同口径）。
//   描述符 = {slot0[5:0], 总长[13:0], ip_id[15:0], pad[3:0]}（40b）。
//   分片几何（与 scripts/afdx_ref.py message_to_frames 一致，改动须两处同步）：
//     首片载荷 = 1456 = ((1471-8)/8)*8；中片 = 1464 = 1471 & ~7；
//     片 k≥1 偏移 = 1456+(k-1)*1464（8B 对齐）。
//
// 入队检查（v2 §5.6 "入队时检查" + C 组定案，结论见 doc/02 §8.2 增补）：
//   - sop：队列深度 < Q_DEPTH 且空闲 slot ≥ 每消息最大占用(4)——此判定在整条
//     消息期间稳定（本 VL 计数只在本 sop 增长，空闲只增不减）→ 中途不停顿；
//   - eop：总长 ≤ MAX_MSG(8192)；Smax 按应用载荷层级：单帧 N ≤ Smax，分片消息
//     最大片 1464 ≤ Smax（违反 = ARINC 配置违例，计 tx_len_err，整条丢弃）；
//     超过 MAX_MSG 的消息在接收中途停写（enq_discard，链恰 4 槽整），eop 丢弃，
//     保证任何长度消息的释放行走都 ≤ 4 slot。
//   - Smax 影子寄存器复位值 0xFFFF = 未配置不限制（配置经 cfg 写透传 W1）。
//=============================================================================

module buf_mgr #(
    parameter integer TX_VL_NUM  = 128,   // 发送 VL 条数
    parameter integer TX_SLOTS   = 32,    // slot 池深度（≤32，位图按 32b 实现）
    parameter integer SLOT_BYTES = 2048,  // 每 slot 字节数（2KB，v2 §4.4）
    parameter integer Q_DEPTH    = 4,     // 每 VL 队列深度（v2 §4.4 锁定值）
    parameter integer MAX_MSG    = 8192,  // 单条消息上限（字节；=4 slot）
    parameter integer SLW = 6,            // slot 索引位宽
    parameter integer SBW = 11,           // slot 内偏移位宽（2048）
    parameter integer VLW = 7,            // VL 索引位宽（128）
    parameter integer QW  = 2,            // 队列指针位宽（深度 4）
    parameter integer CW  = 3             // 队列计数位宽（0..4）
)(
    input  wire        clk,
    input  wire        reset,

    // ---- htx 主机发送流（v2 §4.3 ②；此处只承载应用载荷，VLID 已由上层剥离）----
    input  wire        htx_valid,
    output wire        htx_ready,
    input  wire        htx_sop,
    input  wire        htx_eop,
    input  wire [7:0]  htx_data,
    input  wire [VLW-1:0] htx_vl,          // 仅 sop 拍有效
    input  wire        htx_abort,          // 单拍：丢弃半截入队消息（核级满丢用）

    // ---- cfg 写透传（VL 表写广播；buf_mgr 只用 W1 = Smax[15:0]）----
    input  wire        cw_we,
    input  wire [VLW-1:0] cw_idx,
    input  wire [2:0]  cw_wsel,
    input  wire [31:0] cw_wdata,

    // ---- 队列状态（调度器 eligible 用）----
    output wire [TX_VL_NUM-1:0] q_nonempty,

    // ---- 描述符读（组帧器；1 拍延迟）----
    input  wire        desc_req,
    input  wire [VLW-1:0] desc_vl,
    output wire        desc_vld,
    output wire [SLW-1:0] desc_slot,       // slot0（链头）
    output wire [13:0] desc_len,           // 消息总长（应用载荷字节）
    output wire [15:0] desc_ipid,          // 本消息 IP ID
    output wire [3:0]  desc_fidx,          // 待发分片序号（影子寄存器，组合读）

    // ---- 帧完成（组帧器 → 推进/弹出）----
    input  wire        frag_upd,           // 单拍
    input  wire        frag_last,          // 1 = 本消息最后一帧已发出 → 弹出
    input  wire [VLW-1:0] frag_vl,
    input  wire [SLW-1:0] frag_slot0,      // 弹出时按链释放
    input  wire [13:0] frag_len,           // 消息总长（算链长）

    // ---- 载荷走线读口（slot RAM 读口，1 拍延迟）----
    input  wire [SLW+SBW-1:0] pre_addr,    // {slot, offset}
    output wire [7:0]  pre_data,

    // ---- next_ptr 组合读（组帧器跨 slot / tb 观测链）----
    input  wire [SLW-1:0] nx_ra,
    output wire [SLW-1:0] nx_rd,

    // ---- 事件脉冲（统计，单拍）----
    output reg         ev_enq,             // 消息入队成功
    output reg         ev_qfull,           // sop 因队列满/槽不足被拒
    output reg         ev_lenerr,          // eop 长度/Smax 违规拒收
    output reg         ev_frame            // 消息最后一帧发出（出队完成）
);

    //-------------------------------------------------------------------------
    // 常量（与 scripts/afdx_ref.py 对齐，改动须两处同步）
    //-------------------------------------------------------------------------
    localparam integer MAX_PAYLOAD  = 1471;                       // 单帧载荷上限
    localparam integer FRAG_MID     = MAX_PAYLOAD & ~7;           // 1464 最大片
    localparam integer SLOTS_PER_MSG = (MAX_MSG + SLOT_BYTES - 1) / SLOT_BYTES; // 4

    //-------------------------------------------------------------------------
    // 状态寄存器（先声明后使用）
    //-------------------------------------------------------------------------
    reg                    enq_active;             // 消息接收中（sop 已接受）
    reg [VLW-1:0]          enq_vl;
    reg [SLW-1:0]          enq_head;               // 链头 slot0
    reg [SLW-1:0]          enq_cur;                // 当前写 slot
    reg [SBW-1:0]          enq_off;                // 下一字节 slot 内偏移
    reg [13:0]             enq_cnt;                // 已收字节数（含 sop 字节）
    reg                    enq_discard;            // 超长消息：中途停写，eop 整条丢弃
    reg                    qfull_latch;            // sop 拒绝事件去重

    reg  [31:0]            free_bits;              // 空闲位图（1=空闲）
    reg  [5:0]             free_cnt;

    reg                    walk_busy;              // 释放行走
    reg [SLW-1:0]          walk_cur;
    reg [2:0]              walk_left;
    // 释放请求队列（REV-002-03：abort/拒收/末帧完成三类源同拍并发不丢）
    // 深度 8 依据：请求源受 room（!walk_busy 拒新 sop）与单 htx 流节流——
    // 同拍最多 3 源（abort+拒收+末帧），持续到达率 ≤ 行走排空率（1 请求/≤5 拍），
    // 8 深覆盖最坏突发而余量 ≥2 倍。
    reg [SLW-1:0]          rq_slot [0:7];
    reg [2:0]              rq_len  [0:7];
    reg [2:0]              rq_wp, rq_rp;
    reg [3:0]              rq_cnt;               

    reg [QW-1:0]   q_head [0:TX_VL_NUM-1];
    reg [QW-1:0]   q_tail [0:TX_VL_NUM-1];
    reg [CW-1:0]   q_cnt  [0:TX_VL_NUM-1];
    reg [15:0]     ipid_r [0:TX_VL_NUM-1];         // 消息级 IP ID，复位=1
    reg [3:0]      fidx_r [0:TX_VL_NUM-1];         // 待发分片序号，复位=0
    reg [15:0]     smax_r [0:TX_VL_NUM-1];         // Smax 影子（复位=0xFFFF 不限）

    reg          desc_vld_r;   // 拍1：读已发起
    reg          desc_vld2;    // 拍2：rdata 有效（与 RAM 1 拍读延迟对齐）
    reg [VLW-1:0] desc_vl_r;
    wire [39:0]  desc_rdata;

    //-------------------------------------------------------------------------
    // 空闲 slot 分配（最低位优先，组合）
    //-------------------------------------------------------------------------
    integer fi;
    reg           al_found;
    reg [SLW-1:0] al_idx;
    always @(*) begin
        al_found = 1'b0;
        al_idx   = {SLW{1'b0}};
        for (fi = 0; fi < TX_SLOTS; fi = fi + 1)
            if (!al_found && free_bits[fi]) begin
                al_idx   = fi[SLW-1:0];
                al_found = 1'b1;
            end
    end
    wire [SLW-1:0] slot_alloc = al_idx;

    //-------------------------------------------------------------------------
    // 传输判定
    //-------------------------------------------------------------------------
    wire room = (q_cnt[htx_vl] < Q_DEPTH[CW-1:0])         // 队列有深度
              && (free_cnt >= SLOTS_PER_MSG[5:0])         // 槽可容最大消息
              && !walk_busy;                              // 释放行走中不收新
    assign htx_ready = htx_sop ? (room && !enq_active)
                               : enq_active;             // 消息中恒就绪（头部注释）
    wire do_xfer = htx_valid && htx_ready;
    wire do_sop  = do_xfer && htx_sop;
    wire do_byt  = do_xfer;
    wire do_eop  = do_xfer && htx_eop;

    wire [VLW-1:0] cur_vl    = do_sop ? htx_vl : enq_vl;
    wire [SLW-1:0] chain_head = do_sop ? slot_alloc : enq_head;  // 本拍可见链头
    // 超长吞模式：enq_cnt 已达 MAX_MSG 后停止写槽（链恰为 4 槽整），eop 整条丢弃。
    // msg_over 用当拍组合判定（enq_discard 是 NBA，本拍还看不到）
    wire msg_over  = do_byt && !do_sop && (enq_cnt >= MAX_MSG[13:0]);
    wire slot_wrap = do_byt && !do_sop && !msg_over
                  && (enq_off == SLOT_BYTES[SBW-1:0]);
    // 消息总长：普通 eop = enq_cnt+1；单字节消息（sop+eop 同拍）enq_cnt 尚为
    // 上一条的残值（NBA），必须按 1 计——T6-005 对拍实证
    wire [13:0] msg_total = do_sop ? 14'd1 : (enq_cnt + 14'd1);
    // eop 检查：总长上限 + Smax（应用载荷层级，C 组定案）
    wire len_ok = !enq_discard
              && (msg_total <= MAX_MSG[13:0])
              && ((msg_total <= MAX_PAYLOAD[13:0])
                  ? (msg_total <= smax_r[cur_vl])
                  : (smax_r[cur_vl] >= FRAG_MID[15:0]));
    // 链占用 slot 数 = ceil(已写字节 / SLOT_BYTES)；disc 时 enq_cnt 即已写字节
    wire [13:0] written = enq_discard ? enq_cnt : msg_total;
    wire [14:0] chain_ns = (({1'b0, written} - 15'd1) >> SBW) + 15'd1;
    wire [2:0]  chain_ns3 = (chain_ns > 15'd4) ? 3'd4 : chain_ns[2:0];

    // slot RAM 写口（每传输字节一写；sop/跨 slot 写新槽 0 号偏移；disc 不写）
    wire [SLW+SBW-1:0] slot_wa = (do_sop || slot_wrap)
                               ? {slot_alloc, {SBW{1'b0}}}
                               : {enq_cur, enq_off};
    wire slot_we = do_byt && !msg_over;

    // 描述符 RAM 写（eop 接受时写队尾）
    wire [8:0]  desc_wa = {cur_vl, q_tail[cur_vl]};
    wire [39:0] desc_wd = {chain_head, msg_total, ipid_r[cur_vl], 4'b0000};
    wire        desc_we = do_eop && len_ok;

    // 释放请求源（REV-002-03）：abort 半截丢弃 / eop 拒收 / 最后一帧弹出。
    // 三源同拍并发全部入队（不再单选/覆盖），由请求队列按序行走。
    wire abort_now = htx_abort && enq_active;
    wire [14:0] abort_ns = enq_cnt == 14'd0 ? 15'd1
                             : ((({1'b0, enq_cnt} - 15'd1) >> SBW) + 15'd1);
    wire [14:0] frag_ns = (({1'b0, frag_len} - 15'd1) >> SBW) + 15'd1;
    wire        rel_ab  = abort_now;
    wire        rel_rj  = do_eop && !len_ok;
    wire        rel_fr  = frag_upd && frag_last;
    wire [SLW-1:0] rel_ab_slot = enq_head;
    wire [SLW-1:0] rel_rj_slot = chain_head;
    wire [2:0]  rel_ab_len = (abort_ns > 15'd4) ? 3'd4 : abort_ns[2:0];
    wire [2:0]  rel_rj_len = chain_ns3;
    wire [2:0]  rel_fr_len = (frag_ns > 15'd4) ? 3'd4 : frag_ns[2:0];
    wire        pop_now    = !walk_busy && ((rq_cnt != 4'd0) || rel_ab || rel_rj || rel_fr);
    // 队首旁路：队列空且本拍有新请求时直接取第一推送项（0 拍延迟，同旧行为）
    wire [SLW-1:0] rq_head_slot = (rq_cnt == 4'd0)
                                ? (rel_ab ? rel_ab_slot : (rel_rj ? rel_rj_slot : frag_slot0))
                                : rq_slot[rq_rp];
    wire [2:0]  rq_head_len  = (rq_cnt == 4'd0)
                                ? (rel_ab ? rel_ab_len : (rel_rj ? rel_rj_len : rel_fr_len))
                                : rq_len[rq_rp];
    reg  [2:0]  rq_push_n;                                  // 本拍推送条数（rq_push 块算）

    //-------------------------------------------------------------------------
    // slot 链与 slot RAM
    //-------------------------------------------------------------------------
    reg [SLW-1:0] nx [0:TX_SLOTS-1];      // next_ptr（组合读）
    assign nx_rd = nx[nx_ra];

    sdp_ram #(
        .WIDTH(8), .DEPTH(TX_SLOTS * SLOT_BYTES), .AW(SLW + SBW)
    ) u_slot_ram (
        .clk(clk), .we(slot_we), .waddr(slot_wa), .wdata(htx_data),
        .raddr(pre_addr), .rdata(pre_data)
    );

    //-------------------------------------------------------------------------
    // 队列非空位图（调度器 eligible 用）
    //-------------------------------------------------------------------------
    genvar gi;
    generate
        for (gi = 0; gi < TX_VL_NUM; gi = gi + 1) begin: g_qne
            assign q_nonempty[gi] = (q_cnt[gi] != 0);
        end
    endgenerate

    //-------------------------------------------------------------------------
    // Smax 配置写透传（W1）
    //-------------------------------------------------------------------------
    integer si;
    always @(posedge clk) begin
        if (reset) begin
            for (si = 0; si < TX_VL_NUM; si = si + 1)
                smax_r[si] <= 16'hFFFF;
        end else if (cw_we && cw_wsel == 3'd1)
            smax_r[cw_idx] <= cw_wdata[15:0];
    end

    //-------------------------------------------------------------------------
    // 主时序
    //-------------------------------------------------------------------------
    integer ri;
    always @(posedge clk) begin
        if (reset) begin
            enq_active  <= 1'b0;
            enq_cnt     <= 14'd0;
            enq_discard <= 1'b0;   // 复位补全（FIX2 附带）：缺失会使复位后首条
                                   // 单字节消息（sop+eop 同拍）len_ok=X 被静默
                                   // 丢弃并泄漏 1 槽——多字节首消息会自愈故 tx1
                                   // 未暴露，txsn（首条 1B）必命中
            qfull_latch <= 1'b0;
            walk_busy   <= 1'b0;
            rq_wp       <= 3'd0;
            rq_rp       <= 3'd0;
            rq_cnt      <= 4'd0;
            ev_enq      <= 1'b0;
            ev_qfull    <= 1'b0;
            ev_lenerr   <= 1'b0;
            ev_frame    <= 1'b0;
            free_bits   <= 32'hFFFFFFFF;
            free_cnt    <= TX_SLOTS[5:0];
            for (ri = 0; ri < TX_VL_NUM; ri = ri + 1) begin
                q_head[ri] <= {QW{1'b0}};
                q_tail[ri] <= {QW{1'b0}};
                q_cnt[ri]  <= {CW{1'b0}};
                ipid_r[ri] <= 16'd1;               // 消息级 IP ID 从 1 起（gen_stim 同）
                fidx_r[ri] <= 4'd0;
                // smax_r 复位统一由上方"配置写透传"块驱动（同值 16'hFFFF）；
                // 此处副本会造成多驱动，ehiway_map 拒绝（iverilog 同值双驱动
                // 仿真无感，综合非法），故删除
            end
        end else begin
            //------------------------------------------------------------------
            // 事件脉冲默认值
            //------------------------------------------------------------------
            ev_enq    <= 1'b0;
            ev_qfull  <= 1'b0;
            ev_lenerr <= 1'b0;
            ev_frame  <= 1'b0;

            //------------------------------------------------------------------
            // sop 拒绝（队列满/槽不足）：单拍事件，latch 去重（valid 保持只计一次）
            //------------------------------------------------------------------
            if (htx_valid && htx_sop && !room && !enq_active && !qfull_latch) begin
                ev_qfull    <= 1'b1;
                qfull_latch <= 1'b1;
            end
            if (!htx_valid)
                qfull_latch <= 1'b0;

            //------------------------------------------------------------------
            // 字节接收（含 sop 字节；slot RAM 写口见组合段）
            //------------------------------------------------------------------
            if (do_sop) begin
                enq_active  <= 1'b1;
                enq_vl      <= htx_vl;
                enq_head    <= slot_alloc;
                enq_cur     <= slot_alloc;
                enq_off     <= {{(SBW-1){1'b0}}, 1'b1};
                enq_cnt     <= 14'd1;                   // sop 字节计入
                enq_discard <= 1'b0;
                free_bits[slot_alloc] <= 1'b0;          // 分配：清空闲位
            end else if (do_byt && !enq_discard) begin
                if (msg_over) begin
                    enq_discard <= 1'b1;                // 起超限：停写（链=4 槽整）
                end else begin
                    enq_cnt <= enq_cnt + 14'd1;
                    if (slot_wrap) begin                // 跨 slot：链入下一槽
                        nx[enq_cur] <= slot_alloc;
                        enq_cur     <= slot_alloc;
                        enq_off     <= {{(SBW-1){1'b0}}, 1'b1};
                        free_bits[slot_alloc] <= 1'b0;  // 分配：清空闲位
                    end else begin
                        enq_off <= enq_off + {{(SBW-1){1'b0}}, 1'b1};
                    end
                end
            end

            //------------------------------------------------------------------
            // abort：半截入队消息整条丢弃（槽经行走回收，无描述符）
            //------------------------------------------------------------------
            if (abort_now) begin
                enq_active  <= 1'b0;
                enq_discard <= 1'b0;
                ev_qfull    <= 1'b1;           // 计入"消息被拒"口径
            end

            //------------------------------------------------------------------
            // eop：检查 + 描述符入环 / 拒收释放（q_cnt 净增量见下方单点更新）
            //------------------------------------------------------------------
            if (do_eop) begin
                enq_active  <= 1'b0;
                enq_discard <= 1'b0;
                if (len_ok) begin
                    ev_enq         <= 1'b1;
                    q_tail[cur_vl] <= q_tail[cur_vl] + {{(QW-1){1'b0}}, 1'b1};
                    ipid_r[cur_vl] <= ipid_r[cur_vl] + 16'd1;   // 每消息 +1
                end else begin
                    ev_lenerr <= 1'b1;              // 长度/Smax 违规：整条丢弃
                end
            end

            //------------------------------------------------------------------
            // 分片推进 / 最后一帧弹出（q_cnt 净增量见下方单点更新）
            //------------------------------------------------------------------
            if (frag_upd) begin
                if (frag_last) begin
                    ev_frame        <= 1'b1;
                    q_head[frag_vl] <= q_head[frag_vl] + {{(QW-1){1'b0}}, 1'b1};
                    fidx_r[frag_vl] <= 4'd0;            // 消息生命周期结束，游标归零
                end else begin
                    fidx_r[frag_vl] <= fidx_r[frag_vl] + 4'd1;   // 下一片同 IP ID
                end
            end

            //------------------------------------------------------------------
            // q_cnt 净增量单点更新（REV-002-02）：同拍同 VL 入队(+1)/末帧
            // 弹出(-1) 相抵为 0——两处独立赋值会同拍相互覆盖造成少计
            //------------------------------------------------------------------
            if (do_eop && len_ok && frag_upd && frag_last) begin
                if (cur_vl != frag_vl) begin
                    q_cnt[cur_vl]  <= q_cnt[cur_vl]  + {{(CW-1){1'b0}}, 1'b1};
                    q_cnt[frag_vl] <= q_cnt[frag_vl] - {{(CW-1){1'b0}}, 1'b1};
                end
                // 同 VL 同拍：净变化 0（队尾入环/队首弹出已各自推进）
            end else if (do_eop && len_ok) begin
                q_cnt[cur_vl] <= q_cnt[cur_vl] + {{(CW-1){1'b0}}, 1'b1};
            end else if (frag_upd && frag_last) begin
                q_cnt[frag_vl] <= q_cnt[frag_vl] - {{(CW-1){1'b0}}, 1'b1};
            end

            //------------------------------------------------------------------
            // 释放请求入队（REV-002-03：同拍多源全部入队，不选不弃）
            //------------------------------------------------------------------
            begin : rq_push
                reg [2:0] wp_t;
                wp_t = rq_wp;
                if (rel_ab) begin
                    rq_slot[wp_t] <= rel_ab_slot;
                    rq_len [wp_t] <= rel_ab_len;
                    wp_t = wp_t + 3'd1;
                end
                if (rel_rj) begin
                    rq_slot[wp_t] <= rel_rj_slot;
                    rq_len [wp_t] <= rel_rj_len;
                    wp_t = wp_t + 3'd1;
                end
                if (rel_fr) begin
                    rq_slot[wp_t] <= frag_slot0;
                    rq_len [wp_t] <= rel_fr_len;
                    wp_t = wp_t + 3'd1;
                end
                if (wp_t != rq_wp)
                    rq_wp <= wp_t;
                rq_push_n = wp_t - rq_wp;           // 本拍推送条数（mod 8）
            end

            //------------------------------------------------------------------
            // 释放行走：空闲即取队首请求；行走中每拍释放一槽沿链推进。
            // rq_cnt 单点更新（入队 +n 与出队 -1 同拍相抵，两处赋值会互相覆盖）
            //------------------------------------------------------------------
            if (pop_now) begin
                walk_busy <= 1'b1;
                walk_cur  <= rq_head_slot;
                walk_left <= rq_head_len;
                rq_rp     <= rq_rp + 3'd1;
            end else if (walk_busy) begin
                free_bits[walk_cur] <= 1'b1;
                if (walk_left == 3'd1)
                    walk_busy <= 1'b0;
                else begin
                    walk_cur  <= nx[walk_cur];
                    walk_left <= walk_left - 3'd1;
                end
            end
            rq_cnt <= rq_cnt + {2'b0, rq_push_n} - {3'b0, pop_now};

            //------------------------------------------------------------------
            // 空闲计数（分配/释放同拍相抵；单点赋值）：本拍释放 = walk_busy（旧值）
            //------------------------------------------------------------------
            free_cnt <= free_cnt
                      + {5'b0, walk_busy}
                      - {5'b0, (do_sop || slot_wrap)};
        end
    end

    //-------------------------------------------------------------------------
    // 描述符读口（组帧器；读地址用寄存后的 vl + 当拍队首，1 拍数据延迟）
    //-------------------------------------------------------------------------
    sdp_ram #(
        .WIDTH(40), .DEPTH(TX_VL_NUM * Q_DEPTH), .AW(VLW + QW)
    ) u_desc_ram (
        .clk(clk), .we(desc_we), .waddr(desc_wa), .wdata(desc_wd),
        .raddr({desc_vl_r, q_head[desc_vl_r]}), .rdata(desc_rdata)
    );

    always @(posedge clk) begin
        if (reset) begin
            desc_vld_r <= 1'b0;
            desc_vld2  <= 1'b0;
        end else begin
            desc_vld_r <= desc_req;
            desc_vld2  <= desc_vld_r;
            desc_vl_r  <= desc_vl;
        end
    end

    assign desc_vld  = desc_vld2;
    assign desc_slot = desc_rdata[39:34];
    assign desc_len  = desc_rdata[33:20];
    assign desc_ipid = desc_rdata[19:4];
    assign desc_fidx = fidx_r[desc_vl_r];     // 影子寄存器组合读

endmodule
