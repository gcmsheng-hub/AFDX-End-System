//=============================================================================
// cfg_bus.v — 配置总线译码与读回多路选择（v2 §4.2 core；接口 v2 §4.3 ①）
//
// 地址映射（v2 §4.3 ①，T6/T7 落地范围）：
//   0x00000~0x00FFF 全局寄存器（本模块内置）：
//     0x00000 VERSION(ro) 0x00001 UID[15:0](rw) 0x00002 SRC_IP[31:0](rw)
//     0x00003 CTRL(rw, bit0=tx_en) 0x00004 STATUS(ro, bit0=init_done)
//     0x00005 CFG_REJ_CNT(ro, VL 表非法写拒收计数)
//     0x00006 CFG_REJ_FLAG(ro, 拒收原因粘滞: bit0=BAG 非法 bit1=net_mask=00)
//   0x01000~0x01FFF 发送 VL 表 128×8 字（写广播 cw_*；读回经表 RAM 口 A）
//   0x10000~0x17FFF 接收 VL 表 4096×2 字（T7：写广播 rxt_*；读回经表读口，
//                   地址 = 0x10000 + idx*2 + w，bit0 = W0/W1 字选）
//   0x20000~0x2FFFF 统计（读回经 stats 读口；地址低 10 位 = {vl[6:0], sel[1:0]}）
//   其余区间（端口 CAM/管理平面）P1 落地，现读回 0。
//
// VL 表写门禁（VERDICT-007 FIX2 #5/#7，doc/02 §8.2 增补第 7 条）：
//   W0 bag_ms 仅接受 {1,2,4,8,16,32,64,128}（8b 内 2 的幂即为该集合）；
//   W3 net_mask[1:0]=00（A/B 均不选）拒绝。非法写在 cfg_bus 截下：cw_we 不发出，
//   表/regulator/buf_mgr 均不可见（读回保持旧值），计数与原因置位供 tb 断言。
//
// 读时序：cfg_re 当拍锁存地址，下一拍 cfg_rvalid=1 且 cfg_rdata 有效（1 拍延迟，
// 与 v2 §4.3 ① 冻结语义一致）。RAM 区读数据来自 RAM 读出（同为 1 拍）。
//=============================================================================

module cfg_bus #(
    parameter [31:0] VERSION = 32'h20261031
)(
    input  wire        clk,
    input  wire        reset,

    // ---- 配置总线主侧（tb / 管理平面）----
    input  wire [19:0] cfg_addr,
    input  wire [31:0] cfg_wdata,
    input  wire        cfg_we,
    input  wire        cfg_re,
    output wire [31:0] cfg_rdata,
    output wire        cfg_rvalid,

    // ---- 全局寄存器值（内核直用）----
    output wire [15:0] g_uid,
    output wire [31:0] g_src_ip,
    output wire        g_tx_en,
    input  wire        init_done,

    // ---- 发送 VL 表写广播（0x01000 区写；tx_vl_table / regulator / buf_mgr 消费）----
    output wire        cw_we,
    output wire [6:0]  cw_idx,
    output wire [2:0]  cw_wsel,
    output wire [31:0] cw_wdata,

    // ---- 发送 VL 表 cfg 读（口 A 分时；tbl_busy=1 期间读回值可能为旧值，不干扰内核读）----
    output wire        tbl_re,
    output wire [9:0]  tbl_raddr,
    input  wire [31:0] tbl_rdata,
    input  wire        tbl_busy,

    // ---- 接收 VL 表写广播 + cfg 读（T7；读口内核查表优先，配置期无帧在途）----
    output wire        rxt_we,
    output wire [10:0] rxt_idx,
    output wire        rxt_wsel,
    output wire [31:0] rxt_wdata,
    output wire        rxt_re,
    output wire [10:0] rxt_ridx,
    output wire        rxt_rsel,
    input  wire [31:0] rxt_rdata,

    // ---- 统计读回 ----
    output wire        stats_re,
    output wire [9:0]  stats_raddr,
    input  wire [31:0] stats_rdata
);

    //-------------------------------------------------------------------------
    // 全局寄存器
    //-------------------------------------------------------------------------
    reg [15:0] uid_r;
    reg [31:0] src_ip_r;
    reg        tx_en_r;

    wire [7:0] region  = cfg_addr[19:12];
    wire       wr_glob = cfg_we && (region == 8'h00);
    wire       wr_tbl  = cfg_we && (region == 8'h01);
    wire       wr_rxt  = cfg_we && (region == 8'h10);   // 接收 VL 表（T7）

    // ---- VL 表写门禁（FIX2 #5/#7）----------------------------------------
    // bag_ms ∈ {1,2,4,8,16,32,64,128} ⇔ 8 位非零且为 2 的幂
    wire bag_illegal = wr_tbl && (cfg_addr[2:0] == 3'd0)
                    && ((cfg_wdata[7:0] == 8'h00)
                        || ((cfg_wdata[7:0] & (cfg_wdata[7:0] - 8'd1)) != 8'h00));
    wire mask_illegal = wr_tbl && (cfg_addr[2:0] == 3'd3)
                     && (cfg_wdata[1:0] == 2'b00);
    wire rej_evt      = bag_illegal || mask_illegal;

    assign cw_we   = wr_tbl && !rej_evt;    // 非法写：广播不发，表保持旧值
    assign cw_idx  = cfg_addr[9:3];
    assign cw_wsel = cfg_addr[2:0];
    assign cw_wdata = cfg_wdata;

    // 接收 VL 表广播（T7；无写门禁——en=0 项天然为未配置，mask=00 仅使 VL 永不匹配）
    assign rxt_we    = wr_rxt;
    assign rxt_idx   = cfg_addr[11:1];      // 地址 = 0x10000 + idx*2 + w
    assign rxt_wsel  = cfg_addr[0];
    assign rxt_wdata = cfg_wdata;

    reg  [31:0] rej_cnt;                    // 拒收累计（0x00005，ro）
    reg  [1:0]  rej_flag;                   // 原因粘滞（0x00006，ro）
    always @(posedge clk) begin
        if (reset) begin
            rej_cnt  <= 32'd0;
            rej_flag <= 2'b00;
        end else if (rej_evt) begin
            rej_cnt <= rej_cnt + 32'd1;
            if (bag_illegal)
                rej_flag[0] <= 1'b1;
            if (mask_illegal)
                rej_flag[1] <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            uid_r    <= 16'h0000;
            src_ip_r <= 32'h00000000;
            tx_en_r  <= 1'b0;
        end else begin
            if (wr_glob) begin
                case (cfg_addr[11:0])
                    12'h001: uid_r    <= cfg_wdata[15:0];
                    12'h002: src_ip_r <= cfg_wdata;
                    12'h003: tx_en_r  <= cfg_wdata[0];
                    default: ;
                endcase
            end
        end
    end

    assign g_uid    = uid_r;
    assign g_src_ip = src_ip_r;
    assign g_tx_en  = tx_en_r;

    //-------------------------------------------------------------------------
    // 读路径：re 拍锁存地址 → 下一拍 rvalid + rdata（组合多路选 RAM/寄存器）
    //-------------------------------------------------------------------------
    reg [19:0] raddr_r;
    reg        rvalid_r;

    always @(posedge clk) begin
        if (reset) begin
            rvalid_r <= 1'b0;
            raddr_r  <= 20'h0;
        end else begin
            rvalid_r <= cfg_re;
            raddr_r  <= cfg_addr;
        end
    end

    assign tbl_re    = cfg_re && (region == 8'h01);   // busy 让位由表内裁决
    assign tbl_raddr = cfg_addr[9:0];
    assign rxt_re    = cfg_re && (region == 8'h10);   // 内核查表拍让位（rx_vl_lookup 内）
    assign rxt_ridx  = cfg_addr[11:1];
    assign rxt_rsel  = cfg_addr[0];
    assign stats_re  = cfg_re && (region == 8'h20);
    assign stats_raddr = cfg_addr[9:0];

    reg [31:0] rdata_mux;
    always @(*) begin
        case (raddr_r[19:12])
            8'h00: begin
                case (raddr_r[11:0])
                    12'h000: rdata_mux = VERSION;
                    12'h001: rdata_mux = {16'h0000, uid_r};
                    12'h002: rdata_mux = src_ip_r;
                    12'h003: rdata_mux = {31'h0, tx_en_r};
                    12'h004: rdata_mux = {31'h0, init_done};
                    12'h005: rdata_mux = rej_cnt;
                    12'h006: rdata_mux = {30'h0, rej_flag};
                    default: rdata_mux = 32'h0;
                endcase
            end
            8'h01:   rdata_mux = tbl_rdata;
            8'h10:   rdata_mux = rxt_rdata;
            8'h20:   rdata_mux = stats_rdata;
            default: rdata_mux = 32'h0;
        endcase
    end

    assign cfg_rdata  = rdata_mux;
    assign cfg_rvalid = rvalid_r;

endmodule
