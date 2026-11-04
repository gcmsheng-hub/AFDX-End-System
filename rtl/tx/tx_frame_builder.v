//=============================================================================
// tx_frame_builder.v — 组帧器（v2 §4.2 tx；doc/01 §1 帧格式）
//
// 每任务组一帧：读发送 VL 表（W2 VLID / W3 mask / W5 sport / W6 dport）+
// buf_mgr 队首描述符（slot0/总长/ip_id/片号）+ tx_sn_gen 取 SN → 计算
// 分片几何与 IP 头校验和 → 经 s_stream（v2 §4.3 ④）逐字节发出帧体
// （目的 MAC..SN，不含 FCS——FCS 由 mac_tx 按网各自计算追加，IF_ID 字节
// 由 mac_tx 按网替换，此处源 MAC 第 6 字节恒填 A 网 0x20 占位）。
//
// 分片几何（与 scripts/afdx_ref.py message_to_frames 一致，改动须两处同步）：
//   total ≤ 1471：单帧 L=total, off=0, mf=0, 含 UDP 头（UDP Length = 8+total）。
//   否则片 k(=fidx)：off = 1456+(k-1)*1464（8B 对齐），L = min(1464, total−off)，
//   mf = (off+L < total)；首片含 UDP 头且 UDP Length = 8+消息总长（ISS-06），
//   后续片为纯 IP 分片。
//   ip_total = 20 + (首片 ? 8+L : L)；pad = max(0, 45−ip_total)（IP 包后、SN 前）；
//   body_len = 14 + ip_total + pad + 1（≤1514，加 FCS 4B ≤ 1518）。
//
// s_stream 无气泡：s_data 由字节计数 cnt 组合选择（头部寄存器直出 / 载荷直读 /
// Padding 0 / SN）；载荷读流水——发第 k 字节当拍 pay 寄存器指向第 k+1 字节地址
// （sdp 1 拍读延迟正好对齐），跨 slot 边界经 next_ptr 组合跟链；
// cnt 在 s_valid&&s_ready 消费拍推进（mac 未就绪则停等，无字节丢失）。
// frame_ready 在流开始前单拍发出（调度器据此同拍启动 A/B 两网 mac 锁步消费）。
//=============================================================================

module tx_frame_builder #(
    parameter [7:0] DST_MAC_C0 = 8'h03,   // DST_MAC_CONST（doc/01 §4）
    parameter [7:0] DST_MAC_C1 = 8'h00,
    parameter [7:0] DST_MAC_C2 = 8'h00,
    parameter [7:0] DST_MAC_C3 = 8'h00,
    parameter [7:0] TTL_DEF    = 8'd64,   // doc/01 §4 参数
    parameter [7:0] SRC_MAC_C0 = 8'h02,   // 源 MAC 前 3 字节常量
    parameter [7:0] SRC_MAC_C1 = 8'h00,
    parameter [7:0] SRC_MAC_C2 = 8'h00
)(
    input  wire        clk,
    input  wire        reset,

    // 调度器握手
    input  wire        task_start,         // 单拍
    input  wire [6:0]  task_vl,
    output reg         bld_busy,
    output reg         frame_ready,        // 单拍（流开始前）
    output reg  [10:0] fr_body_len,
    output reg  [13:0] fr_paylen,
    output reg  [1:0]  fr_mask,            // bit0=A bit1=B

    // 发送 VL 表读口（1 拍）
    output reg  [6:0]  rd_idx,
    output reg  [2:0]  rd_wsel,
    input  wire [31:0] rd_data,

    // buf_mgr：描述符 / 载荷走线 / 分片推进
    output reg         desc_req,
    output reg  [6:0]  desc_vl,
    input  wire        desc_vld,
    input  wire [5:0]  desc_slot,
    input  wire [13:0] desc_len,
    input  wire [15:0] desc_ipid,
    input  wire [3:0]  desc_fidx,
    output wire [16:0] pre_addr,           // {slot, off}
    input  wire [7:0]  pre_data,
    output wire [5:0]  nx_ra,
    input  wire [5:0]  nx_rd,
    output reg         frag_upd,           // 单拍：本帧完成
    output reg         frag_last,          // 本帧为消息最后一帧
    output reg  [6:0]  frag_vl,
    output reg  [5:0]  frag_slot0,
    output reg  [13:0] frag_len,           // 消息总长（buf_mgr 释放链用）

    // tx_sn_gen
    output reg         sn_re,
    output reg  [6:0]  sn_vl,
    input  wire        sn_vld,
    input  wire [7:0]  sn_val,

    // 全局寄存器直读线（cfg_bus）
    input  wire [15:0] g_uid,
    input  wire [31:0] g_src_ip,

    // 帧内部流（v2 §4.3 ④；两网 mac 锁步消费）
    output reg         s_valid,
    output wire        s_sop,
    output wire        s_eop,
    output wire [7:0]  s_data,
    input  wire        s_ready
);

    //-------------------------------------------------------------------------
    // 常量（与 scripts/afdx_ref.py 对齐，改动须两处同步）
    //-------------------------------------------------------------------------
    localparam integer MAX_PAYLOAD = 1471;
    localparam integer FRAG_FIRST  = ((MAX_PAYLOAD - 8) / 8) * 8;   // 1456
    localparam integer FRAG_MID    = MAX_PAYLOAD & ~7;              // 1464

    //-------------------------------------------------------------------------
    // 状态
    //-------------------------------------------------------------------------
    localparam [3:0] S_IDLE = 4'd0, S_W3 = 4'd1, S_W5 = 4'd2, S_W6 = 4'd3,
                     S_DESC = 4'd4, S_DESCW = 4'd5, S_SNW = 4'd6, S_CALC = 4'd7,
                     S_ARM = 4'd8, S_RDYW = 4'd9, S_STREAM = 4'd10, S_DONE = 4'd11;

    reg [3:0] state;

    // 捕获的任务字段
    reg [15:0] vlid_r, sport_r, dport_r;
    reg [1:0]  mask_r;
    reg [5:0]  slot0_r;
    reg [13:0] total_r;
    reg [15:0] ipid_r;
    reg [3:0]  fidx_r;
    reg [7:0]  sn_r;

    // 几何寄存器（S_CALC 锁存）
    reg [13:0] L_r;            // 本帧 UDP 载荷字节数
    reg        mf_r, first_r;
    reg [13:0] ip_total_r;
    reg [4:0]  pad_r;          // max(0, 45−ip_total) ≤ 16
    reg [10:0] body_len_r;
    reg [10:0] hdr_total_r;    // 14+20+(首片?8:0) = 42 / 34

    // 头部寄存器（发帧期间组合切片直出）
    reg [111:0] mac_hdr;       // 14B
    reg [159:0] ip_hdr;        // 20B
    reg [63:0]  udp_hdr;       // 8B（仅首片发出）

    // 发送计数与载荷读位置
    reg [10:0] cnt;            // 下一个待发字节索引（消费拍推进）
    reg [10:0] pay_off;        // 载荷读位置：slot 内偏移（指向下一载荷字节）
    reg [5:0]  pay_slot;       // 载荷读位置：slot
    reg [2:0]  arm_steps;      // 分片起始地址预走链步数（frag_off>>11，≤7）

    //-------------------------------------------------------------------------
    // 几何组合（S_CALC 拍锁存；字段来自 S_DESCW 捕获）
    //-------------------------------------------------------------------------
    wire        is_frag_msg = (total_r > MAX_PAYLOAD[13:0]);
    wire [13:0] frag_off    = (fidx_r == 4'd0)
                              ? 14'd0
                              : (FRAG_FIRST[13:0] + (fidx_r - 4'd1) * FRAG_MID[13:0]);
    wire [13:0] geo_L       = is_frag_msg
                              ? ((fidx_r == 4'd0)
                                 ? FRAG_FIRST[13:0]                       // 首片恒 1456
                                 : ((total_r - frag_off > FRAG_MID[13:0])
                                    ? FRAG_MID[13:0] : (total_r - frag_off)))
                              : total_r;
    wire        geo_mf      = is_frag_msg && (frag_off + geo_L < total_r);
    wire        geo_first   = !is_frag_msg || (fidx_r == 4'd0);
    wire [13:0] geo_iptotal = 14'd20 + (geo_first ? (14'd8 + geo_L) : geo_L);
    wire [4:0]  geo_pad     = (geo_iptotal < 14'd45) ? (14'd45 - geo_iptotal) : 5'd0;
    wire [10:0] geo_bodylen = 11'd14 + {4'b0, geo_iptotal} + {6'b0, geo_pad} + 11'd1;

    // 目的 IP = 224.224.VL[15:8].VL[7:0]（doc/01 §1 / P7-44 图3-38）
    wire [31:0] dst_ip_w = {8'd224, 8'd224, vlid_r[15:8], vlid_r[7:0]};

    // IP 头 16 位字（BE；cw5 = 校验和字段置 0）
    wire [15:0] cw0 = 16'h4500;
    wire [15:0] cw1 = {2'b00, geo_iptotal};
    wire [15:0] cw2 = ipid_r;
    // 标志/片偏移 16b：bit15=保留(0), bit14=DF(0), bit13=MF, bits12:0=偏移(8B 单位)
    // （MF=0x2000，与 afdx_ref 的 `mf = 0x2000` 一致；frag_off[13:3] 为 11 位，
    //   高 2 位补零凑 13 位偏移字段——8192B 消息最大偏移 2012 < 2048）
    wire [15:0] cw3 = {1'b0, 1'b0, geo_mf, 2'b00, frag_off[13:3]};
    wire [15:0] cw4 = {TTL_DEF, 8'h11};
    wire [15:0] cw5 = 16'h0000;
    wire [15:0] cw6 = g_src_ip[31:16];
    wire [15:0] cw7 = g_src_ip[15:0];
    wire [15:0] cw8 = dst_ip_w[31:16];
    wire [15:0] cw9 = dst_ip_w[15:0];
    // 反码求和两次折叠（csum_sum ≤ 10×0xFFFF < 0x100000，两次足够）
    wire [19:0] csum_sum = cw0+cw1+cw2+cw3+cw4+cw5+cw6+cw7+cw8+cw9;
    wire [16:0] csum_f1  = {1'b0, csum_sum[15:0]} + {13'b0, csum_sum[19:16]};
    wire [16:0] csum_f2  = {1'b0, csum_f1[15:0]} + {16'b0, csum_f1[16]};
    wire [15:0] ip_csum_w = ~csum_f2[15:0];   // f2 ≤ 0xFFFF（f1 ≤ 0x10008）

    // UDP Length = 8 + 消息总长（ISS-06）
    wire [15:0] udp_len_w = 16'd8 + {2'b0, total_r};

    //-------------------------------------------------------------------------
    // 发送字节组合选择 + 载荷读地址
    //-------------------------------------------------------------------------
    // 帧体布局：[MAC 14][IP 20][UDP 8 若首片][载荷 L][pad][SN]
    reg [7:0] emit_byte;
    always @(*) begin
        if (cnt < 11'd14)
            emit_byte = mac_hdr[111 - cnt*8 -: 8];
        else if (cnt < 11'd34)
            emit_byte = ip_hdr[159 - (cnt-11'd14)*8 -: 8];
        else if (first_r && (cnt < 11'd42))
            emit_byte = udp_hdr[63 - (cnt-11'd34)*8 -: 8];
        else if (cnt < hdr_total_r + L_r)
            emit_byte = pre_data;                       // 载荷直出（读流水对齐）
        else if (cnt < hdr_total_r + L_r + {7'b0, pad_r})
            emit_byte = 8'h00;                          // Padding
        else
            emit_byte = sn_r;                           // SN（末字节）
    end

    assign s_data = emit_byte;
    assign s_sop  = (cnt == 11'd0);
    assign s_eop  = (cnt == body_len_r - 11'd1);

    // 载荷读位置寄存器指向"下一载荷字节"地址；消费拍推进（含跨 slot 跟链）
    assign nx_ra    = pay_slot;
    assign pre_addr = {pay_slot, pay_off};

    wire consuming = s_valid && s_ready;                       // 本拍字节被消费
    // 读流水不变式：regs 在 E_k 拍指向第 k+1 字节地址（rdata 滞后一拍对齐）。
    // 因此"消费第 j 字节"时须推进到 j+2 的地址——包括最后一个头字节（j=hdr_last，
    // 其下一拍即载荷 0，若不推进则载荷首字节重复——S4/T6-005 调试实证）。
    wire next_is_pay  = (cnt + 11'd1 >= hdr_total_r) &&
                        (cnt + 11'd1 <  hdr_total_r + L_r);
    wire cur_is_pay   = (cnt >= hdr_total_r) && (cnt < hdr_total_r + L_r);
    wire adv_pay      = next_is_pay && (cur_is_pay || (cnt == hdr_total_r - 11'd1));

    //-------------------------------------------------------------------------
    // 主时序
    //-------------------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            state <= S_IDLE; bld_busy <= 1'b0; frame_ready <= 1'b0;
            s_valid <= 1'b0;
            frag_upd <= 1'b0; frag_last <= 1'b0; frag_vl <= 0;
            frag_slot0 <= 0; frag_len <= 0;
            sn_re <= 1'b0; desc_req <= 1'b0;
            fr_body_len <= 0; fr_paylen <= 0; fr_mask <= 0;
            cnt <= 0; pay_off <= 0; pay_slot <= 0;
        end else begin
            // 单拍默认
            frame_ready <= 1'b0;
            frag_upd    <= 1'b0;
            frag_last   <= 1'b0;
            sn_re       <= 1'b0;
            desc_req    <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (task_start) begin
                        bld_busy <= 1'b1;
                        rd_idx   <= task_vl;
                        rd_wsel  <= 3'd2;      // W2 地址本拍给出，下拍数据有效
                        state    <= S_W3;
                    end
                end
                // 表读流水：本拍给出的地址，数据在下下拍状态捕获
                // （IDLE 设 wsel=2 → S_W3 拍读 W2 → S_W5 拍捕获 W2 并设 wsel=3…）
                S_W3: begin
                    rd_wsel <= 3'd3;           // 读 W3
                    state   <= S_W5;
                end
                S_W5: begin
                    vlid_r  <= rd_data[15:0];  // W2（上拍读出）
                    rd_wsel <= 3'd5;           // 读 W5
                    state   <= S_W6;
                end
                S_W6: begin
                    mask_r  <= rd_data[1:0];   // W3
                    rd_wsel <= 3'd6;           // 读 W6
                    state   <= S_DESC;
                end
                S_DESC: begin
                    sport_r <= rd_data[15:0];  // W5（上拍读出）
                    desc_req <= 1'b1;
                    desc_vl  <= task_vl;
                    state    <= S_DESCW;
                end
                S_DESCW: begin
                    dport_r <= rd_data[15:0];      // W6（上拍读出）
                    if (desc_vld) begin
                        slot0_r <= desc_slot;
                        total_r <= desc_len;
                        ipid_r  <= desc_ipid;
                        fidx_r  <= desc_fidx;
                        sn_re   <= 1'b1;       // 同拍发起 SN 读
                        sn_vl   <= task_vl;
                        state   <= S_SNW;
                    end
                end
                S_SNW: begin
                    if (sn_vld) begin
                        sn_r  <= sn_val;
                        state <= S_CALC;
                    end
                end
                S_CALC: begin
                    L_r         <= geo_L;
                    mf_r        <= geo_mf;
                    first_r     <= geo_first;
                    ip_total_r  <= geo_iptotal;
                    pad_r       <= geo_pad;
                    body_len_r  <= geo_bodylen;
                    hdr_total_r <= geo_first ? 11'd42 : 11'd34;
                    mac_hdr <= {DST_MAC_C0, DST_MAC_C1, DST_MAC_C2, DST_MAC_C3,
                                vlid_r[15:8], vlid_r[7:0],
                                SRC_MAC_C0, SRC_MAC_C1, SRC_MAC_C2,
                                g_uid[15:8], g_uid[7:0],
                                8'h20 /*IF_ID 占位：mac_tx 按网替换*/,
                                8'h08, 8'h00};
                    ip_hdr <= {cw0, cw1, cw2, cw3, cw4, ip_csum_w,
                               cw6, cw7, cw8, cw9};
                    udp_hdr <= {sport_r, dport_r, udp_len_w, 16'h0000};
                    frame_ready <= 1'b1;
                    fr_body_len <= geo_bodylen;
                    fr_paylen   <= geo_L;
                    fr_mask     <= mask_r;
                    cnt         <= 11'd0;
                    pay_off     <= 11'd0;
                    pay_slot    <= slot0_r;    // 从链头开始预走
                    arm_steps   <= frag_off[13:11];  // 跨 slot 步数（≤7，8192B 消息）
                    state       <= S_ARM;
                end
                // 分片起始地址预走链：把读位置推进到本片在消息中的 slot
                S_ARM: begin
                    if (arm_steps != 3'd0) begin
                        pay_slot  <= nx_rd;      // nx_ra = pay_slot（组合跟链）
                        arm_steps <= arm_steps - 3'd1;
                    end else begin
                        pay_off <= frag_off[10:0];   // slot 内偏移
                        state   <= S_RDYW;
                    end
                end
                S_RDYW: begin
                    if (s_ready) begin         // mac 已进 BODY，可发流
                        s_valid <= 1'b1;
                        state   <= S_STREAM;
                    end
                end
                S_STREAM: begin
                    if (consuming) begin
                        if (cnt == body_len_r - 11'd1) begin
                            // 末字节（SN）已消费：收尾
                            s_valid   <= 1'b0;
                            frag_upd  <= 1'b1;
                            frag_last <= !mf_r;    // 末片（或单帧）→ 消息完成
                            frag_vl   <= task_vl;
                            frag_slot0<= slot0_r;
                            frag_len  <= total_r;
                            bld_busy  <= 1'b0;
                            cnt       <= 11'd0;
                            state     <= S_DONE;
                        end else begin
                            cnt <= cnt + 11'd1;
                            // 载荷读位置推进（含末头字节→载荷 0 的衔接拍）
                            if (adv_pay) begin
                                if (pay_off == 11'd2047) begin
                                    pay_slot <= nx_rd;   // 跨 slot 跟链
                                    pay_off  <= 11'd0;
                                end else
                                    pay_off <= pay_off + 11'd1;
                            end
                        end
                    end
                end
                S_DONE: state <= S_IDLE;       // 等调度器跑完 mac 再回 IDLE
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
