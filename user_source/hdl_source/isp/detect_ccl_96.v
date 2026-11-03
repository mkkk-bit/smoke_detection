module detect_ccl_96 #(
    parameter IMG_WIDTH  = 1280,      // 帧宽
    parameter IMG_HEIGHT = 720,       // 帧高
    parameter MAX_OBJ    = 2          // 输出目标数; 放在参数列表中供端口位宽使用
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [3:0]  I_candidate,   // 红色候选掩码 (bit0=最低列像素, 与 color_mask_96 同拍)
    input  wire        I_tuser,       // 帧开始 (与候选同拍)
    input  wire        I_tlast,       // 行结束
    input  wire        I_tvalid,
    input  wire [19:0] I_min_area,    // 帧内稳定的最小连通区域面积门槛
    output reg  [MAX_OBJ*11-1:0] O_bbox_xmin,   // 多目标外接框, 按面积降序: [i*11+:11]=第i大目标
    output reg  [MAX_OBJ*11-1:0] O_bbox_xmax,   // (帧头锁存上一帧结果, 帧内稳定)
    output reg  [MAX_OBJ*10-1:0] O_bbox_ymin,
    output reg  [MAX_OBJ*10-1:0] O_bbox_ymax,
    output reg  [MAX_OBJ-1:0]    O_bbox_valid,  // bit i = 第i大目标面积 >= MIN_AREA (告警/框选唯一判定源)
    output reg  [MAX_OBJ*20-1:0] O_area         // [i*20+:20] = 第i大目标面积 (标定观察)
);

    localparam [9:0] X_BEAT_MAX = IMG_WIDTH/4 - 1;   // 每行拍数-1 (1280/4-1=319)
    localparam [9:0] Y_MAX      = IMG_HEIGHT - 1;    // 末行号 (719)
    localparam       SEG_MAX    = 16;  // 每行段表深度 (时序收敛: 32→16)

    // ================= 一级流水: 输入寄存 (位置计数与其同步) =================
    reg [3:0] I_candidate_d1;
    reg       I_tvalid_d1, I_tuser_d1, I_tlast_d1, I_tlast_d2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            I_candidate_d1 <= 4'd0;
            I_tvalid_d1    <= 1'b0;
            I_tuser_d1     <= 1'b0;
            I_tlast_d1     <= 1'b0;
            I_tlast_d2     <= 1'b0;
        end else begin
            I_candidate_d1 <= I_candidate;
            I_tvalid_d1    <= I_tvalid;
            I_tuser_d1     <= I_tuser;
            I_tlast_d1     <= I_tlast;
            I_tlast_d2     <= I_tlast_d1;
        end
    end

    // ---- 像素位置计数: 与一级流水同步 (使能=未延时 valid) ----
    reg [9:0] x_beat;    // 拍序号 0..IMG_WIDTH/4-1
    reg [9:0] y_pos;     // 行号 0..IMG_HEIGHT-1
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x_beat <= 10'd0;
            y_pos  <= 10'd0;
        end else if(I_tvalid) begin
            if(I_tuser) begin
                x_beat <= 10'd0;
                y_pos  <= 10'd0;
            end else if(x_beat == X_BEAT_MAX) begin
                x_beat <= 10'd0;
                if(y_pos == Y_MAX)
                    y_pos <= 10'd0;
                else
                    y_pos <= y_pos + 1'b1;
            end else begin
                x_beat <= x_beat + 1'b1;
            end
        end
    end

    // 每拍4像素的列号; 1280宽 → 列号最大1279, 11bit
    wire [11:0] col_base = {x_beat, 2'b00};
    wire [10:0] col_a = col_base[10:0];
    wire [10:0] col_b = col_base[10:0] + 11'd1;
    wire [10:0] col_c = col_base[10:0] + 11'd2;
    wire [10:0] col_d = col_base[10:0] + 11'd3;

    // ================= 段状态 (跨拍) =================
    reg        cur_seg_on;      // 上一拍末像素是否处于候选段中
    reg [10:0] cur_seg_xmin;    // 当前未结束段的起始列
    reg [10:0] cur_seg_xmax;    // 延迟收尾段最后一个实际候选像素的列
    reg        cur_seg_lane2_pending; // 0101模式暂存的lane2单像素段

    // ================= 段表: ping-pong 双缓冲 =================
    // tbl_sel=当前行写表, 1-tbl_sel=上一行查询表
    reg [10:0] seg_xmin_tbl[0:1][0:SEG_MAX-1];
    reg [10:0] seg_xmax_tbl[0:1][0:SEG_MAX-1];
    reg [1:0]  seg_label_tbl[0:1][0:SEG_MAX-1];
    reg        seg_valid_tbl[0:1][0:SEG_MAX-1];
    reg [4:0]  cur_seg_cnt;     // 当前行已提交段数
    reg        tbl_sel;

    // ================= 目标统计 (≤MAX_OBJ=2) =================
    reg [19:0] obj_area[0:MAX_OBJ-1];
    reg [10:0] obj_xmin[0:MAX_OBJ-1], obj_xmax[0:MAX_OBJ-1];
    reg [9:0]  obj_ymin[0:MAX_OBJ-1], obj_ymax[0:MAX_OBJ-1];
    reg        obj_valid[0:MAX_OBJ-1];
    reg [1:0]  label_equiv[0:MAX_OBJ-1];   // 合并后等价映射 (label→主label)

    // ============ 拍内首段解析 (组合) ============
    // 输入: cur_seg_on, I_candidate_d1; 输出: 本拍首段(起点/终点/是否拍内结束) + 拍末续段状态
    reg        segA_act, segA_end_beat, new_on, new_lane2_pending;
    reg [10:0] segA_start, segA_end_col, new_xmin, new_xmax;
    always @(*) begin
        segA_act = 1'b0; segA_end_beat = 1'b0;
        segA_start = 11'd0; segA_end_col = 11'd0;
        new_on = cur_seg_on; new_xmin = cur_seg_xmin; new_xmax = cur_seg_xmax;
        new_lane2_pending = cur_seg_lane2_pending;

        if(cur_seg_on) begin
            // 续段: 起点=cur_seg_xmin; 像素0=0 → 段结束于上一拍末列
            segA_act = 1'b1; segA_start = cur_seg_xmin;
            if(!I_candidate_d1[0]) begin
                segA_end_beat = 1'b1; segA_end_col = cur_seg_xmax; new_on = 1'b0;
            end else if(!I_candidate_d1[1]) begin
                segA_end_beat = 1'b1; segA_end_col = col_a; new_on = 1'b0;
            end else if(!I_candidate_d1[2]) begin
                segA_end_beat = 1'b1; segA_end_col = col_b; new_on = 1'b0;
            end else if(!I_candidate_d1[3]) begin
                segA_end_beat = 1'b1; segA_end_col = col_c; new_on = 1'b0;
            end else begin
                segA_end_beat = 1'b0; new_on = 1'b1;
                new_xmin = cur_seg_xmin; new_xmax = col_d;  // 延到拍末
            end
        end else begin
            // 新段: 取最左 0→1 起点 (拍内多段模式只取首段)
            if(I_candidate_d1[0]) begin
                segA_act = 1'b1; segA_start = col_a;
                if(!I_candidate_d1[1])      begin segA_end_beat = 1'b1; segA_end_col = col_a; end
                else if(!I_candidate_d1[2]) begin segA_end_beat = 1'b1; segA_end_col = col_b; end
                else if(!I_candidate_d1[3]) begin segA_end_beat = 1'b1; segA_end_col = col_c; end
                else begin segA_end_beat = 1'b0; new_on = 1'b1; new_xmin = col_a; new_xmax = col_d; end
            end else if(I_candidate_d1[1]) begin
                segA_act = 1'b1; segA_start = col_b;
                if(!I_candidate_d1[2])      begin segA_end_beat = 1'b1; segA_end_col = col_b; end
                else if(!I_candidate_d1[3]) begin segA_end_beat = 1'b1; segA_end_col = col_c; end
                else begin segA_end_beat = 1'b0; new_on = 1'b1; new_xmin = col_b; new_xmax = col_d; end
            end else if(I_candidate_d1[2]) begin
                segA_act = 1'b1; segA_start = col_c;
                if(!I_candidate_d1[3])      begin segA_end_beat = 1'b1; segA_end_col = col_c; end
                else begin segA_end_beat = 1'b0; new_on = 1'b1; new_xmin = col_c; new_xmax = col_d; end
            end else if(I_candidate_d1[3]) begin
                segA_act = 1'b1; segA_start = col_d;
                segA_end_beat = 1'b0; new_on = 1'b1; new_xmin = col_d; new_xmax = col_d;
            end
        end

        // 若本拍的首段已经结束，但第4个像素仍为候选像素，
        // 保存末尾连续段的起点，供下一拍检测到 0 时提交。
        // 例如 candidate=4'b1001：先提交 lane0，lane3 留到下一拍收尾。
        if(I_candidate_d1[3]) begin
            new_on = 1'b1;
            new_xmax = col_d;
            if(I_candidate_d1[2]) begin
                if(I_candidate_d1[1]) begin
                    if(I_candidate_d1[0])
                        new_xmin = cur_seg_on ? cur_seg_xmin : col_a;
                    else
                        new_xmin = col_b;
                end else begin
                    new_xmin = col_c;
                end
            end else begin
                new_xmin = col_d;
            end
        end

        // 4'b0101 这类图案的第二段结束在 lane2：将其暂存到下一拍，
        // 下一拍 lane0 为0时用保存的实际终点提交，避免把lane3也计入面积。
        if(I_candidate_d1[2] && !I_candidate_d1[3] && segA_act &&
           segA_end_beat && (segA_end_col < col_c)) begin
            new_on = 1'b1;
            new_xmin = col_c;
            new_xmax = col_c;
            new_lane2_pending = 1'b1;
        end

        // 0101后紧接下一拍lane0有候选时，不能把新段并入暂存的lane2段。
        // 本拍提交旧lane2单像素段；当前拍从lane0开始的新段暂存，下一拍再收尾。
        if(cur_seg_lane2_pending && I_candidate_d1[0]) begin
            segA_act = 1'b1;
            segA_start = cur_seg_xmin;
            segA_end_beat = 1'b1;
            segA_end_col = cur_seg_xmax;
            new_lane2_pending = 1'b0;
            new_on = 1'b1;
            new_xmin = col_a;
            if(I_candidate_d1[1]) begin
                new_xmax = col_b;
                if(I_candidate_d1[2]) begin
                    new_xmax = col_c;
                    if(I_candidate_d1[3]) new_xmax = col_d;
                end
            end else begin
                new_xmax = col_a;
            end
        end
    end

    // 段有效终点列: 行尾且段延伸至拍末 → 结束于行尾列
    wire row_end_force = I_tlast_d1 && segA_act && !segA_end_beat;
    wire [10:0] seg_end_eff = row_end_force ? col_d : segA_end_col;

    wire seg_submit = I_tvalid_d1 && segA_act &&
                      (segA_end_beat || row_end_force) && !I_tuser_d1;

    // 行尾末段悬挂项的状态在重叠查询之前声明，兼容 ModelSim 的 Verilog 编译顺序。
    reg        pending_val;
    reg [10:0] pending_start, pending_end;
    reg [1:0]  pending_lbl;

    // ============ 上一行段表重叠查询 (组合, 拍1打一拍收敛时序) ============
    // 4邻域: 上一行段 [xmin,xmax] 与本段 [segA_start, seg_end_eff] 列区间相交即连通
    // + 行尾末段悬挂项 (pending): tlast 拍的段表项晚于行尾翻转落入下一行写表,
    //   查询表看不到 → 用悬挂项补入, 相邻行边沿段恢复连通
    reg [1:0] ovl_comb;
    integer qi;
    always @(*) begin
        ovl_comb = 2'd0;
        for(qi = 0; qi < SEG_MAX; qi = qi + 1) begin
            if(seg_valid_tbl[1-tbl_sel][qi] &&
               (seg_xmin_tbl[1-tbl_sel][qi] <= seg_end_eff) &&
               (seg_xmax_tbl[1-tbl_sel][qi] >= segA_start))
                ovl_comb[seg_label_tbl[1-tbl_sel][qi]] = 1'b1;
        end
        if(pending_val && (pending_start <= seg_end_eff) && (pending_end >= segA_start))
            ovl_comb[pending_lbl] = 1'b1;
    end
    reg [1:0] ovl_comb_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) ovl_comb_q <= 2'd0;
        else         ovl_comb_q <= ovl_comb;
    end

    // 等价映射后的重叠标签集合
    reg [1:0] ovl_m;
    integer li;
    always @(*) begin
        ovl_m = 2'd0;
        for(li = 0; li < MAX_OBJ; li = li + 1)
            if(ovl_comb_q[li])
                ovl_m[label_equiv[li]] = 1'b1;
    end

    // 重叠集合主标签 (最低位)
    reg [1:0] main_label;
    integer bi;
    reg found_main;
    always @(*) begin
        main_label = 2'd0;
        found_main = 1'b0;
        for(bi = 0; bi < MAX_OBJ; bi = bi + 1)
            if(!found_main && ovl_m[bi]) begin
                main_label = bi[1:0];
                found_main = 1'b1;
            end
    end

    // ============ 提交流水线寄存器 (拍1→拍2→拍3) ============
    reg seg_submit_q, seg_submit_q2;
    reg [10:0] segA_start_q, seg_end_eff_q;
    wire [10:0] seg_len = seg_end_eff_q - segA_start_q + 11'd1;
    reg [9:0]  y_pos_q;
    reg [1:0]  t_label_q;
    reg [1:0]  ovl_m_q;
    reg        t_drop_q;
    // 拍2 坐标延迟 (与 seg_submit_q2 对齐, 段表写入用)
    reg [10:0] segA_start_q2, seg_end_eff_q2;
    // 增量结果: 拍1计算寄存(q1), 拍2移位(q2), 拍3写回
    reg [19:0] t_area_inc_q1, t_area_inc_q2;
    reg [10:0] t_xmin_new_q1, t_xmax_new_q1;
    reg [9:0]  t_ymin_new_q1, t_ymax_new_q1;
    reg [10:0] t_xmin_new_q2, t_xmax_new_q2;
    reg [9:0]  t_ymin_new_q2, t_ymax_new_q2;
    reg [1:0]  t_label_q2;
    reg [1:0]  ovl_m_q3;
    reg        t_valid_q3;
    reg        t_fresh_q;    // 拍1: 本提交是否新对象起步
    reg        t_fresh_q2;   // 拍2 移位

    // 占用表 (obj_valid + 在途写回, 消除3拍流水窗口内的标签误复用)
    // 在途1: 本拍正在写回的标签 (t_valid_q3 & t_label_q2)
    // 在途2: 下一拍将要写回的标签 (seg_submit_q2 & !t_drop_q & t_label_q)
    wire t_valid_q3_next = seg_submit_q2 && !t_drop_q;
    wire busy0 = (t_valid_q3 && (t_label_q2 == 2'd0)) ||
                 (t_valid_q3_next && (t_label_q == 2'd0));
    wire busy1 = (t_valid_q3 && (t_label_q2 == 2'd1)) ||
                 (t_valid_q3_next && (t_label_q == 2'd1));
    // 合并写回会释放非主标签。若下一段在合并完成前到达，允许它预占
    // 即将释放的标签；流水线占用(busy)仍优先，避免覆盖该标签上的在途新对象。
    wire merge_release0 =
        (t_valid_q3 && ovl_m_q3[0] && (t_label_q2 != 2'd0)) ||
        (t_valid_q3_next && ovl_m_q[0] && (t_label_q != 2'd0));
    wire merge_release1 =
        (t_valid_q3 && ovl_m_q3[1] && (t_label_q2 != 2'd1)) ||
        (t_valid_q3_next && ovl_m_q[1] && (t_label_q != 2'd1));
    wire       avail0 = !busy0 && (!obj_valid[0] || merge_release0);
    wire       avail1 = !busy1 && (!obj_valid[1] || merge_release1);
    wire       found_free_eff = avail0 || avail1;
    wire [1:0] free_label_eff = avail0 ? 2'd0 : 2'd1;
    // 标签回收: 无空闲标签且无重叠(新目标起步)时, 从"本行未使用且无在途写回"
    // 的有效对象中回收面积最小者。散点噪声面积恒小(≈1)永远先被回收;
    // 面积达到 MIN_AREA 的对象帧内锁定, 不再被回收 — 否则真实目标建成后
    // 仍会被单个散点噪声顶掉 (row_act 保护本行刚用的噪声标签, 唯一候选
    // 反而只剩大目标: 上板 8x8 团块在行310被噪声顶掉的实测根因)
    reg  [1:0] row_act;
    wire cand0 = obj_valid[0] && !busy0 && !row_act[0] && (obj_area[0] < I_min_area);
    wire cand1 = obj_valid[1] && !busy1 && !row_act[1] && (obj_area[1] < I_min_area);
    wire       evict_possible = cand0 || cand1;
    wire       evict0 = cand0 && (!cand1 || (obj_area[0] <= obj_area[1]));
    wire [1:0] evict_label = evict0 ? 2'd0 : 2'd1;
    // 修正9: 回收还需入段长度 > 受害对象面积。生长中目标(面积已积累但
    // 未达 MIN_AREA)不会被散点噪声(段长1)顶掉; 真实新目标首段仍可回收
    // 小面积噪声对象; 仅当新段比待回收对象更长时才允许回收
    wire       evict_ok   = seg_len > (evict0 ? obj_area[0] : obj_area[1]);
    // 新对象起步标志: 无重叠 → 空闲或回收标签 → 拍3覆盖面积并复位等价映射
    wire       t_fresh_sel = (ovl_m == 2'd0);
    // 本段标签与丢弃决策 (组合, 拍1使用)
    wire [1:0] t_label_sel = (ovl_m == 2'd0) ?
                              (found_free_eff ? free_label_eff : evict_label) : main_label;
    wire       t_drop_sel  = (ovl_m == 2'd0) && !found_free_eff && !(evict_possible && evict_ok);

    // ============ 拍1: 标签决策 + 合并增量组合计算 ============
    // (与决策同拍组合计算并直接寄存, 拍2移位, 拍3写回 — 逐级对齐可证明,
    //  修正十六工程原版跨always阻塞赋值同拍采样造成的写回滞后一拍)
    // 合并面积: ovl_m 中除主标签外的其他目标面积之和 (树状, MAX_OBJ=2)
    wire [19:0] m_area0 = (ovl_m[0] && (t_label_sel != 2'd0)) ? obj_area[0] : 20'd0;
    wire [19:0] m_area1 = (ovl_m[1] && (t_label_sel != 2'd1)) ? obj_area[1] : 20'd0;
    wire [20:0] area_merge = m_area0 + m_area1;
    // 本段面积 (列区间长度)
    wire [19:0] t_area_inc = area_merge[19:0] + {9'd0, seg_len};
    // 合并 bbox: 仅"重叠已有目标"时继承旧bbox; 新对象起步(空闲/回收标签)
    // 从本段起算, 不继承被回收对象的陈旧bbox
    wire       base_use = (ovl_m != 2'd0) && obj_valid[t_label_sel];
    wire [10:0] base_xmin = base_use ? obj_xmin[t_label_sel] : segA_start_q;
    wire [10:0] base_xmax = base_use ? obj_xmax[t_label_sel] : seg_end_eff_q;
    wire [9:0]  base_ymin = base_use ? obj_ymin[t_label_sel] : y_pos_q;
    wire [9:0]  base_ymax = base_use ? obj_ymax[t_label_sel] : y_pos_q;
    wire [10:0] m_xmin0 = (ovl_m[0] && (t_label_sel != 2'd0) && (obj_xmin[0] < base_xmin)) ? obj_xmin[0] : base_xmin;
    wire [10:0] m_xmin1 = (ovl_m[1] && (t_label_sel != 2'd1) && (obj_xmin[1] < m_xmin0))    ? obj_xmin[1] : m_xmin0;
    wire [10:0] m_xmax0 = (ovl_m[0] && (t_label_sel != 2'd0) && (obj_xmax[0] > base_xmax)) ? obj_xmax[0] : base_xmax;
    wire [10:0] m_xmax1 = (ovl_m[1] && (t_label_sel != 2'd1) && (obj_xmax[1] > m_xmax0))    ? obj_xmax[1] : m_xmax0;
    wire [9:0]  m_ymin0 = (ovl_m[0] && (t_label_sel != 2'd0) && (obj_ymin[0] < base_ymin)) ? obj_ymin[0] : base_ymin;
    wire [9:0]  m_ymin1 = (ovl_m[1] && (t_label_sel != 2'd1) && (obj_ymin[1] < m_ymin0))    ? obj_ymin[1] : m_ymin0;
    wire [9:0]  m_ymax0 = (ovl_m[0] && (t_label_sel != 2'd0) && (obj_ymax[0] > base_ymax)) ? obj_ymax[0] : base_ymax;
    wire [9:0]  m_ymax1 = (ovl_m[1] && (t_label_sel != 2'd1) && (obj_ymax[1] > m_ymax0))    ? obj_ymax[1] : m_ymax0;
    wire [10:0] t_xmin_new = (segA_start_q  < m_xmin1) ? segA_start_q  : m_xmin1;
    wire [10:0] t_xmax_new = (seg_end_eff_q > m_xmax1) ? seg_end_eff_q : m_xmax1;
    wire [9:0]  t_ymin_new = (y_pos_q       < m_ymin1) ? y_pos_q       : m_ymin1;
    wire [9:0]  t_ymax_new = (y_pos_q       > m_ymax1) ? y_pos_q       : m_ymax1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            seg_submit_q   <= 1'b0;
            seg_submit_q2  <= 1'b0;
            segA_start_q   <= 11'd0;
            seg_end_eff_q  <= 11'd0;
            y_pos_q        <= 10'd0;
            t_label_q      <= 2'd0;
            ovl_m_q        <= 2'd0;
            t_drop_q       <= 1'b0;
            segA_start_q2  <= 11'd0;
            seg_end_eff_q2 <= 11'd0;
            t_area_inc_q1  <= 20'd0;
            t_area_inc_q2  <= 20'd0;
            t_xmin_new_q1  <= 11'd0;
            t_xmax_new_q1  <= 11'd0;
            t_ymin_new_q1  <= 10'd0;
            t_ymax_new_q1  <= 10'd0;
            t_xmin_new_q2  <= 11'd0;
            t_xmax_new_q2  <= 11'd0;
            t_ymin_new_q2  <= 10'd0;
            t_ymax_new_q2  <= 10'd0;
            t_label_q2     <= 2'd0;
            ovl_m_q3       <= 2'd0;
            t_valid_q3     <= 1'b0;
            t_fresh_q      <= 1'b0;
            t_fresh_q2     <= 1'b0;
        end else begin
            seg_submit_q   <= seg_submit;
            seg_submit_q2  <= seg_submit_q;
            segA_start_q   <= segA_start;
            seg_end_eff_q  <= seg_end_eff;
            y_pos_q        <= y_pos;
            // 拍2 坐标: 从 _q 再延迟一拍 (段表写入用)
            segA_start_q2  <= segA_start_q;
            seg_end_eff_q2 <= seg_end_eff_q;
            // 拍1: 标签决策 + 增量组合计算 (与决策同拍寄存)
            if(seg_submit_q) begin
                t_label_q     <= t_label_sel;
                t_drop_q      <= t_drop_sel;
                ovl_m_q       <= ovl_m;
                t_fresh_q     <= t_fresh_sel;
                t_area_inc_q1 <= t_area_inc;
                t_xmin_new_q1 <= t_xmin_new;
                t_xmax_new_q1 <= t_xmax_new;
                t_ymin_new_q1 <= t_ymin_new;
                t_ymax_new_q1 <= t_ymax_new;
            end
            // 拍2: 增量移位寄存 (自由移位, 与写回使能逐级对齐)
            t_area_inc_q2 <= t_area_inc_q1;
            t_xmin_new_q2 <= t_xmin_new_q1;
            t_xmax_new_q2 <= t_xmax_new_q1;
            t_ymin_new_q2 <= t_ymin_new_q1;
            t_ymax_new_q2 <= t_ymax_new_q1;
            t_fresh_q2    <= t_fresh_q;
            // 拍3: 写回使能与标签
            t_valid_q3 <= seg_submit_q2 && !t_drop_q;
            t_label_q2 <= t_label_q;
            ovl_m_q3   <= ovl_m_q;
        end
    end

    // 每标签本行活动标志: 行尾/帧头清零; 本拍提交段落用标签置位 (拍1同拍)
    // 作用: 保护本行正在使用的目标不被回收 (行内多段目标每段置位, 跨行目标
    // 每行首段提交前不受保护 — 但其面积远大于散点噪声, 最小者优先策略不会选中)
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            row_act <= 2'd0;
        end else begin
            if(I_tlast_d1 || I_tuser_d1)
                row_act <= 2'd0;
            else if(seg_submit_q && !t_drop_sel)
                row_act <= row_act | (2'b01 << t_label_sel);
        end
    end

    // 行尾末段悬挂: tlast 拍提交的段, 其段表项写入(拍2)晚于行尾翻转,
    // 落在下一行写表而非查询表 → 相邻行边沿段断连, 贴右边界团块被拆成
    // 交替行对象(每行一段各自成对象, 面积腰斩)。tlast+1 拍用拍1同源
    // 决策锁存, 下一行重叠查询补入 (帧头清零)
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            pending_val   <= 1'b0;
            pending_start <= 11'd0;
            pending_end   <= 11'd0;
            pending_lbl   <= 2'd0;
        end else if(I_tuser_d1) begin
            pending_val   <= 1'b0;
        end else if(I_tlast_d2) begin
            pending_val   <= seg_submit_q && !t_drop_sel;
            pending_start <= segA_start_q;
            pending_end   <= seg_end_eff_q;
            pending_lbl   <= t_label_sel;
        end
    end

    // ============ 帧头结算: 面积降序输出全部 >= MIN_AREA 的连通域 ============
    // box0 = 最大目标 (与原单目标输出语义一致), box1 = 次大; 面积<MIN_AREA 不输出
    // (MAX_OBJ=2 特化排序; 扩充 MAX_OBJ 时此处需改为选择排序)
    wire v0 = obj_valid[0] && (obj_area[0] >= I_min_area);
    wire v1 = obj_valid[1] && (obj_area[1] >= I_min_area);
    wire box0_is_1 = v1 && (!v0 || (obj_area[1] > obj_area[0]));

    // ============ 主状态机: 帧头结算 / 行内段表 / 写回 ============
    integer ci;
    integer oi;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_bbox_xmin  <= {MAX_OBJ*11{1'b0}};
            O_bbox_xmax  <= {MAX_OBJ*11{1'b0}};
            O_bbox_ymin  <= {MAX_OBJ*10{1'b0}};
            O_bbox_ymax  <= {MAX_OBJ*10{1'b0}};
            O_bbox_valid <= {MAX_OBJ{1'b0}};
            O_area       <= {MAX_OBJ*20{1'b0}};
            cur_seg_on   <= 1'b0;
            cur_seg_xmin <= 11'd0;
            cur_seg_xmax <= 11'd0;
            cur_seg_lane2_pending <= 1'b0;
            cur_seg_cnt  <= 5'd0;
            tbl_sel      <= 1'b0;
            for(oi = 0; oi < MAX_OBJ; oi = oi + 1) begin
                obj_valid[oi]    <= 1'b0;
                obj_area[oi]     <= 20'd0;
                obj_xmin[oi]     <= 11'd0; obj_xmax[oi] <= 11'd0;
                obj_ymin[oi]     <= 10'd0; obj_ymax[oi] <= 10'd0;
                label_equiv[oi]  <= oi[1:0];
            end
            for(oi = 0; oi < SEG_MAX; oi = oi + 1) begin
                seg_valid_tbl[0][oi] <= 1'b0;
                seg_valid_tbl[1][oi] <= 1'b0;
            end
        end else begin
            // 帧头结算只看 tuser, 不受 tvalid 门控:
            // 消隐期 tvalid=0 时帧头拍的前一拍无效, 若用 if(I_tvalid_d1) 门控
            // 则快照永不触发 → O_bbox_valid 恒0 (上板无框根因之一)
            if(I_tuser_d1) begin
                    // ===== 帧头: 锁存上一帧全部目标 (面积降序) + 全清零 =====
                    O_bbox_valid        <= {v0 && v1, v0 || v1};
                    O_bbox_xmin[0*11 +: 11] <= box0_is_1 ? obj_xmin[1] : obj_xmin[0];
                    O_bbox_xmax[0*11 +: 11] <= box0_is_1 ? obj_xmax[1] : obj_xmax[0];
                    O_bbox_ymin[0*10 +: 10] <= box0_is_1 ? obj_ymin[1] : obj_ymin[0];
                    O_bbox_ymax[0*10 +: 10] <= box0_is_1 ? obj_ymax[1] : obj_ymax[0];
                    O_area[0*20 +: 20]      <= box0_is_1 ? obj_area[1] : obj_area[0];
                    O_bbox_xmin[1*11 +: 11] <= box0_is_1 ? obj_xmin[0] : obj_xmin[1];
                    O_bbox_xmax[1*11 +: 11] <= box0_is_1 ? obj_xmax[0] : obj_xmax[1];
                    O_bbox_ymin[1*10 +: 10] <= box0_is_1 ? obj_ymin[0] : obj_ymin[1];
                    O_bbox_ymax[1*10 +: 10] <= box0_is_1 ? obj_ymax[0] : obj_ymax[1];
                    O_area[1*20 +: 20]      <= box0_is_1 ? obj_area[0] : obj_area[1];
                    // 清零本帧统计
                    cur_seg_on   <= 1'b0;
                    cur_seg_xmin <= 11'd0;
                    cur_seg_xmax <= 11'd0;
                    cur_seg_lane2_pending <= 1'b0;
                    cur_seg_cnt  <= 5'd0;
                    tbl_sel      <= 1'b0;
                    for(oi = 0; oi < MAX_OBJ; oi = oi + 1) begin
                        obj_valid[oi]    <= 1'b0;
                        obj_area[oi]     <= 20'd0;
                        obj_xmin[oi]     <= 11'd0; obj_xmax[oi] <= 11'd0;
                        obj_ymin[oi]     <= 10'd0; obj_ymax[oi] <= 10'd0;
                        label_equiv[oi]  <= oi[1:0];
                    end
                    for(oi = 0; oi < SEG_MAX; oi = oi + 1) begin
                        seg_valid_tbl[0][oi] <= 1'b0;
                        seg_valid_tbl[1][oi] <= 1'b0;
                    end
                end else if(I_tvalid_d1) begin
                    // ===== 行内: 段表写入(拍2) + 段状态 + 行尾翻转 =====
                    if(seg_submit_q2 && !t_drop_q && cur_seg_cnt < SEG_MAX) begin
                        seg_xmin_tbl[tbl_sel][cur_seg_cnt]  <= segA_start_q2;
                        seg_xmax_tbl[tbl_sel][cur_seg_cnt] <= seg_end_eff_q2;
                        seg_label_tbl[tbl_sel][cur_seg_cnt] <= t_label_q;
                        seg_valid_tbl[tbl_sel][cur_seg_cnt] <= 1'b1;
                        cur_seg_cnt <= cur_seg_cnt + 5'd1;
                    end
                    // 段状态更新 (每拍)
                    cur_seg_on   <= I_tlast_d1 ? 1'b0 : new_on;
                    cur_seg_xmin <= new_xmin;
                    cur_seg_xmax <= new_xmax;
                    cur_seg_lane2_pending <= I_tlast_d1 ? 1'b0 : new_lane2_pending;
                    // 行尾: 切换段表 (当前行→上一行); 清空将成为当前行的表
                    // (防止空行残留上一行段, 导致隔空行错误连通)
                    if(I_tlast_d1) begin
                        tbl_sel     <= ~tbl_sel;
                        cur_seg_cnt <= 5'd0;
                        for(oi = 0; oi < SEG_MAX; oi = oi + 1)
                            seg_valid_tbl[1-tbl_sel][oi] <= 1'b0;
                    end
                end
            // 拍3: 写回 obj (不受 tvalid 门控 — 消隐期完成末行末段写回;
            //      帧头拍跳过, 防止陈旧写回与帧头清零冲突)
            if(t_valid_q3 && !I_tuser_d1) begin
                // 清除被合并的标签 (同时清面积, 防复活标签携带陈旧面积)
                for(ci = 0; ci < MAX_OBJ; ci = ci + 1) begin
                    if(ovl_m_q3[ci] && (ci[1:0] != t_label_q2)) begin
                        obj_valid[ci]    <= 1'b0;
                        label_equiv[ci]  <= t_label_q2;
                        obj_area[ci]     <= 20'd0;
                    end
                end
                // 主标签更新: 新对象起步覆盖面积并复位等价映射; 否则累加
                if(t_fresh_q2) begin
                    obj_valid[t_label_q2]   <= 1'b1;
                    obj_area[t_label_q2]    <= t_area_inc_q2;
                    label_equiv[t_label_q2] <= t_label_q2;
                end else begin
                    if(!obj_valid[t_label_q2])
                        obj_valid[t_label_q2] <= 1'b1;
                    obj_area[t_label_q2] <= obj_area[t_label_q2] + t_area_inc_q2;
                end
                obj_xmin[t_label_q2] <= t_xmin_new_q2;
                obj_xmax[t_label_q2] <= t_xmax_new_q2;
                obj_ymin[t_label_q2] <= t_ymin_new_q2;
                obj_ymax[t_label_q2] <= t_ymax_new_q2;
            end
        end
    end

endmodule
