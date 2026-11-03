/******************************************************************************
 * smoke_bbox_96.v — 烟头暗光亮点包围盒 (帧级极值统计, 无连通域标签)
 *
 * 位置: user_source/hdl_source/isp/smoke_bbox_96.v
 * 时钟: 与 luma_spot 同源 (axi4s_video_aclk = S_csi_rx_clk)
 *
 * 功能: 输入 luma_spot 的亮点掩码 (每拍4bit), 逐像素统计:
 *   ① 帧内亮点像素计数 O_count (面积)
 *   ② 亮点簇包围盒 O_bbox_* (命中像素的 x/y 极值)
 *   ③ O_valid = 面积落在 [CNT_MIN, CNT_MAX] 区间 (烟头判定)
 * 帧头(tuser)锁存上一帧结果, 帧内稳定。
 *
 * 设计依据 (室内烟头检测):
 *   - 点燃的烟头呈现为高亮白炽点簇, 面积小且孤立;
 *   - 白墙/灯光等大面积亮区面积超 CNT_MAX → 不算烟头;
 *   - 散点噪声(单像素)面积 < CNT_MIN → 不算烟头 (抗噪);
 *   - 火焰主体由 color_mask/detect_ccl_96 负责, 本模块只负责暗光亮点。
 *****************************************************************************/
module smoke_bbox_96 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter CNT_MIN    = 20'd20,    // 烟头亮点面积下限 (抗散点噪声)
    parameter CNT_MAX    = 20'd2000   // 烟头亮点面积上限 (排除大面积亮区)
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [3:0]  I_mask,        // 亮点掩码 (bit0=最先像素, 与 luma_spot 同拍)
    input  wire [10:0] I_roi_left,
    input  wire [10:0] I_roi_right,
    input  wire        I_tuser,       // 帧开始
    input  wire        I_tlast,       // 行结束
    input  wire        I_tvalid,
    output reg  [10:0] O_bbox_xmin,   // 亮点簇包围盒 (帧头锁存上一帧, 帧内稳定)
    output reg  [10:0] O_bbox_xmax,
    output reg  [9:0]  O_bbox_ymin,
    output reg  [9:0]  O_bbox_ymax,
    output reg         O_valid,       // 面积在 [CNT_MIN,CNT_MAX] 区间
    output reg  [19:0] O_count,       // 帧内亮点像素数 (标定观察)
    output reg  [7:0]  O_confidence   // 烟头置信度 0~100 (可解释)
);

    localparam [9:0] X_BEAT_MAX = IMG_WIDTH/4 - 1;   // 319
    localparam [9:0] Y_MAX      = IMG_HEIGHT - 1;    // 719

    // ---- 位置计数 (与输入同拍: 使能=I_tvalid) ----
    reg [9:0]  x_beat;
    reg [9:0]  y_pos;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x_beat <= 10'd0;
            y_pos  <= 10'd0;
        end else if(I_tvalid) begin
            if(I_tuser) begin
                x_beat <= 10'd1; // tuser与首拍像素同拍; 下一拍从x=4开始
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

    wire [10:0] col_a = {x_beat, 2'b00};
    wire [10:0] col_b = col_a + 11'd1;
    wire [10:0] col_c = col_a + 11'd2;
    wire [10:0] col_d = col_a + 11'd3;
    wire [9:0]  y_cur = y_pos;
    function roi_inside;
        input [10:0] x;
        input [10:0] left_x;
        input [10:0] right_x;
        begin
            if(left_x < right_x)
                roi_inside = (x > left_x) && (x < right_x);
            else if(left_x > right_x)
                roi_inside = (x > left_x) || (x < right_x);
            else
                roi_inside = 1'b0;
        end
    endfunction
    wire [3:0] roi_mask = {
        I_mask[3] && roi_inside(col_d, I_roi_left, I_roi_right),
        I_mask[2] && roi_inside(col_c, I_roi_left, I_roi_right),
        I_mask[1] && roi_inside(col_b, I_roi_left, I_roi_right),
        I_mask[0] && roi_inside(col_a, I_roi_left, I_roi_right)
    };

    // ---- 帧内累计: 面积 + 极值 ----
    // 注意: 极值必须用组合变量逐像素求 min/max, 再单次非阻塞寄存;
    //       若在 always 内对同一 reg 连续非阻塞赋值, 只有最后一次生效
    reg [19:0] cnt_accum;
    reg [10:0] x_min_r, x_max_r;
    reg [9:0]  y_min_r, y_max_r;
    reg        any_hit;
    // 组合: 本拍 4 像素命中后的新极值 (阻塞赋值逐句更新)
    reg [10:0] n_xmin, n_xmax;
    reg [9:0]  n_ymin, n_ymax;
    reg        n_any;
    always @* begin
        n_xmin = x_min_r; n_xmax = x_max_r;
        n_ymin = y_min_r; n_ymax = y_max_r;
        n_any  = any_hit;
        if(roi_mask[0]) begin
            if(!n_any || (col_a < n_xmin)) n_xmin = col_a;
            if(!n_any || (col_a > n_xmax)) n_xmax = col_a;
            if(!n_any || (y_cur < n_ymin)) n_ymin = y_cur;
            if(!n_any || (y_cur > n_ymax)) n_ymax = y_cur;
            n_any = 1'b1;
        end
        if(roi_mask[1]) begin
            if(!n_any || (col_b < n_xmin)) n_xmin = col_b;
            if(!n_any || (col_b > n_xmax)) n_xmax = col_b;
            if(!n_any || (y_cur < n_ymin)) n_ymin = y_cur;
            if(!n_any || (y_cur > n_ymax)) n_ymax = y_cur;
            n_any = 1'b1;
        end
        if(roi_mask[2]) begin
            if(!n_any || (col_c < n_xmin)) n_xmin = col_c;
            if(!n_any || (col_c > n_xmax)) n_xmax = col_c;
            if(!n_any || (y_cur < n_ymin)) n_ymin = y_cur;
            if(!n_any || (y_cur > n_ymax)) n_ymax = y_cur;
            n_any = 1'b1;
        end
        if(roi_mask[3]) begin
            if(!n_any || (col_d < n_xmin)) n_xmin = col_d;
            if(!n_any || (col_d > n_xmax)) n_xmax = col_d;
            if(!n_any || (y_cur < n_ymin)) n_ymin = y_cur;
            if(!n_any || (y_cur > n_ymax)) n_ymax = y_cur;
            n_any = 1'b1;
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            cnt_accum <= 20'd0;
            x_min_r   <= 11'd0; x_max_r <= 11'd0;
            y_min_r   <= 10'd0; y_max_r <= 10'd0;
            any_hit   <= 1'b0;
        end else if(I_tuser) begin
            cnt_accum <= 20'd0;
            x_min_r   <= 11'd0; x_max_r <= 11'd0;
            y_min_r   <= 10'd0; y_max_r <= 10'd0;
            any_hit   <= 1'b0;
        end else if(I_tvalid) begin
            // 面积: 命中的像素数
            cnt_accum <= cnt_accum + {16'd0, roi_mask[0]} + {16'd0, roi_mask[1]} +
                                    {16'd0, roi_mask[2]} + {16'd0, roi_mask[3]};
            // 极值: 单次非阻塞寄存组合结果
            x_min_r   <= n_xmin;
            x_max_r   <= n_xmax;
            y_min_r   <= n_ymin;
            y_max_r   <= n_ymax;
            any_hit   <= n_any;
        end
    end

    // ---- 形态检查: 烟头亮点近似圆形, 宽高比 <= 4 (排除细长反光/线状亮点) ----
    wire [11:0] bbox_w = {1'b0, x_max_r - x_min_r} + 12'd1;   // 包围盒宽
    wire [11:0] bbox_h = {2'b0, y_max_r - y_min_r} + 12'd1;   // 包围盒高
    wire aspect_ok = (bbox_w <= (bbox_h << 2)) && (bbox_h <= (bbox_w << 2));

    wire [7:0] conf_size_step = (cnt_accum[19:7] > 8'd11) ? 8'd11 : cnt_accum[19:7];
    wire [7:0] conf_calc = !any_hit              ? 8'd0  :
                           (cnt_accum < CNT_MIN) ? 8'd40 :
                           (cnt_accum > CNT_MAX) ? 8'd20 :
                           (!aspect_ok)          ? 8'd55 :
                           (8'd88 + conf_size_step);

    // ---- 帧头结算: 锁存上一帧结果 ----
    // tuser 拍锁存 (此时 cnt_accum 已是上一帧累计值, 因 tuser 拍清零在
    // 同一拍非阻塞赋值, 锁存读到的仍是旧值 — 与 detect_ccl_96 语义一致)
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_bbox_xmin <= 11'd0; O_bbox_xmax <= 11'd0;
            O_bbox_ymin <= 10'd0; O_bbox_ymax <= 10'd0;
            O_valid     <= 1'b0;
            O_count     <= 20'd0;
            O_confidence <= 8'd0;
        end else if(I_tuser) begin
            O_bbox_xmin <= x_min_r;
            O_bbox_xmax <= x_max_r;
            O_bbox_ymin <= y_min_r;
            O_bbox_ymax <= y_max_r;
            O_count     <= cnt_accum;
            O_confidence <= conf_calc;
            O_valid     <= any_hit && (cnt_accum >= CNT_MIN) && (cnt_accum <= CNT_MAX) && aspect_ok;
        end
    end

endmodule
