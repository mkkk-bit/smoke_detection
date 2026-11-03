module color_mask_96 #(
    parameter [7:0] RED_MIN       = 8'd160,
    parameter [7:0] GREEN_MIN     = 8'd0,
    parameter [7:0] RED_GREEN_GAP = 8'd64,
    parameter [7:0] RED_BLUE_GAP  = 8'd48,
    parameter [9:0] LUMA_SUM_MIN = 10'd210
)(
    input wire        I_clk,
    input wire        I_rst_n,
    input wire        I_tlast,
    input wire        I_tuser,
    input wire [95:0] I_tdata,
    input wire        I_tvalid,
    input wire [10:0] I_roi_left,
    input wire [10:0] I_roi_right,
    output wire       O_tlast,
    output wire       O_tuser,
    output wire [95:0] O_tdata,
    output wire       O_tvalid,
    output reg [19:0] O_frame_count,
    output wire [3:0] O_candidate,
    // 帧级统计 (帧头锁存上一帧, 供 feature_fuse 四维融合):
    output reg [29:0] O_cand_edge_sum,   // 候选区水平梯度能量 (高对比度边缘特征)
    output reg [29:0] O_luma_sum,        // 全帧亮度 r+g+b (场景亮度 -> 自适应阈值)
    output reg [29:0] O_cand_sat_sum     // 候选区饱和度 (max-min) 和 (色彩纯度特征)
    // 外接框/告警判定由 detect_ccl_96 (连通域检测) 输出, 本模块只出候选与统计
);
    wire [23:0] pixel0 = I_tdata[95:72];
    wire [23:0] pixel1 = I_tdata[71:48];
    wire [23:0] pixel2 = I_tdata[47:24];
    wire [23:0] pixel3 = I_tdata[23:0];

    function is_candidate;
        input [23:0] pixel;
        reg [7:0] r, g, b;
        reg [8:0] g_gap, b_gap;
        reg [9:0] luma_sum;
        begin
            r = pixel[23:16];
            g = pixel[15:8];
            b = pixel[7:0];
            g_gap = {1'b0,g} + {1'b0,RED_GREEN_GAP};
            b_gap = {1'b0,b} + {1'b0,RED_BLUE_GAP};
            luma_sum = {2'b0,r} + {2'b0,g} + {2'b0,b};
            is_candidate = (r >= RED_MIN) && (g >= GREEN_MIN) &&
                           ({1'b0,r} > g_gap) && ({1'b0,r} > b_gap) &&
                           (luma_sum >= LUMA_SUM_MIN);
        end
    endfunction

    reg [8:0] roi_x_beat;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            roi_x_beat <= 9'd0;
        else if(I_tvalid) begin
            if(I_tuser)
                roi_x_beat <= 9'd1; // tuser与首拍像素同拍; 下一拍从x=4开始
            else if(I_tlast || (roi_x_beat == 9'd319))
                roi_x_beat <= 9'd0;
            else
                roi_x_beat <= roi_x_beat + 1'b1;
        end
    end
    wire [10:0] roi_col_base = I_tuser ? 11'd0 : {roi_x_beat, 2'b00};
    wire [10:0] roi_col0 = roi_col_base;
    wire [10:0] roi_col1 = roi_col_base + 11'd1;
    wire [10:0] roi_col2 = roi_col_base + 11'd2;
    wire [10:0] roi_col3 = roi_col_base + 11'd3;
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
    wire [3:0] red_candidate_raw;
    assign red_candidate_raw[0] = is_candidate(pixel0);
    assign red_candidate_raw[1] = is_candidate(pixel1);
    assign red_candidate_raw[2] = is_candidate(pixel2);
    assign red_candidate_raw[3] = is_candidate(pixel3);

    assign O_candidate[0] = red_candidate_raw[0] && roi_inside(roi_col0, I_roi_left, I_roi_right);
    assign O_candidate[1] = red_candidate_raw[1] && roi_inside(roi_col1, I_roi_left, I_roi_right);
    assign O_candidate[2] = red_candidate_raw[2] && roi_inside(roi_col2, I_roi_left, I_roi_right);
    assign O_candidate[3] = red_candidate_raw[3] && roi_inside(roi_col3, I_roi_left, I_roi_right);

    assign O_tdata[95:72] = red_candidate_raw[0] ? 24'h00ff00 : 24'h000000;
    assign O_tdata[71:48] = red_candidate_raw[1] ? 24'h00ff00 : 24'h000000;
    assign O_tdata[47:24] = red_candidate_raw[2] ? 24'h00ff00 : 24'h000000;
    assign O_tdata[23:0]  = red_candidate_raw[3] ? 24'h00ff00 : 24'h000000;
    assign O_tlast = I_tlast;
    assign O_tuser = I_tuser;
    assign O_tvalid = I_tvalid;

    reg [19:0] count_accum;
    wire [2:0] hit_count = {2'b0,O_candidate[0]} + {2'b0,O_candidate[1]} +
                           {2'b0,O_candidate[2]} + {2'b0,O_candidate[3]};

    // ---- 帧级候选像素计数 (外接框跟踪已迁移至 detect_ccl_96 连通域检测) ----
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            count_accum   <= 20'd0;
            O_frame_count <= 20'd0;
        end else if(I_tuser) begin
            // 帧头: 快照上一帧的计数, 复位开始新帧 (tuser与首字同拍: 立即计入首字)
            O_frame_count <= count_accum;
            count_accum   <= I_tvalid ? {{17{1'b0}},hit_count} : 20'd0;
        end else if(I_tvalid) begin
            count_accum <= count_accum + hit_count;
        end
    end

    // ================= 候选区水平梯度能量 (高对比度边缘特征) =================
    function [7:0] abs_diff;
        input [7:0] a, b;
        begin
            abs_diff = (a > b) ? (a - b) : (b - a);
        end
    endfunction

    // ================= 候选区饱和度 (chroma = max(R,G,B) - min(R,G,B)) =================
    function [7:0] sat_of;
        input [23:0] p;
        reg [7:0] r, g, b, mx, mn;
        begin
            r = p[23:16]; g = p[15:8]; b = p[7:0];
            mx = (r >= g) ? ((r >= b) ? r : b) : ((g >= b) ? g : b);
            mn = (r <= g) ? ((r <= b) ? r : b) : ((g <= b) ? g : b);
            sat_of = mx - mn;
        end
    endfunction

    reg [7:0] prev_r, prev_g, prev_b;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            prev_r <= 8'd0; prev_g <= 8'd0; prev_b <= 8'd0;
        end else if(I_tvalid) begin
            prev_r <= pixel3[23:16];
            prev_g <= pixel3[15:8];
            prev_b <= pixel3[7:0];
        end
    end

    wire [9:0] grad0 = {2'b0, abs_diff(pixel0[23:16], prev_r)} +
                       {2'b0, abs_diff(pixel0[15:8],  prev_g)} +
                       {2'b0, abs_diff(pixel0[7:0],   prev_b)};
    wire [9:0] grad1 = {2'b0, abs_diff(pixel1[23:16], pixel0[23:16])} +
                       {2'b0, abs_diff(pixel1[15:8],  pixel0[15:8])} +
                       {2'b0, abs_diff(pixel1[7:0],   pixel0[7:0])};
    wire [9:0] grad2 = {2'b0, abs_diff(pixel2[23:16], pixel1[23:16])} +
                       {2'b0, abs_diff(pixel2[15:8],  pixel1[15:8])} +
                       {2'b0, abs_diff(pixel2[7:0],   pixel1[7:0])};
    wire [9:0] grad3 = {2'b0, abs_diff(pixel3[23:16], pixel2[23:16])} +
                       {2'b0, abs_diff(pixel3[15:8],  pixel2[15:8])} +
                       {2'b0, abs_diff(pixel3[7:0],   pixel2[7:0])};

    wire [11:0] cand_grad_sum =
        (O_candidate[0] ? {2'b0, grad0} : 12'd0) +
        (O_candidate[1] ? {2'b0, grad1} : 12'd0) +
        (O_candidate[2] ? {2'b0, grad2} : 12'd0) +
        (O_candidate[3] ? {2'b0, grad3} : 12'd0);

    wire [9:0] luma0 = {2'b0,pixel0[23:16]} + {2'b0,pixel0[15:8]} + {2'b0,pixel0[7:0]};
    wire [9:0] luma1 = {2'b0,pixel1[23:16]} + {2'b0,pixel1[15:8]} + {2'b0,pixel1[7:0]};
    wire [9:0] luma2 = {2'b0,pixel2[23:16]} + {2'b0,pixel2[15:8]} + {2'b0,pixel2[7:0]};
    wire [9:0] luma3 = {2'b0,pixel3[23:16]} + {2'b0,pixel3[15:8]} + {2'b0,pixel3[7:0]};
    wire [11:0] beat_luma = {2'b0,luma0} + {2'b0,luma1} + {2'b0,luma2} + {2'b0,luma3};

    wire [11:0] cand_sat_sum =
        (O_candidate[0] ? {4'd0, sat_of(pixel0)} : 12'd0) +
        (O_candidate[1] ? {4'd0, sat_of(pixel1)} : 12'd0) +
        (O_candidate[2] ? {4'd0, sat_of(pixel2)} : 12'd0) +
        (O_candidate[3] ? {4'd0, sat_of(pixel3)} : 12'd0);

    reg [29:0] cand_edge_accum;
    reg [29:0] luma_accum;
    reg [29:0] cand_sat_accum;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            cand_edge_accum <= 30'd0;
            luma_accum      <= 30'd0;
            cand_sat_accum  <= 30'd0;
            O_cand_edge_sum <= 30'd0;
            O_luma_sum      <= 30'd0;
            O_cand_sat_sum  <= 30'd0;
        end else if(I_tuser) begin
            O_cand_edge_sum <= cand_edge_accum;
            O_luma_sum      <= luma_accum;
            O_cand_sat_sum  <= cand_sat_accum;
            cand_edge_accum <= I_tvalid ? {18'd0, cand_grad_sum} : 30'd0;
            luma_accum      <= I_tvalid ? {18'd0, beat_luma}    : 30'd0;
            cand_sat_accum  <= I_tvalid ? {18'd0, cand_sat_sum} : 30'd0;
        end else if(I_tvalid) begin
            cand_edge_accum <= cand_edge_accum + {18'd0, cand_grad_sum};
            luma_accum      <= luma_accum      + {18'd0, beat_luma};
            cand_sat_accum  <= cand_sat_accum  + {18'd0, cand_sat_sum};
        end
    end
endmodule
