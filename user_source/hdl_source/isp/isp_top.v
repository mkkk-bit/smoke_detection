module isp_top (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
	input [19:0]    I_min_area,
	input [7:0]     I_lum_th,       // 烟头暗光亮点亮度阈值 (threshold_calib, csi域)
	input [10:0]    I_roi_left,
	input [10:0]    I_roi_right,
    input           I_quad_page,
    input 			I_edge_key		,	// KEY1: 边缘检测切换按键(低有效, 直连板级I_button[1])
    output [127:0] 	O_tdata			,
    output [19:0]   O_candidate_count,
    // 四分屏拼接帧 (quad_isp: 三路2x2平均 -> 1280x720拼接位流, 每2源帧1拼接帧)
    // 旁路自消费: 不参与 tready 流控/不插入数据通路, 检测链与主通路不受影响
    output [127:0]  O_quad_tdata,
    output          O_quad_tvalid,
    // 多目标外接框 (detect_ccl_96 帧头快照, 按面积降序, 帧内稳定) -> OSD框选
    output [21:0]   O_bbox_xmin,     // [i*11+:11] = 第i大目标 (MAX_OBJ=2)
    output [21:0]   O_bbox_xmax,
    output [19:0]   O_bbox_ymin,     // [i*10+:10] = 第i大目标
    output [19:0]   O_bbox_ymax,
    output [1:0]    O_bbox_valid,    // bit i = 第i大目标有效
    output [39:0]   O_area,          // [i*20+:20] = 第i大目标面积 (标定观察)
    output [19:0]   O_min_area,      // 自适应面积阈值 (feature_fuse, 仪表盘观察)
    output          O_fire_valid,    // 四维融合火情判定 (feature_fuse, 帧级)
    output [7:0]    O_confidence,    // 检测置信度 0~99 (feature_fuse)
    // 烟头暗光亮点检测 (luma_spot + smoke_bbox, 帧级)
    output [10:0]   O_cig_xmin,
    output [10:0]   O_cig_xmax,
    output [9:0]    O_cig_ymin,
    output [9:0]    O_cig_ymax,
    output          O_cig_valid,     // 烟头判定: 亮点面积在 [CNT_MIN,CNT_MAX]
    output [19:0]   O_cig_count,     // 烟头亮点像素数 (标定观察)
    output [7:0]    O_cig_confidence,// 烟头置信度 0~100
    output [95:0]   O_snap_tdata,    // AWB RGB888 96bit 流 (TF卡告警快照源)
    output          O_snap_tuser,
    output          O_snap_tvalid,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready
);

wire         m_aixs_tvalid;  //synthesis keep
  wire [127:0] m_aixs_tdata;  //synthesis keep
  wire         m_aixs_tuser;  //synthesis keep
  wire         m_aixs_tlast;  //synthesis keep
  wire         m_aixs_tready;  //synthesis keep

  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep


  wire         awb_O_tlast;  //synthesis keep
  wire         awb_O_tuser;  //synthesis keep
  wire [ 95:0] awb_O_tdata;  //synthesis keep
  wire         awb_O_tvalid;  //synthesis keep
  wire         awb_O_tready;  //synthesis keep

  wire         edge_O_tlast;  //synthesis keep
  wire         edge_O_tuser;  //synthesis keep
  wire [ 95:0] edge_O_tdata;  //synthesis keep
  wire         edge_O_tvalid;  //synthesis keep
  wire         edge_O_tready;  //synthesis keep  // 仅承接 edge 组合直通输出, 不再回接(防组合环)

  wire         mask_O_tlast;
  wire         mask_O_tuser;
  wire [95:0]   mask_O_tdata;
  wire         mask_O_tvalid;
  wire [3:0]    mask_candidate;
  wire [29:0]   mask_cand_edge_sum;  // 候选区边缘能量 (color_mask -> feature_fuse)
  wire [29:0]   mask_luma_sum;       // 场景亮度 (color_mask -> feature_fuse)
  wire [29:0]   mask_cand_sat_sum;   // 候选区饱和度 (color_mask -> feature_fuse)
  wire [19:0]   fuse_min_area;       // 自适应面积阈值 (feature_fuse -> detect_ccl)
  wire          fuse_fire_valid;     // 融合判定 (feature_fuse -> 顶层)
  wire [7:0]    fuse_confidence;     // 检测置信度 (feature_fuse -> 顶层)

reg [10:0] S_roi_left_frame, S_roi_right_frame;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        S_roi_left_frame  <= 11'd320;
        S_roi_right_frame <= 11'd960;
    end else if(awb_O_tuser) begin
        S_roi_left_frame  <= I_roi_left;
        S_roi_right_frame <= I_roi_right;
    end
end
wire [10:0] S_roi_left_effective  = awb_O_tuser ? I_roi_left  : S_roi_left_frame;
wire [10:0] S_roi_right_effective = awb_O_tuser ? I_roi_right : S_roi_right_frame;

  // ---- 烟头暗光亮点检测 (luma_spot -> smoke_bbox) ----
  wire [3:0]    luma_mask;           // 亮点掩码 (每拍4bit)
  wire          luma_O_tlast, luma_O_tuser, luma_O_tvalid;
  wire [95:0]   luma_O_tdata;        // 亮点伪彩 (暂不接显示通路)

  wire         awb_O_tuser_128;  //synthesis keep
  wire         awb_O_tvalid_128;  //synthesis keep
  wire [127:0] awb_O_tdata_128;  //synthesis keep

  wire S_edge_key_trig;
  reg [1:0] view_mode;  // 0 raw, 1 edge, 2 color mask
  key_remove_shakes u_edge_key_shakes(
      .I_clk          (axi4s_video_aclk),
      .I_rst_n        (I_rst_n),
      .I_key_in       (~I_edge_key),
      .O_key_trig_out (S_edge_key_trig)
  );
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if(!I_rst_n) view_mode <= 2'd0;
      else if(S_edge_key_trig) begin
          if(view_mode == 2'd2) view_mode <= 2'd0;
          else view_mode <= view_mode + 2'd1;
      end
  end

  wire         view_tuser;
  wire         view_tvalid;
  wire [95:0]  view_tdata;
  assign view_tuser  = (view_mode == 2'd1) ? edge_O_tuser  :
                       (view_mode == 2'd2) ? mask_O_tuser  : awb_O_tuser;
  assign view_tvalid = (view_mode == 2'd1) ? edge_O_tvalid :
                       (view_mode == 2'd2) ? mask_O_tvalid : awb_O_tvalid;
  assign view_tdata  = (view_mode == 2'd1) ? edge_O_tdata  :
                       (view_mode == 2'd2) ? mask_O_tdata  : awb_O_tdata;
  // ===================== 改动1: tready 跨时钟域同步 + 组合环修复 =====================
  // 原实现 (组合环根因, TD6.2 TMR-2509):
  //   assign awb_O_tready = edge_O_tready;
  //   assign edge_O_tready = O_tready;
  // 1) edge_O_tready 同时被 assign 与 edge_detect_96 的 O_tready 输出端口驱动
  //    -> 多驱动网表, SYN-5066 断开其中一个驱动;
  // 2) edge_detect_96 内部 assign O_tready = I_tready 纯组合直通, 其 I_tready 又接
  //    awb_O_tready, 与 assign 别名构成纯组合自反馈环:
  //    awb_O_tready -> edge.I_tready -> edge.O_tready -> edge_O_tready -> awb_O_tready
  // 修正: O_tready(下游就绪, 跨时钟域输入) 经双触发器同步到 axi4s_video_aclk 域,
  //       D触发器打断组合环; 同步后的 O_tready_syn 直通 awb 输出流控。
  //       edge_O_tready 不再被 assign 驱动, 仅承接 edge 端口输出且不使用。
  // AXI-Stream 语义: tvalid 不依赖 tready; 就绪允许寄存(协议允许), 2 拍建立
  //       延迟由上游 FIFO 裕量吸收, 不丢数据; 复位期 tready=0 保持握手持稳。
  reg O_tready_sync1;
  reg O_tready_sync2;
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          O_tready_sync1 <= 1'b0;
          O_tready_sync2 <= 1'b0;
      end else begin
          O_tready_sync1 <= O_tready;       // 第1级: 异步输入亚稳态隔离
          O_tready_sync2 <= O_tready_sync1; // 第2级: 稳定采样
      end
  end
  wire O_tready_syn = O_tready_sync2;

  assign awb_O_tready = O_tready_syn;   // 下游就绪(已同步) -> awb 输出流控
  // edge_O_tready 不再被 assign 驱动, 仅由 edge_detect_96 输出端口驱动且不回接

 
    assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = awb_O_tlast;
  assign O_tuser  = awb_O_tuser_128;
  assign O_tvalid = awb_O_tvalid_128;

  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;


demosaic #(
    .IMG_HEIGHT          (720 ), // 图像高度
    .IMG_WIDTH           (1280), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);

data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

awb #(
    .IMG_HEIGHT(720 ),
    .IMG_WIDTH (1280)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

// ---- Red/orange candidate mask and full-frame candidate-pixel counter ----
color_mask_96 #(
    .RED_MIN       (8'd160),
    .GREEN_MIN     (8'd0),
    .RED_GREEN_GAP (8'd64),
    .RED_BLUE_GAP  (8'd48),
    .LUMA_SUM_MIN  (10'd210)
) u_color_mask_96 (
    .I_clk          (axi4s_video_aclk),
    .I_rst_n        (I_rst_n),
    .I_tlast        (awb_O_tlast),
    .I_tuser        (awb_O_tuser),
    .I_tdata        (awb_O_tdata),
    .I_tvalid       (awb_O_tvalid),
    .I_roi_left     (S_roi_left_effective),
    .I_roi_right    (S_roi_right_effective),
    .O_tlast        (mask_O_tlast),
    .O_tuser        (mask_O_tuser),
    .O_tdata        (mask_O_tdata),
    .O_tvalid       (mask_O_tvalid),
    .O_frame_count  (O_candidate_count),
    .O_candidate    (mask_candidate),
    .O_cand_edge_sum(mask_cand_edge_sum),
    .O_luma_sum     (mask_luma_sum),
    .O_cand_sat_sum (mask_cand_sat_sum)
);
// ---- 连通域检测: 红色候选 → 多目标外接框+面积 ----
// 4邻域 union-find 合并, 按面积降序输出全部达标连通域 (MAX_OBJ=2);
// O_bbox_valid[i]=第i大连通域面积>=帧锁存的 I_min_area (复位初值64)
// (绿色像素/掩码显示色不产生候选, 不触发)
reg [19:0] S_min_area_frame;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if(!I_rst_n)
        S_min_area_frame <= 20'd64;
    else if(awb_O_tuser)
        S_min_area_frame <= I_min_area;
end

detect_ccl_96 #(
    .IMG_WIDTH (1280),
    .IMG_HEIGHT(720)
) u_detect_ccl_96 (
    .I_clk        (axi4s_video_aclk),
    .I_rst_n      (I_rst_n),
    .I_candidate  (mask_candidate),
    .I_tuser      (awb_O_tuser),
    .I_tlast      (awb_O_tlast),
    .I_tvalid     (awb_O_tvalid),
    .I_min_area   (fuse_min_area),
    .O_bbox_xmin  (O_bbox_xmin),
    .O_bbox_xmax  (O_bbox_xmax),
    .O_bbox_ymin  (O_bbox_ymin),
    .O_bbox_ymax  (O_bbox_ymax),
    .O_bbox_valid (O_bbox_valid),
    .O_area       (O_area)
);
// ---- 四维特征帧级融合: 候选数/边缘能量/亮度 -> 自适应面积阈值 + 火情判定 ----
// I_area_base 接 S_min_area_frame (按键K3/K4手动门槛的帧锁存值), 自适应在其上叠加
feature_fuse u_feature_fuse(
    .I_clk            (axi4s_video_aclk),
    .I_rst_n          (I_rst_n),
    .I_tuser          (awb_O_tuser),
    .I_candidate_count(O_candidate_count),
    .I_cand_edge_sum  (mask_cand_edge_sum),
    .I_luma_sum       (mask_luma_sum),
    .I_cand_sat_sum   (mask_cand_sat_sum),
    .I_bbox_valid     (O_bbox_valid),
    .I_area_base      (S_min_area_frame),
    .O_min_area       (fuse_min_area),
    .O_fire_valid     (fuse_fire_valid),
    .O_confidence     (fuse_confidence)
);
// 自适应面积阈值导出 (仪表盘 OSD 观察)
assign O_min_area = fuse_min_area;
// ---- 烟头暗光亮点检测: luma_spot(亮点掩码) -> smoke_bbox(包围盒+面积判定) ----
// 常驻并行, 与明火检测链(color_mask->detect_ccl)同源、互不影响。
luma_spot #(
    .IMG_WIDTH (1280),
    .IMG_HEIGHT(720)
) u_luma_spot (
    .I_clk      (axi4s_video_aclk),
    .I_rst_n    (I_rst_n),
    .I_tlast    (awb_O_tlast),
    .I_tuser    (awb_O_tuser),
    .I_tvalid   (awb_O_tvalid),
    .I_tdata    (awb_O_tdata),
    .I_lum_th   (I_lum_th),
    .O_mask     (luma_mask),
    .O_tdata    (luma_O_tdata),
    .O_tlast    (luma_O_tlast),
    .O_tuser    (luma_O_tuser),
    .O_tvalid   (luma_O_tvalid)
);
smoke_bbox_96 #(
    .IMG_WIDTH (1280),
    .IMG_HEIGHT(720)
) u_smoke_bbox_96 (
    .I_clk       (axi4s_video_aclk),
    .I_rst_n     (I_rst_n),
    .I_mask      (luma_mask),
    .I_roi_left  (S_roi_left_effective),
    .I_roi_right (S_roi_right_effective),
    .I_tuser     (awb_O_tuser),
    .I_tlast     (awb_O_tlast),
    .I_tvalid    (awb_O_tvalid),
    .O_bbox_xmin (O_cig_xmin),
    .O_bbox_xmax (O_cig_xmax),
    .O_bbox_ymin (O_cig_ymin),
    .O_bbox_ymax (O_cig_ymax),
    .O_valid     (O_cig_valid),
    .O_count     (O_cig_count),
    .O_confidence(O_cig_confidence)
);
assign O_fire_valid = fuse_fire_valid;
assign O_confidence = fuse_confidence;
assign O_snap_tdata  = awb_O_tdata;
assign O_snap_tuser  = awb_O_tuser;
assign O_snap_tvalid = awb_O_tvalid;
// ---- 边缘检测 (lab_ex2功能, 与原图通路并行运行) ----
edge_detect_96 #(
    .IMG_WIDTH (1280),
    .IMG_HEIGHT(720),
    .EDGE_TH   (11'd80)
) u_edge_detect_96 (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (awb_O_tlast),
    .I_tuser (awb_O_tuser),
    .I_tdata (awb_O_tdata),
    .I_tvalid(awb_O_tvalid),
    .I_tready(awb_O_tready),
    .O_tlast (edge_O_tlast),
    .O_tuser (edge_O_tuser),
    .O_tdata (edge_O_tdata),
    .O_tvalid(edge_O_tvalid),
    .O_tready(edge_O_tready)
);

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),

        .I_96b_frame_start  ( view_tuser	    ),
        .I_96b_valid        ( view_tvalid 		),
        .I_96b_data         ( view_tdata		),

        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );

    reg quad_page_sync1, quad_page_sync2;
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if(!I_rst_n) begin quad_page_sync1 <= 1'b0; quad_page_sync2 <= 1'b0; end
        else begin quad_page_sync1 <= I_quad_page; quad_page_sync2 <= quad_page_sync1; end
    end

    // ---- Four-pane mosaic: diagnostic page or lightweight processing page ----
    quad_isp u_quad_isp(
        .I_clk         (axi4s_video_aclk),
        .I_rst_n       (I_rst_n),
        .I_tlast       (awb_O_tlast),
        .I_tuser       (awb_O_tuser),
        .I_tvalid      (awb_O_tvalid),
        .I_page_select (quad_page_sync2),
        .I_orig_tdata  (awb_O_tdata),
        .I_edge_tdata  (edge_O_tdata),
        .I_mask_tdata  (mask_O_tdata),
        .O_tdata       (O_quad_tdata),
        .O_tvalid      (O_quad_tvalid)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);

endmodule
