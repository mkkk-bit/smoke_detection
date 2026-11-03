module alarm_module #(
    parameter [15:0] DEBOUNCE = 16'd30,   // 防抖帧数: 连续30帧@60fps≈0.5s
    parameter [15:0] HOLD     = 16'd30    // 告警保持帧数: 0.5s@60fps (与 wrapper 实例化一致)
)(
    input         I_clk,           // 像素时钟 (74.25MHz)
    input         I_rst_n,
    input         I_vsync,         // 场同步 (帧起始)
    input         I_hsync,         // 行同步 (预留: 可扩展行级触发)
    input         I_fire_valid,    // 明火目标有效标志 (帧级, 帧内稳定)
    input         I_cig_valid,     // 烟头目标有效标志 (帧级, 帧内稳定)
    output wire   O_fire_alarm,    // 明火告警 (电平) -> LED1
    output wire   O_cig_alarm,     // 烟头告警 (电平) -> LED2
    output wire   O_sys_alarm,     // 总告警 = 明火|烟头 (电平)
    output reg    O_snap_trig      // 存图触发: 总告警上升沿1拍脉冲 -> TF卡
);

// ---- 帧起始脉冲 (vsync 上升沿检测) ----
reg vs_d1;
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n)
        vs_d1 <= 1'b0;
    else
        vs_d1 <= I_vsync;
end
wire frame_tick = I_vsync && !vs_d1;

// ---- 每类独立: 防抖计数 + 保持计数 + 告警电平 ----
// 类别索引: [0]=明火 [1]=烟头
wire [1:0] cand = {I_cig_valid, I_fire_valid};
reg  [1:0] alarm;

genvar gi;
generate
for(gi=0; gi<2; gi=gi+1) begin : GEN_ALARM
    reg [15:0] deb_cnt;    // 连续有效帧计数 (饱和于 DEBOUNCE)
    reg [15:0] hold_cnt;   // 撤警保持窗口 (有效帧刷新为 HOLD, 无效帧递减)

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            deb_cnt   <= 16'd0;
            hold_cnt  <= 16'd0;
            alarm[gi] <= 1'b0;
        end else if(frame_tick) begin
            if(cand[gi]) begin
                // 目标在: 防抖计数饱和递增, 保持窗口持续刷新
                if(deb_cnt != DEBOUNCE)
                    deb_cnt <= deb_cnt + 1'b1;
                hold_cnt <= HOLD;
            end else begin
                // 目标不在: 防抖计数清零, 保持窗口递减
                deb_cnt <= 16'd0;
                if(hold_cnt != 16'd0)
                    hold_cnt <= hold_cnt - 1'b1;
            end
            // 触发: 连续 DEBOUNCE 帧有效
            if(cand[gi] && (deb_cnt == DEBOUNCE - 1'b1))
                alarm[gi] <= 1'b1;
            // 撤警: 目标消失且保持窗口耗尽
            else if(!cand[gi] && (hold_cnt == 16'd1))
                alarm[gi] <= 1'b0;
        end
    end
end
endgenerate

assign O_fire_alarm = alarm[0];
assign O_cig_alarm  = alarm[1];
assign O_sys_alarm  = alarm[0] | alarm[1];

// ---- 总告警上升沿 -> 1拍存图触发脉冲 (接TF卡存图逻辑) ----
reg sys_alarm_d;
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        sys_alarm_d <= 1'b0;
        O_snap_trig <= 1'b0;
    end else begin
        sys_alarm_d <= alarm[0] | alarm[1];
        O_snap_trig <= (alarm[0] | alarm[1]) && !sys_alarm_d;
    end
end

endmodule
