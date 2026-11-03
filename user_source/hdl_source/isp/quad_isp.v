module quad_isp (
    input              I_clk,
    input              I_rst_n,
    input              I_tlast,
    input              I_tuser,
    input              I_tvalid,
    input              I_page_select, // 0=detector views, 1=image-processing views
    input      [95:0]  I_orig_tdata,
    input      [95:0]  I_edge_tdata,
    input      [95:0]  I_mask_tdata,
    output reg         O_tvalid,
    output reg [127:0] O_tdata
);
    reg [8:0] word_cnt;
    reg [9:0] line_cnt;
    reg       frame_par;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            word_cnt <= 9'd0;
            line_cnt <= 10'd0;
            frame_par <= 1'b0;
        end else begin
            if (I_tuser) begin
                line_cnt <= 10'd0;
                frame_par <= ~frame_par;
            end else if (I_tlast) begin
                line_cnt <= line_cnt + 10'd1;
            end

            if (I_tlast)
                word_cnt <= 9'd0;
            else if (I_tvalid)
                word_cnt <= word_cnt + 9'd1;
        end
    end

    wire frame_eff = I_tuser ? ~frame_par : frame_par;
    wire [9:0] line_eff = I_tuser ? 10'd0 : line_cnt;

    reg page_pair;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            page_pair <= 1'b0;
        else if(I_tuser && frame_eff)
            page_pair <= I_page_select;
    end
    wire page_eff = (I_tuser && frame_eff) ? I_page_select : page_pair;

    wire [47:0] orig_pair;
    wire [47:0] edge_pair;
    wire [47:0] mask_pair;

    quad_hv_avg u_hv_orig (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_tvalid(I_tvalid),
        .I_tlast(I_tlast), .I_tuser(I_tuser), .I_word_cnt(word_cnt),
        .I_tdata(I_orig_tdata), .O_v_pair(orig_pair)
    );
    quad_hv_avg u_hv_edge (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_tvalid(I_tvalid),
        .I_tlast(I_tlast), .I_tuser(I_tuser), .I_word_cnt(word_cnt),
        .I_tdata(I_edge_tdata), .O_v_pair(edge_pair)
    );
    quad_hv_avg u_hv_mask (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_tvalid(I_tvalid),
        .I_tlast(I_tlast), .I_tuser(I_tuser), .I_word_cnt(word_cnt),
        .I_tdata(I_mask_tdata), .O_v_pair(mask_pair)
    );
    wire [47:0] detector_pair = !frame_eff ?
                                (line_eff[0] ? edge_pair : orig_pair) :
                                (line_eff[0] ? orig_pair : mask_pair);
    wire [1:0] process_sel = !frame_eff ? (line_eff[0] ? 2'd1 : 2'd0) :
                                          (line_eff[0] ? 2'd3 : 2'd2);

    function [7:0] luma8;
        input [23:0] p;
        reg [9:0] sum;
        begin
            sum = {2'b00,p[23:16]} + {1'b0,p[15:8],1'b0} + {2'b00,p[7:0]};
            luma8 = sum[9:2];
        end
    endfunction
    function [23:0] binary_pixel;
        input [23:0] p;
        begin binary_pixel = (luma8(p) >= 8'd128) ? 24'hFFFFFF : 24'h000000; end
    endfunction
    function [7:0] gamma_channel;
        input [7:0] c;
        reg [9:0] d, v;
        begin
            d = {5'd0,c[4:0]};
            case(c[7:5])
                3'd0: v = (d << 1) + d - (d >> 3);
                3'd1: v = 10'd90 + d + (d >> 2) - (d >> 4);
                3'd2: v = 10'd128 + d - (d >> 3);
                3'd3: v = 10'd156 + d - (d >> 2) + (d >> 5);
                3'd4: v = 10'd181 + (d >> 1) + (d >> 3) + (d >> 5);
                3'd5: v = 10'd202 + (d >> 1) + (d >> 3) - (d >> 5);
                3'd6: v = 10'd221 + (d >> 1) + (d >> 4);
                default: v = 10'd239 + (d >> 1);
            endcase
            if(c == 8'hFF) gamma_channel = 8'hFF;
            else gamma_channel = (v > 10'd255) ? 8'd255 : v[7:0];
        end
    endfunction
    function [23:0] gamma_pixel;
        input [23:0] p;
        begin gamma_pixel = {gamma_channel(p[23:16]),gamma_channel(p[15:8]),gamma_channel(p[7:0])}; end
    endfunction
    function [7:0] sat_channel;
        input [7:0] c, y;
        reg signed [9:0] diff;
        reg signed [10:0] val;
        begin
            diff = $signed({2'b00,c}) - $signed({2'b00,y});
            val = $signed({3'b000,y}) + diff + (diff >>> 1);
            if(val < 0) sat_channel=8'd0;
            else if(val > 11'sd255) sat_channel=8'd255;
            else sat_channel=val[7:0];
        end
    endfunction
    function [23:0] saturation_pixel;
        input [23:0] p;
        reg [7:0] y;
        begin
            y=luma8(p);
            saturation_pixel={sat_channel(p[23:16],y),sat_channel(p[15:8],y),sat_channel(p[7:0],y)};
        end
    endfunction
    function [23:0] heatmap_pixel;
        input [23:0] p;
        reg [7:0] y,r,g,b,d;
        begin
            y=luma8(p);
            if(y<8'd64) begin d=y[5:0]; r=0; g={d,2'b00}; b=8'hFF; end
            else if(y<8'd128) begin d=y-8'd64; r=0; g=8'hFF; b=8'hFF-{d,2'b00}; end
            else if(y<8'd192) begin d=y-8'd128; r={d,2'b00}; g=8'hFF; b=0; end
            else begin d=y-8'd192; r=8'hFF; g=8'hFF-{d,2'b00}; b=0; end
            heatmap_pixel={r,g,b};
        end
    endfunction
    function [47:0] process_transform;
        input [47:0] p;
        input [1:0] mode;
        begin
            case(mode)
                2'd0: process_transform={binary_pixel(p[47:24]),binary_pixel(p[23:0])};
                2'd1: process_transform={gamma_pixel(p[47:24]),gamma_pixel(p[23:0])};
                2'd2: process_transform={saturation_pixel(p[47:24]),saturation_pixel(p[23:0])};
                default: process_transform={heatmap_pixel(p[47:24]),heatmap_pixel(p[23:0])};
            endcase
        end
    endfunction
    wire [47:0] process_pair = process_transform(orig_pair, process_sel);
    wire [47:0] pair_data = page_eff ? process_pair : detector_pair;

    reg [383:0] sh;
    reg [2:0]   pack_count;
    wire [383:0] sh_next = {sh[335:0], pair_data};
    wire group_done = I_tvalid && (pack_count == 3'd7);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            sh <= 384'd0;
            pack_count <= 3'd0;
        end else if (I_tvalid) begin
            sh <= sh_next;
            pack_count <= group_done ? 3'd0 : pack_count + 3'd1;
        end
    end

    reg [127:0] word_b;
    reg [127:0] word_c;
    reg [1:0]   drain_state;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            word_b <= 128'd0;
            word_c <= 128'd0;
            drain_state <= 2'd0;
            O_tvalid <= 1'b0;
            O_tdata <= 128'd0;
        end else begin
            O_tvalid <= 1'b0;

            if (group_done) begin
                O_tvalid <= 1'b1;
                O_tdata <= sh_next[383:256];
                word_b <= sh_next[255:128];
                word_c <= sh_next[127:0];
                drain_state <= 2'd1;
            end else if (drain_state == 2'd1) begin
                O_tvalid <= 1'b1;
                O_tdata <= word_b;
                drain_state <= 2'd2;
            end else if (drain_state == 2'd2) begin
                O_tvalid <= 1'b1;
                O_tdata <= word_c;
                drain_state <= 2'd0;
            end
        end
    end
endmodule
