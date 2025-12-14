module top_DMA_VDMA (
    input clk125,                     // Nối vào chân sys_clock của Zynq

    // -------------------------------------------------------------------------
    // CÁC PORT CỦA ZYNQ/DDR (Copy y chang từ HDMI_wrapper.v)
    // -------------------------------------------------------------------------
    inout [14:0] DDR_addr,
    inout [2:0]  DDR_ba,
    inout        DDR_cas_n,
    inout        DDR_ck_n,
    inout        DDR_ck_p,
    inout        DDR_cke,
    inout        DDR_cs_n,
    inout [3:0]  DDR_dm,
    inout [31:0] DDR_dq,
    inout [3:0]  DDR_dqs_n,
    inout [3:0]  DDR_dqs_p,
    inout        DDR_odt,
    inout        DDR_ras_n,
    inout        DDR_reset_n,
    inout        DDR_we_n,
    inout        FIXED_IO_ddr_vrn,
    inout        FIXED_IO_ddr_vrp,
    inout [53:0] FIXED_IO_mio,
    inout        FIXED_IO_ps_clk,
    inout        FIXED_IO_ps_porb,
    inout        FIXED_IO_ps_srstb,

    // -------------------------------------------------------------------------
    // OUTPUT HDMI VẬT LÝ
    // -------------------------------------------------------------------------
    output       tmds_tx_clk_p,
    output       tmds_tx_clk_n,
    output [2:0] tmds_tx_data_p,
    output [2:0] tmds_tx_data_n
);

    // =========================================================================
    // 1. KHAI BÁO DÂY TÍN HIỆU NỘI BỘ
    // =========================================================================
    
    // Clock lấy từ Block Design
    wire pixel_clk;   // 74.25 MHz
    wire serdes_clk;  // 371.25 MHz
    
    // Video signal lấy từ Block Design
    wire [23:0] vid_data;
    wire        vid_active;
    wire        vid_hsync;
    wire        vid_vsync;

    // Tín hiệu điều khiển HDMI PHY
    wire [9:0] tmds_data [0:2];
    wire [1:0] ctl [0:2];
    wire locked;
    reg [7:0] rstcnt = 0;

    // =========================================================================
    // 2. GỌI MODULE WRAPPER (Map đúng tên trong wrapper của bạn)
    // =========================================================================
    AXI_DMA_system_wrapper system_i (
        // Clock đầu vào (125MHz)
        .sys_clock(clk125),

        // Các chân DDR/Fixed IO (Nối thẳng ra port)
        .DDR_addr(DDR_addr),
        .DDR_ba(DDR_ba),
        .DDR_cas_n(DDR_cas_n),
        .DDR_ck_n(DDR_ck_n),
        .DDR_ck_p(DDR_ck_p),
        .DDR_cke(DDR_cke),
        .DDR_cs_n(DDR_cs_n),
        .DDR_dm(DDR_dm),
        .DDR_dq(DDR_dq),
        .DDR_dqs_n(DDR_dqs_n),
        .DDR_dqs_p(DDR_dqs_p),
        .DDR_odt(DDR_odt),
        .DDR_ras_n(DDR_ras_n),
        .DDR_reset_n(DDR_reset_n),
        .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
        .FIXED_IO_mio(FIXED_IO_mio),
        .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),

        // VIDEO INTERFACE & CLOCKS
        // (Tên bên trái là tên port trong wrapper, tên bên phải là dây trong module top)
        
        .pixel_clk_0   (pixel_clk),    // Output clock 74.25MHz
        .serdes_clk_0  (serdes_clk),   // Output clock 371.25MHz
        .locked_0      (locked),
        
        .vid_io_out_0_active_video (vid_active),
        .vid_io_out_0_data         (vid_data),
        .vid_io_out_0_hsync        (vid_hsync),
        .vid_io_out_0_vsync        (vid_vsync),
        
        // Các chân này wrapper có nhưng ta không dùng đến -> bỏ trống
        .vid_io_out_0_field        (), 
        .vid_io_out_0_hblank       (),
        .vid_io_out_0_vblank       ()
    );

    // =========================================================================
    // 3. LOGIC HDMI PHY (Dùng lại code tmds_encode/oserdes cũ)
    // =========================================================================

    // Gán tín hiệu điều khiển (Blue Channel mang Sync)
    assign ctl[0] = {vid_vsync, vid_hsync};
    assign ctl[1] = 2'b00;
    assign ctl[2] = 2'b00;
    
    always @(posedge pixel_clk or negedge locked)
    begin
        if (~locked) begin
            rstcnt <= 0;
    end else begin
            if (rstcnt != 8'hff) begin
                rstcnt <= rstcnt + 1;
            end
        end
    end

assign rst = (rstcnt == 8'hff) ? 1'b0 : 1'b1;

    // --- Channel 0: Blue (Bits [7:0]) ---
    tmds_encode enc_b (
        .pixel_clk(pixel_clk), 
        .rst(rst), // Reset = 0 (Hoạt động bình thường)
        .ctl(ctl[0]), 
        .active(vid_active), 
        .pdata(vid_data[7:0]), 
        .tmds_data(tmds_data[0])
    );
    tmds_oserdes ser_b (
        .pixel_clk(pixel_clk), 
        .serdes_clk(serdes_clk), 
        .rst(rst), 
        .tmds_data(tmds_data[0]), 
        .tmds_serdes_p(tmds_tx_data_p[0]), 
        .tmds_serdes_n(tmds_tx_data_n[0])
    );

    // --- Channel 1: Green (Bits [15:8]) ---
    tmds_encode enc_g (
        .pixel_clk(pixel_clk), 
        .rst(rst), 
        .ctl(ctl[1]), 
        .active(vid_active), 
        .pdata(vid_data[15:8]), 
        .tmds_data(tmds_data[1])
    );
    tmds_oserdes ser_g (
        .pixel_clk(pixel_clk), 
        .serdes_clk(serdes_clk), 
        .rst(rst), 
        .tmds_data(tmds_data[1]), 
        .tmds_serdes_p(tmds_tx_data_p[1]), 
        .tmds_serdes_n(tmds_tx_data_n[1])
    );

    // --- Channel 2: Red (Bits [23:16]) ---
    tmds_encode enc_r (
        .pixel_clk(pixel_clk), 
        .rst(rst), 
        .ctl(ctl[2]), 
        .active(vid_active), 
        .pdata(vid_data[23:16]), 
        .tmds_data(tmds_data[2])
    );
    tmds_oserdes ser_r (
        .pixel_clk(pixel_clk), 
        .serdes_clk(serdes_clk), 
        .rst(rst), 
        .tmds_data(tmds_data[2]), 
        .tmds_serdes_p(tmds_tx_data_p[2]), 
        .tmds_serdes_n(tmds_tx_data_n[2])
    );

    // --- Clock Channel ---
    tmds_oserdes ser_c (
        .pixel_clk(pixel_clk), 
        .serdes_clk(serdes_clk), 
        .rst(rst), 
        .tmds_data(10'b1111100000), // Pattern clock 1111100000
        .tmds_serdes_p(tmds_tx_clk_p), 
        .tmds_serdes_n(tmds_tx_clk_n)
    );

endmodule