`timescale 1 ns / 1 ps

module axi_dma_shim #(
    parameter M_AXI_LITE_ADDR_WIDTH = 32,
    parameter M_AXI_LITE_DATA_WIDTH = 32,
    parameter AXIS_DATA_WIDTH       = 64,             // Changed to 64
    parameter AXIS_TKEEP_WIDTH      = AXIS_DATA_WIDTH / 8, // Calc: 8 bits
    parameter DMA_BASE_ADDR         = 32'h41E0_0000 
)(
    input  wire                                 clk,
    input  wire                                 resetn,

    // Friendly interface
    input  wire                                 dma_start_transfer,   
    input  wire                                 dma_direction,        // 1 = MM2S, 0 = S2MM
    input  wire [31:0]                          dma_ddr_addr,         
    input  wire [29:0]                          dma_length_bytes,     
    output reg                                  dma_transfer_done,     

    // AXI-Lite Master to AXI DMA (Control Interface - REMAINS 32-BIT)
    output reg  [M_AXI_LITE_ADDR_WIDTH-1:0]     m_axi_lite_awaddr,
    output reg  [2:0]                           m_axi_lite_awprot,
    output reg                                  m_axi_lite_awvalid,
    input  wire                                 m_axi_lite_awready,

    output reg  [M_AXI_LITE_DATA_WIDTH-1:0]     m_axi_lite_wdata,
    output reg  [(M_AXI_LITE_DATA_WIDTH/8)-1:0] m_axi_lite_wstrb,
    output reg                                  m_axi_lite_wvalid,
    input  wire                                 m_axi_lite_wready,

    input  wire [1:0]                           m_axi_lite_bresp,
    input  wire                                 m_axi_lite_bvalid,
    output reg                                  m_axi_lite_bready,

    output reg  [M_AXI_LITE_ADDR_WIDTH-1:0]     m_axi_lite_araddr,
    output reg  [2:0]                           m_axi_lite_arprot,
    output reg                                  m_axi_lite_arvalid,
    input  wire                                 m_axi_lite_arready,

    input  wire [M_AXI_LITE_DATA_WIDTH-1:0]     m_axi_lite_rdata,
    input  wire [1:0]                           m_axi_lite_rresp,
    input  wire                                 m_axi_lite_rvalid,
    output reg                                  m_axi_lite_rready,

    // ---------------------------------------------------------------------
    // AXIS passthrough (64-bit / Parameterized)
    // ---------------------------------------------------------------------
    
    // DMA MM2S -> shim s_axis
    input  wire [AXIS_DATA_WIDTH-1:0]         s_axis_tdata,
    input  wire [AXIS_TKEEP_WIDTH-1:0]        s_axis_tkeep,
    input  wire                               s_axis_tlast,
    input  wire                               s_axis_tvalid,
    output wire                               s_axis_tready,
    
    // shim m_axis -> Accelerator
    output wire [AXIS_DATA_WIDTH-1:0]         m_axis_accel_tdata,
    output wire [AXIS_TKEEP_WIDTH-1:0]        m_axis_accel_tkeep,
    output wire                               m_axis_accel_tlast,
    output wire                               m_axis_accel_tvalid,
    input  wire                               m_axis_accel_tready,

    // Accelerator -> shim s_axis
    input  wire [AXIS_DATA_WIDTH-1:0]         s_accel_axis_tdata,
    input  wire [AXIS_TKEEP_WIDTH-1:0]        s_accel_axis_tkeep,
    input  wire                               s_accel_axis_tlast,
    input  wire                               s_accel_axis_tvalid,
    output wire                               s_accel_axis_tready,
    
    // shim m_axis -> DMA S2MM
    output wire [AXIS_DATA_WIDTH-1:0]         m_axis_tdata,
    output wire [AXIS_TKEEP_WIDTH-1:0]        m_axis_tkeep,
    output wire                               m_axis_tlast,
    output wire                               m_axis_tvalid,
    input  wire                               m_axis_tready,
    
    output reg [3:0] state, // Exposed for debugging
    output reg [3:0] next_state // Exposed for debugging
);

    // MM2S Pass through
    assign m_axis_accel_tdata  = s_axis_tdata;
    assign m_axis_accel_tkeep  = s_axis_tkeep;
    assign m_axis_accel_tlast  = s_axis_tlast;
    assign m_axis_accel_tvalid = s_axis_tvalid;
    // assign s_axis_tready = m_axis_accel_tready; 
    assign s_axis_tready = 1'b1; // Always ready to accept from DMA, will change when accelerator finishes

    // S2MM Pass through
    assign m_axis_tdata  = s_accel_axis_tdata;
    assign m_axis_tkeep  = s_accel_axis_tkeep;
    assign m_axis_tlast  = s_accel_axis_tlast;
    assign m_axis_tvalid = s_accel_axis_tvalid;
    assign s_accel_axis_tready = m_axis_tready;
    
    // =========================================================================
    // FSM LOGIC (Register configuration is always 32-bit)
    // =========================================================================

    // Register offset AXI DMA 
    localparam MM2S_DMACR   = 32'h00;  
    localparam MM2S_DMASR   = 32'h04;  
    localparam MM2S_SA      = 32'h18;  
    localparam MM2S_LENGTH  = 32'h28;  

    localparam S2MM_DMACR   = 32'h30;
    localparam S2MM_DMASR   = 32'h34;
    localparam S2MM_DA      = 32'h48;  
    localparam S2MM_LENGTH  = 32'h58;

    localparam CR_RUN_STOP  = 32'b1 << 0;  
    localparam IRQ_IOC_EN   = 32'b1 << 12; 
    localparam IRQ_IOC_MASK = 32'b1 << 12; 

    // FSM states 
    localparam ST_IDLE      = 4'd0;
    localparam ST_WR_DMACR  = 4'd1;
    localparam ST_WR_ADDR   = 4'd2;
    localparam ST_WR_LEN    = 4'd3;
    localparam ST_POLL_RD   = 4'd4;
    localparam ST_POLL_WAIT = 4'd5;
    localparam ST_ACK_IRQ   = 4'd6;
    localparam ST_DONE      = 4'd7;

    // Latch 
    reg [31:0] latched_addr;
    reg [31:0] latched_len;
    reg        latched_dir; 

    // Addr of target register
    reg [31:0] reg_dmacr_addr;
    reg [31:0] reg_addr_addr;
    reg [31:0] reg_len_addr;
    reg [31:0] reg_dmasr_addr;

    // Handshake flag
    reg aw_done, w_done, ar_done;
    
    // Edge detector for dma_start_transfer
    reg start_d1, start_d2;
    wire start_pulse = start_d1 & ~start_d2;
    
    always @(posedge clk or negedge resetn) begin
      if (!resetn) begin
        start_d1 <= 1'b0;
        start_d2 <= 1'b0;
      end else begin
        start_d1 <= dma_start_transfer; 
        start_d2 <= start_d1;
      end
    end

    // FSM State Register
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            state <= ST_IDLE;
        end else begin
            state <= next_state;
        end
    end

    // FSM Next State Logic
    always @(*) begin
        next_state = state;
        
        case (state)
            ST_IDLE: begin
                if (start_pulse) begin
                    next_state = ST_WR_DMACR;
                end
            end
            
            ST_WR_DMACR: begin
                if (aw_done && w_done && m_axi_lite_bvalid && m_axi_lite_bready) begin
                    next_state = ST_WR_ADDR;
                end
            end
            
            ST_WR_ADDR: begin
                if (aw_done && w_done && m_axi_lite_bvalid && m_axi_lite_bready) begin
                    next_state = ST_WR_LEN;
                end
            end
            
            ST_WR_LEN: begin
                if (aw_done && w_done && m_axi_lite_bvalid && m_axi_lite_bready) begin
                    next_state = ST_POLL_RD;
                end
            end
            
            ST_POLL_RD: begin
                if (m_axi_lite_rvalid && m_axi_lite_rready) begin
                    // Check IOC (bit 12) or Idle (bit 1)
                    if (m_axi_lite_rdata[12] || m_axi_lite_rdata[1]) begin
                        next_state = ST_ACK_IRQ;
                    end else begin
                        next_state = ST_POLL_RD;
                    end
                end
            end
            
            ST_ACK_IRQ: begin
                if (aw_done && w_done && m_axi_lite_bvalid && m_axi_lite_bready) begin
                    next_state = ST_DONE;
                end
            end
            
            ST_DONE: begin
                next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end

    // FSM Output Logic
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            dma_transfer_done <= 1'b0;

            latched_addr <= 32'd0;
            latched_len  <= 32'd0;
            latched_dir  <= 1'b0;

            reg_dmacr_addr <= 32'd0;
            reg_addr_addr  <= 32'd0;
            reg_len_addr   <= 32'd0;
            reg_dmasr_addr <= 32'd0;

            aw_done <= 1'b0;
            w_done  <= 1'b0;
            ar_done <= 1'b0;

            m_axi_lite_awaddr  <= {M_AXI_LITE_ADDR_WIDTH{1'b0}};
            m_axi_lite_awprot  <= 3'b000;
            m_axi_lite_awvalid <= 1'b0;

            m_axi_lite_wdata   <= {M_AXI_LITE_DATA_WIDTH{1'b0}};
            m_axi_lite_wstrb   <= {(M_AXI_LITE_DATA_WIDTH/8){1'b0}};
            m_axi_lite_wvalid  <= 1'b0;

            m_axi_lite_bready  <= 1'b0;

            m_axi_lite_araddr  <= {M_AXI_LITE_ADDR_WIDTH{1'b0}};
            m_axi_lite_arprot  <= 3'b000;
            m_axi_lite_arvalid <= 1'b0;

            m_axi_lite_rready  <= 1'b0;

        end else begin

            // Default values
            m_axi_lite_awvalid <= 1'b0;
            m_axi_lite_wvalid  <= 1'b0;
            m_axi_lite_arvalid <= 1'b0;
            m_axi_lite_rready  <= 1'b0;
            m_axi_lite_bready  <= 1'b0;

            case (state)
                ST_IDLE: begin
                    aw_done <= 1'b0;
                    w_done  <= 1'b0;
                    ar_done <= 1'b0;

                    if (start_pulse) begin
                        dma_transfer_done <= 0;
                        latched_addr <= dma_ddr_addr;
                        latched_len  <= dma_length_bytes; 
                        latched_dir  <= dma_direction;

                        if (dma_direction) begin
                            // 1 = MM2S
                            reg_dmacr_addr <= DMA_BASE_ADDR + MM2S_DMACR;
                            reg_dmasr_addr <= DMA_BASE_ADDR + MM2S_DMASR;
                            reg_addr_addr  <= DMA_BASE_ADDR + MM2S_SA;
                            reg_len_addr   <= DMA_BASE_ADDR + MM2S_LENGTH;
                        end else begin
                            // 0 = S2MM
                            reg_dmacr_addr <= DMA_BASE_ADDR + S2MM_DMACR;
                            reg_dmasr_addr <= DMA_BASE_ADDR + S2MM_DMASR;
                            reg_addr_addr  <= DMA_BASE_ADDR + S2MM_DA;
                            reg_len_addr   <= DMA_BASE_ADDR + S2MM_LENGTH;
                        end
                    end
                end

                ST_WR_DMACR: begin
                    if (!aw_done) begin
                        m_axi_lite_awvalid <= 1'b1;
                        m_axi_lite_awaddr  <= reg_dmacr_addr;
                        if (m_axi_lite_awvalid && m_axi_lite_awready) aw_done <= 1'b1;
                    end
                    if (!w_done) begin
                        m_axi_lite_wvalid <= 1'b1;
                        m_axi_lite_wdata  <= (CR_RUN_STOP | IRQ_IOC_EN);
                        m_axi_lite_wstrb  <= 4'b1111;
                        if (m_axi_lite_wvalid && m_axi_lite_wready) w_done <= 1'b1;
                    end
                    if (aw_done && w_done) begin
                        m_axi_lite_bready <= 1'b1;
                        if (m_axi_lite_bvalid && m_axi_lite_bready) begin
                          m_axi_lite_bready <= 1'b0;
                          aw_done <= 1'b0; w_done <= 1'b0;    
                        end
                    end 
                end

                ST_WR_ADDR: begin
                    if (!aw_done) begin
                        m_axi_lite_awvalid <= 1'b1;
                        m_axi_lite_awaddr  <= reg_addr_addr;
                        if (m_axi_lite_awvalid && m_axi_lite_awready) aw_done <= 1'b1;
                    end
                    if (!w_done) begin
                        m_axi_lite_wvalid <= 1'b1;
                        m_axi_lite_wdata  <= latched_addr;
                        m_axi_lite_wstrb  <= 4'b1111;
                        if (m_axi_lite_wvalid && m_axi_lite_wready) w_done <= 1'b1;
                    end
                    if (aw_done && w_done) begin
                        m_axi_lite_bready <= 1'b1;
                        if (m_axi_lite_bvalid && m_axi_lite_bready) begin
                          m_axi_lite_bready <= 1'b0;
                          aw_done <= 1'b0; w_done <= 1'b0;    
                        end
                    end 
                end

                ST_WR_LEN: begin
                    if (!aw_done) begin
                        m_axi_lite_awvalid <= 1'b1;
                        m_axi_lite_awaddr  <= reg_len_addr;
                        if (m_axi_lite_awvalid && m_axi_lite_awready) aw_done <= 1'b1;
                    end
                    if (!w_done) begin
                        m_axi_lite_wvalid <= 1'b1;
                        m_axi_lite_wdata  <= latched_len; 
                        m_axi_lite_wstrb  <= 4'b1111;
                        if (m_axi_lite_wvalid && m_axi_lite_wready) w_done <= 1'b1;
                    end
                    if (aw_done && w_done) begin
                        m_axi_lite_bready <= 1'b1;
                        if (m_axi_lite_bvalid && m_axi_lite_bready) begin
                          m_axi_lite_bready <= 1'b0;
                          aw_done <= 1'b0; w_done <= 1'b0;    
                        end
                    end 
                end

                ST_POLL_RD: begin
                    if (!ar_done) begin
                        m_axi_lite_arvalid <= 1'b1;
                        m_axi_lite_araddr  <= reg_dmasr_addr;
                        if (m_axi_lite_arvalid && m_axi_lite_arready) ar_done <= 1'b1;
                    end
                    m_axi_lite_rready <= 1'b1;
                    if (m_axi_lite_rvalid && m_axi_lite_rready) begin
                         ar_done <= 1'b0; 
                    end
                end 

                ST_ACK_IRQ: begin
                    if (!aw_done) begin
                        m_axi_lite_awvalid <= 1'b1;
                        m_axi_lite_awaddr  <= reg_dmasr_addr;
                        if (m_axi_lite_awvalid && m_axi_lite_awready) aw_done <= 1'b1;
                    end
                    if (!w_done) begin
                        m_axi_lite_wvalid <= 1'b1;
                        m_axi_lite_wdata  <= IRQ_IOC_MASK; 
                        m_axi_lite_wstrb  <= 4'b1111;
                        if (m_axi_lite_wvalid && m_axi_lite_wready) w_done <= 1'b1;
                    end
                    if (aw_done && w_done) begin
                        m_axi_lite_bready <= 1'b1;
                        if (m_axi_lite_bvalid && m_axi_lite_bready) begin
                          m_axi_lite_bready <= 1'b0;
                          aw_done <= 1'b0; w_done <= 1'b0;    
                        end
                    end 
                end

                ST_DONE: begin
                    dma_transfer_done <= 1'b1;
                end
                
                default: begin end
            endcase
        end
    end

endmodule