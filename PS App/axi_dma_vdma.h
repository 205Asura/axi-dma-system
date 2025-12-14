#ifndef AXI_DMA_VDMA_H
#define AXI_DMA_VDMA_H

#include "xil_printf.h"
#include "xgpio.h"
#include "xaxivdma.h"




extern XGpio GpioOut;
extern XGpio GpioIn;

extern XAxiVdma AxiVdma;
extern XAxiVdma_DmaSetup ReadCfg;

/******************* Function Prototypes ************************************/
void debug_gpio_status();
void dma_start_transfer(u32 addr, u32 length_bytes, u8 direction);
int dma_wait_for_completion(int timeout_ms);
int test_mm2s();
int test_s2mm();

int ReadSetup(XAxiVdma *InstancePtr);
int StartTransfer(XAxiVdma *InstancePtr);
void Resize_Load_Image_To_DDR();

void Update_Classification_From_Memory();
void DrawChar(u32 *frame, int x, int y, char c, u32 color, int scale);
void DrawString(u32 *frame, int x, int y, const char *str, u32 color, int scale);




#define GPIO_OUT_BASEADDR   XPAR_AXI_GPIO_0_BASEADDR
#define GPIO_IN_BASEADDR    XPAR_AXI_GPIO_1_BASEADDR

#define GPIO_ADDR_CHANNEL       1   // dma_ddr_addr[31:0]
#define GPIO_LEN_CHANNEL        2   // {dma_length_bytes[29:0], dma_direction, dma_start_transfer}

// ĐÚNG: Bit positions trong GPIO_LEN_CHANNEL
#define LENGTH_START_BIT    0
#define LENGTH_DIR_BIT      1  
#define LENGTH_DATA_BITS    30 // bits [31:2] cho length

#define GPIO_STATUS_CHANNEL     1   // dma_transfer_done

/* Cấu hình màn hình 720p */
#define H_RES_PIXELS        1280
#define V_RES_LINES         720
#define BYTES_PER_PIXEL     4
#define H_STRIDE            (H_RES_PIXELS * BYTES_PER_PIXEL)
#define FRAME_SIZE          (H_STRIDE * V_RES_LINES) // total of bytes for the 1280x720 image

/* Kích thước đích (ViT Input Size) */
#define TARGET_WIDTH        224
#define TARGET_HEIGHT       224

#define IMG_POS_Y            248
#define IMG_POS_X            300
#define TEXT_POS_Y           360
#define TEXT_POS_X           IMG_POS_X + TARGET_WIDTH + 200

#define TEXT_COLOR           0xFFFFFFFF

#define IMG_OFFSET_BYTES        ((IMG_POS_Y * H_STRIDE) + (IMG_POS_X * 4))

#define DDR_BASE_HDMI           (XPAR_PS7_DDR_0_BASEADDRESS + 0x10000000) // address for the whole pixel data to show on HDMI
#define DDR_BASE_IMG_INPUT      (DDR_BASE_HDMI + IMG_OFFSET_BYTES) // address of the pixel data of the image only (224x224)

#define TX_BUFFER_BASE      DDR_BASE_IMG_INPUT // reading from the base addr of image data
#define RX_BUFFER_BASE      (XPAR_PS7_DDR_0_BASEADDRESS + 0x00300000)
#define TEST_PKT_LEN_BYTES 32  // Test với 1KB trước


#define DDR_BASE_TEXT_FROM_VIT  (XPAR_PS7_DDR_0_BASEADDRESS + 0x12000000)

/* Chỉ dùng 1 Frame Buffer cho VDMA (Continuous Read) */
#define NUMBER_OF_FRAME_SETS  1





#endif 
