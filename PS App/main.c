#include "xparameters.h"
#include "xgpio.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "sleep.h"

#include "axi_dma_vdma.h"

int main(void)
{
    // --- Init ---
    XGpio_Config *cfg;
    int Status;
    XAxiVdma_Config *Config;
    XAxiVdma AxiVdma;

    xil_printf("\r\n--- START HDMI RESIZE SINGLE IMAGE ---\r\n");

    /* 1. Resize và Load ảnh vào DDR */
    Resize_Load_Image_To_DDR();
    Update_Classification_From_Memory();
    /* 2. Khởi tạo VDMA */
    Config = XAxiVdma_LookupConfig(XPAR_XAXIVDMA_0_BASEADDR);
    if (!Config) {
        xil_printf("No Video DMA found for ID %d\r\n", XPAR_XAXIVDMA_0_BASEADDR);
        return XST_FAILURE;
    }

    Status = XAxiVdma_CfgInitialize(&AxiVdma, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    /* 3. Cấu hình VDMA */
    Status = ReadSetup(&AxiVdma);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    /* 4. Bắt đầu truyền */
    Status = StartTransfer(&AxiVdma);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    xil_printf("VDMA Running. Resized Image Displayed.\r\n");

    xil_printf("\r\n=== AXI DMA Shim – Per-direction Tests ===\r\n");

    

    cfg = XGpio_LookupConfig(GPIO_OUT_BASEADDR);
    if (!cfg) {
        xil_printf("Lookup GPIO_OUT failed\r\n");
        return XST_FAILURE;
    }
    Status = XGpio_CfgInitialize(&GpioOut, cfg, cfg->BaseAddress);
    if (Status != XST_SUCCESS) {
        xil_printf("GPIO_OUT init failed\r\n");
        return XST_FAILURE;
    }

    cfg = XGpio_LookupConfig(GPIO_IN_BASEADDR);
    if (!cfg) {
        xil_printf("Lookup GPIO_IN failed\r\n");
        return XST_FAILURE;
    }
    Status = XGpio_CfgInitialize(&GpioIn, cfg, cfg->BaseAddress);
    if (Status != XST_SUCCESS) {
        xil_printf("GPIO_IN init failed\r\n");
        return XST_FAILURE;
    }

    // direction: OUT → điều khiển shim, IN → đọc dma_transfer_done
    XGpio_SetDataDirection(&GpioOut, GPIO_ADDR_CHANNEL, 0x00000000); // outputs
    XGpio_SetDataDirection(&GpioOut, GPIO_LEN_CHANNEL,  0x00000000); // outputs
    XGpio_SetDataDirection(&GpioIn,  GPIO_STATUS_CHANNEL, 0x00000001); // bit0 input

    // --- chạy từng testcase ---
    if (test_mm2s() != XST_SUCCESS) {
        xil_printf("MM2S test FAILED\r\n");
        return XST_FAILURE;
    }

    // if (test_s2mm() != XST_SUCCESS) {
    //     xil_printf("S2MM test FAILED\r\n");
    //     return XST_FAILURE;
    // }

    xil_printf("\r\nAll per-direction tests finished.\r\n");

    

    while (1) {
        // CPU rảnh rỗi, VDMA tự chạy ngầm
    }

    return XST_SUCCESS;
}


