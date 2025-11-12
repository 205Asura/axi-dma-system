#include "xparameters.h"
#include "xgpio.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "sleep.h"

#include "dma.h"



int main(void) {
    xil_printf("\r\n=== AXI DMA Shim Test ===\r\n");

    XGpio_Config *cfg;
    int Status;
    u8 *TxBuffer, *RxBuffer;


    // --- Khởi tạo GPIO ---
    cfg = XGpio_LookupConfig(GPIO_OUT_BASEADDR);
    if (!cfg || XGpio_CfgInitialize(&GpioOut, cfg, cfg->BaseAddress) != XST_SUCCESS) {
        xil_printf("GPIO_Out init failed.\r\n");
        return XST_FAILURE;
    }

    cfg = XGpio_LookupConfig(GPIO_IN_BASEADDR);
    if (!cfg || XGpio_CfgInitialize(&GpioIn, cfg, cfg->BaseAddress) != XST_SUCCESS) {
        xil_printf("GPIO_In init failed.\r\n");
        return XST_FAILURE;
    }

    // Cấu hình hướng GPIO
    XGpio_SetDataDirection(&GpioOut, GPIO_ADDR_CHANNEL, 0x0);
    XGpio_SetDataDirection(&GpioOut, GPIO_LEN_CHANNEL, 0x0);
    XGpio_SetDataDirection(&GpioIn, GPIO_STATUS_CHANNEL, 0x1);

    // --- Chuẩn bị buffer ---
    TxBuffer = (u8*)TX_BUFFER_BASE;
    RxBuffer = (u8*)RX_BUFFER_BASE;
    
    for (int i = 0; i < TEST_PKT_LEN_BYTES; i++) {
        TxBuffer[i] = (i % 256);
        RxBuffer[i] = 0;
    }
    Xil_DCacheFlushRange((UINTPTR)TxBuffer, TEST_PKT_LEN_BYTES);
    Xil_DCacheFlushRange((UINTPTR)RxBuffer, TEST_PKT_LEN_BYTES);

    

    // --- Test MM2S (Memory to Stream) ---  
    xil_printf("Starting MM2S transfer...\r\n");
    dma_start_transfer((u32)TxBuffer, TEST_PKT_LEN_BYTES, 1); // direction=1: MM2S
    if (!dma_wait_for_completion(5000)) {
        xil_printf("❌ MM2S Timeout!\r\n");
        return XST_FAILURE;
    }
    xil_printf("✅ MM2S Done.\r\n");

    // --- Test S2MM (Stream to Memory) ---
    xil_printf("Starting S2MM transfer...\r\n");
    dma_start_transfer((u32)RxBuffer, TEST_PKT_LEN_BYTES, 0); // direction=0: S2MM
    
    if (!dma_wait_for_completion(5000)) {
        xil_printf("❌ S2MM Timeout!\r\n");
        return XST_FAILURE;
    }
    xil_printf("✅ S2MM Done.\r\n");

    

    // --- Verify data ---
    xil_printf("Verifying data...\r\n");
    Xil_DCacheInvalidateRange((UINTPTR)RxBuffer, TEST_PKT_LEN_BYTES);
    
    int errors = 0;
    for (int i = 0; i < TEST_PKT_LEN_BYTES; i++) {
        if (TxBuffer[i] != RxBuffer[i]) {
            if (errors < 10) { // Chỉ in 10 lỗi đầu
                xil_printf("Mismatch @ %d: TX=0x%02X RX=0x%02X\r\n", 
                          i, TxBuffer[i], RxBuffer[i]);
            }
            errors++;
        }
    }
    
    for (int i = 0; i < TEST_PKT_LEN_BYTES; i++)
    {
        xil_printf("0x%02X ", TxBuffer[i]);
    }
    xil_printf("\r\n");

    for (int i = 0; i < TEST_PKT_LEN_BYTES; i++)
    {
        xil_printf("0x%02X ", RxBuffer[i]);
    }
    xil_printf("\r\n");

    if (errors == 0) {
        xil_printf("🎉 All tests PASSED! Data verified successfully.\r\n");
    } else {
        xil_printf("❌ Found %d errors in data verification.\r\n", errors);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}