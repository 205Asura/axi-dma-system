#ifndef DMA_H
#define DMA_H

#include "xil_printf.h"
#include "xgpio.h"




extern XGpio GpioOut;
extern XGpio GpioIn;

void debug_gpio_status();
void dma_start_transfer(u32 addr, u32 length_bytes, u8 direction);
int dma_wait_for_completion(int timeout_ms);

#define GPIO_OUT_BASEADDR   XPAR_AXI_GPIO_0_BASEADDR
#define GPIO_IN_BASEADDR    XPAR_AXI_GPIO_1_BASEADDR

#define GPIO_ADDR_CHANNEL       1   // dma_ddr_addr[31:0]
#define GPIO_LEN_CHANNEL        2   // {dma_length_bytes[29:0], dma_direction, dma_start_transfer}

// ĐÚNG: Bit positions trong GPIO_LEN_CHANNEL
#define LENGTH_START_BIT    0
#define LENGTH_DIR_BIT      1  
#define LENGTH_DATA_BITS    30 // bits [31:2] cho length

#define GPIO_STATUS_CHANNEL     1   // dma_transfer_done

#define TX_BUFFER_BASE (XPAR_PS7_DDR_0_BASEADDRESS + 0x00100000)
#define RX_BUFFER_BASE (XPAR_PS7_DDR_0_BASEADDRESS + 0x00300000)
#define TEST_PKT_LEN_BYTES 16  // Test với 1KB trước

#endif 
