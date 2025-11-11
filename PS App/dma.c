#include "xparameters.h"
#include "xgpio.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "sleep.h"

#include "dma.h"

// extern XGpio GpioOut;
// extern XGpio GpioIn;

XGpio GpioOut;
XGpio GpioIn;

void debug_gpio_status() {
    u32 addr = XGpio_DiscreteRead(&GpioOut, GPIO_ADDR_CHANNEL);
    u32 len_ctrl = XGpio_DiscreteRead(&GpioOut, GPIO_LEN_CHANNEL);
    u32 status = XGpio_DiscreteRead(&GpioIn, GPIO_STATUS_CHANNEL);
    
    xil_printf("Debug: Addr=0x%08X, LenCtrl=0x%08X, Status=0x%08X\r\n", 
               addr, len_ctrl, status);
}

// Hàm tạo xung start đúng cách
void dma_start_transfer(u32 addr, u32 length_bytes, u8 direction) {
    u32 control_value;
    
    // 1. Set address
    XGpio_DiscreteWrite(&GpioOut, GPIO_ADDR_CHANNEL, addr);
    
    // 2. Set length + direction (chưa có start)
    control_value = (length_bytes << 2) | (direction << LENGTH_DIR_BIT);
    XGpio_DiscreteWrite(&GpioOut, GPIO_LEN_CHANNEL, control_value);
    
    // 3. Tạo xung start (set bit start)
    control_value |= (1 << LENGTH_START_BIT);
    XGpio_DiscreteWrite(&GpioOut, GPIO_LEN_CHANNEL, control_value);
    
    
    // 4. Giữ start ít nhất vài cycle
    usleep(1000); 
    
    // 5. Clear start bit
    control_value &= ~(1 << LENGTH_START_BIT);
    XGpio_DiscreteWrite(&GpioOut, GPIO_LEN_CHANNEL, control_value);
}

// Hàm đợi transfer hoàn thành
int dma_wait_for_completion(int timeout_ms) {
    int timeout = 0;
    while (timeout < timeout_ms) {
        if (XGpio_DiscreteRead(&GpioIn, GPIO_STATUS_CHANNEL) & 0x1) {
            return 1; // Done
        }
        usleep(1000); // 1ms
        timeout++;
    }
    return 0; // Timeout
}