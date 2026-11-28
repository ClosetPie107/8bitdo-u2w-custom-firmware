#ifndef HOST_TEST_ENV_H
#define HOST_TEST_ENV_H

#include <stdint.h>

/* Simulated JieLi BR22 globals for host unit tests */
extern uint32_t g_usb_mode;
extern uint32_t g_act_mode;
extern uint8_t  g_usb_ready;
extern uint32_t g_rad_mode;
extern uint32_t g_sav_mode;
extern uint8_t  g_tx_flags;
extern uint8_t  g_mode_act;
extern uint8_t  g_batt_byte;
extern uint8_t  g_imu_bytes[12];
extern uint32_t g_rx_btn;
extern uint32_t g_hk_combo;
extern uint32_t g_hk_count;

#define USB_MODE  g_usb_mode
#define ACT_MODE  g_act_mode
#define USB_READY g_usb_ready
#define RAD_MODE  g_rad_mode
#define SAV_MODE  g_sav_mode
#define TX_FLAGS  g_tx_flags
#define MODE_ACT  g_mode_act
#define BATT_BYTE g_batt_byte
#define IMU_PTR   ((const volatile uint8_t *)g_imu_bytes)
#define RX_BTN    g_rx_btn
#define HK_COMBO  g_hk_combo
#define HK_COUNT  g_hk_count

#define SEC_PACK
#define SEC_SEND
#define SEC_HOTKEY

void orig_pack(uint32_t btn, uint32_t sticks, uint32_t trig, uint8_t *dst);
void usb_send(uint32_t usb_id, uint8_t *buf, uint32_t len);
void post_key(uint32_t ev);
void nvm_save(uint32_t mode);

/* Simulated USB TX / Timer0 / IRQ state */
extern uint32_t g_ep_busy_mask;
extern uint32_t g_t0_tick_2ms;
extern uint32_t g_usb_dev_table[4];
extern uint32_t g_usb_hdl_ptr;
extern uint8_t  g_usb_state_byte;
extern uint32_t g_usb_con0_reg;
extern uint32_t g_usb_txcount_reg[8];
extern uint32_t g_icfg;
extern uint32_t g_ep_maxp[8];
extern uint8_t  g_ep_dma_buf[8][64];

#define EP_BUSY_MASK    g_ep_busy_mask
#define T0_TICK_2MS     g_t0_tick_2ms
#define USB_DEV_TABLE   g_usb_dev_table
#define USB_HDL_PTR     g_usb_hdl_ptr
#define USB_CON0_REG    g_usb_con0_reg
#define USB_TXCOUNT_REG g_usb_txcount_reg
#define USB_STATE_BYTE(hdl) ((void)(hdl), g_usb_state_byte)
#define USB_DEV_DMA_BUF(dev, ep, ep_off) ((void)(dev), (void)(ep_off), (ep))
#define USB_DEV_MAXP(dev, ep_off) ((void)(dev), g_ep_maxp[(ep_off) >> 2])
#define USB_DMA_PTR(dma_buf) ((volatile uint8_t *)g_ep_dma_buf[(dma_buf)])

uint32_t read_icfg(void);
uint32_t usb_read_txcsr(uint32_t ep);
void usb_write_txcsr(uint32_t ep, uint32_t val);

void dinput_pack_hook(uint32_t btn, uint32_t sticks, uint32_t trig, uint8_t *dst);
void dinput_send_hook(uint32_t usb_id, uint8_t *buf, uint32_t len);
void hotkey_hook(uint32_t ev);
void fix_usb_g_intr_write(uint32_t id, uint32_t ep, const uint8_t *buf, uint32_t len);

#endif
