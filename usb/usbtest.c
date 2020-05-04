// Note: All the code is extracted and translated from: 
// https://fpga.kice.tokyo/opencores/usb-sw-jp

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <stdio.h>
#include "usb11hs_plugin.h"
#include "sifive/plic_driver.h"
#include "encoding.h"
#include <uart/uart.h>

typedef union {
  unsigned char PACKED[4];
  struct {
    unsigned char x; // left: 0, center:0x7f. right: 0xff
    unsigned char y; // up: 0, center:0x7f. down: 0xff
    union {
      unsigned char BYTE;
      struct {
        unsigned char button0 : 1;
        unsigned char button1 : 1;
        unsigned char button2 : 1;
        unsigned char button3 : 1;
        unsigned char button4 : 1;
        unsigned char button5 : 1;
        unsigned char button6 : 1;
        unsigned char button7 : 1;
      } BIT;
    } b0;
    union {
      unsigned char BYTE;
      struct {
        unsigned char button0 : 1;
        unsigned char button1 : 1;
        unsigned char : 6;
      } BIT;
    } b1;
  } BYTE;
} st_joypad;

#define USB_NORMAL 0 // TODO: There is no definition of this

st_joypad joypad_status;
unsigned char int_trans_done = 0;
unsigned char usb_state = USB_NORMAL;
unsigned char int_connect_done = 0;

extern function_ptr_t g_ext_interrupt_handlers[PLIC_NUM_INTERRUPTS];
extern function_ptr_t g_time_interrupt_handler;
extern plic_instance_t g_plic;

void interrupt_in_transfer_nb(int adrs, int endp) {
  USB_TX_ADDR_REG.LONG = adrs;
  USB_TX_ENDP_REG.LONG = endp;
  USB_TX_TRANS_TYPE_REG.LONG = 1; // IN TRANS
  int_trans_done = 0;
  USB_CONTROL_REG.LONG = 1;
  puts("Start INT_TRANS\r\n");
}

void interrupt_in_transfer_get(int adrs, int endp) {
  int i, fnum;
  //char buf[64];
  unsigned int status;

  // buffer clear
  for (i=0; i < 4; i++) joypad_status.PACKED[i] = 0;
  while (!int_trans_done) ;
  puts("interrupt in trans done\r\n");

  void *uart = (void*)UART0_CTRL_ADDR;

  status = USB_RX_STATUS_REG.LONG;
  puts("INT RX STATUS ");
  uart_put_hex(uart, status);
  // get IN data
  puts(" \r\ninterrupt in length = ");
  fnum = USB_RX_FIFO_DATA_COUNT_LSB.LONG;
  uart_put_hex(uart, fnum);
  for (i = 0; i < fnum; i++) {
    if (i <4) joypad_status.PACKED[i] = USB_RX_FIFO_DATA.LONG; // 受信データを構造体に格納
  }
  // status
  puts(" \r\nX = ");
  uart_put_hex(uart, joypad_status.BYTE.x);
  puts(" \r\nY = ");
  uart_put_hex(uart, joypad_status.BYTE.y);
  puts(" \r\nB1 = ");
  uart_put_hex(uart, joypad_status.BYTE.b1.BYTE);
  puts(" \r\nB0 = ");
  uart_put_hex(uart, joypad_status.BYTE.b0.BYTE);
  puts("\r\n");
}

// This seems to be the interrupt handler for the TMU1
void INT_TMU1_TUNI1() {
  //unsigned int x;
  puts ("TMU1 INT\r\n");
  // clear TCR0.UNF
  //TMU1.TCR.WORD = TMU1.TCR.WORD & ~0x0100;
  //x = 1;
  //while (x) x = INTC.INT2A0.LONG & 1; // bit0 : TMU0-2
  // start usb interrupt transfer
  if (usb_state == USB_NORMAL) interrupt_in_transfer_nb(5, 1); // アドレス5, エンドポイント1へのインタラプト転送
}

// This seems to be the interrupt handler for the USB ints
void INT_IRL_LEVEL11() {
  // INTR IRQ1
  unsigned int d;
  //char buf[64];
  d = USB_INTERRUPT_STATUS_REG.LONG;
  puts("INT detected\r\n");
  if (d & 0x1) {
    puts("TRANS_DONE_BIT\r\n");
    USB_INTERRUPT_STATUS_REG.LONG = 1; // clear status
    int_trans_done = 1;
    if (usb_state == USB_NORMAL) interrupt_in_transfer_get(5, 1); // データ獲得
  } else if (d & 0x2) {
    puts("RESUME_INT_BIT\r\n");
    USB_INTERRUPT_STATUS_REG.LONG = 2;
  } else if (d & 0x4) {
    puts("CONNECTION_EVENT_BIT\r\n");
    USB_INTERRUPT_STATUS_REG.LONG = 4;
    int_connect_done = 1;
  } else if (d & 0x8){
    puts("SOF_SENT_BIT\r\n");
    USB_INTERRUPT_STATUS_REG.LONG = 8;
  } else {
    puts("UNKOWN INT\r\n");
  }
}

// There is no usb_init function
// This is a compilation from the linux driver provided
// (Also... F*ck you, japanese dude)
void usb_init() {
  // There is a reset in the linux driver, so we are going to put it.
  USB_HOST_SLAVE_CONTROL_REG.LONG = 2;
  // Port power function (port_power)
  // 1. ohs900-hcd.c:94
  puts ("Disabling interrupts\r\n");
  USB_INTERRUPT_MASK_REG.LONG = 0;
  puts ("Clearing interrupts\r\n");
  USB_INTERRUPT_STATUS_REG.LONG = ~0;

  USB_INTERRUPT_MASK_REG.LONG = 0; // NOTE: I am just putting this here
  USB_TX_LINE_CONTROL_REG.LONG = OHS900_TXLCTL_MASK_FS_RATE & OHS900_TXLCTL_MASK_FS_POL;
  USB_TX_SOF_ENABLE_REG.LONG = 0;
  USB_HOST_SLAVE_CONTROL_REG.LONG = OHS900_HS_CTL_INIT;
  puts ("Enabling interrupts\r\n");
  USB_INTERRUPT_MASK_REG.LONG = OHS900_INTMASK_INSRMV;
}

void timer1_config() {
  // TMU1を使用して1秒間隔でインタラプトを発生
  // The timer is handled by the main process.
  // This seems to be a 1s timer, and calls
  // INT_TMU1_TUNI1.
}

void int_config() {
  // Just bind the interrupt vectors
  for(int ii = 20; ii <= 29; ii++) {
    g_ext_interrupt_handlers[ii] = INT_IRL_LEVEL11;
    PLIC_enable_interrupt (&g_plic, ii);
    PLIC_set_priority (&g_plic, ii, 1);
  }
  g_time_interrupt_handler = INT_TMU1_TUNI1;
}

void intr_disable() {
  clear_csr(mstatus, MSTATUS_MIE);
  clear_csr(mie, MIP_MEIP);
  clear_csr(mie, MIP_MTIP);
}

void intr_enable() {
  volatile uint64_t * mtime       = (uint64_t*) (CLINT_CTRL_ADDR + CLINT_MTIME);
  volatile uint64_t * mtimecmp    = (uint64_t*) (CLINT_CTRL_ADDR + CLINT_MTIMECMP);
  uint64_t now = *mtime;
  uint64_t then = now + 1000000; // 1 second
  *mtimecmp = then;

  set_csr(mstatus, MSTATUS_MIE);
  set_csr(mie, MIP_MEIP);
  set_csr(mie, MIP_MTIP);
}

int usb_test(void){
  puts("USB Test\r\n");

  intr_disable();
  int_config();
  timer1_config(); // インタラプト転送用タイマー設定

  usb_init(); // USBホストCoreの初期化
  intr_enable();

  while(1) {
    //インタラプトハンドラでインタラプト転送を実行;
  }
  return 0;
}
