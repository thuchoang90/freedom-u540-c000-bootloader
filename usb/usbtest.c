// Note: All the code is extracted and translated from: 
// https://fpga.kice.tokyo/opencores/usb-sw-jp

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

st_joypad joypad_status;

void timer1_config() {
     // TMU1を使用して1秒間隔でインタラプトを発生
     // TODO: Still do not support timer. 
     // This seems to be a 1s timer, and calls
     // INT_TMU1_TUNI1. For now we are going to use 
     // the main() stuff.
 }
 
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
     char buf[64];
     unsigned int status;
 
     // buffer clear
     for (i=0; i < 4; i++) joypad_status.PACKED[i] = 0;
     while (!int_trans_done) ;
     puts("interrupt in trans done\r\n");
 
     status = USB_RX_STATUS_REG.LONG;
     sprintf(buf, "INT RX STATUS %x \r\n", status);
     puts(buf);
     // get IN data
     fnum = USB_RX_FIFO_DATA_COUNT_LSB.LONG;
     sprintf(buf,"interrupt in length = %d\r\n",fnum);
     puts(buf);
     for (i = 0; i < fnum; i++) {
         if (i <4) joypad_status.PACKED[i] = USB_RX_FIFO_DATA.LONG; // 受信データを構造体に格納
     }
     // status
     sprintf(buf, "X = %x\r\n",joypad_status.BYTE.x);
     puts(buf);
     sprintf(buf, "Y = %x\r\n",joypad_status.BYTE.y);
     puts(buf);
     sprintf(buf, "B1 = %x, %d %d\r\n",joypad_status.BYTE.b1.BYTE,
     joypad_status.BYTE.b1.BIT.button0,
     joypad_status.BYTE.b1.BIT.button1);
     puts(buf);
     sprintf(buf, "B1 = %x, %d %d %d %d %d %d %d %d\r\n",joypad_status.BYTE.b0.BYTE,
         joypad_status.BYTE.b0.BIT.button0,
         joypad_status.BYTE.b0.BIT.button1,
         joypad_status.BYTE.b0.BIT.button2,
         joypad_status.BYTE.b0.BIT.button3,
         joypad_status.BYTE.b0.BIT.button4,
         joypad_status.BYTE.b0.BIT.button5,
         joypad_status.BYTE.b0.BIT.button6,
         joypad_status.BYTE.b0.BIT.button7);
     puts(buf);
 
 }

// This seems to be the interrupt handler for the TMU1
void INT_TMU1_TUNI1() {
     unsigned int x;
     puts ("TMU1 INT\r\n");
     // clear TCR0.UNF
     TMU1.TCR.WORD = TMU1.TCR.WORD & ~0x0100;
     x = 1;
     while (x) x = INTC.INT2A0.LONG & 1; // bit0 : TMU0-2
     // start usb interrupt transfer
     if (usb_state == USB_NORMAL) interrupt_in_transfer_nb(5, 1); // アドレス5, エンドポイント1へのインタラプト転送
 }

// This seems to be the interrupt handler for the USB ints
void INT_IRL_LEVEL11() {
     // INTR IRQ1
     unsigned int d;
     char buf[64];
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
         sprintf(buf, "UNKOWN INT %x \r\n", d);
         puts(buf);
     }
 }
 


int main(void){
     int i;
     char buf[64];
 
     puts ("USB Test\n");
 
     intr_disable();
     int_config();
     timer1_config(); // インタラプト転送用タイマー設定
     intr_enable();
 
     usb_init(); // USBホストCoreの初期化
 
     while(1) {
         //インタラプトハンドラでインタラプト転送を実行;
     }
     return 0;
 }
