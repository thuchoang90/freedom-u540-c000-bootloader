#ifndef _USB11HS_H
#define _USB11HS_H

/* Register offsets */
#define USB11HS_HOST_TX_CONTROL_REG                     0x00
#define USB11HS_HOST_TX_TRANS_TYPE_REG                  0x01
#define USB11HS_HOST_TX_LINE_CONTROL_REG                0x02
#define USB11HS_HOST_TX_SOF_ENABLE_REG                  0x03
#define USB11HS_HOST_TX_ADDR_REG                        0x04
#define USB11HS_HOST_TX_ENDP_REG                        0x05
#define USB11HS_HOST_FRAME_NUM_MSP_REG                  0x06
#define USB11HS_HOST_FRAME_NUM_LSP_REG                  0x07
#define USB11HS_HOST_INTERRUPT_STATUS_REG               0x08
#define USB11HS_HOST_INTERRUPT_MASK_REG                 0x09
#define USB11HS_HOST_RX_STATUS_REG                      0x0A
#define USB11HS_HOST_RX_PID_REG                         0x0B
#define USB11HS_HOST_RX_ADDR_REG                        0x0C
#define USB11HS_HOST_RX_ENDP_REG                        0x0D
#define USB11HS_HOST_RX_CONNECT_STATE_REG               0x0E
#define USB11HS_HOST_SOF_TIMER_MSB_REG                  0x0F

#define USB11HS_HOST_RX_FIFO_DATA                       0x20
#define USB11HS_HOST_RX_FIFO_DATA_COUNT_MSB             0x22
#define USB11HS_HOST_RX_FIFO_DATA_COUNT_LSB             0x23
#define USB11HS_HOST_RX_FIFO_CONTROL_REG                0x24

#define USB11HS_HOST_TX_FIFO_DATA                       0x30
#define USB11HS_HOST_TX_FIFO_CONTROL_REG                0x34

#define USB11HS_ENDPOINT0_CONTROL_REG                   0x40
#define USB11HS_ENDPOINT0_STATUS_REG                    0x41
#define USB11HS_ENDPOINT0_TRANSTYPE_STATUS_REG          0x42
#define USB11HS_ENDPOINT0_NAK_TRANSTYPE_STATUS_REG      0x43
#define USB11HS_ENDPOINT1_CONTROL_REG                   0x44
#define USB11HS_ENDPOINT1_STATUS_REG                    0x45
#define USB11HS_ENDPOINT1_TRANSTYPE_STATUS_REG          0x46
#define USB11HS_ENDPOINT1_NAK_TRANSTYPE_STATUS_REG      0x47
#define USB11HS_ENDPOINT2_CONTROL_REG                   0x48
#define USB11HS_ENDPOINT2_STATUS_REG                    0x49
#define USB11HS_ENDPOINT2_TRANSTYPE_STATUS_REG          0x4A
#define USB11HS_ENDPOINT2_NAK_TRANSTYPE_STATUS_REG      0x4B
#define USB11HS_ENDPOINT3_CONTROL_REG                   0x4C
#define USB11HS_ENDPOINT3_STATUS_REG                    0x4D
#define USB11HS_ENDPOINT3_TRANSTYPE_STATUS_REG          0x4E
#define USB11HS_ENDPOINT3_NAK_TRANSTYPE_STATUS_REG      0x4F

#define USB11HS_SC_CONTROL_REG                          0x50
#define USB11HS_SC_LINE_STATUS_REG                      0x51
#define USB11HS_SC_INTERRUPT_STATUS_REG                 0x52
#define USB11HS_SC_INTERRUPT_MASK_REG                   0x53
#define USB11HS_SC_ADDRESS                              0x54
#define USB11HS_SC_FRAME_NUM_MSP                        0x55
#define USB11HS_SC_FRAME_NUM_LSP                        0x56

#define USB11HS_EP0_RX_FIFO_DATA                        0x60
#define USB11HS_EP0_RX_FIFO_DATA_COUNT_MSB              0x62
#define USB11HS_EP0_RX_FIFO_DATA_COUNT_LSB              0x63
#define USB11HS_EP0_RX_FIFO_CONTROL_REG                 0x64

#define USB11HS_EP0_TX_FIFO_DATA                        0x70
#define USB11HS_EP0_TX_FIFO_CONTROL_REG                 0x74

#define USB11HS_EP1_RX_FIFO_DATA                        0x80
#define USB11HS_EP1_RX_FIFO_DATA_COUNT_MSB              0x82
#define USB11HS_EP1_RX_FIFO_DATA_COUNT_LSB              0x83
#define USB11HS_EP1_RX_FIFO_CONTROL_REG                 0x84

#define USB11HS_EP1_TX_FIFO_DATA                        0x90
#define USB11HS_EP1_TX_FIFO_CONTROL_REG                 0x94

#define USB11HS_EP2_RX_FIFO_DATA                        0xA0
#define USB11HS_EP2_RX_FIFO_DATA_COUNT_MSB              0xA2
#define USB11HS_EP2_RX_FIFO_DATA_COUNT_LSB              0xA3
#define USB11HS_EP2_RX_FIFO_CONTROL_REG                 0xA4

#define USB11HS_EP2_TX_FIFO_DATA                        0xB0
#define USB11HS_EP2_TX_FIFO_CONTROL_REG                 0xB4

#define USB11HS_EP3_RX_FIFO_DATA                        0xC0
#define USB11HS_EP3_RX_FIFO_DATA_COUNT_MSB              0xC2
#define USB11HS_EP3_RX_FIFO_DATA_COUNT_LSB              0xC3
#define USB11HS_EP3_RX_FIFO_CONTROL_REG                 0xC4

#define USB11HS_EP3_TX_FIFO_DATA                        0xD0
#define USB11HS_EP3_TX_FIFO_CONTROL_REG                 0xD4

#define USB11HS_HOST_SLAVE_CONTROL_REG                  0xe0
#define USB11HS_HOST_SLAVE_VERSION_REG                  0xe1

#endif /* _USB11HS_H */
