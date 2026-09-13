#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MCP2517FD SPI Commands (MISRA C:2012 Rule 7.2) */
#define MCP2517FD_CMD_RESET             0x0000U
#define MCP2517FD_CMD_WRITE             0x2000U
#define MCP2517FD_CMD_READ              0x3000U
#define MCP2517FD_CMD_WRITE_CRC         0xA000U
#define MCP2517FD_CMD_READ_CRC          0xB000U
#define MCP2517FD_CMD_WRITE_SAFE        0xC000U

/* Register Addresses (MISRA C:2012 Rule 7.2) */
#define MCP2517FD_REG_CiCON             0x000U
#define MCP2517FD_REG_CiNBTCFG          0x004U
#define MCP2517FD_REG_CiDBTCFG          0x008U
#define MCP2517FD_REG_CiTDC             0x00CU
#define MCP2517FD_REG_CiTBC             0x010U
#define MCP2517FD_REG_CiTSCON           0x014U
#define MCP2517FD_REG_CiVEC             0x018U
#define MCP2517FD_REG_CiINT             0x01CU
#define MCP2517FD_REG_CiRXIF            0x020U
#define MCP2517FD_REG_CiTXIF            0x024U
#define MCP2517FD_REG_CiRXOVIF          0x028U
#define MCP2517FD_REG_CiTXATIF          0x02CU
#define MCP2517FD_REG_CiTXREQ           0x030U
#define MCP2517FD_REG_CiTREC            0x034U
#define MCP2517FD_REG_CiBDIAG0          0x038U
#define MCP2517FD_REG_CiBDIAG1          0x03CU
#define MCP2517FD_REG_CiTEFCON          0x040U
#define MCP2517FD_REG_CiTEFSTA          0x044U
#define MCP2517FD_REG_CiTEFUA           0x048U
#define MCP2517FD_REG_CiTXQCON          0x050U
#define MCP2517FD_REG_CiTXQSTA          0x054U
#define MCP2517FD_REG_CiTXQUA           0x058U

/* FIFO Registers (FIFO 1 to 31, index 1..31) (MISRA C:2012 Rule 20.7) */
#define MCP2517FD_REG_CiFIFOCON(n)      (0x05CU + (12U * ((uint16_t)(n) - 1U)))
#define MCP2517FD_REG_CiFIFOSTA(n)      (0x060U + (12U * ((uint16_t)(n) - 1U)))
#define MCP2517FD_REG_CiFIFOUA(n)       (0x064U + (12U * ((uint16_t)(n) - 1U)))

/* Filter Registers (Filter 0 to 31) (MISRA C:2012 Rule 20.7) */
#define MCP2517FD_REG_CiFLTCON(m)       (0x1D0U + (uint16_t)(m))
#define MCP2517FD_REG_CiFLTOBJ(m)       (0x1F0U + (8U * (uint16_t)(m)))
#define MCP2517FD_REG_CiMASK(m)         (0x1F4U + (8U * (uint16_t)(m)))

/* Oscillator and IO Registers */
#define MCP2517FD_REG_OSC               0xE00U
#define MCP2517FD_REG_IOCON             0xE04U

/* RAM Base */
#define MCP2517FD_RAM_ADDR_START        0x400U
#define MCP2517FD_RAM_SIZE              2048U

/* Operation Modes:
 * REQOP is CiCON bits [26:24] (bits [2:0] in byte 3 at 0x003)
 * OPMOD is CiCON bits [23:21] (bits [7:5] in byte 2 at 0x002)
 */
#define MCP2517FD_MODE_NORMAL_CANFD     0U   /* Normal CAN FD mode (supports both CAN FD and Classic CAN 2.0) */
#define MCP2517FD_MODE_SLEEP            1U   /* Sleep mode */
#define MCP2517FD_MODE_INT_LOOPBACK     2U   /* Internal Loopback mode */
#define MCP2517FD_MODE_LISTEN_ONLY      3U   /* Listen Only mode */
#define MCP2517FD_MODE_CONFIGURATION    4U   /* Configuration mode */
#define MCP2517FD_MODE_EXT_LOOPBACK     5U   /* External Loopback mode */
#define MCP2517FD_MODE_CLASSIC_CAN20    6U   /* Normal CAN 2.0B mode (Classic CAN 2.0 only) */
#define MCP2517FD_MODE_RESTRICTED       7U   /* Restricted Operation mode */

/* Payload Sizes in CiFIFOCONm (bits [7:5] of byte 3) */
#define MCP2517FD_PLSIZE_8              0U
#define MCP2517FD_PLSIZE_12             1U
#define MCP2517FD_PLSIZE_16             2U
#define MCP2517FD_PLSIZE_20             3U
#define MCP2517FD_PLSIZE_24             4U
#define MCP2517FD_PLSIZE_32             5U
#define MCP2517FD_PLSIZE_48             6U
#define MCP2517FD_PLSIZE_64             7U

/* FIFOSTA bits (MISRA C:2012 Rule 10.1 & Rule 12.2) */
#define MCP2517FD_FIFOSTA_TFNRFNIF      (1U << 0U) /* FIFO not empty (RX) / not full (TX) */
#define MCP2517FD_FIFOSTA_TFHALFIF      (1U << 1U) /* FIFO half empty/full */
#define MCP2517FD_FIFOSTA_TFFULLIF      (1U << 2U) /* FIFO full */
#define MCP2517FD_FIFOSTA_RXOVIF        (1U << 3U) /* RX overflow */
#define MCP2517FD_FIFOSTA_TXABT         (1U << 6U) /* TX message aborted */
#define MCP2517FD_FIFOSTA_TXERR         (1U << 7U) /* TX error occurred */

/* FIFOCON control bits (MISRA C:2012 Rule 10.1 & Rule 12.2) */
#define MCP2517FD_FIFOCON_TXEN          (1U << 7U) /* 1 = TX, 0 = RX */
#define MCP2517FD_FIFOCON_TXATIE        (1U << 4U)
#define MCP2517FD_FIFOCON_RXOVIE        (1U << 3U)
#define MCP2517FD_FIFOCON_TFNRFNIE      (1U << 0U)
#define MCP2517FD_FIFOCON_UINC          (1U << 0U) /* Byte 1: Increment head/tail pointer */
#define MCP2517FD_FIFOCON_TXREQ         (1U << 1U) /* Byte 1: Request transmission */
#define MCP2517FD_FIFOCON_FRESET        (1U << 2U) /* Byte 1: Reset FIFO */
#define MCP2517FD_FIFOCON_TXAT_UNLIMITED (3U << 5U) /* Byte 1: Retransmission attempts unlimited */

/* Interrupt flags in CiINT (MISRA C:2012 Rule 10.1 & Rule 12.2) */
#define MCP2517FD_INT_TXIF              (1U << 0U)
#define MCP2517FD_INT_RXIF              (1U << 1U)
#define MCP2517FD_INT_TBCIF             (1U << 2U)
#define MCP2517FD_INT_MODIF             (1U << 3U)
#define MCP2517FD_INT_CEIF              (1U << 8U)
#define MCP2517FD_INT_TEFIF             (1U << 9U)
#define MCP2517FD_INT_TXATIF            (1U << 10U)
#define MCP2517FD_INT_RXOVIF            (1U << 11U)
#define MCP2517FD_INT_SERRIF            (1U << 12U)

/* Diagnostics CiTREC (0x034) (MISRA C:2012 Rule 10.1 & Rule 12.2) */
#define MCP2517FD_TREC_TXBO             (1UL << 21U) /* Transmitter in Bus-Off state */
#define MCP2517FD_TREC_TXBP             (1UL << 20U) /* Transmitter in Bus-Passive state */
#define MCP2517FD_TREC_RXBP             (1UL << 19U) /* Receiver in Bus-Passive state */
#define MCP2517FD_TREC_TXWARN           (1UL << 18U) /* Transmitter in Error-Warning state */
#define MCP2517FD_TREC_RXWARN           (1UL << 17U) /* Receiver in Error-Warning state */
#define MCP2517FD_TREC_EWARN            (1UL << 16U) /* Error Warning */

/* Diagnostics CiBDIAG1 (0x03C) (MISRA C:2012 Rule 10.1 & Rule 12.2)
 * Bits [15:0]  = EFMSGCNT[15:0] (Error-Free Message Counter)
 * Bits [23:16] = Nominal Phase Error Flags
 * Bits [31:24] = Data Phase Error Flags
 */
#define MCP2517FD_BDIAG1_NBIT0_ERR      (1UL << 16U) /* Nominal Bit 0 Error: sent 0, monitored 1 */
#define MCP2517FD_BDIAG1_NBIT1_ERR      (1UL << 17U) /* Nominal Bit 1 Error: sent 1, monitored 0 */
#define MCP2517FD_BDIAG1_NACK_ERR       (1UL << 18U) /* Nominal Acknowledge Error (No ACK from bus) */
#define MCP2517FD_BDIAG1_NFORM_ERR      (1UL << 19U) /* Nominal Form Error */
#define MCP2517FD_BDIAG1_NSTUFF_ERR     (1UL << 20U) /* Nominal Stuff Error */
#define MCP2517FD_BDIAG1_NCRC_ERR       (1UL << 21U) /* Nominal CRC Error */
#define MCP2517FD_BDIAG1_TXBO_ERR       (1UL << 23U) /* Transmit Bus-Off Error */
#define MCP2517FD_BDIAG1_DBIT0_ERR      (1UL << 24U) /* Data Bit 0 Error: sent 0, monitored 1 */
#define MCP2517FD_BDIAG1_DBIT1_ERR      (1UL << 25U) /* Data Bit 1 Error: sent 1, monitored 0 */
#define MCP2517FD_BDIAG1_DFORM_ERR      (1UL << 26U) /* Data Form Error */
#define MCP2517FD_BDIAG1_DSTUFF_ERR     (1UL << 27U) /* Data Stuff Error */
#define MCP2517FD_BDIAG1_DCRC_ERR       (1UL << 28U) /* Data CRC Error */
#define MCP2517FD_BDIAG1_ESI            (1UL << 29U) /* Error State Indicator Error */
#define MCP2517FD_BDIAG1_DLCMM          (1UL << 30U) /* DLC Mismatch Error */

#ifdef __cplusplus
}
#endif
