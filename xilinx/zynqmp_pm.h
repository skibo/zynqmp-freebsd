/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018-2020 Thomas Skibo.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _ZYNQMP_PM_H_
#define _ZYNQMP_PM_H_

#define ZYNQMP_PM_RESET_BASE		1000
#define ZYNQMP_PM_RESET_PCIE_CFG	1000
#define ZYNQMP_PM_RESET_PCIE_BRIDGE	1001
#define ZYNQMP_PM_RESET_PCIE_CTRL	1002
#define ZYNQMP_PM_RESET_DP		1003
#define ZYNQMP_PM_RESET_SWDT_CRF	1004
#define ZYNQMP_PM_RESET_AFI_FM5		1005
#define ZYNQMP_PM_RESET_AFI_FM4		1006
#define ZYNQMP_PM_RESET_AFI_FM3		1007
#define ZYNQMP_PM_RESET_AFI_FM2		1008
#define ZYNQMP_PM_RESET_AFI_FM1		1009
#define ZYNQMP_PM_RESET_AFI_FM0		1010
#define ZYNQMP_PM_RESET_GDMA		1011
#define ZYNQMP_PM_RESET_GPU_PP1		1012
#define ZYNQMP_PM_RESET_GPU_PP0		1013
#define ZYNQMP_PM_RESET_GPU		1014
#define ZYNQMP_PM_RESET_GT		1015
#define ZYNQMP_PM_RESET_SATA		1016
#define ZYNQMP_PM_RESET_ACPU3_PWRON	1017
#define ZYNQMP_PM_RESET_ACPU2_PWRON	1018
#define ZYNQMP_PM_RESET_ACPU1_PWRON	1019
#define ZYNQMP_PM_RESET_ACPU0_PWRON	1020
#define ZYNQMP_PM_RESET_APU_L2		1021
#define ZYNQMP_PM_RESET_ACPU3		1022
#define ZYNQMP_PM_RESET_ACPU2		1023
#define ZYNQMP_PM_RESET_ACPU1		1024
#define ZYNQMP_PM_RESET_ACPU0		1025
#define ZYNQMP_PM_RESET_DDR		1026
#define ZYNQMP_PM_RESET_APM_FPD		1027
#define ZYNQMP_PM_RESET_SOFT		1028
#define ZYNQMP_PM_RESET_GEM0		1029
#define ZYNQMP_PM_RESET_GEM1		1030
#define ZYNQMP_PM_RESET_GEM2		1031
#define ZYNQMP_PM_RESET_GEM3		1032
#define ZYNQMP_PM_RESET_QSPI		1033
#define ZYNQMP_PM_RESET_UART0		1034
#define ZYNQMP_PM_RESET_UART1		1035
#define ZYNQMP_PM_RESET_SPI0		1036
#define ZYNQMP_PM_RESET_SPI1		1037
#define ZYNQMP_PM_RESET_SDIO0		1038
#define ZYNQMP_PM_RESET_SDIO1		1039
#define ZYNQMP_PM_RESET_CAN0		1040
#define ZYNQMP_PM_RESET_CAN1		1041
#define ZYNQMP_PM_RESET_I2C0		1042
#define ZYNQMP_PM_RESET_I2C1		1043
#define ZYNQMP_PM_RESET_TTC0		1044
#define ZYNQMP_PM_RESET_TTC1		1045
#define ZYNQMP_PM_RESET_TTC2		1046
#define ZYNQMP_PM_RESET_TTC3		1047
#define ZYNQMP_PM_RESET_SWDT_CRL	1048
#define ZYNQMP_PM_RESET_NAND		1049
#define ZYNQMP_PM_RESET_ADMA		1050
#define ZYNQMP_PM_RESET_GPIO		1051
#define ZYNQMP_PM_RESET_IOU_CC		1052
#define ZYNQMP_PM_RESET_TIMESTAMP	1053
#define ZYNQMP_PM_RESET_RPU_R50		1054
#define ZYNQMP_PM_RESET_RPU_R51		1055
#define ZYNQMP_PM_RESET_RPU_AMBA	1056
#define ZYNQMP_PM_RESET_OCM		1057
#define ZYNQMP_PM_RESET_RPU_PGE		1058
#define ZYNQMP_PM_RESET_USB0_CORERESET	1059
#define ZYNQMP_PM_RESET_USB1_CORERESET	1060
#define ZYNQMP_PM_RESET_USB0_HIBERRESET	1061
#define ZYNQMP_PM_RESET_USB1_HIBERRESET	1062
#define ZYNQMP_PM_RESET_USB0_APB	1063
#define ZYNQMP_PM_RESET_USB1_APB	1064
#define ZYNQMP_PM_RESET_IPI		1065
#define ZYNQMP_PM_RESET_APM_LPD		1066
#define ZYNQMP_PM_RESET_RTS		1067
#define ZYNQMP_PM_RESET_SYSMON		1068
#define ZYNQMP_PM_RESET_AFI_FM6		1069
#define ZYNQMP_PM_RESET_LPD_SWDT	1070
#define ZYNQMP_PM_RESET_FPD		1071
#define ZYNQMP_PM_RESET_RPU_DBG1	1072
#define ZYNQMP_PM_RESET_RPU_DBG0	1073
#define ZYNQMP_PM_RESET_DBG_LPD		1074
#define ZYNQMP_PM_RESET_DBG_FPD		1075
#define ZYNQMP_PM_RESET_APLL		1076
#define ZYNQMP_PM_RESET_DPLL		1077
#define ZYNQMP_PM_RESET_VPLL		1078
#define ZYNQMP_PM_RESET_IOPLL		1079
#define ZYNQMP_PM_RESET_RPLL		1080
#define ZYNQMP_PM_RESET_GPO3_PL(n)	((n) + 1081)	/* 0..31 */
#define ZYNQMP_PM_RESET_RPU_LS		1113
#define ZYNQMP_PM_RESET_PS_ONLY		1114
#define ZYNQMP_PM_RESET_PL		1115
#define ZYNQMP_PM_RESET_LAST		ZYNQMP_PM_RESET_PL
/* XXX: I don't know why these don't work and hang the system. */
#define ZYNQMP_PM_RESET_GPIO5_EMIO_92	1116
#define ZYNQMP_PM_RESET_GPIO5_EMIO_93	1117
#define ZYNQMP_PM_RESET_GPIO5_EMIO_94	1118
#define ZYNQMP_PM_RESET_GPIO5_EMIO_95	1119

/* Reset actions. */
#define PM_RESET_ACTION_RELEASE		0
#define PM_RESET_ACTION_ASSERT		1
#define PM_RESET_ACTION_PULSE		2

#define ZYNQMP_PM_NODE_APU		1
#define ZYNQMP_PM_NODE_APU_0		2
#define ZYNQMP_PM_NODE_APU_1		3
#define ZYNQMP_PM_NODE_APU_2		4
#define ZYNQMP_PM_NODE_APU_3		5
#define ZYNQMP_PM_NODE_RPU		6
#define ZYNQMP_PM_NODE_RPU_0		7
#define ZYNQMP_PM_NODE_RPU_1		8
#define ZYNQMP_PM_NODE_PLD		9
#define ZYNQMP_PM_NODE_FPD		10
#define ZYNQMP_PM_NODE_OCM_BANK_0	11
#define ZYNQMP_PM_NODE_OCM_BANK_1	12
#define ZYNQMP_PM_NODE_OCM_BANK_2	13
#define ZYNQMP_PM_NODE_OCM_BANK_3	14
#define ZYNQMP_PM_NODE_TCM_0_A		15
#define ZYNQMP_PM_NODE_TCM_0_B		16
#define ZYNQMP_PM_NODE_TCM_1_A		17
#define ZYNQMP_PM_NODE_TCM_1_B		18
#define ZYNQMP_PM_NODE_L2		19
#define ZYNQMP_PM_NODE_GPU_PP_0		20
#define ZYNQMP_PM_NODE_GPU_PP_1		21
#define ZYNQMP_PM_NODE_USB0		22
#define ZYNQMP_PM_NODE_USB1		23
#define ZYNQMP_PM_NODE_TTC_0		24
#define ZYNQMP_PM_NODE_TTC_1		25
#define ZYNQMP_PM_NODE_TTC_2		26
#define ZYNQMP_PM_NODE_TTC_3		27
#define ZYNQMP_PM_NODE_SATA		28
#define ZYNQMP_PM_NODE_GEM0		29
#define ZYNQMP_PM_NODE_GEM1		30
#define ZYNQMP_PM_NODE_GEM2		31
#define ZYNQMP_PM_NODE_GEM3		32
#define ZYNQMP_PM_NODE_UART0		33
#define ZYNQMP_PM_NODE_UART1		34
#define ZYNQMP_PM_NODE_SPI0		35
#define ZYNQMP_PM_NODE_SPI1		36
#define ZYNQMP_PM_NODE_I2C0		37
#define ZYNQMP_PM_NODE_I2C1		38
#define ZYNQMP_PM_NODE_SDIO0		39
#define ZYNQMP_PM_NODE_SDIO1		40
#define ZYNQMP_PM_NODE_DP		41
#define ZYNQMP_PM_NODE_GDMA		42
#define ZYNQMP_PM_NODE_ADMA		43
#define ZYNQMP_PM_NODE_NAND		44
#define ZYNQMP_PM_NODE_QSPI		45
#define ZYNQMP_PM_NODE_GPIO		46
#define ZYNQMP_PM_NODE_CAN0		47
#define ZYNQMP_PM_NODE_CAN1		48
#define ZYNQMP_PM_NODE_EXTERN		49
#define ZYNQMP_PM_NODE_APLL		50
#define ZYNQMP_PM_NODE_VPLL		51
#define ZYNQMP_PM_NODE_DPLL		52
#define ZYNQMP_PM_NODE_RPLL		53
#define ZYNQMP_PM_NODE_IOPLL		54
#define ZYNQMP_PM_NODE_DDR		55
#define ZYNQMP_PM_NODE_IPI_APU		56
#define ZYNQMP_PM_NODE_IPI_RPU_0	57
#define ZYNQMP_PM_NODE_GPU		58
#define ZYNQMP_PM_NODE_PCIE		59
#define ZYNQMP_PM_NODE_PCAP		60
#define ZYNQMP_PM_NODE_RTC		61
#define ZYNQMP_PM_NODE_LPD		62
#define ZYNQMP_PM_NODE_VCU		63
#define ZYNQMP_PM_NODE_IPI_RPU_1	64
#define ZYNQMP_PM_NODE_IPI_PL0		65
#define ZYNQMP_PM_NODE_IPI_PL1		66
#define ZYNQMP_PM_NODE_IPI_PL2		67
#define ZYNQMP_PM_NODE_IPI_PL3		68
#define ZYNQMP_PM_NODE_PL		69
#define ZYNQMP_PM_NODE_FIRST		ZYNQMP_PM_NODE_APU
#define ZYNQMP_PM_NODE_LAST		ZYNQMP_PM_NODE_PL

/* PMU query id */
#define PM_QID_INVALID				0
#define PM_QID_CLOCK_GET_NAME			1
#define PM_QID_CLOCK_GET_TOPOLOGY		2
#define PM_QID_CLOCK_GET_FIXEDFACTOR_PARAMS	3
#define PM_QID_CLOCK_GET_PARENTS		4
#define PM_QID_CLOCK_GET_ATTRIBUTES		5
#define PM_QID_PINCTRL_GET_NUM_PINS		6
#define PM_QID_PINCTRL_GET_NUM_FUNCTIONS	7
#define PM_QID_PINCTRL_GET_NUM_FUNCTION_GROUPS	8
#define PM_QID_PINCTRL_GET_FUNCTION_NAME	9
#define PM_QID_PINCTRL_GET_FUNCTION_GROUPS	10
#define PM_QID_PINCTRL_GET_PIN_GROUPS		11
#define PM_QID_CLOCK_GET_NUM_CLOCKS		12
#define PM_QID_CLOCK_GET_MAX_DIVISOR		13

/* PMU ioctl id */
#define PM_IOCTL_SET_PLL_FRAC_MODE	8
#define PM_IOCTL_GET_PLL_FRAC_MODE	9
#define PM_IOCTL_SET_PLL_FRAC_DATA	10
#define PM_IOCTL_GET_PLL_FRAC_DATA	11

/* PM node request ack parameter */
#define PM_REQ_ACK_NO		1
#define PM_REQ_ACK_BLOCKING	2
#define PM_REQ_ACK_NON_BLOCKING	3

/* PMU return status codes. */
#define XST_PM_SUCCESS			0
#define XST_PM_NO_FEATURE		19
#define XST_PM_INTERNAL			2000
#define XST_PM_CONFLICT			2001
#define XST_PM_NO_ACCESS		2002
#define XST_PM_INVALID_NODE		2003
#define XST_PM_DOUBLE_REQ		2004
#define XST_PM_ABORT_SUSPEND		2005

/* Node capabilities */
#define PM_CAP_ACCESS		1
#define PM_CAP_CONTEXT		2
#define PM_CAP_WAKEUP		4
#define PM_CAP_UNUSABLE		8

/* PMU request ack */
#define PM_REQUEST_ACK_NO		1
#define PM_REQUEST_ACK_BLOCKING		2
#define PM_REQUEST_ACK_NON_BLOCKING	3

#define PM_MAX_QOS		100
#define PM_MAX_LATENCY		0xffffffffu

/* PM PLL modes */
#define PM_PLL_MODE_RESET	0
#define PM_PLL_MODE_INT		1
#define PM_PLL_MODE_FRAC	2

/* PM PLL Parameters */
#define PM_PLL_PARAM_DIV2	0
#define PM_PLL_PARAM_FBDIV	1
#define PM_PLL_PARAM_DATA	2
#define PM_PLL_PARAM_PRE_SRC	3
#define PM_PLL_PARAM_POST_SRC	4
#define PM_PLL_PARAM_LOCK_DLY	5
#define PM_PLL_PARAM_LOCK_CNT	6
#define PM_PLL_PARAM_LFHF	7
#define PM_PLL_PARAM_CP		8
#define PM_PLL_PARAM_RES	9

/* PM node status usage parameter */
#define PM_USAGE_NO_MASTER	0
#define PM_USAGE_CURRENT_MASTER	1
#define PM_USAGE_OTHER_MASTER	2
#define PM_USAGE_BOTH_MASTERS	(PM_USAGE_CURRENT_MASTER | \
				 PM_USAGE_OTHER_MASTER)

/* Chip id returned by zynqmp_pm_get_chipid(). */
#define PM_IDCODE_ZU2	0x14711093
#define PM_IDCODE_ZU3	0x14710093
#define PM_IDCODE_ZU4	0x04721093
#define PM_IDCODE_ZU5	0x04720093
#define PM_IDCODE_ZU6	0x24739093
#define PM_IDCODE_ZU7	0x14730093
#define PM_IDCODE_ZU9	0x24738093
#define PM_IDCODE_ZU11	0x04740093
#define PM_IDCODE_ZU15	0x14750093
#define PM_IDCODE_ZU17	0x14759093
#define PM_IDCODE_ZU19	0x14758093
#define PM_IDCODE_ZU21	0x147E1093
#define PM_IDCODE_ZU25	0x147E5093
#define PM_IDCODE_ZU27	0x147E4093
#define PM_IDCODE_ZU28	0x147E0093
#define PM_IDCODE_ZU29	0x147E2093

#define RETV_CNT		4

#ifdef _KERNEL
int zynqmp_pm_get_api_version(uint32_t *);
int zynqmp_pm_get_chipid(uint32_t *, uint32_t *);
int zynqmp_pm_get_trustzone_version(uint32_t *);
int zynqmp_pm_init_finalize(void);
int zynqmp_pm_query_data(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t *);
int zynqmp_pm_ioctl(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t *);
int zynqmp_pm_mmio_write(uint32_t, uint32_t, uint32_t);
int zynqmp_pm_mmio_read(uint32_t, uint32_t *);
int zynqmp_pm_fpga_load(vm_paddr_t, uint32_t, uint32_t);
int zynqmp_pm_fpga_read(vm_paddr_t, uint32_t, uint32_t, uint32_t *);
int zynqmp_pm_fpga_get_status(uint32_t *);
int zynqmp_pm_reset_action(int, int);
int zynqmp_pm_reset_get_status(int, int *);
int zynqmp_pm_clock_enable(int);
int zynqmp_pm_clock_disable(int);
int zynqmp_pm_clock_get_state(int, int *);
int zynqmp_pm_clock_set_divider(int, int, uint32_t);
int zynqmp_pm_clock_get_divider(int, uint32_t *);
int zynqmp_pm_clock_set_parent(int, int);
int zynqmp_pm_clock_get_parent(int, int *);
int zynqmp_pm_pll_set_param(int, int, uint32_t);
int zynqmp_pm_pll_get_param(int, int, uint32_t *);
int zynqmp_pm_pll_set_mode(int, int);
int zynqmp_pm_pll_get_mode(int, int *);
int zynqmp_pm_request_node(int, uint32_t, uint32_t, int);
int zynqmp_pm_set_requirement(int, uint32_t, uint32_t, int);
int zynqmp_pm_release_node(int);
int zynqmp_pm_get_node_status(int, uint32_t *, uint32_t *, uint32_t *);
#endif /* _KERNEL */
#endif /* _ZYNQMP_PM_H_ */
