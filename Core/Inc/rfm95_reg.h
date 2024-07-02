/************************************************
* @file    rfm95_reg.h 
* @author  APBashara
* @date    6/2024
* 
* @brief   RFM95 Register Address Definitions
* @note    Addresses are for LoRa Mode
***********************************************/

#include <stdint.h>

#define __REG                               volatile uint8_t

/******************  Register Memory Structure  *******************/
typedef struct
{
    __REG RegFifo;
    __REG RegOpMode;
    __REG RegFrfMsb;
    __REG RegFrfMid;
    __REG RegFrfLsb;
    __REG RegPaConfig;
    __REG RegPaRamp;
    __REG RegOcp;
    __REG RegLna;
    __REG RegFifoAddrPtr;
    __REG RegFifoTxBaseAddr;
    __REG RegFifoRxBaseAddr;
    __REG RegIrqFlags;
    __REG RegIrqFlagsMask;
    __REG RegFreqIfMsb;
    __REG RegFreqIFLsb;
    __REG RegSymbTimeoutMsb;
    __REG RegSymbTimeoutLsb;
    __REG RegTxCfg;
    __REG RegPayloadLength;
    __REG RegPreambleMsb;
    __REG RegPreambleLsb;
    __REG RegModulationCfg;
    __REG RegRfMode;
    __REG RegHopPeriod;
    __REG RegNbRxBytes;
    __REG RegRxHeaderInfo;
    __REG RegRxHeaderCntValue;
    __REG RegRxPacketCntValue;
    __REG RegModemStat;
    __REG RegPktSnrValue;
    __REG RegRssiValue;
    __REG RegPktRssiValue;
    __REG RegHopChannel;
    __REG RegRxDataAddr;
    __REG RegDioMapping1;
    __REG RegDioMapping2;
    __REG RegVersion;
    __REG RegTcxo;
    __REG RegPaDac;
    __REG RegFormerTemp;
    __REG RegAgcRef;
    __REG RegAgcThresh1;
    __REG RegAgcThresh2;
    __REG RegAgcThresh3;
} RFM95_TypeDef;

/******************  Register Memory Address Macros  *******************/
#define RegFifo_Address                     (0x00u)
#define RegOpMode_Address                   (0x01u)
#define RegFrfMsb_Address                   (0x06u)
#define RegFrfMid_Address                   (0x07u)
#define RegFrfLsb_Address                   (0x08u)
#define RegPaConfig_Address                 (0x09u)
#define RegPaRamp_Address                   (0x0Au)
#define RegOcp_Address                      (0x0Bu)
#define RegLna_Address                      (0x0Cu)
#define RegFifoAddrPtr_Address              (0x0Du)
#define RegFifoTxBaseAddr_Address           (0x0Eu)
#define RegFifoRxBaseAddr_Address           (0x0Fu)
#define RegIrqFlags_Address                 (0x10u)
#define RegIrqFlagsMask_Address             (0x11u)
#define RegFreqIfMsb_Address                (0x12u)
#define RegFreqIFLsb_Address                (0x13u)
#define RegSymbTimeoutMsb_Address           (0x14u)
#define RegSymbTimeoutLsb_Address           (0x15u)
#define RegTxCfg_Address                    (0x16u)
#define RegPayloadLength_Address            (0x17u)
#define RegPreambleMsb_Address              (0x18u)
#define RegPreambleLsb_Address              (0x19u)
#define RegModulationCfg_Address            (0x1Au)
#define RegRfMode_Address                   (0x1Bu)
#define RegHopPeriod_Address                (0x1Cu)
#define RegNbRxBytes_Address                (0x1Du)
#define RegRxHeaderInfo_Address             (0x1Eu)
#define RegRxHeaderCntValue_Address         (0x1Fu)
#define RegRxPacketCntValue_Address         (0x20u)
#define RegModemStat_Address                (0x21u)
#define RegPktSnrValue_Address              (0x22u)
#define RegRssiValue_Address                (0x23u)
#define RegPktRssiValue_Address             (0x24u)
#define RegHopChannel_Address               (0x25u)
#define RegRxDataAddr_Address               (0x26u)
#define RegDioMapping1_Address              (0x40u)
#define RegDioMapping2_Address              (0x41u)
#define RegVersion_Address                  (0x42u)
#define RegTcxo_Address                     (0x4Bu)
#define RegPaDac_Address                    (0x4Du)
#define RegFormerTemp_Address               (0x5Bu)
#define RegAgcRef_Address                   (0x61u)
#define RegAgcThresh1_Address               (0x62u)
#define RegAgcThresh2_Address               (0x63u)
#define RegAgcThresh3_Address               (0x64u)

/******************  Register Memory Address Pointers  *******************/
#define RegFifo                             ((RFM95_TypeDef *) RegFifo_Address)
#define RegOpMode                           ((RFM95_TypeDef *) RegOpMode_Address)
#define RegFrfMsb                           ((RFM95_TypeDef *) RegFrfMsb_Address)
#define RegFrfMid                           ((RFM95_TypeDef *) RegFrfMid_Address)
#define RegFrfLsb                           ((RFM95_TypeDef *) RegFrfLsb_Address)
#define RegPaConfig                         ((RFM95_TypeDef *) RegPaConfig_Address)
#define RegPaRamp                           ((RFM95_TypeDef *) RegPaRamp_Address)
#define RegOcp                              ((RFM95_TypeDef *) RegOcp_Address)
#define RegLna                              ((RFM95_TypeDef *) RegLna_Address)
#define RegFifoAddrPtr                      ((RFM95_TypeDef *) RegFifoAddrPtr_Address)
#define RegFifoTxBaseAddr                   ((RFM95_TypeDef *) RegFifoTxBaseAddr_Address)
#define RegFifoRxBaseAddr                   ((RFM95_TypeDef *) RegFifoRxBaseAddr_Address)
#define RegIrqFlags                         ((RFM95_TypeDef *) RegIrqFlags_Address)
#define RegIrqFlagsMask                     ((RFM95_TypeDef *) RegIrqFlagsMask_Address)
#define RegFreqIfMsb                        ((RFM95_TypeDef *) RegFreqIfMsb_Address)
#define RegFreqIFLsb                        ((RFM95_TypeDef *) RegFreqIFLsb_Address)
#define RegSymbTimeoutMsb                   ((RFM95_TypeDef *) RegSymbTimeoutMsb_Address)
#define RegSymbTimeoutLsb                   ((RFM95_TypeDef *) RegSymbTimeoutLsb_Address)
#define RegTxCfg                            ((RFM95_TypeDef *) RegTxCfg_Address)
#define RegPayloadLength                    ((RFM95_TypeDef *) RegPayloadLength_Address)
#define RegPreambleMsb                      ((RFM95_TypeDef *) RegPreambleMsb_Address)
#define RegPreambleLsb                      ((RFM95_TypeDef *) RegPreambleLsb_Address)
#define RegModulationCfg                    ((RFM95_TypeDef *) RegModulationCfg_Address)
#define RegRfMode                           ((RFM95_TypeDef *) RegRfMode_Address)
#define RegHopPeriod                        ((RFM95_TypeDef *) RegHopPeriod_Address)
#define RegNbRxBytes                        ((RFM95_TypeDef *) RegNbRxBytes_Address)
#define RegRxHeaderInfo                     ((RFM95_TypeDef *) RegRxHeaderInfo_Address)
#define RegRxHeaderCntValue                 ((RFM95_TypeDef *) RegRxHeaderCntValue_Address)
#define RegRxPacketCntValue                 ((RFM95_TypeDef *) RegRxPacketCntValue_Address)
#define RegModemStat                        ((RFM95_TypeDef *) RegModemStat_Address)
#define RegPktSnrValue                      ((RFM95_TypeDef *) RegPktSnrValue_Address)
#define RegRssiValue                        ((RFM95_TypeDef *) RegRssiValue_Address)
#define RegPktRssiValue                     ((RFM95_TypeDef *) RegPktRssiValue_Address)
#define RegHopChannel                       ((RFM95_TypeDef *) RegHopChannel_Address)
#define RegRxDataAddr                       ((RFM95_TypeDef *) RegRxDataAddr_Address)
#define RegDioMapping1                      ((RFM95_TypeDef *) RegDioMapping1_Address)
#define RegDioMapping2                      ((RFM95_TypeDef *) RegDioMapping2_Address)
#define RegVersion                          ((RFM95_TypeDef *) RegVersion_Address)
#define RegTcxo                             ((RFM95_TypeDef *) RegTcxo_Address)
#define RegPaDac                            ((RFM95_TypeDef *) RegPaDac_Address)
#define RegFormerTemp                       ((RFM95_TypeDef *) RegFormerTemp_Address)
#define RegAgcRef                           ((RFM95_TypeDef *) RegAgcRef_Address)
#define RegAgcThresh1                       ((RFM95_TypeDef *) RegAgcThresh1_Address)
#define RegAgcThresh2                       ((RFM95_TypeDef *) RegAgcThresh2_Address)
#define RegAgcThresh3                       ((RFM95_TypeDef *) RegAgcThresh3_Address)

/***********************  RegFifo  ************************/
#define RegFifo_Fifo_Pos                    (0u)
#define RegFifo_Fifo_Msk                    (0xFFu << RegFifo_Fifo_Pos)
#define RegFifo_Fifo                        RegFifo_Fifo_Msk

/**********************  RegOpMode  ***********************/
#define RegOpMode_Mode_Pos                  (0u)
#define RegOpMode_Mode_Msk                  (0x3u << RegOpMode_Mode_Pos)
#define RegOpMode_Mode                      RegOpMode_Mode_Msk
#define RegOpMode_Mode_0                    (0x1u << RegOpMode_Mode_Pos)
#define RegOpMode_Mode_1                    (0x2u << RegOpMode_Mode_Pos)
#define RegOpMode_Mode_2                    (0x4u << RegOpMode_Mode_Pos)
#define RegOpMode_LowFrequencyModeOn_Pos    (3u)
#define RegOpMode_LowFrequencyModeOn_Msk    (0x1 << RegOpMode_LowFrequencyModeOn_Pos)
#define RegOpMode_LowFrequencyModeOn        RegOpMode_LowFrequencyModeOn_Msk
#define RegOpMode_AccessSharedReg_Pos       (6u)
#define RegOpMode_AccessSharedReg_Msk       (0x1 << RegOpMode_AccessSharedReg_Pos)
#define RegOpMode_AccessSharedReg           RegOpMode_AccessSharedReg_Msk
#define RegOpMode_LongRangeMode_Pos         (7u)
#define RegOpMode_LongRangeMode_Msk         (0x1u << RegOpMode_LongRangeMode_Pos)
#define RegOpMode_LongRangeMode             RegOpMode_LongRangeMode_Msk

/***********************  RegFrMsb  ************************/
#define RegFrMsb_Frf_Pos                    (0u)
#define RegFrMsb_Frf_Msk                    (0xFFu << RegFrMsb_Frf_Pos)
#define RegFrMsb_Frf                        RegFrMsb_Frf_Msk

/***********************  RegFrMid  ************************/
#define RegFrMid_Frf_Pos                    (0u)
#define RegFrMid_Frf_Msk                    (0xFFu << RegFrMid_Frf_Pos)
#define RegFrMid_Frf                        RegFrMid_Frf_Msk

/***********************  RegFrLsb  ************************/
#define RegFrLsb_Frf_Pos                    (0u)
#define RegFrLsb_Frf_Msk                    (0xFFu << RegFrLsb_Frf_Pos)
#define RegFrLsb_Frf                        RegFrLsb_Frf_Msk

/**********************  RegPaConfig  **********************/
#define RegPaConfig_OutputPower_Pos         (0u)
#define RegPaConfig_OutputPower_Msk         (0xFu << RegPaConfig_OutputPower_Pos)
#define RegPaConfig_OutputPower             RegPaConfig_OutputPower_Msk
#define RegPaConfig_MaxPower_Pos            (4u)
#define RegPaConfig_MaxPower_Msk            (0x7u << RegPaConfig_MaxPower_Pos)
#define RegPaConfig_MaxPower                RegPaConfig_MaxPower_Msk
#define RegPaConfig_PaSelect_Pos            (7u)
#define RegPaConfig_PaSelect_Msk            (0x1u << RegPaConfig_PaSelect_Pos)
#define RegPaConfig_PaSelect                RegPaConfig_PaSelect_Msk

/***********************  RegPaRamp  ***********************/
#define RegPaRamp_PaRamp_Pos                (0u)
#define RegPaRamp_PaRamp_Msk                (0xFu << RegPaRamp_PaRamp_Pos)
#define RegPaRamp_PaRamp                    RegPaRamp_PaRamp_Msk

/***********************  RegOcp  *************************/
#define RegOcp_OcpTrim_Pos                  (0u)
#define RegOcp_OcpTrim_Msk                  (0x1Fu << RegOcp_OcpTrim_Pos)
#define RegOcp_OcpTrim                      RegOcp_OcpTrim_Msk
#define RegOcp_OcpOn_Pos                    (5u)
#define RegOcp_OcpOn_Msk                    (0x1u << RegOcp_OcpOn_Pos)
#define RegOcp_OcpOn                        RegOcp_OcpOn_Msk

/***********************  RegLna  *************************/
#define RegLnaBoostHf_Pos                   (0u)
#define RegLnaBoostHf_Msk                   (0x3u << RegLnaBoostHf_Pos)
#define RegLnaBoostHf                       RegLnaBoostHf_Msk
#define RegLnaBoostLf_Pos                   (3u)
#define RegLnaBoostLf_Msk                   (0x3u << RegLnaBoostLf_Pos)
#define RegLnaBoostLf                       RegLnaBoostLf_Msk
#define RegLnaGain_Pos                      (5u)
#define RegLnaGain_Msk                      (0x7u << RegLnaGain_Pos)
#define RegLnaGain                          RegLnaGain_Msk