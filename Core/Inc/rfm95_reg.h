/************************************************
* @file    rfm95_reg.h 
* @author  APBashara
* @date    6/2024
* 
* @brief   RFM95 Register Address Definitions
***********************************************/

#ifndef RFM95_REG_H
#define RFM95_REG_H
#endif

// RFM95W Register Macros

#define RFM95_RegFifo                             (0x00u)
#define RFM95_RegOpMode                           (0x01u)
#define RFM95_RegFrfMsb                           (0x06u)
#define RFM95_RegFrfMid                           (0x07u)
#define RFM95_RegFrfLsb                           (0x08u)
#define RFM95_RegPaConfig                         (0x09u)
#define RFM95_RegPaRamp                           (0x0Au)
#define RFM95_RegOcp                              (0x0Bu)
#define RFM95_RegLna                              (0x0Cu)
#define RFM95_RegFifoAddrPtr                      (0x0Du)
#define RFM95_RegFifoTxBaseAddr                   (0x0Eu)
#define RFM95_RegFifoRxBaseAddr                   (0x0Fu)
#define RFM95_RegIrqFlags                         (0x10u)
#define RFM95_RegIrqFlagsMask                     (0x11u)
#define RFM95_RegFreqIfMsb                        (0x12u)
#define RFM95_RegFreqIFLsb                        (0x13u)
#define RFM95_RegSymbTimeoutMsb                   (0x14u)
#define RFM95_RegSymbTimeoutLsb                   (0x15u)
#define RFM95_RegTxCfg                            (0x16u)
#define RFM95_RegPayloadLength                    (0x17u)
#define RFM95_RegPreambleMsb                      (0x18u)
#define RFM95_RegPreambleLsb                      (0x19u)
#define RFM95_RegModulationCfg                    (0x1Au)
#define RFM95_RegRfMode                           (0x1Bu)
#define RFM95_RegHopPeriod                        (0x1Cu)
#define RFM95_RegNbRxBytes                        (0x1Du)
#define RFM95_RegRxHeaderInfo                     (0x1Eu)
#define RFM95_RegRxHeaderCntValue                 (0x1Fu)
#define RFM95_RegRxPacketCntValue                 (0x20u)
#define RFM95_RegModemStat                        (0x21u)
#define RFM95_RegPktSnrValue                      (0x22u)
#define RFM95_RegRssiValue                        (0x23u)
#define RFM95_RegPktRssiValue                     (0x24u)
#define RFM95_RegHopChannel                       (0x25u)
#define RFM95_RegRxDataAddr                       (0x26u)
#define RFM95_RegDioMapping1                      (0x40u)
#define RFM95_RegDioMapping2                      (0x41u)
#define RFM95_RegVersion                          (0x42u)
#define RFM95_RegTcxo                             (0x4Bu)
#define RFM95_RegPaDac                            (0x4Du)
#define RFM95_RegFormerTemp                       (0x5Bu)
#define RFM95_RegAgcRef                           (0x61u)
#define RFM95_RegAgcThresh1                       (0x62u)
#define RFM95_RegAgcThresh2                       (0x63u)
#define RFM95_RegAgcThresh3                       (0x64u)
