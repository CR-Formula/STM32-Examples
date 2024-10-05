static uint32_t SD_Command(uint32_t cmd, uint32_t resp, uint32_t arg);
static uint32_t SD_Response(uint32_t *response, uint32_t type);
static void SD_Panic(uint32_t code, uint8_t *message);
static void SD_StartBlockTransfer(uint8_t *buf, uint32_t cnt, uint32_t dir);

static uint32_t SDType;
static uint32_t RCA;

static volatile uint32_t SDIOTxRx=0;

//Private Write buffers
static uint8_t DatBuf[512*2];  //2 blocks (One will be in transit while other is being filled)
static uint8_t *pDatBuf=DatBuf;
static uint32_t BufCnt=0;

#define DATATIMEOUT   (0xFFFFFF)  //I simply made this up. A method for computing a realistic values from CSD is described in the specs.

//SDIO Commands  Index 
#define CMD0          ((uint8_t)0)
#define CMD8          ((uint8_t)8)
#define CMD55         ((uint8_t)55)
#define ACMD41        ((uint8_t)41)
#define CMD2          ((uint8_t)2)
#define CMD3          ((uint8_t)3)
#define CMD9          ((uint8_t)9)
#define CMD7          ((uint8_t)7)
#define ACMD6         ((uint8_t)6)
#define CMD24         ((uint8_t)24)
#define CMD25         ((uint8_t)25)
#define CMD12         ((uint8_t)12)
#define CMD13         ((uint8_t)13)
#define CMD17         ((uint8_t)17)
#define CMD18         ((uint8_t)18)

//Auxilary defines
#define NORESP        (0x00)
#define SHRESP        (0x40)
#define LNRESP        (0xC0)
#define R3RESP        (0xF40)  //Note this is totaly out of standard. However, becouse of the masking in SD_Command it will be processed as SHRESP
//R3 does not contain a valid CRC. Therefore, CCRCFAIL is set and CMDREND is never set for R3.
//To properly process R3, exit the loop CCRCFAIL condition and don't check CMDREND

#define RESP_R1       (0x01)
#define RESP_R1b      (0x02)
#define RESP_R2       (0x03)
#define RESP_R3       (0x04)
#define RESP_R6       (0x05)
#define RESP_R7       (0x06)

#define UM2SD         (0x00)  //Transfer Direction
#define SD2UM         (0x02)


void SD_LowLevel_Init(void) {
  uint32_t tempreg;
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  // GPIOC and GPIOD Periph clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

  //Initialize the pins
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_SDIO);

  // Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Configure PD.02 CMD line 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  // Configure PC.12 pin: CLK pin
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  //Enable the SDIO APB2 Clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);

  // Enable the DMA2 Clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  
  //Initialize the SDIO (with initial <400Khz Clock)
  tempreg=0;  //Reset value
  tempreg|=SDIO_CLKCR_CLKEN;  //Clock is enabled
  tempreg|=(uint32_t)0x76;  //Clock Divider. Clock=48000/(118+2)=400Khz
  //Keep the rest at 0 => HW_Flow Disabled, Rising Clock Edge, Disable CLK ByPass, Bus Width=0, Power save Disable
  SDIO->CLKCR=tempreg;
  
  //Power up the SDIO
  SDIO->POWER = 0x03;
  
}

void SD_Init(void){
  //uint32_t data;
  uint32_t response;
  uint32_t TimeOut=0xFFFF;
  uint32_t tempreg;
  
  //CMD0: GO_IDLE_STATE (No Response)
  SD_Command(CMD0, NORESP, 0);
      
  //CMD8: SEND_IF_COND  //Response to CMD8 is R7. But I will ignore that response
  SD_Command(CMD8, SHRESP, 0x000001AA); //Non v2.0 compliant sd's will cause panic here due to the timeout
  SD_Response(&response, RESP_R7); //AA is the check pattern. If response does not match with it, execution will be blocked in panic
  
  
  while (1) {
    ////Send ACMD41    
    //CMD55
    SD_Command(CMD55, SHRESP, 0); //Note that argument should be RCA. But at this point RCA of SD is 0. (It will be changed after cmd3)
    SD_Response(&response, RESP_R1);
    
    //ACMD41 (Response is R3 which does not contain any CRC)
    //Second argument in the argument indicates that host supports SDHC. We will check acmd41 response if the SD is SC or HC
    SD_Command(ACMD41, R3RESP, (uint32_t) 0x80100000 | (uint32_t) 0x40000000);
    SD_Response(&response, RESP_R3);
    
    //Check the ready status in the response (R3) 
    if ((response >> 31) == 1) {  //When card is busy this bit will be 0
      //Card is now initialized. Check to see if SD is SC or HC
      SDType=(response & 0x40000000) >> 30;  //1=HC, 0=SC
      break;
    } else {
      TimeOut--;
      if (!TimeOut) {SD_Panic(ACMD41, "SDIO:ACMD41 Timeout\n");}      
    }
   }
  
  //Now we are in the Ready State. Ask for CID using CMD2
  //Response is R2. RESP1234 are filled with CID. I will ignore them
  SD_Command(CMD2, LNRESP, 0);
  
  //Now the card is in the identification mode. Request for the RCA number with cmd3
  SD_Command(CMD3, SHRESP, 0);
  SD_Response(&response, RESP_R6);
  //Read the RCA
  RCA=response>>16;
  
  //Now the card is in stand-by mode. From this point on I can change frequency as I wish (max24MHz)
  
  
  //Use cmd9 to read the card specific information
  //Response is R2 with CSI. I will ignore the response
  SD_Command(CMD9, LNRESP, (RCA << 16));
  
  
  //Put the Card in the tranfer mode using cmd7. (I will change the clock spped later together with bus width)
  //Bus width can only be changed in transfer mode
  SD_Command(CMD7, SHRESP, (RCA << 16));
  SD_Response(&response, RESP_R1);
  
  //Change the bus-width with cmd6
  //CMD55
  SD_Command(CMD55, SHRESP, (RCA << 16)); //Note the real RCA in the argument
  SD_Response(&response, RESP_R1);
  //ACMD6
  SD_Command(ACMD6, SHRESP, 0x02);
  SD_Response(&response, RESP_R1);
  
  //Configure SDIO->CLKCr for wide-bus width and new clock
  tempreg=0;  //Reset value
  tempreg|=(0x01)<<11; //4 bit Bus Width
  tempreg|=SDIO_CLKCR_CLKEN;  //Clock is enabled
  //Keep the rest at 0=> HW_Flow:Disabled, Rising Edge, Disable bypass, Power save Disable, Clock Division=0
  //As the clock divider=0 => New clock=48/(Div+2)=48/2=24
  SDIO->CLKCR=tempreg;
  
  //Now we can start issuing read/write commands
}