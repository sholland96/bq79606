#include <Arduino.h>
#include <AceCRC.h>
#include <bq79606.h>

#define debug;

using namespace ace_crc::crc16modbus_nibblem;

uint16_t init_poly[] = {0xFFFF};
uint8_t msg[] = {0xD0, 0x01, 0x1D, 0x00};

int UART_RX_RDY;

int bRes = 0;
int count = 10000;
uint8_t pFrame[(MAXBYTES+6)*TOTALBOARDS];
byte bBuf[8];
byte bReturn = 0;
byte response_frame[(MAXBYTES+6)*TOTALBOARDS];
byte response_frame2[(MAXBYTES+6)*TOTALBOARDS];
byte bFrame[(2+6)*TOTALBOARDS];
int nCurrentBoard = 0;
int nWake = 2;
int nFault = 3;
int TX = 18;
int debugIO = 8;
float ADCres = 0.0001907349;
int temp;
crc_t crc = crc_init();

void setup()
{
#ifdef debug
  Serial.begin(57600);
#endif

  pinMode(nWake, OUTPUT);
  digitalWrite(nWake, HIGH);
  
  pinMode(nFault, INPUT);

#ifdef debug
  Serial.println("commReset");
#endif
  commReset();

#ifdef debug
  Serial.println("wake79606");
#endif
  wake79606();
  delay(5*TOTALBOARDS);

#ifdef debug
  Serial.println("autoAddress");
#endif
  autoAddress (0);

  writeReg(0, COMM_TO, 0x00, 1, FRMWRT_ALL_NR);//disable COMM timeout because printf takes a long time between reads
  writeReg(0, SYSFLT1_FLT_RST, 0xFFFFFF, 3, FRMWRT_ALL_NR);//reset system faults
  writeReg(0, SYSFLT1_FLT_MSK, 0xFFFFFF, 3, FRMWRT_ALL_NR);//mask system faults (so we can test boards and not worry about triggering these faults accidentally)
  
  for (int i = 0; i < TOTALBOARDS; i++) {
    readReg(i, ECC_TEST, response_frame2, 1, 0, FRMWRT_SGL_R);
  } 
  
  readReg(0, ECC_TEST, response_frame2, 1, 0, FRMWRT_ALL_R);

  initDevices();

  //memset(response_frame, 0, sizeof(response_frame));
  //read back data (3 cells and 2 bytes each cell)
  Serial.println("Reading CSC #1 cell voltages");
  for (int i = 1; i < TOTALBOARDS; i++) {
    readReg(i, VCELL1H, response_frame, 3*2, 0, FRMWRT_SGL_R);
    Serial.print("Cell #1 HEX: 0x");
    Serial.print(response_frame[4], HEX);
    Serial.print(response_frame[5], HEX);
    temp = response_frame[5]+response_frame[4]*256;
    Serial.print(", ");
    Serial.print((float)temp*ADCres, 3);
    Serial.print(", ");
    delay(2);
    readReg(i, VCELL2H, response_frame, 3*2, 0, FRMWRT_SGL_R);
    Serial.print("Cell #2 HEX: 0x");
    Serial.print(response_frame[4], HEX);
    Serial.print(response_frame[5], HEX);
    temp = response_frame[5]+response_frame[4]*256;
    Serial.print(", ");
    Serial.print((float)temp*ADCres, 3);
    Serial.print(", ");

    readReg(i, VCELL3H, response_frame, 3*2, 0, FRMWRT_SGL_R);
    Serial.print("Cell #3 HEX: 0x");
    Serial.print(response_frame[4], HEX);
    Serial.print(response_frame[5], HEX);
    temp = response_frame[5]+response_frame[4]*256;
    Serial.print(", ");
    Serial.print((float)temp*ADCres, 3);
    Serial.println();
  }

/*
  ringFlip();

  delay(1);

  autoAddress(1);

  for (int i = 0; i < TOTALBOARDS; i++) {
    readReg(i, ECC_TEST, response_frame2, 1, 0, FRMWRT_SGL_R);
  } 
  
  readReg(0, ECC_TEST, response_frame2, 1, 0, FRMWRT_ALL_R);
*/
}

void loop() {
  
  
  
}

void commClear()
{
  Serial1.end();
  pinMode(TX, OUTPUT);
	digitalWrite(TX, LOW); // set output to low
  // TX must be held low for minimum 15 to maximum of 20 bit periods. 
	delayMicroseconds(15);//1/BAUDRATE/16*156*1.01); // ~= 1/BAUDRATE/16*(155+1)*1.01
  digitalWrite(TX, HIGH); // set output to high
	Serial1.begin(BAUDRATE);
}

void commReset()
{
  Serial1.end();
  pinMode(TX, OUTPUT);
	digitalWrite(TX, LOW); // set output to low
	delayMicroseconds(500); // should cover any possible baud rate
  digitalWrite(TX, HIGH); // set output to high
	Serial1.begin(250000); //set microcontroller to 250k to talk to base
  //BASE DEVICE NOW AT 250K BAUDRATE, STACK DEVICES ARE WHATEVER BAUDRATE THEY WERE BEFORE
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //tell the base device to set its baudrate to the chosen BAUDRATE, and propagate to the rest of the stack
  //then set the microcontroller to the appropriate baudrate to match
  if(BAUDRATE == 1000000)
  {
    //set COMM_CTRL and DAISY_CHAIN_CTRL registers
    writeReg(0, COMM_CTRL, 0x3C3C, 2, FRMWRT_ALL_NR);
    //ALL 606 DEVICES ARE NOW AT 1M BAUDRATE

    //set the microcontroller to 1M baudrate
    Serial1.begin(1000000);
  }
  else if(BAUDRATE == 500000)
  {
      writeReg(0, COMM_CTRL, 0x383C, 2, FRMWRT_ALL_NR);
      Serial1.begin(500000);
  }
  else if(BAUDRATE == 250000)
  {
      writeReg(0, COMM_CTRL, 0x343C, 2, FRMWRT_ALL_NR);
      Serial1.begin(250000);
  }
  else if(BAUDRATE == 125000)
  {
      writeReg(0, COMM_CTRL, 0x303C, 2, FRMWRT_ALL_NR);
      Serial1.begin(125000);
  }
  else
  {
      //Choosing default 1M baudrate
      writeReg(0, COMM_CTRL, 0x3C3C, 2, FRMWRT_ALL_NR);
      Serial1.begin(1000000);
  }
  delayMicroseconds(100);
}

void wake79606()
{
  digitalWrite(nWake, LOW);
  //digitalWrite(debugIO, LOW);
  delayMicroseconds(250); //250us to 300us
  digitalWrite(nWake, HIGH); // deassert wake
}

void autoAddress(uint8_t NorthSouth)//North = 0, South = 1
{
  //dummy write to ECC_TEST (sync DLL)
    //writeReg(0,  ECC_TEST, 0x00, 1, FRMWRT_ALL_NR);

    //clear CONFIG in case it is set
    writeReg(0, CONFIG, 0x00, 1, FRMWRT_ALL_NR);

    //enter auto addressing mode
    if(NorthSouth == 0){
      writeReg(0, CONTROL1, 0x01, 1, FRMWRT_ALL_NR);
    }else{
      writeReg(0, CONTROL1, 0x81, 1, FRMWRT_ALL_NR);
    }

    //set addresses for all boards in daisy-chain
    for (nCurrentBoard = 0; nCurrentBoard < TOTALBOARDS; nCurrentBoard++)
    {
        writeReg(nCurrentBoard, DEVADD_USR, nCurrentBoard, 1, FRMWRT_ALL_NR);
    }

    //set all devices as a stack device
    writeReg(0, CONFIG, 0x02, 1, FRMWRT_ALL_NR);

    //if there's only 1 board, it's the base AND the top of stack, so change it to those
    if(TOTALBOARDS==1)
    {
        writeReg(0, CONFIG, 0x01, 1, FRMWRT_SGL_NR);
    }
    //otherwise set the base and top of stack individually
    else
    {
        writeReg(0, CONFIG, 0x00, 1, FRMWRT_SGL_NR);             //base
        writeReg(TOTALBOARDS-1, CONFIG, 0x03, 1, FRMWRT_SGL_NR); //top of stack
    }

    //delay(50);
    //dummy read from ECC_TEST (sync DLL)
    //readReg(TOTALBOARDS-1, ECC_TEST, response_frame2, 1, 0, FRMWRT_SGL_R);
}

int writeReg (byte bID, uint16_t wAddr, uint64_t dwData, byte bLen, byte bWriteType)
{
  bRes = 0;
  memset(bBuf,0,sizeof(bBuf));
  
  switch (bLen) {
	case 1:
		bBuf[0] = dwData & 0x00000000000000FF;
		bRes = writeFrame(bID, wAddr, bBuf, 1, bWriteType);
		break;
	case 2:
		bBuf[0] = (dwData & 0x000000000000FF00) >> 8;
		bBuf[1] = dwData & 0x00000000000000FF;
		bRes = writeFrame(bID, wAddr, bBuf, 2, bWriteType);
		break;
	case 3:
		bBuf[0] = (dwData & 0x0000000000FF0000) >> 16;
		bBuf[1] = (dwData & 0x000000000000FF00) >> 8;
		bBuf[2] = dwData & 0x00000000000000FF;
		bRes = writeFrame(bID, wAddr, bBuf, 3, bWriteType);
		break;
	case 4:
		bBuf[0] = (dwData & 0x00000000FF000000) >> 24;
		bBuf[1] = (dwData & 0x0000000000FF0000) >> 16;
		bBuf[2] = (dwData & 0x000000000000FF00) >> 8;
		bBuf[3] = dwData & 0x00000000000000FF;
		bRes = writeFrame(bID, wAddr, bBuf, 4, bWriteType);
		break;
	case 5:
		bBuf[0] = (dwData & 0x000000FF00000000) >> 32;
		bBuf[1] = (dwData & 0x00000000FF000000) >> 24;
		bBuf[2] = (dwData & 0x0000000000FF0000) >> 16;
		bBuf[3] = (dwData & 0x000000000000FF00) >> 8;
		bBuf[4] = dwData & 0x00000000000000FF;
		bRes = writeFrame(bID, wAddr, bBuf, 5, bWriteType);
		break;
	case 6:
		bBuf[0] = (dwData & 0x0000FF0000000000) >> 40;
		bBuf[1] = (dwData & 0x000000FF00000000) >> 32;
		bBuf[2] = (dwData & 0x00000000FF000000) >> 24;
		bBuf[3] = (dwData & 0x0000000000FF0000) >> 16;
		bBuf[4] = (dwData & 0x000000000000FF00) >> 8;
		bBuf[5] = dwData & 0x00000000000000FF;
		bRes = writeFrame(bID, wAddr, bBuf, 6, bWriteType);
		break;
	case 7:
		bBuf[0] = (dwData & 0x00FF000000000000) >> 48;
		bBuf[1] = (dwData & 0x0000FF0000000000) >> 40;
		bBuf[2] = (dwData & 0x000000FF00000000) >> 32;
		bBuf[3] = (dwData & 0x00000000FF000000) >> 24;
		bBuf[4] = (dwData & 0x0000000000FF0000) >> 16;
		bBuf[5] = (dwData & 0x000000000000FF00) >> 8;
		bBuf[6] = dwData & 0x00000000000000FF;
		bRes = writeFrame(bID, wAddr, bBuf, 7, bWriteType);
		break;
	case 8:
		bBuf[0] = (dwData & 0xFF00000000000000) >> 56;
		bBuf[1] = (dwData & 0x00FF000000000000) >> 48;
		bBuf[2] = (dwData & 0x0000FF0000000000) >> 40;
		bBuf[3] = (dwData & 0x000000FF00000000) >> 32;
		bBuf[4] = (dwData & 0x00000000FF000000) >> 24;
		bBuf[5] = (dwData & 0x0000000000FF0000) >> 16;
		bBuf[6] = (dwData & 0x000000000000FF00) >> 8;
		bBuf[7] = dwData & 0x00000000000000FF;
		bRes = writeFrame(bID, wAddr, bBuf, 8, bWriteType);
		break;
	default:
		break;
	}
	return bRes;
}

int readReg(byte bID, uint16_t wAddr, byte * pData, byte bLen, uint32_t dwTimeOut, byte bWriteType)
{
  bRes = 0;
	count = 100000;
	if (bWriteType == FRMWRT_SGL_R) {
		readFrameReq(bID, wAddr, bLen, bWriteType);
		memset(pData, 0, sizeof(pData));
		//sciEnableNotification(scilinREG, SCI_RX_INT);
		//sciReceive(scilinREG, bLen + 6, pData);
    while(!Serial1.available());
    bRes = bLen + 6;
    size_t responseSize = Serial1.readBytes(pData, bRes);

	} else if (bWriteType == FRMWRT_STK_R) {
		bRes = readFrameReq(bID, wAddr, bLen, bWriteType);
		memset(pData, 0, sizeof(pData));
    //sciEnableNotification(scilinREG, SCI_RX_INT);
    //sciReceive(scilinREG, (bLen + 6) * (TOTALBOARDS - 1), pData);
    while(!Serial1.available());
    bRes = (bLen + 6) * TOTALBOARDS;
    size_t responseSize = Serial1.readBytes(pData, bRes);

	} else if (bWriteType == FRMWRT_ALL_R) {
		bRes = readFrameReq(bID, wAddr, bLen, bWriteType);
		memset(pData, 0, sizeof(pData));
    //sciEnableNotification(scilinREG, SCI_RX_INT);
    //sciReceive(scilinREG, (bLen + 6) * TOTALBOARDS, pData);
    while(!Serial1.available());
    // get incoming byte:
    bRes = (bLen + 6) * TOTALBOARDS;
    size_t responseSize = Serial1.readBytes(pData, bRes);        
	} else {
		bRes = 0;
	}
//  return bRes;
}

int  writeFrame(byte bID, uint16_t wAddr, byte * pData, byte bLen, byte bWriteType)
{
  int bPktLen = 0;
	uint8_t * pBuf = pFrame;
	uint16_t wCRC;
	memset(pFrame, 0x7F, sizeof(pFrame));
	*pBuf++ = 0x80 | (bWriteType) | ((bWriteType & 0x10) ? bLen - 0x01 : 0x00); //Only include blen if it is a write; Writes are 0x90, 0xB0, 0xD0
	if (bWriteType == FRMWRT_SGL_R || bWriteType == FRMWRT_SGL_NR)
	{
		*pBuf++ = (bID & 0x00FF);
	}
	*pBuf++ = (wAddr & 0xFF00) >> 8;
	*pBuf++ = wAddr & 0x00FF;

	while (bLen--)
		*pBuf++ = *pData++;

	bPktLen = pBuf - pFrame;

	wCRC = 0xFFFF;
  wCRC = crc_calculate(pFrame, bPktLen); //CRC16(pFrame, bPktLen);
  //wCRC = CRC16(pFrame, bPktLen);
	*pBuf++ = wCRC & 0x00FF;
	*pBuf++ = (wCRC & 0xFF00) >> 8;
	bPktLen += 2;

  Serial1.write(pFrame, bPktLen);
  delayMicroseconds(250); //250us to 300us

	return bPktLen;
}

int  readFrameReq(byte bID, uint16_t wAddr, byte bByteToReturn, byte bWriteType)
{
  bReturn = bByteToReturn - 1;

	if (bReturn > 127)
		return 0;

	return writeFrame(bID, wAddr, &bReturn, 1, bWriteType);
}

void initDevices() {
    /*******Optional examples of some initialization functions*****/
    delay(1);//ms
    writeReg(0, COMM_TO, 0x00, 1, FRMWRT_ALL_NR); //Communication timeout disabled
    writeReg(0, TX_HOLD_OFF, 0x00, 1, FRMWRT_ALL_NR); //no transmit delay after stop bit
	
    /* mask all low level faults... user should unmask necessary faults */
    writeReg(0, GPIO_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR); //mask GPIO faults
    writeReg(0, UV_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR); //mask UV faults
    writeReg(0, OV_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR); //mask OV faults
    writeReg(0, UT_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR); //mask UT faults
    writeReg(0, OT_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR); //mask OT faults
    writeReg(0, TONE_FLT_MSK, 0x07, 1, FRMWRT_ALL_NR); //mask all tone faults
    writeReg(0, COMM_UART_FLT_MSK, 0x07, 1, FRMWRT_ALL_NR); //mask UART faults
    writeReg(0, COMM_UART_RC_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR); //mask UART fault contd
    writeReg(0, COMM_UART_RR_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_UART_TR_FLT_MSK, 0x03, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COMH_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COMH_RC_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COMH_RR_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COMH_TR_FLT_MSK, 0x03, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COML_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COML_RC_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COML_RR_FLT_MSK, 0x3F, 1, FRMWRT_ALL_NR);
    writeReg(0, COMM_COML_TR_FLT_MSK, 0x03, 1, FRMWRT_ALL_NR);
    writeReg(0, OTP_FLT_MSK, 0x07, 1, FRMWRT_ALL_NR); // mask otp faults
    writeReg(0, RAIL_FLT_MSK, 0xFF, 1, FRMWRT_ALL_NR); //mask power rail faults
    writeReg(0, SYSFLT1_FLT_MSK, 0x7F, 1, FRMWRT_ALL_NR); //sys fault  mask 1
    writeReg(0, SYSFLT2_FLT_MSK, 0xFF, 1, FRMWRT_ALL_NR); //sys fault mask 2
    writeReg(0, SYSFLT3_FLT_MSK, 0x7F, 1, FRMWRT_ALL_NR); //sys  fault  mask 3
    writeReg(0, OVUV_BIST_FLT_MSK, 0x03, 1, FRMWRT_ALL_NR); //mask ov/uv bist faults
    writeReg(0, OTUT_BIST_FLT_MSK, 0xFF, 1, FRMWRT_ALL_NR);
	
    writeReg(0, OVUV_CTRL, 0x07, 1, FRMWRT_ALL_NR); //enable all cell ov/uv
    writeReg(0, UV_THRESH, 0x53, 1, FRMWRT_ALL_NR); //sets cell UV to 2.8V
    writeReg(0, OV_THRESH, 0x5B, 1, FRMWRT_ALL_NR); //sets cell OV to 4.3V
    //WriteReg(0, OTUT_CTRL, 0x3F, 1, FRMWRT_ALL_NR); //enable GPIO OT/UT
    //WriteReg(0, OTUT_THRESH, 0xFF, 1, FRMWRT_ALL_NR); //sets OT to 35% TSREF, UT to 75%, programmabe in 1% increment
    writeReg(0, GPIO_ADC_CONF, 0x3F, 1, FRMWRT_ALL_NR); //configure GPIO as AUX voltage (absolute voltage, set to 0 for ratiometric)

    //SET UP MAIN ADC
    //writeReg(0, CELL_ADC_CTRL, 0x3F, 1, FRMWRT_SGL_NR); //enable conversions for all cells
    writeReg(0, CELL_ADC_CTRL, 0x07, 1, FRMWRT_ALL_NR); //enables ADC for all 3 cell channels
    writeReg(0, CELL_ADC_CONF1, 0x67, 1, FRMWRT_ALL_NR);  //set ADC LPF to 1.2Hz, 256 decimation ratio
    writeReg(0, CELL_ADC_CONF2, 0x08, 1, FRMWRT_ALL_NR);  //set continuous ADC conversions, and set minimum conversion interval
    writeReg(0, CELL1_GAIN, 0xF6, 1, FRMWRT_ALL_NR);  //set ADC gain calibration coefficient
    writeReg(0, CELL2_GAIN, 0xF6, 1, FRMWRT_ALL_NR);  //set ADC gain calibration coefficient
    writeReg(0, CELL3_GAIN, 0xF6, 1, FRMWRT_ALL_NR);  //set ADC gain calibration coefficient
    writeReg(0, CELL4_GAIN, 0xF9, 1, FRMWRT_ALL_NR);  //set ADC gain calibration coefficient
    writeReg(0, CELL1_OFF, 0xFC, 1, FRMWRT_ALL_NR);  //set ADC gain calibration coefficient
    writeReg(0, CELL2_OFF, 0xFC, 1, FRMWRT_ALL_NR);  //set ADC gain calibration coefficient
    writeReg(0, CELL3_OFF, 0xFC, 1, FRMWRT_ALL_NR);  //set ADC gain calibration coefficient  
    writeReg(0, CELL4_OFF, 0xFE, 1, FRMWRT_ALL_NR);  //set ADC gain calibration coefficient  
    writeReg(0, CONTROL2, 0x01, 1, FRMWRT_ALL_NR);  //CELL_ADC_GO = 1
    delayMicroseconds(3*TOTALBOARDS+901); //3us of re-clocking delay per board + 901us waiting for first ADC conversion to complete
}
void ringFlip()
{
  Serial.println("Flipping to the South direction");
  //For the base device: Disable daisy chain high COM RX and COM TX by writing DAISY_CHAIN_CTRL[COMHRX_EN]=0 and DAISY_CHAIN_CTRL [COMHTX_EN]=0
  writeReg(0, DAISY_CHAIN_CTRL, 0x30, 1, FRMWRT_SGL_NR);
  //For the base device: Enable daisy chain Low COM RX and COM TX by writing DAISY_CHAIN_CTRL[COMLRX_EN]=1 and DAISY_CHAIN_CTRL [COMLTX_EN]=1
  writeReg(0, DAISY_CHAIN_CTRL, 0x30, 1, FRMWRT_SGL_NR);
  //For the base device: Write 1 to DAISY_CHAIN_CTRL_EN in CONTROL2 register to ensure the COMH/COML TX/RX function is controlled by DAISY_CHAIN_CTRL register
  writeReg(0, CONTROL2, 0x40, 1, FRMWRT_SGL_NR);
  //For the base device: Write 1 to CONTROL1[DIR_SEL] to reverse the direction of the base and the next subsequent commands go to low side.
  writeReg(0, CONTROL1, 0x80, 1, FRMWRT_SGL_NR);
  //Send Broadcast Write Reverse Direction Command Frame to all devices to switch their direction.
  writeReg(0, CONTROL1, 0x80, 1, FRMWRT_REV_ALL_NR );
  //Send a Broadcast command to clear the CONFIG register of all devices to ensure earlier setting is cleared and the CONFIG[TOP_STACK] is cleared
  //writeReg(0, CONFIG, 0x00, 1, FRMWRT_ALL_NR );
}

