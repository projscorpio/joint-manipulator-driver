#include "encoder.h"

extern SPI_HandleTypeDef hspi1;

uint8_t isFirstMeasurement = 1;
int16_t currentRawPos, lastRawPos;
int32_t indexSum;
int8_t rolloverIndex = 0;
volatile uint8_t sdata[3];

static uint16_t SPIGetPos() {
  static int errorCount = 0;

  uint16_t rawVal = 0;
  uint8_t parity;
  //uint8_t cof, ocf, lin, magInc, magDec;


  HAL_GPIO_WritePin(CS_TEST_GPIO_Port, CS_TEST_Pin, GPIO_PIN_SET);

  rawVal = (((sdata[0] << 8)| sdata[1]) >> 4) & 0x7FFF;

//  ocf = ((sdata[1] >> 2) & 0x01);
//  cof = ((sdata[1] >> 1) & 0x01);
//  lin = sdata[1] & 0x01;
//  magInc = ((sdata[2] >> 7) & 0x01);
//  magDec = ((sdata[2] >> 6) & 0x01);
  parity = ((sdata[2] >> 5) & 0x01);

  if(parity != 0){
    errorCount++;
  }

  return rawVal;
}


void InitSPIRec(){
	 HAL_GPIO_WritePin(CS_TEST_GPIO_Port, CS_TEST_Pin, GPIO_PIN_RESET);

	  if(HAL_SPI_Receive_IT(&hspi1, (uint8_t *)&sdata[0], 3 ) != HAL_OK) {
	    Error_Handler();
	  }

}
float rawPosToRawAngle(int32_t pos) {
  return ((float)(pos)/4096.0f) * 360.0f;
}

int16_t rolloverOut(uint16_t c1, uint16_t c2) {
  return (ENC_MAX - c2) + c1 + 1;
}

int16_t halfRange() {
  return ((ENC_MAX - ENC_MIN) / 2) + 1;
}

int16_t getChange(int32_t valIn, int32_t lastValue) {
  if (valIn == lastValue || valIn < ENC_MIN || valIn > ENC_MAX) {
    return 0;  // No change, or out of range
  }

  int16_t outValue = 0;
  if (abs(valIn - lastValue) >= halfRange()){  // Assume rollover

    if (valIn < lastValue) {     // Rollover, going up
      outValue = rolloverOut(valIn, lastValue);
      rolloverIndex++;
    }
    else if (valIn > lastValue) {     // Rollover, going down
      outValue = -rolloverOut(lastValue, valIn);
      rolloverIndex--;
    }
  }
  else {  // Normal change
    outValue = valIn - lastValue;
  }

  return outValue;
}

float getAngle(void) {
  if(isFirstMeasurement == 1){
    isFirstMeasurement = 0;
    indexSum = SPIGetPos();
    lastRawPos = indexSum;
  }

  currentRawPos = SPIGetPos();
  indexSum += getChange(currentRawPos, lastRawPos);
  lastRawPos = currentRawPos;

  return rawPosToRawAngle(indexSum);
}
