#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "MFRC522.h"



/*
 *
 * Function to get the version
 */
void PCD_Version(spi_device_handle_t spi){

    uint8_t ver = PCD_ReadRegister(spi,VersionReg);



    printf("Version:%x\r\n",ver);
    if (ver == 0x92)
    {
        printf("MFRC522 Version 2 detected.\n");
    }
    else if (ver == 0x91)
    {
        printf("MFRC522 Version 1 detected.\n");
    }
    else{
        printf("Is connected device MFRC522 ? If yes, check the wiring again.\n");

        for (int i = 5; i >= 0; i--) {
            printf("Restarting in %d seconds...\n", i);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        printf("Restarting Now.\n");
        fflush(stdout);
        esp_restart();
    }
}

/*
 *
 * Function to write data to MFRC522
 */
void PCD_WriteRegister(spi_device_handle_t spi , uint8_t Register , uint8_t value){

    esp_err_t ret;
    uint8_t reg = Register;
    uint8_t val = value;
    static spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.flags=SPI_TRANS_USE_TXDATA;
    t.length = 16;
    t.tx_data[0] = reg;
    t.tx_data[1] = val;
    ret = spi_device_queue_trans(spi,&t,10);
    assert(ret==ESP_OK);
    spi_transaction_t *rtrans;
    ret = spi_device_get_trans_result(spi,&rtrans,10);
    assert(ret==ESP_OK);
}

void PCD_WriteRegisterMany(spi_device_handle_t spi , uint8_t Register, uint8_t count, uint8_t *values){

    esp_err_t ret;
    uint8_t total[count+1];
    total[0] = Register;

    for (int i = 1; i <= count; ++i)
    {
        total[i] = values[i-1];
    }

    static spi_transaction_t t1;
    memset(&t1, 0, sizeof(t1));       //Zero out the transaction
    t1.length = 8*(count+1);
    t1.tx_buffer = total;

    ret = spi_device_transmit(spi,&t1);
    assert(ret==ESP_OK);
}

/*
 *
 * Function to read data from MFRC522
 */

uint8_t PCD_ReadRegister(spi_device_handle_t spi , uint8_t Register){

    esp_err_t ret;
    uint8_t reg = Register | 0x80;
    uint8_t val;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;
    t.tx_buffer = &reg;
    t.rx_buffer = &val;
    gpio_set_level(PIN_NUM_CS,0);
    ret = spi_device_transmit(spi,&t);
    assert(ret==ESP_OK);
    t.tx_buffer=(uint8_t*)0;
    ret = spi_device_transmit(spi,&t);
    gpio_set_level(PIN_NUM_CS,1);

    assert(ret==ESP_OK);

    return val;    
}

void PCD_ReadRegisterMany(spi_device_handle_t spi,
                                uint8_t Register,   ///< The register to read from. One of the PCD_Register enums.
                                uint8_t count,         ///< The number of bytes to read
                                uint8_t *values,       ///< Byte array to store the values in.
                                uint8_t rxAlign        ///< Only bit positions rxAlign..7 in values[0] are updated.
                                ) {
    if (count == 0) {
        return;
    }

    esp_err_t ret;
    uint8_t reg = 0x80 | Register;              // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                  //Zero out the transaction
    t.length =128;//8
    t.rxlength = 8*count;
    t.tx_buffer = &reg;
    t.rx_buffer = values;
    gpio_set_level(PIN_NUM_CS,0);
    ret = spi_device_transmit(spi,&t);
    assert(ret==ESP_OK);
    t.tx_buffer=(uint8_t*)0;
    ret = spi_device_transmit(spi,&t);
    gpio_set_level(PIN_NUM_CS,1);
    assert(ret==ESP_OK);

}


/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(spi_device_handle_t spi,uint8_t reg,   ///< The register to update. One of the PCD_Register enums.
                                        uint8_t mask           ///< The bits to clear.
                                      ) {
    uint8_t tmp;
    tmp = PCD_ReadRegister(spi,reg);
    PCD_WriteRegister(spi,reg, tmp & (~mask));      // clear bit mask
}


/**
 * Sets the bits given in mask in register reg.
 */

void PCD_SetRegisterBitMask(spi_device_handle_t spi , uint8_t reg, uint8_t mask){
    uint8_t tmp;
    tmp = PCD_ReadRegister(spi,reg);
    PCD_WriteRegister(spi,reg, tmp | mask);         // set bit mask
}
/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PICC_HaltA(spi_device_handle_t spi) {
	uint8_t result;
	uint8_t buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(spi,buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(spi,buffer, sizeof(buffer), NULL, 0,NULL,0,false);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()


/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void PCD_StopCrypto1(spi_device_handle_t spi) {
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(spi,Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

void PCD_Init(spi_device_handle_t spi){
    
    PCD_Version(spi);
    gpio_pad_select_gpio(PIN_NUM_RST);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);

    // Hard Reset RFID
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Reset baud rates
    PCD_WriteRegister(spi,TxModeReg, 0x00);
    PCD_WriteRegister(spi,RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(spi,ModWidthReg, 0x26);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.

    PCD_WriteRegister(spi,TModeReg, 0x80);          // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    PCD_WriteRegister(spi,TPrescalerReg, 0xA9);     // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25Î¼s.
    PCD_WriteRegister(spi,TReloadRegH, 0x03);       // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    PCD_WriteRegister(spi,TReloadRegL, 0xE8);
    PCD_WriteRegister(spi,TxASKReg, 0x40);      // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    PCD_WriteRegister(spi,ModeReg, 0x3D);       // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    PCD_AntennaOn(spi);                        // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
    printf("Initialization successful.\n");

}

void PCD_AntennaOn(spi_device_handle_t spi){
    uint8_t value = PCD_ReadRegister(spi,TxControlReg);
    if ((value & 0x03) != 0x03) {
        PCD_WriteRegister(spi,TxControlReg, value | 0x03);
    }
    printf("Antenna turned on.\n");
}

bool PICC_IsNewCardPresent(spi_device_handle_t spi){
    static uint8_t bufferATQA[4]={0,0,0,0};
    uint8_t bufferSize = sizeof(bufferATQA);
    // Reset baud rates
    PCD_WriteRegister(spi,TxModeReg, 0x00);
    PCD_WriteRegister(spi,RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(spi,ModWidthReg, 0x26);
      // printf("%x\n",PCD_ReadRegister(spi,ModWidthReg));
       // printf("%x\n",PCD_ReadRegister(spi,TxModeReg));
        //printf("%x\n",PCD_ReadRegister(spi,RxModeReg));


    uint8_t result = PICC_RequestA(spi,bufferATQA, &bufferSize);
    return (result == STATUS_OK || result == STATUS_COLLISION);
}


void PICC_GetTypeName(PICC_Type piccType) {
	switch (piccType) {
		case PICC_TYPE_ISO_14443_4:		printf("PICC compliant with ISO/IEC 14443-4\n"); break;
		case PICC_TYPE_ISO_18092:		printf("PICC compliant with ISO/IEC 18092 (NFC)\n"); break;
		case PICC_TYPE_MIFARE_MINI:		printf("MIFARE Mini, 320 bytes\n"); break;
		case PICC_TYPE_MIFARE_1K:		printf("MIFARE 1KB\n"); break;
		case PICC_TYPE_MIFARE_4K:		printf("MIFARE 4KB\n"); break;
		case PICC_TYPE_MIFARE_UL:		printf("MIFARE Ultralight or Ultralight C\n"); break;
		case PICC_TYPE_MIFARE_PLUS:		printf("MIFARE Plus\n"); break;
		case PICC_TYPE_MIFARE_DESFIRE:	printf("MIFARE DESFire\n");break;
		case PICC_TYPE_TNP3XXX:			printf("MIFARE TNP3XXX\n");break;
		case PICC_TYPE_NOT_COMPLETE:	printf("SAK indicates UID is not complete.\n");break;
		case PICC_TYPE_UNKNOWN:
		default:						printf("Unknown type\n");
	}
} // End PICC_GetTypeName()


/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 *
 * @return PICC_Type
 */
PICC_Type PICC_GetType(uint8_t sak		///< The SAK byte returned from PICC_Select().
										) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} // End PICC_GetType()
bool get_data(spi_device_handle_t spi)
{
	  static uint8_t bufferATQA[18]={0};
	    uint8_t bufferSize = sizeof(bufferATQA);

	    uint8_t result = PICC_CMD_MF(spi,bufferATQA, &bufferSize);

	printf("get-data-result%d",result);

	    return (result == STATUS_OK || result == STATUS_COLLISION);
}
uint8_t PICC_CMD_MF(spi_device_handle_t spi,uint8_t *bufferATQA,uint8_t *bufferSize)
{
	uint8_t buffer[18];

	buffer[0] = PICC_CMD_MF_READ;
		buffer[1] = 0;
	return PCD_TransceiveData(spi,buffer, 4, bufferATQA, bufferSize, NULL,0,false);

}

uint8_t PICC_RequestA(spi_device_handle_t spi,uint8_t *bufferATQA,uint8_t *bufferSize){
    return PICC_REQA_or_WUPA(spi,PICC_CMD_REQA, bufferATQA, bufferSize);
}

uint8_t PICC_REQA_or_WUPA(spi_device_handle_t spi,uint8_t command,uint8_t *bufferATQA,uint8_t *bufferSize){
    uint8_t validBits;
    uint8_t status;
    if(bufferATQA == NULL || *bufferSize < 2){
        return STATUS_NO_ROOM;
    }
    PCD_ClearRegisterBitMask(spi,CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.
    validBits = 7;                                      // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
    status = PCD_TransceiveData(spi,&command, 1, bufferATQA, bufferSize, &validBits,0,false);
    if (status != STATUS_OK) {
        return status;
    }
    if (*bufferSize != 2 || validBits != 0) {           // ATQA must be exactly 16 bits.
        return STATUS_ERROR;                        
    }
    return STATUS_OK;
}

uint8_t PCD_TransceiveData( spi_device_handle_t spi,
                                                    uint8_t *sendData,     ///< Pointer to the data to transfer to the FIFO.
                                                    uint8_t sendLen,       ///< Number of uint8_ts to transfer to the FIFO.
                                                    uint8_t *backData,     ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                                    uint8_t *backLen,      ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                                    uint8_t *validBits,    ///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default nullptr.
                                                    uint8_t rxAlign,       ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                    bool checkCRC       ///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
                                 ) {
    uint8_t waitIRq = 0x30,returnval;        // RxIRq and IdleIRq
    returnval = PCD_CommunicateWithPICC(spi,PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
    return returnval;
}

// TO BE COMPLETED .....
/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PCD_CommunicateWithPICC(spi_device_handle_t spi,
                                                        uint8_t command,       ///< The command to execute. One of the PCD_Command enums.
                                                        uint8_t waitIRq,       ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                                        uint8_t *sendData,     ///< Pointer to the data to transfer to the FIFO.
                                                        uint8_t sendLen,       ///< Number of bytes to transfer to the FIFO.
                                                        uint8_t *backData,     ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                                        uint8_t *backLen,      ///< In: Max number of bytess to write to *backData. Out: The number of uint8_ts returned.
                                                        uint8_t *validBits,    ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                                                        uint8_t rxAlign,       ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                        bool checkCRC          ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
                                     ) {

    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits;      // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
    
    PCD_WriteRegister(spi,CommandReg, PCD_Idle);            // Stop any active command.
    PCD_WriteRegister(spi,ComIrqReg, 0x7F);                 // Clear all seven interrupt request bits
    PCD_WriteRegister(spi,FIFOLevelReg, 0x80);              // FlushBuffer = 1, FIFO initialization
    int sendData_l = 0;

    for(sendData_l = 0; sendData_l<sendLen;sendData_l++ )
    	PCD_WriteRegister(spi,FIFODataReg, sendData[sendData_l]);
 //   PCD_WriteRegisterMany(spi,FIFODataReg,sendLen,sendData);  // Write sendData to the FIFO
    PCD_WriteRegister(spi,BitFramingReg, bitFraming);       // Bit adjustments
    PCD_WriteRegister(spi,CommandReg, command);             // Execute the command
    if (command == PCD_Transceive) {
        PCD_SetRegisterBitMask(spi,BitFramingReg, 0x80);    // StartSend=1, transmission of data starts
    }
    
    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86Î¼s.
    // TODO check/modify for other architectures than Arduino Uno 16bit
    int i;
    uint8_t n,data;
    for (i = 20000; i > 0; i--) {
        n = PCD_ReadRegister(spi,ComIrqReg);   // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq

        if (n & waitIRq) {     //0x30             // One of the interrupts that signal success has been set.

        	break;
        }
        if (n & 0x01) {                     // Timer interrupt - nothing received in 25ms


            return STATUS_TIMEOUT;
        }
    }


    // 35.7ms and nothing happend. Communication with the MFRC522 might be down.
    if (i == 0) {


        return STATUS_TIMEOUT;
    }
    
    // Stop now if any errors except collisions were detected.
    uint8_t errorRegValue = PCD_ReadRegister(spi,ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13) {  // BufferOvfl ParityErr ProtocolErr

    	printf("status error\r\n");
    	return STATUS_ERROR;
    }

    uint8_t _validBits = 0;
    uint8_t h;
    // If the caller wants data back, get it from the MFRC522.


    if (backData && backLen) {

         h = PCD_ReadRegister(spi,FIFOLevelReg);    // Number of bytes in the FIFO
        if (h > *backLen) {
        	printf("no room....\r\n");
            return STATUS_NO_ROOM;
        }
        *backLen = h;                                           // Number of bytes returned

  //     PCD_ReadRegisterMany(spi,FIFODataReg, h, backData, rxAlign);    // Get received data from FIFO


        int k;

        for(k=0 ; k<h;k++)
        {
        	*(backData+k) = PCD_ReadRegister(spi,FIFODataReg);


        }


        _validBits = PCD_ReadRegister(spi,ControlReg) & 0x07;       // RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
        if (validBits) {
            *validBits = _validBits;
        }


        int i;

    }

    // Tell about collisions
    if (errorRegValue & 0x08) {     // CollErr
    	printf("status collision\r\n");
        return STATUS_COLLISION;
    }
    return STATUS_OK;
    // Perform CRC_A validation if requested.
//    if (backData && backLen && checkCRC) {
//        // In this case a MIFARE Classic NAK is not OK.
//        if (*backLen == 1 && _validBits == 4) {
//            return STATUS_MIFARE_NACK;
//        }
//        // We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
//        if (*backLen < 2 || _validBits != 0) {
//            return STATUS_CRC_WRONG;
//        }
//        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
//        uint8_t controlBuffer[2];
//        uint8_t status = PCD_CalculateCRC(spi,&backData[0], *backLen - 2, &controlBuffer[0]);
//        if (status != STATUS_OK) {
//            return status;
//        }
//        if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
//            return STATUS_CRC_WRONG;
//        }
//    }

} // End PCD_CommunicateWithPICC()

/*
 *
 *
 * Function to read the card serial UID, SAK
 */



uint8_t PICC_ReadCardSerial(spi_device_handle_t spi)
{


	 uint8_t result = PICC_Select(spi,&uid,0);

	 return (result == STATUS_OK);

}
/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PICC_Select(spi_device_handle_t spi,	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	uint8_t uidComplete;
	uint8_t selectDone;
	uint8_t useCascadeTag;
	uint8_t cascadeLevel = 1;
	uint8_t result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	uint8_t *responseBuffer;
	uint8_t responseLength;

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
//Serial.println("coming");
	// Prepare MFRC522
	PCD_ClearRegisterBitMask(spi,CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;

			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;

			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.


		if (bytesToCopy) {

			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index] = uid->uidByte[uidIndex + count];



			}



		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(spi,buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(spi,BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			// Transmit the buffer and receive the response.


			result = PCD_TransceiveData(spi,buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign,0);

			if (result == STATUS_COLLISION)
			{ // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = PCD_ReadRegister(spi,CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK)
			{
				return result;
			}
			else { // STATUS_OK


				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID bytes from buffer[] to uid->uidByte[]



		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;


		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(spi,responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
bool PCD_CalculateCRC(	spi_device_handle_t spi,uint8_t *data,uint8_t length,uint8_t *result) {
	PCD_WriteRegister(spi,CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(spi,DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_WriteRegister(spi,FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterMany(spi,FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegister(spi,CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for (uint16_t i = 5000; i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		uint8_t n = PCD_ReadRegister(spi,DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteRegister(spi,CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = PCD_ReadRegister(spi,CRCResultRegL);
			result[1] = PCD_ReadRegister(spi,CRCResultRegH);
			return STATUS_OK;
		}
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()


/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.

*/

void PICC_DumpToSerial(spi_device_handle_t spi,Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
									) {
	// UID
	MIFARE_Key key;

	PICC_DumpDetailsToSerial(uid);
	PICC_Type piccType = PICC_GetType(uid->sak);

	switch (piccType) {
			case PICC_TYPE_MIFARE_MINI:
			case PICC_TYPE_MIFARE_1K:
			case PICC_TYPE_MIFARE_4K:
				// All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
				for (uint8_t i = 0; i < 6; i++) {
					key.keyByte[i] = 0xFF;
				}
				PICC_DumpMifareClassicToSerial(spi,uid, piccType, &key);
				break;

			case PICC_TYPE_MIFARE_UL:
				PICC_DumpMifareUltralightToSerial(spi);
				break;

			case PICC_TYPE_ISO_14443_4:
			case PICC_TYPE_MIFARE_DESFIRE:
			case PICC_TYPE_ISO_18092:
			case PICC_TYPE_MIFARE_PLUS:
			case PICC_TYPE_TNP3XXX:
				printf("Dumping memory contents not implemented for that PICC type.\n");
				break;

			case PICC_TYPE_UNKNOWN:
			case PICC_TYPE_NOT_COMPLETE:
			default:
				break; // No memory dump here
		}

} // End PICC_DumpDetailsToSerial()

/**
 * Dumps memory contents of a MIFARE Classic PICC.
 * On success the PICC is halted after dumping the data.
 */
void PICC_DumpMifareClassicToSerial(spi_device_handle_t spi,	Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
												PICC_Type piccType,	///< One of the PICC_Type enums.
												MIFARE_Key *key		///< Key A used for all sectors.
											) {
	uint8_t no_of_sectors = 0;
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
			// Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
			no_of_sectors = 5;
			break;

		case PICC_TYPE_MIFARE_1K:
			// Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
			no_of_sectors = 16;
			break;

		case PICC_TYPE_MIFARE_4K:
			// Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
			no_of_sectors = 40;
			break;

		default: // Should not happen. Ignore.
			break;
	}

	// Dump sectors, highest address first.
	if (no_of_sectors) {
		printf("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits\r\n");
		for (int8_t i = no_of_sectors - 1; i >= 0; i--) {
			PICC_DumpMifareClassicSectorToSerial(spi,uid, key, i);
		}
	}
	PICC_HaltA(spi); // Halt the PICC before stopping the encrypted session.
	PCD_StopCrypto1(spi);
} // End PICC_DumpMifareClassicToSerial()

/**
 * Dumps memory contents of a sector of a MIFARE Classic PICC.
 * Uses PCD_Authenticate(), MIFARE_Read() and PCD_StopCrypto1.
 * Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
 */
void PICC_DumpMifareClassicSectorToSerial(spi_device_handle_t spi,Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
													MIFARE_Key *key,	///< Key A for the sector.
												 uint8_t sector			///< The sector to dump, 0..39.
													) {
	uint8_t status;
	uint8_t firstBlock;		// Address of lowest address to dump actually last block dumped)
	uint8_t no_of_blocks;		// Number of blocks in sector
	bool isSectorTrailer;	// Set to true while handling the "last" (ie highest address) in the sector.

	// The access bits are stored in a peculiar fashion.
	// There are four groups:
	//		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
	//		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
	//		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
	//		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
	// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
	// The four CX bits are stored together in a nible cx and an inverted nible cx_.
	uint8_t c1, c2, c3;		// Nibbles
	uint8_t c1_, c2_, c3_;		// Inverted nibbles
	bool invertedError;		// True if one of the inverted nibbles did not match
	uint8_t g[4];				// Access bits for each of the four groups.
	uint8_t group;				// 0-3 - active group for access bits
	bool firstInGroup;		// True for the first block dumped in the group

	// Determine position and size of sector.
	if (sector < 32) { // Sectors 0..31 has 4 blocks each
		no_of_blocks = 4;
		firstBlock = sector * no_of_blocks;
	}
	else if (sector < 40) { // Sectors 32-39 has 16 blocks each
		no_of_blocks = 16;
		firstBlock = 128 + (sector - 32) * no_of_blocks;
	}
	else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
		return;
	}

	// Dump blocks, highest address first.
	uint8_t byteCount;
	uint8_t buffer[18];
	uint8_t blockAddr;
	isSectorTrailer = true;
	invertedError = false;	// Avoid "unused variable" warning.
	for (int8_t blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
		blockAddr = firstBlock + blockOffset;
		// Sector number - only on first line
		if (isSectorTrailer) {
			if(sector < 10)
				printf("   "); // Pad with spaces
			else
				printf("  "); // Pad with spaces
				printf("%d",sector);
				printf("   ");
		}
		else {
			printf("       ");
		}
		// Block number
		if(blockAddr < 10)
			printf("   "); // Pad with spaces
		else {
			if(blockAddr < 100)
				printf("  "); // Pad with spaces
			else
				printf(" "); // Pad with spaces
		}
		printf("%d",blockAddr);
		printf("  ");
		// Establish encrypted communications before reading the first block
		if (isSectorTrailer) {
			status = PCD_Authenticate(spi,PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
			if (status != STATUS_OK) {
					printf("PCD_Authenticate() failed: ");
					GetStatusCodeName(status);
				return;
			}
		}
		// Read block
		byteCount = sizeof(buffer);
		status = MIFARE_Read(spi,blockAddr, buffer, &byteCount);
		if (status != STATUS_OK) {
			printf("MIFARE_Read() failed: ");
			GetStatusCodeName(status);
			printf("\n");
			continue;
		}
		// Dump data
		for (uint8_t index = 0; index < 16; index++) {
			if(buffer[index] < 0x10)
				printf(" 0");
			else
				printf(" ");
			printf("%x",buffer[index]);
			if ((index % 4) == 3) {
				printf(" ");
			}
		}
		// Parse sector trailer data
		if (isSectorTrailer) {
			c1  = buffer[7] >> 4;
			c2  = buffer[8] & 0xF;
			c3  = buffer[8] >> 4;
			c1_ = buffer[6] & 0xF;
			c2_ = buffer[6] >> 4;
			c3_ = buffer[7] & 0xF;
			invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
			g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
			g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
			g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
			g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
			isSectorTrailer = false;
		}

		// Which access group is this block in?
		if (no_of_blocks == 4) {
			group = blockOffset;
			firstInGroup = true;
		}
		else {
			group = blockOffset / 5;
			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		}

		if (firstInGroup) {
			// Print access bits
			printf(" [ ");
			printf("%d",(g[group] >> 2) & 1); printf(" ");
			printf("%d",(g[group] >> 1) & 1); printf(" ");
			printf("%d",(g[group] >> 0) & 1);
			printf(" ] ");
			if (invertedError) {
				printf(" Inverted access bits did not match! ");
			}
		}

		if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
			uint32_t value = ((uint32_t)(buffer[3])<<24) | ((uint32_t)(buffer[2])<<16) | ((uint32_t)(buffer[1])<<8) | (uint32_t)(buffer[0]);
				printf(" Value=0x"); printf("%x",value);
				printf(" Adr=0x"); printf("%x",buffer[12]);
		}
		printf("\n");
	}

	return;
} // End PICC_DumpMifareClassicSectorToSerial()

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 *
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
uint8_t PCD_Authenticate(spi_device_handle_t spi,uint8_t command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
		uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
											MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
											Uid *uid			///< Pointer to Uid struct. The first 4 bytes of the UID is used.
											) {
	uint8_t waitIRq = 0x10;		// IdleIRq
uint8_t status;
	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {	// 6 key bytes
		sendData[2+i] = key->keyByte[i];
	}
	// Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
	// section 3.2.5 "MIFARE Classic Authentication".
	// The only missed case is the MF1Sxxxx shortcut activation,
	// but it requires cascade tag (CT) byte, that is not part of uid.
	for (uint8_t i = 0; i < 4; i++) {				// The last 4 bytes of the UID
		sendData[8+i] = uid->uidByte[i+uid->size-4];
	}

	// Start the authentication.
	status =PCD_CommunicateWithPICC(spi,PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData),NULL,0,NULL, 0, false);

return status;

} // End PCD_Authenticate()


void PICC_DumpDetailsToSerial(Uid *uid)
{
	printf("Card UID:");
	for (uint8_t i = 0; i < uid->size; i++) {
		if(uid->uidByte[i] < 0x10)
			printf(" 0");
		else
			printf(" ");
		printf("%x",uid->uidByte[i]);
	}
	printf("\r\n");

	// SAK
	printf("Card SAK: ");
	if(uid->sak < 0x10)
		printf(" 0");
	printf("%x\n",uid->sak);

	// (suggested) PICC type
	PICC_Type piccType = PICC_GetType(uid->sak);
	printf("PICC type: ");
	PICC_GetTypeName(piccType);
	printf("\n");



}
/*
 * Read the block contains
 *
 */
bool MIFARE_Read(spi_device_handle_t spi,	uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
											uint8_t *buffer,		///< The buffer to store the data in
											uint8_t *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										) {
	uint8_t result;

	// Sanity check
	if (buffer == NULL || *bufferSize < 18) {

		printf("status no room .......\r\n");
		return STATUS_NO_ROOM;
	}

	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(spi,buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {

		return result;
	}

	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(spi,buffer, 4, buffer, bufferSize, NULL, 0, true);
} // End MIFARE_Read()





/**
 * Dumps memory contents of a MIFARE Ultralight PICC.
 */
void PICC_DumpMifareUltralightToSerial(spi_device_handle_t spi) {
uint8_t status;
uint8_t byteCount;
uint8_t buffer[18];
uint8_t i;

	printf("Page  0  1  2  3\r\n");
	// Try the mpages of the original Ultralight. Ultralight C has more pages.
	for (uint8_t page = 0; page < 16; page +=4) { // Read returns data for 4 pages at a time.
		// Read pages
		byteCount = sizeof(buffer);
		status = MIFARE_Read(spi,page, buffer, &byteCount);
		if (status != STATUS_OK) {
			printf("MIFARE_Read() failed: ");
//			Serial.println(GetStatusCodeName(status));
			break;
		}
		// Dump data
		for (uint8_t offset = 0; offset < 4; offset++) {
			i = page + offset;
			if(i < 10)
				printf("  "); // Pad with spaces
			else
				printf(" "); // Pad with spaces
			printf("%d",i);
			printf("  ");
			for (uint8_t index = 0; index < 4; index++) {
				i = 4 * offset + index;
				if(buffer[i] < 0x10)
					printf(" 0");
				else
					printf(" ");


				printf("%x",buffer[i]);
			}
			printf("\r\n");
		}
	}
} // End PICC_DumpMifareUltralightToSerial()

/**
 * Serial output for ESP32
 */
void GetStatusCodeName(uint8_t code	///< One of the StatusCode enums.
										) {
	switch (code) {
		case STATUS_OK:				printf("Success.\n"); break;
		case STATUS_ERROR:			printf("Error in communication.\n"); break;
		case STATUS_COLLISION:		printf("Collission detected.\n")  ; break;
		case STATUS_TIMEOUT:		printf("Timeout in communication.\n");break;
		case STATUS_NO_ROOM:		printf("A buffer is not big enough.\n");break;
		case STATUS_INTERNAL_ERROR:	printf("Internal error in the code. Should not happen.\n");break;
		case STATUS_INVALID:		printf("Invalid argument.\n");break;
		case STATUS_CRC_WRONG:		printf("The CRC_A does not match.\n");break;
		case STATUS_MIFARE_NACK:	printf("A MIFARE PICC responded with NAK.\n");break;
		default:					printf("Unknown error\n");
	}
} // End GetStatusCodeName()



