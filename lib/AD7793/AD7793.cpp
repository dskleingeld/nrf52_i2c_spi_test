/***************************************************************************//**
 *   @file   AD7793.c
 *   @brief  Implementation of AD7793 Driver.
 *   @author Bancisor MIhai
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 500
*******************************************************************************

********************************************************************************
 *   Modified and adapted for Arduino by Ph. Sonnet.  March 9, 2019, Version 1.1
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD7793.hpp"				// AD7793 definitions.

/***************************************************************************//**
 * @brief Initializes the AD7793 and checks if the device is present.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/

uint8_t AD7793::begin(uint8_t _cs_pin, uint8_t _data_rdy_pin, SPIClass &_spi_port)
{
    spi_port = &_spi_port;
    cs_pin = _cs_pin;
    data_rdy_pin = _data_rdy_pin;

	uint8_t status = 0x1;

    SPI_Init();
    if((GetRegisterValue(AD7793_REG_ID, 1, 1) & AD7793_ID_MASK) != AD7793_ID)
	{
		status = 0x0;
	}

	return(status);
}
/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @return  None.
*******************************************************************************/
void AD7793::Reset()
{
	uint8_t dataToSend[5] = {0x01, 0xff, 0xff, 0xff, 0xff};

    //digitalWrite(cs_pin,LOW);
	SPI_Write(dataToSend,4);
	//digitalWrite(cs_pin,HIGH);
}
/***************************************************************************//**
 * @brief Reads the Ready Bit for ADC
 *
 * @return status: 1 - if conversion is not yet completed and data is not yet written to the data register.
 *                 0 - if data is written to the ADC data register.
*******************************************************************************/
uint8_t AD7793::Ready()
{
	uint8_t status = 0x0;

	if((GetRegisterValue(AD7793_REG_STAT, 1, 1) & AD7793_STAT_RDY) == 0x80)
	{
		status = 0x1;
	}

	return(status);
}
/***************************************************************************//**
* @brief Reads the ADC error bit.
 *
 * @return status: 1 - result written to ADC data register all clamped has been all clamped to 1 or 0.
 *                 0 - No error due to overrrange, underrange or other error sources.
*******************************************************************************/
uint8_t AD7793::Error()
{
	uint8_t status = 0x0;

    if((GetRegisterValue(AD7793_REG_STAT, 1, 1) & AD7793_STAT_ERR) == 0x40)
	{
		status = 0x1;
	}

	return(status);
}
/***************************************************************************//**
 * @brief Indicates that channel 3 is being converted by the ADC.
 *
 * @return status: 1 - Channel 3 is being converted by the ADC.
 *                 0 - Channel 3 is not being converted by the ADC..
*******************************************************************************/
uint8_t AD7793::Channel3()
{
	uint8_t status = 0x0;

    if((GetRegisterValue(AD7793_REG_STAT, 1, 1) & AD7793_STAT_CH3) == 0x04)
	{
		status = 0x1;
	}

	return(status);
}
/***************************************************************************//**
 * @brief Indicates that channel 2 is being converted by the ADC.
 *
 * @return status: 1 - Channel 2 is being converted by the ADC.
 *                 0 - Channel 2 is not being converted by the ADC..
*******************************************************************************/
uint8_t AD7793::Channel2()
{
	uint8_t status = 0x0;

    if((GetRegisterValue(AD7793_REG_STAT, 1, 1) & AD7793_STAT_CH2) == 0x02)
	{
		status = 0x1;
	}

	return(status);
}
/***************************************************************************//**
 * @brief Indicates that channel 1 is being converted by the ADC.
 *
 * @return status: 1 - Channel 1 is being converted by the ADC.
 *                 0 - Channel 1 is not being converted by the ADC..
*******************************************************************************/
uint8_t AD7793::Channel1()
{
	uint8_t status = 0x0;

    if((GetRegisterValue(AD7793_REG_STAT, 1, 1) & AD7793_STAT_CH1) == 0x01)
	{
		status = 0x1;
	}

	return(status);
}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
uint32_t AD7793::GetRegisterValue(uint8_t regAddress,
                                      uint8_t size,
                                      uint8_t modifyCS)
{
	uint8_t data[5] = {0, 0, 0, 0, 0};
	uint32_t receivedData = 0;

	data[0] = 0x01 * modifyCS;
	data[1] = AD7793_COMM_READ |  AD7793_COMM_ADDR(regAddress);
	SPI_Read(data,(1 + size));
    
    Serial.print("raw read array ");
    for(int i = 0; i< 5; i++){
        Serial.print(+data[i]);
    }
    Serial.println("");
	
    
    for(uint8_t i = 1; i < size +1 ; i ++) {
        receivedData = (receivedData << 8) + data[i];
    }

    return (receivedData);
}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.
*******************************************************************************/
void AD7793::SetRegisterValue(uint8_t regAddress, uint32_t regValue,
                              uint8_t size, uint8_t modifyCS)
{
	uint8_t data[5] = {0, 0, 0, 0, 0};
	uint8_t* dataPointer = (uint8_t*)&regValue;
    uint8_t bytesNr = size + 1;

    data[0] = 1 * modifyCS;
    data[1] = AD7793_COMM_WRITE | AD7793_COMM_ADDR(regAddress);
    while(bytesNr > 1)
    {
        data[bytesNr] = *dataPointer;
        dataPointer++;
        bytesNr--;
    }
	SPI_Write(data,(1 + size));
}
/***************************************************************************//**
 * @brief  Waits for RDY pin to go low.
 *
 * @return None.
*******************************************************************************/
void AD7793::WaitRdyGoLow()
{
    while( digitalRead(data_rdy_pin) ) {;}
}

/***************************************************************************//**
 * @brief Sets the operating mode of AD7793.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.
*******************************************************************************/
void AD7793::SetMode(uint32_t mode)
{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_MODE,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_MODE_SEL(0xFF);
    command |= AD7793_MODE_SEL(mode);
    SetRegisterValue(
            AD7793_REG_MODE,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief Sets the ADC clock source of AD7793.
 *
 * @param mode - Clock source.
 *
 * @return  None.
*******************************************************************************/
void AD7793::SetClockSource(uint32_t clockSource)
{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_MODE,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_MODE_CLKSRC(0xFF);
    command |= AD7793_MODE_CLKSRC(clockSource);
    SetRegisterValue(
            AD7793_REG_MODE,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief Sets the filter update rate of AD7793.
 *
 * @param mode - Filter update rate (Hz).
 *
 * @return  None.
*******************************************************************************/
void AD7793::SetFilterUpdateRate(uint32_t filterRate)
{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_MODE,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_MODE_RATE(0xFF);
    command |= AD7793_MODE_RATE(filterRate);
    SetRegisterValue(
            AD7793_REG_MODE,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief Sets the direction of the internal current source.

 * @param  Current source direction.
 *
 * @return  None.
*******************************************************************************/

void AD7793::SetExcitDirection(uint32_t direction)
{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_IO,
                                      1,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_IEXCDIR(0xF);
    command |= AD7793_IEXCDIR(direction);
    SetRegisterValue(
            AD7793_REG_IO,
            command,
            1,
            1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief Sets the current of the internal current source

 * @param  Current source value.
 *
 * @return  None.
*******************************************************************************/

void AD7793::SetExcitCurrent(uint32_t current)
{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_IO,
                                      1,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_IEXCEN(0xF);
    command |= AD7793_IEXCEN(current);
    SetRegisterValue(
            AD7793_REG_IO,
            command,
            1,
            1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief Enable bias voltage generator of AD7793.
 *
 * @param  Bias voltage.
 *
 * @return  None.
*******************************************************************************/
void AD7793::SetBiasVoltage(uint32_t voltage)
{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_VBIAS(0xFF);
    command |= AD7793_CONF_VBIAS(voltage);
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 *
 * @param  Enable burnout current of AD7793.
 *
 * @return  None.
*******************************************************************************/
void AD7793::EnableBurnoutCurr()

{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_BO_EN;
    command |= AD7793_CONF_BO_EN;
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief Disable burnout current of AD7793.
 *
 * @param  None.
 *
 * @return  None.
*******************************************************************************/
void AD7793::DisableBurnoutCurr()

{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_BO_EN;
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief Enable unipolar coding of AD7793.
 *
 * @param  None.
 *
 * @return  None.
*******************************************************************************/
void AD7793::EnableUnipolar()

{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_UNIPOLAR;
    command |= AD7793_CONF_UNIPOLAR;
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief Enable bipolar coding of AD7793.
 *
 * @param  None.
 *
 * @return  None.
*******************************************************************************/
void AD7793::DisableBipolar()

{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_UNIPOLAR;
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief Enable bias voltage generator current boost of AD7793.
 *
 * @param  None.
 *
 * @return  None.
*******************************************************************************/
void AD7793::EnableCurrBoost()

{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_BOOST;
    command |= AD7793_CONF_BOOST;
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief Disable bias voltage generator current boost of AD7793.
 *
 * @param  None.
 *
 * @return  None.
*******************************************************************************/
void AD7793::DisableCurrBoost()

{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_BOOST;
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief  Set the gain of the In-amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.
*******************************************************************************/
void AD7793::SetGain(uint32_t gain)
	{
		uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_GAIN(0xFF);
    command |= AD7793_CONF_GAIN(gain);
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief Set the reference voltage source for the ADC.
 *
 * @param type - Type of the reference.
 *               Example: AD7793_REFSEL_EXT	- External Reference Selected
 *                        AD7793_REFSEL_INT	- Internal Reference Selected.
 *
 * @return None.
*******************************************************************************/
void AD7793::SetIntReference(uint8_t type)
{
    uint32_t command = 0;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_REFSEL(AD7793_REFSEL_INT);
    command |= AD7793_CONF_REFSEL(type);
    SetRegisterValue(AD7793_REG_CONF,
							command,
							2,
                            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief Enable buffered mode of AD7793.
 *
 * @param  None.
 *
 * @return  None.
*******************************************************************************/
void AD7793::EnableBufMode()
{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_BUF;
    command |= AD7793_CONF_BUF;
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief Disable buffered mode of AD7793.
 *
 * @param  None.
 *
 * @return  None.
*******************************************************************************/
void AD7793::DisableBufMode()
{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7793_CONF_BUF;
    SetRegisterValue(
            AD7793_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief Selects the channel of AD7793.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.
*******************************************************************************/
void AD7793::SetChannel(uint32_t channel)
{
    uint32_t command;

    command = GetRegisterValue(AD7793_REG_CONF, 2, 1); 
    command &= ~AD7793_CONF_CHAN(0xFF);
    command |= AD7793_CONF_CHAN(channel);
    SetRegisterValue(AD7793_REG_CONF,command,2,1);
}
/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7793::Calibrate(uint8_t mode, uint8_t channel)
{
    //unsigned short oldRegValue = 0x0;  //J'ai remplacé short par long	????
    //unsigned short newRegValue = 0x0;

    uint32_t oldRegValue = 0x0;
    uint32_t newRegValue = 0x0;

    SetChannel(channel);
    oldRegValue = GetRegisterValue(AD7793_REG_MODE, 2, 1);
    oldRegValue &= ~AD7793_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7793_MODE_SEL(mode);
    SetRegisterValue(AD7793_REG_MODE,newRegValue,2,0);
	WaitRdyGoLow();
    digitalWrite(cs_pin,HIGH);
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
uint32_t AD7793::SingleConversion() {

    uint32_t command = AD7793_MODE_SEL(AD7793_MODE_SINGLE);
    SetRegisterValue(AD7793_REG_MODE,command,2,0);
    WaitRdyGoLow();
    uint32_t regData = GetRegisterValue(AD7793_REG_DATA, 3, 0);
    digitalWrite(cs_pin,HIGH);

    return(regData);
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
uint32_t AD7793::ContinuousReadAvg(uint8_t sampleNumber) {
    uint32_t samplesAverage = 0x0;
    uint32_t command        = 0x0;
    uint8_t count          = 0x0;

    command = AD7793_MODE_SEL(AD7793_MODE_CONT);
    SetRegisterValue(AD7793_REG_MODE,
                            command,
                            2,
                            0);// CS is not modified by SPI read/write functions.
    for(count = 0;count < sampleNumber;count ++)
    {
        WaitRdyGoLow();
        samplesAverage += GetRegisterValue(AD7793_REG_DATA, 3, 0);  // CS is not modified by SPI read/write functions.
    }
    digitalWrite(cs_pin,HIGH);
    samplesAverage = samplesAverage / sampleNumber;
    return(samplesAverage);
}

/***************************************************************************//**
 * @brief Returns a single measurement, provided continuous mesurement mode has been set up.
 *
 * @return samplesAverage - Result of a single analog-to-digital conversion.
*******************************************************************************/
uint32_t AD7793::ContinuousSingleRead()
{
    uint32_t regData = 0x0;

    WaitRdyGoLow();
    regData = GetRegisterValue(AD7793_REG_DATA, 3, 1);  // CS is modified by SPI read/write functions.
    digitalWrite(cs_pin,HIGH);
    return(regData);
}


uint8_t AD7793::SPI_Init()
{	
    pinMode(cs_pin, OUTPUT);
	digitalWrite(cs_pin,HIGH);
	pinMode(data_rdy_pin,INPUT);

	spi_port->begin(); // configure the SPI port for your SPI device.
	spi_port->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3)); 
    return 1;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param data - Write data buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
uint8_t AD7793::SPI_Write(uint8_t* data, uint8_t bytesNumber)
{
	int SCNumber = data[0];

	digitalWrite(cs_pin,LOW);
	spi_port->transfer(&data[1],bytesNumber);
	if (SCNumber == 1) {
		digitalWrite(cs_pin,HIGH);
	}
	return bytesNumber;
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param data - As an input parameter, data represents the write buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 *               As an output parameter, data represents the read buffer:
 *               - from the first byte onwards are located the read data bytes.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
uint8_t AD7793::SPI_Read(uint8_t* data,
                       uint8_t bytesNumber) {

    digitalWrite(cs_pin,LOW);
    int SCNumber = data[0];
    Serial.println("starting spi_read");
    for(int i = 1 ;i < bytesNumber + 1 ; i ++){
		data[i-1] = spi_port->transfer(data[i]);
	}
    Serial.println("done with spi_read");
    if (SCNumber == 0x1) {
		digitalWrite(cs_pin,HIGH);
	}
 	return(bytesNumber);
}