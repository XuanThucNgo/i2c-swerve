## SMART HOME EQUIPMENT CONTROL SYSTEM WITH BEAGLEBONE BLACK AND YOCTO
The  Yocto  OS  helps  the  application  to  communicate with the stm32 micro-controller through the i2c communication bus of the BeagleBone Black. Here the BeagleBone Black is working as the master in the i2c communication and the stm32 micro-controllers are working as the slaves. BeagleBone Black can send some data to write on the output port of the micro-controller.
## Configuration I2C in STM32CubeIDE
Setup I2C Clock Speed (Hz) to 400000 and slave address is 0x17
Enable Interrupt to received data

<picture>
  <img alt="i2c_configuảtion" height="40%" width="40%" src="https://i.imgur.com/v4QgoUm.jpeg">
</picture>

## Get data from I2C bus
Using HAL API to get data
```
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer, 2, I2C_FIRST_AND_LAST_FRAME);
	else if (TransferDirection == I2C_DIRECTION_RECEIVE)
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer, 1, I2C_FIRST_AND_LAST_FRAME);
	_TransferDirection = TransferDirection;
}
```
After complete received data call HAL_I2C_SlaveRxCpltCallback to get, handle data and clear receive buffer 
```
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (_TransferDirection == I2C_DIRECTION_TRANSMIT) {
		uint8_t reg = rxBuffer[0];
		uint8_t value = rxBuffer[1];
		SetValueParam(value, reg);
		memset(rxBuffer, 0, sizeof(rxBuffer));
	}
}
```
## Run as debug to verify
<picture>
  <img alt="i2c_configuảtion" height="40%" width="40%" src="https://i.imgur.com/ZFemLs0.jpeg">
</picture>

