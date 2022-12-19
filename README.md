A BLE Broadcaster Firmware from the Texas Instruments Sample codes that can be uploaded on the BM2 Battery Monitor as it uses a CC2541 BLE Module with a voltage Divider circuit that is connected to CC2541 I/O pin P0.7 . The code basically reads the voltage value from the voltage divider using CC2541 ADC and publishes it through a BLE Advertise. The BM2 Battery Monitor also has a bundled Bosch BMA250E Compatible MEMS Accelerometer Sensor that is probably used to detect engine motor vibrations. A sample I2c code is included using the CC2541 templates to read/write data to the MEMs Sensor. The MEMs chip is connected to the CC2541 pins (P0.4 SCL, P0.5 SDA, P0.1 INTA).

MCU side can now receive the ADC data in BLE Observer Mode

using the non-calibrated (needs adjustment) formula below

voltage = (double)(( (double)(adc_lowbyte+(adc_hibyte << 8)) / 304.16) -7.1282 + 0.06);

based on the advertised data

voltage = (double)(( (double)(tbuf[17]+(tbuf[18] << 8)) / 304.16) -7.1282 + 0.06);

To upload the Firmware (hex fle) you need a CC-Debugger and connect the GND,3V,DD,DC,RST Lines to the Board (see included image). A Texasinstruments FlashProgrammer program is needed to upload the hex file.

https://www.ti.com/product/CC-DEBUGGER/part-details/CC-DEBUGGER

https://www.ti.com/tool/FLASH-PROGRAMMER

Code can be recompiled for a new hex file using IAR Embedded Workbench IDE

Do take note that the default MAC address of the Bluetooth module was overriden with a custom MAC address on BLE Advertisement code.

Sample Output from ESP32 Code with BM2 conneced to 12V power supply with adjustable voltage output:

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-1318 X-1151 Y-0231 Z-1991 T-21 -- Voltage : 9.002289

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-131d X-1151 Y-0201 Z-19c1 T-21 -- Voltage : 9.018728

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-1316 X-1141 Y-0221 Z-19d1 T-21 -- Voltage : 8.995714

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-131c X-1141 Y-0231 Z-19c1 T-21 -- Voltage : 9.015440

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-131c X-1141 Y-0231 Z-19c1 T-21 -- Voltage : 9.015440

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-1318 X-1141 Y-0231 Z-19b1 T-21 -- Voltage : 9.002289

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-131d X-1151 Y-0211 Z-19e1 T-21 -- Voltage : 9.018728

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-1318 X-1131 Y-0231 Z-19c1 T-21 -- Voltage : 9.002289

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-1319 X-1141 Y-0241 Z-19e1 T-21 -- Voltage : 9.005577

BM2 Address: 80 eb cb 01 12 cd   ADC Value: V-1317 X-1131 Y-0241 Z-19c1 T-21 -- Voltage : 8.999001
