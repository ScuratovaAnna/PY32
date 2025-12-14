## PY32F030K28U6TR 
PY32F030 series microcontrollers are MCUs with high performance 32-bit ARM® Cortex® -M0 + core, wide
voltage operating range. It has embedded up to 64 KBytes flash and 8 KBytes SRAM memory, a maximum
operating frequency of 48 MHz, and contains various products in different package types. The chip integrates multi-channel I2C, SPI, USART and other communication peripherals, one channel 12 bits ADC, five
16 bits timers, and two-channel comparators  

|   *Title* | *Parameter* | 
|:----------|:------------|
|Chip       |PY32F030K28U6TR  | 
|Arch       |Arm Cortex-M0+| 
|Freq       |48 MHz      | 
|Flash      |64 Kbytes   | 
|RAM        |8 Kbytes of SRAM|

![alt-текст](https://github.com/ScuratovaAnna/PY32/blob/main/Images/000.jpg "PY32F030K28U6TR.") 

### Development Environment
---
Visual Studio Code +  Embedded IDE  
Ozone V3.38c  
Использую программатор отладчик [OB-ARM SZFYDOSH](https://aliexpress.ru/item/1005005075938365.html?spm=a2g2w.orderdetail.0.0.145b4aa6LaNEHJ&sku_id=12000034785341692&_ga=2.254358216.1593169754.1751186889-1919995776.1746796316)
___
## Плагин JFlash 
Для версии J-Link выше V7.62 для того, чтобы программное обеспечение J-Link распознавало новые устройства, ранее созданные XML-файлы необходимо поместить в центральную папку JLinkDevices. Для Windows путь до папки будет таким C:\Users<....>\AppData\Roaming\SEGGER\JLinkDevices
Если *JLinkDevices* папка не найдена, вы можете создать ее вручную. 

![alt-текст](https://github.com/ScuratovaAnna/PY32/blob/main/Images/002.jpg "JFlash") 
