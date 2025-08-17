Türkçe Açıklama
Bu proje, DMA, ADC ve FreeRTOS kullanılarak yapılmış basit bir mini osiloskoptur.
OLED ekran (SH1106) için kullanılan kütüphane tarafımdan geliştirilmiştir. Kütüphane ve bağlantı şeması için benimle iletişime geçebilirsiniz.
⚠️ Önemli Notlar:
Sistem 3.3V ile çalışmaktadır. ADC girişine 3.3V üzerinde sinyal verilmesi geliştirme kartına zarar verebilir.
Ölçümlerde küçük hatalar olabilir.
Hesaplamalar, geliştirme kartının Vref değerine göre yapılmaktadır. Bu nedenle ekranda görünen değerler, gerçek ölçülen değerle tam olarak aynı olmayabilir.
Donanımlar: STM32F407 discovery kart , sh1106 sürücülü ı2c oled ekran , potansiyometre veya herhangi ölçümde kullanılacak sensör.
Yapılabilecek geliştirmeler: Daha ayrıntılı gösterim , frekans hesaplama vb işlevler eklenebilir.

English Description
This project is a simple mini oscilloscope implemented using DMA, ADC, and FreeRTOS.
The OLED driver library for the SH1106 display is developed by me. You can contact me to access the library and the wiring diagram.
⚠️ Important Notes:
The system operates at 3.3V. Applying signals higher than 3.3V directly to the ADC input may damage the development board.
Measurement results may contain small errors.
The voltage calculations are based on the development board’s Vref value, so the displayed values might not exactly match the actual measured voltage.

Hardware: STM32F407 discovery board , sh1106 ı2c oled screen , potentiometer or sensor used to be measuere.
Possible improvements: More detailed display, frequency calculation, etc. functions can be added.
