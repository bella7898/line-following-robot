SHELL=cmd
OBJS=usart.o project.o lcd.o oled.o i2c.o
PORTN=$(shell type COMPORT.inc)

avr_printf.elf: $(OBJS)
	avr-gcc -mmcu=atmega328p $(OBJS) -o project.elf
	avr-objcopy -j .text -j .data -O ihex project.elf project.hex
	@echo done!
	
i2c.o: i2c.c i2c.h
	avr-gcc -g -Os -Wall -mmcu=atmega328p -c i2c.c

oled.o: oled.c oled.h
	avr-gcc -g -Os -Wall -mmcu=atmega328p -c oled.c

usart.o: usart.c usart.h
	avr-gcc -g -Os -Wall -mmcu=atmega328p -c usart.c

lcd.o: lcd.c lcd.h
	avr-gcc -g -Os -Wall -mmcu=atmega328p -c lcd.c

project.o: project.c usart.h project.h oled.h i2c.h
	avr-gcc -g -Os -Wall -mmcu=atmega328p -c project.c

clean:
	@del *.hex *.elf *.o 2>nul

FlashLoad:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	spi_atmega -p -v -crystal project.hex
	@cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

dummy: avr_printf.hex
	@echo Hello dummy!

explorer:
	cmd /c start explorer .