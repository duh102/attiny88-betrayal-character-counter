FLAGS = -mmcu=attiny88 -DF_CPU=8000000UL -Os -std=c99 -Werror

card.hex: card.elf
	avr-objcopy -O ihex $< $@
	avr-size -C --mcu=attiny88 $<
	
clean:
	rm -vf card.elf card.hex

card.elf: card.c hsv_rgb.c
	avr-gcc $(FLAGS) $^ -o $@ 

hsv_rgb.c: hsv_rgb.h dim_curve.h
