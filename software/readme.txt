avrdude -c usbtiny -p t85 -U lfuse:w:0xf1:m -U hfuse:w:0xdf:m -U flash:w:TinyLoad_v1.0.hex
