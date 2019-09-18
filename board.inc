# Board-specific settings

# Arduino MEGA
ifeq ($(BOARD),ARDUINO_AVR_MEGA)
PROGRAMMER:=wiring
AVRDUDE_EXTRA:=-D
endif

