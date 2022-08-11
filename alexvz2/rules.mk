OLED_ENABLE = yes
OLED_DRIVER = SSD1306
WPM_ENABLE = yes
VIA_ENABLE = yes
TAP_DANCE_ENABLE = yes      # Multiple clicks

ifeq ($(strip $(OLED_ENABLE)), yes)
	SRC += oled_display.c
endif
