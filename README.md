# tef6686_pa1rkt_arduino
Software to control a TEF6686 based FM/AM/SW tuner with a single Arduino:
- Standalone (4x20 LCD and rotary encoder are sufficient for basic use)
- Converter LO support so the display shows the actual frequency when using converters
- Variable bandwidth, excellent for DX use
- Fast tuning option (1 MHz steps)
- Signal metering in dBuV
- RDS support (service and long text)
- Presets (currently 10), preset 0 is the start frequency
- Version 1.xx is FM only, fits in a standard Arduino Nano (v1.33 recommended)
- v1.40 and up: basic integration with external control software, following the XDR-GTK de facto standard 
