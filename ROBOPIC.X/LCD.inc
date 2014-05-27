;----------------------------------------------------
;   LCD subroutines
;----------------------------------------------------

;----------------------------------------------------
;   LCD initialization
;----------------------------------------------------
LCDinit
    delayXm d'40'             ;wait 40ms

    ;steps 1 to 7 ;knock 3 times
    bcf     LCD_RS  ; command mode
    movlf   b'00110000', tmpLCDsend
    call    LCD_send
    delayXm d'5'             ;wait 5ms

    movlf   b'00110000', tmpLCDsend
    call    LCD_send
    delayXu d'5'             ;wait 50us

    movlf   b'00110000', tmpLCDsend
    call    LCD_send
    delayXu d'5'             ;wait 50us

    ; steps 8 & 9
    movlf   b'00100000', tmpLCDsend  ;set interface to 4bits
    call    LCD_send
    delayXu d'5'             ;wait 50us

    ; step 10 & 11
    movlf   b'00101000', tmpLCDsend ;4bit, 2 lines, 5x8 character
    call    LCD_send2
    delayXu d'5'             ;wait 50us

    ;setp 12 & 13
    movlf   b'00001000', tmpLCDsend ;display off
    call    LCD_send2
    delayXu d'5'             ;wait 50us

    ;step 14 & 15
    call    LCD_clear

    ;step 16 & 17
    movlf   b'00000110', tmpLCDsend ; inc cursor direction, display shift off
    call    LCD_send2
    delayXu d'5'             ;wait 50us

    ;step 18 & delay
    movlf   b'00001110', tmpLCDsend ; display on, cursor on, blink on
    call    LCD_send2
    delayXu d'5'             ;wait 50us

    return

;----------------------------------------------------
; convert from BCD to ASCII and write to LCD
;----------------------------------------------------
LCD_printHex
    addlw   0x30            ;convert to ASCII
    movwf   tmpLCDsend
    call    LCD_printChar
    return

;----------------------------------------------------
; write string to LCD, ending w/ null termination
;----------------------------------------------------
LCD_printString
    movf    TABLAT, w           ;get byte from program memory
    bz      nullStr
    movwf   tmpLCDsend
    call    LCD_printChar       ;and print to LCD
    TBLRD*+
    bra     LCD_printString     ;repeat until done
nullStr
    return

;----------------------------------------------------
; write character to LCD
;----------------------------------------------------
LCD_printChar
    bsf     LCD_RS  ;RS
    call    LCD_send2
    bcf     LCD_RS  ;RS
    delayXu d'5'             ;wait 50us
    return

;----------------------------------------------------
; set address on LCD
;----------------------------------------------------
; 0x00|0x01|0x02|...|0x26|0x27; (40 characters wide)
; 0x40|0x41|0x42|...|0x66|0x67;
;----------------------------------------------------
LCD_setAddr
    bcf     LCD_RS          ;RS
    bsf     tmpLCDsend, 7
    call    LCD_send2
    delayXu d'5'             ;wait 50us
    return

;----------------------------------------------------
;shift display
;----------------------------------------------------
LCD_shiftLCD
    bcf     LCD_RS          ;RS
    movlf   b'00011000', tmpLCDsend ;send shift command
    movf    LCD_shift_dir, W
    bz      LCD_shiftRight
    bcf     tmpLCDsend,2
    bra     LCD_shiftSend
LCD_shiftRight
    bsf     tmpLCDsend,2
LCD_shiftSend
    call    LCD_send2
    delayXu d'5'             ;wait 50us
    decf    LCD_shift_count
    bnz     LCD_shiftSend
    return

;----------------------------------------------------
; return home
;----------------------------------------------------
LCD_returnHome
    bcf     LCD_RS ;RS
    bsf     LCD_E ;E
    movlf   b'00000000', LATD
    bcf     LCD_E ;E
    bsf     LCD_E ;E
    movlf   b'00100000', LATD
    bcf     LCD_E ;E
    delayXm d'2'             ;wait 2ms
    return

;----------------------------------------------------
; clear LCD
;----------------------------------------------------
LCD_clear
    bcf     LCD_RS ;RS
    bsf     LCD_E ;E
    movlf   b'00000000', LATD
    bcf     LCD_E ;E
    bsf     LCD_E ;E
    movlf   b'00010000', LATD
    bcf     LCD_E ;E
    delayXm d'2'             ;wait 2ms
    return

;----------------------------------------------------
; send 4 bits
;----------------------------------------------------
LCD_send
    bsf     LCD_E  ;E enable display
    movff   tmpLCDsend, LATD
    bcf     LCD_E  ;E disable display
    return

;----------------------------------------------------
; send 8 bits
;----------------------------------------------------
LCD_send2
    bsf     LCD_E  ;E enable display
    movff   tmpLCDsend, LATD
    bcf     LCD_E  ;E disable display
    swapf   tmpLCDsend, F
    bsf     LCD_E  ;E enable display
    movff   tmpLCDsend, LATD
    bcf     LCD_E  ;E disable display
    swapf   tmpLCDsend, F
    return