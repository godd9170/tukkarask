;----------------------------------------------------
;   LCD subroutine variables
;----------------------------------------------------
    cblock
        tmpLCDsend      ; byte to send to LCD
        LCD_shift_dir   ; LCD shift direction
        LCD_shift_count ;number of bits to shift

    endc