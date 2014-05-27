;----------------------------------------------------
;   LCD subroutine
;----------------------------------------------------
delayus ;microsecond delay (20us x delay_count_us)
    tstfsz  delay_count_us
    bra     delayus
    return

delayms ;millisecond delay (20ms x delay_count_ms)
    movlf   d'50',delay_count_us    ;50 x 20us
delayms_loop
    tstfsz  delay_count_us
    bra     delayms_loop
    decfsz  delay_multi,F
    bra     delayms
    return