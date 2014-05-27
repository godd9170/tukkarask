;----------------------------------------------------
;   RPG subroutines
;----------------------------------------------------

;----------------------------------------------------
;   gets current RPG signal
;----------------------------------------------------
RPG_get
    movff   RPG_sig,RPG_prev        ;set prev
    movff   PORTD,RPG_sig           ;get current signal from RPG
    movlw   b'00000011'
    andwf   RPG_sig,F               ;clear non-sig bits
    call    RPG_calcCCW
    return

;----------------------------------------------------
;   checks for change in RPG
;----------------------------------------------------
RPG_check
    call    RPG_get
    movf    RPG_prev,W              ;did the RPG change?
    xorwf   RPG_sig,W
    bnz     RPG_changed             ;if not 0, changed
    bra     RPG_doneCheck

RPG_changed
    movf    RPG_sig,W               ;is the current == to the calc ccw?
    xorwf   RPG_ccw,W
    bz      RPG_ccwDir              ;equal
    bra     RPG_cwDir               ;not equal

RPG_ccwDir
    call    RPG_dec
    bra     RPG_doneCheck

RPG_cwDir
    call    RPG_inc

RPG_doneCheck
    return

;----------------------------------------------------
;   calculates next RPG signal if CCW
;----------------------------------------------------
RPG_calcCCW
    movff   RPG_prev, RPG_ccw
    rrcf    RPG_ccw, F              ;rotate through carry
    bnc     RPG_setz                ;if carry == 0
    bcf     RPG_ccw,1
    bra     RPG_doneNext
RPG_setz
    bsf     RPG_ccw,1
RPG_doneNext
    movlw   b'00000011'
    andwf   RPG_ccw,F
    return

;----------------------------------------------------
;   deccrement RPG count
;----------------------------------------------------
RPG_dec
    decf    RPG_count+1
    movlw   0xFF
    cpfseq  RPG_count+1
    bra     RPG_decDone
    decf    RPG_count
RPG_decDone
    return

;----------------------------------------------------
;   increment RPG count
;----------------------------------------------------
RPG_inc
    infsnz  RPG_count+1
    incf    RPG_count
    return

;----------------------------------------------------
;   resets RPG count
;----------------------------------------------------
RPG_resetCount
    clrf    RPG_count+1
    clrf    RPG_count
    return