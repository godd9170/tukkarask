;----------------------------------------------------
;   16bit binary to BCD subroutine
;
;   binary input:
;       BCDconv+3 (MSB)
;       BCDconv+4 (LSB)
;   binary output:
;       BCDconv (MSdigits)
;       to
;       BCDconv+2 (LSdigits)
;----------------------------------------------------
Bin2BCD
    clrf    BCDconv+2           ;clear BCD output regs
    clrf    BCDconv+1
    clrf    BCDconv
    movlf   d'16',BCD_count     ;repeat for all 16 bits
rotateAll
    rlcf    BCDconv+4           ;rotate low binary byte through carry
    rlcf    BCDconv+3           ;rotate high binary byte through carry

    movf    BCDconv+2,W         ;get low byte of BCD output
    addwfc  BCDconv+2,W         ;add the carry bit
    daw                         ;decimal adjust WREG
    movwf   BCDconv+2           ;move back

    movf    BCDconv+1,W         ;get mid byte of BCD output
    addwfc  BCDconv+1,W         ;add carry bit
    daw                         ;decimal adjust
    movwf   BCDconv+1

    rlcf    BCDconv             ;rotate high byte of BCD output
    decfsz  BCD_count           ;if all 16 bits done, return
    bra     rotateAll           ;else repeat
    return

;----------------------------------------------------
;   6 digit packed BCD to unpacked BCD conversion subroutine
;
;   packed BCD input:
;       BCDconv (MSdigits)
;       to
;       BCDconv+2 (LSdigits)
;   unpacked BCD output:
;       BCDunpacked (MSdigit)
;       to
;       BCDconv+5 (LSdigit)
;----------------------------------------------------
unpackBCD
    ;copy all packed BCD registers
    movff   BCDconv, BCDunpacked
    movff   BCDconv, BCDunpacked+1
    movff   BCDconv+1, BCDunpacked+2
    movff   BCDconv+1, BCDunpacked+3
    movff   BCDconv+2, BCDunpacked+4
    movff   BCDconv+2, BCDunpacked+5

    ;and out all low nibbles and rotate high nibbles into low
    movlw   0xf0
    andwf   BCDunpacked, F
    rrcf    BCDunpacked, F
    rrcf    BCDunpacked, F
    rrcf    BCDunpacked, F
    rrcf    BCDunpacked, F
    andwf   BCDunpacked+2, F
    rrcf    BCDunpacked+2, F
    rrcf    BCDunpacked+2, F
    rrcf    BCDunpacked+2, F
    rrcf    BCDunpacked+2, F
    andwf   BCDunpacked+4, F
    rrcf    BCDunpacked+4, F
    rrcf    BCDunpacked+4, F
    rrcf    BCDunpacked+4, F
    rrcf    BCDunpacked+4, F

    ;and out all high nibbles
    movlw   0x0f
    andwf   BCDunpacked+1, F
    andwf   BCDunpacked+3, F
    andwf   BCDunpacked+5, F
    return