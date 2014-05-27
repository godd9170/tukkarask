;----------------------------------------------------
;   IR subroutine
;----------------------------------------------------
IR_check
    
    bcf     PIE1, CCP1IE        ;disable interrupt

    ;get difference of captured value and put in divisor
    movf    IR_prevVal,W
    subwf   IR_currentVal,W
    movwf   BARGB0

    movf    IR_prevVal+1,W
    subwfb  IR_currentVal+1,W
    movwf   BARGB1

    clrf    BARGB2
    clrf    BARGB3

    ;set dividend to 20 000 000
    movlf   0x01,AARGB0
    movlf   0x31,AARGB1
    movlf   0x2D,AARGB2
    movlf   0x00,AARGB3
    clrf    AARGB4
    clrf    AARGB5
    clrf    AARGB6
    clrf    AARGB7

    ;32 by 16 bit divide
    call    FXD3216U

    ;put result in BCD conversion
    movff   AARGB3,BCDconv+4
    movff   AARGB2,BCDconv+3
    call    Bin2BCD             ;convert to BCD
    call    unpackBCD           ;unpack BCD result

    call    usart_sendFreq      ;send using USART

IR_done
    movlf   d'25',event_checkIR ;check IR in 500ms

    bsf     PIE1, CCP1IE        ;enable interrupt

    return



IR_displayFreq  ;display frequency on LCD

    macLCD_setAddr  0x40            ;set position on second line

    movf    BCDunpacked+2, W
    macLCD_printHex BCDunpacked+2
    movf    BCDunpacked+3, W
    macLCD_printHex BCDunpacked+3
    movf    BCDunpacked+4, W
    macLCD_printHex BCDunpacked+4
    movf    BCDunpacked+5, W
    macLCD_printHex BCDunpacked+5

    macLCD_printChar    b'01001000'  ;H

    macLCD_printChar    b'01111010'  ;z

    macLCD_printChar    0x20         ;space

    return