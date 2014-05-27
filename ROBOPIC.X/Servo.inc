;----------------------------------------------------
;   servo motor subroutines
;----------------------------------------------------

;----------------------------------------------------
;   servo A subroutine
;----------------------------------------------------
servoAMain
    tstfsz  mAcounter   ;time to set servo singal low?
    bra     servoADone
    bcf     MOTORA
servoADone
    return

;----------------------------------------------------
;   servo B subroutine
;----------------------------------------------------
servoBMain
    tstfsz  mBcounter   ;time to set servo singal low?
    bra     servoBDone
    nop     ;to compensate for resolution
    nop
    nop
    nop
    nop
    nop
    bcf     MOTORB
servoBDone
    return

;----------------------------------------------------
;   stop motor A
;----------------------------------------------------
servo_stopA
    clrf    mASpeed
    return

;----------------------------------------------------
;   stop motor B
;----------------------------------------------------
servo_stopB
    clrf    mBSpeed
    return

;----------------------------------------------------
;   set motor A speed
;----------------------------------------------------
servo_setA
    movwf   servoTmp
    tstfsz  mADir               ;which direction? 1 = forward
    bra     servo_setAfwd
    ;move servo "backwards"
    movf    servoTmp,W
    sublw   mASpeed0            ;motors are opposite; sub from B speed
    bra     servo_setASpeed

;move servo "forwards"
servo_setAfwd
    movf    servoTmp,W
    addlw   mASpeed0

servo_setASpeed
    movwf   mASpeed
    return

;----------------------------------------------------
;   set motor B speed
;----------------------------------------------------
servo_setB
    movwf   servoTmp
    tstfsz  mBDir               ;which direction? 1 = forward
    bra     servo_setBfwd
    ;move servo "backwards"
    movf    servoTmp,W
    addlw   mBSpeed0            ;motors are opposite; add to A speed 
    bra     servo_setBSpeed

;move servo "forwards"
servo_setBfwd
    movf    servoTmp,W
    sublw   mBSpeed0

servo_setBSpeed
    movwf   mBSpeed
    return

;----------------------------------------------------
;   set motor A direction
;----------------------------------------------------
servo_dirA
    movwf   mADir
    return

;----------------------------------------------------
;   set motor B direction
;----------------------------------------------------
servo_dirB
    movwf   mBDir
    return