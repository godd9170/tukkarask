;----------------------------------------------------
;   Servo motor subroutine variables
;----------------------------------------------------
    cblock
        mASpeed         ;motor high time
        mADir           ;motor dir: 1 = forward
        mBSpeed         ;motor high time
        mBDir           ;motor dir: 1 = forward

        mAcounter       ;current times
        mBcounter

        servoTmp
    endc