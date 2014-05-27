;----------------------------------------------------
;   navigation subroutines
;----------------------------------------------------
navigate

    movlw   d'130'          ;if event_found is between 140 and 150
    cpfslt  event_found
    bra     nav_step

    movlw   d'120'
    cpfsgt  event_found
    bra     nav_step

    call    IR_displayFreq  ;display freq on LCD

nav_step
    movlw   d'50'
    cpfslt  event_found
    bra     nav_found

    ;if both flags = 0, go straight
    tstfsz  navFlag1
    bra     nav_notStraight

    call    nav_straight
    bra     nav_done

nav_notStraight

    btfsc   FRONT_SENSOR        ;something too close in front?
    bra     nav_newDir

    btfsc   SIDE_SENSOR_CLOSE   ;something too close at side?
    bra     nav_avoidWall

    btfsc   SIDE_SENSOR_FAR     ;too far from wall?
    bra     nav_findWall

nav_done
    
    return

nav_found
    ;if found a beacon, pause
    call    servo_stopA
    call    servo_stopB
    bra     nav_done

;sets servo speeds to go sraight
nav_straight

    bcf     LED_RIGHT
    bsf     LED_CENTER
    bcf     LED_LEFT

    macServo_dirA   FORWARD       ;if not, go straight
    macServo_dirB   FORWARD
    macServo_setA   mASpeed3
    macServo_setB   mBSpeed2

    return

nav_avoidWall ;right, combination of spins and straights

    movlw   d'17'               ;spind for some time
    cpfslt  rightTime
    bra     straightRight       ;if not spinning, go straight

    bcf     LED_RIGHT
    bcf     LED_CENTER
    bsf     LED_LEFT
    ;spin
    macServo_dirA   FORWARD
    macServo_dirB   REVERSE
    macServo_setA   mASpeed3
    call    servo_stopB

    tstfsz  rightTime           ;if timer is 0, reset
    bra     nav_avoidWall_done
    movlf   d'100',rightTime
    bra     nav_avoidWall_done

straightRight
    bcf     LED_RIGHT
    bsf     LED_CENTER
    bsf     LED_LEFT
    ;straight
    macServo_dirA   FORWARD
    macServo_dirB   FORWARD
    macServo_setA   mASpeed3
    macServo_setB   mBSpeed3

nav_avoidWall_done
    return


nav_findWall    ;left, combination of spins and straights

    movlw   d'15'           ;spind for some time
    cpfslt  leftTime
    bra     straightLeft    ;if not spinning, go straight

    bsf     LED_RIGHT
    bcf     LED_CENTER
    bcf     LED_LEFT
    ;spin
    macServo_dirA   REVERSE
    macServo_dirB   FORWARD
    call    servo_stopA
    macServo_setB   mBSpeed3

    tstfsz  leftTime        ;if timer is 0, reset
    bra     nav_findWall_done
    movlf   d'100',leftTime
    bra     nav_findWall_done

straightLeft    ;set servos to go straight
    bsf     LED_RIGHT
    bsf     LED_CENTER
    bcf     LED_LEFT
    ;straight
    macServo_dirA   FORWARD
    macServo_dirB   FORWARD
    macServo_setA   mASpeed3
    macServo_setB   mBSpeed3

nav_findWall_done
    return

nav_newDir  ;spind to avoid wall

    bsf     LED_RIGHT
    bsf     LED_CENTER
    bsf     LED_LEFT

    macServo_dirA   FORWARD
    macServo_dirB   REVERSE
    macServo_setA   mASpeed2
    macServo_setB   mBSpeed2

    bra     nav_done
