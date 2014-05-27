;----------------------------------------------------
;   Ultrasonic subroutine
;----------------------------------------------------
setTrigger
    bsf     ULTRA_TRIG          ;start trigger pulse
    movlf   ultraPulseTime,pulseLenTimer
    clrf    ultraSonicCounter   ;reset 20us ultra counter
    movlf   d'1',ultraSonicCounter+1
    clrf    ultraTest
    clrf    ultraTest2
    return

checkPulse
    tstfsz  pulseLenTimer       ;if 0, end pulse
    bra     noTrigger
    bcf     ULTRA_TRIG
    setf    event_checkDist
noTrigger
    return

checkDist
    tstfsz  ultra_sensorSel     ;which sensor to check?
    bra     checkBDist          ;if one, check sensor B
    call    checkDistA          ;else check sensor A
    bra     doneDistCheck
checkBDist
    call    checkDistB
doneDistCheck
    clrf    event_checkDist
    return

checkDistA
    tstfsz  ultraCurrentA       ;is it more than 8 bit (255)
    bra     wallFar
    movlw   d'85'               ;31cm
    cpfsgt  ultraCurrentA+1     ;too close to wall?
    bra     wallClose           ;turn away from wall
    movlw   d'120'              ;44.5cm
    cpfslt  ultraCurrentA+1     ;too far from wall?
    bra     wallFar             ;turn towards wall
    bcf     SIDE_SENSOR_FAR
    bcf     SIDE_SENSOR_CLOSE
    bra     cont                ;else go straight
wallFar
    bsf     SIDE_SENSOR_FAR
    bcf     SIDE_SENSOR_CLOSE
    bra     cont
wallClose
    bsf     SIDE_SENSOR_CLOSE
    bcf     SIDE_SENSOR_FAR
cont
    return


checkDistB
    tstfsz  ultraCurrentB       ;high byte zero?
    bra     ultraB_good          ;if yes, >87cm

    movlw   d'105'               ;20cm
    cpfsgt  ultraCurrentB+1     ;is it greater than 20cm?
    bra     ultraB_tooClose

    bra     ultraB_good

ultraB_tooClose
    bsf     FRONT_SENSOR        ;if not, robot is too close
    bra     ultraBcont

ultraB_good
    bcf     FRONT_SENSOR        ;if so, robot is not too close

ultraBcont
    return