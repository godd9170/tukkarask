;----------------------------------------------------
;   ultrasonic sensor subroutine variables
;----------------------------------------------------
    cblock
        pulseLenTimer           ;trigger pulse high timer
        ultra_sensorSel         ;which sensor to poll
        ultraSonicCounter:2     ;number of 20us intervals between send signal and recieve
        ultraCurrentA:2         ;current distance from wall to UltraA
        ultraCurrentB:2         ;current distance from wall to UltraB

        ultraTest               ;flags to ensure only one ultrasonic sample
        ultraTest2
    endc