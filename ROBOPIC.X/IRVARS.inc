;----------------------------------------------------
;   IR beacon detection subroutine variables
;----------------------------------------------------
    cblock
        IR_currentVal:2         ;current CCP value
        IR_prevVal:2            ;previous CCP value
        IR_freq:2               ;frequency of captured signal
    endc