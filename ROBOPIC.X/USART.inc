;----------------------------------------------------
;   usart subroutines
;----------------------------------------------------
usart_checkRX
;    clrf    event_checkUsart       ;only do if int sets flag

    movlw   0x31                   ;ASCII '1'
    cpfseq  RXtemp
    bra     usart_not1
    bra     usart_storeRX
usart_not1
    movlw   0x32                   ;ASCII '2'
    cpfseq  RXtemp
    bra     usart_not2
    bra     usart_storeRX
usart_not2
    movlw   0x33                   ;ASCII '3'
    cpfseq  RXtemp
    bra     usart_not3
    bra     usart_storeRX
usart_not3
    movlw   0x34                   ;ASCII '4'
    cpfseq  RXtemp
    bra     usart_not4
    bra     usart_storeRX
usart_not4
    movlw   0x35                   ;ASCII '5'
    cpfseq  RXtemp
    bra     usart_not5
    bra     usart_storeRX
usart_not5
    bra     usart_OTHER
usart_doneCheck
    return

usart_storeRX
    movlw   0x30
    subwf   RXtemp,F                ;convert to number
    movff   RXtemp,beaconCount
    movff   RXtemp,beaconCountTmp
    movff   RXtemp,RPG_count+1
    bra     usart_doneCheck

usart_OTHER
    movlw   0x47                   ;ASCII 'G'.. GO!
    cpfseq  RXtemp
    bra     usart_notG

    clrf    momBtnPressed

    return
usart_notG
    movlw   0x50                   ;ASCII 'P'.. Pause
    cpfseq  RXtemp
    bra     usart_notP

    btg     paused,0

usart_notP
    movlw   0x4D                   ;ASCII 'M'.. Manual
    cpfseq  RXtemp
    bra     usart_notM

    btg     manual,0

    macServo_dirA   FORWARD
    macServo_dirB   FORWARD
    macServo_setA   mASpeed3
    macServo_setB   mBSpeed2

usart_notM
    movlw   0x57                   ;ASCII 'W' -> go straight
    cpfseq  RXtemp
    bra     usart_notW

    bcf     LED_RIGHT
    bsf     LED_CENTER
    bcf     LED_LEFT

    macServo_dirA   FORWARD
    macServo_dirB   FORWARD
    macServo_setA   mASpeed3
    macServo_setB   mBSpeed2

usart_notW
    movlw   0x41                   ;ASCII 'A' -> go left
    cpfseq  RXtemp
    bra     usart_notA

    bsf     LED_RIGHT
    bcf     LED_CENTER
    bcf     LED_LEFT

    macServo_dirA   REVERSE
    macServo_dirB   FORWARD
    macServo_setA   mBSpeed2
    macServo_setB   mBSpeed3

usart_notA
    movlw   0x5A                   ;ASCII 'Z' -> go backwards
    cpfseq  RXtemp
    bra     usart_notZ

    bsf     LED_RIGHT
    bcf     LED_CENTER
    bcf     LED_LEFT

    macServo_dirA   REVERSE
    macServo_dirB   REVERSE
    macServo_setA   mBSpeed2
    macServo_setB   mBSpeed3

usart_notZ
    movlw   0x53                   ;ASCII 'S' -> go right
    cpfseq  RXtemp
    return

    bcf     LED_RIGHT
    bcf     LED_CENTER
    bsf     LED_LEFT

    macServo_dirA   FORWARD
    macServo_dirB   REVERSE
    macServo_setA   mASpeed3
    macServo_setB   mBSpeed2

    return


usart_sendFreq

;    decf    event_sendTX

    movf    BCDunpacked+2, W
    addlw   0x30
    movwf   TXREG

    delayXm 2

    movf    BCDunpacked+3, W
    addlw   0x30
    movwf   TXREG

    delayXm 2

    movf    BCDunpacked+4, W
    addlw   0x30
    movwf   TXREG

    delayXm 2

    movf    BCDunpacked+5, W
    addlw   0x30
    movwf   TXREG

    delayXm 2

    movlf   0x0A, W
    movwf   TXREG

    delayXm 2

    movlf   0x0D, W
    movwf   TXREG

    delayXm 4
    return