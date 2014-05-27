;----------------------------------------------------
;   ROBOPIC
;
;   Henry Gaddard & Matthew Hengeveld
;   PC316
;   2014
;----------------------------------------------------

    ;errorlevel -205     ;disable Warning[205]

;----------------------------------------------------
;   assembler directives
;----------------------------------------------------
	list P=PIC18F452, F=INHX32, C=160, X=ON
	#include P18F452.INC
	CONFIG  OSC = HS	;HS oscillator
	CONFIG  PWRT = OFF, BOR = OFF, BORV = 42	;Reset
	CONFIG  WDT = OFF	;Watchdog timer disabled
	CONFIG  CCP2MUX = ON	;CCP2 to RC1 (rather than to RB3)
	CONFIG  LVP = OFF	;RB5 enabled for I/O

;----------------------------------------------------
;   pin definitions
;----------------------------------------------------

;LED pins
#define     LED_ALIVE   LATA,4
#define     LED_RIGHT   LATA,1
#define     LED_CENTER  LATA,2
#define     LED_LEFT    LATA,3

;LCD pins
#define     LCD_E       LATE,1
#define     LCD_RS      LATE,0

;output to scope
#define     SCOPE_PIN   LATE,2

;motor pins
#define     MOTORA      LATB,5
#define     MOTORB      LATB,4

;RPG pins
#define     RPG_B       PORTD,1
#define     RPG_A       PORTD,0

;momentary button
#define     MOM_BTN     PORTD,3

;ultrasonic trigger pin
#define     ULTRA_TRIG  LATB,3      ;write
#define     ULTRA_TRIG_R    PORTB,3 ;read

;----------------------------------------------------
;   constants
;----------------------------------------------------

; main timer adjustment
mainTimerAdj    EQU     d'256' - d'40'  ;20us
servoTimerAdj   EQU     d'65536' - d'6250' - d'19'  ;20ms

; motor speeds
mASpeed3        EQU     d'8'
mASpeed2        EQU     d'6'
mASpeed1        EQU     d'3'
mASpeeds        EQU     d'2'
mASpeedz        EQU     d'0'

mBSpeed3        EQU     d'12'
mBSpeed2        EQU     d'9'
mBSpeed1        EQU     d'5'
mBSpeeds        EQU     d'2'
mBSpeedz        EQU     d'0'

mASpeed0        EQU     d'58'   ;motorA: 70
mBSpeed0        EQU     d'60'   ;motorB: 72-73

FORWARD         EQU     d'1'
REVERSE         EQU     d'0'

;ultrasonic constants
ultraPulseTime  EQU     d'13'   ;number of 20us ints to set trigger high

;----------------------------------------------------
;   file register variables
;----------------------------------------------------
    cblock	0x000	;address of variable space

    paused  ;paused - no movement, ir or ultrasonics
    manual  ;movement is controlled by serial commands; ultrasonics have no
            ;effect on movement

    ;variables for saving context during interrupt
    tmp_W
    tmp_STATUS

    ;events
    event_RPGbeacons        ;counts down until beacons should be checked
    event_printBeacons      ;print # of beacons if changed
    event_ultraA            ;check ultrasonic A
    event_ultraB            ;check ultrasonic B
    event_checkDist         ;should we check a distance?
    event_checkIR           ;should we check for a beacon?
    event_checkUsart        ;should we check USART for data?
    event_found             ;have we found a beacon?
    event_sendTX            ;should we send a frequency over USART
    event_displayFreq       ;should we print a frequency on the LCD

    ;main timer variables
    tmp0l       ; low byte

    tmp1l       ; low byte
    tmp1h       ; high byte

    ;beacon variables
    beaconCount
    beaconCountTmp

    ;momentary button flag
    momBtnState             ;1 or 0
    momBtnPressed           ;has the button been pressed?

    endc

#include <C:\MATH18\MATHVARS.inc>
#include <LCDVARS.inc>
#include <BCDVARS.inc>
#include <DelayVARS.inc>
#include <ServoVARS.inc>
#include <RPGVARS.inc>
#include <UltrasonicVARS.inc>
#include <NavigationVARS.inc>
#include <IRVARS.inc>
#include <USARTVARS.inc>

;----------------------------------------------------
;   Macros
;----------------------------------------------------
; move literal to file register
movlf   macro    literal,dest
        movlw    literal
        movwf    dest
        endm

; move address to TBLPTR
loadTBL macro   var
        movlw   high var
        movwf   TBLPTRH
        movlw   low var
        movwf   TBLPTRL
        TBLRD*+
        endm

; delay x microseconds
delayXu macro   X
        movlf   X,delay_count_us
        call    delayus
        endm

; delay x milliseconds
delayXm macro   X
        movlf   X,delay_multi
        call    delayms
        endm

; LCD macros
;-----------

; convert 16bit binary to 6 digit BCD
macLCD_toBCD    macro   LSB, MSB
                movff   LSB,BCDconv+4
                movff   MSB,BCDconv+3
                call    Bin2BCD
                call    unpackBCD
                endm

; print hex character to LCD
macLCD_printHex macro   char
                movf    char,W
                call    LCD_printHex
                endm

; print string to LCD
macLCD_printString  macro   stringPtr
                    loadTBL stringPtr
                    call    LCD_printString
                    endm

; prints ACSII char to LCD
macLCD_printChar    macro   char
                    movlf   char,tmpLCDsend
                    call    LCD_printChar
                    endm

; sets address on LCD
macLCD_setAddr  macro   addr
                movlf   addr,tmpLCDsend
                call    LCD_setAddr
                endm

; servo macros
;-----------

; set motor A speed
macServo_setA   macro   speed
                movlw   speed
                call    servo_setA
                endm

; set motor B speed
macServo_setB   macro   speed
                movlw   speed
                call    servo_setB
                endm

; set motor A Dir
macServo_dirA   macro   dir
                movlw   dir
                call    servo_dirA
                endm

; set motor B speed
macServo_dirB   macro   dir
                movlw   dir
                call    servo_dirB
                endm

;----------------------------------------------------
;   Interrupt vectors
;----------------------------------------------------
    org	0x0000	;reset vector
    goto	main	;start of program

    org	0x0008	;high priority interrupt vector
    goto	ISR_high

    org	0x0018	;low priority interrupt vector
    goto	ISR_low

;----------------------------------------------------
;   main program
;----------------------------------------------------
main

    ;LEDs
    movlf	b'01100001',TRISA	;set data direction
	movlf	b'10001110',ADCON1	;set digital/analog functions
	clrf	PORTA	;initialize L, C, R LEDs to off

    ;I/O settings
    movlf	b'00001111',TRISD	;set I/O for PORTD
    clrf    PORTD
    movlf	b'00000000',TRISE	;set I/O for PORTE
    clrf    PORTE
    movlf   b'00000110',TRISB   ;set I/O for PORTB
    clrf    PORTB
    movlf   b'10100110',TRISC   ;set I/O for PORTC
    clrf    PORTC

    ;set up timers and interrupt
    movlf   b'11001000',T0CON   ;set up timer0
    bcf     INTCON,TMR0IF       ;clear interrupt flag
    movlf   b'00111001',T1CON   ;set up timer1
    bcf     PIR1,TMR1IF         ;clear interrupt flag
    movlf   b'11011001',T3CON   ;set up timer3

    ;set to prioity interrupts
    bsf     RCON,IPEN           ;enable priority interrupts

    ;timer0 interrupt set up
    bsf     INTCON,TMR0IE       ;enable interrupt
    bsf     INTCON2,INTEDG1     ;set int to rising edge
    bsf     INTCON2,TMR0IP      ;set timer0 int high priority bit

    ;timer1 interrupt set up
    bsf     PIE1,TMR1IE         ;enable interrupt
    bsf     IPR1,TMR1IP         ;set timer0 int high priority bit

    ;timer3 interrupt DISABLE
    bcf     PIE2, TMR3IE        ;disable interrupt

    ;CCP1 set up
    movlf   b'00000111',CCP1CON ;CCP1 in capture mode, every rising edge
    bcf     IPR1, CCP1IP        ;set CCP1 low priority bit

    ;set ultrasonicA external interrupt
    bcf     INTCON3, INT1IP     ;set ultrasonic interrupt A as low priority

    ;set ultrasonicB external interrupt
    bcf     INTCON3, INT2IP     ;set ultrasonic interrupt B as low priority

    ;USART setup
    movlf   b'01000000',SPBRG   ;=64 -> 9.92KBAUD +0.16% error
    movlf   b'00100110',TXSTA
    movlf   b'10010000',RCSTA
    bsf     PIE1,RCIE           ;enable receive
    bcf     IPR1,RCIP           ;set to low priority
    bcf     PIR1,RCIF           ;clear interrupt flag
    bcf     PIE1,TXIE           ;disable transmit interrupts

    ;global enables
    bsf     INTCON, GIEH        ;enable high priority interrupts
    bsf     INTCON, GIEL        ;enable low priority interrupts

    ;intialize LCD
    call    LCDinit

    call    beaconSetup

;set number of beacons to find
beaconLoop

    tstfsz  event_printBeacons  ;should we print number of beacons?
    bra     noprint
    call    printBeacons
noprint
    tstfsz  event_RPGbeacons
    bra     checkBtn
    bra     checkBeacons

checkBeacons
    call    RPG_check
    call    beaconEntry         ;how many beacons to find?

checkBtn
    call    checkMomBtn         ;check if button has been pressed

    tstfsz  momBtnPressed
    bra     beaconLoop          ;if not, loop
endBeaconLoop

    call    mainSetup           ;else setup main

mainLoop

    tstfsz  paused              ;should we pause?
    bra     mainLoop
    ;------------------------------------------------
    ;   always do this code
    ;------------------------------------------------
    tstfsz  mBSpeed             ;is speed A zero?
    call    servoBMain          ;speed is not zero
    tstfsz  mASpeed             ;is speed B zero?
    call    servoAMain          ;speed is not zero

    ;   USART
    ;------------------------------------------------
    tstfsz  event_checkUsart    ;should we check for USART input?
    call    usart_checkRX

    tstfsz  manual              ;should we allow manual control?
    bra     mainLoop

    call    navigate            ;decide where to go based on sensors

noSearch

    ;   ultrasonic
    ;------------------------------------------------
    btfsc   ULTRA_TRIG_R        ;is pulse high?
    call    checkPulse          ;if so, check

    tstfsz  event_checkDist     ;or should we check distance?
    call    checkDist           ;if so, check distance

    tstfsz  event_ultraA        ;is event 0?
    bra     checkB              ;if not, dont trigger
    call    setTrigger   ;start pulse
    clrf    ultra_sensorSel
    movlf   d'20',event_ultraA  ;reset event
    bra     noDistCheck

checkB
    tstfsz  event_ultraB        ;is event 0?
    bra     noDistCheck         ;if not, dont trigger
    call    setTrigger          ;start pulse
    setf    ultra_sensorSel
    movlf   d'25',event_ultraB  ;reset event

noDistCheck

    ;   IR sensor
    ;------------------------------------------------
    tstfsz  event_checkIR       ;should we check for beacon?
    bra     noIRCheck
    call    IR_check
noIRCheck

    bra     mainLoop

;----------------------------------------------------
;   setup routines
;----------------------------------------------------

;setup for main loop
mainSetup
    call    LCD_clear
    macLCD_printString  mess_Search     ;print "Search"

    ;enable interrupts that were not required for beacon entry
    bsf     PIE1, CCP1IE        ;enable CCP interrupt
    bcf     PIR1, CCP1IF        ;clear CCP interrupt flag

    bsf     INTCON3, INT1IE     ;enable ultraA external interrupt
    bcf     INTCON3, INT1IF     ;clear external interrupt flag

    bsf     INTCON3, INT2IE     ;enable ultraB external interrupt
    bcf     INTCON3, INT2IF     ;clear external interrupt flag


    ;move ROBOPIC forward
    macServo_dirA   FORWARD
    macServo_setA   mASpeed2
    macServo_dirB   FORWARD
    macServo_setB   mBSpeed2

;    call    servo_stopA
;    call    servo_stopB

    clrf    ultra_sensorSel             ;which sensor to sense
    clrf    event_ultraA                ;check ultrasonic A (side)
    clrf    event_ultraB                ;check ultrasonic B (front)
    setf    event_checkDist             ;should we check distance?
    movlf   d'50',event_checkIR         ;when should we check for beacon
    clrf    IR_currentVal               ;current IR period count
    clrf    IR_currentVal+1
    clrf    IR_prevVal                  ;prev IR period count
    clrf    IR_prevVal+1
    clrf    navFlag1                    ;status of all sensors and modes
    clrf    paused                      ;should we pause?
    clrf    manual                      ;should we allow manual control?
    clrf    leftTime                    ;how long should we turn left?
    clrf    rightTime                   ;how long should we turn right?
    movlf   d'2',event_displayFreq      
    movlf   ultraPulseTime,pulseLenTimer       ;init

    return

;setup for beacon loop
beaconSetup
    macLCD_printString  mess_Beacons    ;print "beacons"

    movlf   0xFF,RPG_prev
    call    RPG_check
    call    RPG_resetCount      ;reset RPG counter to 0
    call    RPG_inc             ;set to 1

    clrf    event_printBeacons  ;print beacons right away
    movlf   d'1',beaconCountTmp ;start at 1
    movlf   d'1',beaconCount    ;start at 1

    setf    momBtnPressed       ;init flag to not pressed
    clrf    momBtnState         ;init btn state

    ;make ROBOPIC stationary
    call    servo_stopA
    call    servo_stopB

    movlf   d'1',event_RPGbeacons
    return

;----------------------------------------------------
;   main/device routines
;----------------------------------------------------


; allows user to enter # of becons to find
beaconEntry
    movlw   0x06                ;greater than 5?
    cpfseq  RPG_count+1
    bra     notHigh
    call    RPG_dec             ;if so, go back to 5

notHigh
    movlw   0x00                ;less than 1?
    cpfseq  RPG_count+1
    bra     inLimits
    call    RPG_inc             ;if so, go back to 1

inLimits
;    macLCD_setAddr  0x40        ;print at beginning of second line
    movff   RPG_count+1,beaconCountTmp
    movff   RPG_count+1,beaconCount

    call    printBeacons

    movlf   d'1',event_RPGbeacons   ;reset: next check in 20ms
    return

;prints * for each beacon
printBeacons
    macLCD_setAddr  0x40        ;print at beginning of second line
printBeaconsLoop
    macLCD_printChar    b'00101010' ;print "*" for each beacon
    decfsz  beaconCountTmp
    bra     printBeaconsLoop

    movlf   d'4',beaconCountTmp
clearBeacons
    macLCD_printChar    b'00100000' ;print " " 5 times to erase prev
    decfsz  beaconCountTmp
    bra     clearBeacons

    movlf   d'4',event_printBeacons   ;reset: next check in 80ms
    return


; check and debounce momentary button
checkMomBtn
    movff   PORTD,momBtnState   ;get pin value
    movlw   b'00001000'         ;and with 'pressed button state (1)'
    andwf   momBtnState
    tstfsz  momBtnState         ;if zero, button is pressed
    bra     notPressed
    bra     pressed
pressed
    delayXm d'5'                ;if pressed, wait 100ms
    movff   PORTD,momBtnState   ;get pin value
    movlw   b'00001000'         ;and with 'pressed button state (1)'
    andwf   momBtnState
    tstfsz  momBtnState         ;if button is still in pressed state
    bra     notPressed
    clrf    momBtnPressed       ;set button state to pressed
notPressed
    return


;----------------------------------------------------
;   include statements
;----------------------------------------------------
#include <C:\MATH18\FXD3216U.inc>
#include <LCD.inc>
#include <BCD.inc>
#include <Delay.inc>
#include <Servo.inc>
#include <RPG.inc>
#include <Ultrasonic.inc>
#include <Navigation.inc>
#include <IR.inc>
#include <USART.inc>

;----------------------------------------------------
;   High Priority Interrupt Service Routine
;----------------------------------------------------
ISR_high
    movff   STATUS,tmp_STATUS
    movwf   tmp_W                   ;save context

    btfsc   INTCON,TMR0IF           ;if timer0 int flag is clear, skip and go to timer1 int
    bra     ISR_high_pri0
    bra     ISR_high_pri1
    bra     ISR_high_done

ISR_high_pri0 ;20us
    bcf     LED_ALIVE               ;turn on alive LED

    ;reset timer with compensation for time past
    movff   TMR0L, tmp0l
    movlw   low mainTimerAdj
    addwf   tmp0l, F
    movff   tmp0l, TMR0L

    ;decrement delay timer
    decf    delay_count_us

    ;decrement servo counters;
    tstfsz  mAcounter
    decf    mAcounter
    tstfsz  mBcounter
    decf    mBcounter

    ;decrement utlrasonic counters
    tstfsz  pulseLenTimer
    decf    pulseLenTimer


    movlw   d'254'                      ;see if counter has reached 254
    cpfslt  ultraSonicCounter+1
    bsf     INTCON3,INT1IF
    infsnz  ultraSonicCounter+1       ;count 20us iterations until return signal
    incf    ultraSonicCounter

    bcf     INTCON,TMR0IF           ;clear interrupt flag
    bra     ISR_high_done

ISR_high_pri1 ;20ms
    movff   TMR1L, tmp1l            ;compensate for time elapsed
    movff   TMR1H, tmp1h
    movlw   low servoTimerAdj
    addwf   tmp1l, F
    movlw   high servoTimerAdj
    addwfc  tmp1h, F
    movff   tmp1h, TMR1H
    movff   tmp1l, TMR1L

    ;decrement RPG event timers
    decf    event_RPGbeacons        ;either 0 or 1(execute)
    tstfsz  event_RPGbeacons        ;0?
    clrf    event_RPGbeacons        ;should be 0, if not, clear
    tstfsz  event_printBeacons
    decf    event_printBeacons

    ;decrement motor counters
    bsf     MOTORA
    movff   mASpeed,mAcounter
    bsf     MOTORB
    movff   mBSpeed,mBcounter
    ;decrement turning variables
    tstfsz  rightTime
    decf    rightTime
    tstfsz  leftTime
    decf    leftTime

    ;decrement ultrasonic event timers
    tstfsz  event_ultraA
    decf    event_ultraA
    tstfsz  event_ultraB
    decf    event_ultraB

    ;decrement IR sensor event timer
    tstfsz  event_checkIR
    decf    event_checkIR

    ;decrement IR sensor event timer
    tstfsz  event_found
    decf    event_found

    bcf     PIR1,TMR1IF

ISR_high_done
    
    bsf     LED_ALIVE               ;turn off alive LED
    movf    tmp_W,W
    movff   tmp_STATUS, STATUS      ;restore context
    retfie


;----------------------------------------------------
;   Low Priority Interrupt Service Routine
;
;   Priority:
;----------------------------------------------------
ISR_low
    movff   STATUS,tmp_STATUS
    movwf   tmp_W                   ;save context

    btfsc   PIR1, CCP1IF            ;did ccp fire?
    bra     ISR_CCP1
    btfsc   INTCON3, INT1IF         ;if ext int1, ultraA
    bra     ISR_ultraA
    btfsc   INTCON3, INT2IF         ;if ext int2, ultraB
    bra     ISR_ultraB
    btfsc   PIR1,RCIF               ;has USART received data?
    bra     ISR_usartRX
    bra     ISR_low_done

ISR_CCP1
    movff   IR_currentVal+1,IR_prevVal+1    ;save previous value from CCP
    movff   IR_currentVal,IR_prevVal
    movff   CCPR1L, IR_currentVal+1         ;get value from CCP
    movff   CCPR1H, IR_currentVal

    tstfsz  event_found             ;has the robot already found a beacon?
    bra     CCPdone                 ;if yes, finish ISR
    bra     setFound                ;else set timer

setFound
    movlf   d'200',event_found      ;stop robot when it finds a beacon
    clrf    event_sendTX            ;set flag to start USART transmit

CCPdone
    bcf     PIR1, CCP1IF            ;reset interrupt flag
    bra     ISR_low_done

ISR_ultraA
    tstfsz  ultraTest               ;time to test?
    bra     ISR_low_doneU
    setf    ultraTest
    ;get time from trigger pulse to response
    movff   ultraSonicCounter,ultraCurrentA
    movff   ultraSonicCounter+1,ultraCurrentA+1
    bra     ISR_low_doneU

ISR_ultraB
    tstfsz  ultraTest2              ;time to test?
    bra     ISR_low_doneU
    setf    ultraTest2
    ;get time from trigger pulse to response
    movff   ultraSonicCounter,ultraCurrentB
    movff   ultraSonicCounter+1,ultraCurrentB+1
    bra     ISR_low_doneU

ISR_usartRX
    movff   RCREG,RXtemp            ;store data

    call    usart_checkRX           ;interpret data

    bcf     PIR1,RCIF
    bra     ISR_low_done

ISR_low_doneU
    bcf     INTCON3, INT1IF
    bcf     INTCON3, INT2IF
ISR_low_done
    movf    tmp_W,W
    movff   tmp_STATUS, STATUS     ;restore context
    retfie

progEnd
	goto	$                       ;stop

;----------------------------------------------------
;   Strings
;----------------------------------------------------
mess_Beacons    da  "Beacons", 0x00
mess_Search     da  "Search", 0x00

	end