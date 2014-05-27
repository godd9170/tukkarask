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
	CONFIG  PWRT = OFF, BOR = ON, BORV = 42	;Reset
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

#define     SCOPE_PIN   LATE,2

;----------------------------------------------------
;   constants
;----------------------------------------------------

; main timer adjustment
mainTimerAdj    EQU     d'65536' - d'2500' + d'10'   ;1msec

; motor A speeds
mAFast          EQU     d'10'
mAMed           EQU     d'10'
mASlow          EQU     d'10'
; motor B speeds
mBFast          EQU     d'10'
mBMed           EQU     d'10'
mBSlow          EQU     d'10'

;----------------------------------------------------
;   file register variables
;----------------------------------------------------
    cblock	0x000	;address of variable space

    ;variables for saving context during interrupt
    tmp_W
    tmp_STATUS

    ;events
    event_LCD

    ;main timer variables
    tmp0l       ; low byte
    tmp0h       ; hight byte

    endc

#include <LCDVARS.inc>
#include <BCDVARS.inc>
#include <DelayVARS.inc>
#include <ServoVARS.inc>

;----------------------------------------------------
;   Interrupt vectors
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

; delay x milliseconds
delayX  macro   X
        movlf   X,delay_count
        call    delay
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

    ;set up LEDs
    movlf	b'01100001',TRISA	;set data direction
	movlf	b'10001110',ADCON1	;set digital/analog functions
	clrf	PORTA	;initialize L, C, R LEDs to off

    ;LCD pins
    movlf	b'00001111',TRISD	;set I/O for PORTD
    clrf    PORTD
    movlf	b'00000000',TRISE	;set I/O for PORTE
    clrf    PORTE

    ;set up timer and interrupt
    movlf   b'10001000',T0CON   ;set up timer0
    movlf   b'00001001',T1CON   ;set up timer1
    bcf     INTCON,TMR0IF       ;clear interrupt flag
    bcf     PIR1,TMR1IF         ;clear interrupt flag

    ;set to prioity interrupts
    bsf     RCON,IPEN           ;enable priority interrupts

    ;timer0 interrupt set up
    bsf     INTCON,TMR0IE       ;enable interrupt
    bsf     INTCON2,INTEDG1     ;set int to rising edge
    bsf     INTCON2,TMR0IP      ;set timer0 int high priority bit

    ;timer1 interrupt set up
    bsf     PIE1,TMR1IE         ;enable interrupt
    bsf     IPR1,TMR1IP         ;set timer1 int high priority bit

    ;global enables
    bsf     INTCON, GIEH        ;enable high priority interrupts
    bsf     INTCON, GIEL        ;enable low priority interrupts

    ;intialize LCD
    call    LCDinit

    loadTBL mess_Hello
    call    LCD_printString
    

; time dependent state machine
;       - timer0 causes 40usec interrupt
;       - interrupt decrements 'event' timers
;       - when even timer == 0, execute event
;----------------------------------------------------
    

mainLoop


;    btg     LED_CENTER
;    btg     SCOPE_PIN
    bra     mainLoop

;----------------------------------------------------
;   main/device routines
;----------------------------------------------------



;----------------------------------------------------
;   include statements
;----------------------------------------------------
#include <LCD.inc>
#include <BCD.inc>
#include <Delay.inc>
#include <Servo.inc>


;----------------------------------------------------
;   High Priority Interrupt Service Routine
;----------------------------------------------------
ISR_high
    movff   STATUS,tmp_STATUS
    movwf   tmp_W                   ;save context

    btfsc   INTCON,TMR0IF           ;did timer0 cause interrupt?
    bra     ISR_high_pri1
    btfsc   PIR1,TMR1IF             ;did timer1 cause interrupt?
    bra     ISR_high_pri2

ISR_high_pri1
    bcf     LED_ALIVE               ;turn on alive LED
;    btg     SCOPE_PIN

    ;reset timer with compensation for time past
    movff   TMR0L, tmp0l
    movff   TMR0H, tmp0h
    movlw   low mainTimerAdj
    addwf   tmp0l, F
    movlw   high mainTimerAdj
    addwfc  tmp0h, F
    movff   tmp0h, TMR0H
    movff   tmp0l, TMR0L

    ;decrement event timers
    decf    event_LCD

    ;decrement delay timer
    decf    delay_count

    bsf     LED_ALIVE               ;turn off alive LED

    bcf     INTCON,TMR0IF           ;clear interrupt flag

    bra     ISR_high_return

ISR_high_pri2
    nop
    nop
    nop
    nop
    nop
    bcf     PIR1,TMR1IF
    btg     SCOPE_PIN  
    bra     ISR_high_return

ISR_high_return
    movf    tmp_W,W
    movff   tmp_STATUS, STATUS      ;restore context
    retfie


;----------------------------------------------------
;   Low Priority Interrupt Service Routine
;
;   Priority:
;       1. Servo motors
;       2. Ultrasonic sensors
;       3. USART
;----------------------------------------------------
ISR_low
    movff   STATUS,tmp_STATUS
    movwf   tmp_W                  ;save context

    movf    tmp_W,W
    movff   tmp_STATUS, STATUS     ;restore context
    retfie

progEnd
	goto	$               ;stop

;----------------------------------------------------
;   Strings
;----------------------------------------------------
mess_Hello  da  "Hello", 0x00


	end