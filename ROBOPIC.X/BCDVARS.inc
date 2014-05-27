;----------------------------------------------------
;   BCD conversion subroutine variables
;----------------------------------------------------
    cblock
        ;16 bit binary to 3 byte (6 digit) BCD conversion registers
        BCDconv:5       ;3 bytes BCD:2 bytes binary number
        BCD_count       ;rotate counter

        ;6 byte (digit) unpacked BCD holding registers
        BCDunpacked:6   ;0000xxxx where xxxx is BCD number
    endc