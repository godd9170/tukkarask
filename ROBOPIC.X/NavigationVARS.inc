;----------------------------------------------------
;   navigation subroutine variables
;----------------------------------------------------
    cblock
        searchTime:2

        rightTime       ; time to spin right
        leftTime        ; time to spin left

        navFlag1        
    endc

#define MODE                navFlag1,7
#define SIDE_SENSOR_FAR     navFlag1,3
#define SIDE_SENSOR_CLOSE   navFlag1,2
#define FRONT_SENSOR        navFlag1,0