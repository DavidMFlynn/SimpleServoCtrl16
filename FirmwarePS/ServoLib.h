;====================================================================================================
;
;    Filename:      ServoLib.h
;    Date:          11/28/2015
;    File Version:  1.0
;    
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;  Part of SimpleServo16
;    Defines constants and variables in Banks 3,4,5 and 6
;
;    History:
;
; 1.0   11/28/2015	First rev'd version.
;
;====================================================================================================
;====================================================================================================
; Constants:
;
DefaultMaxSpeed	EQU	0x30	;1uS/20mS^2
DefaultAccel	EQU	0x01
DefaultSFlags	EQU	b'00100010'	;In Position
;
CCPCON_Clr	EQU	b'00001001'	;Clear output on match
CCPCON_Set	EQU	b'00001000'	;Set output on match
CCPCON_Int	EQU	b'00001010'	;Interupt only on match
;
kCenterPulseWidth	EQU	d'3000'	;1500uS
kMinPulseWidth	EQU	d'1800'	;900uS
kMaxPulseWidth	EQU	d'4200'	;2100uS
kServoDwellTimeA	EQU	d'4500'	;Address Change time 2250uS
kServoAddrTime	EQU	d'500'	;250uS
kServoDwellTime	EQU	d'5000'	;2.5mS/Channel
;
;================================================================================================
; Linear data memory 0x2000 .. 0x29AF access using FSR
	cblock	0x20F0	;beginning of bank 3
;  Bank3 Ram 1A0h-1EFh 80 Bytes
	ServoMaxSpeed:10		;0=no Accel, 1..255 counts/20mS
	ServoAccelValue:10		;1..8 counts/20mS squared
	ServoCurSpeed:10		;0=Stopped, MSb=Direction, 1..127
	CMDSigTime:20		;Commanded position MinTime .. MaxTime
; bank4 Ram 220h-26Fh 80 Bytes
	MinTime:20		;Minimum pulse time (900uS=1800)
	MaxTime:20		;Maximum pulse time (2100uS=4200)
	ServoFlags:8		;4 bits per servo
	ServoFlags2:8		;4 bits per servo
; bank 5 Ram 2A0h-2EFh 80 Bytes
	SigOutTime:20		;Current position
	DwellTime:20
	endc
;
	cblock	0x2E0	;locate after DwellTime
; these are in bank 5 because the CCP1 and CCP2 CONs are here, accessed w/o FSR
	CMDServoIDX
	ServoIDX		;Index 0..7
	ServoCtlFlags	
	CalcdDwell		;scratch var
	CalcdDwellH
; there are .11 bytes left in bank 5
	endc
;
	cblock	0x21E0	;beginning of bank 6
;  Bank6 Ram 320h-26Fh 80 Bytes
	AccelRampLen:20
	SequencerData:80
	endc
;
;
;================================================================================================
;
;
; ServoCtlFlags Flag bits, ToDo at Next ISR:
CyclePulseStart	EQU	0	;Start cycle banks 1 and 2
CyclePulseEnd1	EQU	2	;End pulse, begin dwell
CyclePulseEnd2	EQU	3	; Set when a pulse is started.
AddrChngDwell	EQU	4	;dwell 100uS , change address
;
; ServoFlags Flag bits
ValueSentFlag0_7	EQU	0
InPositionFlag0_7	EQU	1
ServoOnBit0_7	EQU	2
MovingFWD0_7	EQU	3
ValueSentFlag8_15	EQU	4
InPositionFlag8_15	EQU	5
ServoOnBit8_15	EQU	6
MovingFWD8_15	EQU	7
;
; ServoFlags2 Flag bits
AccelComplete0_7	EQU	0
AccelComplete8_15	EQU	4
;
;=========================================================================================
;