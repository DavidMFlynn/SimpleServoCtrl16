;====================================================================================================
;
;    Filename:      ServoLib.h
;    Date:          4/7/2021
;    File Version:  1.1
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
; 1.1   4/7/2021	Moved to contiguous ram, added sequencer.
; 1.0   11/28/2015	First rev'd version.
;
;=========================================================================================
; Sequencer Notes:
;
kSeqMemSize	EQU	0x80
;
; Time values are UInt8 x 16, 0x01 = 16/100 seconds, .01 Second timebase,
;   0xFF = 255*16/100 = 40.8 Seconds maximum time.
;
; Position values are UInt8 x 8 + 2048, 0x01 = 1*8+2048 = 2056, 0.5 uS timebase,
;   0x00 = 2048, 0xFF = 255*8+2048 = 4088 = 0.002044 Seconds maximum pulse width.
;
; Command values:
;  The high nibble is the command 0..F, low nibble is servo number or other data.
;
kSeqCmd_End	EQU	0x00
kSeqCmd_Move	EQU	0x10	;+ServoNum, Dest
kSeqCmd_SetSpeed	EQU	0x20	;+ServoNum, Speed
kSeqCmd_SetAccel	EQU	0x30	;+ServoNum, Accel
kSeqCmd_SetMin	EQU	0x40	;+ServoNum, MinL, MinH
kSeqCmd_SetMax	EQU	0x50	;+ServoNum, MaxL, MaxH
kSeqCmd_SetLoopTime	EQU	0x60	;+TimerL high nibble, TimerH
;  load this value into master sequence timer, time counts down to zero, 0.01 second timebase
kSeqCmd_WaitUntil	EQU	0x70	;+TimerL high nibble, TimerH
;  wait here until master sequence timer is less than this value
kSeqCmd_Mov2Min	EQU	0x80	;+ServoNum
kSeqCmd_Mov2Max	EQU	0x90	;+ServoNum
kSeqCmd_Mov2Ctr	EQU	0xA0	;+ServoNum
kSeqCmd_WaitInPos	EQU	0xB0	;+ServoNum
kSeqCmd_Stop	EQU	0xC0	;+Flags?
kSeqCmd_WaitForBtn	EQU	0xD0	;+Btn#, 2 or 3 only
;
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
	SequencerFlags
	SequencerPtr		;Offset into SequencerData
; there are 9 bytes left in bank 5
	endc
;
	cblock	0x21E0	;beginning of bank 6
;  Bank6 Ram 320h-26Fh 80 Bytes
	AccelRampLen:20
	SequencerData:kSeqMemSize
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
; SequencerFlags Flag bits
SeqActive	EQU	0
SeqWaitForTimer	EQU	1
SeqWaitForInPos	EQU	2
SeqOptionStop	EQU	3
SeqWaitForBtn2	EQU	4
SeqWaitForBtn3	EQU	5
;
;=========================================================================================
;