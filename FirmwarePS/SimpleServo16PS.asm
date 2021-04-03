;====================================================================================================
;
;    Filename:      SimpleServo16PS.asm
;    Date:          3/31/2021
;    File Version:  1.0d1
;    
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;    SimpleServo is a 16 servo controller with speed, accel and position control.
;    SimpleServo16PS is the TTL packet serial version.
;
;    Features:	TTL Packet Serial
;	R/C Servo PWM output 16 channel at 20mS
;	3 Buttons/LEDs for config
;
;    History:
; 1.0d1   3/31/2021	Copied from Simple Servo 16 1.0d3
;
;====================================================================================================
; Options
I2C_ADDRESS	EQU	0x34	; Slave address
;
;====================================================================================================
;====================================================================================================
; What happens next:
;   At power up the system LED will blink.
;
;====================================================================================================
; PIC16F1847 on SimpleServo16 PCB Packet Serial Version
;
;   Pin 1 (RA2/AN2) Address A2 (output)
;   Pin 2 (RA3/AN3) Enable Servos 0..7 (active low output)
;   Pin 3 (RA4/AN4) Enable Servos 8..15 (active low output)
;   Pin 4 (RA5/MCLR*) Vpp
;   Pin 5 (GND) Ground
;   Pin 6 (RB0) SW1/LED1 (Active Low Input/Output) (System LED)
;   Pin 7 (RB1/AN11/SDA1) RX Data
;   Pin 8 (RB2/AN10/RX)   TX Data 
;   Pin 9 (RB3/CCP1) Pulse output for Servos 0..7
;
;   Pin 10 (RB4/AN8/SLC1) SW2/LED2 (Active Low Input/Output)
;   Pin 11 (RB5/AN7)  SW3/LED3 (Active Low Input/Output)
;   Pin 12 (RB6/AN5/CCP2) ICSPCLK
;   Pin 13 (RB7/AN6) ICSPDAT
;   Pin 14 (Vcc) +5 volts
;   Pin 15 (RA6) N.C.
;   Pin 16 (RA7/CCP2) Pulse output for Servos 8..15
;   Pin 17 (RA0) Address A0 (output)
;   Pin 18 (RA1) Address A1 (output)
;
;====================================================================================================
;
;
	list	p=16f1847,r=hex,W=1	; list directive to define processor
	nolist
	include	p16f1847.inc	; processor specific variable definitions
	list
;
	__CONFIG _CONFIG1,_FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF & _IESO_OFF
;
;
; INTOSC oscillator: I/O function on CLKIN pin
; WDT disabled
; PWRT disabled
; MCLR/VPP pin function is digital input
; Program memory code protection is disabled
; Data memory code protection is disabled
; Brown-out Reset enabled
; CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
; Internal/External Switchover mode is disabled
; Fail-Safe Clock Monitor is enabled
;
	__CONFIG _CONFIG2,_WRT_OFF & _PLLEN_OFF & _LVP_OFF
;
; Write protection off
; 4x PLL disabled
; Stack Overflow or Underflow will cause a Reset
; Brown-out Reset Voltage (Vbor), low trip point selected.
; Low-voltage programming enabled
;
; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
;
	constant	oldCode=0
	constant	useRS232=1
	constant	UseEEParams=1
	constant	UseAuxLEDBlinking=0
;
	constant	UseAltSerialPort=0
	constant	RP_LongAddr=0
	constant	RP_AddressBytes=1
	constant	RP_DataBytes=4
	constant	UseRS232SyncBytes=1
kRS232SyncByteValue	EQU	0xDD
	constant	UseRS232Chksum=1
	constant               UsePID=0
;
kRS232_MasterAddr	EQU	0x01	;Master's Address
kRS232_SlaveAddr	EQU	0x02	;This Slave's Address
kSysMode	EQU	.0	;Default Mode
kMaxMode	EQU	.0	;Last mode
kssFlags	EQU	.0
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;
;====================================================================================================
	nolist
	include	F1847_Macros.inc
	list
;
;    Port A bits
PortADDRBits	EQU	b'01100000'
PortAValue	EQU	b'00011000'
ANSELA_Val	EQU	b'00000000'
;
#Define	Servo_A0	LATA,0	;Output
#Define	Servo_A1	LATA,1	;Output
#Define	Servo_A2	LATA,2	;Output
#Define	Enable0_7	LATA,3	;Output RA3
#Define	Enable8_15	LATA,4	;Output RA4
#Define	RA5_In	PORTA,5	;unused, Vpp
#Define	RA6_In	PORTA,6	;unused, n/c
#Define	RA7_Out	PORTA,7	;CCP2 Output
;
Servo_AddrDataMask	EQU	0xF8
;
;
;    Port B bits
PortBDDRBits	EQU	b'11110111'	;LEDs Out Others In
PortBValue	EQU	b'00000100'
ANSELB_Val	EQU	b'00000000'
;
#Define	SW1_In	PORTB,0	;SW1/LED1
#Define	RB1_In	PORTB,1	;RX Data
#Define	RB2_In	PORTB,2	;TX Data
#Define	RB3_Out	PORTB,3	;CCP1 Output
#Define	SW2_In	PORTB,4	;SW2/LED2
#Define	SW3_In	PORTB,5	;SW3/LED3
#Define	RB6_In	PORTB,6	;N.C. ICSPCLK
#Define	RB7_In	PORTB,7	;N.C. ICSPDAT
LED1_Bit	EQU	0	;LED1 (Active Low Output)
LED2_Bit	EQU	4	;LED2 (Active Low Output)
LED3_Bit	EQU	5	;LED3 (Active Low Output)
#Define	LED1_Tris	TRISB,LED1_Bit	;LED1 (Active Low Output)
#Define	LED2_Tris	TRISB,LED2_Bit	;LED2 (Active Low Output)
#Define	LED3_Tris	TRISB,LED3_Bit	;LED3 (Active Low Output)
;
;
;========================================================================================
;========================================================================================
;
;Constants
All_In	EQU	0xFF
All_Out	EQU	0x00
;
;OSCCON_Value	EQU	b'01110010'	;8MHz
OSCCON_Value	EQU	b'11110000'	;32MHz
;T2CON_Value	EQU	b'01001110'	;T2 On, /16 pre, /10 post
T2CON_Value	EQU	b'01001111'	;T2 On, /64 pre, /10 post
PR2_Value	EQU	.125
;
LEDTIME	EQU	d'100'	;1.00 seconds
LEDErrorTime	EQU	d'10'
LEDFastTime	EQU	d'20'
;
T1CON_Val	EQU	b'00000001'	;PreScale=1,Fosc/4,Timer ON
TMR1L_Val	EQU	0x3C	; -2500 = 2.5 mS, 400 steps/sec
TMR1H_Val	EQU	0xF6
;
;TMR1L_Val	EQU	0x1E	; -1250 = 1.25 mS, 800 steps/sec
;TMR1H_Val	EQU	0xFB
;
;TMR1L_Val	EQU	0x8F	; -625 = 0.625 mS, 1600 steps/sec
;TMR1H_Val	EQU	0xFD
;
TXSTA_Value	EQU	b'00100100'	;8 bit, TX enabled, Async, high speed
RCSTA_Value	EQU	b'10010000'	;RX enabled, 8 bit, Continious receive
BAUDCON_Value	EQU	b'00001000'	;BRG16=1
; 32MHz clock low speed (BRGH=1,BRG16=1)
Baud_300	EQU	.26666	;300, 0.00%
Baud_1200	EQU	.6666	;1200, 0.00%
Baud_2400	EQU	.3332	;2400, +0.01%
Baud_9600	EQU	.832	;9604, +0.04%
Baud_19200	EQU	.416	;19.18k, -0.08%
Baud_38400	EQU	.207	;38.46k, +0.16%
Baud_57600	EQU	.138	;57.55k, -0.08%
BaudRate	EQU	Baud_38400
;
;
;
DebounceTime	EQU	d'10'
;
;================================================================================================
;***** VARIABLE DEFINITIONS
; there are 256 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xEF, Bank2 0x120..0x16F
; there are 256 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;================================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
	cblock	0x20 
;
	SysLED_Time		;sys LED time
	SysLED_Blinks		;0=1 flash,1,2,3
	SysLED_BlinkCount
	SysLED_Count		;sys LED Timer tick count
;
	EEAddrTemp		;EEProm address to read or write
	EEDataTemp		;Data to be writen to EEProm
;
	Timer1Lo		;1st 16 bit timer
	Timer1Hi		; one second RX timeiout
	Timer2Lo		;2nd 16 bit timer
	Timer2Hi		;
	Timer3Lo		;3rd 16 bit timer
	Timer3Hi		;GP wait timer
	Timer4Lo		;4th 16 bit timer
	Timer4Hi		; debounce timer
;
	TXByte		;Next byte to send
	RXByte		;Last byte received
	SerFlags
;
	ssStatus:4		;Status and condition flags for the user.
;
	Flags
	SendingIdx
;---------------------
;Below here are saved in eeprom
;
	SysMode
	RS232_MasterAddr
	RS232_SlaveAddr
	ssFlags		;Serial Servo flags
	SysFlags
;
	endc
;
#Define	SW1_Flag	SysFlags,0
#Define	SW2_Flag	SysFlags,1
#Define	SW3_Flag	SysFlags,2
#Define	LED2_Flag	SysFlags,3
#Define	LED3_Flag	SysFlags,4
;
#Define	FirstRAMParam	SysMode
#Define	LastRAMParam	SysFlags
;
#Define	DataReceivedFlag	SerFlags,1
#Define	DataSentFlag	SerFlags,2
;
;----ssStatus bits
#Define	ssRX_Timeout	ssStatus,3	;cleared by host read
;
; all bits of ssStatus+1 are cleared by a host kCmd_GetStatus command.
;
;================================================================================================
;  Bank1 Ram 0A0h-0EFh 80 Bytes
	cblock	0x0A0
	RX_ParseFlags
	RX_Flags
	RX_DataCount
	RX_CSUM
	RX_SrcAdd:RP_AddressBytes
	RX_DstAdd:RP_AddressBytes
	RX_TempData:RP_DataBytes
	RX_Data:RP_DataBytes
	TX_Data:RP_DataBytes
	endc
;
;================================================================================================
;  Bank2 Ram 120h-16Fh 80 Bytes
;
#Define	Ser_Buff_Bank	2
;
	cblock	0x120
	Ser_In_Bytes		;Bytes in Ser_In_Buff
	Ser_Out_Bytes		;Bytes in Ser_Out_Buff
	Ser_In_InPtr
	Ser_In_OutPtr
	Ser_Out_InPtr
	Ser_Out_OutPtr
	Ser_In_Buff:20
	Ser_Out_Buff:20
	endc
;
;
;====================================================================================================
;  Part of SimpleServo16
;    Defines constants and variables in Banks 3,4,5 and 6
;
	include <ServoLib.h>
;
;=======================================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70
	Param70	
	Param71	
	Param72	
	Param73
	Param74
	Param75
	Param76
	Param77
	Param78
	Param79
	Param7A
	Param7B
	Param7C
	Param7D
	Param7E
	Param7F
	endc
;
;
;=========================================================================================
;Conditions
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
;=========================================================================================
;==============================================================================================
; ID Locations
	__idlocs	0x10d1
;
;==============================================================================================
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
;
; default values
	ORG	0xF000
;
	de	kSysMode	;nvSysMode
	de	kRS232_MasterAddr	;nvRS232_MasterAddr
	de	kRS232_SlaveAddr	;nvRS232_SlaveAddr
	de	kssFlags	;nvssFlags
;
; add any new params here
;
	de	0x00	;nvSysFlags
;
	ORG	0xF0FF
	de	0x00	;Skip BootLoader
;
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
;
	cblock	0x0000
;
	nvSysMode
	nvRS232_MasterAddr
	nvRS232_SlaveAddr
	nvssFlags
	nvSysFlags
;
	endc
;
#Define	nvFirstParamByte	nvSysMode
#Define	nvLastParamByte	nvSysFlags
;
;
;==============================================================================================
;============================================================================================
;
BootLoaderStart	EQU	0x1E00
;
	ORG	0x000	; processor reset vector
	movlp	BootLoaderStart
	goto	BootLoaderStart
ProgStartVector	CLRF	PCLATH
  	goto	start	; go to beginning of program
;
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.008192 seconds
;
;
	ORG	0x004	; interrupt vector location
	CLRF	BSR	; bank0
	clrf	PCLATH
;
; Timer 2
	BTFSS	PIR1,TMR2IF
	goto	SystemTick_end
;
	BCF	PIR1,TMR2IF	; reset interupt flag bit
;------------------
; These routines run 100 times per second
;------------------
;Decrement timers until they are zero
; 
	CLRF	FSR0H
	call	DecTimer1	;if timer 1 is not zero decrement
	call	DecTimer2
	call	DecTimer3
	call	DecTimer4
;
;-----------------------------------------------------------------
; blink LEDs
	MOVLW	LOW TRISB
	MOVWF	FSR0L
	MOVLW	HIGH TRISB
	MOVWF	FSR0H
; All LEDs off
	BSF	INDF0,LED1_Bit
	BSF	INDF0,LED2_Bit
	BSF	INDF0,LED3_Bit
;
; Read SW's
	BCF	SW1_Flag
	BCF	SW2_Flag
	BCF	SW3_Flag
	BTFSS	SW1_In
	BSF	SW1_Flag
	BTFSS	SW2_In
	BSF	SW2_Flag
	BTFSS	SW3_In
	BSF	SW3_Flag
; Dec LED time
	DECFSZ	SysLED_Count,F	;Is it time?
	bra	TMR2_Done	; No, not yet
;
	MOVF	SysLED_Time,W
	MOVWF	SysLED_Count
; Flash LEDs
	BCF	INDF0,LED1_Bit
	BTFSC	LED2_Flag
	BCF	INDF0,LED2_Bit
	BTFSC	LED3_Flag
	BCF	INDF0,LED3_Bit
;
;
TMR2_Done:
SystemTick_end:	
;
;=========================================================================================
	MOVLB	0	;Bank0
	BTFSC	PIR1,CCP1IF
	CALL	ISR_ServoCCP1
;
	BTFSC	PIR2,CCP2IF
	CALL	ISR_ServoCCP2
;
;=========================================================================================
;-----------------------------------------------------------------------------------------
;--------------------------------------------------------------------
;
	retfie		; return from interrupt
;
;
;==============================================================================================
;==============================================================================================
;
	include <F1847_Common.inc>
	include <SerBuff1938.inc>
	include <RS232_Parse.inc>
;
;==============================================================================================
;
start	mLongCall	InitializeIO
;
;=========================================================================================
; Setup default servo data
;
	CALL	ServoInit16
	CALL	StartServos
;
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
MainLoop	CLRWDT
;
	call	GetSerInBytes
	SKPZ		;Any data?
	CALL	RS232_Parse	; yes
;
	movlb	1
	btfss	RXDataIsNew
	bra	ML_1
	mLongCall	HandleRXData
ML_1:
;
;
	CALL	IdleServos
;	CALL	TestServoLib	;tc
;
	MOVLB	0x00
	BTFSC	SW1_Flag	;manual reset
	GOTO	start
;
;---------------------
; Handle Serial Communications
	BTFSC	PIR1,TXIF	;TX done?
	CALL	TX_TheByte	; Yes
;
; move any serial data received into the 32 byte input buffer
	BTFSS	DataReceivedFlag
	BRA	ML_Ser_Out
	MOVF	RXByte,W
	BCF	DataReceivedFlag
	CALL	StoreSerIn
;
; If the serial data has been sent and there are bytes in the buffer, send the next byte
;
ML_Ser_Out	BTFSS	DataSentFlag
	BRA	ML_Ser_End
	CALL	GetSerOut
	BTFSS	Param78,0
	BRA	ML_Ser_End
	MOVWF	TXByte
	BCF	DataSentFlag
ML_Ser_End:
;----------------------
	goto	MainLoop
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;
	include <ServoLib.inc>
;
;
	org 0x800
	include <SSC16PSCmds.inc>
	include <ssInit.inc>
;
	org BootLoaderStart
	include <BootLoader1847.inc>
;
;
;
;
	END
;
