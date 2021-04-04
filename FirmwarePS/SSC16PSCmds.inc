;=========================================================================================
;
;    Filename:      SerialServoCmds.inc
;    Date:          4/1/2021
;    File Version:  1.0.0
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;=========================================================================================
; Notes:
;  Command interpreter for Simple Servo Controller 16 Packet Serial
;
; History
; 1.0.0  4/1/2021	Copied from SerialServo
;=========================================================================================
;
kCmd_SetMode	EQU	0x81	;+1 data (SysMode), return ACK
kCmd_GetMode	EQU	0x01
kCmd_SetCmdPos	EQU	0x82	;+3 data (ServoNum,CMDSigTime), return ACK
kCmd_GetCmdPos	EQU	0x02
kCmd_SetAccel	EQU	0x83	;+2 data (ServoNum,ServoAccelValue), return ACK
kCmd_GetAccel	EQU	0x03
kCmd_SetServoEnable	EQU	0x84	;+2 data (ServoNum, Enable), return ACK
kCmd_GetServoEnable	EQU	0x04
kCmd_SetMaxSpeed	EQU	0x85	;+2 data (ServoNum,ServoMaxSpeed), return ACK
kCmd_GetMaxSpeed	EQU	0x05
kCmd_SetMin_uS	EQU	0x86	;+3 data (ServoNum,MinTime), return ACK
kCmd_GetMin_uS	EQU	0x06
kCmd_SetMax_uS	EQU	0x87	;+3 data (ServoNum,MaxTime), return ACK
kCmd_GetMax_uS	EQU	0x07
;
;
kCmd_GetCurSpeed	EQU	0x91	;+1 Data (ServoNum) return ServoCurSpeed
kCmd_GetCurPos	EQU	0x92	;+1 Data (ServoNum) Return SigOutTime
kCmd_GetInPosition	EQU	0x93	;+1 Data (ServoNum) Return InPositionFlag
;
kCmd_SaveParams	EQU	0x94	;Save all eeprom params, return ACK
kCmd_RestoreParams	EQU	0x95	;Copy to ram, return ACK
;
kCmd_GetStatus	EQU	0x98	;return ssStatus
kCmd_ReBoot	EQU	0x99	;ReBoot the controller
kCmd_RunBootloader	EQU	0x9A	;Set EEPROM (0xFF) = 0xFF and reboot
;
; these commands save params and return a 0,0,0,0 packet with the new address
kCmd_SetMasterAddr	EQU	0xA1	;+1 data, return a zero packet
kCmd_SetSlaveAddr	EQU	0xA2	;+1 data, return a zero packet
;
;Special packet (0xDD, 0xDD, 0x01, 0x02, 0xAA, 0x00, 0x00, 0x00, checksum)
kCmd_SendIDString	EQU	0xAA	; returns "DMFE SSC16PS\n"
;
;=========================================================================================
;
RS232_Send_S0	mLongCall	RS232_Send
	return
;
;=========================================================================================
; Entry: RXDataIsNew=1, data in RX_Data
;
HandleRXData	movlb	RX_Flags	;bank 1
	bcf	RXDataIsNew
	btfsc	RXDataValidFlag	;from master to me?
	bra	HandleRXData_1
HandleRXData_Rtn	movlb	0	;bank 0
	return		; no, ignore this packet
;
HandleRXData_1	movlb	0	;bank 0
                       mMOVLF	RX_Data,FSR1
	mMOVLF	TX_Data,FSR0
;Dispatcher
;
	moviw	FSR1++	;get Command
	movwf	Param78	;aka RX_Data(0)
	andlw                  0xF0
	SKPZ
	bra                    Dispatcher_1
	movf                   Param78,W
	andlw                  0x0F
;Jump table for 0x00 to 0x0F
                       brw
                       return                                        ;0x00 not used
                       bra                    Cmd_GetMode
                       bra                    Cmd_GetCmdPos
                       goto                   Cmd_GetAccel
                       goto                   Cmd_GetServoEnable
                       goto                   Cmd_GetMaxSpeed
                       goto                   Cmd_GetMin_uS
                       goto                   Cmd_GetMax_uS
                       return
                       return		       ;9
                       return
                       return
                       return
                       return
                       return
                       return
;
Dispatcher_1           sublw                  0x80
                       SKPZ
                       bra                    Dispatcher_2
                       movf                   Param78,W
	andlw                  0x0F
;Jump table for 0x80 to 0x8F
                       brw
                       return                                        ;0x80 not used
                       bra                    Cmd_SetMode
                       bra                    Cmd_SetCmdPos
                       goto                   Cmd_SetAccel
                       goto                   Cmd_SetServoEnable
                       goto                   Cmd_SetMaxSpeed
                       goto                   Cmd_SetMin_uS
                       goto                   Cmd_SetMax_uS
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       return
;
;Jump table for 0x20 to 0x27
Dispatcher_2           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0x20
                       SKPZ
                       bra                    Dispatcher_3
                       return                                        ; Yes, oops
;
Dispatcher_3           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0xB0
                       SKPZ
                       bra                    Dispatcher_4
                       return                                        ; Yes, oops
;
;Jump table for 0x90 to 0x9F
Dispatcher_4           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0x90
                       SKPZ
                       bra                    Dispatcher_5
                       movlw                  0x0F
                       andwf                  Param78,W
                       brw
                       return                                        ;0x90 not used
                       goto                   Cmd_GetCurSpeed
                       goto                   Cmd_GetCurPos
                       goto	Cmd_GetInPosition
                       goto                   Cmd_SaveParams
                       goto                   Cmd_RestoreParams
                       return                 
                       return
                       goto                   Cmd_GetStatus
                       reset                                         ;Cmd_ReBoot
                       goto                   Cmd_RunBootloader
                       return
                       return
                       return
                       return
                       return
;
;Jump table for 0xA0 to 0xAF
Dispatcher_5           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0xA0
                       SKPZ
                       bra                    Dispatcher_6
                       movlw                  0x0F
                       andwf                  Param78,W
                       brw
Dispatcher_6           return                                        ;0xA0 not used
                       goto                   Cmd_SetMasterAddr
                       goto                   Cmd_SetSlaveAddr
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       goto                   Cmd_SendIDString       ;0xAA
                       return
                       return
                       return
                       return
                       return
;
;---kCmd_SetMode----------------
; Set Mode
Cmd_SetMode	movlw	kMaxMode+1
	subwf	INDF1,W
	SKPB		;kMaxMode+1>Data
	bra	HandleRXData_Rtn
;
	moviw	FSR1++
	movwf	SysMode
	goto	TX_ACK
;
;---kCmd_GetMode------------------
Cmd_GetMode	movf	SysMode,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetCmdPos-------------------
; Set Command Position
Cmd_SetCmdPos	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	moviw	FSR1++
	movwf	Param7C
	moviw	FSR1++
	movwf	Param7D
	movf	Param78,W
	mLongCall	StartMotion
	goto	TX_ACK
;
Cmd_SetWord7879	movf	Param79,W
	movwi	FSR1++
	movf	Param7A,W
	movwi	FSR1++
	goto	TX_ACK
;
;---kCmd_GetCmdPos------------------
; Get Command Position
Cmd_GetCmdPos	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	lslf	Param78,F	;x2
	LOADFSR1	CMDSigTime,Param78
Cmd_GetWord	moviw	FSR1++
	movwi	FSR0++
	moviw	FSR1++
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetAccel-------------------
; Set Acceleration
Cmd_SetAccel	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	moviw	FSR1++
	movwf	Param79
	LOADFSR1	ServoAccelValue,Param78
Cmd_SetByte79	movf	Param79,W
	movwi	FSR1++
	goto	TX_ACK
;
;---kCmd_GetAccel------------------
; Get Command Position
Cmd_GetAccel	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	LOADFSR1	ServoAccelValue,Param78
Cmd_GetByte	moviw	FSR1++
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetServoEnable--------------
Cmd_SetServoEnable	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	moviw	FSR1++
	movwf	Param79	;0 = Off, 1= On
	movf	Param78,W
	andlw	0x07	;2 servos per ServoFlags byte
	LOADFSR1W	ServoFlags
	btfss	Param79,0	;boolean value, Enable?
	bra	Cmd_SetServoEnable_1	; No, disable
	BTFSS	Param78,3
	BSF	INDF1,ServoOnBit0_7	;enable servo
	BTFSC	Param78,3
	BSF	INDF1,ServoOnBit8_15	;enable servo
	goto	TX_ACK
;
Cmd_SetServoEnable_1	BTFSS	Param78,3
	BCF	INDF1,ServoOnBit0_7	;disable servo
	BTFSC	Param78,3
	BCF	INDF1,ServoOnBit8_15	;disable servo
	goto	TX_ACK
;
;---kCmd_GetServoEnable--------------
Cmd_GetServoEnable	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	andlw	0x07	;2 servos per ServoFlags byte
	LOADFSR1W	ServoFlags
	BTFSC	Param78,3
	bra	Cmd_GetServoEnable_1
	btfsc	INDF1,ServoOnBit0_7	;Is enabled?
	bsf	INDF0,0	; Yes
	goto	RS232_Send_S0
;
Cmd_GetServoEnable_1	btfsc	INDF1,ServoOnBit8_15	;Is enabled?
	bsf	INDF0,0	; Yes
	goto	RS232_Send_S0
;
;---kCmd_SetMaxSpeed-----------------
Cmd_SetMaxSpeed	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	moviw	FSR1++
	movwf	Param79
	LOADFSR1	ServoMaxSpeed,Param78
	goto	Cmd_SetByte79
;
;---kCmd_GetMaxSpeed-----------------
Cmd_GetMaxSpeed	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	LOADFSR1	ServoMaxSpeed,Param78
	goto	Cmd_GetByte
;
;---kCmd_SetMin_uS-------------------
; Set ServoMin_uS
Cmd_SetMin_uS	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	lslf	Param78,F	;x2
	moviw	FSR1++
	movwf	Param79
	moviw	FSR1++
	movwf	Param7A
	LOADFSR1	MinTime,Param78
	goto	Cmd_SetWord7879
;
;---kCmd_GetMin_uS----------------
; Get ServoMin_uS
Cmd_GetMin_uS	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	lslf	Param78,F	;x2
	LOADFSR1	MinTime,Param78
	goto	Cmd_GetWord
;
;---kCmd_SetMax_uS-------------------
; Set ServoMax_uS
Cmd_SetMax_uS	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	lslf	Param78,F	;x2
	moviw	FSR1++
	movwf	Param79
	moviw	FSR1++
	movwf	Param7A
	LOADFSR1	MaxTime,Param78
	goto	Cmd_SetWord7879
;
;---kCmd_GetMax_uS-----------------
; Get ServoMax_uS
Cmd_GetMax_uS	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	lslf	Param78,F	;x2
	LOADFSR1	MaxTime,Param78
	goto	Cmd_GetWord
;
;---kCmd_GetCurSpeed----------------	+1 Data (ServoNum) return ServoCurSpeed
Cmd_GetCurSpeed	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	LOADFSR1	ServoCurSpeed,Param78
	goto	Cmd_GetByte
;
;---kCmd_GetCurPos------------------	+1 Data (ServoNum) Return SigOutTime
Cmd_GetCurPos	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	lslf	Param78,F	;x2
	LOADFSR1	SigOutTime,Param78
	goto	Cmd_GetWord
;
;---kCmd_GetInPosition---------------
Cmd_GetInPosition	moviw	FSR1++
	andlw	0x0F	;0 .. 15 only
	movwf	Param78	;Servo Number
	andlw	0x07	;2 servos per ServoFlags byte
	LOADFSR1W	ServoFlags
	BTFSC	Param78,3
	bra	Cmd_GetInPosition_1
	btfsc	INDF1,InPositionFlag0_7 ;Is in position?
	bsf	INDF0,0	; Yes
	goto	RS232_Send_S0
;
Cmd_GetInPosition_1	btfsc	INDF1,InPositionFlag8_15 ;Is in position?
	bsf	INDF0,0	; Yes
	goto	RS232_Send_S0
;
;---kCmd_SaveParams-----------------
Cmd_SaveParams	mLongCall	SaveParams
	goto	TX_ACK
;
;---kCmd_RestoreParams--------------
Cmd_RestoreParams	mLongCall	CopyToRam
	goto	TX_ACK
;
;
;---kCmd_GetStatus----------------
Cmd_GetStatus	movf	ssStatus,W
	movwi	FSR0++
	movf	ssStatus+1,W
	movwi	FSR0++
	movf	ssStatus+2,W
	movwi	FSR0++
	movf	ssStatus+3,W
	movwi	FSR0++
;reset flags
	bcf	ssRX_Timeout
;
	clrf	ssStatus+1
;	
	goto	RS232_Send_S0
;
;---kCmd_RunBootloader--------------
Cmd_RunBootloader	movlw	0xFF
	movwf	EEDataTemp
	mLongCall	EEWriteW	;Write EEPROM address in W
	reset
;
;---kCmd_SetMasterAddr------------
Cmd_SetMasterAddr	moviw	FSR1++
	movwf	RS232_MasterAddr
	mLongCall	SaveParams
	goto	RS232_Send_S0
;
;---kCmd_SetSlaveAddr-------------
Cmd_SetSlaveAddr	moviw	FSR1++
	movwf	RS232_SlaveAddr
	mLongCall	SaveParams
	goto	RS232_Send_S0
;
;---kCmd_SendIDString------------
Cmd_SendIDString	clrf	Param79
Cmd_SendIDString_L1	movf	Param79,W
	call	IDString
	iorlw	0x00
	SKPNZ
	return
	call	StoreSerOut_S0
	incf	Param79,F
	bra	Cmd_SendIDString_L1
;
IDString	brw
	retlw	'D'
	retlw	'M'
	retlw	'F'
	retlw	'E'
	retlw	' '
	retlw	'S'
	retlw	'S'
	retlw	'C'
	retlw	'1'
	retlw	'6'
	retlw	'P'
	retlw	'S'
	retlw	' '
	retlw	'v'
	retlw	'1'
	retlw	'.'
	retlw	'0'
	retlw	' '
	retlw	'\n'
	retlw	0x00
;
;--------------------------------
	return
;
TX_ACK	movlw	0xFF
StoreSerOut_S0	mLongCall	StoreSerOut
	return
;
;=========================================================================================
