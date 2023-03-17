; Fennec 6800 ROM monitor/mini-OS
; 2022/2023 Chartreuse 
;
; Usage:
;   All commands begin with a single letter, addr/ess are specified
;   as a full 16-bit (4 digit) hex value, and data as a full 8-bit
;   (2 digit) hex value. 
;   There is no backspace as input is processed immediately.
;   Invalid characters are silently ignored.
;   If a mistake is made, press ESC (Ctrl-[) to cancel the line and
;   return to the prompt.
;
;   Commands:
;    E - (E)xamine RAM, performs a hex dump from between two specified
;        addresses. The dash between addresses is NOT entered by the 
;        user but automatically by the monitor
;        eg.
;           E0000-0010
;        OUTPUT:
;	    0000: 00 20 08 20 00 00 DF 46 2C 0E 6F 36 7E FA 08 7E
;  
;    D - (D)eposit RAM, allows the user to enter hex data in memory 
;        starting at the specified address. After entering the address
;        the starting address will be displayed and the user can then
;        start entering bytes. After every byte (2 digits) entered a 
;        space will be returned showing the byte has been entered into
;        RAM. Every 16 bytes a newline and the next address will be 
;        displayed similar to the examine command. Press return or ESC
;        after you are done with entering bytes.
;	 eg. 
;            D0100
;            0100:12 34(ESC)
;            *
;    G - (G)oto address. Jumps to the specified address and resumes
;        execution at that point. This is performed as a subroutine 
;        jump, so if a program preserves the monitors stack, it can 
;        RTS back into the monitor.
;        eg.
;             G0100
;
;    R - (R)ecv XMODEM. Recieves a program using XMODEM (checksum) 
;        starting at the specified address. After specifying address
;        the user will be prompted to start the xmodem transfer on 
;        the remote computer. At this point start the transfer and when
;        completed you will be returned to the monitor prompt. If the
;        prompt is not visible, try hitting ESC to get it to reprint.
;        eg.
;             R0100
;             SEND FILE NOW
;             [user starts xmodem transfer in their terminal program]
;             *
;
;    S - (S)end XMODEM. Sends a block of memory using XMODEM (checksum)
;        from between the two specified addresses. After specifying the
;        second address, the user will be prompted to start an xmodem
;        recieve on their terminal. At this point, start the transfer
;        and when completed you will be returned to the monitor prompt.
;        If the prompt is not visible, try hitting ESC to get it to
;        reprint.
;        eg.
;             S0100-0200
;             RECV FILE NOW
;             [user starts xmodem recieve in their terminal program]
;             *
;
; Changelog
;-----------------------------------------------------------------------
; 0.3.2
;   Added AUX to device routine options (DEVIN/DEVOUT/DEVINNB)
; 0.3.1
;   Added option rom detection and execution
; 0.3 
;   Converted to changable I/O routine pointers in preparation for 
;   VTI board and other option roms to change the default console and
;   others.
;
; 0.2 Added XMODEM send and recieve
; 0.1 Initial version
;-----------------------------
	CPU 6800

;-----------------------------------------------------------------------
; Address constants
RAMTOP	EQU $1FFF	; Last address to try sizing till
			; $1FFF = Full 8kB system (16x 2114 sram)
MONSTK	EQU $00FF	; Monitor reserves first 256 bytes of RAM
PSTART	EQU $8000	; Start of peripheral block
ROMSPA	EQU $04		; Spacing between option ROMS to test at *256
ENDROM	EQU $E0		; Check all the way till EXP0
MAGIC	EQU $F10F	; Magic word for option ROMS
MAGIC0	EQU $F1
MAGIC1	EQU $0F

UART1	EQU $EFF0	; Onboard UART A (lower port)
UART2	EQU $EFF2	; Onboard UART B (upper port)

;-----------------------------------------------------------------------
; Constants
PROMPT	EQU '*'

; ASCII constants
SOH	EQU $01
EOT	EQU $04
ACK	EQU $06
NAK	EQU $15
ETB	EQU $17
ESC	EQU $1B

;-----------------------------------------------------------------------
; Option ROM setup
ROMSIG	EQU $F10F	; Signature to load ROM
ROMNLEN	EQU 10		; Length of ROM name
ROMOFF	EQU 16		; Code start byte of ROM

;-----------------------------------------------------------------------
; Variables
ADDRT	EQU $0000	; 2 - Temp address 
RAMKB	EQU $0002	; 1 - # of kB of ram
BYTET	EQU $0003	; 1 - Temp byte
ADDRP	EQU $0004	; 2 - printing address
TIMER	EQU $0006	; 2 - timer for delays
SEQ	EQU $0008	; 1 - Current block for XMODEM
CSUM	EQU $0009	; 1 - Current checksum for XMODEM
COUNT	EQU $000A	; 1 - Count of bytes (XMODEM)
LAST	EQU $000B	; 1 - Last acknowledgement (XMODEM)
; Controllable routines
;  - expect no reg preserved, result if any in A
CONTR	EQU $000C	; Start of controllable routines

CONIN	EQU CONTR	; 3 - JMP to GETC routine for console
CONOUT	EQU CONIN+3	; 3 - JMP TO PUTC routine for console
CONINNB	EQU CONOUT+3	; 3 - JMP to GETC (non-blocking) for console

AUXIN	EQU CONINNB+3	; 3 - JMP to GETC routine for aux 
AUXOUT	EQU AUXIN+3	; 3 - JMP TO PUTC routine for aux 
AUXINNB	EQU AUXOUT+3	; 3 - JMP to GETC (non-blocking) for aux

DEVIN	EQU AUXINNB+3	; 3 - JMP to device routine GETC
DEVOUT	EQU DEVIN+3	; 3 - JMP to device routine PUTC
DEVINNB	EQU DEVOUT+3	; 3 - JMP to device routine GETC (non-blocking)
			;  Device routines take device # in B
			;  0 = console, 1 = aux, 2 = uart a, 3 = uart b, 
			;  4+ rom or user defined
CONTRE	EQU DEVINNB+3	; End of controllable routines






;CONOUT EQU S0OUT
;CONIN EQU S0IN
;-----------------------------------------------------------------------
; Program
	ORG $F800	; 2kB ROM
START
	JMP ENTER
; Default controllable routines
DEFRT
	JMP S0IN
	JMP S0OUT
	JMP S0IN_NB
	JMP S1IN
	JMP S1OUT
	JMP S1IN_NB
	JMP DDEVIN
	JMP DDEVOUT
	JMP DDEVINNB
DEFRTE
DEFRTLEN EQU *-DEFRT


ENTER
	LDS #MONSTK	; Stack pointer
	JSR UARTINIT
	JSR CONTINIT	; Initialize controlable routines
	JSR RAMSIZE	; Size ram
	JSR FINDROM	; Find and run any option roms
	JSR BANNER
	JSR PRINTROM	; Find roms and print their names
	
WARMST	LDS #MONSTK	; Reset Stack pointer
LOOP	JSR NEWLINE
	LDAA #PROMPT
	JSR CONOUT
LOOPI	JSR CONIN
	JSR TOUPPER	; Convert to upper
	CMPA #'E'	; Examine
	BEQ EXAMINE
	CMPA #'D'	; Deposit
	BEQ DEPOSIT
	CMPA #'G'	; Goto      
	BEQ GOTO
	CMPA #'R'	; XMODEM Recieve
	BEQ XRECV
	CMPA #'S'	; XMODEM Send
	BEQ XSEND
	CMPA #'\r'
	BEQ LOOP
	BRA LOOPI	; Invalid character
EXAMINE
	JSR ADDR2
	JSR DUMP
	BRA LOOP
DEPOSIT	JSR ADDR1
	JSR DEPLOOP
	BRA LOOP
GOTO	JSR ADDR1
	LDX ADDRT
	JSR 0,X
	BRA LOOP
XRECV	JSR ADDR1
	LDAA #1		; Send message
	JSR XMSG
	JSR XMDMRECV
	BRA LOOP
XSEND	JSR ADDR2
	LDAA #0		; Recv message
	JSR XMSG
	JSR XMDMSEND
	BRA LOOP
; Address prompts for command
ADDR2:
	JSR CONOUT
	JSR READADDR
	LDAA #'-'
	LDX ADDRT
ADDR1:	JSR CONOUT
	JSR READADDR
	JSR NEWLINE
	RTS
; Xmodem messages
XMSG:
	STX ADDRP	; Save address
	LDX #S_RECV
	TSTA 
	BEQ .RECV
	LDX #S_SEND
.RECV	JSR PRINTSTR
	LDX #S_FILE
	JSR PRINTSTR
	LDX ADDRP	; Restore addr
	RTS
;-----------------------------------------------------------------------
; Find and execute any option roms	
FINDROM
	LDX #PSTART	; Start of peripheral block
.LOOP
	LDAA #MAGIC0
	CMPA 0,X
	BNE .NEXT
	LDAA #MAGIC1
	CMPA 1,X
	BEQ .FOUND	; Both magic # match
.NEXT	STX ADDRT
	LDAA ADDRT
	ADDA #ROMSPA	; Move to next addr to check
	STAA ADDRT
	LDX ADDRT
	CMPA #ENDROM	; Have we reached the last block to check?
	BCS .LOOP	; (unsigned LT) Continue if not past last addr
	RTS		; Last block, we're done
.FOUND	; Found a valid signature
	STX ADDRT	; Preserve address
	JSR 16,X	; Jump to 16th byte of ROM to start
	LDX ADDRT	; We're back, restore and keep looking
	BRA .NEXT
	
;-----------------------------------------------------------------------
; Find and print option rom names	
;  We can't do this in the FINDROM routine as the console may be defined
;  by an option ROM (like the VTI)
PRINTROM
	LDX #S_OPTION
	JSR PRINTSTR
	LDX #PSTART	; Start of peripheral block
.LOOP
	LDAA #MAGIC0
	CMPA 0,X
	BNE .NEXT
	LDAA #MAGIC1
	CMPA 1,X
	BEQ .FOUND	; Both magic # match
.NEXT	STX ADDRT
	LDAA ADDRT
	ADDA #ROMSPA	; Move to next addr to check
	STAA ADDRT
	LDX ADDRT
	CMPA #ENDROM	; Have we reached the last block to check?
	BCS .LOOP	; (unsigned LT) Continue if not past last addr
	RTS		; Last block, we're done
.FOUND	; Found a valid signature
	STX ADDRT	; Preserve address
	STX ADDRP	; For printing
	JSR PRINTWORD
	LDAA #'-'
	JSR CONOUT
	INX
	INX		; String starts from 2nd byte of ROM
	JSR PRINTSTR
	LDX ADDRT	; Restore and resume
	BRA .NEXT
	

	
	
;-----------------------------------------------------------------------
DEPLOOP
	LDX ADDRT
.NEXT
	STX ADDRP
	JSR PRINTWORD
	LDAA #':'
	JSR CONOUT
	LDAB #16	; Max 16 per line
.LOOP
	PSHB
.IGNORE
	JSR CONIN
	JSR TOUPPER
	CMPA #ESC
	BNE .T2
	JMP WARMST
.T2	CMPA #'\r'
	BEQ .DONE
	CMPA #'0'
	BLT .IGNORE
	CMPA #'9'
	BLE .DIGIT
	CMPA #'A'
	BLT .IGNORE
	CMPA #'F'
	BGT .IGNORE
.DIGIT	JSR READBYTEP
	LDAA BYTET
	STAA 0,X
	INX
	LDAA #' '
	JSR CONOUT
	PULB
	DECB
	BNE .LOOP
	JSR NEWLINE
	BRA .NEXT
.DONE	PULB
	RTS
	
;-----------------------------------------------------------------------
; Dump hex from addr in X to ADDRT
DUMP
	; Print addr 1st column
	STX ADDRP
	JSR PRINTWORD
	LDAA #':'
	JSR CONOUT
	LDAB #16	; Print 16 bytes per line
.LOOP
	PSHB
	LDAA #' '
	JSR CONOUT
	LDAA 0,X
	JSR PRINTBYTE
	PULB
	INX
	CPX ADDRT
	BEQ .DONE
	DECB	
	BNE .LOOP
	JSR NEWLINE
	JSR CONINNB	; Check if user wants intervention
	CMPA #ESC
	BNE DUMP
.DONE
	RTS
	
;-----------------------------------------------------------------------
; Read byte with first character preset
READBYTEP 
	LDAB #0
	STAB BYTET
	LDAB #2
	PSHB
	BRA READBYTE.NEXT3
	
;-----------------------------------------------------------------------
; Read in 2 byte address
READADDR
	JSR READBYTE
	LDAA BYTET
	STAA ADDRT
	JSR READBYTE
	LDAA BYTET
	STAA ADDRT+1
	RTS
;-----------------------------------------------------------------------
; Read in 1 byte in hex
READBYTE
	LDAA #0
	STAA BYTET
	LDAB #2		; Expect 2 digits
.NEXT	PSHB
.NEXT2	JSR CONIN
	JSR TOUPPER
	CMPA #ESC
	BNE .NEXT3
	JMP WARMST
.NEXT3	CMPA #'0'
	BLT .NEXT2	; Ignore
	CMPA #'9'
	BLE .DIGIT
	CMPA #'A'
	BLT .NEXT2	; Ignore
	CMPA #'F'
	BGT .NEXT2	; Ignore
	JSR CONOUT
	; A-F
	SUBA #'A'-':'	; Convert to 3A-3F
	BRA .DIGIT2
.DIGIT
	JSR CONOUT
.DIGIT2
	ASL BYTET
	ASL BYTET
	ASL BYTET
	ASL BYTET
	SUBA #'0'
	ADDA BYTET
	STAA BYTET
	PULB
	DECB
	BNE .NEXT	; Go till we get all digits
	RTS		; Address in BYTET
	
;-----------------------------------------------------------------------
PRINTWORD
	LDAA ADDRP
	JSR PRINTBYTE
	LDAA ADDRP+1
	; Fall into printbyte
;-----------------------------------------------------------------------
PRINTBYTE
	PSHA
	ASR A
	ASR A
	ASR A
	ASR A
	JSR PRINTNYB
	PULA
	; Fall into printnyb
;-----------------------------------------------------------------------
PRINTNYB
	ANDA #$0F
	ADDA #'0'
	CMPA #'9'
	BLE .NOFIX
	ADDA #'A'-':'
.NOFIX	JMP CONOUT
	; Tail call
	
;-----------------------------------------------------------------------
; Initialize both UARTs
UARTINIT
	LDAA #$03	; Master reset
	STAA UART1
	STAA UART2
	LDAA #$15	; /16 8N1, RTS asserted no interrupts
	STAA UART1
	STAA UART2
	RTS

CONTINIT
	LDX #DEFRTE-1	; End of default set in X
	STS ADDRT	; Store stack pointer
	LDS #CONTRE-1	; End of controllable
	
	LDAB #DEFRTLEN	; Number of bytes to copy
.COPY	LDAA 0,X
	PSHA		; Copy using stack operations
	DEX
	DECB
	BNE .COPY
	; Routines copied restore stack
	LDS ADDRT
	RTS
	
;-----------------------------------------------------------------------
; We assume that at least 1kB of RAM has to exist since we're using 
; subroutines here. So the first bank must be populated. Test up till
; RAMTOP (8kB by default)
RAMSIZE
	LDX #$0400	; Start just after 1kB
	STX ADDRT
	LDAB #1		; Count of KB
.SIZE	LDAA #$55	; Store 55AA
	STAA 4,X       
	LDAA #$AA
	STAA 5,X
	
	LDAA #$55	; Do a readback test
	CMPA 4,X
	BNE .FAIL
	LDAA #$AA
	CMPA 5,X
	BNE .FAIL
	; Readback fine, check next kB
	INCB
	LDAA ADDRT
	ADDA #4
	STAA ADDRT
	LDX ADDRT	; Next pointer to check
	
	CPX #RAMTOP
	BLT .SIZE
.FAIL   STAB RAMKB	; Save kB count
	RTS
;-----------------------------------------------------------------------

; Print out the banner and ram size
BANNER
	LDX #S_BANNER
	JSR PRINTSTR
	LDAA RAMKB
	ADDA #'0'
	JSR CONOUT
	LDX #S_MEM
	JSR PRINTSTR
	RTS
;-----------------------------------------------------------------------
; Onboard UART routines
S0IN	LDAB #$01	; Receive data full
.L      BITB UART1      
	BEQ .L
	LDAA UART1+1
	RTS
S1IN	LDAB #$01	; Receive data full
.L      BITB UART2      
	BEQ .L
	LDAA UART2+1
	RTS
S0OUT	LDAB #$02	; Transmit data empty
.L:	BITB UART1
	BEQ .L
	STAA UART1+1	; Send character
	RTS
S1OUT	LDAB #$02	; Transmit data empty
.L:	BITB UART2
	BEQ .L
	STAA UART2+1	; Send character
	RTS
; Non-blocking, 23 cycles (for XMODEM timeout)
S0IN_NB	LDAA #0		; 2 - Default return
	LDAB #$01	; 2 - 
	BITB UART1	; 4 -
	BEQ .E		; 4 -
	LDAA UART1+1	; 4 -
.E	TST A		; 2 - Set flags based on A
	RTS		; 5-
; Non-blocking, 23 cycles (for XMODEM timeout)
S1IN_NB	LDAA #0		; 2 - Default return
	LDAB #$01	; 2 - 
	BITB UART2	; 4 -
	BEQ .E		; 4 -
	LDAA UART2+1	; 4 -
.E	TST A		; 2 - Set flags based on A
	RTS		; 5-

;-----------------------------------------------------------------------
; Default device routines
DDEVIN
	TSTB
	BEQ	.CON
	CMPB #1
	BEQ	.AUX
	CMPB #2
	BEQ	.S0
	CMPB #3
	BEQ	.S1
	; Invalid ignore
	RTS
.CON	JMP CONIN
.AUX	JMP AUXIN
.S0	JMP S0IN
.S1	JMP S1IN


DDEVOUT
	TSTB
	BEQ	.CON
	CMPB #1
	BEQ	.AUX
	CMPB #2
	BEQ	.S0
	CMPB #3
	BEQ	.S1
	; Invalid ignore
	RTS
.CON	JMP CONOUT
.AUX	JMP AUXOUT
.S0	JMP S0OUT
.S1	JMP S1OUT

DDEVINNB
	TSTB
	BEQ	.CON
	CMPB #1
	BEQ	.AUX
	CMPB #2
	BEQ	.S0
	CMPB #3
	BEQ	.S1
	; Invalid ignore
	RTS
.CON	JMP CONINNB
.AUX	JMP AUXINNB
.S0	JMP S0IN_NB
.S1	JMP S1IN_NB

	
;-----------------------------------------------------------------------
; Print newline
NEWLINE	LDAA #'\r'
	JSR CONOUT
	LDAA #'\n'
	JMP CONOUT	; Tail call
	
;-----------------------------------------------------------------------
; Print null terminated string in X
PRINTSTR
	LDAA 0,X
	BEQ .END
	JSR CONOUT
	INX
	BRA PRINTSTR
.END    RTS

;-----------------------------------------------------------------------
; Convert to uppercase
TOUPPER
	CMPA #'a'
	BLT .DONE
	CMPA #'z'
	BGT .DONE
	ANDA #$DF	; Convert
.DONE	RTS

;-----------------------------------------------------------------------
; Revieve program over xmodem, loaded starting at $0100
XMDMRECV
	LDX #0
	STX TIMER	; Reset timeout
	STX SEQ		; Clear seq and CSUM
	INC SEQ		; Set first sequence to 1
	LDX ADDRT	; Destination pointer
	; Send NAK, and wait for start
.START	LDAA #NAK
	STAA LAST
	JSR CONOUT
.AWAIT	JSR CONINNB	; 9+23, 32 -
	BNE .GOT1	; 4 -
	INC TIMER+1	; 6 - Timeout counter
	BVC .AWAIT	; 4- (Each loop is 46 cycles * 256 = 11776 (12.7ms)
	INC TIMER+0	; 6 - 
	BVC .AWAIT	; 4 - Each loop is 11786 * 256 = 3,017,216 cycles
			; At 921.6kHz this takes at least 3.27 seconds to happen
	BRA .START	; Try again
.GOT1	; We got a character, is it our start of packet
	CMPA #SOH
	BNE .START	; It's not, continue trying.
	LDAA #0
	STAA TIMER
	STAA TIMER+1
	; Start of 1st packet, let the protocol begin
	BRA .S1A	; Skip waiting for SOH
.S1	JSR CONINNB
	BNE .GOT2
	INC TIMER+1
	BVC .S1		; Each loop is 12.7ms
	; Sender probably lost our last reply, resend
	LDAA LAST
	JSR CONOUT
	INC TIMER	; Use upper count of timer as timeout loop
	LDAB TIMER
	CMPB #10	; 10 tries before giving up
	BEQ .TIMEOUT
	BRA .S1
.TIMEOUT
	LDX #S_TIME
	JSR PRINTSTR
	JMP ENTER	; Cold start
.GOT2
	LDAB #0
	STAB TIMER+1	; Reset timers
	STAB TIMER+0
	STAB CSUM	; Reset checksum
	CMPA #EOT	
	BEQ .END	; End of transmission
	CMPA #SOH
	BNE .PFAIL	; 
.S1A	JSR CONIN
	CMPA SEQ	; Does the sequence match? 
	BNE .PFAIL
	JSR CONIN	; Get inverted seq
	COMA		; Invert
	CMPA SEQ	; Does it match?
	BNE .PFAIL
	; Okay now we're at the data
	LDAA #128	; # of bytes in data
	STAA COUNT
.S2	JSR CONIN	
	STAA 0,X	; Store byte
	INX		; Next address
	ADDA CSUM	; Compute checksum
	STAA CSUM
	DEC COUNT
	BNE .S2		; Loop till we get out 128 bytes
	; Get the checksum
	JSR CONIN
	CMPA CSUM
	BNE .PFAIL	; Checksum failed
	; Packet was good, ACK and continue
	LDAA #ACK
	STAA LAST
	JSR CONOUT
	INC SEQ
	BRA .S1
.PFAIL	LDAA #NAK
	STAA LAST
	JSR CONOUT	; We did not recieve the packet correctly
	BRA .S1
.END	RTS
	
;-----------------------------------------------------------------------
; Send via XMODEM starting at addr X to ADDRT
XMDMSEND	
	LDAA #0
	STAA BYTET
	LDAA #1
	STAA SEQ
.AWAIT	JSR CONIN	; Wait for NAK to start
	CMPA #NAK
	BNE .AWAIT
	; Okay now send the packet
.S1	LDAA #SOH	; Start of packet
	JSR CONOUT
	LDAA SEQ	; Sequence number
	JSR CONOUT
	COMA		; Inverted sequence
	JSR CONOUT
	LDAA #0
	STAA CSUM
	LDAA #128	; Bytes in packet
	STAA COUNT
	STX ADDRP	; Save address of start of packet 
	; Now the data
.S2	LDAA 0,X	; Read byte
	JSR CONOUT	; Send
	ADDA CSUM	; Compute checksum
	STAA CSUM
	INX		; Next address
	CPX ADDRT	; Are we at the stop address yet?
	BNE .S2A
	INC BYTET	; Indicate we went past the end
.S2A	DEC COUNT
	BNE .S2
	; Now the checksum
	LDAA CSUM
	JSR CONOUT
	; Check for ACK or NAK
.S3	JSR CONIN
	CMPA #ACK
	BEQ .NEXT
	CMPA #NAK
	BNE .S3		; Keep looking for response
.REDO	LDX ADDRP	; Reset address
	BRA .S1		; Try again
.NEXT	INC SEQ
	TST BYTET	; Did we hit the stop address before?
	BEQ .S1		; X < ADDRT
.DONE	LDAA #EOT
	JSR CONOUT
	JSR CONIN	; ACK?
	CMPA #ACK
	BNE .DONE
.DONE2	LDAA #ETB
	JSR CONOUT
	RTS


S_BANNER DB "FENNEC-68MON 0.3\r\n", 0
S_MEM    DB "kB MEMORY\r\n",0
S_TIME	 DB "TIMEOUT\r\n",0
S_SEND	 DB "SEND",0
S_RECV	 DB "RECV",0 
S_FILE	 DB " FILE NOW\r\n",0
S_OPTION DB "OPTION ROMS:\r\n", 0





IRQHND
SWIHND
NMIHND	
HALTL	BRA HALTL

	ORG $FFF8	; Vectors
IRQV	DW IRQHND
SWIV	DW SWIHND
NMIV	DW NMIHND
RESV	DW START
