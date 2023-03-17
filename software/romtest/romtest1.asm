
	CPU 6800

UART1	EQU $EFF0
UART2	EQU $EFF2

	ORG $F800	; 2kB ROM
START
	LDAA #$03	; Master reset
	STAA UART1
	STAA UART2
	LDAA #$15	; /16 8N1, RTS asserted no interrupts
	STAA UART1
	STAA UART2

	LDAA #$02	; Transmit data empty
	LDAB #'A'	; Character to send
TXLOOP	BITA UART1
	BEQ TXLOOP
	STAB UART1+1	; Send character
	BRA TXLOOP	; Repeat
	

IRQHND
SWIHND
NMIHND	
HALTL	BRA HALTL

	ORG $FFF8	; Vectors
IRQV	DW IRQHND
SWIV	DW SWIHND
NMIV	DW NMIHND
RESV	DW START
