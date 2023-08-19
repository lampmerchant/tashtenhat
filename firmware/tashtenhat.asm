;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  TashTenHat: I2C X10 Interface and UART for Raspberry Pi
;;;
;


;;; Connections ;;;

;;;                                                                    ;;;
;                               .--------.                               ;
;                       Supply -|01 \/ 14|- Ground                       ;
;          X10 Out <---    RA5 -|02    13|- RA0    <--> ICSP Data        ;
;           X10 In --->    RA4 -|03    12|- RA1    <--- ICSP Clock       ;
;         ICSP Vpp --->    RA3 -|04    11|- RA2    ----                  ;
;    Zero Crossing --->    RC5 -|05    10|- RC0    <--> I2C SCL          ;
;          UART TX <---    RC4 -|06    09|- RC1    <--> I2C SDA          ;
;          UART RX --->    RC3 -|07    08|- RC2    ---> Driver Enable    ;
;                               '--------'                               ;
;;;                                                                    ;;;


;;; Assembler Directives ;;;

	list		P=PIC16F1704, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P16F1704.inc
	errorlevel	-302	;Suppress "register not in bank 0" messages
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PPS1WAY_OFF & _ZCDDIS_ON & _PLLEN_ON & _STVREN_ON & _LVP_OFF
			;_WRT_OFF	Write protection off
			;_PPS1WAY_OFF	PPS can change more than once
			;_ZCDDIS_ON	Zero crossing detector disabled
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

I2CBASE	equ	0x58	;I2C base address
			;Base address + 0: X10 data stream
			;Base address + 1: X10 control register/AC cycle count
			;  X10 control register (write):
			;    7-4: TBD
			;    3-0: Outbound frame priority (default 0x0):
			;      0x0 - highest (wait 6 bit times from idle line)
			;      ...
			;      0xF - lowest (wait 21 bit times from idle line)
			;  AC cycle count (read):
			;    Ticks once per second, according to the AC line
			;Base address + 2: UART data stream
			;Base address + 3: UART control/status register
			;  Control (write):
			;    7-4: TBD
			;    3-0: Baud rate (default 0xC):
			;      0x0 - 300 Hz
			;      0x1 - 600 Hz
			;      0x2 - 1200 Hz
			;      0x3 - 1800 Hz
			;      0x4 - 2400 Hz
			;      0x5 - 4800 Hz
			;      0x6 - 9600 Hz
			;      0x7 - 19.2 kHz
			;      0x8 - 28.8 kHz
			;      0x9 - 38.4 kHz
			;      0xA - 57.6 kHz
			;      0xB - 78.8 kHz
			;      0xC - 115.2 kHz
			;      0xD - 500 kHz
			;      0xE - 1 MHz
			;      0xF - 2 MHz
			;  Status (read):
			;    7-4: UART I2C read queue length
			;    3-0: UART I2C write queue length
			;    If length is 0xF, actual length is 15 or greater

DEF_PRI	equ	7	;Default priority for X10 frames (lower=higher, 7=top)

XO_PORT	equ	PORTA	;X10 output port
XO_PIN	equ	RA5	;X10 output pin
XI_PORT	equ	PORTA	;X10 input port
XI_PIN	equ	RA4	;X10 input pin
SD_PPSO	equ	RC1PPS	;I2C SDA PPS output value
SC_PPSO	equ	RC0PPS	;I2C SCL PPS output value
DE_PORT	equ	PORTC	;Driver Enable port
DE_PIN	equ	RC2	;Driver Enable pin
TX_PORT	equ	PORTC	;UART Tx port
TX_PIN	equ	RC4	;UART Tx pin
TX_PPSO	equ	RC4PPS	;UART Tx PPS output value
SD_PPSI	equ	0x11;RC1;I2C SDA PPS input value
SC_PPSI	equ	0x10;RC0;I2C SCL PPS input value
RX_PPSI	equ	0x13;RC3;UART Rx pin PPS input value
ZC_PPSI	equ	0x15;RC5;Zero crossing PPS input value

			;FLAGS:
X10RECV	equ	7	;Set when an X10 receive is in progress
X10XMIT	equ	6	;Set when an X10 transmit is in progress
POLARTY	equ	5	;Toggles to indicate expected state of zero crossing
T0STAGE	equ	4	;Timer0 handler stage
I2COFS1	equ	1	;Active I2C address offset
I2COFS0	equ	0	; "


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	FLAGS	;You've got to have flags
	RQ_PUSH	;Push point for X10 I2C read queue
	RQ_ZERO	;Last-zero point for X10 I2C read queue
	RQ_POP	;Pop point for X10 I2C read queue
	WQ_PUSH	;Push point for X10 I2C write queue
	WQ_POP	;Pop point for X10 I2C write queue
	UQ_PUSH	;Push point for UART I2C read queue
	UQ_POP	;Pop point for UART I2C read queue
	XQ_PUSH	;Push point for UART I2C write queue
	XQ_POP	;Pop point for UART I2C write queue
	CCOUNTH	;Cycle counter
	CCOUNTL	; "
	BACKOFF	;Countdown of cycles (plus one) to wait before transmitting
	PRIORTY	;Value to set BACKOFF to when we see a one on the line
	X1
	X0
	
	endc

	;Linear memory:
	;0x2000-0x207F - UART I2C read (UART to PIC to RPi) queue
	;0x2080-0x20FF - UART I2C write (RPi to PIC to UART) queue
	;0x2100-0x217F - X10 I2C read (X10 to PIC to RPi) queue
	;0x2180-0x21BF - X10 I2C write (RPi to PIC to X10) queue


;;; Vectors ;;;

	org	0x0		;Reset vector
	goto	Init

	org	0x4		;Interrupt vector


;;; Interrupt Handler ;;;

Interrupt
	movlb	0		;If the last bit of the zero crossing shift
	btfsc	PIR3,CLC3IF	; register has changed, handle it
	bra	IntClc		; "
	btfsc	INTCON,TMR0IE	;If Timer0 is enabled and flagged an interrupt,
	btfss	INTCON,TMR0IF	; handle it
	bra	$+2		; "
	bra	IntTimer0	; "
	btfsc	PIR1,SSP1IF	;If there's been an SSP event, handle it
	bra	IntSsp		; "
	btfsc	PIR1,RCIF	;If a byte has come in from the UART, handle it
	bra	IntRx		; "
	btfsc	PIR1,TXIF	;If the UART wants a byte, handle it
	bra	IntTx		; "
	retfie

IntTx
	movf	XQ_PUSH,W	;If the write queue is empty, skip ahead to
	xorwf	XQ_POP,W	; disable the interrupt
	btfsc	STATUS,Z	; "
	bra	IntTx0		; "
	movlb	2		;We're transmitting, so make sure the driver
	bsf	DE_PORT,DE_PIN	; enable pin is high
	movf	XQ_POP,W	;Load write queue pop point into FSR0 so it can
	movwf	FSR0L		; be read from
	bcf	FSR0H,0		; "
	movf	INDF0,W		;Pop the next byte off the queue and load it
	movlb	3		; for transmission
	movwf	TXREG		; "
	incf	XQ_POP,F	;Advance and wrap the push point
	bsf	XQ_POP,7	; "
	retfie			;Return
IntTx0	movlb	1		;Disable the Tx interrupt
	bcf	PIE1,TXIE	; "
	retfie			;Return

IntRx
	movlb	3		;UART regs in bank 3
	incf	UQ_PUSH,W	;If this byte would overflow the queue, skip
	andlw	B'01111111'	; ahead to just throw it away
	xorwf	UQ_POP,W	; "
	btfsc	STATUS,Z	; "
	bra	IntRx0		; "
	movf	UQ_PUSH,W	;Load read queue push point into FSR1 so it can
	movwf	FSR1L		; be written to
	bcf	FSR1H,0		; "
	movf	RCREG,W		;Push the received byte onto the queue
	movwf	INDF1		; "
	incf	UQ_PUSH,F	;Advance and wrap the push point
	bcf	UQ_PUSH,7	; "
	retfie			;Return
IntRx0	movf	RCREG,W		;Throw away received byte, nowhere to put it
	retfie			;Return

IntSsp
	bcf	PIR1,SSP1IF	;Clear SSP interrupt
	movlb	4		; "
	btfsc	SSPSTAT,D_NOT_A	;If byte received was data, skip handling the
	bra	IntSsp0		; address
	movlw	B'11111100'	;Shift and mask the address to get the offset,
	andwf	FLAGS,F		; store in the low two bits of FLAGS
	lsrf	SSPBUF,W	; "
	andlw	B'00000011'	; "
	iorwf	FLAGS,F		; "
	btfss	SSPSTAT,R_NOT_W	;If this is a write, return and wait for the
	retfie			; next byte now that we've been addressed
	bra	IntSspRead	;This is a read so load first byte for transmit
IntSsp0	btfss	SSPSTAT,R_NOT_W	;If this is a write, skip ahead to handle write
	bra	IntSspWrite	; "
	btfsc	SSPCON2,ACKSTAT	;This is a read and the last byte in was data,
	retfie			; so if we got a NACK, return immediately
	;fall through

IntSspRead
	movf	FLAGS,W		;Branch off depending on the I2C address offset
	andlw	B'00000011'	; "
	brw			; "
	bra	IntSspReadX10	;0 - Read X10 stream
	bra	IntSspReadCycle	;1 - Read cycle count
	bra	IntSspReadUart	;2 - Read UART stream
	bra	IntSspReadUStat	;3 - Read UART status register

IntSspWrite
	movf	FLAGS,W		;Branch off depending on the I2C address offset
	andlw	B'00000011'	; "
	brw			; "
	bra	IntSspWriteX10	;0 - Write X10 stream
	bra	IntSspWriteX10Ct;1 - Write X10 control register
	bra	IntSspWriteUart	;2 - Write UART stream
	bra	IntSspWriteUCtrl;3 - Write UART control register

IntSspReadX10
	movf	RQ_POP,W	;Load read queue pop point into FSR0 so it can
	movwf	FSR0L		; be read from
	bsf	FSR0H,0		; "
	movf	INDF0,W		;Pick up the next byte, load for transmission
	movwf	SSPBUF		; "
	bsf	SSPCON1,CKP	;Release the clock so it can be sent
	movf	RQ_POP,W	;If the pop point is equal to the last-zero
	xorwf	RQ_ZERO,W	; point, return
	btfsc	STATUS,Z	; "
	retfie			; "
	incf	RQ_POP,F	;Otherwise, advance and loop the pop point
	bcf	RQ_POP,7	; "
	retfie			;Return

IntSspWriteX10Ct
	movf	SSPBUF,W	;Bottom four bits of control register are the
	andlw	B'00001111'	; priority (7 gives us 6 cycles from the last
	addlw	7		; one bit, so we make that 0x0, the highest
	movwf	PRIORTY		; priority value)
	;TODO define and act on the rest of the control register
	retfie			;Return

IntSspWriteX10
	movf	WQ_PUSH,W	;Load write queue push point into FSR1 so it
	movwf	FSR1L		; can be written to
	bsf	FSR1H,0		; "
	movf	SSPBUF,W	;Push the received byte onto the queue
	movwf	INDF1		; "
	incf	WQ_PUSH,F	;Advance and wrap the push point
	bcf	WQ_PUSH,6	; "
	retfie			;Return

IntSspReadUart
	movf	UQ_POP,W	;Load read queue pop point into FSR0 so it can
	movwf	FSR0L		; be read from
	bcf	FSR0H,0		; "
	xorwf	UQ_PUSH,W	;If the queue is empty, skip ahead
	btfsc	STATUS,Z	; "
	bra	IntSRU0		; "
	incf	UQ_POP,F	;If the queue is not empty, advance and wrap
	bcf	UQ_POP,7	; the push point
IntSRU0	movf	INDF0,W		;Pick up the next byte, load for transmission
	movwf	SSPBUF		; "
	bsf	SSPCON1,CKP	;Release the clock so it can be sent
	retfie			;Return

IntSspWriteUart
	movf	XQ_PUSH,W	;Load write queue push point into FSR1 so it
	movwf	FSR1L		; can be written to
	bcf	FSR1H,0		; "
	movf	SSPBUF,W	;Push the received byte onto the queue
	movwf	INDF1		; "
	incf	XQ_PUSH,F	;Advance and wrap the push point
	bsf	XQ_PUSH,7	; "
	movlb	1		;Enable Tx interrupt if it's not enabled
	bsf	PIE1,TXIE	; already
	retfie			;Return

IntSspReadUStat
	movf	UQ_POP,W	;Compute length of UART read queue
	subwf	UQ_PUSH,W	; "
	sublw	15		;If it's greater than 15, truncate it to 15
	btfss	STATUS,C	; "
	movlw	0		; "
	sublw	15		; "
	movwf	FSR0L		;Make this number the upper nibble of the reply
	swapf	FSR0L,F		; using FSR0L as temporary storage
	movf	XQ_POP,W	;Compute length of UART write queue
	subwf	XQ_PUSH,W	; "
	sublw	15		;If it's greater than 15, truncate it to 15
	btfss	STATUS,C	; "
	movlw	0		; "
	sublw	15		; "
	iorwf	FSR0L,W		;Make this number the lower nibble of the reply
	movwf	SSPBUF		; and load it for transmission
	bsf	SSPCON1,CKP	;Release the clock so it can be sent
	retfie			;Return

IntSspWriteUCtrl
	movf	SSPBUF,W	;Save a copy of the control register contents
	movwf	FSR1L		; using FSR1L as temporary storage
	andlw	B'00001111'	;Use the low four bits as an index into the
	lslf	WREG,W		; 16-bit baud rate lookup table
	movwf	FSR0L		; "
	movlw	0x80|(high Baud); "
	movwf	FSR0H		; "
	movlb	3		;Dereference the baud rate and load it into the
	moviw	FSR0++		; baud rate registers
	movwf	SPBRGH		; "
	moviw	FSR0++		; "
	movwf	SPBRGL		; "
	;TODO define and act on the rest of the control register
	retfie			;Return

IntSspReadCycle
	movf	CCOUNTH,W	;Load the cycle count for transmission
	movwf	SSPBUF		; "
	bsf	SSPCON1,CKP	;Release the clock so it can be sent
	retfie			;Return

IntClc
	bcf	PIR3,CLC3IF	;Clear the interrupt
	movlb	30		;Check to see if the shift register's bits all
	movf	CLCDATA,W	; agree that the zero crossing signal has
	btfsc	FLAGS,POLARTY	; flipped
	xorlw	B'00000111'	; "
	btfss	STATUS,Z	;If not, return
	retfie			; "
	movlw	1 << POLARTY	;Flip the polarity flag for next time
	xorwf	FLAGS,F		; "
	movlb	1		;Disable the CLC interrupt while the timer is
	bcf	PIE3,CLC3IE	; active
	movlb	0		;Arm Timer0 to interrupt twice, once after 512
	clrf	TMR0		; us to sample the X10 input signal and once
	bsf	INTCON,TMR0IE	; after 1.024 ms to turn off the 120 kHz
	bcf	INTCON,TMR0IF	; oscillator if it was on
	bcf	FLAGS,T0STAGE	; "
	decfsz	CCOUNTL,F	;Increment the cycle counter
	bra	IntClc0		; "
	incf	CCOUNTH,F	; "
	movlw	120		; "
	movwf	CCOUNTL		; "
IntClc0	movf	WQ_POP,W	;Load write queue pop point into FSR0 so it can
	movwf	FSR0L		; be read from
	bsf	FSR0H,0		; "
	bcf	STATUS,C	;If a transmit is already in progress, skip
	btfsc	FLAGS,X10XMIT	; ahead with carry clear
	bra	IntClc1		; "
	movf	BACKOFF,F	;If the backoff counter is nonzero, decrement
	btfss	STATUS,Z	; it; if it's still nonzero, we're waiting for
	decf	BACKOFF,F	; an idle period on the line before we start to
	btfss	STATUS,Z	; transmit, so return
	retfie			; "
	xorwf	WQ_PUSH,W	;If the queue is empty, return
	btfsc	STATUS,Z	; "
	retfie			; "
	bsf	FLAGS,X10XMIT	;Flag that a transmission has started
	bsf	STATUS,C	;Set carry so we know where byte ends
IntClc1	rlf	INDF0,F		;Rotate next bit to be transmitted into carry
	movf	INDF0,F		;If byte still has bits, skip ahead
	btfss	STATUS,Z	; "
	bra	IntClc2		; "
	incf	WQ_POP,W	;Advance and wrap the pop point
	andlw	B'10111111'	; "
	movwf	WQ_POP		; "
	movwf	FSR0L		; "
	xorwf	WQ_PUSH,W	;If we've emptied the queue, skip ahead
	btfsc	STATUS,Z	; "
	bra	IntClc3		; "
	rlf	INDF0,F		;Rotate first bit of next byte into C (C was 1)
IntClc2	movlb	2		;If bit to be sent is 1, turn on the 120 kHz
	btfsc	STATUS,C	; oscillator (to be turned off ~1 ms later)
	bcf	XO_PORT,XO_PIN	; "
	retfie			;Return
IntClc3	bcf	FLAGS,X10XMIT	;Flag that transmit is no longer in progress
	retfie			;Return

IntTimer0
	bcf	INTCON,TMR0IF	;Clear Timer0 interrupt
	btfsc	FLAGS,T0STAGE	;Proceed to the first or second stage depending
	bra	IntTimer0Second	; on the flag
	;fall through

IntTimer0First
	bsf	FLAGS,T0STAGE	;Set Timer0 to run second stage next time
	bcf	STATUS,C	;Copy state of X10 input pin into carry flag,
	btfss	XI_PORT,XI_PIN	; inverted so 1 means 120 kHz detected
	bsf	STATUS,C	; "
	movf	PRIORTY,W	;If we received a 1, set the backoff counter so
	btfsc	STATUS,C	; we wait for this transmission to finish and
	movwf	BACKOFF		; some time to pass
	movf	RQ_PUSH,W	;Load read queue push point into FSR1 so it can
	movwf	FSR1L		; be written to
	bsf	FSR1H,0		; "
	btfss	STATUS,C	;If the received bit is a zero and no receive
	btfsc	FLAGS,X10RECV	; is in progress, return
	bra	$+2		; "
	retfie			; "
	movlw	B'00000001'	;If a receive wasn't in progress before, set up
	btfss	FLAGS,X10RECV	; to receive bits into the queue push point and
	movwf	INDF1		; flag that a receive is in progress now
	bsf	FLAGS,X10RECV	; "
	rlf	INDF1,F		;Rotate current bit into the queue push point
	btfss	STATUS,C	;If the byte isn't yet full, return
	retfie			; "
	incf	RQ_PUSH,W	;The byte is full, so advance and wrap the push
	andlw	B'01111111'	; point
	movwf	RQ_PUSH		; "
	xorwf	RQ_POP,W	;If the push point has reached the pop point,
	btfsc	STATUS,Z	; the queue has overflowed, so reset it
	bra	IntT0F1		; "
	movf	INDF1,F		;If the byte we just filled is all zeroes, skip
	btfsc	STATUS,Z	; ahead
	bra	IntT0F0		; "
	movf	RQ_PUSH,W	;The byte has at least one one bit in it so the
	movwf	FSR1L		; receive continues, set up for the next byte
	movlw	B'00000001'	; "
	movwf	INDF1		; "
	retfie			;Return
IntT0F0	bcf	FLAGS,X10RECV	;The byte is all zeroes, so receive is over
	movf	FSR1L,W		;Update the last-zero pointer to indicate that
	movwf	RQ_ZERO		; the queue should not be read beyond here
	retfie			;Return
IntT0F1	movlw	1		;Reset the queue since it has overflowed
	movwf	RQ_PUSH		; "
	clrf	RQ_ZERO		; "
	clrf	RQ_POP		; "
	clrf	FSR1L		; "
	clrf	INDF1		; "
	bcf	FLAGS,X10RECV	; "
	retfie			;Return

IntTimer0Second
	bcf	INTCON,TMR0IE	;Turn off Timer0 interrupts
	bcf	FLAGS,T0STAGE	;Set Timer0 to run first stage next time
	movlb	1		;Reenable CLC interrupt since timer is no
	bsf	PIE3,CLC3IE	; longer active
	movlb	2		;Turn off 120 kHz oscillator; if it was on, it
	bsf	XO_PORT,XO_PIN	; has been on for ~1 ms at this point
	retfie			;Done


;;; Initialization ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON

	banksel	OSCSTAT		;Spin until PLL is ready and instruction clock
	btfss	OSCSTAT,PLLR	; gears up to 8 MHz
	bra	$-1

	banksel	RCSTA		;UART async mode, 115200 Hz
	movlw	B'01001000'
	movwf	BAUDCON
	clrf	SPBRGH
	movlw	68
	movwf	SPBRGL
	movlw	B'00100110'
	movwf	TXSTA
	movlw	B'10010000'
	movwf	RCSTA
	clrf	TXREG

	banksel	SSPCON1		;SSP in I2C slave mode at I2CBASE, with low two
	movlw	I2CBASE << 1	; (not three) bits of address used as register
	movwf	SSPADD		; select, SMBus input logic levels so we can
	movlw	B'11111000'	; communicate at 3.3V levels even when powered
	movwf	SSPMSK		; at 5V
	movlw	B'01000000'
	movwf	SSPSTAT
	movlw	B'00110110'
	movwf	SSPCON1

	banksel	CLC1CON		;
	movlw	B'00011010'	;            CLC1      CLC2      CLC3
	movwf	CLC1SEL0 ;Timer2;
	movwf	CLC2SEL0 ;Timer2;              0         0         0
	movwf	CLC3SEL0 ;Timer2;            __|__     __|__     __|__
	clrf	CLC1SEL1 ;CLCIN0;           |  S  |   |  S  |   |  S  |
	movlw	B'00000100'	;   CLCIN0--|D   Q|---|D   Q|---|D   Q|--LC3OUT
	movwf	CLC2SEL1 ;LC1OUT;           |     |   |     |   |     |
	movlw	B'00000101'	;         ,-|>    | ,-|>    | ,-|>    |
	movwf	CLC3SEL1 ;LC2OUT;         | |__R__| | |__R__| | |__R__|
	movlw	B'00000010'	;         |    |    |    |    |    |
	movwf	CLC1GLS0	;         |    0    |    0    |    0
	movwf	CLC2GLS0	; Timer2--+---------+---------'
	movwf	CLC3GLS0	;
	movlw	B'00001000'
	movwf	CLC1GLS1
	movwf	CLC2GLS1
	movwf	CLC3GLS1
	clrf	CLC1GLS2
	clrf	CLC2GLS2
	clrf	CLC3GLS2
	clrf	CLC1GLS3
	clrf	CLC2GLS3
	clrf	CLC3GLS3
	clrf	CLC1POL
	clrf	CLC2POL
	clrf	CLC3POL
	movlw	B'10000100'
	movwf	CLC1CON
	movwf	CLC2CON
	movlw	B'10011100'
	movwf	CLC3CON

	banksel	OPTION_REG	;Weak pull-ups on for reading jumpers, Timer0
	movlw	B'01010011'	; ticks 1:16 with instruction clock so that it
	movwf	OPTION_REG	; overflows after 512 us

	banksel	T2CON		;Timer2 ticks 1:1 with instruction clock and
	movlw	B'00000100'	; has period of 12 us, used to clock samples of
	movwf	T2CON		; the zero crossing line into the CLC shift
	movlw	96		; register
	movwf	PR2

	banksel	RA0PPS		;Assign pins for SDA, SCL, Tx
	movlw	B'00010001'
	movwf	SD_PPSO
	movlw	B'00010000'
	movwf	SC_PPSO
	movlw	B'00010100'
	movwf	TX_PPSO

	banksel	RXPPS		;Assign pins for SDA, SCL, Rx, Zero Crossing
	movlw	SD_PPSI		; (CLC input 0)
	movwf	SSPDATPPS
	movlw	SC_PPSI
	movwf	SSPCLKPPS
	movlw	RX_PPSI
	movwf	RXPPS
	movlw	ZC_PPSI
	movwf	CLCIN0PPS

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA
	clrf	ANSELC

	banksel	LATA		;X10 output pin is high (120 KHz off), driver
	bsf	XO_PORT,XO_PIN	; enable pin is low (driver off)
	bcf	DE_PORT,DE_PIN

	banksel	TRISA		;X10 out, Tx, and driver enable output, all
	bcf	XO_PORT,XO_PIN	; others input
	bcf	TX_PORT,TX_PIN
	bcf	DE_PORT,DE_PIN

	banksel	PIE1		;SSP, Rx, and CLC3 peripheral interrupts on
	movlw	(1 << SSP1IE) | (1 << RCIE)
	movwf	PIE1
	movlw	1 << CLC3IE
	movwf	PIE3

	movlw	0x20		;Set up FSRs to point more or less permanently
	movwf	FSR0H		; to linear memory
	movwf	FSR1H

	clrf	FLAGS		;Initialize key globals
	movlw	1
	movwf	RQ_PUSH
	clrf	RQ_ZERO
	clrf	RQ_POP
	clrf	FSR1L
	bsf	FSR1H,0
	clrf	INDF1
	clrf	UQ_PUSH
	clrf	UQ_POP
	movlw	0x80
	movwf	WQ_PUSH
	movwf	WQ_POP
	movwf	XQ_PUSH
	movwf	XQ_POP
	clrf	CCOUNTH
	clrf	CCOUNTL
	movlw	DEF_PRI
	movwf	PRIORTY
	movwf	BACKOFF

	movlw	B'11001000'	;IOC interrupts, peripheral interrupts, and
	movwf	INTCON		; interrupt subsystem on
	;fall through


;;; Mainline ;;;

Main
	bsf	INTCON,GIE	;Reenable interrupts
	movlb	2		;If the driver enable is already off, don't
	btfss	DE_PORT,DE_PIN	; bother checking whether we should turn it off
	bra	Main		; "
	movlb	3		;If the UART is transmitting, do not turn off
	btfss	TXSTA,TRMT	; the driver enable
	bra	Main		; "
	bcf	INTCON,GIE	;Check if write queue is empty
	movf	XQ_PUSH,W	; "
	xorwf	XQ_POP,W	; "
	btfss	STATUS,Z	;If write queue is not empty, do not turn off
	bra	Main		; the driver enable
	movlb	2		;If the UART is not transmitting and the write
	bcf	DE_PORT,DE_PIN	; queue is empty, turn off the driver enable
	bra	Main		;Loop


;;; Lookup Tables ;;;

	org	0xF00

;Baud rate LUT for UART
Baud				; Baud Rate  Actual       % Err
	dt	0x68,0x2A	;   300 Hz      299.9 Hz  0.00%
	dt	0x34,0x14	;   600 Hz      600.0 Hz  0.00%
	dt	0x1A,0x0A	;  1200 Hz     1199.9 Hz  0.00%
	dt	0x11,0x5B	;  1800 Hz     1800.1 Hz  0.01%
	dt	0x0D,0x04	;  2400 Hz     2400.2 Hz  0.01%
	dt	0x06,0x82	;  4800 Hz     4799.0 Hz  0.02%
	dt	0x03,0x40	;  9600 Hz     9603.8 Hz  0.04%
	dt	0x01,0xA0	;  19.2 kHz   19184.6 Hz  0.08%
	dt	0x01,0x15	;  28.8 kHz   28776.9 Hz  0.08%
	dt	0x00,0xCF	;  38.4 kHz   38461.5 Hz  0.16%
	dt	0x00,0x8A	;  57.6 kHz   57553.9 Hz  0.08%
	dt	0x00,0x65	;  78.8 kHz   78431.3 Hz  0.47%
	dt	0x00,0x44	; 115.2 kHz  115942.0 Hz  0.64%
	dt	0x00,0x0F	;   500 kHz    500000 Hz  0.00%
	dt	0x00,0x07	;     1 MHz   1000000 Hz  0.00%
	dt	0x00,0x03	;     2 MHz   2000000 Hz  0.00%


;;; End of Program ;;;
	end
