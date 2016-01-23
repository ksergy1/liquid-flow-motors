.include "m8def.inc"
; INFO INFO INFO
; fosc = 8MHz
; Fuse-bits : MSB ... LSB (76543210)
;   High byte: 11111111
;   Low byte : 11100100

; 68 cycles per each drive w/o spi chat 
; 5bytes*8bits/byte = 40bits for spi chat = 160 clocks
; 68+160 +7 = 235 clocks
; SPI RAM operation uses protocol for Microchip 23A640 64kbit RAM IC
; необходимо ожидать 7 такта после выбора ведомого устройства

; PROGRAM

; maximum Number of drives that could be controlled
.equ MAX_NDRIVES_GLOBAL = 0x04 ; it should be a power of two
; number of last drive
.equ MAX_DRIVE_NUM = 0x03 ; it is MAX_NDRIVES_GLOBAL-1
; size of drive comand
.equ DR_COM_SZ = 0x01
; size of comand indices
.equ COM_IND_SZ = 0x02
; max number of comands for each drive
.equ MAX_NUM_COM_DRV = 0x0400
; number of clocks that one iteration of OneDriveCycle cycle gets to be done (approx) (dec - 235)
.equ RealTimeOneCycle = 0xeb

.def temp = r17
.def temp2 = r18
.def dataTemp = r16
.def temp3 = r14

; just for drives number and comands r/w -------------
; temporary 2bytes
.def tempL = r4
.def tempH = r5
; counter of comands already recieved and writen to (or read from in further operation) RAM for current drive
.def CurComNumL = r6
.def CurComNumH = r7
; temporary place for TimeScale (as a counter)
.def tempScale = r10
; Time scale (0..255)
.def TimeScale = r11
; temp place for comand
.def temp_com = r12
; index of current drive ( which is worked now )
.def CurDrvNum = r17
; drive commands number SRAM field size (as register)
.def DrComNumSzReg = r18
; Number of bytes in RAM that is used for commands for drive. = MAX_NUM_COM_DRV
.def coms_pages = r19
; Max number of comands per drive (as register)
.def MaxNumComsL = r20
.def MaxNumComsH = r21
; Max number of drives to be controlled (as register)
.def MaxNDrivesReg = r22
; Number of drives to be controlled
.def Ndrv = r23
; Low address for RAM
.def RAMaddrL = r24
; High address for RAM
.def RAMaddrH = r25
; Total Number of comands for current drive (when read from UART)
.def DrvComNumL = r26
.def DrvComNumH = r27

; Drive selecting I/O port
.equ SelectorPort = PORTC
; Drive selecting register
.def SelectorReg = r12
; Pin for deselecting drives
.equ DrivesDeselPin = PC5
; This reg is working as fully clear reg for operations like adc with 0 operand
.def CleanReg = r11
; ----------------------------------

.def tempBaudRateH = r17
.def tempBaudRateL = r16
.equ DDR_SPI = DDRB
.equ DD_SCK = DDB5
.equ DD_MISO = DDB4
.equ DD_MOSI = DDB3

; Memory read/write data and read/write status register commands
.equ RAM_RDSR = 0b00000101
.equ RAM_WRSR = 0b00000001
.equ RAM_READ = 0b00000011
.equ RAM_WRTE = 0b00000010

; enable RAM byte operation mode without a HOLD functionality
.equ RAM_SB_byte_oper = 0b00000011

; uart baudrate coeff
.equ uart_baudrate_h = 0b00000000
.equ uart_baudrate_l = 0b00110011

; uart data start byte from PC
.equ UartDataStartByte = 0b11001001
; uart ack for data start
.equ UartDataStartAck = 0b00110110
; uart continue byte
.equ UartContByte = 0b01101101

; #MemorySelect pin number 0 @ port B
; #MemorySelect
.equ MemSelPort = PORTB
.equ MemSelBit = PB0

; Status leds
.equ StatusReadyPort = PORTD
.equ StatusConfigDone = PD7
.equ StatusUartTxRx = PD6
.equ StatusSpiTxRx = PD5

; SRAM offset
.equ SRAMoffset = 0x0100
.equ NdrivesOff = 0x0000
.equ NdrivesSz = 0x0001
.equ DrComNumSz = 0x0001

; motor syncing port and pin
.equ SyncMotorPort = PORTB
.equ SyncMotorPin = PB1

; zero address
rjmp MAIN;

; ------------ MAIN --------------
MAIN:
  ; configurations block
  ; stack init
	ldi temp, low(RAMEND);
	ldi temp2, high(RAMEND);
	out SPH, temp2;
	out SPL, temp;

  ; communications configuration
	rcall ConfigPorts;
	rcall SPI_MasterInit;
	rcall RAM_Config;
	rcall RAM_Clear;
	rcall UART_Init;
	sbi StatusReadyPort, StatusConfigDone;

  ; communication with PC block
  WaitDataStart:
	rcall UART_Rcv;
	cpi dataTemp, UartDataStartByte;
	brne WaitDataStart;
  ; begin of data from PC
	sbi StatusReadyPort, StatusUartTxRx;
	ldi dataTemp, UartDataStartAck;
	rcall UART_Snd;

  ; begin communication with PC
  ; Ndrives in
  UartNdrivesIn:
	rcall UART_Rcv;
	mov temp, dataTemp;
  ; Write Ndrives to SRAM
	ldi ZH, High(SRAMoffset+NdrivesOff);
	ldi ZL, Low(SRAMoffset+NdrivesOff);
	st Z, dataTemp;
	adiw ZH:ZL, NdrivesSZ;
  ; save Ndrives to Ndrv
	mov Ndrv, dataTemp;

  ; Allow further communication with PC
	ldi dataTemp, UartContByte;
	rcall UART_Snd;

  ; recieve TimeScale byte
	rcall UART_Rcv;
	mov TimeScale, dataTemp;

  ; Allow further communication with PC
	ldi dataTemp, UartContByte;
	rcall UART_Snd;

  ; Recieving commands numbers
	mov CurDrvNum, Ndrv;
  CommNumRcv:
		rcall UART_Rcv;
		st Z+, dataTemp;
		rcall UART_Rcv;
		st Z+, dataTemp;
;		adiw ZH:ZL, DrComNumSz;
		dec CurDrvNum;
	brcc CommNumRcv;
  ; Recieving drives commands number is over

  ; allowing further communications
	ldi dataTemp, UartContByte;
	rcall UART_Snd;

  ; Now - recieving comands
	ldi ZH, High(SRAMoffset+NdrivesOff+NdrivesSZ);
	ldi ZL, Low(SRAMoffset+NdrivesOff+NdrivesSZ);
	clr RAMaddrH;
	clr RAMaddrL;
	ldi coms_pages, High(MAX_NUM_COM_DRV); coms_pages has a size for comands in full 256bytes pages 
	; select RAM for SPI operation
	cbi MemSelPort, MemSelBit

	mov CurDrvNum, Ndrv;
  CommandsRcv:
	ld DrvComNumL, Z+;		; number of comands for current drive is read from SRAM
	ld DrvComNumH, Z+;
	clr CurComNumL;			; number of comands already read from UART
	clr CurComNumH;
  OneComandRcv:
		rcall UART_Rcv;
		mov temp_com, dataTemp;
	; save dataTemp contents to RAM
		ldi dataTemp, RAM_WRTE;
		rcall SPI_MasterTransmit;

		mov dataTemp, RAMaddrH;
		rcall SPI_MasterTransmit;
		mov dataTemp, RAMaddrL;
		rcall SPI_MasterTransmit;

		mov dataTemp, temp_com;
		rcall SPI_MasterTransmit;

		adiw RAMaddrH:RAMaddrL, DR_COM_SZ;

		ldi dataTemp, UartContByte;
		rcall UART_Snd;
		; One command recieved

		subi DrvComNumL, 0x01;
		sbci DrvComNumH, 0x00;
		; если DrvComNumH:DrvComNumL != 0x0000
		brcc OneComandRcv;
		; commands for CurDrvNum drive recieved
	clr RAMaddrL;
	add RAMaddrH, coms_pages;

	dec CurDrvNum;
	brcc CommandsRcv;
  ; now deselect ram from spi operation
	sbi MemSelPort, MemSelBit;

  ; waiting for allowement for drives controlling and SPI tx/rx with slave MCUs
  WaitDataStart_for_SPI:
	rcall UART_Rcv;
	cpi dataTemp, UartContByte;
	brne WaitDataStart_for_SPI;
  ; It is ok to start working with slave MCUs

	cbi StatusReadyPort, StatusUartTxRx;

  ; now operating on SPI
	sbi StatusReadyPort, StatusSpiTxRx;

	clr CleanReg
	clr SelectorReg;
	out SelectorPort, CleanReg;

  ; make register copyes of some literals (immidiates)
	ldi DrComNumSzReg, COM_IND_SZ;
	ldi MaxNDrivesReg, MAX_NDRIVES_GLOBAL;
	ldi MaxNumComsL, Low(MAX_NUM_COM_DRV);
	ldi MaxNumComsH, High(MAX_NUM_COM_DRV);
  ; need that for "and"
	ldi Ndrv, (MAX_NDRIVES_GLOBAL-0x01)
  LargeHadronCollider:
	OneDriveCycle:
  ; time scaling implementation
		mov tempScale, TimeScale;
	TimeScaleFull:
 		; dataTemp reg is unused now, though we can use it
		ldi dataTemp, RealTimeOneCycle;
	TimeScaleOneTime:
			dec dataTemp;
			; dataTemp is still not a zero? dec it once more!
			brcc TimeScaleOneTime;
		dec tempScale;
		; tempScale is still not a zero? dec it once more
		brcc TimeScaleFull;

  ; loading current drive total commands number
		ldi ZH, High(SRAMoffset+NdrivesOff+NdrivesSz);
		ldi ZL, Low(SRAMoffset+NdrivesOff+NdrivesSz);
  ; address shift for current drive total commands number (in SRAM)
		mul DrComNumSzReg, SelectorReg;
		add ZL, r0;
		adc ZH, r1;
  ; load current drive total commands number
		ld DrvComNumL, Z+;
		ld DrvComNumH, Z+;
		sbiw ZH:ZL, COM_IND_SZ;
  ; address shift for current drive current command index (in SRAM)
		mul DrComNumSzReg, MaxNDrivesReg;
		add ZL, r0;
		adc ZH, r1;
  ; load current drive command index
		ld CurComNumH, Z+;
		ld CurComNumL, Z+;
  ; check if command index is less than total commands number
  ; when ot is not - we make index equal to zero
		movw tempH:tempL, DrvComNumH:DrvComNumL;
		sub tempL, CurComNumL;
		brne IndexOk;
		sbc tempH, CurComNumH;
		brne IndexOk;
  ClearComandIndex:
		clr CurComNumL;
		clr CurComNumH;
  IndexOk:
  ; load comand from external RAM
		mul MaxNumComsH, SelectorReg;
		mov r2,r0;
		mul MaxNumComsL, SelectorReg;
		add r2, r1;
  ; copy r0->RAMaddrL, r1->RAMaddrH
		movw RAMaddrL, r0;
  ; offset responsively to current command index
		add RAMaddrL, CurComNumL;
		adc RAMaddrH, CurComNumH;
  ; select RAM
		cbi MemSelPort, MemSelBit;
  ; Retrieve data from RAM
		ldi dataTemp, RAM_READ;
		rcall SPI_MasterTransmit;

		mov dataTemp, RAMaddrH;
		rcall SPI_MasterTransmit;

		mov dataTemp, RAMaddrL;
		rcall SPI_MasterTransmit;

		rcall SPI_Recieve;

  ; deselect RAM from SPI operation
		sbi MemSelPort, MemSelBit;
  ; select current drive for SPI operation
		out SelectorPort, SelectorReg;
		sbi SelectorPort, DrivesDeselPin;
  ; waiting 7 clocks for slave to eneble its SPI xfering
		nop;
		nop;
		nop;
		nop;
		nop;
		nop;
		nop;
  ; send dataTemp (command is still there) to selected slave
		rcall SPI_MasterTransmit;
  ; deselect slave
		cbi SelectorPort, DrivesDeselPin;
  ; change to next command for this drive;
		ldi dataTemp, 0x01;
		add CurComNumL, dataTemp;
		ldi dataTemp, 0x00;
		adc CurComNumH, dataTemp;
  ; store new comand index to SRAM (Z-register has not changed yet, that sounds like good ;-)
		sbiw ZH:ZL, COM_IND_SZ;
		st Z+, CurComNumL;
		st Z+, CurComNumH;

  ; change to next drive
		inc SelectorReg;
		and SelectorReg, Ndrv;
	brcc OneDriveCycle;					68 cycles per each drive w/o spi chat
	sbi SyncMotorPort, SyncMotorPin;
	nop;
	cbi SyncMotorPort, SyncMotorPin;
	rjmp LargeHadronCollider;

; ------------------------------- MAIN FINISHED -----------------------------

; ----------- PROCS ------------
; configure ports
; In - NONE
; Out - NONE
ConfigPorts:
	; config drive selector (PORTC) - 0,1,2,3,4,5 - out, 6 - in
	ldi temp, 0b00111111;
	out DDRC, temp;
	; config memory selector (PORTB) - 0, MOSI, SCK, [1, 2 - unsused,for safety] - out, MISO - in.
	ldi temp, 0b00101111;
	out DDRB, temp;
	; config UART and status (PORTD) - 1,2,3,5,6,7 - out, 0,4 - in
	ldi temp, 0b11101110;
	out DDRD, temp;
	; globaly disable interrupts
	cli;
	ret

; allow SPI
; In - NONE
; Out - NONE
SPI_MasterInit:
; Set MOSI and SCK direction to output, all others are set to input
	ldi temp, (1<<DD_MOSI)|(1<<DD_SCK)|(1<<MemSelBit)|(1<<1)|(1<<2)
	out DDR_SPI,temp
	; Enable SPI, Master, set clock rate fck/4
	ldi temp, (1<<SPE)|(1<<MSTR)
	out SPCR, temp
	; Double the clock rate! up to fck/2
;	sbi SPSR, SPI2X
	ret

; SPI tranmition as master
; In - dataTemp - byte to be tranmitted
; Out - NONE
SPI_MasterTransmit:
; Start transmission of data (r16)
	out SPDR, dataTemp
  Wait_Transmit:
; Wait for transmission complete
	sbis SPSR,SPIF
	rjmp Wait_Transmit
	ret

; SPI recieve routine (it uses FIFO-ed SPI with 0b00000000 data to be sent)
; In - NONE
; Out - dataTemp - retrieved byte
SPI_Recieve:
	clr dataTemp
	rcall SPI_MasterTransmit
	in dataTemp, SPDR
	ret

; prepare ram for byte operation (before communication with PC)
; In - NONE
; Out - NONE
RAM_Config:
  ; select RAM for SPI operation
	cbi MemSelPort, MemSelBit
  ; ram preparation for byte operation mode
	ldi dataTemp, RAM_WRSR
	rcall SPI_MasterTransmit

	ldi dataTemp, RAM_SB_byte_oper
	rcall SPI_MasterTransmit

  ; deselct ram for SPI transmition
	sbi PORTB, MemSelBit
	ret

; clear RAM
; In - NONE
; Out - NONE
RAM_Clear:
	; select RAM for SPI operation
	cbi MemSelPort, MemSelBit
	ldi Ndrv, MAX_NDRIVES_GLOBAL;
	clr RAMaddrL;
	clr RAMaddrH;
  ClearRAMAllDrives:
	ldi temp, High(MAX_NUM_COM_DRV);
	add temp, RAMaddrH;
  ClearRAMOneDrive:
		ldi dataTemp, RAM_WRTE;
		rcall SPI_MasterTransmit;

		mov dataTemp, RAMaddrH;
		rcall SPI_MasterTransmit;

		mov dataTemp, RAMaddrL;
		rcall SPI_MasterTransmit;

		clr dataTemp;
		rcall SPI_MasterTransmit;

		adiw RAMaddrH:RAMaddrL, 0x01;

		cp temp, RAMaddrH;
	brne ClearRAMOneDrive;

	dec Ndrv;
	brne ClearRAMAllDrives;
	ret

; UART initialization @ 9600 bps
; In - NONE
; Out - NONE
UART_Init:
  ; setting baudrate
	ldi tempBaudRateH, uart_baudrate_h
	ldi tempBaudRateL, uart_baudrate_l
	out UBRRH, tempBaudRateH
	out UBRRL, tempBaudRateL
  ; setting frame format: 8data bits, 2 stop bits
	ldi dataTemp, (1<<URSEL)|(1<<USBS)|(3<<UCSZ0)
	out UCSRC, dataTemp
  ; Enable reciever and transmitter
	ldi dataTemp, (1<<RXEN)|(1<<TXEN)
	out UCSRB, dataTemp
	ret

; UART recieve proc
; In - NONE
; Out - dataTemp - recieved byte
UART_Rcv:
	sbis UCSRA, RXC
	rjmp UART_Rcv
	in dataTemp, UDR
	ret

; UART send proc
; In - dataTemp - byte to be transmitted over UART
; Out - NONE
UART_Snd:
	sbis UCSRA, UDRE
	rjmp UART_Snd
CheckFinishPrevSnd:
	sbis UCSRA, TXC
	rjmp CheckFinishPrevSnd
	out UDR, dataTemp
	ret

