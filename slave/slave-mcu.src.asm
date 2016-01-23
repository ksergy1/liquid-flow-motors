.include "tn2313def.inc"

; fosc = 8MHz
; Fuse-bits
;	High byte: 11011111
;	Low byte : 11100100

.equ SpiPort = PORTB
.equ DDR_SPI = DDRB
.equ SPI_PINS = PINB
.equ SpiSCK = PORTB7
.equ SpiDO = PORTB6
.equ SpiDI = PORTB5

.equ ControlPort = PORTB
.equ StDir = PORTB0
.equ StClk = PORTB1
.equ StSize = PORTB2

.equ DirBit = 7
.equ SzBit = 6
;.equ JustTicks = 0b0011111


.def temp = r17
.def dataTemp = r16
.def command = r18

.equ StatusPort = PORTD
.equ StatusSyncMtr = PORTD4
.equ StatusDataOut = PORTD5
.equ StatusDataIn = PORTD6
.equ SelectorPin = PORTD3
.equ SyncPin = PORTD2

; векторы прерываний
	rjmp MAIN;
	rjmp SyncMotor			; External Interrupt0 Handler
	rjmp CrystalSelected	; External Interrupt1 Handler
	rjmp NILL_INT			; Timer1 Capture Handler
	rjmp NILL_INT			; Timer1 CompareA Handler
	rjmp NILL_INT			; Timer1 Overflow Handler
	rjmp NILL_INT			; Timer0 Overflow Handler
	rjmp NILL_INT			; USART0 RX Complete Handler
	rjmp NILL_INT			; USART0,UDR Empty Handler
	rjmp NILL_INT			; USART0 TX Complete Handler
	rjmp NILL_INT			; Analog Comparator Handler
	rjmp NILL_INT			; Pin Change Interrupt
	rjmp NILL_INT			; Timer1 Compare B Handler
	rjmp NILL_INT			; Timer0 Compare A Handler
	rjmp NILL_INT			; Timer0 Compare B Handler
	rjmp NILL_INT			; USI Start Handler
	rjmp NILL_INT			; USI Overflow Handler
	rjmp NILL_INT			; EEPROM Ready Handler
	rjmp NILL_INT			; Watchdog Overflow Handler

; инициализация
MAIN:
	ldi temp, Low(RAMEND);
	out SPL, temp;
; инициализация SPI (fck/2) и портов ввода-вывода (SPI - без прерываний)
	ldi temp, (1<<SpiDO)|(1<<StDir)|(1<<StClk)|(1<<StSize)
	out DDR_SPI, temp;
	ldi temp, (1<<StatusSyncMtr)|(1<<StatusDataOut)|(1<<StatusDataIn);
	out StatusPort, temp;
	ldi temp, (1<<USIWM0)|(1<<USICS1);
	out USICR, temp;
; включение spi - по прерыванию
; сначала разрешим прерывания внешние int0 и int1
	; включение Crystal select (int1) - по срезу, SyncMotor (int0) - по росту
	; (+отключение sleepmode, sleepmode = power-down)
	ldi temp, (1<<SM1)|(1<<SM0)|(1<<ISC11)|(1<<ISC00)|(1<<ISC01);
	out MCUCR, temp;
	; разрешение external-INT0 и external-INT1 и отключение PinChange прерываний
	ldi temp, (1<<INT0)|(1<<INT1);
	out GIMSK, temp;
	; Отключение pin change прерываний (по одному, в довесок)
	clr temp;
; ATtiny2313
	out PCMSK, temp;
	; Инициализация контроллера ШД
	cbi ControlPort, StDir;
	cbi ControlPort, StClk;
	cbi ControlPort, StSize;
	; разрешаем прерывания...
	sei;

	; Выполняем цикл
  LargeHadronCollider:
	rjmp LargeHadronCollider;

; пустое прерывание
NILL_INT:
	reti;

; Прерывание на синхронизацию двигателей (выполнение команды)
SyncMotor:
	sbi StatusPort, StatusSyncMtr;
	; выбираем направление
	sbrc command, dirBit;
	sbi ControlPort, StDir;
	; выбираем шаг
	sbrc command, szBit;
	sbi ControlPort, StSize;

	; выделяем чистую команду
	cbr command, (1<<dirBit)|(1<<szBit);

	; выполняем команду
  command_cycle:
		sbi ControlPort, StClk;
		nop;
		nop;
		nop;
		cbi ControlPort, StClk;
		; формируем необходимую задержку
		nop;
		nop;
		dec command;
	brne command_cycle;

	cbi ControlPort, StDir;
	cbi ControlPort, StClk;
	cbi ControlPort, StSize;
	cbi StatusPort, StatusSyncMtr;
	reti;

; Прерывание на выбор кристала (прием команды от ведущего)
CrystalSelected:
	clr dataTemp;
	out USIDR, dataTemp;
	sbi USISR, USIOIF;
	sbi StatusPort, StatusDataIn;
  xfer_complete_check_atCS:
	sbic USISR, USIOIF;
	rjmp xfer_complete_check_atCS;
	in dataTemp, USIDR;
	mov command, dataTemp;
	cbi StatusPort, StatusDataIn;
	reti;

; функция приеме-передачи данных по SPI
; IN - dataTemp - byte to send
; OUT - dataTemp - recieved byte
SPI_xfer:
	out USIDR, dataTemp;
	sbi USISR, USIOIF;
  xfer_complete_check:
	sbic USISR, USIOIF;
	rjmp xfer_complete_check;
	in dataTemp, USIDR;
	ret;

