 .include"m2560def.inc"
 .def temp = r16
 .equ freq = 0xff
 .equ ConstCount = 60
 .equ ListPhaseShift = (ConstCount-2)*2/3
 .equ BAUD = 9600						; Скорость для UART в Бодах
 .equ XTAL = 8000000						; Частота в герцах
 .equ UBRR_value = (XTAL/(BAUD*16))-1	; Расчитывание значения для регистра UBRR
 .equ BUF_LEN =15						; Размер буфера для передачи данных
 .def CRC = r22                          ; Регистр для расчёта CRC
 .def msg = r21							; Регистр для чтения и записи сообщения (Буферный байт)
 .def run = r23
 .def lenght = r20

 // Название для пинов
 .equ	Phase1Pl = 7
 .equ	Phase1Mi = 6

 .equ	Phase2Pl = 5
 .equ	Phase2Mi = 3

  .equ	Phase3Pl = 1
 .equ	Phase3Mi = 0

 .equ	OCRnA = 1331
 .equ OCRnBHigh = 0x04
 .equ OCRnBLow = 0x8f





 ///////////////////////////////////////Фунцкия обработки данных фрейма///////////////////////////////
.macro Read_Func
	push r16
	in r16, SREG
	push r16
	push r17
	push r19
	//push r20
	push r21
	push r22


	cpi @0, 0x01					//Установка соединения
	breq connection_command			//

	cpi @0, 0x26					//Запрос настроек
	breq check_settings_command		//

	cpi @0, 0x15					//работа с текущими настройками
	breq start_command				//

	cpi @0, 0x30					//смнена настроек и начало/продолжение работы
	breq change_settings			//


	ldi r16,0
	sts in_count, r16	
	sts in_offset, r16
	rjmp Read_Func_end


//--------------------------Установка соединения-----------------------------------
	connection_command:
	ldi r17, 0x01
	SendSet r17
	rjmp Read_Func_end
//--------------------------Запрос настроек----------------------------------------
	check_settings_command:
	push XH
	push XL

	ldi XH, high(temperature_max)
	ldi XL, low(temperature_max)

	ld r19, X+
	//EERead 0x00
	//lds r19, temperature_max
	//EERead 0x01
	ld r17, X

	lds r21, volt_max

	SendDataSet r19,r17,r21

	pop XL
	pop XH
	rjmp Read_Func_end

//--------------------------Работа с текущими настройками--------------------------
	start_command:
	ldi run, 1
	rjmp Read_Func_end

//--------------------------смнена настроек и начало/продолжение работы------------
	change_settings:
	//пишем данные пока не закончились в еепром
	push XH
	push XL

	ldi XH, high(temperature_max)
	ldi XL, low(temperature_max)
	//lds r16, ADCH
	//lds r16, ADCL
	ld r17, Z+
	EEWrite 0x00, r17
	st X+, r17
	ld r17, Z+
	EEWrite 0x01, r17
	st X, r17
	//sts temperature_max, r17
	ld r17, Z+
	EEWrite 0x02, r17
	sts volt_max, r17
	//После записи отвечаем управляшке
	ldi r19, 0x15
	SendSet r19
	ldi run, 1

	pop XL
	pop XH

	rjmp Read_Func_end
//--------------------------------------------------------------------------------
	Read_Func_end:

	pop r22
	pop r21
	//pop r20
	pop r19
	pop r17
	pop r16
	out SREG, r16
	pop r16

.endm
;_______________________________________________________________________________________;


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//Дятел кривой, не забывай про структуру своего же протокола, даун конченый!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

////////////////////////////////////Функция сборки ответной команды///////////////////////
.macro SendSet
	push r16
	in r16, SREG
	push r16

	ldi r16, 0x81
	st Y+, r16
	inc lenght
	st Y+, @0
	inc lenght

	pop r16
	out SREG, r16
	pop r16
.endm
;_______________________________________________________________________________________;

/////////////////////////////////////Запись в энергонезависимую память///////////////////
.macro EEWrite  // @0 Адресс @1 Данные
	push r16
	in r16, SREG
	push r16
	cli
	EEPR_write:
	; Ждём пока можно будет записать
	sbic EECR, EEPE
	rjmp EEPR_write
	ldi r16, @0
	out EEARL, r16
	mov r16, @1
	out EEDR, r16
	sbi EECR,EEMPE
	sbi EECR,EEPE
	sei
	pop r16
	out SREG, r16
	pop r16
.endm

/////////////////////////////////////Запись в энергонезависимую память///////////////////
//Читается в temp
.macro EERead // @0 Адресс
	EEPR_read:
		sbic EECR, EEPE
	rjmp EEPR_read
	ldi temp, @0
	out EEARL, temp
	sbi EECR, EERE
	in temp, EEDR
.endm

////////////////////////////////////Функция сборки пакета с настройками//////////////////////
.macro SendDataSet
	push r16
	in r16, SREG
	push r16

	ldi r16, 0x42
	st Y+, r16
	inc lenght
	st Y+, @0
	inc lenght
	st Y+, @1
	inc lenght
	st Y+, @2
	inc lenght

	pop r16
	out SREG, r16
	pop r16
.endm
;_______________________________________________________________________________________;

////////////////////////////////////Функция сборки пакета с данными//////////////////////
.macro SendData
	push r16
	in r16, SREG
	push r16

	ldi r16, 0x42
	st Y+, r16
	inc lenght
	st Y+, @0
	inc lenght
	st Y, @1
	inc lenght

	pop r16
	out SREG, r16
	pop r16
.endm
;_______________________________________________________________________________________;

/////////////////////////////////////Передача одного байта////////////////////////////////
.macro USART_Trancieve
	push r16
	in r16, SREG
	push r16

trans_loop:
	lds r16, UCSR0A
	sbrs r16, UDRE0					; Проверяем, не занят ли буфер передачи USART
	rjmp trans_loop
	sts UDR0, @0	
					; Передаём байт+
	pop r16
	out SREG, r16
	pop r16
.endm
;_______________________________________________________________________________________;








 .dseg
	.org $200
	phase_register:	   .BYTE 1
	Constlist_L1shift: .BYTE 2
	Constlist_L2shift: .BYTE 2
	Constlist_L3shift: .BYTE 2 

	in_data: .BYTE BUF_LEN				; Буфер принятых данных
	in_offset: .BYTE 1					; Смещение в буфере при принятии
	in_count: .BYTE 1					; число принятых байтов
	in_mes_count: .BYTE 1				; число байт которые надо принять по информации из заголовка
	in_flag: .BYTE 1					; флаг для чтения пакета (0 при конце пакета)


	out_data: .BYTE BUF_LEN				; Буфер данных для отправки
	out_offset: .BYTE 1					; Смещение в буфере при передаче
	out_flag: .BYTE 1					; Флаг по окончании передачи пакета

	temperature: .BYTE 2				;температура с датчика
	temperature_max: .BYTE 2			;предельная температура
	volt_max: .BYTE 1
	
.cseg
 .org $0000
 jmp Reset

.org $0024
rjmp TIMER1_COMPB

.org $0026
rjmp TIMER1_COMPC

.org $0028 
rjmp TIMER1_OVF

; вектора адресов USART
.org $0032
jmp USART_RXC 

;вектор adc
.org $003A
rjmp ADC_Conv_Complete

.org $0042 
rjmp TIMER3_COMPB

.org $0044 
rjmp TIMER3_COMPC

.org $0046 
rjmp TIMER3_OVF

.org $0056
rjmp TIMER4_COMPB

.org $0058
rjmp TIMER4_COMPC

.org $005A
rjmp TIMER4_OVF






 Reset:

	cli
    ldi temp, high(ramend)
    out sph, temp
    ldi temp, low(ramend)
    out spl, temp

	ldi temp, 0xff
	out ddrc, temp
	ldi temp, 0x00
	out PORTC, temp

    ldi temp, 0xff
    out ddrg, temp
	out ddrb, temp
	cbi	ddrb, 5
    out ddre, temp
	sts ddrh, temp
	ldi temp, 0x00
	out ddrf, temp
	out PORTB, temp

	ldi temp, 0

	sts OCR1BH, temp
	sts OCR1BL, temp
	sts OCR1CH, temp
	sts OCR1CL, temp
	sts 0x8A, temp
	sts 0x8B, temp
	sts 0x8C, temp
	sts 0x8D, temp
	nop
	nop
	nop
	nop

	sts OCR3BH, temp
	sts OCR3BL, temp
	sts OCR3CH, temp
	sts OCR3CL, temp
	nop
	nop
	nop
	nop

	sts OCR4BH, temp
	sts OCR4BL, temp
	sts OCR4CH, temp
	sts OCR4CL, temp
	nop
	nop
	nop
	nop

	ldi		r17,	high(OCRnA)
	ldi		r16,	low(OCRnA)
	sts		OCR1AH,	r17
	sts		OCR1AL,	r16
	sts		OCR3AH,	r17
	sts		OCR3AL,	r16
	sts		OCR4AH,	r17
	sts		OCR4AL,	r16

	ldi		r18,	0x00
	sts		Constlist_L1shift,		r18
	sts		Constlist_L1shift+1,		r18

	ldi		r18,	high(ListPhaseShift)
	ldi		r17,	low(ListPhaseShift)
	sts		Constlist_L2shift,		r18
	sts		Constlist_L2shift+1,		r17
	sts		Constlist_L3shift,		r18
	sts		Constlist_L3shift+1,		r17

	rcall PWM_PFC_Init

	ldi temp, 0b00111000
	sts	phase_register, temp

	//Для юсарта
	ldi CRC,0x00

	rcall USART_Init

	ldi ZL, low(in_data)
    ldi ZH, high(in_data)

	ldi YH, high(out_data)
	ldi YL, low(out_data)

	//ldi temp, 0x00
	//out EEARH, temp

	EERead 0x00
	ldi XH, high(temperature_max)
	ldi XL, low(temperature_max)
	st X+, r16

	EERead 0x01
	st X, r16

	EERead 0x02
	sts volt_max, r16

	//Настройка ацп
	clr temp
	ldi temp, (1 << REFS1) | (1 <<  REFS0)// | (1 << ADLAR)
	//ldi	temp,	0b11000000	//В качестве порта ставим ADC8 для работы без деления
	sts	ADMUX,	temp		//

	ldi temp, 0b11001111	//Разрешаем прерывания и прочую лабуду
	sts ADCSRA, temp		////////////////
	
	clr temp
	sts in_offset, temp

	clr temp
	sts out_offset, temp
sei
//rjmp Main


 Main:
	lds temp, in_offset
	cpi temp, 0
	brne USARTMod_Communication
	cpi run, 102
	breq Main
	/////
	//rjmp Temp_check

	////Снова включаем АЦП
	//Настройка ацп
	//cpi temp, 17
	send_standart:
	cpi run, 14
	brlo main
	
	////////////////
	//lds r17, temperature
	////////////////
	ldi YH, high(out_data)
	ldi YL, low(out_data)
	////////////////
	//lds r17, temperature
	//lds r18, temperature
	ldi XH, high(temperature)
	ldi XL, low(temperature)
	ld r18, X+
	ld r17, X
	////////////////
	//ldi r17, 54
	SendData r18, r17
	ldi run, 1
	rjmp sending
 rjmp Main

 SHIM_OFF:
	clr temp
	sts TIMSK1, temp
	sts TIMSK3, temp
	sts TIMSK4, temp
	ldi run, 102
	ldi r17, 0xf1
	SendSet r17
	rjmp sending
rjmp Main

 //------------------------Обработка принятых данных------------------------------
USARTMod_Communication:

	lds temp, in_flag
	and temp, temp					//дожидаемся конца приёма пакета
	brne USARTMod_Communication		

	ldi ZH, high(in_data)			//берём ссылку на начало буфера с данными
	ldi ZL, low(in_data)			/////////////////////////////////

	lds temp, in_offset				//берём смещение в буфере, для подсчёта CRC, если temp станет равным 0, то мы посчитали CRC для фрейма
	dec temp						/////////////////////////////////

	crc_check:						//считаем и сравниваем CRC с данными из пакета
	ld msg, Z+						//если сошлось идём дальше, нет? не беда, дропаемся из функции
	add CRC, msg					//
	dec temp						//
	brne crc_check					//
									//
	ld msg, Z						//
	cpse CRC, msg					//
	rjmp USARTMod_Communication_end	////////////////////////////////////////

	ldi ZH, high(in_data)			//обновляем сслыки на начала буферов
	ldi ZL, low(in_data)			//
	ldi YH, high(out_data)			//
	ldi YL, low(out_data)			////////////////////////////////////

	ld msg, Z+						//изменяем сдвиг в соответствии с данными из фрейма
	ldi temp, 0x3f
	and msg, temp					//чтобы работать только с полем данных
	dec msg
	sts in_offset, msg				///////////////////////////////////////////////////

	ld msg, Z+						//считываем команду из первого байта поля данных
	Read_Func msg					//Вызываем функцию обработки поступивших данных и формирования ответа на них
	
	sending:
	//------------------------Отправка данных----------------------------------------
	ldi YH, high(out_data)
	ldi YL, low(out_data)
	clr CRC
	//in temp, lenght
	sts out_offset, lenght
	lds temp, out_offset

	cpi temp, 0
	breq Send_end

	ldi msg, 0xff
	USART_Trancieve msg

	send_loop:
	ld msg, Y+
	add CRC, msg

	USART_Trancieve msg
	dec temp
	brne send_loop

	mov msg, CRC
	USART_Trancieve msg

	ldi msg, 0xf0
	USART_Trancieve msg

//-----------------Очистка буферов после ответа на запрос-------------------------
USARTMod_Communication_end:
Send_end:
	clr temp
	sts in_count, temp
	sts in_offset, temp
	ldi YL, low(out_data)
    ldi YH, high(out_data)
	ldi ZL, low(in_data)
    ldi ZH, high(in_data)
	clr CRC
	ldi lenght, 0
rjmp main


Temp_check:
	push r17
	push r18
	push r20
	push r21
	ldi XH, high(temperature)
	ldi XL, low(temperature)
	ld r17, X+
	ld r18, X
	ldi XH, high(temperature_max)
	ldi XL, low(temperature_max)
	ld r21, X+
	ld r20, X
	cp r17, r20
	cpc r18, r21
	brlo PC + 2
	rjmp SHIM_OFF
	pop r21
	pop r20
	pop r18
	pop r17
rjmp send_standart


//---------------Настройка USART-----------------------------
USART_Init:  
	//1 Устанавливаем скорость передачи данных
	ldi temp, high(UBRR_Value) 
	sts UBRR0H, temp
	ldi temp, low(UBRR_Value)
	sts UBRR0L, temp
	//1 конец

	//2 настройка по даташиту
	clr temp
	sts UCSR0A, temp

	ldi temp, (1<<RXCIE0)|(0<<TXCIE0)|(1<<RXEN0)|(1<<TXEN0)
	sts UCSR0B, temp

	ldi r16, (0<<UMSEL0)|(3<<UCSZ00)
	sts UCSR0C,r16
	sei
reti


//------------------Прерывание по окончанию преобразования-------------------------//
ADC_Conv_Complete:					; Прерывание по окончанию преобразования
	push	XH
	push	XL
	push r16
	in r16, SREG
	push r16
	push r17

	ldi XH, high(temperature)
	ldi XL, low(temperature)

	lds r16, ADCL
	st X+, r16
	lds r16, ADCH
	st X, r16

	pop r17
	pop r16
	out SREG, r16
	pop r16
	pop XL
	pop XH
reti


//-------------Прерывание по приёму байта-----------------
USART_RXC:

		push r17
		in r17,SREG
		push r17
		push r20

		lds r17, UCSR0A
		sbrs r17, RXC0
		rjmp USART_RXC

		lds r20, in_flag					//проверяем флаг приёма пакета, 
		cpi r20, 0							//если 0 то это либо помеха либо начало пакета
		brne in_get_byte

		lds r17, UDR0
		cpi r17, 0xff						//если флаг 0 то проверяем байт на начало пакета
		brne RXC_end						//если не начало пакета то пропускаем

		inc r20								//если начало пакета то флаг в 1
		sts in_flag, r20					//и идём в конец чтобы не записывать лишнее в буфер
		rjmp RXC_end

	in_get_byte:							
		lds r17, UDR0					    //читаем байт и проверяем на конец пакета
		cpi r17, 0xf0						//если не конец пакета идём записывать
		brne in_set_byte

		ldi r20, 0							//если конец то флаг в 0 и выходим из функции
		sts in_flag, r20
		rjmp RXC_end

	in_set_byte:
		st Z+,r17							//записываем пришедший байт в память
		lds r20, in_offset					//увеличиваем смещение в буфере
		inc r20
		sts in_offset, r20					//инфа в буфере имеет вид (|HEADER| ||COMAND| |ELSE DATA|| |CRC|) без флагов начала и конца

	RXC_end:
		pop r20
		pop r17
		out SREG, r17
		pop r17

reti




//Дальше идёт часть кода с ШИМОМ
 constlist:.DW  128,  139,  250,  277,  345,  412,  478,  542,  605,  667, 726,  784,  839,  892,  943,  991,  1036, 1078, 1118, 1154, 1188, 1218, 1244, 1265, 1270, 1275, 1280, 1285, 1290, 1297
 //Масси указателей на массивы
 //constlist2: .DW constlist


 PWM_PFC_Init:
	
	ldi temp, 0b00101001
    sts TCCR1A, temp
	sts TCCR3A, temp
	sts TCCR4A, temp
    ldi temp, 0b00010001
    sts TCCR1B, temp
	sts TCCR3B, temp
	sts TCCR4B, temp

    ldi temp, (1<< TOIE1)|(1 <<OCIE1B)|(1 <<OCIE1C)
    sts TIMSK1, temp
	ldi temp, (1<< TOIE3)|(1 <<OCIE3B)|(1 <<OCIE3C)
    sts TIMSK3, temp
	ldi temp, (1<< TOIE4)|(1 <<OCIE4B)|(1 <<OCIE4C)
    sts TIMSK4, temp
ret


TIMER1_COMPC:
reti

// прерывание переполнения таймера (опустошение)
TIMER1_OVF:
	push	r16
	in		r16,	SREG
	push	r16
	push	r17
	push	XH
	push	XL
	push	ZH
	push	ZL

	;sei

	//	phase register отвечает за то, какая четвердь синусоиды сейчас идёт у каждой фазы
	//	Он разбит на 3 блока по 2 бита, где 0бит - восходящая или нисхожящая четверть,
	//	а 1бит - положительная или отрицательная полуволна синусоиды
	lds		r16,	phase_register
	//	сохраняем его в стек
	push	r16
	lds		XH,	Constlist_L1shift
	lds		XL,	Constlist_L1shift+1
	ldi		ZH,	high(2*constlist)
	ldi		ZL,	low(2*constlist)
	
	//	Сбрасываем не нужные биты и сравниваем
	cbr		r16,	0b11111110
	cpi		r16,	0b00000000
		breq	TIMER1_OVF_rising
	cpi		r16,	0b00000001
		breq	TIMER1_OVF_falling

		;-------Восходящая четверть------;
		TIMER1_OVF_rising:	

			//	Проверяем не равен ли наш сдвиг размерности массива
			cpi		XL,		low(ConstCount)
				brne	TIMER1_OVF_rising_1
			cpi		XH,		high(ConstCount)
				brne	TIMER1_OVF_rising_1
				// Если да, то переходим на нисходящую четверть, выставляя соответствующий бит в регистре
				pop		r16
				sbr		r16,	0b00000001
				sts		phase_register,		r16
				push	r16
				rjmp	TIMER1_OVF_falling_1

			TIMER1_OVF_rising_1:
			//	Если мы не дошли до конца массива,
			//	Сдвигаем указатель на массив на X
			ADD		ZL,		XL
			ADC		ZH,		XH

			//	Считываем необходимое значение из массива
			LPM     r16,	Z+			//
			LPM     r17,	Z
			
			//	Заносим полученное значение в регистр сравнения
			sts		OCR1BH, r17
			sts		OCR1BL, r16
			//	Увеличиваем сдвиг
			adiw	X,	2
			sts		Constlist_L1shift,		XH
			sts		Constlist_L1shift+1,	XL

			pop		r16
			rjmp	TIMER1_OVF_End

		;-------Нисходящая четверть------;
		TIMER1_OVF_falling:

			//	Проверяем не равен ли наш сдвиг 0
			cpi		XL,		0
				brne	TIMER1_OVF_falling_1
			cpi		XH,		0
				brne	TIMER1_OVF_falling_1
				//	Если да, то преходим на восходящую четверть и меняем полуволну на противоположную, 
				//	выставляя соответствующие биты в регистре
				pop		r16
				cbr		r16,	0b00000001
				sbrc	r16,	1
					rjmp	PC+3
				sbr		r16,	0b00000010
				rjmp	PC+2
				cbr		r16,	0b00000010

				sts		phase_register,		r16
				push	r16

				//Тема для отправки пакета
				cpi run, 0
				breq step_flag
				inc run
				step_flag:

				rjmp	TIMER1_OVF_rising_1
			TIMER1_OVF_falling_1:
			//	Если мы не ушли за массив,
			//	Сдвигаем указатель на массив на X
			ADD		ZL,		XL
			ADC		ZH,		XH
			//	Считываем необходимое значение из массива
			sbiw	Z,		1
			LPM     r17,	Z
			sbiw	Z,		1
			LPM     r16,	Z
			//	Заносим полученное значение в регистр сравнения
			sts		OCR1BH, r17
			sts		OCR1BL, r16
			//	Уменьшаем сдвиг
			sbiw	X,	2
			sts		Constlist_L1shift,		XH
			sts		Constlist_L1shift+1,	XL
			pop		r16
			rjmp	TIMER1_OVF_End
			
	TIMER1_OVF_End:
	cli
	pop		ZL
	pop		ZH
	pop		XL
	pop		XH
	pop		r17
	pop		r16
	out		SREG,	r16
	pop		r16
reti

//Прерывание по сравнению 
TIMER1_COMPB:
	push	r16
	in		r16,	SREG
	push	r16
	push	r17
	push	XH
	push	XL

	;sei

	//считываем начальное значение счётчика, 
	//чтобы определить низходящий или восходящий таймер   
	lds		r16,	0x84
	lds		r17,	0x85
	mov		XL,		r16
	mov		XH,		r17

	// определяем какая полуволна положительная или отрицательная
	lds		r16,	phase_register
	cbr		r16,	0b11111101
	cpi		r16,	0b00000000
		breq	TIMER1_COMPB_pozitive
	cpi		r16,	0b00000010
		breq	TIMER1_COMPB_negative
	rjmp	TIMER1_COMPB_End
		TIMER1_COMPB_pozitive:
			// снова считываем значение таймера счётчика 
			// для сравнения
			

			cbi		PORTC,	Phase1Mi
			lds		r16,	0x84
			lds		r17,	0x85
			sub		r16,	XL
			sbc		r17,	XH
			brmi	PC+3
				cbi		PORTC,	Phase1Pl
				rjmp	PC+2
			sbi		PORTC,	Phase1Pl

			rjmp	TIMER1_COMPB_End

		TIMER1_COMPB_negative:
			// считываем значение таймера для сравнения
			/*cpi run, 1
			brne step_flag
			inc run
			inc run
			step_flag:*/
			cbi		PORTC,	Phase1Pl
			lds		r16,	0x84
			lds		r17,	0x85
			sub		r16,	XL
			sbc		r17,	XH
			brmi	PC+3
				cbi		PORTC,	Phase1Mi
				rjmp	PC+2
			sbi		PORTC,	Phase1Mi
	TIMER1_COMPB_End:
	cli
	pop		XL
	pop		XH
	pop		r17
	pop		r16
	out		SREG,	r16
	pop		r16
reti



TIMER3_COMPB:
	push	r16
	in		r16,	SREG
	push	r16
	push	r17
	push	XH
	push	XL
	lds		r16,	0x84
	lds		r17,	0x85
	mov		XL,		r16
	mov		XH,		r17
	lds		r16,	phase_register
	cbr		r16,	0b11110111
	cpi		r16,	0b00000000
		breq	TIMER3_COMPB_pozitive
	cpi		r16,	0b00001000
		breq	TIMER3_COMPB_negative
	rjmp	TIMER3_COMPB_End
		TIMER3_COMPB_pozitive:
			cbi		PORTC,	Phase2Mi
			lds		r16,	0x84
			lds		r17,	0x85
			sub		r16,	XL
			sbc		r17,	XH
			brmi	PC+3
				cbi		PORTC,	Phase2Pl
				rjmp	PC+2
			sbi		PORTC,	Phase2Pl

			rjmp	TIMER1_COMPB_End

		TIMER3_COMPB_negative:
			cbi		PORTC,	Phase2Pl
			;cbi		PORTC,	7
			lds		r16,	0x84
			lds		r17,	0x85
			sub		r16,	XL
			sbc		r17,	XH
			brmi	PC+3
				cbi		PORTC,	Phase2Mi
				rjmp	PC+2
			sbi		PORTC,	Phase2Mi
	TIMER3_COMPB_End:
	cli
	pop		XL
	pop		XH
	pop		r17
	pop		r16
	out		SREG,	r16
	pop		r16
reti



TIMER3_COMPC:
reti


TIMER3_OVF:
	push	r16
	in		r16,	SREG
	push	r16

	push	r17
	push	XH
	push	XL
	push	ZH
	push	ZL
	lds		r16,	phase_register
	push	r16

	lds		XH,	Constlist_L2shift
	lds		XL,	Constlist_L2shift+1
	ldi		ZH,		high(2*constlist)
	ldi		ZL,		low(2*constlist)

	cbr		r16,	0b11111011
	cpi		r16,	0b00000000
		breq	TIMER3_OVF_rising
	cpi		r16,	0b00000100
		breq	TIMER3_OVF_falling

		TIMER3_OVF_rising:	
			cpi		XL,		low(ConstCount)
				brne	TIMER3_OVF_rising_1
			cpi		XH,		high(ConstCount)
				brne	TIMER3_OVF_rising_1
				pop		r16
				;sbrc	r16,	1
				;	cbr		r16,	1
				sbr		r16,	0b00000100
				sts		phase_register,		r16
				push	r16;;;;;;
				;sbiw	X,	2
				rjmp	TIMER3_OVF_falling_1;;TIMER1_OVF_End

			TIMER3_OVF_rising_1:
			ADD		ZL,		XL
			ADC		ZH,		XH
			LPM     r16,	Z+			//
			LPM     r17,	Z
			sts		OCR3BH, r17
			sts		OCR3BL, r16
			adiw	X,	2
			sts		Constlist_L2shift,		XH
			sts		Constlist_L2shift+1,	XL
			pop		r16
			rjmp	TIMER3_OVF_End

		;*/
		TIMER3_OVF_falling:
			;/*
			cpi		XL,		0
				brne	TIMER3_OVF_falling_1
			cpi		XH,		0
				brne	TIMER3_OVF_falling_1
				pop		r16
				cbr		r16,	0b00000100
				;cbr		r16,	0b11111101
				sbrc	r16,	3
					rjmp	PC+3
				sbr		r16,	0b00001000
				rjmp	PC+2
				cbr		r16,	0b00001000

				sts		phase_register,		r16
				;push	r17
				push	r16;;;;;;
				rjmp	TIMER3_OVF_rising_1;TIMER1_OVF_End
				;
			TIMER3_OVF_falling_1:
			ADD		ZL,		XL
			ADC		ZH,		XH
			sbiw	Z,		1
			LPM     r17,	Z			//
			sbiw	Z,		1
			LPM     r16,	Z
			sts		OCR3BH, r17
			sts		OCR3BL, r16
			sbiw	X,	2
			sts		Constlist_L2shift,		XH
			sts		Constlist_L2shift+1,	XL
			pop		r16
			rjmp	TIMER3_OVF_End;TIMER1_OVF_rising;
			
	TIMER3_OVF_End:
	cli
	pop		ZL
	pop		ZH
	pop		XL
	pop		XH
	pop		r17
	pop		r16
	out		SREG,	r16
	pop		r16
reti



TIMER4_COMPB:
	push	r16
	in		r16,	SREG
	push	r16
	push	r17
	push	XH
	push	XL
	lds		r16,	0x84
	lds		r17,	0x85
	mov		XL,		r16
	mov		XH,		r17
	lds		r16,	phase_register
	cbr		r16,	0b11011111
	cpi		r16,	0b00000000
		breq	TIMER4_COMPB_pozitive
	cpi		r16,	0b00100000
		breq	TIMER4_COMPB_negative
	rjmp	TIMER4_COMPB_End
		TIMER4_COMPB_pozitive:
			cbi		PORTC,	Phase3Mi
			;cbi		PORTC,	6
			lds		r16,	0x84
			lds		r17,	0x85
			sub		r16,	XL
			sbc		r17,	XH
			brmi	PC+3
			;cp		r16,	r17
			;brsh	PC+3
				cbi		PORTC,	Phase3Pl
				rjmp	PC+2
			sbi		PORTC,	Phase3Pl
			rjmp	TIMER4_COMPB_End

		TIMER4_COMPB_negative:
			cbi		PORTC,	Phase3Pl
			;cbi		PORTC,	7
			lds		r16,	0x84
			lds		r17,	0x85
			sub		r16,	XL
			sbc		r17,	XH
			brmi	PC+3
				cbi		PORTC,	Phase3Mi
				rjmp	PC+2
			sbi		PORTC,	Phase3Mi
	TIMER4_COMPB_End:
	cli
	pop		XL
	pop		XH
	pop		r17
	pop		r16
	out		SREG,	r16
	pop		r16
reti


TIMER4_COMPC:
reti


TIMER4_OVF:
	push	r16
	in		r16,	SREG
	push	r16

	push	r17
	
	;push	r18
	push	XH
	push	XL
	push	ZH
	push	ZL

	;sei

	lds		r16,	phase_register
	push	r16

	lds		XH,		Constlist_L3shift
	lds		XL,		Constlist_L3shift+1
	ldi		ZH,		high(2*constlist)
	ldi		ZL,		low(2*constlist)

	cbr		r16,	0b11101111
	cpi		r16,	0b00000000
		breq	TIMER4_OVF_rising
	cpi		r16,	0b00010000
		breq	TIMER4_OVF_falling

		TIMER4_OVF_rising:	
			cpi		XL,		low(ConstCount)
				brne	TIMER4_OVF_rising_1
			cpi		XH,		high(ConstCount)
				brne	TIMER4_OVF_rising_1
				pop		r16
				;sbrc	r16,	1
				;	cbr		r16,	1
				sbr		r16,	0b00010000
				sts		phase_register,		r16
				push	r16;;;;;;
				;sbiw	X,	2
				rjmp	TIMER4_OVF_falling_1;;TIMER1_OVF_End
			TIMER4_OVF_rising_1:
			ADD		ZL,		XL
			ADC		ZH,		XH

			
			LPM     r16,	Z+			//
			LPM     r17,	Z
			
			sts		OCR4BH, r17
			sts		OCR4BL, r16
			adiw	X,	2
			sts		Constlist_L3shift,		XH
			sts		Constlist_L3shift+1,	XL

			pop		r16
			rjmp	TIMER4_OVF_End
		TIMER4_OVF_falling:	
			cpi		XL,		0
				brne	TIMER4_OVF_falling_1
			cpi		XH,		0
				brne	TIMER4_OVF_falling_1
				pop		r16
				cbr		r16,	0b00010000
				;cbr		r16,	0b11111101
				sbrc	r16,	3
					rjmp	PC+3
				sbr		r16,	0b00100000
				rjmp	PC+2
				cbr		r16,	0b00100000

				sts		phase_register,		r16
				;push	r17
				push	r16;;;;;;
				rjmp	TIMER4_OVF_rising_1;TIMER1_OVF_End
				;
			TIMER4_OVF_falling_1:

			ADD		ZL,		XL
			ADC		ZH,		XH
			sbiw	Z,		1
			LPM     r17,	Z			//
			sbiw	Z,		1
			LPM     r16,	Z
			
			sts		OCR4BH, r17
			sts		OCR4BL, r16
			sbiw	X,	2
			sts		Constlist_L3shift,		XH
			sts		Constlist_L3shift+1,	XL
			pop		r16

			rjmp	TIMER4_OVF_End;TIMER1_OVF_rising;		
	TIMER4_OVF_End:
	cli
	pop		ZL
	pop		ZH
	pop		XL
	pop		XH
	pop		r17
	pop		r16
	out		SREG,	r16
	pop		r16
reti
