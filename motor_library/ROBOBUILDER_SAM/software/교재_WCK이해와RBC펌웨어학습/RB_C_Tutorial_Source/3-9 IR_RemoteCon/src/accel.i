// CodeVisionAVR C Compiler
// (C) 1998-2004 Pavel Haiduc, HP InfoTech S.R.L.
// I/O registers definitions for the ATmega128
#pragma used+
sfrb PINF=0;
sfrb PINE=1;
sfrb DDRE=2;
sfrb PORTE=3;
sfrb ADCL=4;
sfrb ADCH=5;
sfrw ADCW=4;      // 16 bit access
sfrb ADCSRA=6;
sfrb ADMUX=7;
sfrb ACSR=8;
sfrb UBRR0L=9;
sfrb UCSR0B=0xa;
sfrb UCSR0A=0xb;
sfrb UDR0=0xc;
sfrb SPCR=0xd;
sfrb SPSR=0xe;
sfrb SPDR=0xf;
sfrb PIND=0x10;
sfrb DDRD=0x11;
sfrb PORTD=0x12;
sfrb PINC=0x13;
sfrb DDRC=0x14;
sfrb PORTC=0x15;
sfrb PINB=0x16;
sfrb DDRB=0x17;
sfrb PORTB=0x18;
sfrb PINA=0x19;
sfrb DDRA=0x1a;
sfrb PORTA=0x1b;
sfrb EECR=0x1c;
sfrb EEDR=0x1d;
sfrb EEARL=0x1e;
sfrb EEARH=0x1f;
sfrw EEAR=0x1e;   // 16 bit access
sfrb SFIOR=0x20;
sfrb WDTCR=0x21;
sfrb OCDR=0x22;
sfrb OCR2=0x23;
sfrb TCNT2=0x24;
sfrb TCCR2=0x25;
sfrb ICR1L=0x26;
sfrb ICR1H=0x27;
sfrw ICR1=0x26;   // 16 bit access
sfrb OCR1BL=0x28;
sfrb OCR1BH=0x29;
sfrw OCR1B=0x28;  // 16 bit access
sfrb OCR1AL=0x2a;
sfrb OCR1AH=0x2b;
sfrw OCR1A=0x2a;  // 16 bit access
sfrb TCNT1L=0x2c;
sfrb TCNT1H=0x2d;
sfrw TCNT1=0x2c;  // 16 bit access
sfrb TCCR1B=0x2e;
sfrb TCCR1A=0x2f;
sfrb ASSR=0x30;
sfrb OCR0=0x31;
sfrb TCNT0=0x32;
sfrb TCCR0=0x33;
sfrb MCUCSR=0x34;
sfrb MCUCR=0x35;
sfrb TIFR=0x36;
sfrb TIMSK=0x37;
sfrb EIFR=0x38;
sfrb EIMSK=0x39;
sfrb EICRB=0x3a;
sfrb RAMPZ=0x3b;
sfrb XDIV=0x3c;
sfrb SPL=0x3d;
sfrb SPH=0x3e;
sfrb SREG=0x3f;
#pragma used-
// Interrupt vectors definitions
//==============================================================================
// 하드웨어 의존적인 정의들
//==============================================================================
// 배터리 기준 전압 -----------------------------------------------
//#define	U_T_OF_POWER		12000		// 리튬폴리머 11.1V 배터리용
//#define	M_T_OF_POWER		9800
//#define	L_T_OF_POWER		9700
//==============================================================================
//						플랫폼(로봇 형태) 관련
//==============================================================================
//==============================================================================
//						버튼 관련
//==============================================================================
//==============================================================================
//						에러 코드(F_ERR_CODE에 저장)
//==============================================================================
//==============================================================================
//						UART 통신 관련
//==============================================================================
//==============================================================================
//						A/D 관련
//==============================================================================
//==============================================================================
//						액션/모션 관련
//==============================================================================
//==============================================================================
//						IR 리모컨 관련
//==============================================================================
//==============================================================================
// DATA TYPE
//==============================================================================
//==============================================================================
// BIT SET
//==============================================================================
//==============================================================================
// BIT CLEAR
//==============================================================================
//==============================================================================
// BIT CHECK
//==============================================================================
//==============================================================================
// BIT MASK
//==============================================================================
void AccStart(void);
void AccStop(void);
void AccAckRead(void);
void AccAckWrite(void);
void AccNotAckWrite(void);
char AccByteRead(void);
void AccByteWrite(unsigned char bData);
void AccRead(void);
char AccReadData(unsigned char addr);
void Acc_init(void);
void AccGetData(void);
//==============================================================//
// Start
//==============================================================//
void AccStart(void)
{
(DDRE |= 0x20);
(DDRE |= 0x10);
	if(1) (PORTE |= 0x20);else (PORTE &= 0xDF);
	if(1) (PORTE |= 0x10);else (PORTE &= 0xEF);
	#asm("nop");
	#asm("nop");
	if(0) (PORTE |= 0x20);else (PORTE &= 0xDF);
	#asm("nop");
	#asm("nop");
	if(0) (PORTE |= 0x10);else (PORTE &= 0xEF);
	#asm("nop");
	#asm("nop");
}
//==============================================================//
// Stop
//==============================================================//
void AccStop(void)
{
(DDRE |= 0x20);
(DDRE |= 0x10);
	if(0) (PORTE |= 0x20);else (PORTE &= 0xDF);
	if(1) (PORTE |= 0x10);else (PORTE &= 0xEF);
	#asm("nop");
	#asm("nop");
	if(1) (PORTE |= 0x20);else (PORTE &= 0xDF);
	#asm("nop");
	#asm("nop");
(DDRE &= 0xDF);
(DDRE &= 0xEF);
}
//==============================================================//
//
//==============================================================//
void AccByteWrite(unsigned char bData)
{
	unsigned char	i;
	unsigned char	bTmp;
(DDRE |= 0x20);
	for(i=0; i<8; i++){
		bTmp = (bData & 0x80);
    	if(bTmp){
			if(1) (PORTE |= 0x20);else (PORTE &= 0xDF);
		}else{
			if(0) (PORTE |= 0x20);else (PORTE &= 0xDF);
		}
		#asm("nop");
		#asm("nop");
		if(1) (PORTE |= 0x10);else (PORTE &= 0xEF);;
		#asm("nop");
		#asm("nop");
		#asm("nop");
		#asm("nop");
		if(0) (PORTE |= 0x10);else (PORTE &= 0xEF);
		#asm("nop");
		#asm("nop");
		bData =	bData << 1;
	}
}
//==============================================================//
//
//==============================================================//
char AccByteRead(void)
{
	unsigned char	i;
	char	bTmp = 0;
(DDRE &= 0xDF);
	for(i = 0; i < 8;	i++){
		bTmp = bTmp << 1;
		#asm("nop");
		#asm("nop");
		#asm("nop");
		#asm("nop");
		if(1) (PORTE |= 0x10);else (PORTE &= 0xEF);
		#asm("nop");
		#asm("nop");
		if((PINE & 0x20))	bTmp |= 0x01;
		#asm("nop");
		#asm("nop");
		if(0) (PORTE |= 0x10);else (PORTE &= 0xEF);
	}
(DDRE |= 0x20);
	return	bTmp;
}
//==============================================================//
//
//==============================================================//
void AccAckRead(void)
{
(DDRE &= 0xDF);
	#asm("nop");
	#asm("nop");
	if(1) (PORTE |= 0x20);else (PORTE &= 0xDF);
	#asm("nop");
	#asm("nop");
	if(1) (PORTE |= 0x10);else (PORTE &= 0xEF);
	#asm("nop");
	#asm("nop");
	if(0) (PORTE |= 0x10);else (PORTE &= 0xEF);
	#asm("nop");
	#asm("nop");
(DDRE |= 0x20);
	#asm("nop");
	#asm("nop");
}
//==============================================================//
//
//==============================================================//
void AccAckWrite(void)
{
(DDRE |= 0x20);
	#asm("nop");
	#asm("nop");
	if(0) (PORTE |= 0x20);else (PORTE &= 0xDF);
	#asm("nop");
	#asm("nop");
	if(1) (PORTE |= 0x10);else (PORTE &= 0xEF);
	#asm("nop");
	#asm("nop");
	if(0) (PORTE |= 0x10);else (PORTE &= 0xEF);
	#asm("nop");
	#asm("nop");
	if(1) (PORTE |= 0x20);else (PORTE &= 0xDF);
	#asm("nop");
	#asm("nop");
}
//==============================================================//
//
//==============================================================//
void AccNotAckWrite(void)
{
(DDRE |= 0x20);
	#asm("nop");
	#asm("nop");
	if(1) (PORTE |= 0x20);else (PORTE &= 0xDF);
	#asm("nop");
	#asm("nop");
	if(1) (PORTE |= 0x10);else (PORTE &= 0xEF);
	#asm("nop");
	#asm("nop");
	if(0) (PORTE |= 0x10);else (PORTE &= 0xEF);
	#asm("nop");
	#asm("nop");
}
//==============================================================//
//==============================================================//
void Acc_init(void)
{
	AccStart();
	AccByteWrite(0x70);
	AccAckRead();
	AccByteWrite(0x14);
	AccAckRead();
	AccByteWrite(0x03);
	AccAckRead();
	AccStop();
}
//==============================================================//
//==============================================================//
void AccGetData(void)
{
	signed char	bTmp = 0;
	AccStart();
	AccByteWrite(0x70);
	AccAckRead();
	AccByteWrite(0x02);
	AccAckRead();
	AccStop();
	#asm("nop");
	#asm("nop");
	#asm("nop");
	#asm("nop");
	AccStart();
	AccByteWrite(0x71);
	AccAckRead();
	bTmp = AccByteRead();
	AccAckWrite();
	bTmp = AccByteRead();
	AccAckWrite();
	gAccX = bTmp;
	bTmp = AccByteRead();
	AccAckWrite();
	bTmp = AccByteRead();
	AccAckWrite();
	gAccY = bTmp;
	bTmp = AccByteRead();
	AccAckWrite();
	bTmp = AccByteRead();
	AccNotAckWrite();
	gAccZ = bTmp;
	AccStop();
}
