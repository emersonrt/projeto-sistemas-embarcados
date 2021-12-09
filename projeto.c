/*--------------------------------------------------|
| UNISC -  Engenharia de Computacao                 |
|---------------------------------------------------|
|   NOME      :   Display de Cristal L?quido        |
|   VERSAO    :   1.0                               |
|   COMPILADOR:   XC08 V2.32                        |
|   DATA      :   Outubro 01, 2021                  |
|   AUTOR     :   Emerson R. Teixeira               |
|   PLACA     :   Simulacao (PICSimLab) PICGENIUS   |
|   uC        :   PIC18F4550                        |
|---------------------------------------------------|
|---------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <p18cxxx.h>


#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

  

//Definicao dos pinos de entrada e saida
//7 segmentos PORTD
#define D7SEG_M PORTAbits.RA2//PORTBbits.RB7
#define D7SEG_C PORTAbits.RA3//PORTBbits.RB6
#define D7SEG_D PORTAbits.RA4//PORTBbits.RB5
#define D7SEG_U PORTAbits.RA5//PORTBbits.RB4
#define BOTAO   PORTAbits.RB1

//LCD PORTD
#define RS PORTEbits.RE2
#define EN PORTEbits.RE1

//I/O
#define COOLER PORTCbits.RC2
#define HEATER PORTCbits.RC5

//Botões RB0 e RB1
#define _RB0 0x10
#define _RB1 0x11

//Definicao dos comandos do LCD
#define linha1_ini 0x80
#define linha1_fim 0x8a
#define linha2_ini 0xc0
#define linha2_fim 0xca
#define linha3 0x90           // Posicao dos caracteres no LCD
#define linha4 0xd0
#define limpa_lcd 0x01        // |0x80|0x81|0x82|0x83|0x84| ... |0x8f|
#define cursor_piscando 0x0f  // +----+----+----+----+----+-----+----+
#define cursor_off 0x0c       // |0xc0|0xc1|0xc2|0xc3|0xc4| ... |0xcf|
#define cursor_on 0x0e        // +----+----+----+----+----+-----+----+
#define rotaciona_direita 0x1c

//Declaracao das funcoes
void lcd_cmd(unsigned char dado);
void lcd_escreve(unsigned char dado);
void lcd_puts(char *s);
void lcd_init(void);
void delay(void);
void delay_ms(unsigned int a);
void converte_LCD(int linha, int valor);

//Funcoes do PWM
void PWM1_Init(void);
void PWM1_Start(void);
void PWM1_Set_Duty(unsigned char d);

//Funções do ADC
void ConvertADC(void);
unsigned int ReadADC(void);
void adc_init(void);

//Funções Memória EEPROM
unsigned char e2prom_read(unsigned char endereco);
void e2prom_write(unsigned char endereco,unsigned char dado);

//Funções dígitos Memória
int lerInteiroMemoria4Digitos();
void gravarInteiroMemoria4Digitos();

//Função para leitura dos botões
unsigned char digitalRead(unsigned char pin);


int tempAtual;
int tempIdeal;
int velocidadeCooler = 10;

//-------------------------------------------------------
//		Funcao Principal
//-------------------------------------------------------
void main(void){

    TRISA=0x04; //entrada -> sensor de temp
    
    //saídas
    TRISC=0x00; 
    TRISD=0x00;
    TRISE=0x00;
            
    lcd_init(); //inicia visor
    adc_init(); //inicia conversor
    PWM1_Init(); //inicia pwm
    PWM1_Start();
    
    lcd_cmd(linha1_ini);
    lcd_puts("T. atual:");
    lcd_cmd(linha2_ini);
    lcd_puts("T. ideal: ");
    converte_LCD(linha2_fim, tempIdeal);
    lcd_cmd(linha3);
    lcd_puts("RB1 para + temp");
    lcd_cmd(linha4);
    lcd_puts("RB0 para - temp");
    
    while(1) {
        
        ConvertADC();                       //inicia conversão
        delay_ms(1);                        //espera fim da conversao
        tempAtual = ReadADC();              //le o resultado
        tempAtual = (tempAtual*10)/2.040;   //calculo do sensor de temperatura
        converte_LCD(linha1_fim, tempAtual);
        
        
        //lógica para controlar temperatura
        if (tempAtual > tempIdeal) {
            if (velocidadeCooler < 255) { //controle para não zerar a velocidade
                velocidadeCooler++;
            }
            HEATER=0;
        } else if (velocidadeCooler > 1){
            velocidadeCooler--;
            HEATER=1;
        }
        PWM1_Set_Duty(velocidadeCooler);  //PWM controla cooler
        
        
        tempIdeal = lerInteiroMemoria4Digitos();
        //leitura push buttons
        if (digitalRead(_RB0) == 0) {
            tempIdeal --;
            delay_ms(80);
        } else if (digitalRead(_RB1) == 0) {
            tempIdeal ++;
            delay_ms(80);
        }        
        gravarInteiroMemoria4Digitos();
        converte_LCD(linha2_fim, tempIdeal);
        
    }
    
}


//-------------------------------------------------------
//      Funções gravar dígitos na memória
//-------------------------------------------------------
int lerInteiroMemoria4Digitos() {
    unsigned int milhar=(int) e2prom_read('3');
    unsigned int centena=(int) e2prom_read('2');
    unsigned int dezena=(int) e2prom_read('1');
    unsigned int unidade=(int) e2prom_read('0');
    return (milhar*1000)+(centena*100)+(dezena*10)+(unidade);
}

void gravarInteiroMemoria4Digitos() {    
    unsigned int milhar=0;
    unsigned int centena=0;
    unsigned int dezena=0;
    unsigned int unidade=0;
    milhar = tempIdeal/1000;
    centena = (tempIdeal-(milhar*1000))/100;
    dezena = (tempIdeal-(milhar*1000)-(centena*100))/10;
    unidade = tempIdeal-(milhar*1000)-(centena*100)-(dezena*10);
    
    //se memória vazia, inicializar variáveis
    if ((char) milhar > 9) {        
        milhar = 0;
        centena = 3;
        dezena = 0;
    }
    
    e2prom_write('3', (char) milhar);
    e2prom_write('2', (char) centena);
    e2prom_write('1', (char) dezena);
    e2prom_write('0', (char) unidade);
}

//-------------------------------------------------------
//      Função para ler push buttons
//-------------------------------------------------------
unsigned char digitalRead(unsigned char pin)
{
    unsigned char val = 1<<(0x0F & pin);
    switch((pin & 0xF0)>>4)
    {
       case 0:
         return (PORTA & val) == val;
       case 1:
         return (PORTB & val) == val;  
       case 2:
         return (PORTC & val) == val;
        case 3:
         return (PORTD & val) == val;  
       case 4:
         return (PORTE & val) == val;      
    }
    return 0;
}

//-------------------------------------------------------
//      Funções ADC
//-------------------------------------------------------
void adc_init(void) {
  ADCON0=0b00001000;//AN2 
  ADCON1=0b00001100;//VSS, VDD ref. somente AN0, AN1 e AN2
  ADCON2=0b10001000;
  ADCON0bits.ADON = 0x01; //habilita ADC
}

void ConvertADC(void) {
    //inicio o conversor A/D
    ADCON0bits.GO = 1;
}

unsigned int ReadADC(void) {
    unsigned int temp;
    temp = (ADRESH << 0) | ADRESL;
    return temp;
}


//-------------------------------------------------------
//		Funcao que executa comandos no LCD
//-------------------------------------------------------
void lcd_cmd(unsigned char dado){
	PORTD=dado; 	// Coloca dado na porta
	RS=0; 		// Modo comando
	EN=1; 		// Clock +---------+
	delay_ms(2);	// | <-2ms-> |
	EN=0;		// | | <-2ms->
	delay_ms(2); 	// + +----------
}

//-------------------------------------------------------
//		Funcao que escreve um caracter no LCD
//-------------------------------------------------------
void lcd_escreve(unsigned char dado){
	PORTD=dado; // Coloca dado na porta
	RS=1;		// Modo escrita
	EN=1; 		// Clock +----------+
	delay_ms(2); 	//
	EN=0; 		//
	delay_ms(2);	// + +-----------
}

//-------------------------------------------------------
//		Funcao que inicializa display para 8 bits
//-------------------------------------------------------
void lcd_init(void){
	lcd_cmd(0x38); // Display em 8 bit's
	lcd_cmd(0x38); // Display em 8 bit's (duas vezes)
	lcd_cmd(0x06); // Escreve da esquerda para direita
	lcd_cmd(cursor_off); // Cursor desligado
	lcd_cmd(limpa_lcd); // Limpa LCD
}


//-------------------------------------------------------
//		Funcao que escreve uma string no LCD
//-------------------------------------------------------
void lcd_puts(char *s)
{
   while(*s)
      lcd_escreve(*s++);
}

void converte_LCD(int linha, int valor) {
    unsigned int milhar=0;
    unsigned int centena=0;
    unsigned int dezena=0;
    unsigned int unidade=0;
    milhar = valor/1000;
    centena = (valor-milhar*1000)/100;
    dezena = (valor-milhar*1000-centena*100)/10;
    unidade = valor-milhar*1000-centena*100-dezena*10;
    lcd_cmd(linha);
    lcd_escreve(milhar+0x30); //Soma para converter para ascii
    lcd_escreve(centena+0x30); //Soma para converter para ascii
    lcd_escreve(dezena+0x30); //Soma para converter para ascii
    lcd_escreve('.');
    lcd_escreve(unidade+0x30); //Soma para converter para ascii
}

//-------------------------------------------------------
//      Funcao Delay 1ms
//-------------------------------------------------------
void delay(void){
   int a=0;
   while(a<164){
   a++;}

}

//-------------------------------------------------------
//      Funcao Delay em ms
//-------------------------------------------------------
void delay_ms(unsigned int a){
   while(a!=0){delay();
   a--;}

}


//-------------------------------------------------------
//      Leitura e2prom
//-------------------------------------------------------
unsigned char e2prom_read(unsigned char endereco) {
 EEADR= endereco;
 EECON1bits.RD=1;
 return EEDATA;
}

//-------------------------------------------------------
//      Escrita e2prom
//-------------------------------------------------------
void e2prom_write(unsigned char endereco,unsigned char dado) {
 EEADR= endereco;
 EEDATA=dado;
 EECON1bits.WREN=1;
 EECON2=0x55;
 EECON2=0xAA;
 EECON1bits.WR=1;
 while(EECON1bits.WR==1);
 EECON1bits.WREN=0;
 return;
}


//-------------------------------------------------------
//      Inicializa o PWM
//-------------------------------------------------------

void PWM1_Init(void) {
    CCP1CON=0x00;//desliga PWM
    TRISCbits.TRISC2=1; 
    TRISDbits.TRISD5=1;
    PORTCbits.RC2=0; 
    PORTDbits.RD5=0;
    CCPR1L=0;
    T2CONbits.TMR2ON=0;          
    T2CONbits.T2CKPS=0; 
    PR2=0x3f;
    T2CONbits.TOUTPS=0;
}

//-------------------------------------------------------
//      Inicia o PWM
//-------------------------------------------------------
void PWM1_Start(void) {
    TRISCbits.TRISC2=0; 
    TRISDbits.TRISD5=0;
    CCP1CON=0x0F; //CCP -> PWM mode 0x0F
    T2CONbits.TMR2ON=1;
    //espera PWM normalizar
    PIR1bits.TMR2IF=0;
    while(PIR1bits.TMR2IF == 0);
    PIR1bits.TMR2IF=0;
    while(PIR1bits.TMR2IF == 0);
}

//-------------------------------------------------------
//     Seta Duty 
//-------------------------------------------------------
void PWM1_Set_Duty(unsigned char d) {
    unsigned int temp;      
    temp=(((unsigned long)(d))*((PR2<<2)|0x03))/255;
    CCPR1L= (0x03FC&temp)>>2;
    CCP1CON=((0x0003&temp)<<4)|0x0F;
}
