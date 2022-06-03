/* 
 * File:   Main_Otto.c
 * Author: Santiago Rivera y Santiago Penagos
 * Carnés: 20269 y 20296
 *
 * Created on 12 de mayo de 2022, 06:22 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdint.h>
/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define FLAG_SPI 0xFF
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor máximo de entrada del potenciometro
#define OUT_MIN 0               // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 200             // Valor máximo de ancho de pulso de señal PWM
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
char cont_master = 0;
char cont_slave = 0xFF;
char val_temporal = 0;
char POT_1 = 0;
char POT_2 = 0;
char POT_3 = 0;
char POT_4 = 0;

uint8_t cont;

uint8_t address_pi1;
uint8_t address_pr1;
uint8_t address_ci1;
uint8_t address_cr1;
uint8_t address_pi2;
uint8_t address_pr2;
uint8_t address_ci2;
uint8_t address_cr2;



uint8_t POTS[2] = {0, 0};

unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal
unsigned short CCPR_2 = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);

unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0){    // Verificamos sea AN0 el canal seleccionado
            POT_1 = ADRESH;         // Mostramos ADRESL en PORTD
            PORTAbits.RA7 = 1;
            __delay_ms(1000);       // Esperamos un segundo para mandar datos
            if(SSPSTATbits.BF){     // Revisamos que no haya comunicación en proceso
                SSPBUF = POT_1;
            }
        }
        else if (ADCON0bits.CHS == 0b0001){
            POT_2 = ADRESH;
            PORTAbits.RA7 = 1;
            __delay_ms(1000);       // Esperamos un segundo para mandar datos
            if(SSPSTATbits.BF){     // Revisamos que no haya comunicación en proceso
                SSPBUF = POT_2;      // Movemos el valor del contador para enviarlo
              }
            }
        else if (ADCON0bits.CHS == 0b0010){
            POT_3 = ADRESH;
            CCPR = map(POT_3, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
        }
        else if (ADCON0bits.CHS == 0b0011){
            POT_4 = ADRESH;
            CCPR_2 = map(POT_4, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR2L = (uint8_t)(CCPR_2>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP2CONbits.DC2B0 = CCPR_2 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
            CCP2CONbits.DC2B1 = CCPR_2 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
        }
        PIR1bits.ADIF = 0;
    }
    if(INTCONbits.RBIF){
        if (!RB0)
        {
           write_EEPROM(address_pi1, POT_1);
           write_EEPROM(address_pr1, POT_2);
           write_EEPROM(address_ci1, POT_3);
           write_EEPROM(address_cr1, POT_4);
        }
        if (!RB1)
        {
            CCPR = read_EEPROM(address_ci1);
            CCPR_2 = read_EEPROM(address_cr1);
            POT_1 = read_EEPROM(address_pi1);
            POT_2 = read_EEPROM(address_pr1);
        }
        if (!RB2)
        {
           write_EEPROM(address_pi2, POT_1);
           write_EEPROM(address_pr2, POT_2);
           write_EEPROM(address_ci2, POT_3);
           write_EEPROM(address_cr2, POT_4);
        }
        if (!RB3){
            CCPR = read_EEPROM(address_ci2);
            CCPR_2 = read_EEPROM(address_cr2);
            POT_1 = read_EEPROM(address_pi2);
            POT_2 = read_EEPROM(address_pr2);
        }
        if (!RB4){
            cont++;
        }
        INTCONbits.RBIF = 0;
    }
    return;
}
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){   
        
        address_pi1 = 0x71;
        address_pr1 = 0x72;
        address_ci1 = 0x73;
        address_cr1 = 0x74;
        
        address_pi2 = 0x80;
        address_pr2 = 0x81;
        address_ci2 = 0x82;
        address_cr2 = 0x83;
        
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            if(ADCON0bits.CHS == 0b0000)    
                ADCON0bits.CHS = 0b0001;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0001)
                ADCON0bits.CHS = 0b0010;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0010)
                ADCON0bits.CHS = 0b0011;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0011)
                ADCON0bits.CHS = 0b0000;    // Cambio de canal
            
            __delay_us(40);
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversión
        }
        switch (cont)
        {
            case 0:
                PIE1bits.ADIE = 1;
                PORTDbits.RD0 = 1;
                PORTDbits.RD1 = 0;
                PORTDbits.RD2 = 0;
                
                break;
                
            case 1:
                PIE1bits.ADIE = 0;
                PORTDbits.RD0 = 0;
                PORTDbits.RD1 = 1;
                PORTDbits.RD2 = 0;
                
                break;
            case 2:
                //aqui va el modo de compu
                PIE1bits.ADIE = 1;
                PORTDbits.RD0 = 0;
                PORTDbits.RD1 = 0;
                PORTDbits.RD2 = 1;
                
                break;
            default:
                cont = 0;
        }
    }
    return;
}
/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b00001111;         // Puerto RA1 como entrada analógica
    ANSELH = 0;
    
    TRISB = 0b00011111;
    PORTB = 0;
    
    TRISD = 0;
    PORTD = 0;
    
    TRISA = 0b00001111;         // Encendemos RA1 como entrada
    PORTA = 0;
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    // Configuracion de SPI
    // Configs de Maestro
    TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
    PORTC = 0;
    
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
    SSPBUF = val_temporal;              // Enviamos un dato inicial
        
    ADCON0bits.ADCS = 0b00;     // Fosc/2
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0011;    // Seleccionamos el AN0 , AN1, AN2 y AN3
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
    
    // Configuración PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    PR2 = 62;                  // periodo de 2ms
    TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP2
    
    // Configuración CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP2CON = 0;
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    
    
    CCP2CONbits.CCP2M = 0b1100; // PWM
    
    CCPR1L = 125>>2;
    CCP1CONbits.DC1B = 125 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
    
    CCPR2L = 125>>2;
    CCP2CONbits.DC2B0 = 125 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
    CCP2CONbits.DC2B1 = 125 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
    
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM
    
    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    //Configuración botones
    OPTION_REGbits.nRBPU = 0;   // Habilitamos resistencias de pull-up del PORTB
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1;
    WPUBbits.WPUB2 = 1;
    WPUBbits.WPUB3 = 1;
    WPUBbits.WPUB4 = 1;
    
    INTCONbits.RBIE = 1;         // Habilitamos int. globales
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB1 = 1;
    IOCBbits.IOCB2 = 1;
    IOCBbits.IOCB3 = 1;
    IOCBbits.IOCB4 = 1;
    
    INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción
    
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
    
    // Configuraciones de interrupciones
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de perifericos
    PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepción
}

// FUNCIONES

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato 
}

void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;         // Habilitamos interrupciones
}