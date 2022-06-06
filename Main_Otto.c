/* 
 * File:   Main_Otto.c
 * Author: Santiago Rivera y Santiago Penagos
 * Carnés: 20269 y 20296
 *
 * Created on 12 de mayo de 2022, 06:22 PM
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pic16f887.h>

/*
 * CONSTANTES
 */
#define _XTAL_FREQ 500000 //OSCILADOR A 500 kHz
#define delay_EEPROM 250 //Constante para indicar un delay de read
#define IN_MIN 0 // Valor mínimo de entrada de potenciómetros
#define IN_MAX 255 // Valor máximo de entrada del potenciómetros
#define OUT_MIN 15  //Valor mínimo de ancho de pulso de señal PWM (0.48 ms)
#define OUT_MAX 75 // Valor máximo de ancho de pulso de señal PWM (2.4 ms)
#define LEN_MSG 9 //Constante para delimitar mensaje de respuesta EUSART

/*
 * VARIABLES
 */
int modo = 0; //Variable que indica estado en que nos encontramos
 
uint8_t servo1;
uint8_t servo2;
uint8_t servo3;
uint8_t servo4;
        
uint8_t terminal = 0;
uint8_t bandera = 0;
uint8_t valor_old = 0;
uint8_t indice = 0;     // Variables para la comunicación EUSART

//Variable para almacenar ancho de pulso en interpolación lineal servos
unsigned short CCPR_S1 = 0;
unsigned short CCPR_S2 = 0;
unsigned short CCPR_S3 = 12;
unsigned short CCPR_S4 = 140;

//Array para mensaje de respuesta del EUSART
char mensaje[LEN_MSG] = {'M', 'o', 'v', 'e', 'r', ' ', ' ', 0x0D, 0x0A};

// Prototipos de funciones

void setup(void); //Configuración de módulos, registros y puertos del PIC

uint8_t read_EEPROM(uint8_t address); //Función lectura en EEPROM
void write_EEPROM(uint8_t address, uint8_t data);//Función escritura en EEPROM

unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*
 * INTERRUPCIONES
 */
void __interrupt() isr(void){
    //Interrupción UART
    if(PIR1bits.RCIF){ //Se verifica si hay un nuevo dato en el serial
        mensaje[6] = RCREG; //Se guarda valor recibido en mensaje de respuesta
        terminal = RCREG; //Se guarda valor obtenido de la interfaz               
    }    
    //Se revisa interrupción ADC
    if (PIR1bits.ADIF){
        if (ADCON0bits.CHS == 0){ //Se verifica canal AN0
            if(modo == 0){
                servo1 = ADRESH;
                // Valor de ancho de pulso variable Servo 1
                CCPR_S1 = map(servo1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            }
            else if (modo == 1){
                // Valor de ancho de pulso variable Servo 1
                CCPR_S1 = map(servo1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            } 
            else if (modo == 2){
                // Valor de ancho de pulso variable Servo 1
                CCPR_S1 = map(servo1, IN_MIN, 31, OUT_MIN, OUT_MAX);
            } 
            // Se guardan los 6 bits más significativos en CPR2L
            CCPR2L = (uint8_t)(CCPR_S1>>2);  

            // Se guardan los 2 bits menos significativos en DC2B
            CCP2CONbits.DC2B0 = CCPR_S1 & 1; 
            CCP2CONbits.DC2B1 = (CCPR_S1 >> 1) & 1; 
        }
        else if (ADCON0bits.CHS == 1){ //Se verifica canal AN1
            if(modo == 0){
                servo2 = ADRESH;
                // Valor de ancho de pulso variable Servo 2
                CCPR_S2 = map(servo2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            }
            else if (modo == 1){
                // Valor de ancho de pulso variable Servo 2
                CCPR_S2 = map(servo2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            }
            else if (modo == 2){
                // Valor de ancho de pulso variable Servo 2
                CCPR_S2 = map(servo2, IN_MIN, 31, OUT_MIN, OUT_MAX);
            }

            // Se guardan los 6 bits más significativos en CPR1L
            CCPR1L = (uint8_t)(CCPR_S2>>2); 
            // Se guardan los 2 bits menos significativos en DC1B
            CCP1CONbits.DC1B = CCPR_S2 & 0b11;
        }
        else if (ADCON0bits.CHS == 2){ //Se verifica canal AN2
            if(modo == 0){
                servo3 = ADRESH;
                // Valor de ancho de pulso variable Servo 3
                CCPR_S3 = map(servo3, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            }
            else if (modo == 1){
                // Valor de ancho de pulso variable Servo 3
                CCPR_S3 = map(servo3, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            } 
            else if (modo == 2){
                // Valor de ancho de pulso variable Servo 3
                CCPR_S3 = map(servo3, IN_MIN, 31, OUT_MIN, OUT_MAX);
            }

            // Se añade indicativo de servo 3
            CCPR_S3 = CCPR_S3 & 0b01111111; //bit 7 = 0 (servo 3)
        }
        else if (ADCON0bits.CHS == 3){ //Se verifica canal AN3
            if(modo == 0){
                servo4 = ADRESH;
                // Valor de ancho de pulso variable Servo 4
                CCPR_S4 = map(servo4, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            }
            else if (modo == 1){
                // Valor de ancho de pulso variable Servo 4
                CCPR_S4 = map(servo4, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            } 
            else if (modo == 2){
                // Valor de ancho de pulso variable Servo 4
                CCPR_S4 = map(servo4, IN_MIN, 31, OUT_MIN, OUT_MAX);
            } 

            // Se añade indicativo de servo 4
            CCPR_S4 = (CCPR_S4 & 0b01111111) + 128;//bit 7 = 1 (servo 4)
        }
        PIR1bits.ADIF = 0; // Limpiamos bandera ADC
    }
    //Se revisa interrupción de puerto B
    if (INTCONbits.RBIF){
        if (!PORTBbits.RB0){ //Pb de cambio de modo
            modo ++; //Cada pulsación se cambia de modo
            if(modo > 2) //Si ya se llego al estado máximo -> 2 se reinicia
                modo = 0;
        } 
        else if (!PORTBbits.RB1){
            write_EEPROM(1, servo1);
            write_EEPROM(2, servo2);
            write_EEPROM(3, servo3);
            write_EEPROM(4, servo4);
        }
        else if (!PORTBbits.RB2){
            servo1 = read_EEPROM(1);
            __delay_ms(delay_EEPROM); //Delay de lectura
            servo2 = read_EEPROM(2);
            __delay_ms(delay_EEPROM); //Delay de lectura
            servo3 = read_EEPROM(3);
            __delay_ms(delay_EEPROM); //Delay de lectura
            servo4 = read_EEPROM(4);
            __delay_ms(delay_EEPROM); //Delay de lectura
                
        }
        else if (!PORTBbits.RB3){
            write_EEPROM(5, servo1);
            write_EEPROM(6, servo2);
            write_EEPROM(7, servo3);
            write_EEPROM(8, servo4);
        }
        else if (!PORTBbits.RB4){
            servo1 = read_EEPROM(5);
            __delay_ms(delay_EEPROM); //Delay de lectura
            servo2 = read_EEPROM(6);
            __delay_ms(delay_EEPROM); //Delay de lectura
            servo3 = read_EEPROM(7);
            __delay_ms(delay_EEPROM); //Delay de lectura
            servo4 = read_EEPROM(8);
            __delay_ms(delay_EEPROM); //Delay de lectura       
        }
        INTCONbits.RBIF = 0; //Limpieza de bandera
    }
    return;
}
// MAIN
void main(void) {
    setup(); // Se pasa a configuraciones del PIC
        
    while(1){
        
        SSPBUF = (uint8_t) CCPR_S3; //Se carga al buffer ancho de pulso servo 3
        while(!SSPSTATbits.BF); //Se espera que termine de enviar
        __delay_ms(2); //Delay de operación
        SSPBUF = (uint8_t) CCPR_S4; //Se carga al buffer ancho de pulso servo 4
        while(!SSPSTATbits.BF); //Se espera que termine de enviar
        
        if(modo == 0){ //Si estamos en modo 0
            PORTD = 1; //LED indicador de estado
        }
        
        else if (modo == 1){
            PORTD = 2; //LED indicador de estado
        }
        else if (modo == 2){
            PORTD = 4; //LED indicador de estado   
            if (valor_old != mensaje[6]){ // Si hay nuevo valor en terminal
               if (PIR1bits.TXIF){ // Si TXREG está libre
                        TXREG = mensaje[6]; // Se carga carácter a enviar      
               }
               // Se actualiza valor old para siguiente repetición
                valor_old = mensaje[6]; 
            }
            
        bandera = terminal & 0b01100000;//Bits indicadores de servo
        if(bandera == 0){
            //Servo 1 - Se actualiza potenciómetro (5 bits de la terminal)
            servo1 = terminal & 0b00011111;
        }
        else if(bandera == 32){
            //Servo 2 - Se actualiza potenciómetro (5 bits de la terminal)
            servo2 = terminal & 0b00011111;
        }
        else if(bandera == 64){
            //Servo 3 - Se actualiza potenciómetro (5 bits de la terminal)
            servo3 = terminal & 0b00011111;
        }
        else if(bandera == 96){
            //Servo 4 - Se actualiza potenciómetro (5 bits de la terminal)
            servo4 = terminal & 0b00011111;
        }
        }
        if(ADCON0bits.GO == 0){ // Si no hay proceso de conversión del ADC
                if(ADCON0bits.CHS == 0) //¿Estamos en canal AN0?
                     ADCON0bits.CHS = 1;    // Cambio a AN1
                else if(ADCON0bits.CHS == 1) //¿Estamos en canal AN1?
                     ADCON0bits.CHS = 2;    // Cambio a AN2
                else if(ADCON0bits.CHS == 2) //¿Estamos en canal AN2?
                     ADCON0bits.CHS = 3;    // Cambio a AN0
                else if(ADCON0bits.CHS == 3) //¿Estamos en canal AN3?
                     ADCON0bits.CHS = 0;    // Cambio a AN0
                __delay_us(40); //Sample time
                ADCON0bits.GO = 1; // Se inicia proceso de conversión
        }
    }
    return;
}
// FUNCIONES

/* Función para hacer la interpolación lineal del valor de la entrada analógica 
*  usando solo el registro ADRESH (8 bits) al ancho de pulso del PWM (10 bits),  
*/
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

uint8_t read_EEPROM(uint8_t address){
    EEADR = address; //Se carga address
    EECON1bits.EEPGD = 0; // Se indica lectura a la EEPROM
    EECON1bits.RD = 1; // Se obtiene el dato de la EEPROM
    __delay_us(1); //Delay de lectura
    return EEDAT;  // La función regresa el dato 
}
void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address; //Se carga dirección
    EEDAT = data; //Se carga dato
    
    EECON1bits.EEPGD = 0; // Se indica modo escritura a la EEPROM
    EECON1bits.WREN = 1; // Se habilita escritura en la EEPROM 
    
    INTCONbits.GIE = 0; // Se deshabilitan interrupciones
    
    // Secuencia para iniciar escritura
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1; // Se inicia escritura
    
    while(!PIR2bits.EEIF); //Mientras no se haya completado la escritura espera
    
    PIR2bits.EEIF = 0; //Limpieza de bandera de escritura
    EECON1bits.WREN = 0; // Se deshabilita escritura en la EEPROM
    INTCONbits.RBIF = 0; // Se limpian interrupciones del puerto B
    PIR1bits.ADIF = 0; // Limpieza de bandera del ADC
    INTCONbits.GIE = 1;  // Se habilitan interrupciones
    
    return;
}

/*
 * CONFIGURACIONES
 */
void setup(void) {
    //Configuración de puertos
    ANSEL = 0b00001111; //Se configura RA0-RA3 como entrada analógica
    ANSELH = 0;   //I/O DIGITALES
       
    TRISA = 0b00001111; //AN0, AN1, AN2, AN3 como entrada    
    PORTA = 0;    //Limpiamos puerto A

    TRISD = 0; //PORTD como salida
    PORTD = 0; //Limpiamos puerto D
    
    //Configuración de oscilador
    OSCCONbits.IRCF = 0b011;   //Oscilador interno de 500 KHz
    OSCCONbits.SCS = 1;         //Oscilador interno
    
    //Configuración PORTB
    TRISBbits.TRISB0 = 1; //RB0, RB1 , RB2, RB3 y RB4 como input
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1;
    
    PORTB = 0;      //CLEAR DE PUERTO B
    
    OPTION_REGbits.nRBPU = 0; // Se habilitan pull-up de PORTB
    WPUBbits.WPUB0 = 1; //Se habilita pull de RB0
    WPUBbits.WPUB1 = 1; //Se habilita pull de RB1
    WPUBbits.WPUB2 = 1; //Se habilita pull de RB2
    WPUBbits.WPUB3 = 1; //Se habilita pull de RB3
    WPUBbits.WPUB4 = 1; //Se habilita pull de RB4
    
    //Config ADC
    ADCON0bits.ADCS = 0b11; // FRC 
    ADCON1bits.VCFG0 = 0;  // Referencia VDD
    ADCON1bits.VCFG1 = 0;  // Referencia VSS
    ADCON0bits.CHS = 0; // Se selecciona PORTA0/AN0 inicialmente
    ADCON1bits.ADFM = 0; // Justificado a la izquierda
    ADCON0bits.ADON = 1; // Se habilita el modulo ADC
    __delay_us(420);     // Delay para sample time
    
    //Config PWM
    TRISCbits.TRISC2 = 1; // RC2/CCP1 como salida deshabilitado
    TRISCbits.TRISC1 = 1; // RC1/CCP2 como salida deshabilitado
    CCP1CON = 0; // Se apaga CCP1
    CCP2CON = 0; // Se apaga CCP2
    PR2 = 156; // Período de 20 ms 
    
    // Config CCP
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // Modo PWM para CCP1
    CCP2CONbits.CCP2M = 0b1100; // Modo PWM para CCP2
    //Servo 1
    CCPR1L = 47>>2; //Ciclo de trabajo base pues se va a variar
    CCP1CONbits.DC1B = 47 & 0b11; // Base de 1 ms ancho de pulso

    //Servo 2
    CCPR2L = 47>>2; //Ciclo de trabajo base pues se va a variar
    
    // Se guardan los 2 bits menos significativos en DC2B
    CCP2CONbits.DC2B0 = 47 & 1; 
    CCP2CONbits.DC2B1 = (47>> 1) & 1; 

    //TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende TMR2
    while(!PIR1bits.TMR2IF);    // Se espera un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Se habilita salida de PWM (CCP1)
    TRISCbits.TRISC1 = 0;       // Se habilita salida de PWM (CCP2)
    
    //Configuraciones de comunicación serial
    
    TXSTAbits.SYNC = 0; //Modo asíncrono
    TXSTAbits.BRGH = 1;//High speed Baud Rate
    BAUDCTLbits.BRG16 = 1; //Contador de 16 bits para el Baud Rate
    SPBRGH = 0;
    SPBRG = 12; //Baud rate aprox. en 9600 (valor real 9,615)
    
    RCSTAbits.SPEN = 1; //Se habilita comunicación serial
    
    //Se indica que solo 8 bits serán de transmisión y recepción
    RCSTAbits.RX9 = 0; 
    TXSTAbits.TX9 = 0; 
    
    TXSTAbits.TXEN = 1; //Se habilita transmisor
    RCSTAbits.CREN = 1;//Se habilita el receptor
    
    // Configuracion de SPI
    // Configs de Maestro
    
    TRISC = 0b10010000; // RX y SDI entrada, SCK y SD0 como salida
    PORTC = 0; //Clean de puerto

    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;  // SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0; // Reloj inactivo en 0
    SSPCONbits.SSPEN = 1; // Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1; // Dato enviado cada flanco de subida del SCK
    SSPSTATbits.SMP = 1;  // Dato al final del pulso de reloj
    SSPBUF = 0;  // Enviamos un dato inicial
    
    
    //Config interrupciones
    INTCONbits.GIE = 1; //Se habilitan interrupciones globales
    PIR2bits.EEIF = 0; //Limpieza de bandera de escritura
    
    PIE1bits.ADIE = 1;  //Se habilita interrupcion del ADC
    PIR1bits.ADIF = 0; // Limpieza de bandera del ADC
    
    INTCONbits.RBIE = 1; //Se habilitan interrupciones del PORTB
    IOCBbits.IOCB0 = 1; //Se habilitan interrupción por cambio de estado de RB0
    IOCBbits.IOCB1 = 1; //Se habilitan interrupción por cambio de estado de RB1
    IOCBbits.IOCB2 = 1; //Se habilitan interrupción por cambio de estado de RB2
    IOCBbits.IOCB3 = 1; //Se habilitan interrupción por cambio de estado de RB3
    IOCBbits.IOCB4 = 1; //Se habilitan interrupción por cambio de estado de RB
    INTCONbits.RBIF = 0; //Limpieza de bandera de interrupción de IOCB4
    
    PIE1bits.RCIE = 1; //Se habilitan interrupciones de recepción UART
    INTCONbits.PEIE = 1; // Se habilitan interrupciones de periféricos
   
    return;
}