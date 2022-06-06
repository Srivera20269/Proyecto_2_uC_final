/* 
 * File:   Main_slaveOtto.c
 * Author: Santiago Rivera y Santiago Penagos
 * Carn�s: 20269 y 20296
 *
 * Created on 12 de mayo de 2022, 06:23 PM
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

/*
 * LIBRERIAS 
 */
#include <xc.h>
#include <stdint.h>
#include <pic16f887.h>

/*
 * CONSTANTES
 */
#define _XTAL_FREQ 500000 //OSCILADOR A 500 kHz
#define OUT_MIN 12  //Valor m�nimo de ancho de pulso de se�al PWM (0.4 ms)
#define OUT_MAX 82 // Valor m�ximo de ancho de pulso de se�al PWM (2.6 ms)

/*
 * VARIABLES
 */
uint8_t val_PWM = 0; //Variable que almacena valores enviados por master
int flag_servo = 0; //Variable bandera para seleccionar servo

// Prototipos de funciones

void setup(void); //Configuraci�n de m�dulos, registros y puertos del PIC


/*
 * INTERRUPCIONES
 */
void __interrupt() isr(void){
    if (PIR1bits.SSPIF){
        val_PWM = SSPBUF;
        flag_servo = (val_PWM < 128); //Si valor menor a 128 (bit 7 = 0)
        val_PWM = val_PWM & 127 ; //Se guardan los 7 bits de width pulse
        if (!flag_servo){ //Bandera en 0 indica servo 4
            // Se guardan los 8 bits m�s significativos en CPR2L
            CCPR2L = (uint8_t)((val_PWM>>2));  

            // Se guardan los 2 bits menos significativos en DC2B
            CCP2CONbits.DC2B0 = val_PWM & 1; 
            CCP2CONbits.DC2B1 = (val_PWM >> 1) & 1; 
        }
        else{ //Bandera en 1 indica servo 3
            // Se guardan los 8 bits m�s significativos en CPR1L
            CCPR1L = (uint8_t)(val_PWM>>2); 

            // Se guardan los 2 bits menos significativos en DC1B
            CCP1CONbits.DC1B = val_PWM & 0b11;
        }
     PIR1bits.SSPIF = 0;   
    }
    return;
}

/*
 * LOOP PRINCIPAL
 */
void main(void) {
    setup(); //Configuraci�n del pic
    
    while(1){
        //Envio de datos del maestro
    }
    return;
}

/*
 * CONFIGURACIONES
 */
void setup(void) {
    // I/O DIGITALES
    ANSEL = 0;
    ANSELH = 0;
    
    TRISAbits.TRISA5 = 1; //SS como input
    
    //Configuraci�n de oscilador
    OSCCONbits.IRCF = 0b011;   //Oscilador interno de 500 KHz
    OSCCONbits.SCS = 1;         //Oscilador interno 
    
    //Config interrupciones
    INTCONbits.GIE = 1; //Se habilitan interrupciones globales
    PIR1bits.SSPIF = 0;  // Se limpia bandera de SPI
    PIE1bits.SSPIE = 1;  // Se habilita interrupci�n de SPI
    INTCONbits.PEIE = 1; // Se habilitan interrupciones de perif�ricos
    
    //Config PWM
    TRISCbits.TRISC2 = 1; // RC2/CCP1 como salida deshabilitado
    CCP1CON = 0; // Se apaga CCP1
    CCP2CON = 0; // Se apaga CCP2
    PR2 = 156; // Per�odo de 20 ms 
    
    // Config CCP
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // Modo PWM para CCP1
    CCP2CONbits.CCP2M = 0b1100; // Modo PWM para CCP2
    //Servo 3
    CCPR1L = 47>>2; //Ciclo de trabajo base pues se va a variar
    CCP1CONbits.DC1B = 47 & 0b11; // Base de 90� ancho de pulso

    //Servo 4
    CCPR2L = 47>>2; //Ciclo de trabajo base pues se va a variar
    
    // Se guardan los 2 bits menos significativos en DC2B
    CCP2CONbits.DC2B0 = 47 & 1; 
    CCP2CONbits.DC2B1 = (47 >> 1) & 1; 

    //TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende TMR2
    while(!PIR1bits.TMR2IF);    // Se espera un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Se habilita salida de PWM (CCP1)
    TRISCbits.TRISC1 = 0;       // Se habilita salida de PWM (CCP2)
    
    // Configuracion de SPI
    // Configs del esclavo
    TRISC = 0b00011000; // SDI y SCK entradas, SD0 como salida
    PORTC = 0;
    
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100; // SPI Esclavo, SS habilitado
    SSPCONbits.CKP = 0; //Reloj inactivo en 0
    SSPCONbits.SSPEN = 1; // Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1; // Dato enviado cada flanco de subida del SCK
    SSPSTATbits.SMP = 0; // Dato al final del pulso de reloj
    
    return;
}

