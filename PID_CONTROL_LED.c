/*
 * File:   RETO.c
 * Author: renet
 *
 * Created on 30 de noviembre de 2021, 11:03 PM
 */
#include<xc.h>
#include "config_header.h"

//declaración de funciones
enum oscillator{oscillator1MHz=0b00110010};
void portsInit(void); 
void adcInit(void); //se lee el pin A0
void pwmInit(void);    //GENERA EL PWM
void timer2Init(void); //GENERA EL PWM (SE UTILIZA EL pin C1)
void read_adc(void);
void state_machine(void);
void InterruptsInit(void); //UTILIZAMOS EL TIMER0
//declaración de variables
float VFADC;
unsigned short current_voltage,S90=900,S85=850, S80=800, S75=750;
unsigned int nuevopwm, present_state, next_state;

//INTERRUPCIÓN DE ALTA PRIORIDAD
void __interrupt(high_priority) high_isr(void)
{   
    if(INTCONbits.TMR0IF) {
    T0CONbits.TMR0ON = 0;  /* APAGA EL TIMER0 */
    INTCONbits.TMR0IF = 0;   /* LIMPIA LA BANDERA DE
                             INTERRUPCIÓN */
    LATD = ~LATD;		   /*CAMBIA EL ESTADO DEL LED
                           CONECTADO EN EL PUERTO D*/
    read_adc();            /*LLAMA A LA FUNCIÓN ADC, QUE DEVUELVE
                           UN VALOR EN 10 bits en current_voltage*/ 
    VFADC=(float)(current_voltage*5.0/1023.0); /*CREAMOS UNA VARIABLE EN [V]*/
    state_machine();        /*LLAMA A LA FUNCIÓN MÁQUINA DE ESTADOS*/
    TMR0H = 0x67;          
    TMR0L = 0x69;          /* INTERRUPCIÓN GENERADA CADA: T=20 s */
    
    T0CONbits.TMR0ON = 1;  /* SE ENCIENDE EL TIMER0 */
    }
}

//PROGRAMA PRINCIPAL
void main(void) {
    present_state=S90; //EMPEZAMOS en 90% DE DC
    portsInit(); //INICIALIZACIÓN DE LOS PUERTOS
    adcInit(); //INCIALIZAMOS LOS REGISTROS CORRESPONDIENTES AL ADC
    pwmInit(); //INCIALIZAMOS LOS REGISTROS CORRESPONDIENTES AL PWM
    timer2Init(); //AQUÍ COMIENZA A FUNCIONAR EL PWM 
    InterruptsInit(); //INCIALIZAMOS LA INTERRUPCIÓN QUE SE ACTIVA CADA 20 SEG
    while(1){
       //ESTE CICLO EVITA QUE NUESTRO PROGRAMA TERMINE
    }
return;
}

//MANTENER 60°C EN EL LED!
void state_machine(void){
    if (present_state==S90){
        if (VFADC<3.37){ //LA COMPARACIÓN LA HAREMOS CON FLOATS!
            next_state=S85; //EL VALOR ESTÁ POR ENCIMA DEL UMBRAL!
        }
        else if(VFADC>=3.37){
            next_state=S90; //EL VALOR ESTA DENTRO DEL UMBRAL!
        }
        else{
            next_state=S90;//EL VALOR ESTA POR DEBAJO DEL UMBRAL!
        }
    }
        
        
    if(present_state==S85){
        if (VFADC<3.04){
            next_state=S80; //EL VALOR ESTÁ POR ENCIMA DEL UMBRAL!
        }
        else if(VFADC>=3.04 && VFADC<3.06){ //3.18
            next_state=S85; //EL VALOR ESTA DENTRO DEL UMBRAL!
        }
        else{
            next_state=S90;//EL VALOR ESTA POR DEBAJO DEL UMBRAL!
        }
    }
        
        if(present_state==S80){
        if (VFADC<2.75){
            next_state=S75; //EL VALOR ESTÁ POR ENCIMA DEL UMBRAL!
        }
        else if(VFADC>=2.74 && VFADC<2.75){
            next_state=S80; //EL VALOR ESTA DENTRO DEL UMBRAL!
        }
        else{
            next_state=S85;//EL VALOR ESTA POR DEBAJO DEL UMBRAL!
        }
        }
       
        
        
        
        if(present_state==S75){
        if (VFADC<2.509){
            next_state=S75; //EL VALOR ESTÁ POR ENCIMA DEL UMBRAL! NO SE PUEDE BAJAR MÁS! PONER WARNING
        }
        else if(VFADC>=2.509 && VFADC<2.512){
            next_state=S75; //EL VALOR ESTA DENTRO DEL UMBRAL!
        }
        else{
            next_state=S80;//EL VALOR ESTA POR DEBAJO DEL UMBRAL!
        }
        }
        
        //ASIGNAMOS EL NUEVO PWM!
        nuevopwm=next_state;
        CCPR2L=(unsigned char)(nuevopwm>>2);//8 MSB 
        CCP2CONbits.DC2B=nuevopwm; //2 LSB 
   
       //NUEVO ESTADO
        present_state=next_state; 

}


//----------DEFINICIÓN DE FUNCIONES----------//



void adcInit(void){
    ADCON0bits.CHS=0b00000; //Select channel AN0 (A0)9+1
    ADCON0bits.GO=0; //covnersion not in progress
    ADCON0bits.ADON=1; //ADC enabled
    ADCON2bits.ADCS=0b000; //Fosc/2 para una correcta conversión
    ADCON2bits.ADFM=0; //la justificación esta a la izquierda.
                       //El registro Low tendrá los Least Significant Bits
}

void InterruptsInit(void)
{
    RCONbits.IPEN = 1;         /* Interrupts with different priorities*/
    INTCONbits.TMR0IF = 0;	   /* Clear TMR0IF flag*/
    T0CONbits.TMR0ON = 0;       /* Timer off */
    T0CONbits.T0PS = 0b110;        /* Prescaler is 128 */
    T0CONbits.PSA = 0;          /* prescaler enabled */ 
    T0CONbits.T0SE = 0;         /* Timer mode Fosc/4, no event counting mode */
    T0CONbits.T0CS = 0;         /* rising edge for tocki- disabled */
    T0CONbits.T08BIT = 0;       /* 16-bit mode */
    INTCONbits.TMR0IE = 1;	    /* Enable INT0 external interrupt*/
    INTCONbits.GIEL = 1;		/* Enable Global Interrupt (low priority */
    INTCONbits.GIEH = 1;        /* Enable Global interrupt (high prio) */
    TMR0H = 0x67;          
    TMR0L = 0x69;          /* Interrupt generated every: T=20ms */
    
    T0CONbits.TMR0ON = 1;       /* Timer on */
}
void timer2Init( void ){
    TMR2 = 0b00000000;// clear TMR2 count
    PIR1bits.TMR2IF = 0;// clear TMR2 overflow flag
    T2CONbits.T2CKPS=01;//prescaler is 4
    T2CONbits.T2OUTPS = 0000;// configure the TMR2 postscaler (1)
    T2CONbits.TMR2ON = 1;// Start PWM
    //in order to send a complete duty cycle and period on the 1st PWM output
    while(PIR1bits.TMR2IF == 0);//    wait until TMR2 overflows
    TRISCbits.TRISC1 = 0;//    enable the C1 pin output (for PWM)
}

void read_adc(void){
    ADCON0bits.GO=1; //STARTS AD COVERSION CYCLE. AUTOMATICALLY CLEARED
                         //WHEN AD IS FINISHED
        while(ADCON0bits.DONE); //IF AD IS FINISHED, STORE RESULTS ON PRODH REGISTER
        current_voltage= ADRESH << 2;
        current_voltage=(ADRESL>>6)|current_voltage;           
}
void portsInit( void ){
    OSCCON = oscillator1MHz;//      set internal clk and oscillator to 4MHz
    TRISCbits.TRISC1 = 1; //disable output pin (will be activated latter)
    TRISAbits.TRISA0=1; //A0 as input
    ANSELAbits.ANSA0=1; //A0 as analog
    
//    //USEFUL FOR INTERRUPT!
//    LATD = 0;  /* init to zero */
//    ANSELD = 0;  /* al D outputs are digital */
//    TRISD = 0;   /* all terminals of Port D are outputs*/
}

//THESE TWO FOR PWM!!!
void pwmInit( void ){
    PR2 = 249;//load PR2 with the PWM period value (4ms)
    CCPR2L = 0;//Cargar un DC=90%
    CCP2CONbits.DC2B=0b00; //2 bits menos significativos
    CCP2CONbits.CCP2M=0b1100;//Activar modo PWM
}