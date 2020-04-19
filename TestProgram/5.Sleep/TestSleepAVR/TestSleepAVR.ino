#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

void RTC_init(void);
void LED0_init(void);
void LED0_toggle(void);
void SLPCTRL_init(void);


void RTC_init(void){
  
  uint8_t temp;    
  
  /* Initialize 32.768kHz Oscillator: */    
  /* Disable oscillator: */    
  temp = CLKCTRL.XOSC32KCTRLA;    
  temp &= ~CLKCTRL_ENABLE_bm;
  /* Enable writing to protected register */
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;
  while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm){
    ; /* Wait until XOSC32KS becomes 0 */    
    }    
    
    /* SEL = 0 (Use External Crystal): */    
    temp = CLKCTRL.XOSC32KCTRLA;   
    temp &= ~CLKCTRL_SEL_bm; 
    /* Enable writing to protected register */   
    CPU_CCP = CCP_IOREG_gc; 
    CLKCTRL.XOSC32KCTRLA = temp;
    /* Enable oscillator: */ 
    temp = CLKCTRL.XOSC32KCTRLA; 
    temp |= CLKCTRL_ENABLE_bm;
    /* Enable writing to protected register */   
    CPU_CCP = CCP_IOREG_gc; 
    CLKCTRL.XOSC32KCTRLA = temp; 
    /* Initialize RTC: */ 
    while (RTC.STATUS > 0)    {  
      ; /* Wait for all register to be synchronized */  
      }    
      /* 32.768kHz External Crystal Oscillator (XOSC32K) */
    RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
    /* Run in debug: enabled */ 
    RTC.DBGCTRL = RTC_DBGRUN_bm;  
    RTC.PITINTCTRL = RTC_PI_bm; /* Periodic Interrupt: enabled */ 
    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 32768 */  
                 | RTC_PITEN_bm; /* Enable: enabled */
}
                 
                 
 void LED0_init(void){ 
  /* Make High (OFF) */ 
  PORTB.OUT |= PIN5_bm; 
  /* Make output */  
  PORTB.DIR |= PIN5_bm;
  }
  
void LED0_toggle(void){
  PORTB.IN |= PIN5_bm;
  }
ISR(RTC_PIT_vect){
  /* Clear flag by writing '1': */  
  RTC.PITINTFLAGS = RTC_PI_bm;
  
  LED0_toggle();
    }

void SLPCTRL_init(void){ 
  SLPCTRL.CTRLA |= SLPCTRL_SMODE_PDOWN_gc;
  SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
  }
  
int main(void){ 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  LED0_init(); 
  RTC_init(); 
  SLPCTRL_init();
  
  /* Enable Global Interrupts */  
  interrupts();
   
  while (1){ 
    /* Put the CPU in sleep */  
    sleep_cpu();
   /* The PIT interrupt will wake the CP */
  }
}


 
