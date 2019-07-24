
#include "msp.h"

double distance = 0, milliseconds = 0;

void Port1_Init(void){                  // LED and Button

    P1->SEL0 &= ~BIT1 | BIT0;           // Set as GPIO
    P1->SEL1 &= ~BIT1 | BIT0;
    P1->OUT &= ~BIT0;                   // Led off
    P1->DIR |= BIT0;                    // Set led as output P1.0
    P1->DIR &= ~BIT1;                   // Set button as input P1.1
    P1->REN |= BIT1;                    // Resistor enable
    P1->OUT |= BIT1;                    // Pull-up
    P1->IES |= BIT1;                    // Falling edge trigger
    P1->IFG &= ~BIT1;                   // Clear interrupt flag
    P1->IE |= BIT1;                     // Enable interrupt
    __enable_interrupt();               // Enable global interrupts
    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);
}

void Port2_Init(void){                  // Ultrasonic Sensor Input

    P2->SEL0 &= ~BIT4 | BIT0;           // Set as GPIO
    P2->SEL1 &= ~BIT4 | BIT0;
    P2->OUT &= ~BIT0;                   // Led off
    P2->DIR |= BIT0;                    // Set led as output P2.0
    P2->DIR &= ~BIT4;                   // Set P2.4 as input
    P2->IES &= ~BIT4;                   // Rising edge trigger
    P2->IFG &= ~BIT4;                   // Clear interrupt flag
    P2->IE |= BIT4;                     // Enable interrupt
    __enable_interrupt();               // Enable global interrupts
    NVIC->ISER[1] = 1 << ((PORT2_IRQn) & 31);
}

void Port3_Init(void){                  // Ultrasonic Sensor Output

    P3->SEL0 &= BIT3;                   // Set as GPIO
    P3->SEL1 &= BIT3;
    P3->OUT &= ~BIT3;
    P3->DIR |= BIT3;                    // Set P3.3 as output

}

void ShutdownPins(void){                // Shutdown all pins for low power

    // Set as GPIO
    P1->SEL0 &= ~0xFF; P2->SEL0 &= ~0xFF; P3->SEL0 &= ~0xFF; P4->SEL0 &= ~0xFF;
    P5->SEL0 &= ~0xFF; P6->SEL0 &= ~0xFF; P7->SEL0 &= ~0xFF; P8->SEL0 &= ~0xFF;
    P9->SEL0 &= ~0xFF; P10->SEL0 &= ~0xFF; PJ->SEL0 &= ~0xFF;

    P1->SEL1 &= ~0xFF; P2->SEL1 &= ~0xFF; P3->SEL1 &= ~0xFF; P4->SEL1 &= ~0xFF;
    P5->SEL1 &= ~0xFF; P6->SEL1 &= ~0xFF; P7->SEL1 &= ~0xFF; P8->SEL1 &= ~0xFF;
    P9->SEL1 &= ~0xFF; P10->SEL1 &= ~0xFF; PJ->SEL1 &= ~0xFF;

    // Set as Output
    P1->DIR |= 0xFF; P2->DIR |= 0xFF; P3->DIR |= 0xFF; P4->DIR |= 0xFF;
    P5->DIR |= 0xFF; P6->DIR |= 0xFF; P7->DIR |= 0xFF; P8->DIR |= 0xFF;
    P9->DIR |= 0xFF; P10->DIR |= 0xFF; PJ->DIR |= 0xFF;

    // Set low
    P1->OUT &= ~0xFF; P2->OUT &= ~0xFF; P3->OUT &= ~0xFF; P4->OUT &= ~0xFF;
    P5->OUT &= ~0xFF; P6->OUT &= ~0xFF; P7->OUT &= ~0xFF; P8->OUT &= ~0xFF;
    P9->OUT &= ~0xFF; P10->OUT &= ~0xFF; PJ->OUT &= ~0xFF;

}

void Timer32_1(void){                   // Timer32 for 10 us timing

    // Default clock SMCLK = 3.0MHz
    // Setting predivider = 1
    // Max value = 30 = 10 us
    // One shot mode, interrupt enabled
    TIMER32_1->CONTROL |= TIMER32_CONTROL_SIZE | TIMER32_CONTROL_ONESHOT |
                          TIMER32_CONTROL_PRESCALE_0;

    NVIC->ISER[0] = 1 << ((T32_INT1_IRQn) & 31);
    __enable_interrupt();
    TIMER32_1->CONTROL |= TIMER32_CONTROL_IE;

}

void Timer32_2(void){                   // Timer32 for timing of input

    // Default clock SMCLK = 3.0MHz
    // Setting predivider = 1
    // Max value = 30 = 10 us
    // One shot mode, interrupt enabled
    TIMER32_2->CONTROL |= TIMER32_CONTROL_SIZE | TIMER32_CONTROL_ONESHOT |
                          TIMER32_CONTROL_PRESCALE_0;

    NVIC->ISER[0] = 1 << ((T32_INT2_IRQn) & 31);
    __enable_interrupt();
    TIMER32_2->CONTROL |= TIMER32_CONTROL_IE;

}


void main(void){

    WDTCTL = (WDTPW | WDTHOLD);         // Hold watchdog timer

    ShutdownPins();                     // Shutdown all pins for low power
    Port1_Init();                       // Initializes Port1.0+1.1 (LED and button)
    Port2_Init();                       // Initializes Port2.4 (Ultrasonic sensor input)
    Port3_Init();                       // Initializes Port3.3 (Ultrasonic sensor output)
    Timer32_1();                        // Initializes Timer32_1
    Timer32_2();                        // Initializes Timer32_2



    while (1){

          __sleep();

          P2->OUT &= ~BIT0;
    }

}


void PORT1_IRQHandler(void){            // Interrupt handler for Button

  P1->IE &= ~BIT1;                      // Disable interrupt for debouncing
  P3->OUT |= BIT3;                      // Set Ultrasonic sensor output high
  TIMER32_1->LOAD= 0x1E;                // 0x1E = 10us
  TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE;
  P1->IFG &= ~BIT1;                     // Clear flag
  P1->IE |= BIT1;                       // Enable interrupt

}

void PORT2_IRQHandler(void){            // Interrupt handler for Ultrasonic sensor

     P2->IE &= ~BIT4;                   // Disable interrupt for debouncing

  if(!(P2IES & BIT4))                   // Rising edge interrupt
  {
      P1->OUT |= BIT0;
      TIMER32_2->LOAD= 0x15F90;         //0x15F90 = 30 ms
      TIMER32_2->CONTROL |= TIMER32_CONTROL_ENABLE;

      P2->IES |= BIT4;                  // Toggle interrupt to falling edge


  }
  else
  {
      P1->OUT &= ~BIT0;                // Falling edge interrupt
      P2->IES &= ~BIT4;                // Toggle interrupt to rising edge
    TIMER32_2->CONTROL &= ~(TIMER32_CONTROL_ENABLE);
    milliseconds = TIMER32_2->VALUE;
    distance = 17150.0* (90000.0 - (double)milliseconds)/3000000.0;


  }

  P2->IFG &= ~BIT4;                     // Clear flag
  P2->IE |= BIT4;                       // Enable interrupt


}

void T32_INT1_IRQHandler(void){         // Interrupt handler for Timer32_1

    P3->OUT &= ~BIT3;                   // Set Ultrasonic sensor output low
    TIMER32_1->INTCLR |= BIT0;

}

void T32_INT2_IRQHandler(void){         // Interrupt handler for Timer32_2

    P2->OUT |= BIT0;                    // Ultrasonic sensor timed out
    TIMER32_2->INTCLR |= BIT0;
}



