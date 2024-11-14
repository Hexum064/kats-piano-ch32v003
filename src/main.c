#include <ch32v003fun.h>
#include <stdio.h>
#include "ch32v003_touch.h"
#include "audio.h"

#define TOUCH_0     PA2
#define TOUCH_1     PA1
#define TOUCH_2     PC4
#define TOUCH_3     PD2
#define TOUCH_4     PD3
#define TOUCH_5     PD5
#define TOUCH_6     PD6
#define TOUCH_7     PD4

#define PWR_OFF     PC3
#define PWM_OUT     PD0  //T1CH1N
#define AUDIO_EN    PC6
#define LEDS        PC5
#define SWIO        PD1


#define TOUCH_THRESHHOLD 7000	// Has to be over this to register as a touch
#define RELEASE_THRESHHOLD 5750 // Has to be under this to register as a release

#define TOUCH_ITERATIONS 3

// #define DEBUG

uint16_t sample_delays[] = {
    8533,
    8067,
    7559,
    7111,
    6761,
    6358,
    6000,
    5647,
    5333,
    5026,
    4776,
    4528,
    4248
};

uint16_t idx = 0;
uint8_t read_last = 0;
uint8_t read_previous = 0;
uint8_t audio_playing = 0;

void gpios_init()
{
	funPinMode(PWR_OFF, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(PWR_OFF, FUN_HIGH);
    funPinMode(LEDS, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); 
	// funPinMode(LED_D, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // D LED
    // funPinMode(LED_E, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // E LED
	// funPinMode(LED_F, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // F LED
	// funPinMode(LED_I, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // I LED
	// funPinMode(LED_U, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // U LED
	funPinMode(PWM_OUT, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF); //PWM Pin
    funPinMode(AUDIO_EN, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
	// funPinMode(LED_0, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	// funPinMode(LED_1, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	// funPinMode(LED_2, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	// funPinMode(LED_3, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	// funPinMode(LED_4, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	// funPinMode(LED_5, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	// funPinMode(LED_6, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	// funPinMode(LED_7, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED

	// PC4 is T1CH4, 10MHz Output alt func, push-pull
	// GPIOA->CFGLR &= ~(0xf << (4 * PWM_OUT));
	// GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * PWM_OUT);
}

void audio_start(uint8_t freq_index)
{	
	idx = 0;
    audio_playing = 1;
	// turn off audio disable pin
	funDigitalWrite(AUDIO_EN, FUN_LOW);

#ifdef DEBUG
            printf("audio start: freq %d\n", sample_delays[freq_index]);
#endif

    TIM2->ATRLR = sample_delays[freq_index];

	// Enable TIM1
	TIM1->CTLR1 |= TIM_CEN;
    // Enable TIM2
	TIM2->CTLR1 |= TIM_CEN;
}

void audio_stop()
{
	
	// turn on audio disable pin. disabled high
	funDigitalWrite(AUDIO_EN, FUN_HIGH);
	TIM1->CTLR1 &= ~TIM_CEN;
	TIM2->CTLR1 &= ~TIM_CEN;
    idx = 0;
    audio_playing = 0;
    TIM1->CH1CVR = 255;
}

void audio_update()
{
// #ifdef DEBUG
//         printf("%d\n", idx);
// #endif    
	if (idx >= AUDIO_LEN)
	{

		idx = LOOP_START;
	}
	else
	{
		TIM2->INTFR = TIM_CC1IF;

        TIM1->CH1CVR = note_audio[idx];
		
	}
}

void TIM2_IRQHandler(void) __attribute__((interrupt));
void TIM2_IRQHandler(void)
{
	idx++;

	audio_update();
	// dfiu_leds_update();
}

//TODO: Change pwm out to the correct IO

void pwm_init()
{
	// Reset TIM1 to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	// Prescaler
	TIM1->PSC = 0x0000;

	// Auto Reload - sets period. Also means we have 8 bits of resolution
	TIM1->ATRLR = 255;

	// Reload immediately
	TIM1->SWEVGR |= TIM_UG;

	// Enable CH1N output, positive pol
	TIM1->CCER |= TIM_CC1NE | TIM_CC1NP;

	// Enable CH4 output, positive pol
	//TIM1->CCER |= TIM_CC2E | TIM_CC2P;

	// CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;

	// CH2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	//TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;

	// Set the Capture Compare Register value to 50% initially
	 TIM1->CH1CVR = 128;
	//TIM1->CH2CVR = 128;

	// Enable TIM1 outputs
	TIM1->BDTR |= TIM_MOE;
}

void sample_timer_init()
{
    	// Reset TIM2 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;


	TIM2->CTLR1 = TIM_ARPE;

	TIM2->PSC = 0x0000;



	TIM2->SWEVGR |= TIM_UG;

	TIM2->DMAINTENR = TIM_UIE;

	NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->CTLR1 &= ~TIM_CEN;
}

void leds_send_one(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t i;

    i = 8;
    while(i-- > 0)
    {
        funDigitalWrite(LEDS, FUN_HIGH);
        Delay_Tiny(2);
        funDigitalWrite(LEDS, (g >> i) & 0x01)
        Delay_Tiny(2);
        funDigitalWrite(LEDS, FUN_LOW);
    }

    i = 8;
    while(i-- > 0)
    {
        funDigitalWrite(LEDS, FUN_HIGH);
        Delay_Tiny(2);
        funDigitalWrite(LEDS, (r >> i) & 0x01)
        Delay_Tiny(2);
        funDigitalWrite(LEDS, FUN_LOW);
    }
    
    i = 8;   
    while(i-- > 0)
    {
        funDigitalWrite(LEDS, FUN_HIGH);
        Delay_Tiny(2);
        funDigitalWrite(LEDS, (b >> i) & 0x01)
        Delay_Tiny(2);
        funDigitalWrite(LEDS, FUN_LOW);
    }        
}

void leds_update(uint8_t index)
{

}

void leds_all_off()
{
}

void led_test()
{
    leds_send_one(64, 0 ,0);
    leds_send_one(0, 64 ,0);
    leds_send_one(0, 0 ,64);
    leds_send_one(64, 0 ,64);
    leds_send_one(64, 0 ,0);
    leds_send_one(0, 64 ,0);
    leds_send_one(0, 0 ,64);    
    leds_send_one(64, 0 ,0);
    leds_send_one(0, 64 ,0);
    leds_send_one(0, 0 ,64);    
    leds_send_one(64, 0 ,0);
    leds_send_one(0, 64 ,0);
    leds_send_one(0, 0 ,64);
    leds_send_one(64, 0 ,0);
    leds_send_one(0, 64 ,0);
    leds_send_one(0, 0 ,64);    
}

void read_handler()
{
    uint8_t freq_index = 0;

    if (!(read_last))
    {
        
        if (audio_playing)
        {
#ifdef DEBUG
            printf("audio stop\n");
#endif
            read_previous = 0;
            audio_stop();
            leds_all_off();
        }
        return;
    }

    if (read_last == read_previous || audio_playing)
    {
        return;
    }

#ifdef DEBUG
        printf("last: %d, prev: %d, touch xor %d\n", read_last, read_previous, (read_previous ^ read_last));
#endif

    switch (read_last)
    {
        case 0:
        default:
            return;
        case 0x01: //C4 0000 0001
            freq_index = 0;
            break;
        case 0x03: //C4# 0000 0011
            freq_index = 1;
            break;
        case 0x02: //D4 0000 0010
            freq_index = 2;
            break;
        case 0x06: //D4# 0000 0110
            freq_index = 3;
            break;
        case 0x04: //E4 0000 0100
            freq_index = 4;
            break;
        case 0x08: //F4 0000 1000
            freq_index = 5;
            break;
        case 0x18: //F4# 0001 1000
            freq_index = 6;
            break;
        case 0x10: //G4 0001 00000 
            freq_index = 7;
            break;                
        case 0x30: //G4# 0011 00000 
            freq_index = 8;
            break;                                                                                                
        case 0x20: //A5 0010 00000 
            freq_index = 9;
            break;                            
        case 0x60: //A5# 0110 00000 
            freq_index = 10;
            break;                            
        case 0x40: //B5 0100 00000 
            freq_index = 11;
            break;                            
        case 0x80: //C5 1000 00000 
            freq_index = 12;
            break;                            
    }

#ifdef DEBUG
        printf("Changing audio. Freq: %d\n", freq_index);
#endif

    leds_update(freq_index);
    audio_stop();    
    audio_start(freq_index);
    read_previous = read_last;
}

int main() 
{
    SystemInit();

    	// Enable GPIOs and timers
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1;
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	gpios_init();
    sample_timer_init();
    pwm_init();

	InitTouchADC();
    
    audio_stop();

    PWR->CTLR |= PWR_CTLR_PDDS;

    uint32_t read;

#ifdef DEBUG
    uint32_t previous_touched = 0;
#endif

    uint8_t i = 0;


    uint32_t count = 0;

    while(1)
    {        
         Delay_Ms(50);

        read_last = 0;
        i = 0;
        read = ReadTouchPin(GPIOD, 4, 7, TOUCH_ITERATIONS);        
        read_last  |= ((read > 7000 ? 1 : 0) << (i++)); //A7
        read = ReadTouchPin(GPIOD, 6, 6, TOUCH_ITERATIONS);
        read_last  |= ((read > 7000 ? 1 : 0) << (i++)); //A6
        read = ReadTouchPin(GPIOD, 5, 5, TOUCH_ITERATIONS) ;
        read_last  |= ((read > 7000 ? 1 : 0) << (i++)); //A5
        read =  ReadTouchPin(GPIOD, 3, 4, TOUCH_ITERATIONS);
        read_last  |= ((read > 7000 ? 1 : 0) << (i++)); //A4
        read =  ReadTouchPin(GPIOD, 2, 3, TOUCH_ITERATIONS);
        read_last  |= ((read > 7000 ? 1 : 0) << (i++)); //A3   
        read =  ReadTouchPin(GPIOC, 4, 2, TOUCH_ITERATIONS);
        read_last  |= ((read > 7000 ? 1 : 0) << (i++)); //A2
        read =  ReadTouchPin(GPIOA, 1, 1, TOUCH_ITERATIONS);
        read_last  |= ((read > 7000 ? 1 : 0) << (i++)); //A1
        read =  ReadTouchPin(GPIOA, 2, 0, TOUCH_ITERATIONS);        
        read_last  |= ((read > 7000 ? 1 : 0) << (i++)); //A0

#ifdef DEBUG
        if (previous_touched != read_last)
        {
            printf ("7: %s, 6: %s, 5: %s, 4: %s, 3: %s, 2: %s, 1: %s, 0: %s\n", 
            (read_last & 0x01) ? "X" : "_",
            (read_last & 0x02) ? "X" : "_",
            (read_last & 0x04) ? "X" : "_",
            (read_last & 0x08) ? "X" : "_",
            (read_last & 0x10) ? "X" : "_",
            (read_last & 0x20) ? "X" : "_",
            (read_last & 0x40) ? "X" : "_",
            (read_last & 0x80) ? "X" : "_");
            
        }
        previous_touched = read_last;
#endif        
        read_handler();
        led_test();

        if (count++ > 200) {
            // __WFE();
             funDigitalWrite(PWR_OFF, FUN_LOW);
        }

        // Delay_Ms(250);
        // funDigitalWrite(LED_D, FUN_HIGH);
        // Delay_Ms(250);        
       
    }
}