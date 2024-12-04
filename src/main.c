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
#define TOUCH_DELTA 1700
#define TOUCH_DELTA_SHARP 1200

#define TOUCH_ITERATIONS 3
#define MAIN_LOOP_DELAY 50 //mS
#define POWER_DOWN_SECONDS 60
#define POWER_DOWN_COUNT (POWER_DOWN_SECONDS * (1000/MAIN_LOOP_DELAY)) //Number of delays per second * number of seconds
#define LED_SPEED 2

#define LED_COUNT 13
#define LED_COLOR_LEN 16
// Color correction factors (approximate values for RGB LEDs)
#define R_CORRECTION 1.0F
#define G_CORRECTION 0.8F  // LEDs tend to overemphasize green, so we adjust it
#define B_CORRECTION 0.7F  // LEDs tend to overemphasize blue, so we adjust it
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
uint8_t color_index_start = 0;
uint32_t pwr_down_counter = 0;
uint32_t touch_base[8] = {0};

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color;

color base[] = {
    {24, 0, 0},   // Red
    {24, 12, 0},  // Red-Orange
    {24, 24, 0},  // Orange-Yellow
    {12, 24, 0},  // Yellow-Green
    {0, 24, 0},   // Green
    {0, 24, 12},  // Green-Cyan
    {0, 24, 24},  // Cyan
    {0, 12, 24},  // Cyan-Blue
    {0, 0, 24},   // Blue
    {12, 0, 24},  // Blue-Purple
    {24, 0, 24},  // Purple
    {24, 0, 12},  // Purple-Red
    {24, 12, 12}, // Red-Pink
    {24, 24, 12}, // Pink-Yellow
    {12, 24, 12}, // Yellow-Green
    {0, 24, 24}   // Green-Cyan
};

color leds[LED_COLOR_LEN];


// Function to apply the color correction
uint8_t apply_correction(uint8_t value, float correction) {
    int corrected_value = value * correction;
    return (corrected_value > 32) ? 32 : corrected_value; // Clamp to max value of 32
}

void led_colors_init()
{
    // Apply color correction to each color in the rainbow
    for (int i = 0; i < 16; i++) {
        leds[i].r = apply_correction(base[i].r, R_CORRECTION);
        leds[i].g = apply_correction(base[i].g, G_CORRECTION);
        leds[i].b = apply_correction(base[i].b, B_CORRECTION);
    }    
}

void gpios_init()
{
	funPinMode(PWR_OFF, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(PWR_OFF, FUN_HIGH);
    funPinMode(LEDS, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); 
	funPinMode(PWM_OUT, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF); //PWM Pin
    funPinMode(AUDIO_EN, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
}

void audio_start(uint8_t freq_index)
{	
	idx = 0;
    audio_playing = 1;
    pwr_down_counter = 0;
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
    pwr_down_counter = 0;
}

void audio_update()
{
// #ifdef DEBUG
//         printf("%d\n", idx);
// #endif
    //Don't let the power down happen while in the audio loop
    pwr_down_counter = 0;
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

void leds_highlight_note(uint8_t index)
{
    color col;

    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        col = leds[(i + color_index_start) % LED_COLOR_LEN];
        if (i == index)
        {
            col.r = 128;
            col.g = 128;
            col.b = 128;
        }
        
        leds_send_one(col.r, col.g, col.b);
    }

}

void cycle_leds()
{
    color col;

    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        col = leds[(i + color_index_start) % LED_COLOR_LEN];
        leds_send_one(col.r, col.g, col.b);
    }

    color_index_start++;
}

void leds_all_off()
{
    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        leds_send_one(0, 0, 0);
    }
    
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
        }
        return;
    }

    if (read_last == read_previous) // || audio_playing)
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

    
    audio_stop();    
    leds_highlight_note(freq_index);
    audio_start(freq_index);
    read_previous = read_last;
}

typedef struct 
{
    GPIO_TypeDef * port;
    int pin;
    int adc;
} s_touch_info;

s_touch_info touch_info[8] = {
    {GPIOD, 2, 3},
    {GPIOD, 3, 4},
    {GPIOD, 4, 7},
    {GPIOD, 5, 5},
    {GPIOD, 6, 6},
    {GPIOA, 1, 1},
    {GPIOA, 2, 0},
    {GPIOC, 4, 2}
};


uint32_t touch_read(s_touch_info info)
{
    return  ReadTouchPin(info.port, info.pin, info.adc, TOUCH_ITERATIONS);
}

uint8_t touch_read_sharp_bits(uint8_t index0, uint8_t index1)
{
    uint32_t read;
    uint32_t read2;
    
    read =  touch_read(touch_info[index0]); 
    read2 =  touch_read(touch_info[index1]);

    if (read > touch_base[index0] && read - touch_base[index0] > TOUCH_DELTA_SHARP 
    && read2 > touch_base[index1] && read2 - touch_base[index1] > TOUCH_DELTA_SHARP ) 
    {
        return (1 << index0) | (1 << index1);
    }

    return 0;
}

void touch_get_base() 
{
    uint32_t touch_val = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        touch_base[i] = touch_read(touch_info[i]);
    }
    
}


int main() 
{
    SystemInit();
    led_colors_init();

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

    uint8_t bit = 0;
    uint8_t led_speed_cnt = 0;

    touch_get_base();

    while(1)
    {        
         Delay_Ms(MAIN_LOOP_DELAY);
    // printf("bases: %d, %d, %d, %d, %d, %d, %d, %d\n\t", touch_base[0],
    // touch_base[1],
    // touch_base[2],
    // touch_base[3],
    // touch_base[4],
    // touch_base[5],
    // touch_base[6],
    // touch_base[7]);
         read_last = 0;
        

        

        read_last |= touch_read_sharp_bits(0, 1); //C#
        read_last |= touch_read_sharp_bits(1, 2); //D#
        read_last |= touch_read_sharp_bits(3, 4); //F#
        read_last |= touch_read_sharp_bits(4, 5); //G#
        read_last |= touch_read_sharp_bits(5, 6); //A#

      
        for (uint8_t i = 0; i < 8; i++)
        {
            read =  touch_read(touch_info[i]);   //A3  
            bit = ((read > touch_base[i] && (read - touch_base[i]) > TOUCH_DELTA) ? 1 : 0);
            // printf("%d:%d, ", read, bit);
            read_last  |= (bit << i);  
        }
        
#ifdef DEBUG
        printf("read: %d, base: %d, diff, %d\n", read, touch_base, read - touch_base);
#endif

#ifdef DEBUG
        if (previous_touched != read_last)
        {
            printf ("c: %s, d: %s, e: %s, f: %s, g: %s, a: %s, b: %s, c: %s\n", 
            (read_last & 0x01) ? "X" : "_",
            (read_last & 0x02) ? "X" : "_",
            (read_last & 0x04) ? "X" : "_",
            (read_last & 0x08) ? "X" : "_",
            (read_last & 0x10) ? "X" : "_",
            (read_last & 0x20) ? "X" : "_",
            (read_last & 0x40) ? "X" : "_",
            (read_last & 0x80) ? "X" : "_");
            previous_touched = read_last;
        }
        
#endif        
        read_handler();

        if (!(led_speed_cnt++ % LED_SPEED) && !audio_playing) {
            cycle_leds();
        }

        if (pwr_down_counter++ > POWER_DOWN_COUNT) {      
            leds_all_off();
            funDigitalWrite(PWR_OFF, FUN_LOW);
        }
       
    }
}