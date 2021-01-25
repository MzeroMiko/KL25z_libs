#define EXP4_3_C
// #define __PRINT_CAPTURE__
// #define __OPEN_LOOP__
// #define __FILTER__

#if defined(EXP4_3_C)

#include "redrivers/pit.h"
#include "redrivers/tpm.h"
#include "redrivers/port.h"
#include "redrivers/gpio.h"
#include "redrivers/uart.h"
#include "redrivers/dac_adc.h"
#include "sh.h"


#define TPM_PWM_ID 2
#define TPM_PWM_FREQ 10000
// #define TPM_PWM_FREQ 500
#define TPM_CAPTURE_ID 1
#define TPM_CAPTURE_IRQn_ID TPM1_IRQn
#define Speed_Capture_ISR TPM1_IRQHandler
// #define TPM_CAPTURE_FREQ 1000
#define TPM_CAPTURE_TIME_PERIOD 0xFFFF
#define TPM_CAPTURE_IRQn_PRIORITY 0x20
#define MAX_OVERFLOW_COUNT 5
#define FILTER_DEPTH 5
#define MAX_RECORD 800
// #define LED_SENSE_FREQ 100
// #define PIT_CAPTURE_IRQn_PRIORITY 0xFF // lowest priority 
// #define Distance_Capture_ISR PIT_IRQHandler
// #define Distance_Capture_ISR2 ADC0_IRQHandler
#define LIGHT_BIT(x) GPIO_WriteBit(PORT_D, 1, ~(x >> 2)); GPIO_WriteBit(PORT_B, 19, ~(x >> 1)); GPIO_WriteBit(PORT_B, 18, ~(x >> 0))

typedef struct _motor_rec {
    int32_t start, idx, enPrint, rec[MAX_RECORD];
} motor_rec_t;

typedef struct _motor_data {
    struct { uint32_t pwmChannel, capChannel, pwmPort, pwmBack, capPort; } conf;
    struct { int32_t fakeKr, fakeKp, target, duty;} stat;
    int32_t timeDelta, passOverflow, lastBase, curIdx, nextIdx, deltas[FILTER_DEPTH];
    // fakeKr = 1000 / K, fakeKp = 1000 * Kp 
} motor_data_t;

typedef struct _led_sense {
    uint32_t port, pinx, adcChannel;
    int32_t boundNear, boundFar, currentVal, currentDelta;
} led_sense_t;

uint32_t FAKERATE = (0xFFFFFFFF / (DEFAULT_SYSTEM_CLOCK / (1U << TPM_Prescale_Divide_4)) * 390); // 139230; 0x2EFDE
uint32_t maxSpeed = (5 * (0xFFFFFFFF / (DEFAULT_SYSTEM_CLOCK / (1U << TPM_Prescale_Divide_4)) * 390));
int32_t timePeriod = 0, max_timePeriod = 0;
motor_data_t moto[2];
led_sense_t sense[4];

#if defined(__PRINT_CAPTURE__)
motor_rec_t motoRec[2];
#endif

void motor_config(motor_data_t *mo, int32_t duty, int32_t fakeKr, int32_t fakeKp, uint32_t pwmChannel, uint32_t capChannel, uint32_t pwmBack, uint32_t pwmPort, uint32_t capPort) {
    mo->stat.fakeKr = fakeKr;
    mo->stat.fakeKp = fakeKp;
    mo->stat.duty = duty;
    mo->stat.target = 0;
    mo->conf.pwmChannel = pwmChannel;
    mo->conf.capChannel = capChannel;
    mo->conf.pwmBack = pwmBack;
    mo->conf.pwmPort = pwmPort;
    mo->conf.capPort = capPort;
}

void led_sense_config(led_sense_t *se, uint32_t port, uint32_t pinx, uint32_t adcPort, uint32_t adcChannel) {
    se->port = port;
    se->pinx = pinx;
    se->adcChannel = adcChannel;
    PORT_Set_Func(adcPort);
    PORT_Enable_Clock(se->port);
    GPIO_Init(se->port, se->pinx, GPIO_Output_0);
}

void setDuty(motor_data_t *mo) {
#if !defined(__OPEN_LOOP__)
    int64_t curSpeed = 0, curDuty;
#if defined(__FILTER__)
		int64_t j, curDelta, minDelta, maxDelta;
    for (j=0; j<FILTER_DEPTH; j++) {
        curDelta = (uint32_t)(mo->deltas[j]);
        curSpeed += (int64_t)curDelta;
        if (maxDelta < curDelta) maxDelta = curDelta;
        if (minDelta > curDelta) minDelta = curDelta;
    }
    curSpeed = ((uint32_t)(-1)) / ((curSpeed - maxDelta - minDelta) / (FILTER_DEPTH - 2));
#else
    curSpeed = ((uint32_t)(-1)) / (uint32_t)(mo->deltas[mo->curIdx]);
#endif
    curDuty = (((int64_t)mo->stat.fakeKr) * ((int64_t)mo->stat.target) + ((int64_t)mo->stat.fakeKp) * (((int64_t)mo->stat.target) - curSpeed)) / 1000 / FAKERATE;
    if (curDuty > 100) mo->stat.duty = 100;
    else if (curDuty < 0) mo->stat.duty = 0;
    else mo->stat.duty = (uint32_t)curDuty;
    TPM_PWM_Set_DutyPercent(TPM_PWM_ID, mo->conf.pwmChannel, 100 - mo->stat.duty);
#endif
}

void setLED(int32_t initFrontNear, int32_t initFrontFar, int32_t initSideNear) {
    int i;
    // config sense0, sense1 near -----------------
    LIGHT_BIT(4); for(i = 0; i < 10000000; i++);
    LIGHT_BIT(1); for(i = 0; i < 50000000; i++);
    LIGHT_BIT(0);
    sense[0].boundNear = 0;
    sense[1].boundNear = 0;
    for (i=0; i<10; i++) {
        ADC_Channel_Select(0, sense[0].adcChannel);
        while(!ADC_Get_Conversion_Complete_Flag(0));
        sense[0].boundNear += ADC_Read(0);
        ADC_Channel_Select(0, sense[1].adcChannel);
        while(!ADC_Get_Conversion_Complete_Flag(0));
        sense[1].boundNear += ADC_Read(0);
    }
    sense[0].boundNear = sense[0].boundNear / 10;
    sense[1].boundNear = sense[1].boundNear / 10;
    // check if use default value -----------------
    if (sense[0].boundNear >= initFrontFar || sense[1].boundNear >= initFrontFar) {
        for(i = 0; i < 2; i++) {
            sense[i].boundNear = initFrontNear;
            sense[i].boundFar = initFrontFar;
            sense[i].currentVal = sense[i].boundFar;
            sense[i].currentDelta = sense[i].boundFar - sense[i].boundNear;
        }
        for(i = 2; i < 4; i++) {
            sense[i].boundNear = initSideNear;
            sense[i].boundFar = 0xFFFF;
            sense[i].currentVal = sense[i].boundFar;
            sense[i].currentDelta = sense[i].boundFar - sense[i].boundNear;
        }
        return;
    }
    // config sense0, sense1 far -----------------
    LIGHT_BIT(2); for(i = 0; i < 50000000; i++);
    LIGHT_BIT(0);
    sense[0].boundFar = 0;
    sense[1].boundFar = 0;
    for (i=0; i<10; i++) {
        ADC_Channel_Select(0, sense[0].adcChannel);
        while(!ADC_Get_Conversion_Complete_Flag(0));
        sense[0].boundFar += ADC_Read(0);
        ADC_Channel_Select(0, sense[1].adcChannel);
        while(!ADC_Get_Conversion_Complete_Flag(0));
        sense[1].boundFar += ADC_Read(0);
    }
    sense[0].boundFar = sense[0].boundFar / 10;
    sense[1].boundFar = sense[1].boundFar / 10;
    // config sense[2] near -------------------------
    LIGHT_BIT(6); for(i = 0; i < 10000000; i++);
    LIGHT_BIT(1); for(i = 0; i < 50000000; i++);
    LIGHT_BIT(0);
    ADC_Channel_Select(0, sense[2].adcChannel);
    while(!ADC_Get_Conversion_Complete_Flag(0));
    sense[2].boundNear = ADC_Read(0);
    sense[2].boundFar = 0xFFFF;
    // config sense[3] near -------------------------
    LIGHT_BIT(6); for(i = 0; i < 10000000; i++);
    LIGHT_BIT(1); for(i = 0; i < 50000000; i++);
    LIGHT_BIT(0);
    ADC_Channel_Select(0, sense[3].adcChannel);
    while(!ADC_Get_Conversion_Complete_Flag(0));
    sense[3].boundNear = ADC_Read(0);
    sense[3].boundFar = 0xFFFF;
    // finish initial ------------------------------
    for(i = 0; i < 4; i++) {
        if (sense[i].boundFar < sense[i].boundNear) {
            sense[i].currentVal = sense[i].boundFar;
            sense[i].boundFar = sense[i].boundNear;
            sense[i].boundNear = sense[i].currentVal;
        }
        sense[i].currentVal = sense[i].boundFar;
        sense[i].currentDelta = sense[i].boundFar - sense[i].boundNear;
    }
}

// DO NOT use TPM_ISR = &__void_function in this function, or unknown mistake would take account
// as the encoder have 30 * 13 periods per cycle, which means one period equals 1/30/13 cycle
// cycle_rate (rather to say CR ) = 1 / 30 / 13 / (record_delta / (TPM_clock / prescale)) (cycle/second) 
// let FakeCR = CR * ( MAX_UINT / (TPM_clock / prescale) * 30 * 13), FakeCR = MAX_UINT / record_data;
// FakeCR would be the max CR in the field of uint32_t, which may be the most accurate one
// when timer overflow and channel interrupt cames together, the case is hard:
// whether the overflow is before or after channel interrupt becomes a problem 
// this problem can not be avoided but can be fixed in some cases with some assuming
// if print out, we want to wait the first signal to get the exact responding curve,
// but if no print, we want this function keeps setting pwm that once target turns none-zero, pwm works. 
// we added FixedFeedBack System, as we assume motor system is a L-R-E system with order 2
// The Final FeedBack System is like 
// (Motor Step Response) As = K / ((t1 * s + 1) * (t2 * s + 1))
// Speed = conv(h_As, duty); duty = Kr * target + Kp * (target - Speed); Kr = 1 / K;
// Kp = (t1 + t2)^2 / ( t1 * t2 * 4 * zeta^2 ) / K;
// zeta would control the system (z=1.0 should be the best.) 
// but remember: This is Discrete system, so just get a proper zeta is ok.
void Speed_Capture_ISR(void) { 
    int32_t i, j, overflow = 0, currentBase, lastDelta;
    if (timePeriod == 0) { // first time set timePeriod
        timePeriod = TPM_Get_Period(TPM_CAPTURE_ID);
        max_timePeriod = MAX_OVERFLOW_COUNT * timePeriod;
        // padding with max_timerPeriod, used when filting the data
        for(i=0; i<2; i++) {
            moto[i].timeDelta = 0;
            moto[i].passOverflow = 0;
            moto[i].lastBase = -1;
            moto[i].curIdx = 0;
            moto[i].nextIdx = 1;
            for(j=0; j<FILTER_DEPTH; j++) {
                moto[i].deltas[j] = max_timePeriod;
            }
        }
    }
    if (TPM_Timer_Get_IT_Flag(TPM_CAPTURE_ID)) {
        TPM_Timer_Clear_IT_Flag(TPM_CAPTURE_ID);   
        overflow = 1;
        for (i=0; i<2; i++) {
        // for(i=1; i>=0; i--) {
            moto[i].timeDelta += timePeriod;
            if (moto[i].timeDelta >= max_timePeriod) {
                moto[i].timeDelta = 0;
                moto[i].lastBase = TPM_Get_Current_Count(TPM_CAPTURE_ID);
                moto[i].curIdx = moto[i].nextIdx;
                moto[i].nextIdx = (moto[i].nextIdx == (FILTER_DEPTH - 1)) ? 0 : (moto[i].nextIdx + 1);
                moto[i].deltas[moto[i].curIdx] = max_timePeriod;
                setDuty(&(moto[i]));
                TPM_Channel_Clear_IT_Flag(TPM_CAPTURE_ID, moto[i].conf.capChannel);
#if defined(__PRINT_CAPTURE__)
                if (motoRec[i].start) {
                    motoRec[i].rec[motoRec[i].idx] = moto[i].deltas[moto[i].curIdx];
                    motoRec[i].idx ++;
                } 
#endif
            }
        }
    }
    for (i=0; i<2; i++) {
    // for(i=1; i>=0; i--) {
        if (TPM_Channel_Get_IT_Flag(TPM_CAPTURE_ID, moto[i].conf.capChannel)) {
            TPM_Channel_Clear_IT_Flag(TPM_CAPTURE_ID, moto[i].conf.capChannel);
            // get the delta value, the first value is a base but not usable
            currentBase = TPM_Capture_Get_Current_Count(TPM_CAPTURE_ID, moto[i].conf.capChannel);
            lastDelta = moto[i].deltas[moto[i].curIdx];
            moto[i].curIdx = moto[i].nextIdx;
            moto[i].nextIdx = (moto[i].nextIdx == (FILTER_DEPTH - 1)) ? 0 : (moto[i].nextIdx + 1);
            moto[i].timeDelta = moto[i].passOverflow + moto[i].timeDelta + currentBase - moto[i].lastBase;
            moto[i].passOverflow = 0;             
            // do not now why, but it works. which suggests that time overflow could also come late  
            if (moto[i].timeDelta < 0) {
                moto[i].passOverflow = -timePeriod;
                moto[i].timeDelta = moto[i].timeDelta + timePeriod;
            }
            if (overflow) {
                if (currentBase <=  moto[i].lastBase);  // surely right
                else if (moto[i].timeDelta - lastDelta >= (timePeriod >> 1)) {
                    moto[i].timeDelta = moto[i].timeDelta - timePeriod;
                    moto[i].passOverflow = timePeriod;
                }
            }
            // if (moto[i].timeDelta > lastDelta * 8 || lastDelta > moto[i].timeDelta * 8) {
            //     moto[i].timeDelta = lastDelta;
            // }
            moto[i].deltas[moto[i].curIdx] = moto[i].timeDelta;
            setDuty(&(moto[i]));
            moto[i].timeDelta = 0;
            moto[i].lastBase = currentBase;
#if defined(__PRINT_CAPTURE__)
            if (motoRec[i].start) {
                motoRec[i].rec[motoRec[i].idx] = moto[i].deltas[moto[i].curIdx];
                motoRec[i].idx ++;
            } else {
                motoRec[i].start = 1;
            }
#endif

        }
    }

#if defined(__PRINT_CAPTURE__)
    if (motoRec[0].idx == MAX_RECORD - 1 || motoRec[1].idx == MAX_RECORD - 1) {
        TPM_Stop(TPM_CAPTURE_ID);
        TPM_Timer_IT_Disable(TPM_CAPTURE_ID);
        TPM_Channel_IT_Disable(TPM_CAPTURE_ID, moto[0].conf.capChannel);
        TPM_Channel_IT_Disable(TPM_CAPTURE_ID, moto[1].conf.capChannel);
        TPM_Channel_Clear_IT_Flag(TPM_CAPTURE_ID, moto[0].conf.capChannel);
        TPM_Channel_Clear_IT_Flag(TPM_CAPTURE_ID, moto[1].conf.capChannel);
        TPM_Stop(TPM_PWM_ID);
        TPM_PWM_Set_DutyPercent(TPM_PWM_ID, moto[0].conf.pwmChannel, 100 - 0);
        TPM_PWM_Set_DutyPercent(TPM_PWM_ID, moto[1].conf.pwmChannel, 100 - 0);
        TPM_Start(TPM_PWM_ID, 0);
        motoRec[0].enPrint = 1;        
        motoRec[1].enPrint = 1;        
    } 
#endif 
}

// if speed == (v, a * v) and car_width == w and turn_radius == (x, x + w) 
// car would turn left, and theta == v / x  == a * v / (x + w), x =  w / (a-1), theta = (a*v - v) / w
void play(void) {
    int32_t sense_order = 0, test = 0;
#if defined(__PRINT_CAPTURE__)
    int32_t i, j;
#endif
    while(1) {
        led_sense_t *se = &(sense[sense_order]);        
#if defined(__PRINT_CAPTURE__)
        if (motoRec[0].enPrint || motoRec[1].enPrint) {
            for (i=0; i<2; i++) {
                shell_printf("\n\n\n---- test = %d, i = %d, final duty = %d, target delta = %d ----\n", 
                    test, i, moto[i].stat.duty, (uint32_t)(-1) / moto[i].stat.target);
                for(j = 0; j < motoRec[i].idx; j++) {
                    shell_printf("%d  ", motoRec[i].rec[j]);
                }
                motoRec[i].idx = 0;
                motoRec[i].start = 0;
                motoRec[i].enPrint = 0;
            }
            timePeriod = 0;     // to re init in isr 
#if defined(__OPEN_LOOP__) 
            if (test < 3) {
                moto[0].stat.duty = 30 * test + 20;
                moto[1].stat.duty = 30 * test + 20;
            } else {
                while(1);
            }
            TPM_Stop(TPM_PWM_ID);
            TPM_PWM_Set_DutyPercent(TPM_PWM_ID, moto[0].conf.pwmChannel, 100 - moto[0].stat.duty);
            TPM_PWM_Set_DutyPercent(TPM_PWM_ID, moto[1].conf.pwmChannel, 100 - moto[1].stat.duty);
            TPM_Start(TPM_PWM_ID, 0);
#else
            if (test < 5) {
                moto[0].stat.target = (test + 1) * FAKERATE;
                moto[1].stat.target = (test + 1) * FAKERATE;
            } else {
                while(1);
            }
#endif       
            test += 1;

            TPM_Timer_Clear_IT_Flag(TPM_CAPTURE_ID);
            TPM_Channel_Clear_IT_Flag(TPM_CAPTURE_ID, moto[0].conf.capChannel);
            TPM_Channel_Clear_IT_Flag(TPM_CAPTURE_ID, moto[1].conf.capChannel);
            TPM_Timer_IT_Enable(TPM_CAPTURE_ID);
            TPM_Channel_IT_Enable(TPM_CAPTURE_ID, moto[0].conf.capChannel);
            TPM_Channel_IT_Enable(TPM_CAPTURE_ID, moto[1].conf.capChannel);
            TPM_Start(TPM_CAPTURE_ID, 0);
        }
#else
        ADC_Channel_Select(0, se->adcChannel);
        while (!ADC_Get_Conversion_Complete_Flag(0));
        se->currentVal = ADC_Read(0);
        se->currentDelta = (se->currentVal - se->boundNear);
        if (sense[0].currentDelta <= 0 && sense[1].currentDelta <= 0 ) {
            moto[0].stat.target = 0; 
            moto[1].stat.target = 0;
            LIGHT_BIT(7);
        } else if (sense[0].currentDelta <= 0 ) {
            moto[0].stat.target = 4 * maxSpeed / 5; 
            moto[1].stat.target = 1 * maxSpeed / 5;
            LIGHT_BIT(6);
        } else if (sense[1].currentDelta <= 0 ) {
            moto[0].stat.target = 1 * maxSpeed / 5; 
            moto[1].stat.target = 4 * maxSpeed / 5;
            LIGHT_BIT(5);
        } else if (sense[0].currentVal < sense[0].boundFar || sense[1].currentVal < sense[1].boundFar) {
            test = 0x1000 * sense[0].currentDelta / (sense[0].boundFar - sense[0].boundNear) - 0x1000 * sense[1].currentDelta / (sense[1].boundFar - sense[1].boundNear);
            if (test < 0) {
                moto[0].stat.target = 4 * maxSpeed / 5; 
                moto[1].stat.target = 1 * maxSpeed / 5;
                LIGHT_BIT(6);
            } else if (test > 0) {
                moto[0].stat.target = 1 * maxSpeed / 5; 
                moto[1].stat.target = 4 * maxSpeed / 5;
                LIGHT_BIT(5);
            } else {
                moto[0].stat.target = 4 * maxSpeed / 5; 
                moto[1].stat.target = 4 * maxSpeed / 5;
                LIGHT_BIT(3);
            }
        } else if (sense[2].currentDelta <= 0) {
            moto[0].stat.target = 3 * maxSpeed / 5;
            moto[1].stat.target = 2 * maxSpeed / 5;
            LIGHT_BIT(4);
        } else if (sense[3].currentDelta <= 0) {
            moto[0].stat.target = 2 * maxSpeed / 5;
            moto[1].stat.target = 3 * maxSpeed / 5;
            LIGHT_BIT(2);
        } else {
            moto[0].stat.target = 5 * maxSpeed / 5;
            moto[1].stat.target = 5 * maxSpeed / 5;
            LIGHT_BIT(0);
        }
        sense_order = (sense_order + 1) % 4;
#if defined(__OPEN_LOOP__)
        if (moto[0].stat.duty > moto[0].stat.target * 150 / maxSpeed) moto[0].stat.duty = 0;
        else moto[0].stat.duty = moto[0].stat.target * 100 / maxSpeed;
        if (moto[1].stat.duty > moto[1].stat.target * 150 / maxSpeed) moto[1].stat.duty = 0;
        else moto[1].stat.duty = moto[1].stat.target * 100 / maxSpeed;
        TPM_PWM_Set_DutyPercent(TPM_PWM_ID, moto[0].conf.pwmChannel, 100 - moto[0].stat.duty);
        TPM_PWM_Set_DutyPercent(TPM_PWM_ID, moto[1].conf.pwmChannel, 100 - moto[1].stat.duty);
#endif
#endif
    }
}

int main() {
    __asm volatile ("cpsid i");    
    // motor config
    motor_config(
        &(moto[0]), 
        0, (uint32_t)(1000 * 18.4843), (uint32_t)(1000 * 34.8607),
        0, 0, (4 << 16) + 29,  PORT_TPM2_CH0_PTE22_Mux3, PORT_TPM1_CH0_PTE20_Mux3
    );
    motor_config(
        &(moto[1]), 
        0, (uint32_t)(1000 * 18.4843), (uint32_t)(1000 * 34.8607),
        1, 1, (4 << 16) + 30, PORT_TPM2_CH1_PTE23_Mux3, PORT_TPM1_CH1_PTE21_Mux3
    );

    // LED Sense config
    ADC_Enable(); 
    ADC_Set_Resolution_Mode(ADC_Resolution_Single_16bit); 
    ADC_Low_Power_Enable(); 
    ADC_Set_Long_Sample_Mode(ADC_LongSample_Cycle24);
    // ADC_Conversion_Complete_IT_Enable(0);
    // NVIC_Init(ADC0_IRQn, PIT_CAPTURE_IRQn_PRIORITY);
    // PIT_Init(0, DEFAULT_BUS_CLOCK / LED_SENSE_FREQ / 2, PIT_CAPTURE_IRQn_PRIORITY, 0);
    // PIT_Channel_Enable(0);
    GPIO_Init(PORT_B, 18, GPIO_Output_1);
    GPIO_Init(PORT_B, 19, GPIO_Output_1);
    GPIO_Init(PORT_D, 1, GPIO_Output_1);
    led_sense_config(&sense[0], PORT_E, 2, PORT_ADC0_SE8_PTB0_Mux0, ADC_Channel_AD8);
    led_sense_config(&sense[1], PORT_E, 3, PORT_ADC0_SE9_PTB1_Mux0, ADC_Channel_AD9);
    led_sense_config(&sense[2], PORT_E, 4, PORT_ADC0_SE12_PTB2_Mux0, ADC_Channel_AD12);
    led_sense_config(&sense[3], PORT_E, 5, PORT_ADC0_SE13_PTB3_Mux0, ADC_Channel_AD13);

    // PWM config, attention: using slow decay, so duty should be 100 - duty 
    TPM_Enable_Clock(TPM_PWM_ID, TPM_Clock_MCGFLLCLK);
    TPM_Set_Prescale(TPM_PWM_ID, TPM_Prescale_Divide_4);
    TPM_Set_Count_Mode(TPM_PWM_ID, TPM_EdgeAligned);    
    TPM_Set_Freq(TPM_PWM_ID, TPM_PWM_FREQ, DEFAULT_SYSTEM_CLOCK);
    TPM_Start(TPM_PWM_ID, 0);
    PORT_Set_Func(moto[0].conf.pwmPort);
    PORT_Set_Func(moto[1].conf.pwmPort);
    TPM_Channel_Set_Mode(TPM_PWM_ID, moto[0].conf.pwmChannel, TPM_PWM_HighLevel);
    TPM_Channel_Set_Mode(TPM_PWM_ID, moto[1].conf.pwmChannel, TPM_PWM_HighLevel);
    TPM_PWM_Set_DutyPercent(TPM_PWM_ID, moto[0].conf.pwmChannel, 100 - moto[0].stat.duty);
    TPM_PWM_Set_DutyPercent(TPM_PWM_ID, moto[1].conf.pwmChannel, 100 - moto[1].stat.duty);
    GPIO_Init((moto[0].conf.pwmBack >> 16) & 0xFF, moto[0].conf.pwmBack & 0xFF, GPIO_Output_1);
    GPIO_Init((moto[1].conf.pwmBack >> 16) & 0xFF, moto[1].conf.pwmBack & 0xFF, GPIO_Output_1);

#if (defined(__PRINT_CAPTURE__) || !defined(__OPEN_LOOP__))
    // Capture config, in TPM_Capture_Mode, it would auto set to TPM_EdgeAligned 
    TPM_Enable_Clock(TPM_CAPTURE_ID, TPM_Clock_MCGFLLCLK);
    TPM_Set_Prescale(TPM_CAPTURE_ID, TPM_Prescale_Divide_4);
    TPM_Set_Count_Mode(TPM_CAPTURE_ID, TPM_EdgeAligned);    
    TPM_Set_Period(TPM_CAPTURE_ID, TPM_CAPTURE_TIME_PERIOD);
    TPM_Start(TPM_CAPTURE_ID, 0);
    PORT_Set_Func(moto[0].conf.capPort);
    PORT_Set_Func(moto[1].conf.capPort);
    TPM_Capture_Mode(TPM_CAPTURE_ID, moto[0].conf.capChannel, TPM_Capture_RisingEdge);
    TPM_Capture_Mode(TPM_CAPTURE_ID, moto[1].conf.capChannel, TPM_Capture_RisingEdge);
    TPM_Timer_IT_Enable(TPM_CAPTURE_ID);
    TPM_Channel_IT_Enable(TPM_CAPTURE_ID, moto[0].conf.capChannel);
    TPM_Channel_IT_Enable(TPM_CAPTURE_ID, moto[1].conf.capChannel);
    NVIC_Init(TPM_CAPTURE_IRQn_ID, TPM_CAPTURE_IRQn_PRIORITY);
#endif

#if defined(__PRINT_CAPTURE__)    
    UART_Enable_Clock(0);
    UART_Tx_Disable(0);
    UART_Rx_Disable(0);
    UART0_Set_Clock_Source(UART0_Clock_MCGFLLCLK);
    UART0_PORT_Enable();
    UART_Set_Baud(0, 9600, DEFAULT_SYSTEM_CLOCK, 16);
    UART_Set_Parity(0, UART_No_Parity);
    UART_Set_StopBit(0, 1);
    UART_Tx_Enable(0);
    UART_Rx_Enable(0);
    motoRec[0].enPrint = 1;
    motoRec[0].start = 0;
    motoRec[0].idx = 0;
    motoRec[1].enPrint = 1;
    motoRec[1].start = 0;
    motoRec[1].idx = 0;
#else
    setLED(0x3000, 0xD000, 0x3000);
#endif

    // NVIC_ClearPendingIRQ(ADC0_IRQn);
    // NVIC_ClearPendingIRQ(PIT_IRQn);
    NVIC_ClearPendingIRQ(TPM_CAPTURE_IRQn_ID);
    __asm volatile ("cpsie i");
    play();
}

#endif


