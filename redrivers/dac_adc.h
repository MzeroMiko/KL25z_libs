#ifndef __DAC_ADC_H__
#define __DAC_ADC_H__

#include "MKL25Z4.h"

// Page 537 461

typedef enum _dac_ref_votage {
    DAC_Ref_VREFH = 0U,
    DAC_Ref_VDDA = 1U,
} dac_ref_votage_t;

typedef enum _dac_buffer_trigger_mode {
    DAC_Buffer_Hardware_Trigger = 0U,
    DAC_Buffer_Software_Trigger = 1U,
} dac_buffer_trigger_mode_t;

typedef enum _dac_buffer_scan_mode {
    DAC_Buffer_Recycle_Mode = 0U,
    DAC_Buffer_One_Scan_Mode = 1U,
} dac_buffer_scan_mode_t;

typedef enum _dac_buffer_interrupt_mode {
    DAC_Buffer_Interrupt_Disable = 0U,
    DAC_Buffer_Interrupt_Bottom = 1U,
    DAC_Buffer_Interrupt_Top = 2U,
    DAC_Buffer_Interrupt_Both = 3U,
} dac_buffer_interrupt_mode_t;

// 537 / 807 in KL25RefMan.pdf // have only one DAC ON PORTE_30 --------------------------------------- //
#define DAC_Enable() SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK; DAC0->C0 |= DAC_C0_DACEN_MASK
#define DAC_Disable() SIM->SCGC6 &= ~SIM_SCGC6_DAC0_MASK; DAC0->C0 &= ~DAC_C0_DACEN_MASK 

#define DAC_PORT_Enable() PORT_Enable_Clock(PORT_E); (PORT(PORT_E)->PCR)[30] &= ~PORT_PCR_MUX_MASK // 162 / 807 in KL25RefMan.pdf, Set MUX as 0b000

#define DAC_Set_Ref_Voltage(ref_vol) DAC0->C0 = (DAC0->C0 & ~DAC_C0_DACRFS_MASK) | DAC_C0_DACRFS(ref_vol)
#define DAC_LowPowerMode_Enable() DAC0->C0 |= DAC_C0_LPEN_MASK
#define DAC_LowPowerMode_Disable() DAC0->C0 &= ~DAC_C0_LPEN_MASK
#define DAC_Push_Data(x, n) DAC0->DAT[x].DATH = ((n) >> 8) & 0x0F; DAC0->DAT[x].DATL = (n) & 0xFF

#define DAC_Buffer_Enable() DAC0->C1 |= DAC_C1_DACBFEN_MASK
#define DAC_Buffer_Disable() DAC0->C1 &= ~DAC_C1_DACBFEN_MASK
#define DAC_Buffer_Trigger_Mode(trigger) DAC0->C0 = DAC0->C0 & ~(DAC_C0_DACTRGSEL_MASK) | DAC_C0_DACTRGSEL(trigger)
#define DAC_Buffer_Set_IT_Mode(it_mode) DAC0->C0 = DAC0->C0 & ~(DAC_C0_DACBTIEN_MASK | DAC_C0_DACBBIEN_MASK) | it_mode
#define DAC_Buffer_Do_Software_Trig() DAC0->C0 |= DAC_C0_DACSWTRG_MASK
#define DAC_BUffer_DMA_Enable() DAC0->C1 |= DAC_C1_DMAEN_MASK
#define DAC_BUffer_DMA_Disable() DAC0->C1 &= ~DAC_C1_DMAEN_MASK
#define DAC_Buffer_Scan_Mode(scan) DAC0->C1 = DAC0->C1 & ~(DAC_C1_DACBFMD_MASK) | DAC_C1_DACBFMD(scan);
#define DAC_Buffer_Upper_Limit(limit) DAC0->C2 = DAC0->C2 & ~DAC_C2_DACBFUP_MASK | DAC_C2_DACBFUP(limit);
#define DAC_Buffer_Set_Read_Index(idx) DAC0->C2 = DAC0->C2 & ~DAC_C2_DACBFRP_MASK | DAC_C2_DACBFRP(idx);
#define DAC_Buffer_Get_Read_Index(idx) ((DAC0->C2 & DAC_C2_DACBFRP_MASK) >> DAC_C2_DACBFRP_SHIFT)
#define DAC_Buffer_Get_IT_Flags(it_mode) ((DAC0->SR >> DAC_SR_DACBFRPBF_SHIFT) & it_mode)
#define DAC_Buffer_Clear_IT_Flags(it_mode) DAC0->SR &= ~(it_mode << DAC_SR_DACBFRPBF_SHIFT) // Writing 0 to a field clears it

#define DAC_Init() DAC_Enable(); DAC_LowPowerMode_Enable(); DAC_PORT_Enable(); DAC_Buffer_Disable();

typedef enum _adc_ref_voltage {
    ADC_Ref_Vref = 0U,
    ADC_Ref_Valt = 1U,
} adc_ref_voltage_t;

typedef enum _adc_clock_source {
    ADC_Bus_Clock = 0U,
    ADC_Bus_Div_Clock = 1U,
    ADC_Alt_Clock = 2U,
    ADC_Async_Clock = 3U,
} adc_clock_source_t;

typedef enum _adc_resolution_mode {
    ADC_Resolution_Single_8bit = 0U,
    ADC_Resolution_Single_12bit = 1U,
    ADC_Resolution_Single_10bit = 2U,
    ADC_Resolution_Single_16bit = 3U,
    ADC_Resolution_Diff_9bit = 0U,
    ADC_Resolution_Diff_13bit = 1U,
    ADC_Resolution_Diff_11bit = 2U,
    ADC_Resolution_Diff_16bit = 3U,
} adc_resolution_mode_t;

typedef enum _adc_clock_divider {
    ADC_Clock_Divider1 = 0U,
    ADC_Clock_Divider2 = 1U,
    ADC_Clock_Divider4 = 2U,
    ADC_Clock_Divider8 = 3U,
} adc_clock_divider_t;

typedef enum _adc_long_sample_mode {
    ADC_LongSample_Cycle24 = 0U,
    ADC_LongSample_Cycle16 = 1U,
    ADC_LongSample_Cycle10 = 2U,
    ADC_LongSample_Cycle6 = 3U, 
    ADC_LongSample_Disabled = 4U,
} adc_long_sample_mode_t;

typedef enum _adc_channel_select {
    ADC_Channel_DAD0 = 0U,
    ADC_Channel_DAD1 = 1U,
    ADC_Channel_DAD2 = 2U,
    ADC_Channel_DAD3 = 3U,
    ADC_Channel_AD4 = 4U,
    ADC_Channel_AD5 = 5U,
    ADC_Channel_AD6 = 6U,
    ADC_Channel_AD7 = 7U,
    ADC_Channel_AD8 = 8U,
    ADC_Channel_AD9 = 9U,
    ADC_Channel_AD10 = 10U,
    ADC_Channel_AD11 = 11U,
    ADC_Channel_AD12 = 12U,
    ADC_Channel_AD13 = 13U,
    ADC_Channel_AD14 = 14U,
    ADC_Channel_AD15 = 15U,
    ADC_Channel_AD16 = 16U,
    ADC_Channel_AD17 = 17U,
    ADC_Channel_AD18 = 18U,
    ADC_Channel_AD19 = 19U,
    ADC_Channel_AD20 = 20U,
    ADC_Channel_AD21 = 21U,
    ADC_Channel_AD22 = 22U,
    ADC_Channel_AD23 = 23U,
    ADC_Channel_Temprature = 26U,
    ADC_Channel_BandGap = 26U,
    ADC_Channel_VREFSH = 29U,
    ADC_Channel_VREFSL = 30U,
    ADC_Channel_Disable = 31U,
} adc_channel_select_t;

typedef enum _adc_hardware_average_mode {
    ADC_Average_Count4 = 0U,
    ADC_Average_Count8 = 1U,
    ADC_Average_Count16 = 2U,
    ADC_Average_Count32 = 3U,
    ADC_Average_Disabled = 4U,
} adc_hardware_average_mode_t;

typedef enum _adc16_hardware_compare_mode
{
    ADC_Compare_LessThanValue1 = 0U,
    ADC_Compare_GreaterThanValue1 = 1U,
    ADC_Compare_LtVal2_GtVal2 = 2U,
    ADC_Compare_GtVal1_LtVal2 = 3U,
    ADC_Compare_Disable = 4U,
} adc16_hardware_compare_mode_t;


#define ADC_Enable() SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK
#define ADC_Disable() SIM->SCGC6 &= ~SIM_SCGC6_ADC0_MASK

#define ADC_Get_Conversion_Complete_Flag(c) ((ADC0->SC1[c] >> ADC_SC1_COCO_SHIFT) & 1U)
#define ADC_Conversion_Complete_IT_Enable(c) ADC0->SC1[c] |= ADC_SC1_AIEN_MASK 
#define ADC_Conversion_Complete_IT_Disable(c) ADC0->SC1[c] &= ~ADC_SC1_AIEN_MASK 
#define ADC_Conversion_Mode_Diff(c) ADC0->SC1[c] |= ADC_SC1_DIFF_MASK 
#define ADC_Conversion_Mode_Single(c) ADC0->SC1[c] &= ~ADC_SC1_DIFF_MASK
#define ADC_Channel_Select(c, code) ADC0->SC1[c] = (ADC0->SC1[c] & ~ADC_SC1_ADCH_MASK) | ADC_SC1_ADCH(code)

#define ADC_Set_Clock_Source(src) ADC0->CFG1 = (ADC0->CFG1 & ~ADC_CFG1_ADICLK_MASK) | ADC_CFG1_ADICLK(src)
#define ADC_Set_Resolution_Mode(mode) ADC0->CFG1 = (ADC0->CFG1 & ~ADC_CFG1_MODE_MASK) | ADC_CFG1_MODE(mode)
#define ADC_Set_Clock_Devide(div) ADC0->CFG1 = (ADC0->CFG1 & ~ADC_CFG1_ADIV_MASK) | ADC_CFG1_ADIV(div)
#define ADC_Low_Power_Enable() ADC0->CFG1 |= ADC_CFG1_ADLPC_MASK
#define ADC_Low_Power_Disable() ADC0->CFG1 &= ~ADC_CFG1_ADLPC_MASK
#define ADC_Set_Long_Sample_Mode(mode) ADC0->CFG1 = (ADC0->CFG1 & ~ADC_CFG1_ADLSMP_MASK) | ADC_CFG1_ADLSMP(!(mode >> 2)); ADC0->CFG2 = (ADC0->CFG2 & ~ADC_CFG2_ADLSTS_MASK) | ADC_CFG2_ADLSTS(mode & 0x3)
#define ADC_High_Speed_Enable() ADC0->CFG2 |= ADC_CFG2_ADHSC_MASK
#define ADC_High_Speed_Disable() ADC0->CFG2 &= ~ADC_CFG2_ADHSC_MASK
#define ADC_Async_Clock_Enable() ADC0->CFG2 |= ADC_CFG2_ADACKEN_MASK
#define ADC_Async_Clock_Disable() ADC0->CFG2 &= ~ADC_CFG2_ADACKEN_MASK
#define ADC_Set_Channel_MUXB() base->CFG2 |= ADC_CFG2_MUXSEL_MASK
#define ADC_Set_Channel_MUXA() base->CFG2 &= ~ADC_CFG2_MUXSEL_MASK
#define ADC_Set_Ref_Voltage(mode) ADC0->SC2 = (ADC0->SC2 & ~ADC_SC2_REFSEL_MASK) | ADC_SC2_REFSEL(mode)
#define ADC_Set_Compare_Mode(mode, value1, value2) ADC0->SC2 = (ADC0->SC2 & ~(ADC_SC2_ACFE_MASK | ADC_SC2_ACFGT_MASK | ADC_SC2_ACREN_MASK)) | ADC_SC2_ACFE(!(mode >> 2)) | ADC_SC2_ACFGT(mode & 0x1) | ADC_SC2_ACREN(mode >> 1); ADC0->CV1 = ADC_CV1_CV(value1); ADC0->CV2 = ADC_CV2_CV(value2)
#define ADC_Sample_Continue() ADC0->SC3 |= ADC_SC3_ADCO_MASK 
#define ADC_Sample_Discrete() ADC0->SC3 &= ~ADC_SC3_ADCO_MASK
#define ADC_Set_Average_Mode(mode) ADC0->SC3 = ADC0->SC3 & ~(ADC_SC3_AVGE_MASK | ADC_SC3_AVGS_MASK) | ADC_SC3_AVGE(!(mode >> 2)) | ADC_SC3_AVGS(mode)

#define ADC_Read(x) (ADC0->R[x])  // Have Sign bits (all the same) in DIFF MODE as the MSB

#define ADC_Init() ADC_Enable(); ADC_Set_Resolution_Mode(ADC_Resolution_Single_16bit); ADC_Low_Power_Enable(); ADC_Set_Long_Sample_Mode(ADC_LongSample_Cycle24)



#endif

#ifdef DEMO

static inline void ADC_init(adc_resolution_mode_t resolution, uint8_t low_power, adc_long_sample_mode_t sample) { 
    ADC_Enable();
    ADC_Set_Resolution_Mode(resolution);    // ADC_Resolution_Single_16bit
    ADC_Set_Long_Sample_Mode(sample);       // ADC_LongSample_Cycle24
    if (low_power) ADC_Low_Power_Enable();
    else ADC_Low_Power_Disable();
}
int DAC_test(void *args[]) {
    int d = (uint32_t)(intarg(args[0]));
    int s = (uint32_t)(intarg(args[1]));
    int i, adc_res;
    d = (d)? d : 0xFFF;
    DAC_Init();
    while (d > 0) {
        DAC_Push_Data(0, d);
        d -= s;
        for(i=0;i<100;i++);
    }
    ADC_Init();
    ADC_Channel_Select(0, ADC_Channel_AD15); // PORT_C 1
    while(!ADC_Get_Conversion_Complete_Flag(0));
    adc_res = ADC_Read(0);
 
    return 0;
}


#endif

