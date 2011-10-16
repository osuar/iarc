#include <string.h>
#include "adc.h"

/*! How many samples for each ADC channel.*/
#define SAMPLE_COUNT 100

void adc_ch3_callback(ADC_t *adc, uint8_t ch, adc_result_t res);

/*! Sample storage (all four channels).*/
int16_t adcSamples[4][SAMPLE_COUNT];

uint16_t interrupt_count = 0;
int8_t offset;

int main(void)
{
	struct adc_config           adc_conf;
	struct adc_channel_config       adc_ch_conf_ch0;
	struct adc_channel_config       adc_ch_conf_ch1;
	struct adc_channel_config       adc_ch_conf_ch2;
	struct adc_channel_config       adc_ch_conf_ch3;
    uint16_t        ADCA_cal;

	/* Move stored calibration values to ADC A. */
    /* I can not find any function to insert this calibration data into the ADC
    ADCA_cal = adc_get_calibration_data(ADC_CAL_ADCA); */

	// Clear the configuration structures.
	memset(&adc_conf, 0, sizeof(struct adc_config));
	memset(&adc_ch_conf_ch0, 0, sizeof(struct adc_channel_config));
	memset(&adc_ch_conf_ch1, 0, sizeof(struct adc_channel_config));
	memset(&adc_ch_conf_ch2, 0, sizeof(struct adc_channel_config));
	memset(&adc_ch_conf_ch3, 0, sizeof(struct adc_channel_config));

	/* Set up ADC A to have signed conversion mode and 12 bit resolution. 
       and set ref to internal VCC/1.6 V.*/
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_ON, ADC_RES_12, ADC_REF_VCC);

	/* Set sample rate */
	adc_set_clock_rate(&adc_conf, 200000UL);

    /* Write ADC module configuration */
    adc_write_configuration(&ADCA, &adc_conf);

   	/* Get offset value for ADC A. */
	adcch_set_input(&adc_ch_conf_ch0, ADCCH_POS_PIN1, ADCCH_NEG_PIN1, 1);
    adcch_write_configuration(&ADCA, 0, &adc_ch_conf_ch0);

	adc_enable(&ADCA);

	/* Wait until common mode voltage is stable which take exactly one conversion */
    adc_start_conversion(&ADCA, ADC_CH0);
    adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);

    /* Get offset value */
    adc_start_conversion(&ADCA, ADC_CH0);
    adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
 	offset = adcch_get_result(&ADCA, 0);
    adc_disable(&ADCA);
    
	/* Setup channel 0, 1, 2 and 3 to have single ended input.
	   Set input to the channels in ADC A to be PIN 4, 5, 6 and 7. */
	adcch_set_input(&adc_ch_conf_ch0, ADCCH_POS_PIN4, ADCCH_NEG_NONE, 1);
	adcch_set_input(&adc_ch_conf_ch1, ADCCH_POS_PIN5, ADCCH_NEG_NONE, 1);
	adcch_set_input(&adc_ch_conf_ch2, ADCCH_POS_PIN6, ADCCH_NEG_NONE, 1);
	adcch_set_input(&adc_ch_conf_ch3, ADCCH_POS_PIN7, ADCCH_NEG_NONE, 1);

    /* Write configuration to channels */
    adcch_write_configuration(&ADCA, 0, &adc_ch_conf_ch0);
    adcch_write_configuration(&ADCA, 1, &adc_ch_conf_ch1);
    adcch_write_configuration(&ADCA, 2, &adc_ch_conf_ch2);
    adcch_write_configuration(&ADCA, 3, &adc_ch_conf_ch3);
   
	/* Setup freerunning sweep of all four virtual channels.*/
    adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN_SWEEP, 4, 0);
    adc_write_configuration(&ADCA, &adc_conf);

	/* Enable ADC A with free running mode, VCC reference and signed conversion.*/
	adc_enable(&ADCA);

    for(int i = 0;  i < SAMPLE_COUNT; ++i)
    {
        adc_start_conversion(&ADCA, ADC_CH0);
        adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
  	    adcSamples[0][i] = adcch_get_signed_result(&ADCA, 0) - offset;

        adc_start_conversion(&ADCA, ADC_CH1);
        adc_wait_for_interrupt_flag(&ADCA, ADC_CH1);
    	adcSamples[1][i] = adcch_get_signed_result(&ADCA, 1) - offset;

        adc_start_conversion(&ADCA, ADC_CH2);
        adc_wait_for_interrupt_flag(&ADCA, ADC_CH2);
	    adcSamples[2][i] = adcch_get_signed_result(&ADCA, 2) - offset;

        adc_start_conversion(&ADCA, ADC_CH3);
        adc_wait_for_interrupt_flag(&ADCA, ADC_CH3);
	    adcSamples[3][i] = adcch_get_signed_result(&ADCA, 3) - offset;
    }

    adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 4, 0);
    adc_write_configuration(&ADCA, &adc_conf);
	adc_disable(&ADCA);

}



