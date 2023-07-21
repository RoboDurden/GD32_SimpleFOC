#include <Arduino.h>
#include <HardwareTimer.h>
#include <SimpleFOC.h>

#define DEBUG_STLINK rtt              // Uncomment to enable DEBUG over stlink dongle
//#define DEBUG_UART  Serial2

#ifdef DEBUG_UART
	#define SERIALDEBUG DEBUG_UART
#else
	#ifdef DEBUG_STLINK
		#define SERIALDEBUG DEBUG_STLINK
	#endif	
#endif	
#ifdef SERIALDEBUG
	#define DEBUG(code)	{code}
	#define OUT(s)	{SERIALDEBUG.print(s);}
	#define OUT2(s,i)	{SERIALDEBUG.print(s);SERIALDEBUG.print(": ");SERIALDEBUG.print(i);}
	#define OUT2T(s,i)	{SERIALDEBUG.print(s);SERIALDEBUG.print(": ");SERIALDEBUG.print(i);SERIALDEBUG.print("  ");}
	#define OUT2N(s,i)	{SERIALDEBUG.print(s);SERIALDEBUG.print(": ");SERIALDEBUG.print(i);SERIALDEBUG.println();}
	#define OUTN(s)	{SERIALDEBUG.println(s);}
	#define OUTI4(i){if (i<10) SERIALDEBUG.print("   ");	else if (i<100)	SERIALDEBUG.print("  ");	else if (i<1000)	SERIALDEBUG.print(" ");	SERIALDEBUG.print(i);}
#else
	#define DEBUG(code)
	#define OUT(s)
	#define OUT2(s,i)
	#define OUT2T(s,i)
	#define OUT2N(s,i)
	#define OUTN(s)
	#define OUTI4(i)
#endif
#ifdef DEBUG_STLINK
  #include <RTTStream.h>
  RTTStream rtt;
#endif



#define LED_GREEN PA15
#define LED_ORANGE PA12
#define LED_RED PB3

HardwareTimer t(TIMER2);

unsigned long iCount = 0;
#define TIMER2_HZ 8000

long iMicrosTimerCb = 0;
long iMicrosAdcReady = 0;

void timer_cb() 
{
    digitalWrite(LED_RED, digitalRead(LED_RED) ^ 1);
	iCount++;
    digitalWrite(LED_ORANGE, (iCount % TIMER2_HZ) < (TIMER2_HZ/4) );

	// Start ADC conversion
	adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
	iMicrosTimerCb = getCurrentMicros();
}

// ADC defines
#define VBATT_PIN	GPIO_PIN_4
#define VBATT_PORT GPIOA
#define VBATT_CHANNEL ADC_CHANNEL_4
#define CURRENT_DC_PIN	GPIO_PIN_6
#define CURRENT_DC_PORT GPIOA
#define CURRENT_DC_CHANNEL ADC_CHANNEL_6

// DMA (ADC) structs
dma_parameter_struct dma_init_struct_adc;
// ADC buffer struct
typedef struct
{
  uint16_t v_batt;
	uint16_t current_dc;
} adc_buf_t;

adc_buf_t adc_buffer;

extern "C"
{
	void RCU_init(void)
	{
		// enable GPIOA clock
		rcu_periph_clock_enable(RCU_GPIOA);
		// enable ADC clock
		rcu_periph_clock_enable(RCU_ADC);
		// enable DMA clock
		rcu_periph_clock_enable(RCU_DMA);
		// config ADC clock
		rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
	}


	void DMA_init(void)
	{
	// Configure ADC clock (APB2 clock is DIV1 -> 72MHz, ADC clock is DIV6 -> 12MHz)
		rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
		
		// Interrupt channel 0 enable
		nvic_irq_enable(DMA_Channel0_IRQn, 1, 0);
		
		// Initialize DMA channel 0 for ADC
		dma_deinit(DMA_CH0);
		dma_init_struct_adc.direction = DMA_PERIPHERAL_TO_MEMORY;
		dma_init_struct_adc.memory_addr = (uint32_t)&adc_buffer;
		dma_init_struct_adc.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
		dma_init_struct_adc.memory_width = DMA_MEMORY_WIDTH_16BIT;
		dma_init_struct_adc.number = 2;
		dma_init_struct_adc.periph_addr = (uint32_t)&ADC_RDATA;
		dma_init_struct_adc.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
		dma_init_struct_adc.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
		dma_init_struct_adc.priority = DMA_PRIORITY_ULTRA_HIGH;
		dma_init(DMA_CH0, &dma_init_struct_adc);
		
		// Configure DMA mode
		dma_circulation_enable(DMA_CH0);
		dma_memory_to_memory_disable(DMA_CH0);
		
		// Enable DMA transfer complete interrupt
		dma_interrupt_enable(DMA_CH0, DMA_CHXCTL_FTFIE);
		
		// At least clear number of remaining data to be transferred by the DMA 
		dma_transfer_number_config(DMA_CH0, 2);
		
		// Enable DMA channel 0
		dma_channel_enable(DMA_CH0);
	}	

	// Gen2 code that needs a TIMER to trigger the adc conversion which then triggers an event handler
	void ADC_init(void)
	{
		adc_channel_length_config(ADC_REGULAR_CHANNEL, 2);
		adc_regular_channel_config(0, VBATT_CHANNEL, ADC_SAMPLETIME_13POINT5);
		adc_regular_channel_config(1, CURRENT_DC_CHANNEL, ADC_SAMPLETIME_13POINT5);
		adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
		
		// Set trigger of ADC
		adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
		adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);	// ADC_EXTTRIG_REGULAR_T2_TRGO or ADC_EXTTRIG_REGULAR_NONE
		
		// Disable the temperature sensor, Vrefint and vbat channel
		adc_tempsensor_vrefint_disable();
		adc_vbat_disable();

		// ADC analog watchdog disable
		adc_watchdog_disable();
		
		// Enable ADC (must be before calibration)
		adc_enable();
		
		// Calibrate ADC values
		adc_calibration_enable();
		
		// Enable DMA request
		adc_dma_mode_enable();

		adc_interrupt_enable(ADC_INT_EOC);
		//nvic_irq_enable(ADC_CMP_IRQn, 0U);    
		adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
		
		// Set ADC to scan mode
		adc_special_function_config(ADC_SCAN_MODE, ENABLE);		//  without extern"C" this makes the MCU hang: processor receives an unexpected interrupt
	}
	// This function handles DMA_Channel0_IRQHandler interrupt
	// Is called, when the ADC scan sequence is finished
	void DMA_Channel0_IRQHandler(void)
	{
		iMicrosAdcReady = getCurrentMicros() - iMicrosTimerCb;

		//motor.loopFOC();
		digitalWrite(LED_GREEN, (iCount % TIMER2_HZ) < (TIMER2_HZ/2) );

		if (dma_interrupt_flag_get(DMA_CH0, DMA_INT_FLAG_FTF))
			dma_interrupt_flag_clear(DMA_CH0, DMA_INT_FLAG_FTF);        
	}


	// code found at https://gitee.com/yhalin/gd32-e230/blob/master/code/simple_app/main_adjust_Joystick.c
	void ADC_init2(void)
	{
		// ADC contineous function enable
		adc_special_function_config(ADC_SCAN_MODE, ENABLE);
		adc_special_function_config(ADC_CONTINUOUS_MODE, ENABLE);
		// ADC trigger config
		adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE); 
		// ADC data alignment config
		adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
		// ADC channel length config
		adc_channel_length_config(ADC_REGULAR_CHANNEL, 2U);
	
		// ADC regular channel config
		adc_regular_channel_config(0U, VBATT_CHANNEL, ADC_SAMPLETIME_55POINT5);
		adc_regular_channel_config(1U, CURRENT_DC_CHANNEL, ADC_SAMPLETIME_55POINT5);

		adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);

		// enable ADC interface
		adc_enable();
		delay(1);
		// ADC calibration and reset calibration
		adc_calibration_enable();

		// ADC DMA function enable
		adc_dma_mode_enable();
		// ADC software trigger enable

		adc_interrupt_enable(ADC_INT_EOC);
		nvic_irq_enable(ADC_CMP_IRQn, 0,0);    
		adc_software_trigger_enable(ADC_REGULAR_CHANNEL);	// without extern"C" this makes the MCU hang: processor receives an unexpected interrupt
	}
	long iMicrosAdcCmpCb = 0;
	void ADC_CMP_IRQHandler(void)
	{
		long iNow = getCurrentMicros();
		iMicrosAdcReady = iNow - iMicrosAdcCmpCb;
		iMicrosAdcCmpCb = iNow;

		adc_interrupt_flag_clear(ADC_INT_EOC);
		digitalWrite(LED_GREEN, (iCount % TIMER2_HZ) < (TIMER2_HZ/2) );
		//OUT2T(adc_buffer.v_batt,adc_buffer.current_dc);
		//OUTN("yes")
	}

}

void TIMER_Init(void)
{
    t.setPeriodTime(TIMER2_HZ, FORMAT_HZ);
    t.attachInterrupt(&timer_cb);
    t.start();
}


void setup()
{
	#ifdef DEBUG_STLINK
	SimpleFOCDebug::enable(&rtt);
	#endif
	OUTN("Hello gd32-adc :-)")

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);

	delay(500);	// wait for the citical init stuff to happen - might be in conflict with st-link flash..

	adc_buffer.v_batt = adc_buffer.current_dc = 42;

	gpio_mode_set(VBATT_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VBATT_PIN);
	gpio_mode_set(CURRENT_DC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CURRENT_DC_PIN);
	RCU_init();
	DMA_init();
	ADC_init();	// the Gen2 style :-)
	//ADC_init2();	// some other code found in the net

	TIMER_Init();

}

// ADC value conversion defines
#define MOTOR_AMP_CONV_DC_AMP 0.201465201465  // 3,3V * 1/3 - 0,004Ohm * IL(ampere) = (ADC-Data/4095) *3,3V
#define ADC_BATTERY_VOLT      0.024169921875 	// V_Batt to V_BattMeasure = factor 30: ( (ADC-Data/4095) *3,3V *30 )
int16_t offsetcount = 0;
int16_t offsetdc = 2000;

void loop() 
{
	// Calibrate ADC offsets for the first 1000 cycles
	if (offsetcount < 1000)
	{  
		offsetcount++;
		offsetdc = (adc_buffer.current_dc + offsetdc) / 2;
		return;
	}
	float fVoltage = adc_buffer.v_batt * ADC_BATTERY_VOLT;
	float fCurrent = (adc_buffer.current_dc - offsetdc) * MOTOR_AMP_CONV_DC_AMP;

	OUT2T(fVoltage ,fCurrent);
	OUTN(iMicrosAdcReady);
	delay(10);
}