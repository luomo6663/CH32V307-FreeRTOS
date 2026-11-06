#ifndef TEST_CH32_H
#define TEST_CH32_H
#include <stdint.h>
typedef struct { uint32_t CH1CVR,CH2CVR,CH3CVR,PSC,ATRLR,CTLR1,CNT; } TIM_TypeDef;
typedef struct { uint32_t CFGLR,OUTDR; } GPIO_TypeDef;
typedef struct { uint32_t PCFR1; } AFIO_TypeDef;
extern TIM_TypeDef test_tim;
extern GPIO_TypeDef test_gpiob,test_gpiod;
extern AFIO_TypeDef test_afio;
#define TIM3 (&test_tim)
#define GPIOB (&test_gpiob)
#define GPIOD (&test_gpiod)
#define AFIO (&test_afio)
#define GPIO_Pin_0 1U
#define GPIO_Pin_2 4U
#define GPIO_Pin_4 16U
#define GPIO_Pin_5 32U
#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define RCC_APB2Periph_AFIO 1
#define RCC_APB2Periph_GPIOB 2
#define RCC_APB2Periph_GPIOD 4
#define RCC_APB1Periph_TIM3 1
#define GPIO_PartialRemap_TIM3 1
#define GPIO_Speed_50MHz 1
#define GPIO_Mode_Out_PP 1
#define GPIO_Mode_AF_PP 2
#define TIM_CounterMode_Up 1
#define TIM_CKD_DIV1 1
#define TIM_OCMode_PWM1 1
#define TIM_OutputState_Enable 1
#define TIM_OutputState_Disable 0
#define TIM_OCPolarity_High 1
#define TIM_OCPreload_Enable 1
#define TIM_OCMode_Timing 0
#define TIM_EventSource_Update 1
#define TIM_IT_CC4 16
#define TIM3_IRQn 1
typedef struct { uint32_t GPIO_Pin,GPIO_Speed,GPIO_Mode; } GPIO_InitTypeDef;
typedef struct { uint32_t TIM_Prescaler,TIM_Period,TIM_CounterMode,TIM_ClockDivision; } TIM_TimeBaseInitTypeDef;
typedef struct { uint32_t TIM_OCMode,TIM_OutputState,TIM_OCPolarity,TIM_Pulse; } TIM_OCInitTypeDef;
typedef struct { uint32_t PCLK1_Frequency,HCLK_Frequency,PCLK2_Frequency; } RCC_ClocksTypeDef;
typedef struct { uint32_t NVIC_IRQChannel,NVIC_IRQChannelPreemptionPriority,NVIC_IRQChannelSubPriority,NVIC_IRQChannelCmd; } NVIC_InitTypeDef;
extern uint32_t test_pclk,test_hclk;
static inline void GPIO_SetBits(GPIO_TypeDef *g,uint32_t p) { g->OUTDR |= p; }
static inline void GPIO_ResetBits(GPIO_TypeDef *g,uint32_t p) { g->OUTDR &= ~p; }
static inline void RCC_GetClocksFreq(RCC_ClocksTypeDef *c) { c->PCLK1_Frequency=test_pclk;c->HCLK_Frequency=test_hclk;c->PCLK2_Frequency=test_hclk; }
static inline void TIM_TimeBaseInit(TIM_TypeDef *t,TIM_TimeBaseInitTypeDef *b) { t->PSC=b->TIM_Prescaler;t->ATRLR=b->TIM_Period; }
static inline void TIM_SetCompare1(TIM_TypeDef *t,uint16_t v) { t->CH1CVR=v; }
static inline void TIM_SetCompare2(TIM_TypeDef *t,uint16_t v) { t->CH2CVR=v; }
static inline void TIM_SetCompare3(TIM_TypeDef *t,uint16_t v) { t->CH3CVR=v; }
static inline int TIM_GetITStatus(TIM_TypeDef *t,int f) { (void)t;(void)f;return 1; }
#define RCC_APB2PeriphClockCmd(...) ((void)0)
#define RCC_APB1PeriphClockCmd(...) ((void)0)
#define TIM_Cmd(...) ((void)0)
#define TIM_ITConfig(...) ((void)0)
#define GPIO_PinRemapConfig(...) ((void)0)
#define GPIO_Init(...) ((void)0)
#define TIM_OC1Init(...) ((void)0)
#define TIM_OC2Init(...) ((void)0)
#define TIM_OC3Init(...) ((void)0)
#define TIM_OC4Init(...) ((void)0)
#define TIM_OC1PreloadConfig(...) ((void)0)
#define TIM_OC2PreloadConfig(...) ((void)0)
#define TIM_OC3PreloadConfig(...) ((void)0)
#define TIM_ARRPreloadConfig(...) ((void)0)
#define TIM_GenerateEvent(...) ((void)0)
#define TIM_ClearITPendingBit(...) ((void)0)
#define NVIC_Init(...) ((void)0)
#endif
