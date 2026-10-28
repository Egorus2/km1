#ifndef HX711_H_
#define HX711_H_


/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD
  Instagram:  http://instagram.com/github.NimaLTD
  Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
  
  Version:    1.1.1
  
  
  Reversion History:
  
  (1.1.1):
    Add power down/up.
  (1.1.0):
    Add structure, Add calibration, Add weight, change names, ...
  (1.0.0):
    First Release.
*/

#ifdef __cplusplus
extern "C" {
#endif
  
#include "main.h"

//####################################################################################################################

typedef struct
{
  GPIO_TypeDef  *clk_gpio;
  GPIO_TypeDef  *dat_gpio;
  uint16_t      clk_pin;
  uint16_t      dat_pin;
  int32_t       offset;
  float         coef;
  uint8_t       lock;    
  
}hx711_t;

//####################################################################################################################

void        hx711_init(volatile hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin);
int32_t     hx711_value(volatile hx711_t *hx711);
int32_t     hx711_value_ave(volatile hx711_t *hx711, uint16_t sample);

void        hx711_coef_set(volatile hx711_t *hx711, float coef);
float       hx711_coef_get(volatile hx711_t *hx711);
void        hx711_calibration(volatile hx711_t *hx711, int32_t value_noload, int32_t value_load, float scale);
void        hx711_tare(volatile hx711_t *hx711, uint16_t sample);
float       hx711_weight(volatile hx711_t *hx711, uint16_t sample);
void        hx711_power_down(volatile hx711_t *hx711);
void        hx711_power_up(volatile hx711_t *hx711);

//####################################################################################################################

#ifdef __cplusplus
}
#endif

#endif 
