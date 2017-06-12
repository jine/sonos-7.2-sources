#ifndef __MPC83XX_GPIO_H__
#define __MPC83XX_GPIO_H__

#include <linux/types.h>

/* GPIO Registers */
#define MPC8315_GPIO_OFF           0x00000C00
#define MPC8315_GPIO_GPDIR         0x00000000
#define MPC8315_GPIO_GPODR         0x00000004
#define MPC8315_GPIO_GPDAT         0x00000008
#define MPC8315_GPIO_GPIER         0x0000000C
#define MPC8315_GPIO_GPIMR         0x00000010
#define MPC8315_GPIO_GPICR         0x00000014

extern u32  mpc83xx_read_gpio(int reg);
extern void mpc83xx_write_gpio(int reg, u32 val); 
extern void mpc83xx_set_sicr(u32 sicrl_mask, u32 sicrl_bits, 
                             u32 sicrh_mask, u32 sicrh_bits);
extern void mpc83xx_get_sicr(u32 *sicrl_bits, u32 *sicrh_bits);
extern void fenway_gpio_init(void);
extern u32 mpc83xx_read_sepnr(void);

#endif
