/*
 * drivers/input/keyboard/pxa3xx_keypad.c
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/pxa3xx_keypad.h>

/*
 * Keypad Controller registers
 */
#define KPC             0x0000 /* Keypad Control register */
#define KPDK            0x0008 /* Keypad Direct Key register */
#define KPREC           0x0010 /* Keypad Rotary Encoder register */
#define KPMK            0x0018 /* Keypad Matrix Key register */
#define KPAS            0x0020 /* Keypad Automatic Scan register */

/* Keypad Automatic Scan Multiple Key Presser register 0-3 */
#define KPASMKP0        0x0028
#define KPASMKP1        0x0030
#define KPASMKP2        0x0038
#define KPASMKP3        0x0040
#define KPKDI           0x0048

#define KEYPAD_MMIO_SIZE	(0x100)

/* bit definitions */
#define KPC_MKRN(n)	((((n) & 0x7) - 1) << 26) /* matrix keypad row number */
#define KPC_MKCN(n)	((((n) & 0x7) - 1) << 23) /* matrix keypad column number */
#define KPC_DKN(n)	((((n) & 0x7) - 1) << 6)  /* direct key number */

#define KPC_AS          (0x1 << 30)  /* Automatic Scan bit */
#define KPC_ASACT       (0x1 << 29)  /* Automatic Scan on Activity */
#define KPC_MI          (0x1 << 22)  /* Matrix interrupt bit */
#define KPC_IMKP        (0x1 << 21)  /* Ignore Multiple Key Press */

#define KPC_MS7         (0x1 << 20)  /* Matrix scan line 7 */
#define KPC_MS6         (0x1 << 19)  /* Matrix scan line 6 */
#define KPC_MS5         (0x1 << 18)  /* Matrix scan line 5 */
#define KPC_MS4         (0x1 << 17)  /* Matrix scan line 4 */
#define KPC_MS3         (0x1 << 16)  /* Matrix scan line 3 */
#define KPC_MS2         (0x1 << 15)  /* Matrix scan line 2 */
#define KPC_MS1         (0x1 << 14)  /* Matrix scan line 1 */
#define KPC_MS0         (0x1 << 13)  /* Matrix scan line 0 */
#define KPC_MS_ALL      (KPC_MS0 | KPC_MS1 | KPC_MS2 | KPC_MS3 | \
			 KPC_MS4 | KPC_MS5 | KPC_MS6 | KPC_MS7)

#define KPC_ME          (0x1 << 12)  /* Matrix Keypad Enable */
#define KPC_MIE         (0x1 << 11)  /* Matrix Interrupt Enable */
#define KPC_DK_DEB_SEL	(0x1 <<  9)  /* Direct Keypad Debounce Select */
#define KPC_DI          (0x1 <<  5)  /* Direct key interrupt bit */
#define KPC_RE_ZERO_DEB (0x1 <<  4)  /* Rotary Encoder Zero Debounce */
#define KPC_REE1        (0x1 <<  3)  /* Rotary Encoder1 Enable */
#define KPC_REE0        (0x1 <<  2)  /* Rotary Encoder0 Enable */
#define KPC_DE          (0x1 <<  1)  /* Direct Keypad Enable */
#define KPC_DIE         (0x1 <<  0)  /* Direct Keypad interrupt Enable */

#define KPDK_DKP        (0x1 << 31)
#define KPDK_DK7        (0x1 <<  7)
#define KPDK_DK6        (0x1 <<  6)
#define KPDK_DK5        (0x1 <<  5)
#define KPDK_DK4        (0x1 <<  4)
#define KPDK_DK3        (0x1 <<  3)
#define KPDK_DK2        (0x1 <<  2)
#define KPDK_DK1        (0x1 <<  1)
#define KPDK_DK0        (0x1 <<  0)
#define KPDK_DK(n)	((n) & 0xff)

#define KPREC_OF1       (0x1 << 31)
#define kPREC_UF1       (0x1 << 30)
#define KPREC_OF0       (0x1 << 15)
#define KPREC_UF0       (0x1 << 14)

#define KPREC_RECOUNT0(n)	((n) & 0xff)

#define KPMK_MKP        (0x1 << 31)
#define KPAS_SO         (0x1 << 31)
#define KPASMKPx_SO     (0x1 << 31)

#define KPAS_MUKP(n)	(((n) >> 26) & 0x1f)
#define KPAS_RP(n)	(((n) >> 4) & 0xf)
#define KPAS_CP(n)	((n) & 0xf)

#define KPASMKP_MKC_MASK	(0xff)

#define keypad_readl(mmio, off)		__raw_readl(mmio + off)
#define keypad_writel(mmio, off, val)	__raw_writel(val, mmio + off)

#define MAX_MATRIX_KEY_NUM		(8 * 8)

struct pxa3xx_keypad {
	struct pxa3xx_keypad_platform_data *pdata;
	struct input_dev *input_dev;

	void __iomem *mmio_base;

	/* matrix key code map */
	unsigned int matrix_key_map[MAX_MATRIX_KEY_NUM];

	/* state row bits of each column scan */
	uint32_t matrix_key_state[MAX_MATRIX_KEY_COLS];
	uint32_t direct_key_state;

	unsigned int rotary_count;
};

static void pxa3xx_keypad_build_keymap(struct pxa3xx_keypad *keypad)
{
	struct pxa3xx_keypad_platform_data *pdata = keypad->pdata;
	unsigned int *key;
	int i;

	key = &pdata->matrix_key_map[0];
	for (i = 0; i < pdata->matrix_key_map_size; i++, key++) {
		int row = ((*key) >> 28) & 0xf;
		int col = ((*key) >> 24) & 0xf;
		int code = (*key) & 0xffffff;

		keypad->matrix_key_map[row * 8 + col] = code;
		set_bit(code, keypad->input_dev->keybit);
	}

	key = &pdata->direct_key_map[0];
	for (i = 0; i < pdata->direct_key_map_size; i++, key++)
		set_bit(*key, keypad->input_dev->keybit);
}

static inline unsigned int lookup_matrix_keycode(
		struct pxa3xx_keypad *keypad, int row, int col)
{
	return keypad->matrix_key_map[(row << 3) + col];
}

static void pxa3xx_keypad_scan_matrix(struct pxa3xx_keypad *keypad)
{
	int row, col, num_keys_pressed = 0;
	uint32_t new_state[MAX_MATRIX_KEY_COLS];
	uint32_t kpas = keypad_readl(keypad->mmio_base, KPAS);

	num_keys_pressed = KPAS_MUKP(kpas);

	memset(new_state, 0, sizeof(new_state));

	if (num_keys_pressed == 0)
		goto scan_finished;

	if (num_keys_pressed == 1) {
		col = KPAS_CP(kpas);
		row = KPAS_RP(kpas);

		/* if data invalid, treat as no key pressed */
		if (col >= MAX_MATRIX_KEY_COLS || row >= MAX_MATRIX_KEY_ROWS)
			goto scan_finished;

		new_state[col] = (1 << row);
		goto scan_finished;
	}

	if (num_keys_pressed > 1) {
		uint32_t kpasmkp0 = keypad_readl(keypad->mmio_base, KPASMKP0);
		uint32_t kpasmkp1 = keypad_readl(keypad->mmio_base, KPASMKP1);
		uint32_t kpasmkp2 = keypad_readl(keypad->mmio_base, KPASMKP2);
		uint32_t kpasmkp3 = keypad_readl(keypad->mmio_base, KPASMKP3);

		new_state[0] = kpasmkp0 & KPASMKP_MKC_MASK;
		new_state[1] = (kpasmkp0 >> 16) & KPASMKP_MKC_MASK;
		new_state[2] = kpasmkp1 & KPASMKP_MKC_MASK;
		new_state[3] = (kpasmkp1 >> 16) & KPASMKP_MKC_MASK;
		new_state[4] = kpasmkp2 & KPASMKP_MKC_MASK;
		new_state[5] = (kpasmkp2 >> 16) & KPASMKP_MKC_MASK;
		new_state[6] = kpasmkp3 & KPASMKP_MKC_MASK;
		new_state[7] = (kpasmkp3 >> 16) & KPASMKP_MKC_MASK;
	}

scan_finished:
	for (col = 0; col < MAX_MATRIX_KEY_COLS; col++) {
		uint32_t bits_changed;
	       
		bits_changed = keypad->matrix_key_state[col] ^ new_state[col];
		if (bits_changed == 0)
			continue;

		for (row = 0; row < MAX_MATRIX_KEY_ROWS; row++) {
			if ((bits_changed & (1 << row)) == 0)
				continue;

			input_report_key(keypad->input_dev,
				lookup_matrix_keycode(keypad, row, col),
				new_state[col] & (1 << row));
		}
	}

	memcpy(keypad->matrix_key_state, new_state, sizeof(new_state));
}

#define DEFAULT_ROTARY_COUNT		(0x7f)

static void pxa3xx_keypad_scan_rotary(struct pxa3xx_keypad *keypad)
{
	struct pxa3xx_keypad_platform_data *pdata = keypad->pdata;

	unsigned int new_rotary_count;
	unsigned int key_code;
	uint32_t kprec = keypad_readl(keypad->mmio_base, KPREC);

	new_rotary_count = KPREC_RECOUNT0(kprec);

	if (kprec & (KPREC_UF0 | KPREC_OF0)) {
		/* overflow/underflow: reset to default */
		keypad->rotary_count = DEFAULT_ROTARY_COUNT;
		keypad_writel(keypad->mmio_base, KPREC, keypad->rotary_count);

		key_code = (kprec & KPREC_OF0) ?
			pdata->rotary_up_key : pdata->rotary_down_key;
	} else {
		if (new_rotary_count == keypad->rotary_count)
			return;

		key_code = (new_rotary_count > keypad->rotary_count) ?
			pdata->rotary_up_key : pdata->rotary_down_key;
	}

	keypad->rotary_count = new_rotary_count;

	/* simulate a press-n-release */
	input_report_key(keypad->input_dev, key_code, 1);
	input_report_key(keypad->input_dev, key_code, 0);
}

static void pxa3xx_keypad_scan_direct(struct pxa3xx_keypad *keypad)
{
	struct pxa3xx_keypad_platform_data *pdata = keypad->pdata;

	int i, start_key = 0;
	unsigned int new_state;
	uint32_t kpdk, bits_changed;
       
	kpdk = keypad_readl(keypad->mmio_base, KPDK);

#if	0
	if (!(kpdk & KPDK_DKP))
		return;
#endif

	if (pdata->enable_rotary_key) {
		pxa3xx_keypad_scan_rotary(keypad);
		/* direct key 0 & 1 are used by rotary encoder */
		start_key = 2;
	}

	if (pdata->direct_key_map == NULL)
		return;

	new_state = KPDK_DK(kpdk);
	bits_changed = keypad->direct_key_state ^ new_state;

	if (bits_changed == 0)
		return;

	for (i = start_key; i < MAX_DIRECT_KEYS; i++) {
		if (bits_changed & (1 << i))
			input_report_key(keypad->input_dev,
					pdata->direct_key_map[i],
					(new_state & (1 << i)));
	}
	keypad->direct_key_state = new_state;
}

static irqreturn_t pxa3xx_keypad_interrupt(int irq, void *dev_id)
{
	struct pxa3xx_keypad *keypad = dev_id;
	uint32_t kpc = keypad_readl(keypad->mmio_base, KPC);

#ifdef	CONFIG_LITTLETON_BACKLIGHT
	extern led_keypad_press(void);
	led_keypad_press();
#endif

	if (kpc & KPC_MI)
		pxa3xx_keypad_scan_matrix(keypad);

	if (kpc & KPC_DI)
		pxa3xx_keypad_scan_direct(keypad);

        return IRQ_HANDLED;
}

extern void pxa3xx_enable_keyp_pins(void);

static void pxa3xx_keypad_config(struct pxa3xx_keypad *keypad)
{
	struct pxa3xx_keypad_platform_data *pdata = keypad->pdata;
	uint32_t kpc = 0, debounce = 0;

	/* enable matrix key with automatic scan */
	if (pdata->enable_matrix_key) {
		kpc |= KPC_MKRN(pdata->matrix_key_rows) |
		       KPC_MKCN(pdata->matrix_key_cols);
		kpc |= KPC_ASACT | KPC_MIE | KPC_ME | KPC_MS_ALL;
	}

	/* enable direct key */
	if (pdata->enable_direct_key)
		kpc |= KPC_DKN(pdata->direct_key_num) | KPC_DE | KPC_DIE;

	/* enable rotary key if necessary */
	if (pdata->enable_rotary_key)
		kpc |= (KPC_REE0 | KPC_RE_ZERO_DEB);

	keypad_writel(keypad->mmio_base, KPC, kpc);
	keypad_writel(keypad->mmio_base, KPREC, DEFAULT_ROTARY_COUNT);

	debounce = ((pdata->direct_key_debounce & 0xff) << 8) |
		    (pdata->matrix_key_debounce & 0xff);
	keypad_writel(keypad->mmio_base, KPKDI, debounce);

	//FIXME, not needed, really?
	//pxa3xx_enable_keyp_pins();
}

static int pxa3xx_keypad_suspend(struct platform_device *dev, pm_message_t state)
{
	pxa_set_cken(CKEN_KEYPAD, 0);
	return 0;
}

static int pxa3xx_keypad_resume(struct platform_device *dev)
{
	struct pxa3xx_keypad *keypad = platform_get_drvdata(dev);

	/* re-initialize the keypad controller */
	pxa_set_cken(CKEN_KEYPAD, 1);
	pxa3xx_keypad_config(keypad);

	return 0;
}

static int __init pxa3xx_keypad_probe(struct platform_device *pdev)
{
	struct pxa3xx_keypad *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int ret;

	keypad = kzalloc(sizeof(struct pxa3xx_keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev) {
		kfree(keypad);
		return -ENOMEM;
	}

	keypad->input_dev = input_dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res = request_mem_region(res->start, res->end - res->start + 1,
				"pxa3xx_keypad");
	if (!res) {
		ret = -EBUSY;
		goto failed;
	}

	keypad->mmio_base = ioremap_nocache(res->start, KEYPAD_MMIO_SIZE);
	if (keypad->mmio_base == NULL) {
		dev_err(&pdev->dev, "unable to remap keypad registers\n");
		release_mem_region(res->start, res->end - res->start + 1);
		goto failed;
	}

	ret = request_irq(IRQ_KEYPAD, pxa3xx_keypad_interrupt, IRQF_DISABLED,
			"pxa3xx_keypad", keypad);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
		ret = -EBUSY;
		goto failed_freeio;
	}

	platform_set_drvdata(pdev, keypad);

	/* setup input device */
	input_dev->name = "pxa3xx_keypad";
	input_dev->phys = "pxa3xx_keypad/input0";

	input_dev->cdev.dev = &pdev->dev;
	input_dev->private = keypad;

	/* initialize the keypad controller with platform
	 * specific parameters and build matrix key code
	 * map for fast lookup
	 */
	keypad->pdata = pdev->dev.platform_data;
	pxa3xx_keypad_build_keymap(keypad);

	set_bit(EV_KEY, input_dev->evbit);
	if (keypad->pdata->enable_repeat_key)
		set_bit(EV_REP, input_dev->evbit);

	ret = input_register_device(input_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register input device\n");
		goto failed_freeio;
	}

	pxa_set_cken(CKEN_KEYPAD, 1);
	pxa3xx_keypad_config(keypad);

	return 0;

failed_freeio:
	release_mem_region(res->start, res->end - res->start + 1);
	iounmap(keypad->mmio_base);

failed:
	input_free_device(input_dev);
	kfree(keypad);
	return -EINVAL;
}

static int pxa3xx_keypad_remove(struct platform_device *pdev)
{
	struct pxa3xx_keypad *keypad = platform_get_drvdata(pdev);

	pxa_set_cken(CKEN_KEYPAD, 0);

	free_irq(IRQ_KEYPAD, keypad);
	input_unregister_device(keypad->input_dev);
	kfree(keypad);
	return 0;
}

static struct platform_driver pxa3xx_keypad_driver = {
	.driver		= {
		.name	= "pxa3xx_keypad",
	},
	.probe		= pxa3xx_keypad_probe,
	.remove		= pxa3xx_keypad_remove,
	.suspend	= pxa3xx_keypad_suspend,
	.resume		= pxa3xx_keypad_resume,
};

static int __init pxa3xx_keypad_init(void)
{
	return platform_driver_register(&pxa3xx_keypad_driver);
}

static void __exit pxa3xx_keypad_exit(void)
{
	platform_driver_unregister(&pxa3xx_keypad_driver);
}

module_init(pxa3xx_keypad_init);
module_exit(pxa3xx_keypad_exit);
