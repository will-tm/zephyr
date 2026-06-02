#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	int i = 0;

	while (1) {
		printk("[D0] tick %d\n", i++);
		k_msleep(1000);
	}
	return 0;
}
