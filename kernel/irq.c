#include "kernel.h"
int handle_IRQ_event(int irq, struct pt_regs *);
int setup_irq(int irq, struct irqaction *new);
/**生成一个irqacion，并挂到中断服务队列里
* @irq note! should sub by 0x20.
* note, the status is marked IRQ_DISABLED, so you should clear this bit manually
* after you do 'request_irq',(see time.c init_time()) otherwise...
*/
int request_irq(int irq, void (*handler)(int, void*, struct pt_regs *), unsigned flags, void *dev){
	
	return 0;
}
