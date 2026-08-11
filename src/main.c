#include <motors.h>

int main(void)
{   
    printk("Iniciando programa...\n");

    motors_init();

    for(;;) {

        go();
        k_msleep(5000);
        k_msleep(2000);

    }
    
    return 0;
}