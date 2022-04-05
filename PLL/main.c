#include "Si5351A_PLL.h"
#include twi_master.h

int main(void){
    // set up I2C
    tw_init();

    // set frequency
    setLOF1(10000000);

    return 0;
}