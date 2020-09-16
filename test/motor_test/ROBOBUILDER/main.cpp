#include "sam160.h"
#include <time.h>

int main() {
    char val;
    MyMotor motor;
    u8 SamId = (u8) 15;
    u8 Torq = (u8) 0x0;
    //motor.Quick_PosControl_CMD(SamId, Torq, (u8)0);
    
    //motor.Quick_PosControl_CMD(0x0, Torq, (u8)30);

    for(int cnt = 30; cnt < 100; cnt+=10) {
        printf("%d\n", cnt);
        motor.Quick_PosControl_CMD(SamId, Torq, (u8)cnt);
        sleep(1);
    }
    
    for(int cnt = 100; cnt >= 30; cnt-=10) {
        printf("%d\n", cnt);
        motor.Quick_PosControl_CMD(SamId, Torq, (u8)cnt);
        sleep(1);
    }
    
    motor.Quick_BrakeMode_CMD(SamId);
    motor.Close();

    return 0;
}