

#include "GO_M8010_6_Driver.h"


GO_Motorfield GO_motor_info[4];
const float gravit_const =9.81;

void GO_M8010_init (){
    for (uint8_t id= 0; id <4 ;id++)
    {
        GO_motor_info[id].id        = id;
        GO_motor_info[id].mode      = 1;
        GO_motor_info[id].correct   = 1;
        GO_motor_info[id].MError    = 0;
        GO_motor_info[id].Temp      = 0;
        GO_motor_info[id].tar_pos   = 0;
        GO_motor_info[id].tar_w     = 0;
        GO_motor_info[id].T         = 0;
        GO_motor_info[id].W         = 0;
        GO_motor_info[id].Pos       = 0;
        GO_motor_info[id].footForce = 0;
        GO_motor_info[id].buffer[0] = 0xFE;
        GO_motor_info[id].buffer[1] = 0xEE;
        
    }
}



// GO_Motorfield GO_M8010_init (uint8_t id, uint8_t mode)
// {   
//     GO_Motorfield cmd;
//     cmd.id    = id;
//     cmd.mode  = mode;
//     cmd.correct = 1;
//     cmd.MError=0;
//     cmd.Temp =0;
//     cmd.tar_pos=0;
//     cmd.T=0;
//     cmd.W=0;
//     cmd.Pos=0;
//     cmd.footForce = 0;
//     cmd.buffer[0] = 0xFE;
//     cmd.buffer[1] = 0xEE;
//     return cmd;
// }



uint8_t Temp_buffer[16];


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // CRC checking
    uint16_t crc = do_crc_table(Temp_buffer,sizeof(Temp_buffer)-2);
    if ((Temp_buffer[14] != (crc&0xFF)) || (Temp_buffer[15] != ((crc>>8) & 0xFF)))
    { 
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);           // indicate CRC uncorrect
        return;
    }

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);                 // indicate CRC correct
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);                              // indicate GO_M8010_6 running 

    GO_M8010_recv_data(Temp_buffer);

}

void uartTxCB(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_DMA(huart,Temp_buffer,sizeof(Temp_buffer));
}


void GO_M8010_send_data(UART_HandleTypeDef *huart, int id,int rev,float T,float W,
                         float Pos,float K_P,float K_W)
{
    // a pointer to target motor
    GO_Motorfield* motor;
    motor = GO_motor_info+id;

    // send preious cmd
    if (rev==1){
        HAL_UART_Transmit_IT(huart,motor->buffer,sizeof(motor->buffer));
        return;
    }

    // assign motor target goal to the buffer
    motor->motor_send_data.head[0]=0xFE;
    motor->motor_send_data.head[1]=0xEE;

    motor->motor_send_data.mode.id = motor->id & 0xF;
    motor->motor_send_data.mode.status = motor->mode & 0x7;
    motor->motor_send_data.mode.none = 0x0;
    motor->motor_send_data.comd.tor_des = T*256;

    motor->motor_send_data.comd.spd_des = (W*256*6.33)/(6.28319);
    motor->motor_send_data.comd.pos_des = (Pos*32768*6.33)/(6.28319);
    motor->motor_send_data.comd.k_pos = K_P*1280;
    motor->motor_send_data.comd.k_spd = K_W*1280;

    uint8_t* mode = (uint8_t *)&(motor->motor_send_data.mode);
    motor->buffer[2] = *mode;
    motor->buffer[3] = (motor->motor_send_data.comd.tor_des) & 0xFF;
    motor->buffer[4] =  (motor->motor_send_data.comd.tor_des>>8) & 0xFF;
    motor->buffer[5] =  (motor->motor_send_data.comd.spd_des) & 0xFF;
    motor->buffer[6] =  (motor->motor_send_data.comd.spd_des>>8) & 0xFF;
    motor->buffer[7] =  (motor->motor_send_data.comd.pos_des) & 0xFF;
    motor->buffer[8] =  (motor->motor_send_data.comd.pos_des>>8) & 0xFF;
    motor->buffer[9] =  (motor->motor_send_data.comd.pos_des>>16) & 0xFF;
    motor->buffer[10] =  (motor->motor_send_data.comd.pos_des>>24) & 0xFF;
    motor->buffer[11] =  (motor->motor_send_data.comd.k_pos) & 0xFF;
    motor->buffer[12] = (motor->motor_send_data.comd.k_pos>>8) & 0xFF;
    motor->buffer[13] =  (motor->motor_send_data.comd.k_spd) & 0xFF;
    motor->buffer[14] =  (motor->motor_send_data.comd.k_spd>>8) & 0xFF;

    //crc calulation 
    uint16_t crc = do_crc_table(motor->buffer, sizeof(motor->buffer)-2);
    motor->buffer[15] = (crc) & 0xFF;
    motor->buffer[16] = (crc>>8) & 0xFF;

    // interrupt buffer sending
    HAL_UART_Transmit_IT(huart,motor->buffer,sizeof(motor->buffer));
    
}






void GO_M8010_recv_data(uint8_t* Temp_buffer)
{
    uint8_t ID=-1;
    GO_Motorfield* motor;

    ID = Temp_buffer[2] & 0xF;

    motor = GO_motor_info + ID;
    memcpy(motor->Rec_buffer,Temp_buffer,16);

    
    motor->mode = Temp_buffer[2]>>4 & 0xF;
    motor->correct = 1;
    motor->MError = 0;
    int16_t T = Temp_buffer[4]<<8 | Temp_buffer[3];
    motor->T = T;
    motor->T /= 256;
    int16_t W = Temp_buffer[6]<<8 | Temp_buffer[5];
    motor->W = W;
    motor->W = (motor->W * 6.28319 )/(256*6.33);
    int32_t Pos = Temp_buffer[10]<<24 | Temp_buffer[9]<<16 | Temp_buffer[8]<<8 | Temp_buffer[7];
    motor->Pos = Pos;
    motor->Pos = (motor->Pos * 6.28319 )/(32768*6.33);
    int8_t Temp = Temp_buffer[11] & 0xFF;
    motor->Temp = Temp;
    motor->MError = Temp_buffer[12] & 0x7;
    
}

void basic_ForceControl (UART_HandleTypeDef *huart, int id, float bias, float length, float mass, 
                        float tar_pos, float tar_w, float K_P,float K_W)
{
    float forw_T = mass* gravit_const* length* (cos(GO_motor_info[id].Pos));
    forw_T += bias;
    forw_T =0;
    // GO_M8010_send_data(huart,id,0,forw_T,tar_w,tar_pos,K_P,K_W);
        GO_M8010_send_data(&huart1, id,0,forw_T,
                         tar_w,6,0.02,0);
}

