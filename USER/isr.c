

/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		isr
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

#include "isr.h"
#include "C_H.h"
#include "Steer.h"
#include "System.h"
#include "car.h"
#include "icm.h"
#include "isr_config.h"
#include "magnet.h"
#include "scheduler.h"


int start_flag=1;
// PIT�жϺ���  ʾ��
//���PID �źŲ���  20ms
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY) {
  PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);
  magnet_routine();  //��в���
  if (SystemData.SteerOK == 1) {
    switch (SystemData.Model) {
      case 0:
        if (ImageStatus.Barn_Flag == 2 &&
                abs(icmdata.Yaw) < 80)  // ImageStatus.Barn_Lenth* OX < 65
          SteerPID_Realize_8(0 - 39);
        else if (ImageStatus.Barn_Flag == 2 && abs(icmdata.Yaw) > 80)
          SteerControl(steer_middle);
        else if (!SystemData.Stop) {  // p 2.23
//         if (ImageStatus.Road_type != Ramp)
          SteerPID_Realize_8(ImageStatus.Det_True -ImageStatus.MiddleLine);  //ģ������ ADC.AD_Det
//        else
//              SteerPID_Realize(Car.status.magnetSensors.error);
        }

        break;
      case 1:
              if (      SystemData.SpeedData.Length * OX < 100
                      &&icmdata.Yaw > -40
                      &&ImageStatus.Barn_Flag == 0
                      &&start_flag==1) {

                if (SystemData.SpeedData.Length * OX < 10)
                  SteerPID_Realize_8(0);  // g5.22
                else {
                  SteerPID_Realize_8(79 - 39);
                }
              } else if (ImageStatus.Barn_Flag == 2 &&abs(icmdata.Yaw) >- 80)  // ImageStatus.Barn_Lenth* OX < 65
                SteerPID_Realize_8(79 - 39);
              else if (ImageStatus.Barn_Flag == 2 && abs(icmdata.Yaw)<- 80)
                SteerControl(steer_middle);
              else if (!SystemData.Stop) {  // p 2.23
                //         if(ImageStatus.Road_type!=Ramp)
                  start_flag=0;
                SteerPID_Realize_8(ImageStatus.Det_True -ImageStatus.MiddleLine);  //ģ������ ADC.AD_Det
                                                             //         else
                //            SteerPID_Realize(Car.status.magnetSensors.error);
              }

              break;
      case 2:
        if (SystemData.SpeedData.Length * OX < 100 && icmdata.Yaw < 45&&ImageStatus.Barn_Flag == 0) {
          if (SystemData.SpeedData.Length * OX < 20)
            SteerPID_Realize_8(0);  // g5.22
          else {
            SteerPID_Realize_8(0 - 39);
          }
        } else if (ImageStatus.Barn_Flag == 2 &&
                abs(icmdata.Yaw) < 80)  // ImageStatus.Barn_Lenth* OX < 65
          SteerPID_Realize_8(0 - 39);
        else if (ImageStatus.Barn_Flag == 2 && abs(icmdata.Yaw) > 80)
          SteerControl(steer_middle);
        else if (!SystemData.Stop) {  // p 2.23
          //         if(ImageStatus.Road_type!=Ramp)
          SteerPID_Realize_8(ImageStatus.Det_True -
                             ImageStatus.MiddleLine);  //ģ������ ADC.AD_Det
                                                       //         else
          //            SteerPID_Realize(Car.status.magnetSensors.error);
        }
        break;
      case 3:
        SteerPID_Realize(Car.status.magnetSensors.error);
        break;
      default:
        break;
    }


  }

  ICM_OneOrderFilter();  //�����ǲ���
}


//����ж� 5ms
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY) {
  PIT_CLEAR_FLAG(CCU6_0, PIT_CH1);
  GetSpeed(&SystemData.SpeedData);
  if (ImageStatus.Barn_Flag == 2 && abs(icmdata.Yaw) > 80 &&ImageStatus.Barn_Lenth * OX > 75)
    SystemData.Stop = 1;
  if (SystemData.SpeedData.Length * OX > SystemData.debug_lenth)
    SystemData.Stop = 1;
  //        //if(SystemData.SpeedData.nowspeed>=0)
  //        uart_putbuff(WIRELESS_UART,&buffer_1,1);
  //        uart_putbuff(WIRELESS_UART,&buffer_2,1);
  //       seekfree_wireless_send_buff(&SystemData.SpeedData.nowspeed,4);//����sizeof�����ַ����ĳ��Ȱ��������һ��0�������Ҫ��һ
  //       uart_putbuff(WIRELESS_UART,&buffer_3,1);
  //       uart_putbuff(WIRELESS_UART,&buffer_4,1);
  //       Hypogynous_Machine(&SystemData.SpeedData.nowspeed,4);

  if (SystemData.Stop == 1)
    MOTOR(0);
  else if (SystemData.GO_OK == 1 && SystemData.MotorOK == 1) {
    Control_Speed();
    MOTOR(SystemData.SpeedData.expect_True_speed);
  }
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY) {
  enableInterrupts();  //�����ж�Ƕ��
  PIT_CLEAR_FLAG(CCU6_1, PIT_CH0);
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY) {
  enableInterrupts();  //�����ж�Ƕ��
  PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);
  scheduler_tick();
}

IFX_INTERRUPT(eru_ch0_ch4_isr, 0, ERU_CH0_CH4_INT_PRIO) {
  enableInterrupts();                     //�����ж�Ƕ��
  if (GET_GPIO_FLAG(ERU_CH0_REQ4_P10_7))  //ͨ��0�ж�
  {
    CLEAR_GPIO_FLAG(ERU_CH0_REQ4_P10_7);
    // printf("eru ch0 be triggered\n");
  }

  if (GET_GPIO_FLAG(ERU_CH4_REQ13_P15_5))  //ͨ��4�ж�
  {
    CLEAR_GPIO_FLAG(ERU_CH4_REQ13_P15_5);
    // printf("eru ch4 be triggered\n");
  }
}

IFX_INTERRUPT(eru_ch1_ch5_isr, 0, ERU_CH1_CH5_INT_PRIO) {
  enableInterrupts();                     //�����ж�Ƕ��
  if (GET_GPIO_FLAG(ERU_CH1_REQ5_P10_8))  //ͨ��1�ж�
  {
    CLEAR_GPIO_FLAG(ERU_CH1_REQ5_P10_8);
    // printf("eru ch1 be triggered\n");
  }

  if (GET_GPIO_FLAG(ERU_CH5_REQ1_P15_8))  //ͨ��5�ж�
  {
    CLEAR_GPIO_FLAG(ERU_CH5_REQ1_P15_8);
    // printf("eru ch5 be triggered\n");
  }
}

//��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
// IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)

IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO) {
  enableInterrupts();                     //�����ж�Ƕ��
  if (GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))  //ͨ��3�ж�
  {
    CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
    if (CAMERA_GRAYSCALE == camera_type)
      mt9v03x_vsync();
  }
  if (GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))  //ͨ��7�ж�
  {
    CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);
  }
}

IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��

  if (CAMERA_GRAYSCALE == camera_type)
    mt9v03x_dma();
}

//�����жϺ���  ʾ��
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrReceive(&uart0_handle);
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrError(&uart0_handle);
}

//����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrReceive(&uart1_handle);
  if (CAMERA_GRAYSCALE == camera_type)
    mt9v03x_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrError(&uart1_handle);
}

//����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrReceive(&uart2_handle);
  // wireless_uart_callback();
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrError(&uart2_handle);
}

IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrReceive(&uart3_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO) {
  enableInterrupts();  //�����ж�Ƕ��
  IfxAsclin_Asc_isrError(&uart3_handle);
}
