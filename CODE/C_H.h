/*
 * C__H.h
 *
 *  Created on: 2021��1��26��
 *      Author: ������
 */

#ifndef _C_H_H_
#define _C_H_H_

#include "Platform_Types.h"

#define LCDH 60                              //����ͼ����ͼ��ĸ߶�
#define LCDW 80                              //����ͼ����ͼ��Ŀ��
#define uLCDH 120                            //����ͼ����ʾ��ͼ��߶�
#define uLCDW 160                            //����ͼ����ʾ��ͼ����
#define HMAX (LCDH - 1)                      //û��
#define WMAX (LCDW - 1)                      //û��
#define LimitL(L) (L = ((L < 1) ? 1 : L))    //���Ʒ���
#define LimitH(H) (H = ((H > 78) ? 78 : H))  //���Ʒ���
#define ImageSensorMid 39                    //ͼ����Ļ�е�
extern int ImageScanInterval;  //ɨ�߷�Χ    ��һ�еı߽�+-ImageScanInterval
extern int ImageScanInterval_Cross;  //ʮ��ɨ�߷�Χ
typedef struct {
  int point;
  uint8 type;
} JumpPointtypedef;

//ÿһ�е�����
typedef struct {
  /*���ұ߽߱��־    TΪ���������    WΪ�ޱ�   PΪ�ϰ�������ߵ��ڱ�*/
  uint8 IsRightFind;      //�ұ��б߱�־
  uint8 IsLeftFind;       //����б߱�־
  uint8 isBlackFind;      //�����
  int Wide;               //�߽���
  int LeftBorder;         //��߽�
  int RightBorder;        //�ұ߽�
  int close_LeftBorder;   //���߽߱�
  int close_RightBorder;  //���߽߱�
  int opp_LeftBorder;     //����߽�
  int opp_RightBorder;    //����߽�
  int Center;             //����
  int RightTemp;          //�ұ���ʱֵ
  int LeftTemp;           //�����ʱֵ
  int CenterTemp;         //������ʱֵ
  int Black_Point;        //���кڵ�����

  // fork   ����ı������ڲ�·�ڵļ��
  int BlackWide;     //����ں�ɫ���
  int Black_Wide_L;  //����ں�ɫ���
  int Black_Wide_R;  //����ں�ɫ�ұ�
  int Black_Pro;     //����ںڵ����
  int mid_temp;      //������ʱֵ
} ImageDealDatatypedef;

//Ԫ������
typedef enum {
  Normol,       //���κ�����
  Straight,     ////ֱ��
  Cross,        ////ʮ��
  Ramp,         //�µ�
  LeftCirque,   ////��Բ��
  RightCirque,  ////��Բ��
  Forkin,       //��·����
  Forkout,      //��·����
  Barn_out,     //����
  Barn_in,      //���
  Cross_ture,


} RoadType_e;

typedef struct {
  /*���¹���ȫ��ͼ����������*/

  //ͼ����Ϣ
  int TowPoint;             //��ʼ�Ƚϵ�           ����ǰհ
  int TowPointAdjust_v;     //��ʼ�Ƚϵ���Ӧ�ٶ�    û��
  int TowPoint_True;        //ʵ�ʱȽϵ�           û��
  int TowPoint_Gain;        //����ϵ��             û��
  int TowPoint_Offset_Max;  //���ƫ��ϵ��         û��
  int TowPoint_Offset_Min;  //��Сƫ��ϵ��         û��
  int Det_True;             //��GetDet()�������������ƽ��ͼ��ƫ��
  int Det_all;              //ͼ��Ľ��˵�Զ�˵���ƫ��
  float Det_all_k;          //б��

  uint8 Threshold;          //��ֵ����ֵ
  uint32 Threshold_static;  //��ֵ����̬����
  uint8 Threshold_detach;   //�����㷨�ָ���ֵ
  uint8 MiddleLine;         //��Ļ����
  int Foresight;            //ƫ���ٶ�����   Զ�˵�ƫ��  ���ڿ���
  uint8 OFFLine;            /////ͼ�񶥱�
  uint8 WhiteLine;          ////˫�߶�����
  float ExpectCur;          /////ͼ����������    û��
  float White_Ritio;        //��Ч�а׵����     û��
  int Black_Pro_ALL;        //����ڵ����%      û��

  // PID
  float Piriod_P;  //�ֶ�P   ֵ     û��
  float MU_P;

  RoadType_e Road_type;  //Ԫ������

  /****Բ��***/
  uint8 IsCinqueOutIn;  //����Բ��
  uint8 CirquePass;     //Բ����
  uint8 CirqueOut;      //��Բ��
  uint8 CirqueOff;      //Բ������
  int Pass_Lenth;       //�뻷����  ���ڷ���
                        /****Բ��***/

  int Out_Lenth;     //
  int Fork_Out_Len;  //����ڷ���
  int Dowm_lenth;    //������پ���  //g5.13
  int Cross_Lenth;  // 270��ת��֮��һ����ʮ��·��   ��Ϊ����ڵ�����
  int Cross_ture_lenth;
  int Sita;  //�ݴ��ж��ڹ�ȥһ��·����ת�����ٽǶ�

  //����
  int Barn_Flag;   //�жϿ�Ĵ���
  int Barn_Lenth;  //���ֹͣ����

  //����
  int Stop_lenth;  //���籣�������о���
  //�µ�����
  int Ramp_lenth;

  int variance;  //ֱ�������ֵ����

  int straight_acc;  //ֱ�����ٱ�־λ
  int  variance_acc;    //���ڼ��ٵ���ֵ����

  int ramptestlenth;//�µ������
} ImageStatustypedef;

extern uint8 Image_Use[LCDH][LCDW];
extern uint8 Pixle[LCDH][LCDW];
extern uint8 UImage_Use[uLCDH][uLCDW];
extern ImageStatustypedef ImageStatus;
extern ImageDealDatatypedef ImageDeal[60];
extern float variance;  //����
extern int ForkLinePointy;
extern float variance_acc;
extern int Fork_dowm;
extern int circle_r;
extern float circlk_k;
extern int circleoff_lenth;//
extern int forklenth;
extern int barnlenth;
extern int ramplenth;
extern int ramptestflag;
extern int rampnum;
void camera_display(void);  //ͼ����ʾ
void compressimage();       ////ѹ��ͼ��
void Get01change();         ////��ֵ��ͼ��
void Pixle_Filter();        ////�����˲���Ч������
void Draw_Road(void);       ///���������߽�
void ImageProcess(void);    ////ͼ����������
void uncompressimage();     //ͼ���ѹ
void ForkTest();            //��·�ڼ��
void Cross_Test();          //Բ�����
void Cross_Handle();        //Բ������
void Element_Test();        //Ԫ�ؼ��
void Ramp_Test();           //�µ����
void Barn_test_in();        //�����
void Element_Handle();      //Ԫ�ش���
void Fork_Handle();         //���洦��
void Barn_in_Handle();      //û��
void GetLenSita();          //û����
#endif                  /* CODE_C_H_H_ */
