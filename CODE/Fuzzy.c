/*
 * Fuzzy.c
 *
 *  Created on: 2021��2��22��
 *      Author:������
 */


//ģ��������

#include "headfile.h"
#include "Fuzzy.h"

#define H_Min   2
MH      MHstruct;
MH      MHstructFastCar = {


          {2.259, 2.259, 2.259, 2.259, 2.259, 2.259, 2.259 } ,//���Pֵ��L
          {2.259, 2.259, 2.259, 2.259, 2.259, 2.259, 2.259 },//���Pֵ��R



  //ģ����|-*-���᣺������Ӿ������-*-|-*-���᣺��ֵƫ��仯����-*-|
   {
    {
      {//L-IN
              { 0, 1, 2, 3, },
              { 1, 2, 3, 4, },
              { 3, 4, 5, 6, },
              { 5, 6, 6, 6, }
      }
    },

    {
      {//R-IN
              { 0, 1, 2, 3, },
              { 1, 2, 3, 4, },
              { 3, 4, 5, 6, },
              { 5, 6, 6, 6, }
      }
    },

    {
      {//L-OUT
              { 0, 1, 2, 3, },
              { 1, 2, 3, 4, },
              { 3, 4, 5, 6, },
              { 5, 6, 6, 6, }
      }
    },

    {
      {//R-OUT
              { 0, 1, 2, 3, },
              { 1, 2, 3, 4, },
              { 3, 4, 5, 6, },
              { 5, 6, 6, 6, }
      }
    }
   },

  //���Pֵ��
  { 2.0, 2.3 ,2.6, 3.0, 3.3, 3.8, 4.3 },
  //���Iֵ��
  { 10, 16.625, 21.25, 32.5, 55, 77.5, 100 },
  //{ 10, 16.625, 21.25, 32.5, 55, 77.5, 100 }{ 50, 59.375, 68.75, 87.5, 125, 162.5, 200 },

  //���ģ����
  //ģ����|-*-���᣺������Ӿ������-*-|-*-���᣺����ƫ��仯����-*-|
  {
    {
      {//ADD-IN
        { 0, 1, 2, 3, },
        { 1, 2, 3, 4, },
        { 3, 4, 5, 6, },
        { 5, 6, 6, 6, }
      }
    },

    {
      {//SUB-IN
        { 0, 1, 2, 4, },
        { 1, 3, 5, 6, },
        { 3, 5, 6, 6, },
        { 5, 6, 6, 6, }
      }
    },

    {
      {//ADD-OUT
        { 0, 1, 2, 4, },
        { 1, 3, 5, 6, },
        { 3, 5, 6, 6, },
        { 5, 6, 6, 6, }
      }
    },

    {
      {//SUB-OUT
        { 0, 1, 2, 3, },
        { 1, 2, 3, 4, },
        { 3, 4, 5, 6, },
        { 5, 6, 6, 6, }
      }
    },
  }
};

/************************************************************************
��������ģ�����ʼ��
************************************************************************/
void InitMH(void) {
  MHstruct = MHstructFastCar;
  MHstruct.f_SizeOfViewE = 30; //��Чƫ��
  MHstruct.f_SizeOfViewH = 45; //��Ч���Ӿ���
}

//�ӹ��ƵĴ�������ֲ������
/************************************************************************
����������ȡ���Pֵ
���ܣ�ͨ��������Ӿ������ֵƫ��ó���ͬPֵ
������i32p_P------���Pָ��
      i16_ViewH---Offline
      i16_ViewE---��ֵƫ��
************************************************************************/
void DuoJi_GetP(float *i32p_P, int16 i16_ViewH, int16 i16_ViewE)

{


  MHstruct.i16_ViewH = i16_ViewH;

  float VH = f_Get_H_approximation(i16_ViewH - H_Min);
  float VE = f_Get_E_approximation(i16_ViewE, MHstruct.f_SizeOfViewE);
  float X2Y = 0;
  float X1Y = 0;
  float Y2X = 0;
  float Y1X = 0;

  int8 VH1 = (int)VH;
  if (VH1 > VH) {
    VH--;
  }
  int8 VH2 = VH1 + 1;

  int8 VE1 = (int)VE;
  if (VE1 > VE) {
    VE1--;
  }
  int8 VE2 = VE1 + 1;

  if (VH1 > 3) {
    VH1 = 3;
  }

  if (VH2 > 3) {
    VH2 = 3;
  }

  if (VE1 > 3) {
    VE1 = 3;
  }

  if (VE2 > 3) {
    VE2 = 3;
  }

  X2Y = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE2] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1]) *
            (VE - VE1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1];

  X1Y = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE2] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE1]) *
            (VE - VE1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE1];

  Y2X = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE1] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1]) *
            (VH - VH1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1];

  Y1X = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE2] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE2]) *
            (VH - VH1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE2];

  float P_approximation = (X2Y + X1Y + Y2X + Y1X) / 4.0;

  int8 P1 = (int)P_approximation;
  if (P1 > P_approximation) {
    P1--;
  }
  int8 P2 = P1 + 1;

    if (i16_ViewE < 0) {

  *i32p_P = (MHstruct.f_DuoJiP_TableL[P2] - MHstruct.f_DuoJiP_TableL[P1])*(P_approximation - P1) +MHstruct.f_DuoJiP_TableL[P1];
    }
    else
    {
  *i32p_P = (MHstruct.f_DuoJiP_TableR[P2] - MHstruct.f_DuoJiP_TableR[P1])*(P_approximation - P1) + MHstruct.f_DuoJiP_TableR[P1];
    }

}

/************************************************************************
����������ȡ����������
���ܣ���ȡget_p()�����������
�ӿڣ���
���ã�ͨ��get_p()��������
************************************************************************/
float f_Get_H_approximation(sint16 i16_ViewH) {
  float H_approximation;

  if (i16_ViewH < 0) {
    i16_ViewH = 0;
  }

  H_approximation = i16_ViewH * 3 / MHstruct.f_SizeOfViewH;

  return H_approximation;
}

/************************************************************************
����������ȡ�����������
���ܣ���ȡget_p()�����������
�ӿڣ���
���ã�ͨ��get_p()��������
************************************************************************/
float f_Get_E_approximation(sint16 i16_E, float f_E_Size) {
  float E_approximation;

  if (i16_E < 0) {
    i16_E = -i16_E;
  }

  E_approximation = i16_E * 3 / f_E_Size;

  return E_approximation;
}

//
//#define PB  6
//#define PM  5
//#define PS  4
//#define ZO  3
//#define NS  2
//#define NM  1
//#define NB  0
//float Fuzzy_P(int E)
//{
//    /*������P����ֵ������*/
//    float EFF[7]={-21,-14,-7,0,7,14,21};
////    /*������D����ֵ������*/
////    float VFF[7]={20,24,28,32,36,40,44};
////    /*�����U����ֵ������(������������ѡ��ͬ�����ֵ)*/
//    float PFF[7]={0.5,0.8,1.1,1.4,1.7,2.0,2.3};

//    int rule[7][7]={
//
//    // E  0   1   2   3   4   5   6    V
//        { 6 , 4 , 4 , 3 , 4 , 4 , 6},//0
//        { 5 , 4 , 4 , 3 , 4 , 4 , 5},//1
//        { 4 , 3 , 4 , 3 , 4 , 3 , 4},//2
//        { 4 , 3 , 3 , 3 , 3 , 3 , 4},//3
//        { 3 , 2 , 3 , 4 , 3 , 2 , 3},//4
//        { 3 , 2 , 3 , 5 , 3 , 2 , 3},//5
//        { 3 , 1 , 3 , 5 , 3 , 1 , 3},//6
//    };

//     int rule[7][7]={
//    //    0  1    2  3    4   5   6
//        { PB , PB , PM , PM , PS , ZO , ZO},//0
//        { PB , PB , PM , PS , PS , ZO , NS},//1
//        { PB , PM , PM , PS , ZO , NS , NS},//2
//        { PM , PM , PS , ZO , NS , NM , NM},//3
//        { PS , PS , ZO , NS , NS , NM , NB},//4
//        { PS , ZO , NS , NM , NM , NM , NB},//5
//        { ZO , ZO , NM , NM , NM , NB , NB},//6
//    };
//
//    float U=0;  /*ƫ��,ƫ��΢���Լ����ֵ�ľ�ȷ��*/
//    float PF[2]={0},DF[2]={0},UF[4]={0};
//    /*ƫ��,ƫ��΢���Լ����ֵ��������*/
//    int Pn=0,Dn=0,Un[4]={0};
//    float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
//    /*�����ȵ�ȷ��*/
//    /*����PD��ָ������ֵ�����Ч������*/
//    if(E>EFF[0] && E<EFF[6])
//    {
//        if(E<=EFF[1])
//        {
//            Pn=0;
//            PF[0]=(EFF[1]-E)/(EFF[1]-EFF[0]);
//        }
//        else if(E<=EFF[2])
//        {
//            Pn=1;
//            PF[0]=(EFF[2]-E)/(EFF[2]-EFF[1]);
//        }
//        else if(E<=EFF[3])
//        {
//            Pn=2;
//            PF[0]=(EFF[3]-E)/(EFF[3]-EFF[2]);
//        }
//        else if(E<=EFF[4])
//        {
//            Pn=3;
//            PF[0]=(EFF[4]-E)/(EFF[4]-EFF[3]);
//        }
//        else if(E<=EFF[5])
//        {
//            Pn=4;
//            PF[0]=(EFF[5]-E)/(EFF[5]-EFF[4]);
//        }
//        else if(E<=EFF[6])
//        {
//            Pn=5;
//            PF[0]=(EFF[6]-E)/(EFF[6]-EFF[5]);
//        }
//    }
//
//    else if(E<=EFF[0])
//    {
//        Pn=0;
//        PF[0]=1;
//    }
//    else if(E>=EFF[6])
//    {
//        Pn=5;
//        PF[0]=0;
//    }
//
//    PF[1]=1-PF[0];
//
//
//    //�ж�D��������
//    if(EC>DFF[0]&&EC<DFF[6])
//    {
//        if(EC<=DFF[1])
//        {
//            Dn=-2;
//            DF[0]=(DFF[1]-EC)/(DFF[1]-DFF[0]);
//        }
//        else if(EC<=DFF[2])
//        {
//            Dn=-1;
//            DF[0]=(DFF[2]-EC)/(DFF[2]-DFF[1]);
//        }
//        else if(EC<=DFF[3])
//        {
//            Dn=0;
//            DF[0]=(DFF[3]-EC)/(DFF[3]-DFF[2]);
//        }
//        else if(EC<=DFF[4])
//        {
//            Dn=1;
//            DF[0]=(DFF[4]-EC)/(DFF[4]-DFF[3]);
//        }
//        else if(EC<=DFF[5])
//        {
//            Dn=2;
//            DF[0]=(DFF[5]-EC)/(DFF[5]-DFF[4]);
//        }
//        else if(EC<=DFF[6])
//        {
//            Dn=3;
//            DF[0]=(DFF[6]-EC)/(DFF[6]-DFF[5]);
//        }
//    }
//    //���ڸ�����������
//    else if (EC<=DFF[0])
//    {
//        Dn=-2;
//        DF[0]=1;
//    }
//    else if(EC>=DFF[6])
//    {
//        Dn=3;
//        DF[0]=0;
//    }
//
//    DF[1]=1-DF[0];
//
//    /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
//    /*���ֵʹ��13����������,����ֵ��UFF[7]ָ��*/
//    /*һ�㶼���ĸ�������Ч*/
//    Un[0]=rule[Pn+2][Dn+2];
//    Un[1]=rule[Pn+3][Dn+2];
//    Un[2]=rule[Pn+2][Dn+3];
//    Un[3]=rule[Pn+3][Dn+3];
//
//    if(PF[0]<=DF[0])    //��С
//        UF[0]=PF[0];
//    else
//        UF[0]=DF[0];
//    if(PF[1]<=DF[0])
//        UF[1]=PF[1];
//    else
//        UF[1]=DF[0];
//    if(PF[0]<=DF[1])
//        UF[2]=PF[0];
//    else
//        UF[2]=DF[1];
//    if(PF[1]<=DF[1])
//        UF[3]=PF[1];
//    else
//        UF[3]=DF[1];
//    /*ͬ���������������ֵ���*/
//    if(Un[0]==Un[1])
//    {
//        if(UF[0]>UF[1])
//            UF[1]=0;
//        else
//            UF[0]=0;
//    }
//    if(Un[0]==Un[2])
//    {
//        if(UF[0]>UF[2])
//            UF[2]=0;
//        else
//            UF[0]=0;
//    }
//    if(Un[0]==Un[3])
//    {
//        if(UF[0]>UF[3])
//            UF[3]=0;
//        else
//            UF[0]=0;
//    }
//    if(Un[1]==Un[2])
//    {
//        if(UF[1]>UF[2])
//            UF[2]=0;
//        else
//            UF[1]=0;
//    }
//    if(Un[1]==Un[3])
//    {
//        if(UF[1]>UF[3])
//            UF[3]=0;
//        else
//            UF[1]=0;
//    }
//    if(Un[2]==Un[3])
//    {
//        if(UF[2]>UF[3])
//            UF[3]=0;
//        else
//            UF[2]=0;
//    }
//    t1=UF[0]*UFF[Un[0]];
//    t2=UF[1]*UFF[Un[1]];
//    t3=UF[2]*UFF[Un[2]];
//    t4=UF[3]*UFF[Un[3]];
//    temp1=t1+t2+t3+t4;
//    temp2=UF[0]+UF[1]+UF[2]+UF[3];//ģ�������
//    U=temp1/temp2;
//    return U;
//}

