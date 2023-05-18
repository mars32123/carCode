/*
 * C_H.c
 *
 *  Created on: 2021��1��26��
 *      Author: ������
 */

#include "C_H.h"
#include <math.h>
#include "Steer.h"
#include "System.h"
#include "car.h"
#include "headfile.h"
#include "icm.h"
#include "led.h"
#include "magnet.h"
#include "road_info.h"
#include "DataStore.h"
int ImageScanInterval;                         //ɨ�߷�Χ    ��һ�еı߽�+-ImageScanInterval
int ImageScanInterval_Cross;                   //270��������ʮ�ֵ�ɨ�߷�Χ
IFX_ALIGN(4) uint8 Image_Use[LCDH][LCDW];      //�Ҷ�ͼ��
IFX_ALIGN(4) uint8 Pixle[LCDH][LCDW];          //���ڴ���Ķ�ֵ��ͼ��
IFX_ALIGN(4) uint8 uPixle[uLCDH][uLCDW];       //������ʾ��ѹ��Ķ�ֵ��ͼ��
IFX_ALIGN(4) uint8 UImage_Use[uLCDH][uLCDW];   //������ʾ��ѹ��ĻҶ�ͼ��
static int Ysite = 0, Xsite = 0;               //Y����=��
static uint8* PicTemp;                         //���浥��ͼ��
static int IntervalLow = 0, IntervalHigh = 0;  //����ߵ�ɨ������
static int ytemp = 0;                          //�����
static int TFSite = 0, FTSite = 0;             //�����
static float DetR = 0, DetL = 0;               //���б��
static int BottomBorderRight = 79,             //59���ұ߽�
BottomBorderLeft = 0,                          //59����߽�
BottomCenter = 0;                              //59���е�
ImageDealDatatypedef ImageDeal[60];            //��¼���е���Ϣ
ImageStatustypedef ImageStatus;                //ͼ���ȫ�ֱ���
ImageStatustypedef ImageData;                  //��Ҫ�޸ĵ�ͼ����ֵ����
int forklenth;                                 //ȷ������λ��
int barnlenth;                                 //ȷ������λ��
int ramplenth;                                 //ȷ���µ�λ��
float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77,0.71, 0.65, 0.59, 0.53, 0.47};//10��Ȩ�ز�����������ģ�������Ӱ�죬���°�����̬�ֲ�����
uint8 ExtenLFlag = 0;  //�Ƿ����ӳ���־
uint8 ExtenRFlag = 0;  //�Ƿ����ӳ���־
void camera_display(void) {
  if (mt9v03x_finish_flag == 1) {  //��ʾ���� ��������
    if (SystemData.GO_OK || SystemData.CameraOK == 1 ||
        SystemData.OldCameraOK == 1)
         ImageProcess(); //5-6ms  

    static int cnt = 0;
    ++cnt;
    cnt %= 2;
    if (!cnt) {
      if (SystemData.CameraOK == 1)  //���Ե�ʱ���  �ܵ�ʱ�����
        ips114_displayimage01(uPixle[0], uLCDW, uLCDH);
      else if (SystemData.OldCameraOK == 1)
        ips114_displayimage032(UImage_Use[0], uLCDW, uLCDH);
    }
  }
}


float Mh = MT9V03X_H;
float Lh = LCDH;
float Mw = MT9V03X_W;
float Lw = LCDW;

void compressimage() {
  int i, j, row, line;
  const float div_h = Mh / Lh, div_w = Mw / Lw;
  for (i = 0; i < LCDH; i++) {
    row = i * div_h + 0.5;
    for (j = 0; j < LCDW; j++) {
      line = j * div_w + 0.5;
      Image_Use[i][j] = mt9v03x_image[row][line];
    }
  }
  mt9v03x_finish_flag = 0;  //ʹ����һ֡DMA�����ͼ��ͼ��  ���Կ�ʼ������һ֡
}


int HD_thre;  //��ʱ�۲����
//��ֵ��
void Get01change() {
  uint8 thre;
  uint8 i, j;
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
      if (j <= 15)
        thre = ImageStatus.Threshold_static - 10;
      else if ((j > 70 && j <= 75))
        thre = ImageStatus.Threshold_static - 15;
      else if (j >= 65)
        thre = ImageStatus.Threshold_static - 15;
      else
        thre = ImageStatus.Threshold_static;

      if (Image_Use[i][j] >
          (thre))  //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
        Pixle[i][j] = 1;  //��
      else
        Pixle[i][j] = 0;  //��
    }
  }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      �Ż��Ĵ��
//  @param      image  ͼ������
//  @param      clo    ��
//  @param      row    ��
//  @param      pixel_threshold ��ֵ����
//  @return     uint8
//  @since      2021.6.23
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 Threshold_deal(uint8* image,
                     uint16 col,
                     uint16 row,
                     uint32 pixel_threshold) {
#define GrayScale 256
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8 threshold = 0;
  uint8* data = image;  //ָ���������ݵ�ָ��
  for (i = 0; i < GrayScale; i++) {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
  for (i = 0; i < height; i += 1) {
    for (j = 0; j < width; j += 1) {
      // if((sun_mode&&data[i*width+j]<pixel_threshold)||(!sun_mode))
      //{
      pixelCount[(
          int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
      gray_sum += (int)data[i * width + j];  //�Ҷ�ֵ�ܺ�
      //}
    }
  }

  //����ÿ������ֵ�ĵ�������ͼ���еı���
  for (i = 0; i < GrayScale; i++) {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }


  //�����Ҷȼ�[0,255]
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < pixel_threshold; j++) {
    w0 +=
        pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮�� ���������ֵı���
    u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;    //����ƽ���Ҷ�
    u1 = u1tmp / w1;    //ǰ��ƽ���Ҷ�
    u = u0tmp + u1tmp;  //ȫ��ƽ���Ҷ�
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax) {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax) {
      break;
    }
  }
  return threshold;
}

void Get01change_dajin() {
  ImageStatus.Threshold = Threshold_deal(Image_Use[0], LCDW, LCDH, ImageStatus.Threshold_detach);
  if (ImageStatus.Threshold < ImageStatus.Threshold_static)
    ImageStatus.Threshold = ImageStatus.Threshold_static;
  uint8 i, j = 0;
  uint8 thre;
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
      if (j <= 15)
        thre = ImageStatus.Threshold - 10;
      else if ((j > 70 && j <= 75))
        thre = ImageStatus.Threshold - 10;
      else if (j >= 65)
        thre = ImageStatus.Threshold - 10;
      else
        thre = ImageStatus.Threshold;

      if (Image_Use[i][j] >
          (thre))         //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
        Pixle[i][j] = 1;  //��
      else
        Pixle[i][j] = 0;  //��
    }
  }
}

//�����˲�
void Pixle_Filter() {
  int nr;  //��
  int nc;  //��

  for (nr = 10; nr < 40; nr++) {
    for (nc = 10; nc < 70; nc = nc + 1) {
      if ((Pixle[nr][nc] == 0) && (Pixle[nr - 1][nc] + Pixle[nr + 1][nc] +
                                       Pixle[nr][nc + 1] + Pixle[nr][nc - 1] >=
                                   3)) {
        Pixle[nr][nc] = 1;
      }
      //      else
      //      if((Pixle[nr][nc]==1)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]<2))
      //      {
      //        Pixle[nr][nc]=0;
      //      }
    }
  }
}

// 01��ѹͼ��
void uncompressimage() {
  int i = 0, j = 0, row = 0, line = 0;
  int div_h = uLCDH / LCDH;
  int div_w = uLCDW / LCDW;
  for (i = 0; i < uLCDH; i++) {
    for (j = 0; j < uLCDW; j++) {
      row = (int)(i / div_h);
      line = (int)(j / div_w);
      uPixle[i][j] = Pixle[row][line];
    }
  }
}
//�ҶȽ�ѹͼ��
void uncompressimageHD() {
  int i = 0, j = 0, row = 0, line = 0;
  int div_h = uLCDH / LCDH;
  int div_w = uLCDW / LCDW;
  for (i = 0; i < uLCDH; i++) {
    for (j = 0; j < uLCDW; j++) {
      row = (int)(i / div_h);
      line = (int)(j / div_w);
      UImage_Use[i][j] = Image_Use[row][line];
    }
  }
}



void GetJumpPointFromDet(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q)  //��һ��������Ҫ���ҵ����飨80���㣩
                                                                               //�ڶ���ɨ����߻���ɨ�ұ���
{                                                                              //�����ǿ�ʼ�ͽ�����
  int i = 0;
  if (type == 'L')                              //ɨ�������
  {
    for (i = H; i >= L; i--) {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   //�ɺڱ��
      {
        Q->point = i;                           //��¼�����
        Q->type = 'T';                          //��ȷ����
        break;
      } else if (i == (L + 1))                  //����ɨ�����Ҳû�ҵ�
      {
        if (*(p + (L + H) / 2) != 0)            //����м��ǰ׵�
        {
          Q->point = (L + H) / 2;               //��Ϊ��������е�
          Q->type = 'W';                        //����ȷ�������м�Ϊ�ף���Ϊû�б�
          break;
        } else                                  //����ȷ�������м�Ϊ��
        {
          Q->point = H;                         //����м��Ǻڵ�
          Q->type = 'H';                        //�����ֱ�����ֵ����Ϊ�Ǵ�����
          break;
        }
      }
    }
  } else if (type == 'R')                       //ɨ���ұ���
  {
    for (i = L; i <= H; i++)                    //��������ɨ
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   //���ɺڵ��׵�����
      {
        Q->point = i;                           //��¼
        Q->type = 'T';
        break;
      } else if (i == (H - 1))                  //����ɨ�����Ҳû�ҵ�
      {
        if (*(p + (L + H) / 2) != 0)            //����м��ǰ׵�
        {
          Q->point = (L + H) / 2;               //�ұ������е�
          Q->type = 'W';
          break;
        } else                                  //����е��Ǻڵ�
        {
          Q->point = L;                         //�����ֱ�����ֵ
          Q->type = 'H';
          break;
        }
      }
    }
  }
}





static uint8 DrawLinesFirst(void) {
  PicTemp = Pixle[59];
  if (*(PicTemp + ImageSensorMid) == 0)                 //����ױ�ͼ���е�Ϊ�ڣ��쳣���
  {
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)    //�����ұ���
    {
      if (*(PicTemp + ImageSensorMid - Xsite) != 0)     //һ���ҵ���������������ľ��룬��break
        break;                                          //���Ҽ�¼Xsite
      if (*(PicTemp + ImageSensorMid + Xsite) != 0)
        break;
    }

    if (*(PicTemp + ImageSensorMid - Xsite) != 0)       //�����������ߵĻ�
    {
      BottomBorderRight = ImageSensorMid - Xsite + 1;   // 59���ұ�������
      for (Xsite = BottomBorderRight; Xsite > 0; Xsite--)  //��ʼ��59�������
      {
        if (*(PicTemp + Xsite) == 0 &&
            *(PicTemp + Xsite - 1) == 0)                //���������ڵ㣬�˲�
        {
          BottomBorderLeft = Xsite;                     //������ҵ�
          break;
        } else if (Xsite == 1) {
          BottomBorderLeft = 0;                         //����������ˣ�����������ߣ��������Ϊ��0
          break;
        }
      }
    } else if (*(PicTemp + ImageSensorMid + Xsite) != 0)  //����������ұߵĻ�
    {
      BottomBorderLeft = ImageSensorMid + Xsite - 1;    // 59�����������
      for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++)  //��ʼ��59�������
      {
        if (  *(PicTemp + Xsite) == 0 
            &&*(PicTemp + Xsite + 1) == 0)              //���������ڵ㣬�˲�
        {
          BottomBorderRight = Xsite;                    //�ұ����ҵ�
          break;
        } else if (Xsite == 78) {
          BottomBorderRight = 79;                       //����������ˣ��������ұ��ߣ��������Ϊ��79
          break;
        }
      }
    }
  } else                                                //������е��ǰ׵ģ��Ƚ����������
  {
    for (Xsite = ImageSensorMid; Xsite < 79; Xsite++)   //һ����һ����������ұ���
    {
      if (  *(PicTemp + Xsite) == 0 
          &&*(PicTemp + Xsite + 1) == 0)                //���������ڵ㣬�˲�
      {
        BottomBorderRight = Xsite;                      //�ҵ��ͼ�¼
        break;
      } else if (Xsite == 78) {
        BottomBorderRight = 79;                         //�Ҳ�����Ϊ79
        break;
      }
    }
    for (Xsite = ImageSensorMid; Xsite > 0; Xsite--)    //һ����һ��������������
    {
      if (  *(PicTemp + Xsite) == 0 
          &&*(PicTemp + Xsite - 1) == 0)                //���������ڵ㣬�˲�
      {
        BottomBorderLeft = Xsite;                       //�ҵ��ͼ�¼
        break;
      } else if (Xsite == 1) {
        BottomBorderLeft = 0;                           //�Ҳ�����Ϊ0
        break;
      }
    }
  }
  BottomCenter =(BottomBorderLeft + BottomBorderRight) / 2;   // 59���е�ֱ��ȡƽ��
  ImageDeal[59].LeftBorder = BottomBorderLeft;                //�����������¼һ����Ϣ����һ������һ�����
  ImageDeal[59].RightBorder = BottomBorderRight;
  ImageDeal[59].Center = BottomCenter;                        //ȷ����ױ�
  ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;  //�洢�����Ϣ
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';
  for (Ysite = 58; Ysite > 54; Ysite--)                       //���м�������ȷ���ױ�����
  {
    PicTemp = Pixle[Ysite];
    for (Xsite = ImageDeal[Ysite + 1].Center; Xsite < 79;
         Xsite++)                                             //��ǰ��һ��������
    {
      if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0) {
        ImageDeal[Ysite].RightBorder = Xsite;
        break;
      } else if (Xsite == 78) {
        ImageDeal[Ysite].RightBorder = 79;
        break;
      }
    }
    for (Xsite = ImageDeal[Ysite + 1].Center; Xsite > 0;
         Xsite--)                                             //��ǰ��һ��������
    {
      if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0) {
        ImageDeal[Ysite].LeftBorder = Xsite;
        break;
      } else if (Xsite == 1) {
        ImageDeal[Ysite].LeftBorder = 0;
        break;
      }
    }
    ImageDeal[Ysite].IsLeftFind = 'T';                        //��Щ��Ϣ�洢��������
    ImageDeal[Ysite].IsRightFind = 'T';
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /2; //�洢�е�
    ImageDeal[Ysite].Wide =
        ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;      //�洢���
  }
  return 'T';
}                                                             //�������Ҫ��������������Ȳ����ܵ����ţ�����Ҫ�ڰ�װ��ʱ���������ͷ���ӽ�

/*����׷����µõ�ȫ������*/
static void DrawLinesProcess(void)  //////���ø���
{
  uint8 L_Found_T = 'F';  //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_L_line = 'F';  //�ҵ���һ֡ͼ��Ļ�׼��б��
  uint8 R_Found_T = 'F';  //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_R_line = 'F';  //�ҵ���һ֡ͼ��Ļ�׼��б��
  float D_L = 0;           //�ӳ��������б��
  float D_R = 0;           //�ӳ����ұ���б��
  int ytemp_W_L;           //��ס�״��󶪱���
  int ytemp_W_R;           //��ס�״��Ҷ�����
  ExtenRFlag = 0;          //��־λ��0
  ExtenLFlag = 0;
  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--)            //ǰ5�д�����ˣ������55�е����趨�Ĳ��������OFFLine��
  {                        //̫Զ��ͼ���ȶ���OFFLine�Ժ�Ĳ�����
    PicTemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];                                          // 0��1��
    if (ImageStatus.Road_type != Cross) {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval;             //����һ���ұ���-Interval�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
      IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;           //����һ���ұ���+Interval�ĵ������ȷ��ɨ������㣩
    } else {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval_Cross;       //����һ���ұ���-Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;    //����һ���ұ���+Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
    }

    LimitL(IntervalLow);   //ȷ����ɨ�����䲢��������
    LimitH(IntervalHigh);  //ȷ����ɨ�����䲢��������
    GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);     //ɨ�ұ���




    if (ImageStatus.Road_type != Cross) {
      IntervalLow =ImageDeal[Ysite + 1].LeftBorder -ImageScanInterval;                //����һ���ұ���-5�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
      IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;               //����һ���ұ���+5�ĵ������ȷ��ɨ������㣩
    } else {
      IntervalLow =ImageDeal[Ysite + 1].LeftBorder -ImageScanInterval_Cross;                //����һ���ұ���-5�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
      IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval_Cross;               //����һ���ұ���+5�ĵ������ȷ��ɨ������㣩
    }


    LimitL(IntervalLow);   //ȷ����ɨ�����䲢��������
    LimitH(IntervalHigh);  //ȷ����ɨ�����䲢��������
    GetJumpPointFromDet(PicTemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);     

    if (JumpPoint[0].type =='W')                                                    //�����������߲��������䣬����10���㶼�ǰ׵�
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                 //�������������һ�е���ֵ
    } else                                                                          //���������
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                             //��¼������
    }

    if (JumpPoint[1].type == 'W')                                                   //��������ұ��߲���������
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;               //�����ұ�������һ�е���ֵ
    } else                                                                          //�ұ�������
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                            //��¼������
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                 //��¼�����Ƿ��ҵ����ߣ�����������
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

    //����ȷ����Щ������ı�Ե
    if (( ImageDeal[Ysite].IsLeftFind == 'H' 
         ||ImageDeal[Ysite].IsRightFind == 'H')) {
      if (ImageDeal[Ysite].IsLeftFind == 'H')                                   //�������ߴ�����
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);
             Xsite <= (ImageDeal[Ysite].RightBorder - 1);
             Xsite++)                                                           //���ұ���֮������ɨ��
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0)) {
            ImageDeal[Ysite].LeftBorder =Xsite;                                 //�����һ������ߵ��ұ��кڰ�������Ϊ���Ա���ֱ��ȡ��
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0)                                   //һ�����ְ׵���ֱ������
            break;
          else if (Xsite ==(ImageDeal[Ysite].RightBorder - 1))   
          {
             ImageDeal[Ysite].LeftBorder = Xsite;
             ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
        }

      if (ImageDeal[Ysite].IsRightFind == 'H')
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);
             Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--) {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0)) {
            ImageDeal[Ysite].RightBorder =
                Xsite;                    //����ұ��ߵ���߻��кڰ�������Ϊ���Ա���ֱ��ȡ��
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1)) 
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
        }
    }
      if ((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <=7)                              //ͼ�����޶�
      {
        ImageStatus.OFFLine = Ysite + 1;  //������б�7С�˺���ֱ�Ӳ�Ҫ��
        break;
      }
   /***********����ȷ���ޱ���************/
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;


    if( SystemData.SpeedData.Length * OX <SystemData.barn_lenth
            &&ImageStatus.Road_type != Ramp){
    if (    ImageDeal[Ysite].IsRightFind == 'W'
          &&Ysite > 10
          &&Ysite < 50
          &&ImageStatus.Road_type!=Barn_in
          )                     //������ֵ��ޱ���
    {
      if (Get_R_line == 'F')    //��һ֡ͼ��û���ܹ�����һ�׼�ߵĴ���β�����
      {
        Get_R_line = 'T';       //����  һ֡ͼ��ֻ��һ�� ��ΪT 
        ytemp_W_R = Ysite + 2;  
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsRightFind =='T')  //���ޱ�����������  һ�㶼���бߵ�
            R_found_point++;
        }
        if (R_found_point >8)                      //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�   ���бߵĵ�������8
        {
          D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder - ImageDeal[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));     
                                                  //��������Щ����������б��
                                                  //�ø��ޱ������ӳ��������׼
          if (D_R > 0) {
            R_Found_T ='T';                       //���б�ʴ���0  ��ô�ҵ��������׼��  ��Ϊ���λ���
                                                  //����һ���������б�ʴ���0  С��0�����Ҳ�����ӳ� û��Ҫ
          } else {
            R_Found_T = 'F';                      //û���ҵ������׼��
            if (D_R < 0)
              ExtenRFlag = 'F';                   //�����־λ����ʮ�ֽǵ㲹��  ��ֹͼ�����õ�
          }
        }
      }
      if (R_Found_T == 'T')
        ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  //����ҵ��� ��ô�Ի�׼�����ӳ���

      LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
      LimitH(ImageDeal[Ysite].RightBorder);  //�޷�
    }

    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 &&
        ImageStatus.Road_type != Barn_in)    //����ͬ��  ��߽�
    {
      if (Get_L_line == 'F') {
        Get_L_line = 'T';
        ytemp_W_L = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsLeftFind == 'T')
            L_found_point++;
        }
        if (L_found_point > 8)              //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�
        {
          D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -ImageDeal[Ysite + L_found_point].LeftBorder)) /((float)(L_found_point - 3));
          if (D_L > 0) {
            L_Found_T = 'T';

          } else {
            L_Found_T = 'F';
            if (D_L < 0)
              ExtenLFlag = 'F';
          }
        }
      }

      if (L_Found_T == 'T')
        ImageDeal[Ysite].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);

      LimitL(ImageDeal[Ysite].LeftBorder);  //�޷�
      LimitH(ImageDeal[Ysite].LeftBorder);  //�޷�
    }
}

    if (   ImageDeal[Ysite].IsLeftFind == 'W'
         &&ImageDeal[Ysite].IsRightFind == 'W')
          ImageStatus.WhiteLine++;  //Ҫ�����Ҷ��ޱߣ�������+1

      LimitL(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitH(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
      LimitH(ImageDeal[Ysite].RightBorder);  //�޷�

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    if (ImageDeal[Ysite].Wide <= 7)         //����ȷ�����Ӿ���
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }

    else if (  ImageDeal[Ysite].RightBorder <= 10 
             ||ImageDeal[Ysite].LeftBorder >= 70) {
              ImageStatus.OFFLine = Ysite + 1;
              break;
    }                                        //��ͼ����С��0�������ұߴﵽһ��������ʱ������ֹѲ��
  }


  return;
}

//�ӳ��߻��ƣ���������˵�Ǻ�׼ȷ��
static void DrawExtensionLine(void)        //�����ӳ��߲�����ȷ������ ���Ѳ��߲���б��
{
  if (    Fork_dowm == 0 
        &&ImageStatus.CirquePass == 'F' 
        &&ImageStatus.IsCinqueOutIn == 'F' 
        &&ImageStatus.CirqueOut == 'F' 
        &&ImageStatus.Road_type != Forkin 
        &&ImageStatus.Road_type != Forkout
        &&ImageStatus.Road_type != Barn_in
        &&ImageStatus.Road_type != Ramp
        )                                  // g5.22  6.22����ע��  �ǵøĻ���
  {
    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == LeftCirque)
      TFSite = 55;
    if (ExtenLFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)                       //�ӵ����п�ʼ����ɨɨ��������������   ��β���
                                          //������ֻ��һ��
      {
        PicTemp = Pixle[Ysite];           //�浱ǰ��
        if (ImageDeal[Ysite].IsLeftFind =='W')                          //���������߽�ûɨ����ɨ�����ǰ�ɫ��˵������û����߽��
        {
          //**************************************************//**************************************************
          if (ImageDeal[Ysite + 1].LeftBorder >= 70)                    //�����߽�ʵ����̫�ұ�
          {
            ImageStatus.OFFLine = Ysite + 1;
            break;                        //ֱ�����������������
          }
          //************************************************//*************************************************

          while (Ysite >= (ImageStatus.OFFLine + 4))                    //��ʱ��ûɨ������
          {
            Ysite--;                      //��������ɨ
            if (  ImageDeal[Ysite].IsLeftFind == 'T' 
                &&ImageDeal[Ysite - 1].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 2].IsLeftFind == 'T' 
                &&ImageDeal[Ysite - 2].LeftBorder > 0
                &&ImageDeal[Ysite - 2].LeftBorder <70
                )                                                       //���ɨ�����г����˲��ұ��������������ж�����߽�㣨��߽��ڿհ��Ϸ���
            {
              FTSite = Ysite - 2;          //�ѱ�������ĵڶ��д���FTsite
              break;
            }
          }

          DetL =
              ((float)(ImageDeal[FTSite].LeftBorder -
                       ImageDeal[TFSite].LeftBorder)) /
              ((float)(FTSite - TFSite));  //��߽��б�ʣ��е������/�е������
          if (FTSite > ImageStatus.OFFLine)
            for (
                ytemp = TFSite; ytemp >= FTSite; ytemp--)               //�ӵ�һ��ɨ������߽������ڶ��е����꿪ʼ����ɨֱ���հ��Ϸ�����߽��������ֵ
            {
              ImageDeal[ytemp].LeftBorder =
                  (int)(DetL * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite]
                      .LeftBorder;                                      //�����ڼ�Ŀհ״����ߣ���б�ߣ���Ŀ���Ƿ���ͼ����
            }
        } else
          TFSite = Ysite + 2;                                           //���ɨ���˱��е���߽磬���д��������棬����б�ʣ�
      }

    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    // g5.22
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == RightCirque)
      TFSite = 55;
    if (ExtenRFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)               //�ӵ����п�ʼ����ɨɨ��������������
      {
        PicTemp = Pixle[Ysite];  //�浱ǰ��

        if (ImageDeal[Ysite].IsRightFind =='W')                       //��������ұ߽�ûɨ����ɨ�����ǰ�ɫ��˵������û���ұ߽�㣬���Ǵ��������ڵ�
        {
          if (ImageDeal[Ysite + 1].RightBorder <= 10)                 //����ұ߽�ʵ����̫���
          {
            ImageStatus.OFFLine =Ysite + 1;                           //ֱ��������˵�����������������������
            break;
          }
          while (Ysite >= (ImageStatus.OFFLine + 4))                  //��ʱ��ûɨ��������������
          {
            Ysite--;
            if (  ImageDeal[Ysite].IsRightFind == 'T' 
                &&ImageDeal[Ysite - 1].IsRightFind == 'T' 
                &&ImageDeal[Ysite - 2].IsRightFind == 'T' 
                &&ImageDeal[Ysite - 2].RightBorder < 79 
                &&ImageDeal[Ysite - 2].RightBorder > 10
                )                                                      //���ɨ�����г����˲��ұ��������������ж�����߽�㣨��߽��ڿհ��Ϸ���
            {
              FTSite = Ysite - 2;                                      // �ѱ�������ĵڶ��д���FTsite
              break;
            }
          }

          DetR =((float)(ImageDeal[FTSite].RightBorder -ImageDeal[TFSite].RightBorder)) /((float)(FTSite - TFSite));         //�ұ߽��б�ʣ��е������/�е������
          if (FTSite > ImageStatus.OFFLine)
            for (ytemp = TFSite; ytemp >= FTSite;ytemp--)              //�ӵ�һ��ɨ�����ұ߽������ڶ��е����꿪ʼ����ɨֱ���հ��Ϸ����ұ߽��������ֵ
            {
              ImageDeal[ytemp].RightBorder =(int)(DetR * ((float)(ytemp - TFSite))) +ImageDeal[TFSite].RightBorder;          //�����ڼ�Ŀհ״����ߣ���б�ߣ���Ŀ���Ƿ���ͼ����
            }
        } else
          TFSite =Ysite +2;                                           //������е��ұ߽��ҵ��ˣ���Ѹ�������ڶ��������͸�TFsite
      }
  }
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].Center =(ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /2;                                //ɨ�����������һ�龭�Ż�֮����м�ֵ����
    ImageDeal[Ysite].Wide =-ImageDeal[Ysite].LeftBorder +ImageDeal[Ysite].RightBorder;                                       //���Ż�֮��Ŀ�ȴ���
  }
}

//���ֶ��ߵ�ʱ��  ����ȷ���ޱ��е�����
static void RouteFilter(void) {
  for (Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5);
       Ysite--)                                     //�ӿ�ʼλ��ֹͣλ
  {
    if (   ImageDeal[Ysite].IsLeftFind == 'W'
         &&ImageDeal[Ysite].IsRightFind == 'W' 
         &&Ysite <= 45 
         &&ImageDeal[Ysite - 1].IsLeftFind == 'W' 
         &&ImageDeal[Ysite - 1].IsRightFind =='W')  //��ǰ�����Ҷ��ޱߣ�������ǰ45��   �˲�
    {
      ytemp = Ysite;
      while (ytemp >= (ImageStatus.OFFLine +5))     // �ĸ����ԣ�-6Ч����һЩ
      {
        ytemp--;
        if (  ImageDeal[ytemp].IsLeftFind == 'T' 
            &&ImageDeal[ytemp].IsRightFind == 'T')  //Ѱ�����߶������ģ��ҵ��뱾������ľͲ�����
        {
          DetR = (float)(ImageDeal[ytemp - 1].Center - ImageDeal[Ysite + 2].Center) /(float)(ytemp - 1 - Ysite - 2);          //��б��
          int CenterTemp = ImageDeal[Ysite + 2].Center;
          int LineTemp = Ysite + 2;
          while (Ysite >= ytemp) {
            ImageDeal[Ysite].Center =(int)(CenterTemp +DetR * (float)(Ysite - LineTemp));                                     //��б�ʲ�
            Ysite--;
          }
          break;
        }
      }
    }
    ImageDeal[Ysite].Center =(ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) /3;                                  //��ƽ����Ӧ�û�Ƚϻ�  ��������������ƽ��
  }
}

int icm_start_test_cross = 0;  //����icm���ֱ�־λ

/****ʮ�ּ��*****/  //������㷨�Ѿ��˳���ʮ��  �൱�ڲ�Ҫ����
void Cross_Test() {
  if (ImageStatus.OFFLine > 15)                       //�����Ӿ���ȽϽ���ʱ�� �������ִ���  �������⴦��270�����֮���ʮ��  
    icm_start_test_cross = 1;
  else
    icm_start_test_cross = 0;

  if (abs(icmdata.Yaw) > 100)
    ImageStatus.Road_type = Cross;
  if (ImageStatus.Road_type == Cross && ImageStatus.Cross_Lenth * OX > 100)
    ImageStatus.Road_type = 0;
}

//��ȡ���кڵ�����  ���û���õ�
int Black_Point = 0;
int Black_Point_ALL = 0;
void GetBlack_Point()  
{
  for (Ysite = 59; Ysite > 0; Ysite--) {
    PicTemp = Pixle[Ysite];
    for (Xsite = 0; Xsite < 80; Xsite++) {
      if (*(PicTemp + Xsite) == 0)
        Black_Point++;
    }
    Black_Point_ALL = Black_Point_ALL + Black_Point;
    ImageDeal[Ysite].Black_Point = Black_Point;
    Black_Point = 0;
  }
  ImageStatus.Black_Pro_ALL = Black_Point_ALL / 48;
  Black_Point_ALL = 0;
}


 //�������   ���û���õ�
void GetClose_Bord()  
{
  for (Ysite = 59; Ysite > 0; Ysite--) {
    PicTemp = Pixle[Ysite];
   
    for (Xsite = 0; Xsite < ImageStatus.MiddleLine; Xsite++) {
      if (*(PicTemp + Xsite) == 0 && (*(PicTemp + Xsite + 1) == 1) &&
          (*(PicTemp + Xsite + 2) == 1)) {
        ImageDeal[Ysite].close_LeftBorder = Xsite;
        break;
      } else
        ImageDeal[Ysite].close_LeftBorder = 0;
    }

    for (Xsite = 79; Xsite > ImageStatus.MiddleLine; Xsite--) {
      if (*(PicTemp + Xsite) == 0 && (*(PicTemp + Xsite - 1) == 1) &&
          (*(PicTemp + Xsite - 2) == 1)) {
        ImageDeal[Ysite].close_RightBorder = Xsite;
        break;
      } else
        ImageDeal[Ysite].close_RightBorder = 79;
    }

    //�ҷ������
    for (Xsite = ImageDeal[Ysite].close_LeftBorder + 1;
         Xsite < ImageStatus.MiddleLine; Xsite++) {
      if (*(PicTemp + Xsite) == 1 && (*(PicTemp + Xsite + 1) == 0) &&
          (*(PicTemp + Xsite + 2) == 0)) {
        ImageDeal[Ysite].opp_LeftBorder = Xsite;
        break;
      } else
        ImageDeal[Ysite].opp_LeftBorder = ImageDeal[Ysite].close_LeftBorder + 1;
    }

    for (Xsite = ImageDeal[Ysite].close_RightBorder - 1;
         Xsite > ImageStatus.MiddleLine; Xsite--) {
      if (*(PicTemp + Xsite) == 1 && (*(PicTemp + Xsite - 1) == 0) &&
          (*(PicTemp + Xsite - 2) == 0)) {
        ImageDeal[Ysite].opp_RightBorder = Xsite;
        break;
      } else
        ImageDeal[Ysite].opp_RightBorder =
            ImageDeal[Ysite].close_RightBorder - 1;
    }
  }
}

uint8 Pass_flag = 'F';
int for_err = 0;
int L_T_R_W;
uint8 Circle_Off = 'F';  //�ر�Բ����״̬λ  �����ޱ߾���
uint8 Out_Flag = 0;      //�Ƿ��ҵ������ǵ�
int circleoff_lenth;     //���������о���
int circle_just_one=1;


//�������
void CircleTest() {
  if ( (Car.status.magnetSensors.rawSum >= 6000) 
      &&ImageStatus.CirqueOut != 'T' 
      && ImageStatus.CirquePass != 'T'
      &&ImageStatus.CirqueOff !='T')                              //��Ŵ���ĳ����ֵ����û�д���Բ����  ��ô��⵽Բ��
  {
    ImageStatus.IsCinqueOutIn = 'T';                              //�˱�־λ����Բ������
  }

  if (ImageStatus.IsCinqueOutIn == 'T')                           //�ж���������
  {
    for (Ysite = 55; Ysite > (ImageStatus.OFFLine + 2); Ysite--)  //��Բ��
    {
      if ((ImageDeal[Ysite].LeftBorder + 8 < ImageDeal[Ysite - 2].LeftBorder) 
         &&ImageStatus.Road_type != RightCirque) {
        ImageStatus.Road_type = LeftCirque;
        Pass_flag = 'F';
        break;
      }
      else if (( ImageDeal[Ysite].RightBorder - 8 >ImageDeal[Ysite - 2].RightBorder) 
               &&ImageStatus.Road_type != LeftCirque)               //��Բ��
      {
        ImageStatus.Road_type = RightCirque;
        Pass_flag = 'F';
        break;
      }
      Pass_flag ='T';  
                                                                    // �Ѿ�ʶ��Բ��Բ������û���ҵ�����������˵����ʱ���Ѿ�����Բ����
                                                                    // �����Ϳ��Ըı�־λ��
    }

    if (Pass_flag == 'T')                                           //û��ʶ������������˵���Ѿ�������
    {
      if (icmdata.Yaw > 55 ||icmdata.Yaw < -55)                     // �����������źŵ� ˵���Ѿ�����Բ��
                                                                    // �е�����о��뻷��������Ҫ�Ľ�����
      {
        ImageStatus.CirquePass ='T';                                //�˱�־��ʾ�Ѿ����뵽Բ����  ͨ�������־λȡ���뻷����
        ImageStatus.IsCinqueOutIn = 'F';
        Pass_flag = 'F';                                            //�����־λ
      }
    }
  }

  if (ImageStatus.CirquePass == 'T' &&ImageStatus.Pass_Lenth * OX > 100)    //�Ѿ����뻷֮���Ҫ�жϳ����� //g5.22
                                                                            //��ûд �� ����е㲻��
  {
    if (ImageStatus.Road_type == RightCirque)                               //��Բ�������
    {
      for (Ysite = 53; Ysite > (ImageStatus.OFFLine + SystemData.OutCicle_line);Ysite--)  //��Բ����������������ߵĽ�  ʶ������Ǿ���
                                                                                          //���ǽ�����������̫ǰ  �ϰ벿�ֵ�ͼ���Ǻ���
      {
        if (((  ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite -10].LeftBorder >1)         //�м������С������
             &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite +10].LeftBorder > 1)) 
             ||(ImageDeal[Ysite + 1].IsLeftFind == 'T' 
              &&ImageDeal[Ysite].IsLeftFind == 'T' 
              &&ImageDeal[Ysite - 1].IsLeftFind == 'W')) {
          ImageStatus.CirquePass = 'F';
          ImageStatus.IsCinqueOutIn = 'F';
          ImageStatus.CirqueOut = 'T';                                                    //ʶ�𵽽ǵ�  ��ʱ״̬��ɳ���״̬

          break;
        }
      }
    }

    if (ImageStatus.Road_type == LeftCirque)                                              //g������ͬ��  ��Բ��
    {
      for (Ysite = 53; Ysite > (ImageStatus.OFFLine + SystemData.OutCicle_line);Ysite--)   
      {
        if (((ImageDeal[Ysite -10].RightBorder - ImageDeal[Ysite].RightBorder >1)
           &&(ImageDeal[Ysite +10].RightBorder - ImageDeal[Ysite].RightBorder >1)) 
           ||(ImageDeal[Ysite + 1].IsRightFind == 'T'
           && ImageDeal[Ysite].IsRightFind == 'T'
           && ImageDeal[Ysite - 1].IsRightFind == 'W')) {
              ImageStatus.CirquePass = 'F';
              ImageStatus.IsCinqueOutIn = 'F';
              ImageStatus.CirqueOut = 'T';

          break;
        }
      }
    }
  }

  if (ImageStatus.CirqueOut == 'T')                       //�ڳ�����״̬  ���г����Ĳ��ߴ���  ��ҲҪ�ж�ʲôʱ��ֹͣ����
                                                          //û��������  ������߼����Ǻ���  ��Ϊͼ����ʱ���е��
  {
    if (icmdata.Yaw > 300 || icmdata.Yaw < -280)          //   ���������ֹͣ���ߵ�����������
                                                          // ��ô�ͽ���Բ��  �����ص�����״̬
    {
      ImageStatus.CirquePass = 'F';
      ImageStatus.IsCinqueOutIn = 'F';
      ImageStatus.CirqueOut = 'F';
      ImageStatus.CirqueOff = 'T';                        //�˱�־��ʾ��������Ҫ��һ�ξ������ж��뻷  ��ֹ�����뻷
    }
  }

  if (ImageStatus.CirqueOff == 'T' 
      &&ImageStatus.Out_Lenth * OX >100)                 //�ڽ����������1-2m�����б�־λ��ʼ��������ֹ���ν���//150
                                                          //������������OFF��־��� �ص����״̬
  {
      ImageStatus.CirquePass = 'F';
      ImageStatus.IsCinqueOutIn = 'F';
      ImageStatus.CirqueOut = 'F';
      ImageStatus.CirqueOff = 'F';
      ImageStatus.Road_type = 0;  //�ص�ԭ����״̬
      circle_just_one=0;
      SystemData.clrcle_num++;
  }
}

static int Yjump1 = 0;
int circle_r = 12;
float circlk_k = 1.02;
void Circle_Handle() {
    //�뻷����
  if (ImageStatus.IsCinqueOutIn == 'T')                           //��ʱ�����뻷״̬
  {
    //��Բ��
    if (ImageStatus.Road_type == LeftCirque) {
      for (Ysite = 55; Ysite > ImageStatus.OFFLine + 2; Ysite--)  //�ҵ����ߵ�
      {
        if (ImageDeal[Ysite].LeftBorder + 8 < ImageDeal[Ysite - 2].LeftBorder) {
          Yjump1 = Ysite;
          ImageStatus.OFFLine = Yjump1;
          break;
        }
      }

      if (Yjump1 > 0) {                                           //��ʼ����
        for (Ysite = Yjump1; Ysite < 55; Ysite++) {
          int temp =ImageDeal[Yjump1].LeftBorder + circle_r * sqrt(Ysite - Yjump1);
          if (temp < ImageDeal[Ysite].RightBorder)
          {
            ImageDeal[Ysite].RightBorder = temp;
            if (Pixle[Ysite][ImageDeal[Ysite].RightBorder] == 0) {
              for (Xsite = ImageDeal[Ysite].RightBorder;Xsite > ImageDeal[Ysite].LeftBorder; Xsite--) {
                if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite - 1] == 1) {
                    ImageDeal[Ysite].RightBorder = Xsite;
                }
              }
            }
          }
          ImageDeal[Ysite].Center =
              (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
        }
      }
    }

    //��Բ��
    if (ImageStatus.Road_type == RightCirque) {
      for (Ysite = 55; Ysite > ImageStatus.OFFLine + 2; Ysite--)  //�ҵ����ߵ�
      {
        if (ImageDeal[Ysite].RightBorder - 8 >ImageDeal[Ysite - 2].RightBorder) {
            Yjump1 = Ysite;
            ImageStatus.OFFLine = Yjump1;
            break;
        }
      }

      if (Yjump1 > 0) {                                           //��ʼ����
        for (Ysite = Yjump1; Ysite < 55; Ysite++) {
          int temp =ImageDeal[Yjump1].RightBorder - circle_r * sqrt(Ysite - Yjump1);
          if (temp > ImageDeal[Ysite].LeftBorder)
          {
            ImageDeal[Ysite].LeftBorder = temp;
            if (Pixle[Ysite][ImageDeal[Ysite].LeftBorder] == 0) {
              for (Xsite = ImageDeal[Ysite].LeftBorder;
                   Xsite < ImageDeal[Ysite].RightBorder; Xsite++) {
                if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 1) {
                  ImageDeal[Ysite].LeftBorder = Xsite;
                }
              }
            }
          }

          ImageDeal[Ysite].Center =
              (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
        }
      }
    }
  }

  ////��������

  if (ImageStatus.CirqueOut == 'T')  //��ʱ���ڳ���״̬
  {
    //��Բ��
    float Det_R;
    if (ImageStatus.Road_type == LeftCirque) {
      Det_R = circlk_k * 79 / (55 - ImageStatus.OFFLine);                         //���㲹��б��
      for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
        int temp = (int)(ImageDeal[55].RightBorder - Det_R * (55 - Ysite));
        if (temp < 0) {
          temp = 0;
        }
        if (temp < ImageDeal[Ysite].RightBorder) {                                //����
          ImageDeal[Ysite].RightBorder = temp;
          ImageDeal[Ysite].LeftBorder = 0;
          ImageDeal[Ysite].Center =(ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
          ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
        }
      }
    }
//ͬ��
    float Det_L;
    if (ImageStatus.Road_type == RightCirque) {
      Det_L = circlk_k * 79 / (55 - ImageStatus.OFFLine);
      for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
        int temp = (int)(ImageDeal[55].LeftBorder + Det_L * (55 - Ysite));
        if (temp > 79) {
          temp = 79;
        }
        if (temp > ImageDeal[Ysite].LeftBorder) {
          ImageDeal[Ysite].LeftBorder = temp;
          ImageDeal[Ysite].RightBorder = 79;
          ImageDeal[Ysite].Center =
              (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
          ImageDeal[Ysite].Wide =
              ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
        }
      }
    }
  }
}

//����ڼ��
uint8 f1 = 0;  //û��
uint8 f2 = 0;  //û��
uint8 f3 = 0;  //û��
uint8 Fork_in_1 = 0;//����������
uint8 Fork_in_2 = 0;//Զ��������
uint8 Fork_in = 0;  //�������־
uint8 ForkLinePointx_l = 0;//���浹���������
uint8 ForkLinePointx_r = 0;//���浹�����ұ���
int ForkLinePointy = 0;    //���浹���ǵױ���
int Fork1_Y = 0;           //�������������
int Fork2_Y = 0;           //����Զ��������          
int Fork_dowm = 0;         //������ٱ�־  �����������˳�
void ForkTest() {
  int wide = 0;   //��ʱ����
  int Fork_thr;   //�л�ǿ���жϵ���ֵ
  if (SystemData.Model < 2)
    Fork_thr = 5000;
  else
    Fork_thr = 5000;

  //����ֱ���䵽����ĵ�һ����
  if (Car.status.magnetSensors.rawSum > Fork_thr)                 //������������
  {
    for (Ysite = 53; Ysite > (ImageStatus.OFFLine + 6);Ysite--)   //��ֹYsite���

    {
      if ((  ImageDeal[Ysite].IsRightFind == 'T' 
           &&ImageDeal[Ysite + 1].IsRightFind == 'T') 
          ||(ImageDeal[Ysite].IsLeftFind == 'T' 
           &&ImageDeal[Ysite + 1].IsLeftFind =='T'))            //�������ʱ��һ��ῴ����������120*��Բ��
      {
        if (((ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 6].LeftBorder) >2 
             &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 6].LeftBorder) <8 
             &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 6].LeftBorder) >2 
             &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 6].LeftBorder) <8)        //��ߵĽ�
            ||((ImageDeal[Ysite - 6].RightBorder - ImageDeal[Ysite].RightBorder) >3 
             &&(ImageDeal[Ysite - 6].RightBorder - ImageDeal[Ysite].RightBorder) <8 
             &&(ImageDeal[Ysite + 6].RightBorder - ImageDeal[Ysite].RightBorder) >2       //�ұߵĽ� ����һ���Ϳ���  ��Ϊ������������©��
             &&(ImageDeal[Ysite + 6].RightBorder -ImageDeal[Ysite].RightBorder) < 8))     //��ֵ��Ҫ����
        {
          Fork_in_1 = 'T';  //��ʾ���˵Ľǵ������ҵ�
          Fork1_Y = Ysite;  //��¼��һ�����������
          break;
        }

        else {
          Fork_in_1 = 'F';  //û�ҵ�GG
        }
      }
    }

    //�ڶ�����  �Һ�ɫ���ǿ�  ������õ����ͼ����Ϣ
    for (Ysite = Fork1_Y; Ysite > (ImageStatus.OFFLine);Ysite--)                                          //�ӵ�һ�����㿪ʼ��������
    {
      PicTemp = Pixle[Ysite];
      for (Xsite = ImageDeal[Ysite].LeftBorder; Xsite < 50;Xsite++)                                       //������ں�ɫ���ǿ�
      {
        if ((*(PicTemp + Xsite) != 0) && (*(PicTemp + Xsite + 1) == 0) &&(*(PicTemp + Xsite + 2) == 0))   //�ҵ���ɫ�ǿ�����
        {
          ImageDeal[Ysite].Black_Wide_L = Xsite + 1;                                                       //��¼��ʱ����ڱ߽�
          break;
        } else
          ImageDeal[Ysite].Black_Wide_L =ImageDeal[Ysite].Center;                                          //û�ҵ������е�
      }

      for (Xsite = ImageDeal[Ysite].RightBorder; Xsite > 30;Xsite--)                                       //������ں�ɫ���ǿ�
      {
        if (  (*(PicTemp + Xsite) == 0) 
           && (*(PicTemp + Xsite - 1) == 0) 
            &&(*(PicTemp + Xsite + 1) != 0))      //�ҵ���ɫ�ǿ���ұ�
        {
          ImageDeal[Ysite].Black_Wide_R = Xsite;  //��¼��ʱ���Һڱ߽�
          break;
        } else
          ImageDeal[Ysite].Black_Wide_R = ImageDeal[Ysite].Center;                                          //û�ҵ������е�
      }

      for (Xsite = ImageDeal[Ysite].Black_Wide_L;
           Xsite <= ImageDeal[Ysite].Black_Wide_R; Xsite++) {
        if (ImageDeal[Ysite].Black_Wide_L == ImageDeal[Ysite].Center 
            ||ImageDeal[Ysite].Black_Wide_R ==ImageDeal[Ysite].Center)                                       //������е�ֵ��ôGG��Ϊ����û�ҵ�
          break;
        else if ((*(PicTemp + Xsite) == 0))     //������ں��Һ�֮��ĺڵ���
        {
          wide++;                               //�����ҵ����ǿ�ı��кڵ���
        }
      }

      ImageDeal[Ysite].BlackWide = wide;        //��¼������
      ImageDeal[Ysite].Black_Pro =ImageDeal[Ysite].BlackWide / ImageDeal[Ysite].Wide;                         //ͼ��ڵ����
      wide = 0;                                 //��0
    }

    //�ж��Ƿ�Ϊ����ĺ�ɫ���ǿ�
    for (Ysite = Fork1_Y; Ysite >= (ImageStatus.OFFLine + 1); Ysite--)  // g
    {
      if (( ImageDeal[Ysite].BlackWide - ImageDeal[Ysite + 3].BlackWide) >=2 //��������ں��Һ�֮��ڵ����Ƚ϶� �������������ε���״                                                             
          &&ImageDeal[Ysite].BlackWide > 21 
          &&ImageDeal[Ysite + 1].BlackWide > 17 
          &&ImageDeal[Ysite - 1].BlackWide > 17
          &&(39 - ImageDeal[Ysite].Black_Wide_L) > 0                        //�˳�бʮ��
          &&(ImageDeal[Ysite].Black_Wide_R - 39) > 0) {
        ForkLinePointx_r = ImageDeal[Ysite].Black_Wide_R;                   //���ڲ��ߵĵ�
        ForkLinePointx_l = ImageDeal[Ysite].Black_Wide_L;
        ForkLinePointy = Ysite;
        Fork2_Y = Ysite;                                                    //��¼�ڶ������������
        if (Fork1_Y - Fork2_Y > 6)  
        {
          Fork_in_2 ='T';                                                   //������������������10   ���ж�Ϊ�뻷������  ���ڷ�����
          break;
        }

      } else
        Fork_in_2 = 'F';
    }

    if ((Fork_in_1 == 'T') && (Fork_in_2 == 'T'))
      Fork_in = 'T';
    else if ((Fork_in == 'T') && (Fork_in_2 == 'T'))
      Fork_in = 'T';
    else
      Fork_in = 'F';

    if (Fork_in == 'T')
      ImageStatus.Road_type = Forkin;
  }

  //�Ѿ����������   ���ͷֱ���ֵ  ���õ�ʶ������   �Ͳ���ʶ���һ������
  //ֻҪʶ��ڶ������� ͬ��
  else if (Car.status.magnetSensors.rawSum < Fork_thr) {
    //�ڶ�����  �Һ�ɫ���ǿ�  ������õ����ͼ����Ϣ
    for (Ysite = 55; Ysite > (ImageStatus.OFFLine); Ysite--)  
    {
      PicTemp = Pixle[Ysite];
      for (Xsite = ImageDeal[Ysite].LeftBorder; Xsite < 55;Xsite++)  //������ں�ɫ���ǿ�
      {
        if ((  *(PicTemp + Xsite) != 0) 
            &&(*(PicTemp + Xsite + 1) == 0) 
            &&(*(PicTemp + Xsite + 2) == 0))                          //�ҵ���ɫ�ǿ����
        {
          ImageDeal[Ysite].Black_Wide_L = Xsite + 1;
          break;
        } else
          ImageDeal[Ysite].Black_Wide_L = ImageDeal[Ysite].Center;
      }

      for (Xsite = ImageDeal[Ysite].RightBorder; Xsite > 25;Xsite--)  //������ں�ɫ���ǿ�//g
      {
        if ((   *(PicTemp + Xsite) == 0) 
             &&(*(PicTemp + Xsite - 1) == 0)
             &&(*(PicTemp + Xsite + 1) != 0))                         //�ҵ���ɫ�ǿ��ұ�
        {
          ImageDeal[Ysite].Black_Wide_R = Xsite;
          break;
        } else
          ImageDeal[Ysite].Black_Wide_R = ImageDeal[Ysite].Center;
      }




      for (Xsite = ImageDeal[Ysite].Black_Wide_L;Xsite <= ImageDeal[Ysite].Black_Wide_R; Xsite++) {
        if (  ImageDeal[Ysite].Black_Wide_L == ImageDeal[Ysite].Center 
            ||ImageDeal[Ysite].Black_Wide_R == ImageDeal[Ysite].Center)
          break;
        else if ((*(PicTemp + Xsite) == 0)) {
          wide++;
        }
      }

      ImageDeal[Ysite].BlackWide = wide;
      ImageDeal[Ysite].Black_Pro =ImageDeal[Ysite].BlackWide / ImageDeal[Ysite].Wide;  //ͼ��ڵ����
      wide = 0;
      if (ImageDeal[Ysite].BlackWide > 2)
        ImageDeal[Ysite].isBlackFind = 'T';
      else
        ImageDeal[Ysite].isBlackFind = 'F';
    }

    //�ж��Ƿ�Ϊ����ĺ�ɫ���ǿ�
    for (Ysite = 55; Ysite >= (ImageStatus.OFFLine); Ysite--)  // g
    {
      if (( ImageDeal[Ysite].BlackWide - ImageDeal[Ysite + 3].BlackWide) > 1 
          &&ImageDeal[Ysite].BlackWide > 16 
          &&ImageDeal[Ysite + 1].BlackWide > 15
          && (39 - ImageDeal[Ysite].Black_Wide_L) > 0                                 //�˳�бʮ��
          && (ImageDeal[Ysite].Black_Wide_R - 39) > 0) {
        ForkLinePointx_r = ImageDeal[Ysite].Black_Wide_R;
        ForkLinePointx_l = ImageDeal[Ysite].Black_Wide_L;
        ForkLinePointy = Ysite;
        Fork_in_2 = 'T';
        break;
      } else
        Fork_in_2 = 'F';
    }
    if (Fork_in_2 == 'T')
      Fork_in = 'T';
    else
      Fork_in = 'F';

    f3 = Fork_in;
    if (Fork_in == 'T') {
      ImageStatus.Road_type = Forkout;
    }
  }

  if (ImageStatus.Road_type == Forkin || ImageStatus.Road_type == Forkout)
    Fork_dowm = 1;

}

void Fork_Handle() {
  float Det_Fork_L;
  float Det_Fork_R;
  if (  (ImageStatus.Road_type == Forkin || ImageStatus.Road_type == Forkout) 
      &&(ImageStatus.Barn_Flag == 1))                                                   //��һȦ�Ҳ���
  {
    Det_Fork_R = 0.7*(79 - ForkLinePointx_l) / (55 - ForkLinePointy);
    for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      int temp = (int)(ImageDeal[55].RightBorder - Det_Fork_R * (55 - Ysite));
      if (temp < 0) {
        temp = 0;
      }
      if (temp < ImageDeal[Ysite].RightBorder) {
        ImageDeal[Ysite].RightBorder = temp;
        if (Pixle[Ysite][ImageDeal[Ysite].RightBorder] == 0) {
          for (Xsite = ImageDeal[Ysite].RightBorder;Xsite > ImageDeal[Ysite].LeftBorder; Xsite--) {
            if (Pixle[Ysite][Xsite] == 1 
             && Pixle[Ysite][Xsite - 1] == 1) {
              ImageDeal[Ysite].RightBorder = Xsite;
            }
          }
        }
      }
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
      ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder -ImageDeal[Ysite].LeftBorder;  //��ȸ���
    }
  }
  if ((ImageStatus.Road_type == Forkin || ImageStatus.Road_type == Forkout) &&
      ImageStatus.Barn_Flag == 0)   //�ڶ�Ȧ����
  {
    Det_Fork_L = 1.05*ForkLinePointx_r / (55 - ForkLinePointy);
    for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      int temp = (int)(ImageDeal[55].LeftBorder + Det_Fork_L * (55 - Ysite));
      if (temp > 79) {
        temp = 79;
      }
      if (temp > ImageDeal[Ysite].LeftBorder) {
        ImageDeal[Ysite].LeftBorder = temp;
        if (Pixle[Ysite][ImageDeal[Ysite].LeftBorder] == 0) {
          for (Xsite = ImageDeal[Ysite].LeftBorder;Xsite < ImageDeal[Ysite].RightBorder; Xsite++) {
            if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 1) {
              ImageDeal[Ysite].LeftBorder = Xsite;
            }
          }
        }
      }
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
      ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder -ImageDeal[Ysite].LeftBorder;  //��ȸ���
    }
  }
}
int ramp_flag = 1;
void Barn_test_in() {
  uint8 j = 0;

  for (Ysite = 30; Ysite < 55; Ysite++) {
    j = 0;
    for (Xsite = 0; Xsite < 79; Xsite++) {
      if (Pixle[Ysite][Xsite] != Pixle[Ysite][Xsite + 1])  //��⵽�кڰ������
        j++;
    }
    if (j > 9 &&rampnum==2) {
      ImageStatus.Road_type = Barn_in;
      ramp_flag = 1;                                       //��⵽��Ͱ������־λ��1
      rampnum=0;
      barnlenth=SystemData.SpeedData.Length;
      SystemData.clrcle_num = 0;
      circle_just_one=1;
      ++SystemData.rounds;
      break;
    }
  }

  if (  ImageStatus.Road_type == Barn_in 
      &&ImageStatus.Barn_Lenth * OX > 100 
      &&ImageStatus.Barn_Flag == 0)                         //��һ�μ�⵽����  ����־λ��Ϊ1
  {
    ImageStatus.Barn_Flag = 1;
    ImageStatus.Road_type = 0;
    SystemData.SpeedData.Length = 0;
  }

  if (ImageStatus.Road_type == Barn_in 
    &&ImageStatus.Barn_Lenth * OX > 100 
    &&ImageStatus.Barn_Flag == 1)                         //�ڶ���ʶ��  ʶ���˾Ͱѱ�־λ��Ϊ2  ��Ϊ����Ȧ���Ե�2�Ž���
  {
      ImageStatus.Barn_Flag = 0;
      ImageStatus.Road_type = 0;
      SystemData.SpeedData.Length = 0;
  }
}


//��ⲹ��  ���û��
static int Yjump4 = 0;
void Barn_in_Handle(void) {
  if (ImageStatus.Barn_Flag == 2) {
    for (Ysite = 10; Ysite < 59; Ysite++) {
      if ((ImageDeal[Ysite].close_RightBorder -
           ImageDeal[Ysite - 2].close_RightBorder) > 5) {
        Yjump4 = Ysite - 2;
        break;
      }
    }
    ImageStatus.OFFLine = Yjump4;
    if (Yjump4 > 0) {
      for (Ysite = Yjump4; Ysite < 55; Ysite++) {
        int temp =
            (int)(ImageDeal[Yjump4].close_RightBorder - (Ysite - Yjump4));
        if (temp < 0)
          temp = 0;
        ImageDeal[Ysite].LeftBorder = temp;
        ImageDeal[Ysite].RightBorder = 79;
        ImageDeal[Ysite].Center =
            (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
      }
    }
  }
}

//���ⲹ��  ���û��
void Barn_out_Handle(void) {
  if (ImageStatus.Road_type == Barn_out) {
    for (Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
      int temp = (int)(ImageDeal[59].LeftBorder + 1.2 * (59 - Ysite));
      if (temp > 79)
        temp = 79;

      ImageDeal[Ysite].LeftBorder = temp;
      ImageDeal[Ysite].RightBorder = 79;
      ImageDeal[Ysite].Center =
          (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
    }
  }
}

//���ڼ��ٵ�ֱ�����
float variance, variance_acc;  //����
void Straightacc_Test(void) {
  int sum = 0;
  for (Ysite = 55; Ysite > ImageStatus.OFFLine + 1; Ysite--) {
    sum += (ImageDeal[Ysite].Center - ImageStatus.MiddleLine) *(ImageDeal[Ysite].Center - ImageStatus.MiddleLine);
  }
  variance_acc = (float)sum / (54 - ImageStatus.OFFLine);

  if ( variance_acc < ImageStatus.variance_acc 
    && ImageStatus.OFFLine <= 4 
    && ImageStatus.Road_type != Ramp) {
    ImageStatus.straight_acc = 1;
  } else
    ImageStatus.straight_acc = 0;
}

//���ڱ������ֱ�����
void Straight_Test_2(void) {
  float midd_k, sum;
  midd_k = (ImageDeal[55].Center - ImageDeal[ImageStatus.OFFLine + 1].Center) /(float)(55 - ImageStatus.OFFLine - 1);
  for (Ysite = 55; Ysite > ImageStatus.OFFLine + 1; Ysite--) {
    ImageDeal[Ysite].mid_temp =ImageDeal[55].Center - midd_k * (55 - Ysite) + 0.5;
    sum += pow(ImageDeal[Ysite].Center - ImageDeal[Ysite].mid_temp, 2);
  }

      variance = sum / (54 - ImageStatus.OFFLine);
  if (variance < ImageStatus.variance && ImageStatus.OFFLine <= 9 ) {
      ImageStatus.Road_type = Straight;
  }
}


/****�µ����***/  
int ramptestflag=1;//�µ��жϱ�־λ
int rampnum=0;     //�µ�����
void Ramp_Test() {
  int i = 0;
  if (ImageStatus.OFFLine == 2 && ImageStatus.Ramp_lenth == 0&&ramptestflag==1) {
    for (Ysite = ImageStatus.OFFLine; Ysite < 7; Ysite++) {
      if (       ImageDeal[Ysite].Wide > 26
//            && ImageDeal[Ysite].IsRightFind == 'T'
              &&ImageDeal[Ysite].IsLeftFind == 'T'
              &&ImageDeal[Ysite].LeftBorder < 40
              &&ImageDeal[Ysite].RightBorder > 40
              &&Pixle[Ysite][ImageDeal[Ysite].Center] == 1
              &&Pixle[Ysite][ImageDeal[Ysite].Center-2] == 1
              &&Pixle[Ysite][ImageDeal[Ysite].Center+2] == 1)
                i++;
      if (i >= 4) {
        ImageStatus.Road_type = Ramp;
        rampnum++;
        ramptestflag=0;
        break;
      }
    }
  }
  if (ImageStatus.Ramp_lenth * OX > 140) {
    ImageStatus.Road_type = 0;
    ramp_flag = 0;
  }

  if(ImageStatus.ramptestlenth* OX>400){
      ramptestflag=1;
  }
}



void Cross_Test2(){
    int leftlowlen=0;
    int leftmiddlen=0;
    int lefthighlen=0;
    int rightlowlen=0;
    int rightmiddlen=0;
    int righthighlen=0;
        //�����ʮ������
    for (Ysite = 54; Ysite > ImageStatus.OFFLine+2; Ysite--) {

        //ʮ�ֵױߵ��б�
        if(ImageDeal[Ysite].IsLeftFind=='T'&&ImageDeal[Ysite-1].IsLeftFind=='T'&&leftlowlen==0){
            while(ImageDeal[Ysite].IsLeftFind=='T'){
                leftlowlen++;
                Ysite--;
                if(ImageDeal[Ysite].LeftBorder<ImageDeal[Ysite+5].LeftBorder-2){
                    leftlowlen=1;
                    break;
                }

            }
        }

        //ʮ���м�Ķ���
        if(ImageDeal[Ysite].IsLeftFind=='W'&&ImageDeal[Ysite-1].IsLeftFind=='W'&&leftmiddlen==0){
            while(ImageDeal[Ysite].IsLeftFind=='W'){
                leftmiddlen++;
                Ysite--;
            }
        }

        //ʮ���ϱߵ��б�
        if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite-1].IsRightFind=='T'&&lefthighlen==0){
            while(ImageDeal[Ysite].IsRightFind=='T'){
                lefthighlen++;
                Ysite--;
            }
        }


    }



        //���ұ�ʮ������
    for (Ysite = 54; Ysite > ImageStatus.OFFLine+2; Ysite--) {


        //ʮ�ֵױߵ��б�
        if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite-1].IsRightFind=='T'&&rightlowlen==0){
            while(ImageDeal[Ysite].IsRightFind=='T'){
                rightlowlen++;
                Ysite--;
                if(ImageDeal[Ysite].RightBorder>ImageDeal[Ysite+5].RightBorder+2){
                    rightlowlen=1;
                    break;
                }
            }
        }

        //ʮ�ֵױߵĶ���
        if(ImageDeal[Ysite].IsRightFind=='W'&&ImageDeal[Ysite-1].IsRightFind=='W'&&rightmiddlen==0){
            while(ImageDeal[Ysite].IsRightFind=='W'){
                rightmiddlen++;
                Ysite--;
            }
        }

        //ʮ���ϱߵ��б�
        if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite-1].IsRightFind=='T'&&righthighlen==0){
            while(ImageDeal[Ysite].IsRightFind=='T'){
                righthighlen++;
                Ysite--;
            }
        }

    }




    if(     leftlowlen>3
          &&leftmiddlen>7
          &&lefthighlen>3
          &&rightlowlen>3
          &&rightmiddlen>7
          &&righthighlen>3
          )
        ImageStatus.Road_type=Cross_ture;

     if(ImageStatus.Cross_ture_lenth*OX>60)
        ImageStatus.Road_type=0;
     else
         Fork_dowm=0;


}



/****Ԫ�ؼ��*****/  //Բ�� ʮ�� ���� ��Ҫͨ�������������Լ�����Ԫ�ر�־  ������ֻ��һ˲��
                     //����û��⵽������ֱ�Ӵ˴�����
void Element_Test(void) {
  if (  ImageStatus.Road_type != Cross  //����������0
      &&ImageStatus.Road_type != LeftCirque
      &&ImageStatus.Road_type != RightCirque
      &&ImageStatus.Road_type != Barn_in
      &&ImageStatus.Road_type != Ramp
      &&ImageStatus.Road_type != Cross_ture
      )
    ImageStatus.Road_type = 0;

  if (ImageStatus.Road_type != Cross  //����������0
      &&ImageStatus.Road_type != LeftCirque
      &&ImageStatus.Road_type != RightCirque
      &&ImageStatus.Road_type != Barn_in
      &&ImageStatus.Road_type != Ramp
      &&ImageStatus.Road_type != Cross_ture
      )
    Straight_Test_2();

    Straightacc_Test();

  if (ImageStatus.Road_type <= Ramp && Fork_dowm == 0&&SystemData.SpeedData.Length * OX>SystemData.ramp_lenth_start)//1800
    Ramp_Test();                     //�µ����



  if (ImageStatus.Road_type != LeftCirque &&
      ImageStatus.Road_type != RightCirque &&
      ImageStatus.Road_type != Ramp)  //Բ�����ƽǶ����
    Cross_Test();


//  if (      ImageStatus.Road_type != LeftCirque
//          &&ImageStatus.Road_type != RightCirque
//          &&ImageStatus.Road_type != Ramp
//          &&ImageStatus.Road_type != Cross
////          &&ImageStatus.Road_type != Straight
//          )  //Բ�����ƽǶ����
//    Cross_Test2();


  if (ImageStatus.Road_type != Barn_in  //Բ�����
      && ImageStatus.Road_type != Cross
      && ImageStatus.Road_type != Barn_out
      && ImageStatus.Road_type != Ramp
      )
    CircleTest();

  if (  ImageStatus.Road_type != Cross  //����·���
      &&ImageStatus.Road_type != Barn_in
      &&ImageStatus.Road_type != Barn_out 
      &&ImageStatus.Road_type != LeftCirque 
      &&ImageStatus.Road_type != RightCirque
      &&ImageStatus.Road_type != Ramp
      &&SystemData.SpeedData.Length * OX >SystemData.fork_lenth_start
      ) {
    ForkTest();
  }

  if (ImageStatus.Dowm_lenth * OX > 50)
    Fork_dowm = 0;


  if (   ImageStatus.Road_type != Barn_out
       &&ImageStatus.Road_type != Ramp
       )  
    Barn_test_in();                   //������

}

void Element_Handle() {
  Circle_Handle();  //Բ������
  Fork_Handle();    //����ڴ���
}
/****���紦��********/  
void Stop_Test() {                    
  if (SystemData.SpeedData.Length * OX > 150) {//ǿ����

      if (ImageStatus.OFFLine >= 55)           //�����ź�С���ҿ��Ӿ������û�оͱ�ʾ����
        SystemData.Stop = 1;  

  }
}

void Stop_Test2() {                            //������
  if (  ImageStatus.OFFLine >= 55 && SystemData.Stop == 0 
      &&SystemData.SpeedData.Length * OX > 150)
        SystemData.Stop = 2;

  if (      SystemData.Stop == 2
          &&ImageStatus.Stop_lenth * OX > 80
          &&ImageStatus.OFFLine >= 55)
    SystemData.Stop = 1;
  else if ( SystemData.Stop == 2
          &&ImageStatus.Stop_lenth * OX > 80
          &&ImageStatus.OFFLine < 55)
    SystemData.Stop = 0;
}


void DrawLine()  //���߽�  �õ���
{
  uint8 i;
  for (i = 59; i > ImageStatus.OFFLine; i--) {
    Pixle[i][ImageDeal[i].LeftBorder + 2] = 0;  //�ƶ���λ���ڹ۲�
    Pixle[i][ImageDeal[i].RightBorder - 2] = 0;
    Pixle[i][ImageDeal[i].Center] = 0;
  }
}

/*****************��Ȩ����������**********************/
void GetDet() {
  float DetTemp = 0;
  int TowPoint = 0;
  float SpeedGain = 0;
  float UnitAll = 0;

  SpeedGain=(SystemData.SpeedData.nowspeed - SystemData.SpeedData.MinSpeed) * 0.2 +0.5 ;//�����ٶȵ���ǰհ������   �ٶȸ���80  ����Ϊ����  ǰհ�ӳ�

    if (SpeedGain >= 3)
      SpeedGain = 3;  
    else if (SpeedGain <= -1)
      SpeedGain = -1;  

    if (ImageStatus.Road_type == RightCirque ||ImageStatus.Road_type == LeftCirque)
    TowPoint = 27;                                                                      //Բ��ǰհ
    else if (ImageStatus.Road_type == Straight) {
    TowPoint = SystemData.straighet_towpoint;
  } else if(Fork_dowm==1){                                                              //������ǰհ
      TowPoint=29;
  } else
    TowPoint = ImageStatus.TowPoint-SpeedGain;                                          //�ٶ�Խ��ǰհԽ��

  if (TowPoint < ImageStatus.OFFLine)
    TowPoint = ImageStatus.OFFLine + 1;
    
  if (TowPoint >= 49)
    TowPoint = 49;

  if ((TowPoint - 5) >= ImageStatus.OFFLine) {                                          //ǰհȡ�趨ǰհ���ǿ��Ӿ���  ��Ҫ���������
    for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
      DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);

  } else if (TowPoint > ImageStatus.OFFLine) {
    for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) {
      DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
  } else if (ImageStatus.OFFLine < 49) {
    for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);

  } else
    DetTemp =ImageStatus.Det_True;                                                     //����ǳ���OFFLine>50�����������һ�ε�ƫ��ֵ

  ImageStatus.Det_True = DetTemp;                                                      //��ʱ�Ľ��������ƽ��ͼ��ƫ��

  ImageStatus.TowPoint_True = TowPoint;                                                //��ʱ��ǰհ
}



float Det = 0;
void ImageProcess(void) {
    compressimage();          //ͼ��ѹ�� 0.6ms
    ImageStatus.OFFLine = 2;  //���ֵ������ʵ����õ��������������
    ImageStatus.WhiteLine = 0;
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].IsLeftFind = 'F';
    ImageDeal[Ysite].IsRightFind = 'F';
    ImageDeal[Ysite].LeftBorder = 0;
    ImageDeal[Ysite].RightBorder = 79;
    ImageDeal[Ysite].LeftTemp = 0;
    ImageDeal[Ysite].RightTemp = 79;
    ImageDeal[Ysite].Black_Wide_L = 39;
    ImageDeal[Ysite].Black_Wide_R = 39;
    ImageDeal[Ysite].BlackWide = 0;

    // g  5.12
    ImageDeal[Ysite].close_LeftBorder = 0;
    ImageDeal[Ysite].close_RightBorder = 0;
    ImageDeal[Ysite].opp_LeftBorder = 0;
    ImageDeal[Ysite].opp_RightBorder = 0;

  }                     //�߽����־λ��ʼ��

  Get01change_dajin();  //ͼ���ֵ��    2.7ms
  Pixle_Filter();       //��ʴ         1.7ms


  DrawLinesFirst();     //���Ƶױ�      30us
  DrawLinesProcess();   //�õ���������  8us

  /***Ԫ��ʶ��*****/
  Element_Test();                   //5us
  /***Ԫ��ʶ��*****/
  
  DrawExtensionLine();  //�õ��ӳ���   8us
  RouteFilter();        //�����˲�ƽ�� 2us
  
  /***Ԫ�ش���*****/
  Element_Handle();  // 3us
  /***Ԫ�ش���*****/

  if (ImageStatus.Road_type != Barn_in
    &&ImageStatus.Road_type != Ramp)
    Stop_Test();           //���籣��   ������  ����������ȴ�

  if (SystemData.CameraOK == 1)
    uncompressimage();    //ͼ���ѹ 2.5ms
  if (SystemData.OldCameraOK == 1)
    uncompressimageHD();  //�Ҷ�ͼ���ѹ

  GetDet();               //��ȡ��̬ǰհ  ���Ҽ���ͼ��ƫ�� 3us

  ImageStatus.Foresight = ((((ImageDeal[ImageStatus.OFFLine + 1].Center) +
                             (ImageDeal[ImageStatus.OFFLine + 2].Center) +
                             (ImageDeal[ImageStatus.OFFLine + 3].Center)) /3) -40);  

  ImageStatus.Det_all = (ImageStatus.Foresight + 40) - ImageDeal[54].Center;
  ImageStatus.Det_all_k =(float)(ImageStatus.Det_all) / (ImageStatus.OFFLine + 2 - 54) * 30;
  ImageStatus.Foresight = abs(ImageStatus.Foresight);

  gpio_set(CORE_LED3_PIN, ImageStatus.Road_type != Straight);//������
  gpio_set(CORE_LED2_PIN, ImageStatus.Road_type != Forkout); //������

}
