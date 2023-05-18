/*
 * buzzer.c
 *
 *  Created on: 2021Äê6ÔÂ23ÈÕ
 *      Author: ???
 */

#include "buzzer.h"
#include "C_H.h"

void buzzer_event(void) {
//  if (ImageStatus.Road_type==Straight)//||ImageStatus.Road_type==Straight)
////          )||
////      ImageStatus.Road_type == LeftCirque ||ImageStatus.Road_type == RightCirque)
////      ImageStatus.Road_type == RightCirque|| ImageStatus.Road_type==Forkin||ImageStatus.Road_type==Forkout/*|| ImageStatus.Road_type == Forkin ||
////      ImageStatus.Road_type == Forkout*/)  // ImageStatus.Road_type==Cross
//    buzzer_on();
//  else
//    buzzer_off();
//  static int cnt;
//  ++cnt;
//  if (beep_times > 3 || cnt % beep_interval)
//    return;
//
//  buzzer_freq(beep_freq[beep_times]);
//  if (beep_times > 2) {
//    if (beep_loop)
//      beep_times = 0;
//    buzzer_off();
//  } else
//    buzzer_on();
//  ++beep_times;
//
//  music();

  // if(dial0_read()){
//    if (ImageStatus.Road_type==Barn_in)
//    if (ImageStatus.straight_acc)
//      if (ImageStatus.Road_type==LeftCirque||ImageStatus.Road_type==RightCirque||ImageStatus.Road_type==Forkin||ImageStatus.Road_type==Forkout)
//    if(ImageStatus.CirqueOff == 'T')||



//    buzzer_freq(buzzer_note[ImageStatus.Road_type + 1]);


//    if(ImageStatus.Road_type==Cross_ture)
//          buzzer_on();
//        else
//          buzzer_off();
  // }

//
//  if(dial1_read()){
//      if (ImageStatus.Road_type==Cross)
//          buzzer_on();
//        else
//          buzzer_off();
//  }
//
//
//  if(dial2_read()){
      if (
//              ImageStatus.Road_type==Barn_in
//              ||ImageStatus.Road_type==Forkin
              ImageStatus.Road_type==Forkout
              ||ImageStatus.Road_type==LeftCirque
              ||ImageStatus.Road_type==RightCirque
              ||ImageStatus.Road_type==Ramp
//              Fork_dowm==1
              )
          buzzer_on();
        else
          buzzer_off();

//  }
//
//  if(dial3_read()){
//      if (ImageStatus.Road_type==LeftCirque||ImageStatus.Road_type==RightCirque)
//          buzzer_on();
//        else
//          buzzer_off();
//  }

//          if (ImageStatus.straight_acc==1)
//              buzzer_on();
//            else
//              buzzer_off();


}

