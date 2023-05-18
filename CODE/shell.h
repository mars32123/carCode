/*
 * shell.h
 *
 *  Created on: 2021Äê6ÔÂ16ÈÕ
 *      Author: hlK
 */

#ifndef CODE_SHELL_H_
#define CODE_SHELL_H_

#include "Comm/Ifx_Console.h"
#include "Comm/Ifx_Shell.h"
#include "IfxCpu.h"

#define PRINTF(...)                                         \
  {                                                         \
    IfxCpu_setSpinLock(&g_shellSpinLock, -1);               \
    IfxStdIf_DPipe_print(g_shellInterface.io, __VA_ARGS__); \
    IfxCpu_resetSpinLock(&g_shellSpinLock);                 \
  }

extern IfxCpu_spinLock g_shellSpinLock;
extern Ifx_Shell g_shellInterface;

void shell_init(void);  //!< @reference isr.c
void shell_run(void);
void cpu_usage_refresh(void);

#endif /* CODE_SHELL_H_ */
