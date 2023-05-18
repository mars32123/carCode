/*
 * shell.c
 *
 *  Created on: 2021Äê6ÔÂ16ÈÕ
 *      Author: hlK
 */

#include "shell.h"
#include "Motor.h"
#include "car.h"
#include "vt100.h"
#include "DataStore.h"
#include "zf_uart.h"

#pragma section all "cpu0_dsram"

IFX_ALIGN(4) IfxCpu_spinLock g_shellSpinLock;
Ifx_Shell g_shellInterface;            /* Shell interface object */
static IfxStdIf_DPipe g_ascStandardInterface; /* Standard interface object */
volatile static unsigned cpu_counter;

/* Function to show information about the example through the shell */
boolean printShellInfo(pchar args, void* data, IfxStdIf_DPipe* io) {
  IfxStdIf_DPipe_print(io, "Hello BYD OX 2021!\n\r");
  return TRUE;
}

static boolean setSteerP(pchar args, void* data, IfxStdIf_DPipe* io) {

  return TRUE;
}

static boolean storeParam(pchar args, void* data, IfxStdIf_DPipe* io) {
  data_store();
  return TRUE;
}

/* Array that stores the supported Shell commands */
static const Ifx_Shell_Command g_shellCommands[] = {
    {"info", " : show info", &g_shellInterface, &printShellInfo},
    {"steerp", " : set steer p", &g_shellInterface, &setSteerP},
    {"store", " : store parameters", &g_shellInterface, &storeParam},
    {"help", " : show help", &g_shellInterface, &Ifx_Shell_showHelp},
    {"maxvoff", " : max speed offline ", &g_shellInterface, &max_voffline_fun},
    IFX_SHELL_COMMAND_LIST_END};

void shell_init(void) {
  uart_init(DEBUG_UART, DEBUG_UART_BAUD, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
  
  /* Initialize the Standard Interface */
  IfxAsclin_Asc_stdIfDPipeInit(&g_ascStandardInterface, &uart2_handle);

  /* Initialize the Console */
  Ifx_Console_init(&g_ascStandardInterface);

  /* Initialize the shell */
  Ifx_Shell_Config shellConf;
  Ifx_Shell_initConfig(
      &shellConf); /* Initialize the structure with default values         */

  shellConf.standardIo =
      &g_ascStandardInterface; /* Set a pointer to the standard interface */
  shellConf.commandList[0] =
      &g_shellCommands[0]; /* Set the supported command list */

  Ifx_Shell_init(
      &g_shellInterface,
      &shellConf); /* Initialize the Shell with the given configuration    */
}

void shell_run(void) {
  ++cpu_counter;
  /* Process the received data */
  Ifx_Shell_process(&g_shellInterface);
}

void cpu_usage_refresh(void) {
  Car.status.cpu0_usage = cpu_counter;
  cpu_counter = 0;
}

#pragma section all restore
