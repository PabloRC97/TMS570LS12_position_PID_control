/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
//**********************************************[BEGIN (0)]************
/* --> [Conexiones fisicas] <--
 * J11_1    +5Vcc
 * J11_2    GND
 * J11_3    +3Vcc3
 * J11_5    GIOB_2      DIR_1
 * J11_6    GIOB_3      DIR_2
 * J11_20   N2HET1_0    INT_B_MOTOR / EDGE 0
 * J11_21   N2HET1_1    INT_A_MOTOR / EDGE 1
 * J11_11   N2HET1_11   PWM_MOTOR
 *
 * J10_20   N2HET1_10   PMW_SERVO
 *
 * P - 450
 * D - 40
 */

#define PORT_DIR_1 gioPORTB
#define BIT_DIR_1 2

#define PORT_DIR_2 gioPORTB
#define BIT_DIR_2 3

#define PORT_INT_B_MOTOR hetPORT1 // hetRAM1
#define BIT_INT_B_MOTOR 0

#define PORT_INT_A_MOTOR hetPORT1 // hetRAM1
#define BIT_INT_A_MOTOR 1

#define PORT_PWM_MOTOR hetRAM1




// |---> [Librerias varias]
#include "het.h"
#include "gio.h"
#include "rti.h"

//#include "rti.h"
#include "sys_core.h"

#include <math.h>
// <---| [Librerias varias]

// |---> [Librerias para comunicacion UART]
#include "sci.h"
#include "stdio.h"
#include "stdlib.h"
// <---| [Librerias para comunicacion UART]

// |---> [Estructuras para incrementar la resolucion del periodo del PWM]
static const uint32 s_het1pwmPolarity[8U] = { 3U, 3U, 3U, 3U, 3U, 3U, 3U, 3U, };
// <---| [Estructuras para incrementar la resolucion del periodo del PWM]
//**********************************************[END (0)]**************
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
//**********************************************[BEGIN (1)]************



// |---> [Variables para comunicacion UART]
unsigned char receivedData[3];
char command[200];
// <---| [Variables para comunicacion UART]

// |---> [Prototipos de funciones para el TIMER]
void rtiNotification(uint32 notification);
// <---| [Prototipos de funciones para el TIMER]

uint32_t timer_1ms = 0;


hetSIGNAL_t pwm1_het10;
//**********************************************[END (1)]**************
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
//**********************************************[BEGIN (2)]************
// |---> [Prototipos de funciones para comunicacion UART]
bool sciEnviarDatos(uint8 numOfDat, char* charDat, bool sc);
// <---| [Prototipos de funciones para comunicacion UART]

// |---> [Prototipos de funciones para incrementar la resolucion del periodo del PWM]
void pwmSetSignal_Mio(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal);
// <---| [Prototipos de funciones para incrementar la resolucion del periodo del PWM]

//void gioNotification(gioPORT_t *port, uint32 bit);
//void hetNotification(hetBASE_t *het, uint32 offset);
void edgeNotification(hetBASE_t * hetREG,uint32 edge);

void inicializaciones_puertos(void);

hetSIGNAL_t PWM_MOTOR; // PWM_MOTOR


float Kp = 500;      // 147
float Ki = 0.3;  // 0.0145
float Kd = 20;     // 0.0625
float Tp = 0.01;    // 0.01
float referencia_deg = 0;
float e_k = 0;
float e_k_1 = 0;
int condicion = 2800;

int PWMServo = 700; // ---->>> Para el servo de la base limites: [ladoIzq = 200; Medio = 700; LadoDer = 1200]

float compGrav = 0.0;
float mlg = 0.0;



int32_t DC_k = 0;
int32_t DC_k_1 = 0;

int32_t posicion = 0;
float posicion_deg = 0;

enum {
    LOW = 0,
    HIGH,
    sin_detectar,
    horario,
    antihorario
};

#define noGirar             gioSetBit(PORT_DIR_1, BIT_DIR_1, LOW); \
    gioSetBit(PORT_DIR_2, BIT_DIR_2, LOW)
#define girarDerecha        gioSetBit(PORT_DIR_1, BIT_DIR_1, LOW); \
    gioSetBit(PORT_DIR_2, BIT_DIR_2, HIGH)
#define girarIzquierda      gioSetBit(PORT_DIR_1, BIT_DIR_1, HIGH); \
    gioSetBit(PORT_DIR_2, BIT_DIR_2, LOW)

uint8_t sentidoGiro = sin_detectar;

//**********************************************[END (2)]**************
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
//**********************************************[BEGIN (3)]************
        // Se configuran los puertos parametros iniciales.
        inicializaciones_puertos();
        // Para 10kHz 10E-6 Periodo, es decir 100.
        PWM_MOTOR.period = 20E3;        // Microsegundos del PWM
        PWM_MOTOR.duty = 0;         // Duty Cycle (0 - 10,000)

        pwm1_het10.period = 20E3;       // Microsegundos del PWM
        pwm1_het10.duty = 0;

        noGirar;

        while (1) {

            if (timer_1ms >= 10) {

                // Comienza control.
                posicion_deg = posicion * 9.0/16.0; //* 9.0/8.0; //1.125 en el ejemplo no se realiza la division entre 2

                e_k = referencia_deg - posicion_deg;

                if (e_k > 0) {
                    girarIzquierda;
                    condicion = 2000;
                } else if (e_k < 0) {
                    girarDerecha;
                    condicion = 3100;
                } else {
                    noGirar;
                }

                if (e_k < 0) {
                    e_k *= -1.0;
                }

                DC_k = e_k*(Kp + Ki*Tp + Kd/Tp) - e_k_1*Kd/Tp + Ki*DC_k_1;

                if (DC_k  > condicion) {
                    DC_k = condicion;
                }
                else if (DC_k < 0 ) {
                    DC_k = 0;
                }

                compGrav = mlg*cosf( posicion_deg*(M_PI/180.0) );

                PWM_MOTOR.duty = DC_k + compGrav;
                pwmSetSignal_Mio(PORT_PWM_MOTOR, pwm0, PWM_MOTOR);

                pwm1_het10.duty = PWMServo;
                pwmSetSignal_Mio(hetRAM1, pwm1, pwm1_het10);

                DC_k_1 = DC_k;
                e_k_1 = e_k;
                // Termina control.

                if (sentidoGiro == horario) {
    //              sciEnviarDatos( sprintf(command, "[%d] Horario, error ", (int)posicion), command, true);
                    sciEnviarDatos( sprintf(command, "%d", (int)posicion_deg), command, true);
                } else if (sentidoGiro == antihorario) {
    //              sciEnviarDatos( sprintf(command, "[%d] Anti-Horario", (int)posicion), command, true);
                    sciEnviarDatos( sprintf(command, "%d", (int)posicion_deg), command, true);
                } else {
    //              sciEnviarDatos( sprintf(command, "[%d] Sin detectar", (int)posicion), command, true);
                    sciEnviarDatos( sprintf(command, "%d", (int)posicion_deg), command, true);
                }
                timer_1ms = 0;
            }

        }
//**********************************************[END (3)]**************
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
//**********************************************[BEGIN (4)]************
void inicializaciones_puertos(void) {
    // |---> [Inicializaciones varias]
    rtiInit();
    hetInit();
    gioInit();
    // <---| [Inicializaciones varias]

    // |---> [Inicializacion del TIMER]
    edgeEnableNotification(hetREG1, 0);
    edgeEnableNotification(hetREG1, 1);
    _enable_interrupt_();
    _enable_IRQ();
    // <---| [Inicializacion del TIMER]

    // |---> [Inicializacion del TIMER]
    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);

    rtiStartCounter(rtiCOUNTER_BLOCK0);
    _enable_interrupt_();
    // <---| [Inicializacion del TIMER]



    // |---> [Inicializaciones para comunicacion UART]
    sciInit();
    sciReceive(scilinREG, 0x01, receivedData);
    // <---| [Inicializaciones para comunicacion UART]
}

// |---> [Funciones para comunicacion UART]
bool sciEnviarDatos(uint8 numOfDat, char* charDat, bool sc) {
    sciSend(scilinREG, numOfDat, (uint8 *) charDat);
    if (sc) {
        sciSend(scilinREG, 0x02, (unsigned char *) "\r\n");
    }
    return true;
}
// <---| [Funciones para comunicacion UART]

// |---> [Funciones para incrementar la resolucion del periodo del PWM]
void pwmSetSignal_Mio(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal) {
    uint32 action;
    uint32 pwmPolarity = 0U;
    float64 pwmPeriod = 0.0F;

    if (hetRAM == hetRAM1) {
        pwmPeriod = (signal.period * 1000.0F) / 800.000F;
        pwmPolarity = s_het1pwmPolarity[pwm];
    }

    if (signal.duty == 0U) {
        action = (pwmPolarity == 3U) ? 0U : 2U;
    } else if (signal.duty >= 10000U) {
        action = (pwmPolarity == 3U) ? 2U : 0U;
    } else {
        action = pwmPolarity;
    }

    hetRAM->Instruction[(pwm << 1U) + 41U].Control = ((hetRAM->Instruction[(pwm
            << 1U) + 41U].Control) & (~(uint32) (0x00000018U)))
            | (action << 3U);
    hetRAM->Instruction[(pwm << 1U) + 41U].Data = ((((uint32) pwmPeriod
            * signal.duty) / 10000U) << 7U) + 128U;
    hetRAM->Instruction[(pwm << 1U) + 42U].Data = ((uint32) pwmPeriod << 7U)
            - 128U;
}
// <---| [Funciones para incrementar la resolucion del periodo del PWM]

enum {
    cond_horario_1 = 0,
    cond_horario_2,
    cond_antiHorario_3,
    cond_antiHorario_4
};
uint8_t condiciones[4] = {false, false, false, false};

void edgeNotification(hetBASE_t * hetREG,uint32 edge) {
    uint8_t flanco_INTA = gioGetBit(PORT_INT_A_MOTOR, BIT_INT_A_MOTOR);
    uint8_t flanco_INTB = gioGetBit(PORT_INT_B_MOTOR, BIT_INT_B_MOTOR);

    // Condiciones para Horario.
    if (edge == BIT_INT_A_MOTOR && flanco_INTA == 1) {
        if (flanco_INTB == 0) {
            condiciones[cond_horario_1] = true;
        } else {
            condiciones[cond_horario_1] = false;
        }
    }
    if (edge == BIT_INT_A_MOTOR && flanco_INTA == 0) {
        if (flanco_INTB == 1) {
            condiciones[cond_horario_2] = true;
        } else {
            condiciones[cond_horario_2] = false;
        }
    }

    // Condiciones para Anti-Horario.
    if (edge == BIT_INT_A_MOTOR && flanco_INTA == 1) {
        if (flanco_INTB == 1) {
            condiciones[cond_antiHorario_3] = true;
        } else {
            condiciones[cond_antiHorario_3] = false;
        }
    }
    if (edge == BIT_INT_A_MOTOR && flanco_INTA == 0) {
        if (flanco_INTB == 0) {
            condiciones[cond_antiHorario_4] = true;
        } else {
            condiciones[cond_antiHorario_4] = false;
        }
    }

    // Evaluacion.
    if (condiciones[cond_horario_1] && condiciones[cond_horario_2]) {
        sentidoGiro = horario;
        posicion++;

    } else if (condiciones[cond_antiHorario_3] && condiciones[cond_antiHorario_4]) {
        sentidoGiro = antihorario;
        posicion--;
    }
//  else {
//      sentidoGiro = sin_detectar;
//  }
}

// |---> [Funciones para el TIMER]
void rtiNotification(uint32 notification) {
    timer_1ms++;
}
// <---| [Funciones para el TIMER]
//**********************************************[END (4)]**************
/* USER CODE END */
