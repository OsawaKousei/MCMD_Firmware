/*
 * udpcontroller.h
 *
 *  Created on: Dec 23, 2019
 *      Author: endoKikaiken
 */

#ifndef UDPCONTROLLER_H_
#define UDPCONTROLLER_H_

#include "lwip.h"
#include "lwip/sockets.h"

// コントローラーの設定


// Client: F767zi
// Server: PC (Raspberry Pi or Linux PC)

// Settings of F767zi
#define CLIENT_IP "192.168.0.30"
#define CLIENT_PORT 7777

// param of controller
#define ANALOG_STICK_DEADZONE 0.30f //アナログスティック不感帯


struct controller_data {
    uint16_t button;
    uint16_t reserved;
    float l_x;
    float l_y;
    float r_x;
    float r_y;
};

typedef struct {
	float x;
	float y;
	float r;
	float theta;
} analog_stick_f;



//ボタンマスク用
typedef enum {
	CONTROLLER_LEFT     = 0b1000000000000000,
	CONTROLLER_DOWN     = 0b0100000000000000,
	CONTROLLER_RIGHT    = 0b0010000000000000,
	CONTROLLER_UP       = 0b0001000000000000,
	CONTROLLER_START    = 0b0000100000000000,
	CONTROLLER_SELECT   = 0b0000000100000000,
	CONTROLLER_SQUARE   = 0b0000000010000000,
	CONTROLLER_CROSS    = 0b0000000001000000,
	CONTROLLER_CIRCLE   = 0b0000000000100000,
	CONTROLLER_TRIANGLE = 0b0000000000010000,
	CONTROLLER_L1       = 0b0000000000000100,
	CONTROLLER_L2       = 0b0000000000000001,
	CONTROLLER_R1       = 0b0000000000001000,
	CONTROLLER_R2       = 0b0000000000000010,
	CONTROLLER_LSTICK   = 0b0000001000000000,
	CONTROLLER_RSTICK   = 0b0000010000000000
} ControllerButtons;

void UDPControllerReceive(void const *argument);  // UDP controller用のタスク

//prototype declaration
void UDPController_GetLeftStick(analog_stick_f*);
void UDPController_GetRightStick(analog_stick_f*);
void UDPController_Norm2Polar(analog_stick_f*,float x,float y);
uint16_t UDPController_GetControllerButtons();

#endif /* UDPCONTROLLER_H_ */
