/*
 * udpcontroller.c
 *
 *  Created on: Dec 23, 2019
 *      Author: endoKikaiken
 */

#include <math.h>
#include <UDPController.h>
#include <string.h>

//refer this global variable to obtain controller data
//do NOT overwrite this variable
static struct controller_data controller_raw = { };
struct timeval tv;

//コントローラーの値を表示するための関数
void printControllerValue(struct controller_data *d){
    printf("(lx, ly, rx, ry) : (%d, %d, %d, %d)\r\n", (int)(d->l_x * 256), (int)(d->l_y * 256), (int)(d->r_x * 256), (int)(d->r_y * 256));
    printf("button: ");
    for(int i=0; i<16; i++){
        if((d->button >> i) & 1) printf("1");
        else printf("0");
    }
    printf("\n\r");

}

void UDPControllerReceive(void const *argument) {

    printf("This is UdpControllerReceive\r\n"); //これがないとなぜか動かない
    fd_set rset;

    int sock;
    char buffer[256];
    struct sockaddr_in server_addr, client_addr;

    sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    memset((char*) &server_addr, 0, sizeof(server_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_len = sizeof(server_addr);
   server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
//	server_addr.sin_addr.s_addr = inet_addr(CLIENT_IP);
    server_addr.sin_port = htons(CLIENT_PORT);

    if (fcntl(sock, F_SETFL, fcntl(sock, F_GETFL,0) | O_NONBLOCK) < 0) {
        // handle error
    }

    FD_ZERO(&rset);

    int err = lwip_bind(sock, (struct sockaddr*) &server_addr, sizeof(server_addr));
    if (err != 0) {
        printf("UDPController:ERROR \r\n");
    } else {
        printf("UDPController:Socket Opened!\r\n");
    }
    int timeout = 0;
    struct controller_data controller_null;
    bzero(&controller_null, sizeof(struct controller_data));
    controller_null.l_x = 0.0f;
    controller_null.l_y = 0.0f;
    controller_null.r_x = 0.0f;
    controller_null.r_y = 0.0f;
    //FD_SET(sock, &rset);

    int maxfdp1 = sock + 1;

    while (1) {
        FD_SET(sock, &rset);
        tv.tv_usec = 1000;
        select(maxfdp1, &rset, NULL, NULL, &tv);
        if (timeout < 100) {
            timeout++;
        } else {
            memcpy(&controller_raw, &controller_null,
                   sizeof(struct controller_data));
        }

        if (FD_ISSET(sock, &rset)) {
            socklen_t n;
            socklen_t len = sizeof(client_addr);
            n = lwip_recvfrom(sock, (char*) buffer, 256, (int) NULL,
                              (struct sockaddr*) &client_addr, &len);
            if (n > 0) {
                timeout = 0;
                if (n < sizeof(struct controller_data)) {
                    printf("invalid data : \r\n");
                    continue;
                }

                struct controller_data *d = (struct controller_data*) &buffer;

                //コントローラーの値を表示
                //printControllerValue(d);


                memcpy(&controller_raw, d, sizeof(struct controller_data));
            }
        }
        osDelay(100);
    }

}


uint16_t UDPController_GetControllerButtons() {
    return controller_raw.button;
}

void UDPController_GetLeftStick(analog_stick_f *left_stick) {
    UDPController_Norm2Polar(left_stick, controller_raw.l_x, controller_raw.l_y);
}

void UDPController_GetRightStick(analog_stick_f *right_stick) {
    UDPController_Norm2Polar(right_stick, controller_raw.r_x, controller_raw.r_y);
}


void UDPController_Norm2Polar(analog_stick_f *stick, float x, float y) {
    stick->r = fminf(hypotf(x, y), 1.0f);  // 非負の値を返す
    stick->x = x;
    stick->y = y;
    //deadzone
    if (stick->r < ANALOG_STICK_DEADZONE) {
        stick->r = 0;
        stick->theta = 0;
        stick->x = stick->y = 0;
    }
    if (stick->r != 0) {
//      stick->theta = atan2f(stick->x, stick->y);
//		stick->theta = Modify_XYCoordinate(stick);  // 座標変換
        stick->theta = atan2f(stick->y, stick->x);  // [-pi, pi]で出力される
        if((stick->theta) < 0.0f)stick->theta = stick->theta + 2.0f*(float)M_PI; // 角度の範囲を[0, 2pi]に修正
    }
}

