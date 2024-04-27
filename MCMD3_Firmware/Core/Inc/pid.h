/*
 * pid.h
 *
 *  Created on: 2024/04/26
 *      Author: tetud
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{
    // gains
    float kp;
    float ki;
    float kd;
    float td;

    float kff; // FF制御のゲイン
    float ctrl_period; //制御周期

    // values
    float integral;//積分
    float prev_error;//前回の誤差
    //不完全微分用変数
    float prev_diff;//前回の微分
    float diff;//微分
} PID_StructTypedef;

typedef struct
{
   //accel limitは従来の実装から出力値(ctrl_val)を制限する実装に変える
   bool if_accel_limit;
   float max_accel;

   //従来の形の誤差を制限するリミッター
   bool if_error_limit;
   float max_error;

   bool if_max_duty;
   float max_duty;
   //0<abs(ctrl_val)<min_dutyならctrl_val=min_dutyにする。ストール防止？
   bool if_min_duty;
   float min_duty;
   bool if_max_integral;
   float max_integral;
   bool integral_reset_frag; //外部からこれを変更することでintegralを初期化できる

   bool if_torelance; //許容誤差を設定するか
   Torelance_type torelance_type; //許容誤差適用時の挙動
   //(STOP:ctrl_val = 0 KEEP:ctrl_val = prev_val)
   float torelance_range; //許容誤差範囲
   bool torelance_frag;// 許容誤差が適用されているかのフラグ

   bool vel_zero_assist; //速度指令０の時はctrl_valを０にするか
   bool use_rerative_ctrl; // ctrl_val = prev_val + PID_val

   bool Scurve_accelaration; // S字加減速を行うか

   float prev_val;
} PID_CtrlTypedef;

float pure_PID(float error, PID_StructTypedef *param);
float PID_ctrl(float target, float current_val, PID_StructTypedef *param, PID_CtrlTypedef *ctrl_param);

#endif /* INC_PID_H_ */
