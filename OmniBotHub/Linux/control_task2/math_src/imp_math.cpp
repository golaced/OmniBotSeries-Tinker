#include "include.h"
#include "eso.h"
#include "locomotion_header.h"
#include "gait_math.h"

float cal_BezierP3(float s, float a, float b, float c, float d)//三阶贝塞尔
{
    float temp = 1 - limitw(s, 0, 1);
    float o_temp = a * temp* temp * temp + 3 * b * s * temp * temp + 3 * c * s * s * temp + d * s * s * s;
    return o_temp;
}

float cal_Impluse_Fz(float alfa, float s_st, float s_peak)//Z方力前馈
{
    float fz;
    float s1, s2;

    if (s_st <= s_peak) {
        s1 = s_st / s_peak;
        fz = alfa * cal_BezierP3(s1, 0, 0.8, 1, 1);
    }
    else {
        s2 = (s_st - s_peak) / (1 - s_peak);
        fz = alfa * cal_BezierP3(s2, 1, 1, 0.8, 0);

    }
    return fz;
}

float cal_Impluse_gs(float s_st)//gs
{
    float s_peak = 0.5;
    float s1, s2;
    float gs;
    if (s_st <= s_peak) {
        s1 = s_st / s_peak;
        gs = cal_BezierP3(s1, 0, 1, 1, 1);
    }
    else {
        s2 = (s_st - s_peak) / (1 - s_peak);
        gs = cal_BezierP3(s2, 1, 1, 1, 0);

    }
    return gs;
}

float cal_Impluse_tauP(float alfa, float s_st, char sel)//俯仰力矩前馈
{
    float tauP;
    float s_peak = 0.5;
    float s1, s2;

    if (sel == Fs) {
        if (s_st <= s_peak) {
            s1 = s_st / s_peak;
            tauP = -alfa * cal_BezierP3(s1, 0, 0.8, 1, 1);
        }
        else {
            s2 = (s_st - s_peak) / (1 - s_peak);
            tauP = -alfa * cal_BezierP3(s2, 1, 1, 0.8, 0);
        }
    }
    else {
        if (s_st <= s_peak) {
            s1 = s_st / s_peak;
            tauP = alfa * cal_BezierP3(s1, 0, 0.8, 1, 1);
        }
        else {
            s2 = (s_st - s_peak) / (1 - s_peak);
            tauP = alfa * cal_BezierP3(s2, 1, 1, 0.8, 0);
        }
    }
    return tauP;
}

float cal_Impluse_sw_z(float dh, float s_st, float s_peak)//Z方力前馈
{
    float fz;
    float s1, s2;

    if (s_st <= s_peak) {
        s1 = s_st / s_peak;
        fz = dh* cal_BezierP3(s1, 0, 0.8, 1, 1);
    }
    else {
        s2 = (s_st - s_peak) / (1 - s_peak);
        fz = dh * cal_BezierP3(s2, 1, 1, 0.8, 0);

    }
    return fz;
}
