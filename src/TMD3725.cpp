/* MIT License
 * Copyright (c) 2024 Vasilii Zhuravskii zhuravsky.cc
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * 
 */


#include "TMD3725.h"

int TMD3725::I2CGetreg(unsigned char addr, int reg) {
    unsigned char data = -1;
    _i2cPort.beginTransmission(addr);
    _i2cPort.write(reg);
    _i2cPort.endTransmission(false);
    _i2cPort.requestFrom(addr, 1);
    while (_i2cPort.available()) data = _i2cPort.read();
    _i2cPort.endTransmission(true);
    if (data >= 0) {
        return data;
    }
    else {
        return -1;
    }
}

int TMD3725::I2CSetreg (unsigned char addr, int reg, int value) {
    _i2cPort.beginTransmission(addr);
    _i2cPort.write(reg);
    _i2cPort.write(value);
    int error_code = _i2cPort.endTransmission();
    if (error_code != 0) {
        return -1;
    }
    return 0;
}

bool TMD3725::connected() {
    _i2cPort.beginTransmission(_address);
    uint8_t retval = _i2cPort.endTransmission();
    return !retval;
}

int TMD3725::set_atime(int reginfo[], int cycle_No) {
    if ((cycle_No <= 256) && (cycle_No >= 1)) {
        reginfo[1] = cycle_No - 1;
    }
    else 
        return -1;
    return I2CSetreg(TMD3725ADDR, ATIME_ADDR, reginfo[1]);
}

int TMD3725::get_all_data(int reginfo[]) {
    for (int i=0; i<9; i++) {
        reginfo[i] = I2CGetreg(TMD3725ADDR, ENABLE_ADDR+i);
        if (reginfo[i] == -1) return -1;
    }
    if ((reginfo[9] = I2CGetreg(TMD3725ADDR, PIHT_ADDR)) == -1) return -1;
    for (int i=0; i<4; i++) {
        if ((reginfo[10+i] = I2CGetreg(TMD3725ADDR, PERS_ADDR+i)) == -1) return -1;
    }
    for (int i=0; i<13; i++) {
        if ((reginfo[14+i] = I2CGetreg(TMD3725ADDR, CFG1_ADDR+i)) == -1) return -1;
    }
    if ((reginfo[27] = I2CGetreg(TMD3725ADDR, CFG2_ADDR)) == -1) return -1;
    if ((reginfo[28] = I2CGetreg(TMD3725ADDR, CFG3_ADDR)) == -1) return -1;
    if ((reginfo[29] = I2CGetreg(TMD3725ADDR, POFFSETL_ADDR)) == -1) return -1;
    if ((reginfo[30] = I2CGetreg(TMD3725ADDR, POFFSETH_ADDR)) == -1) return -1;
    if ((reginfo[31] = I2CGetreg(TMD3725ADDR, CALIB_ADDR)) == -1) return -1;
    if ((reginfo[32] = I2CGetreg(TMD3725ADDR, CALIBCFG_ADDR)) == -1) return -1;
    if ((reginfo[33] = I2CGetreg(TMD3725ADDR, CALIBSTAT_ADDR)) == -1) return -1;
    if ((reginfo[34] = I2CGetreg(TMD3725ADDR, INTENAB_ADDR)) == -1) return -1;
    return 0;
}

int TMD3725::set_cfg1(int reginfo[], int IRtoG_flag, int again_flag) {
    if (IRtoG_flag) { reginfo[14] |= 0x08; }
    else { reginfo[14] &= 0xF7; }
    switch (again_flag) {
        case 1:
            reginfo[14] &= 0xFC;
            break;
        case 4:
            reginfo[14] &= 0xFD;
            reginfo[14] |= 0x01;
            break;
        case 16:
            reginfo[14] &= 0xFE;
            reginfo[14] |= 0x02;
            break;
        case 64:
            reginfo[14] |= 0x03;
            break;
        default:
            reginfo[14] &= 0xFD;
            reginfo[14] |= 0x01;
    }
    return I2CSetreg(TMD3725ADDR, CFG1_ADDR, reginfo[14]);
}

int TMD3725::enable_sensor(int reginfo[], int wait_flag, int prox_flag, int als_flag) {
    if (wait_flag) { reginfo[0] |= 0x08; }
    else { reginfo[0] &= 0xF7; }
    if (prox_flag) { reginfo[0] |= 0x04; }
    else { reginfo[0] &= 0xFB; }
    if (als_flag) { reginfo[0] |= 0x03; }
    else { reginfo[0] &= 0xFC; }
    return I2CSetreg(TMD3725ADDR, ENABLE_ADDR, reginfo[0]);
}

int TMD3725::init(int reginfo[]) {
    if (set_atime(reginfo, 1) == -1) return -1;
    if (set_cfg1(reginfo, 0, x4) == -1) return -1;
    if (enable_sensor(reginfo, 0, 1, 1) == -1) return -1;
    return 0;
}

int TMD3725::get_optics_data(int color_array[]) {
    for (int i=0; i<9; i++) {
        if ((color_array[i] = I2CGetreg(TMD3725ADDR, CDATAL_ADDR+i)) == -1)
            return -1;
    }
    return 0;
}

hsv TMD3725::rgb2hsv(rgb in) {
    hsv out;
    double min, max, delta;
    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;
    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;
    out.v = max;
    delta = max - min;
    if (delta < 0.00001) {
        out.s = 0; out.h = 0;
        return out;
    }
    out.s = (max > 0.0) ? (delta / max) : 0.0;
    if (out.s == 0.0) { out.h = 0.0; return out; }
    if (in.r >= max)
        out.h = (in.g - in.b) / delta;
    else if (in.g >= max)
        out.h = 2.0 + (in.b - in.r) / delta;
    else
        out.h = 4.0 + (in.r - in.g) / delta;
    out.h *= 60.0;
    if (out.h < 0.0) out.h += 360.0;
    return out;
}

rgb TMD3725::hsv2rgb(hsv in) {
    double hh, p, q, t, ff;
    long i;
    rgb out;
    if(in.s <= 0.0) { out.r = in.v; out.g = in.v; out.b = in.v; return out; }
    hh = in.h; if(hh >= 360.0) hh = 0.0; hh /= 60.0; i = (long)hh; ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));
    switch(i) {
        case 0: out.r = in.v; out.g = t; out.b = p; break;
        case 1: out.r = q; out.g = in.v; out.b = p; break;
        case 2: out.r = p; out.g = in.v; out.b = t; break;
        case 3: out.r = p; out.g = q; out.b = in.v; break;
        case 4: out.r = t; out.g = p; out.b = in.v; break;
        case 5: default: out.r = in.v; out.g = p; out.b = q; break;
    }
    return out;
}

optics_val TMD3725::get_calib_color(const int reginfo[]) {
    int colorarray[9];
    optics_val calibed;
    if ((get_optics_data(colorarray)) == -1) return calibed;
    calibed = calib_color(colorarray, reginfo);
    return calibed;
}

optics_val TMD3725::calib_color(const int colorarray[], const int reginfo[]) {
    optics_val calibed;
    int rawr, rawg, rawb, rawc, Again;
    float Atime;
    rawr = combine_color(colorarray, R);
    rawg = combine_color(colorarray, G);
    rawb = combine_color(colorarray, B);
    rawc = combine_color(colorarray, C);
    Atime = 2.81 * (reginfo[1] + 1);
    if (reginfo[11] & 0x02) Atime *= 12;
    Again = power(2.0, (reginfo[14] & 0x03) * 2);
    if (!(reginfo[27] & 0x04)) Again /= 2;
    calibed.IR = ((rawr + rawg + rawb) - rawc)/2;
    calibed.CPL = (Again * Atime)/DGF;
    calibed.Lux = ((C_coef * rawc) + (R_coef * rawr) + (G_coef * rawg) + (B_coef * rawb))/calibed.CPL;
    calibed.red = rawr - calibed.IR;
    calibed.green = rawg - calibed.IR;
    calibed.blue = rawb - calibed.IR;
    calibed.clear = rawc - calibed.IR;
    calibed.CCT = (CT_coef * (calibed.blue/calibed.red)) + CT_offset;
    return calibed;
}

int TMD3725::combine_color(const int color_array[], int flag) {
    int tempcolor = 0;
    switch(flag) {
        case 1: tempcolor = (color_array[1] << 8) + color_array[0]; break;
        case 2: tempcolor = (color_array[3] << 8) + color_array[2]; break;
        case 3: tempcolor = (color_array[5] << 8) + color_array[4]; break;
        case 4: tempcolor = (color_array[7] << 8) + color_array[6]; break;
        default: tempcolor = -1;
    }
    return tempcolor;
}

void TMD3725::print_color(const optics_val color_data) { (void)color_data; }

float TMD3725::power(float base, int power) {
    float result = 1;
    for (int i=0; i<power; i++) result *= base;
    return result;
}

void TMD3725::print_color_json(optics_val color_data, uint32_t timestamp) {
    (void)color_data;
    (void)timestamp;
}
