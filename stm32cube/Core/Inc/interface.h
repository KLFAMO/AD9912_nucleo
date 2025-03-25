/*
 * interface.h
 *
 *  Created on: Apr 10, 2022
 *      Author: UMK
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#define CMD_SEP ';'

typedef struct{
    void* p;
    char* type;
} pointer;

typedef struct{
    int is;
} ison;

typedef struct{
    int tabsize;
    int tabcount;
    int tabpos;
    double* ptab[2];
}mestab;

typedef struct {
    double min;
    double max;
    double val;
    char* cmdset;
    ison tabon;
    mestab mes;
} value;

typedef struct{
	value on;
	value hzps;
}sded;

typedef struct {
    double version;
    value save;
    value load;
    value ver;
    value f;
    value fm;
    value ftw;
    value rf;
    value rftw;
    value cur;
    value mode;
    value dv; // received value
    value dv_last;
    value adv; // accumulated dv
    value ddv;
    value dvs;
    value dvi; // integrating factor
    value dvp; // proportional factor
    value dvd; // derivative factor
    value dvmaxf;
    value dvminf;
    value dvrange;
    value dvrst;  // reset correction?
    sded ded;
} parameters;

pointer getPointer(pointer,char * );
void initInterface(void);
void setParam(value*, double);


#endif /* INC_INTERFACE_H_ */
