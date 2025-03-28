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

typedef struct{
	value on;
    value laston;
    value rst;
    value range;
    value fm;
    value p; // proportional factor
    value i; // integrating factor
    value d; // derivative factor
    value e; // error
    value laste;
    value ae; // accumulated dv
    value aer; // accumulated error range
    value de; // derivative error
    value der; // derivative error range
}spll;

typedef struct{
	value on;
    value laston;
    value rst;
    value range;
    value fm;
    value p; // proportional factor
    value i; // integrating factor
    value d; // derivative factor
    value e; // error
    value laste;
    value ae; // accumulated dv
    value aer; // accumulated error range
    value de; // derivative error
    value der; // derivative error range
    value fcset; // center frequency set
    value fc; // center frequency
    value emint; // minimum error treshold
    value emaxt; // maximum error treshold
}sfll;

typedef struct {
    double version;
    value save;
    value load;
    value ver;
    value f;
    value ftw;
    value rf;
    value rftw;
    value cur;
    value mode;
    value apf; // auto switch pll - fll
    value dv; // received value
    value dv_last;
    value dvs; // dv scale
    sded ded;
    spll pll;
    sfll fll;
} parameters;

pointer getPointer(pointer,char * );
void initInterface(void);
void setParam(value*, double);


#endif /* INC_INTERFACE_H_ */
