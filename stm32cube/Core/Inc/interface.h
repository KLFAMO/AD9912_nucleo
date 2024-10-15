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
    value f;
    value ftw;
    value rf;
    value rftw;
    value cur;
    sded ded;
} parameters;

pointer getPointer(pointer,char * );
void initInterface(void);
void setParam(value*, double);


#endif /* INC_INTERFACE_H_ */
