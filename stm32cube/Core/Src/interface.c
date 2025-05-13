/*
 * interface.c
 *
 *  Created on: Apr 10, 2022
 *      Author: UMK, Piotr Morzynski
 */

#include "interface.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

int rmwhite(char *str);
double atofmy(char *str);
int cmd_string_interpret(char *sin, char *sout);
int cmd_interpret(char *sin, char *ssend);

int ftostr(char *str, double val);

parameters par;

pointer getPointer(pointer p, char *s)
{
  pointer pout = {0, ""};
  if (strcmp(p.type, "parameters") == 0)
  {
    parameters *ptmp = (parameters *)p.p;
    if (strcasecmp(s, "F") == 0)
	    pout = (pointer){.p = (void *)&(ptmp->f), .type = "value"};
    if (strcasecmp(s, "FOUT") == 0)
      pout = (pointer){.p = (void *)&(ptmp->fout), .type = "value"};
    if (strcasecmp(s, "FTW") == 0)
	    pout = (pointer){.p = (void *)&(ptmp->ftw), .type = "value"};
    if (strcasecmp(s, "RF") == 0)
	    pout = (pointer){.p = (void *)&(ptmp->rf), .type = "value"};
    if (strcasecmp(s, "RFTW") == 0)
	    pout = (pointer){.p = (void *)&(ptmp->rftw), .type = "value"};
    if (strcasecmp(s, "CUR") == 0)
	    pout = (pointer){.p = (void *)&(ptmp->cur), .type = "value"};
    if (strcasecmp(s, "DED") == 0)
	    pout = (pointer){.p = (void *)&(ptmp->ded), .type = "ded"};
    if (strcasecmp(s, "PLL") == 0)
	    pout = (pointer){.p = (void *)&(ptmp->pll), .type = "pll"};
    if (strcasecmp(s, "FLL") == 0)
	    pout = (pointer){.p = (void *)&(ptmp->fll), .type = "fll"};
    if (strcasecmp(s, "MODE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->mode), .type = "value"};
    if (strcasecmp(s, "APF") == 0)
      pout = (pointer){.p = (void *)&(ptmp->apf), .type = "value"};
    if (strcasecmp(s, "DV") == 0)
      pout = (pointer){.p = (void *)&(ptmp->dv), .type = "value"};
    if (strcasecmp(s, "DVS") == 0)
	    pout = (pointer){.p = (void *)&(ptmp->dvs), .type = "value"};
    if (strcasecmp(s, "SAVE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->save), .type = "value"};
    if (strcasecmp(s, "LOAD") == 0)
      pout = (pointer){.p = (void *)&(ptmp->load), .type = "value"};
    if (strcasecmp(s, "VER") == 0)
      pout = (pointer){.p = (void *)&(ptmp->ver), .type = "value"};
  }

  if (strcmp(p.type, "ded") == 0)
  {
    sded *ptmp = (sded *)p.p;
    if (strcasecmp(s, "ON") == 0)
      pout = (pointer){.p = (void *)&(ptmp->on), .type = "value"};
    if (strcasecmp(s, "HZPS") == 0)
      pout = (pointer){.p = (void *)&(ptmp->hzps), .type = "value"};
  }

  if (strcmp(p.type, "pll") == 0)
  {
    spll *ptmp = (spll *)p.p;
    if (strcasecmp(s, "ON") == 0)
      pout = (pointer){.p = (void *)&(ptmp->on), .type = "value"};
    if (strcasecmp(s, "LASTON") == 0)
      pout = (pointer){.p = (void *)&(ptmp->laston), .type = "value"};
    if (strcasecmp(s, "RST") == 0)
      pout = (pointer){.p = (void *)&(ptmp->rst), .type = "value"};
    if (strcasecmp(s, "RANGE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->range), .type = "value"};
    if (strcasecmp(s, "FM") == 0)
      pout = (pointer){.p = (void *)&(ptmp->fm), .type = "value"};
    if (strcasecmp(s, "P") == 0)
      pout = (pointer){.p = (void *)&(ptmp->p), .type = "value"};
    if (strcasecmp(s, "I") == 0)
      pout = (pointer){.p = (void *)&(ptmp->i), .type = "value"};
    if (strcasecmp(s, "D") == 0)
      pout = (pointer){.p = (void *)&(ptmp->d), .type = "value"};
    if (strcasecmp(s, "E") == 0)
      pout = (pointer){.p = (void *)&(ptmp->e), .type = "value"};
    if (strcasecmp(s, "LASTE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->laste), .type = "value"};
    if (strcasecmp(s, "AE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->ae), .type = "value"};
    if (strcasecmp(s, "AER") == 0)
      pout = (pointer){.p = (void *)&(ptmp->aer), .type = "value"};
    if (strcasecmp(s, "DE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->de), .type = "value"};
    if (strcasecmp(s, "DER") == 0)
      pout = (pointer){.p = (void *)&(ptmp->der), .type = "value"};
  }

  if (strcmp(p.type, "fll") == 0)
  {
    sfll *ptmp = (sfll *)p.p;
    if (strcasecmp(s, "ON") == 0)
      pout = (pointer){.p = (void *)&(ptmp->on), .type = "value"};
    if (strcasecmp(s, "LASTON") == 0)
      pout = (pointer){.p = (void *)&(ptmp->laston), .type = "value"};
    if (strcasecmp(s, "RST") == 0)
      pout = (pointer){.p = (void *)&(ptmp->rst), .type = "value"};
    if (strcasecmp(s, "RANGE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->range), .type = "value"};
    if (strcasecmp(s, "FM") == 0)
      pout = (pointer){.p = (void *)&(ptmp->fm), .type = "value"};
    if (strcasecmp(s, "P") == 0)
      pout = (pointer){.p = (void *)&(ptmp->p), .type = "value"};
    if (strcasecmp(s, "I") == 0)
      pout = (pointer){.p = (void *)&(ptmp->i), .type = "value"};
    if (strcasecmp(s, "D") == 0)
      pout = (pointer){.p = (void *)&(ptmp->d), .type = "value"};
    if (strcasecmp(s, "E") == 0)
      pout = (pointer){.p = (void *)&(ptmp->e), .type = "value"};
    if (strcasecmp(s, "LASTE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->laste), .type = "value"};
    if (strcasecmp(s, "AE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->ae), .type = "value"};
    if (strcasecmp(s, "AER") == 0)
      pout = (pointer){.p = (void *)&(ptmp->aer), .type = "value"};
    if (strcasecmp(s, "DE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->de), .type = "value"};
    if (strcasecmp(s, "DER") == 0)
      pout = (pointer){.p = (void *)&(ptmp->der), .type = "value"};
    if (strcasecmp(s, "FCSET") == 0)
      pout = (pointer){.p = (void *)&(ptmp->fcset), .type = "value"};
    if (strcasecmp(s, "FC") == 0)
      pout = (pointer){.p = (void *)&(ptmp->fc), .type = "value"};
    if (strcasecmp(s, "EMINT") == 0)
      pout = (pointer){.p = (void *)&(ptmp->emint), .type = "value"};
    if (strcasecmp(s, "EMAXT") == 0)
      pout = (pointer){.p = (void *)&(ptmp->emaxt), .type = "value"};
  }

  if (strcmp(p.type, "value") == 0)
  {
    value *ptmp = (value *)p.p;
    if (strcasecmp(s, "VAL") == 0)
      pout = (pointer){.p = (void *)&(ptmp->val), .type = "double"};
    if (strcasecmp(s, "MIN") == 0)
      pout = (pointer){.p = (void *)&(ptmp->min), .type = "double"};
    if (strcasecmp(s, "MAX") == 0)
      pout = (pointer){.p = (void *)&(ptmp->max), .type = "double"};
    if (strcasecmp(s, "TABON") == 0)
      pout = (pointer){.p = (void *)&(ptmp->tabon), .type = "ison"};
    if (strcasecmp(s, "MES") == 0)
      pout = (pointer){.p = (void *)&(ptmp->mes), .type = "mestab"};
  }

  if (strcmp(p.type, "mestab") == 0)
  {
    mestab *ptmp = (mestab *)p.p;
    if (strcasecmp(s, "SIZE") == 0)
      pout = (pointer){.p = (void *)&(ptmp->tabsize), .type = "int"};
    if (strcasecmp(s, "COUNT") == 0)
      pout = (pointer){.p = (void *)&(ptmp->tabcount), .type = "int"};
    if (strcasecmp(s, "POS") == 0)
      pout = (pointer){.p = (void *)&(ptmp->tabpos), .type = "int"};
  }
  if (strcmp(p.type, "ison") == 0)
  {
    ison *ptmp = (ison *)p.p;
    if (strcasecmp(s, "IS") == 0)
      pout = (pointer){.p = (void *)&(ptmp->is), .type = "int"};
  }
  return pout;
}

void setParam(value *p, double val)
{
  if (val > p->max)
    val = p->max;
  if (val < p->min)
    val = p->min;
  p->val = val;

  if (p->tabon.is == 1 && p->mes.ptab[0] != 0 && p->mes.ptab[0] != 0)
  {
    ((p->mes.ptab)[p->mes.tabcount % 2])[p->mes.tabpos] = val;
    p->mes.tabpos++;
    if (p->mes.tabpos > p->mes.tabsize - 1)
    {
      p->mes.tabpos = 0;
      p->mes.tabcount++;
    }
  }
}

void initInterface(void)
{
  par.version = 3; // version of parameters structure, increment if structure changes
  par.ver = (value){.val = 1, .min = 0, .max = 100};
  par.f = (value){.val = 0, .min = 0, .max = 400};
  par.fout = (value){.val = 0, .min = 0, .max = 400};
  par.ftw = (value){.val = 0, .min = 0, .max = 0xFFFFFFFFFFFFFF};
  par.rf = (value){.val = 0, .min = 0, .max = 400};
  par.rftw = (value){.val = 0, .min = 0, .max = 0xFFFFFFFFFFFFFF};
  par.cur = (value){.val = 31.7, .min = 8.6, .max = 31.7};
  par.mode = (value){.val = 1, .min = 0, .max = 1};
  par.apf = (value){.val = 0, .min = 0, .max = 1};
  par.dv = (value){.val = 0, .min = -100000, .max = 100000};
  par.dv_last = (value){.val = 0, .min = -10, .max = 10};
  par.dvs = (value){.val = 2e-6, .min = 0, .max = 1};

  par.ded.on = (value){.val = 0, .min = 0, .max = 1};
  par.ded.hzps = (value){.val = 0.0, .min = -10, .max = 10};

  par.pll.on = (value){.val = 0, .min = 0, .max = 1};
  par.pll.laston = (value){.val = 0, .min = 0, .max = 1};
  par.pll.rst = (value){.val = 0, .min = 0, .max = 1};
  par.pll.range = (value){.val = 0.01, .min = 0, .max = 1};
  par.pll.fm = (value){.val = 0, .min = -1, .max = 1};
  par.pll.p = (value){.val = 0, .min = -1000, .max = 1000};
  par.pll.i = (value){.val = 0, .min = -1000, .max = 1000};
  par.pll.d = (value){.val = 0, .min = -1000, .max = 1000};
  par.pll.e = (value){.val = 0, .min = -10, .max = 10};
  par.pll.laste = (value){.val = 0, .min = -10, .max = 10};
  par.pll.ae = (value){.val = 0, .min = -1000000000, .max = 1000000000};
  par.pll.aer = (value){.val = 0, .min = 0, .max = 10000000};
  par.pll.de = (value){.val = 0, .min = -20, .max = 20};
  par.pll.der = (value){.val = 0, .min = 0, .max = 1};

  par.fll.on = (value){.val = 0, .min = 0, .max = 1};
  par.fll.laston = (value){.val = 0, .min = 0, .max = 1};
  par.fll.rst = (value){.val = 0, .min = 0, .max = 1};
  par.fll.range = (value){.val = 0.1, .min = 0, .max = 1};
  par.fll.fm = (value){.val = 0, .min = -1, .max = 1};
  par.fll.p = (value){.val = 0, .min = -1000, .max = 1000};
  par.fll.i = (value){.val = 0, .min = -1000, .max = 1000};
  par.fll.d = (value){.val = 0, .min = -1000, .max = 1000};
  par.fll.e = (value){.val = 0, .min = -10, .max = 10};
  par.fll.laste = (value){.val = 0, .min = -10, .max = 10};
  par.fll.ae = (value){.val = 0, .min = -1000000000, .max = 1000000000};
  par.fll.aer = (value){.val = 0, .min = 0, .max = 10000000};
  par.fll.de = (value){.val = 0, .min = -20, .max = 20};
  par.fll.der = (value){.val = 0, .min = 0, .max = 1};
  par.fll.fcset = (value){.val = 44.5875, .min = 44.3, .max = 45}; // MHz
  par.fll.fc = (value){.val = 0, .min = -0.1, .max = 0.1}; // MHz
  par.fll.emint = (value){.val = 1000, .min = 0, .max = 1e9}; // Hz
  par.fll.emaxt = (value){.val = 1e6, .min = 0, .max = 1e9}; // Hz

  par.save = (value){.val = 0, .min = 0, .max = 1};
  par.load = (value){.val = 0, .min = 0, .max = 1};
}

/*------------------------*/
/*-----------------------------------------------------------*/



/*********************************************************/
/* separate commands  */
int cmd_string_interpret(char *sin, char *sout)
{
  char scmd[100];
  char ssend[2300];
  int i = 0, iscmd = 0, sinlen = 0;
  strcpy(ssend, "");
  sinlen = strlen(sin);
  sin[sinlen] = ';';
  for (i = 0; i <= sinlen; i++)
  {
    if (sin[i] == ';')
    {
      scmd[iscmd] = 0;
      iscmd = 0;

      cmd_interpret(scmd, ssend);
    }
    else
    {
      scmd[iscmd] = sin[i];
      iscmd++;
    }
  }
  strcpy(sout, ssend);
  return 0;
}

/*********************************************************/
/* interpret single command  */
int cmd_interpret(char *sin, char *ssend)
{
  int slen, isarg = 0, isin = 0;
  char scom[100] = {0};
  char sarg[100] = {0};
  char stmp[100] = {0};

  rmwhite(sin);
/*   strcat(ssend,"\n");
    strcat(ssend,sin);
    strcat(ssend,"\n");
*/
  slen = strlen(sin);

  while (isin < slen && sin[isin] != ' ')
  {
    scom[isin] = sin[isin];
    isin++;
  }
  scom[isin] = 0; /*tu mamy komende*/
  isin++;
  while (isin < slen && sin[isin] != ' ')
  {
    sarg[isarg] = sin[isin];
    isin++;
    isarg++;
  }
  sarg[isarg] = 0; /*Tu mamy argument*/

/*     strcat(ssend,"\ncom: ");
    strcat(ssend,scom);
    strcat(ssend,"  arg: ");
    strcat(ssend,sarg);			*/

  /*if command "REG" sent then just execute it*/
  /*if (strcmp(scom, "REG") == 0)
  {
    dds_write_bin_simp(sarg);
    return 0;
  }
*/
  /*get pointer to arg===============*/
  int iscom = -1;
  int istmp = 0;
  pointer parg = {.p = (void *)&par, .type = "parameters"};
  pointer pargback = parg;

  while (scom[iscom] != 0 || iscom==-1)
  {
    iscom++;
    istmp = 0;
    while (scom[iscom] != ':' && scom[iscom] != 0)
    {
      stmp[istmp] = scom[iscom];
      istmp++;
      iscom++;
    }
    stmp[istmp] = 0;

    pargback = parg;
    parg = getPointer(parg, stmp);
   // strcat(ssend,"\n:");
   // strcat(ssend,stmp);
  }
  /*================================*/

  if (sarg[0] == '?')
  {
//	  strcat(ssend, "*?*\n");
    if (strcmp(parg.type, "double") == 0){
    	//strcat(ssend, "*double*\n");
    	ftostr(stmp, *((double *)(parg.p)));
    }
    if (strcmp(parg.type, "value") == 0){
    	//strcat(ssend, "*value*\n");
    	ftostr(stmp, ((value *)(parg.p))->val);
    }
    if (strcmp(parg.type, "int") == 0)
      ftostr(stmp, (double)(*((int *)(parg.p))));
    if (strcmp(parg.type, "mestab") == 0)
    {
      mestab *m;
      m = (mestab *)(parg.p);
      if (m->ptab[0] != 0 && m->ptab[1] != 0)
      {
        int i = 0;
        for (i = 0; i < m->tabsize; i++)
        {
          ftostr(stmp, m->ptab[((m->tabcount + 1) % 2)][i]);
          strcat(ssend, stmp);
          strcat(ssend, ";");
        }
      }
    }
    else
    {
    	//strcat(ssend, "*else*\n");
      strcat(ssend, stmp);
    }
    strcat(ssend, "\n");
  }
  else
  {
    if (strcmp(parg.type, "double") == 0)
      *((double *)(parg.p)) = atofmy(sarg);
    if (strcmp(parg.type, "value") == 0)
      setParam((value *)parg.p, atofmy(sarg));
    if (strcmp(parg.type, "int") == 0)
      *((int *)(parg.p)) = (int)atofmy(sarg);
    if (strcmp(parg.type, "ison") == 0)
    {
      mestab *m;
      m = &(((value *)(pargback.p))->mes);
      if (atofmy(sarg) > 0)
      {
        ((ison *)(parg.p))->is = 1;
        if (m->tabsize == 0)
          m->tabsize = 10;
        m->ptab[0] = (double *)calloc(m->tabsize, sizeof(double));
        m->ptab[1] = (double *)calloc(m->tabsize, sizeof(double));
        m->tabpos = 0;
        m->tabcount = 0;
      }
      else
      {
        ((ison *)(parg.p))->is = 0;
        if (m->ptab[0])
          free(m->ptab[0]);
        if (m->ptab[1])
          free(m->ptab[1]);
        m->ptab[0] = 0;
        m->ptab[1] = 0;
      }
    }
  }

  return 0;
}

int rmwhite(char *str)
{
  int lstr = 0;
  int istr = 0;
  int isout = 0;
  int spacestate = 0;

  lstr = strlen(str);
  if (lstr > 100)
    lstr = 100;
  //omijamy spacje z poczatku
  while (str[istr] == ' ' && istr < lstr)
    istr++;
  //teraz reszta spacji
  while (istr < lstr)
  {
    if (str[istr] == ' ' || str[istr] == '\n')
    {
      if (spacestate == 0)
      {
        spacestate = 1;
        str[isout] = str[istr];
        isout++;
        istr++;
      }
      else
        istr++;
    }
    else
    {
      spacestate = 0;
      str[isout] = str[istr];
      isout++;
      istr++;
    }
  }
  str[isout] = 0;
  return 0;
}

//double atofmy(char *str)
//{
//  double out;
//  int isdot = 0, i, len, dotpos = 0, inttemp;
//  len = strlen(str);
//  if (str[len - 1] == '\n')
//    len = len - 1;
//  for (i = 0; i < len; i++)
//  {
//    if (str[i] == '.')
//    {
//      if (isdot == 1)
//        return 0;
//      isdot = 1;
//      dotpos = i;
//    }
//    if (isdot == 1)
//    {
//      str[i] = str[i + 1];
//    }
//  }
//  inttemp = atoi(str);
//  out = (double)inttemp;
//  if (isdot)
//    out = out * pow(10, -1 * (len - dotpos - 1));
//  return out;
//}


double atofmy(char *str) {
    double result = 0.0;  // result
    double fraction_part = 0.0;
    int sign = 1;
    int i = 0;
    int is_fraction = 0;
    double divisor = 10.0;

    // cut on the first white space
    while (str[i] == ' ' || str[i] == '\t') {
        i++;
    }

    // check sign
    if (str[i] == '-') {
        sign = -1;
        i++;
    } else if (str[i] == '+') {
        i++;
    }

    // integer part
    while (str[i] != '\0') {
        if (str[i] == '.') {
            // start fractional part
            is_fraction = 1;
            i++;
            continue;
        }

        // Sprawdzanie, czy mamy cyfrďż˝
        if (str[i] >= '0' && str[i] <= '9') {
            if (!is_fraction) {
                result = result * 10 + (str[i] - '0');
            } else {
                fraction_part += (str[i] - '0') / divisor;
                divisor *= 10.0;
            }
        } else {
            break;
        }
        i++;
    }
    result += fraction_part;
    result *= sign;
    return result;
}

int ftostr(char *str, double val)
{
  int istr = 0, i, j, isdot = 0;
  double factor;
  int order;
  /*Jesli wartosc jest 0 to wpisz "0" i wyjdz z funkcji*/
  if (val == 0)
  {
    strcpy(str, "0");
    return 0;
  }
  /*sprawdzamy znak*/
  if (val < 0)
  {
    str[istr] = '-';
    istr++;
    val = -1 * val;
  }
  factor = 10000000000000000;
  while (factor > 0.000000000001 && factor > val)
    factor = factor / 10;
  order = (int)log10(factor);
  if (order < 0)
  {
    order = -1 * order;
    str[istr] = '0';
    istr++;
    str[istr] = '.';
    istr++;
    for (i = 1; i < order; i++)
    {
      str[istr] = '0';
      istr++;
    }
  }
  for (j = 0; j < 15; j++)
  {
    for (i = 9; i >= 0; i--)
    {
      if ((val - i * factor) >= 0)
      {
        str[istr] = '0' + i;
        istr++;
        break;
      }
    }
    val = val - i * factor;
    factor = factor / 10;
    if (factor < 0.5 && factor > 0.05)
    {
      str[istr] = '.';
      istr++;
      isdot = 1;
    }
    if (val == 0 && isdot)
    {
      if (str[istr - 1] == '.')
        istr--;
      str[istr] = 0;
      break;
    }
  }
  str[istr] = 0;
  return 0;
}
