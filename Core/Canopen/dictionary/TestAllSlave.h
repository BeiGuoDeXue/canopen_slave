
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef TESTALLSLAVE_H
#define TESTALLSLAVE_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 TestAllSlave_valueRangeTest (UNS8 typeValue, void * value);
const indextable * TestAllSlave_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode);

/* Master node data struct */
extern CO_Data TestAllSlave_Data;
extern UNS8 data[4];		/* Mapped at index 0x2000, subindex 0x01 - 0x04 */
extern UNS8 sdo_data;		/* Mapped at index 0x2001, subindex 0x00*/

#endif // TESTALLSLAVE_H
