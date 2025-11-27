#ifndef BME_H_
#define BME_H_

#include "std_types.h"

void  bme_config(void);
uint32 bme_read_oC(void);    /* °C * 100 */
uint32 bme_read_hPa(void);  /* hPa */

#endif /* BME_H_ */


