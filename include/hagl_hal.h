#ifndef _HAGL_HAL_H
#define _HAGL_HAL_H

#ifdef __cplusplus
extern "C" {

#endif

#include <stdint.h>
#include <hagl/backend.h>
#include "sdkconfig.h"

void hagl_hal_init( hagl_backend_t* backend );

#ifdef __cplusplus
}
#endif
#endif /* _HAGL_HAL_H */
