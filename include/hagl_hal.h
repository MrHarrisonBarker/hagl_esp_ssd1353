#ifndef _HAGL_HAL_H
#define _HAGL_HAL_H

#ifdef __cplusplus
extern "C" {

#endif

#include <stdint.h>
#include <hagl/backend.h>
#include "sdkconfig.h"

void hagl_hal_init( hagl_backend_t* backend );

#define DISPLAY_WIDTH       (CONFIG_SOLOMON_DISPLAY_WIDTH)
#define DISPLAY_HEIGHT      (CONFIG_SOLOMON_DISPLAY_HEIGHT)
#define DISPLAY_DEPTH       (CONFIG_SOLOMON_DISPLAY_DEPTH)

#ifdef __cplusplus
}
#endif
#endif /* _HAGL_HAL_H */
