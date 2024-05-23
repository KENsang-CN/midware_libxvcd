/**
 ******************************************************************************
 * @file    libxvcd.h
 * @author  *
 * @version V1.0
 * @history 2024-5-06 (V1.0), first edition
 * @date	5月. 6th, 2024
 * @brief	* Function Prototypes
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2024 K'sP</center></h2>
 ******************************************************************************
 */
#ifndef __LIBXVCD_H
#define __LIBXVCD_H

#include <stdint.h>
#include <stdbool.h>

#if defined(XVCD_DLL) && (defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) || defined(__WIN64__))
#	if defined(COMPILING_XVCD)
#		define LIBAPI __declspec(dllexport)
#	else
#		define LIBAPI __declspec(dllimport)
#	endif
#else
#	define LIBAPI
#endif

#ifdef __cplusplus
extern "C" {
#endif

LIBAPI int version(char *ver);
LIBAPI int ftdi_device_scan(void);
LIBAPI int ftdi_device_info(int index, char *chipname, uint32_t *id,
		char *serialnum, char *desc);

LIBAPI void* xvcd_start(int index, int port, int max_vector_len, double freq);
LIBAPI void xvcd_stop(void *handle);
LIBAPI bool xvcd_connect_info(void *handle, char *ip, int *port, double *freq);

#ifdef __cplusplus
}
#endif

#endif
