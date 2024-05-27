/**
 ******************************************************************************
 * @file    libxvcd.h
 * @author  KEN
 * @version V1.0.1
 * @history 2024-05-06 (V1.0.0), first edition
 *          2024-05-27 (V1.0.1), add api function comment
 * @date	May. 27th, 2024
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

/******************************************************************************
 * @brief get libxvcd version string
 *
 * @param ver: version string buffer
 *
 * @retval 0 for success, else for error number
 *****************************************************************************/
LIBAPI int version(char *ver);

/******************************************************************************
 * @brief scan jtag devices in chain
 *
 * @param jtag: jtag handler, which get from <jtag_open>
 *
 * @retval >0 for number of device in chain, else for error number
 *****************************************************************************/
LIBAPI int ftdi_device_scan(void);

/******************************************************************************
 * @brief get jtag device bsdl information
 *
 * @param jtag: jtag handler, which get from <jtag_open>
 * @param index: jtag chain index, when < 0, index equal to current selected
 *                   device index
 * @param part: device part string buffer, can be NULL
 * @param vendor: device vendor string buffer, can be NULL
 * @param family: device family string buffer, can be NULL
 * @param ir_length: device ir length buffer, can be NULL
 * @param id_code: device ir code buffer, can be NULL
 *
 * @retval 0 for success, else for error number
 *****************************************************************************/
LIBAPI int ftdi_device_info(int index, char *chipname, uint32_t *id,
		char *serialnum, char *desc);

/******************************************************************************
 * @brief start xvc daemon thread
 *
 * @param index: jtag device index, which can be selected from
 *                   <jtag_ftdi_device_scan> and <jtag_ftdi_device_info>
 *                   function
 * @param port: xvc server ip port, must be >0
 * @param max_vector_len: xvc shift vector maximum byte length, must be >0
 * @param freq: forced tck frequency value, count in Hz, auto config by xvc
 *                  protocol (settck) if <= 0
 *
 * @retval NULL for failed, else for xvc daemon handle
 *****************************************************************************/
LIBAPI void* xvcd_start(int index, int port, int max_vector_len, double freq);

/******************************************************************************
 * @brief stop xvc daemon thread
 *
 * @param handle: xvc daemon handle, which get from <xvcd_start>
 *****************************************************************************/
LIBAPI void xvcd_stop(void *handle);

/******************************************************************************
 * @brief get xvc daemon connected client info
 *
 * @param handle: xvc daemon handle, which get from <xvcd_start>
 * @param ip: client ip address buffer, can be NULL
 * @param port: client ip port buffer, can be NULL
 * @param freq: actual tck frequency value buffer, count in Hz, can be NULL
 *
 * @retval true for has connected client, else for none
 *****************************************************************************/
LIBAPI bool xvcd_connect_info(void *handle, char *ip, int *port, double *freq);

#ifdef __cplusplus
}
#endif

#endif
