/**
 ******************************************************************************
 * @file    version.h
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
#ifndef __VERSION_H
#define __VERSION_H

#define VER_MAIN					1
#define VER_PATCHLEVEL				0
#define VER_SUBLEVEL				3

#ifndef __stringify
#define __stringify_1(x...)			#x
#define __stringify(x...)			__stringify_1(x)
#endif

#define LIB_VERSION(a, b, c) \
	(((a) << 16) + ((b) << 8) + (c))

#define LIB_VERSION_CODE \
		LIB_VERSION(VER_MAIN, VER_PATCHLEVEL, VER_SUBLEVEL)

#define LIB_VERSION_STRING \
	__stringify(VER_MAIN.VER_PATCHLEVEL.VER_SUBLEVEL)

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif
