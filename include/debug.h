/**
 ******************************************************************************
 * @file    debug.h
 * @author  KEN
 * @version V1.0.0
 * @history 2021-02-27 (V1.0.0), first edition
 * @date	Feb. 27th, 2021
 * @brief	Debug Function Prototypes
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2021 K'sP</center></h2>
 ******************************************************************************
 */
#ifndef __DEBUG_H
#define __DEBUG_H

#define LIBDBG_SOH						"\001"

#define LIBDBG_ERR						LIBDBG_SOH "3"
#define LIBDBG_WARNING					LIBDBG_SOH "4"
#define LIBDBG_NOTICE					LIBDBG_SOH "5"
#define LIBDBG_INFO						LIBDBG_SOH "6"
#define LIBDBG_DEBUG					LIBDBG_SOH "7"

#define TRACELOG_ERR					3
#define TRACELOG_WARNING				4
#define TRACELOG_NOTICE					5
#define TRACELOG_INFO					6
#define TRACELOG_DEBUG					7

#define libdbg_err(fmt, args...) 		libdbg_tracelog((char*)(LIBDBG_ERR "(%s - <%s>, L.%d) " fmt), __FILE__, __func__, __LINE__, ##args)
#define libdbg_warn(fmt, args...) 		libdbg_tracelog((char*)(LIBDBG_WARNING "(%s - <%s>, L.%d) " fmt), __FILE__, __func__, __LINE__, ##args)
#define libdbg_notice(fmt, args...) 	libdbg_tracelog((char*)(LIBDBG_NOTICE fmt), ##args)
#define libdbg_info(fmt, args...) 		libdbg_tracelog((char*)(LIBDBG_INFO fmt), ##args)
#define libdbg_dbg(fmt, args...) 		libdbg_tracelog((char*)(LIBDBG_DEBUG fmt), ##args)

#define DEFAULT_LOG_FILE				"libxvcd.log"
#define DEFAULT_TRACE_LEVEL				TRACELOG_NOTICE
#define DEFAULT_LOG_LEVEL				TRACELOG_INFO

#ifdef __cplusplus
extern "C" {
#endif

void libdbg_tracelog(char *fmt, ...);
void libdbg_hex_print(char *fmt, uint64_t phy, void *buf, int len, int align);

void libdbg_logfile(char *file);
void libdbg_tracelevel(int level);
void libdbg_loglevel(int level);

void libdbg_logclear(void);

#ifdef __cplusplus
}
#endif

#endif
