/**
 ******************************************************************************
 * @file    debug.c
 * @author  KEN
 * @version V1.0.0
 * @history 2024-05-25 (V1.0.0), first edition
 * @date	May. 25th, 2024
 * @brief	* Function Implements
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2024 K'sP</center></h2>
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <time.h>
#include <ctype.h>

#include "debug.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void hex_print(FILE *fp, uint64_t phy, uint8_t *buf, int len, int align);

/* Private variables ---------------------------------------------------------*/
static char *logfile = DEFAULT_LOG_FILE;
static int tracelevel = DEFAULT_TRACE_LEVEL;
static int loglevel = DEFAULT_LOG_LEVEL;

const char *lvl2color[] =
{
	"\e[0m", //
	"\e[0m", //
	"\e[0m", //
	"\e[31m", //LIBDBG_ERR
	"\e[33m", //LIBDBG_WARNING
	"\e[33m", //LIBDBG_NOTICE
	"\e[0m", //LIBDBG_INFO
	"\e[0m", //LIBDBG_DEBUG
	NULL

};

/* Private functions ---------------------------------------------------------*/
static void hex_print(FILE *fp, uint64_t phy, uint8_t *buf, int len, int align)
{
	int i, j, offset;
	int phy_dw = 0;
	uint64_t phy_end = phy + len - 1;

	for (phy_dw = 0; phy_end > 0; phy_end >>= 4, phy_dw++)
	{
	}

	fprintf(fp, "%*s", phy_dw + 2, "");

	for (i = 0; i < align; i++)
	{
		fprintf(fp, " %2X", (unsigned int) i);
	}
	fprintf(fp, "\n");

	for (j = 0; j < ((phy % align) + len + align - 1) / align; j++)
	{
		fprintf(fp, "0x%0*lX", phy_dw,
				(unsigned long) (phy - (phy % align) + j * align));

		for (i = 0; i < align; i++)
		{
			offset = j * align + i - (phy % align);

			if ((offset >= 0) && (offset < len))
			{
				fprintf(fp, " %02X", buf[offset]);
			} else
			{
				fprintf(fp, " %2s", "");
			}
		}

		fprintf(fp, " | ");
		for (i = 0; i < align; i++)
		{
			offset = j * align + i - (phy % align);

			if ((offset >= 0) && (offset < len))
			{
				fprintf(fp, "%c", isprint(buf[offset]) ? buf[offset] : '.');
			} else
			{
				fprintf(fp, " ");
			}
		}

		fprintf(fp, "\n");
	}
}

/* Public functions ----------------------------------------------------------*/
void libdbg_tracelog(char *fmt, ...)
{
	va_list args;
	time_t t;
	struct tm *ptm;

	static FILE *fp;
	int lvl;

	if (fmt[0] != '\001')
	{
		return;
	}

	lvl = fmt[1] - '0';

	if (lvl <= tracelevel)
	{
		printf("%s", lvl2color[lvl]);
		va_start(args, fmt);
		vprintf(fmt + 2, args);
		va_end(args);
		printf("\e[0m");

		fflush(stdout);
	}

	if (lvl <= loglevel)
	{
		fp = fopen(logfile, "a");

		if (fp)
		{
			time(&t);
			ptm = gmtime(&t);
			fprintf(fp, "[%04d-%02d-%02d %02d:%02d:%02d] ", 1900 + ptm->tm_year,
					ptm->tm_mon + 1, ptm->tm_mday, 8 + ptm->tm_hour,
					ptm->tm_min, ptm->tm_sec);

			va_start(args, fmt);
			vfprintf(fp, fmt + 2, args);
			va_end(args);

			fflush(fp);
			fclose(fp);
		}
	}
}

void libdbg_hex_print(char *fmt, uint64_t phy, void *buf, int len, int align)
{
	time_t t;
	struct tm *ptm;

	static FILE *fp;
	int lvl;

	if (fmt[0] != '\001')
	{
		return;
	}

	lvl = fmt[1] - '0';

	if (lvl <= tracelevel)
	{
		printf("%s", lvl2color[lvl]);
		hex_print(stdout, phy, buf, len, align);
		printf("\e[0m");

		fflush(stdout);
	}

	if (lvl <= loglevel)
	{
		fp = fopen(logfile, "a");

		if (fp)
		{
			time(&t);
			ptm = gmtime(&t);
			fprintf(fp, "[%04d-%02d-%02d %02d:%02d:%02d] \n",
					1900 + ptm->tm_year, ptm->tm_mon + 1, ptm->tm_mday,
					8 + ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

			hex_print(fp, phy, buf, len, align);

			fflush(fp);
			fclose(fp);
		}
	}
}

void libdbg_logfile(char *file)
{
	logfile = file;
}

void libdbg_tracelevel(int level)
{
	tracelevel = level;
}

void libdbg_loglevel(int level)
{
	loglevel = level;
}

void libdbg_logclear(void)
{
	static FILE *fp;
	fp = fopen(logfile, "w");

	if (fp)
	{
		fclose(fp);
	}
}

