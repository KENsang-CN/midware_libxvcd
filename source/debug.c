#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <time.h>

#include "debug.h"

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

