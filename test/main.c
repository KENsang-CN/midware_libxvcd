/**
 ******************************************************************************
 * @file    main.c
 * @author  *
 * @version V1.0
 * @history 2024-5-8 (V1.0), first edition
 * @date	5月. 8th, 2024
 * @brief	* Function Implements
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2024 K'sP</center></h2>
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "libxvcd.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void sig_handler(int signum);
static void ftdi_scan(void);

/* Private variables ---------------------------------------------------------*/
void *handle;
bool running;

/* Private functions ---------------------------------------------------------*/
static void sig_handler(int signum)
{
	switch (signum)
	{
		case SIGINT:
		case SIGTERM:
#if !(defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) || defined(__WIN64__))
		case SIGKILL:
#endif
			xvcd_stop(handle);
			running = false;
			printf("\nxvc daemon stoped\n");
			fflush(stdout);
			break;
	}

}

static void ftdi_scan(void)
{
	int i, dev_cnt;
	char chipname[128];
	uint32_t chipid;
	char serialnum[128];
	char desc[128];

	dev_cnt = ftdi_device_scan();
	if (dev_cnt <= 0)
	{
		printf("none ftdi device found\n");
		return;
	}

	printf("ftdi found %d devices\n", dev_cnt);
	for (i = 0; i < dev_cnt; i++)
	{
		chipname[0] = serialnum[0] = desc[0] = 0;
		if (ftdi_device_info(i, chipname, &chipid, serialnum, desc) < 0)
		{
			printf("[%d] ?\n", i);
		} else
		{
			printf("[%d] chipname <%s>, "
					"id <0x%08X>, "
					"serialnum <%s>, "
					"desc <%s>\n", i, chipname, chipid, serialnum, desc);
		}
	}

	fflush(stdout);
}

/* Public functions ----------------------------------------------------------*/
int main(int argc, char **argv)
{
	int ret = 0;
	int index;

	if (argc != 2)
	{
		printf("Usage: %s <ftdi_index>\n", argv[0]);
		goto DONE;
	}

	ftdi_scan();

	index = atoi(argv[1]);

	handle = xvcd_start(index, 2542, 8192, 0);
	if (handle == NULL)
	{
		ret = -EPERM;
		goto DONE;
	}

	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);
#if !(defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) || defined(__WIN64__))
	signal(SIGKILL, sig_handler);
#endif

	running = true;

	while (running)
	{
		sleep(1);
	}

DONE:
	return ret;
}
