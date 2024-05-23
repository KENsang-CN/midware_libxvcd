/**
 ******************************************************************************
 * @file    xvcd.c
 * @author  *
 * @version V1.0
 * @history 2024-5-6 (V1.0), first edition
 * @date	5月. 6th, 2024
 * @brief	* Function Implements
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2024 K'sP</center></h2>
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "libxvcd.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

#include "ftd2xx.h"
#include "version.h"
#include "debug.h"

/* Private typedef -----------------------------------------------------------*/
struct xvcd;

#if (defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) || defined(__WIN64__))
#include <winsock.h>

typedef SOCKET sock_t;
typedef int socklen_t;

#define IS_ERR(s)					(((void*)(s)) == NULL)
#define SOCK_NONE					0

#define socket_write(s,b,l)			send(s,b,l,0)
#define socket_read(s,b,l)			recv(s,b,l,0)
#define socket_close				closesocket

#define WSA_PATCH

#else /*linux gnuc*/
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <fcntl.h>

#include <libusb-1.0/libusb.h>

typedef int sock_t;

#define IS_ERR(s)					(s < 0)
#define SOCK_NONE					(-1)

#define socket_write				write
#define socket_read					read
#define socket_close				close

#endif

typedef enum
{
	FT_CMD_SET_CLK_DIV = 0x86, //
	FT_CMD_MCLK_DIV5_DISABLE = 0x8A, //
	FT_CMD_3PHASE_DATA_DISABLE = 0x8D, //
	FT_CMD_ADAPT_CLK_DISABLE = 0x97, //
	FT_CMD_LOOPBACK_DISABLE = 0x85, //
	FT_CMD_SET_GPIO_L = 0x80, //ADBUS
	FT_CMD_SET_GPIO_H = 0x82, //ACBUS
	FT_CMD_SHIFT_DATA_BYTE_NR = 0x19, //Clock Data Bytes Out on -ve clock edge LSB first (no read)
	FT_CMD_SHIFT_DATA_BIT_NR = 0x1B, //Clock Data Bits Out on -ve clock edge LSB first (no read)
	FT_CMD_SHIFT_DATA_BYTE = 0x39, //Clock Data Bytes In and Out LSB first
	FT_CMD_SHIFT_DATA_BIT = 0x3B, //Clock Data Bits In and Out LSB first
	FT_CMD_CLOCK_BYTE = 0x8F, //Clock For n x 8 bits with no data transfer
	FT_CMD_CLOCK_BIT = 0x8E, //Clock For n bits with no data transfer
	FT_CMD_SHIFT_TMS = 0x4B, //Clock Data to TMS pin (no read), TMS with LSB first on -ve clk edge
} ftdi_command_t;

struct ftdi_stack
{
	int bit_offset;
	int bit_len;
	bool is_last;
};

struct dpmem
{
	void *ptr;
	int size;
	int wpos, rpos;
};

struct ftdi
{
	void *parent;
	int index;
	FT_HANDLE dev;
	uint8_t gpiol_base_dir;
	uint8_t gpiol_base_val;
	double req_tck_frequency;
	double actual_tck_frequency;
	bool last_tms;
	bool last_tdi;

	struct dpmem cmd;
	struct dpmem stack;
};

struct tcp_socket
{
	void *parent;
	int port;
	sock_t local_sd;
	sock_t remote_sd;
	fd_set fds_raw;
	fd_set fds_selected;
};

typedef int (*xvc_cmd_callback)(struct xvcd *xvcd);

struct xvc_cmd
{
	const char *cmd;
	const xvc_cmd_callback *callbacks;
};

typedef enum
{
	STAGE_IDLE = 0, //
	STAGE_CMD, //
	STAGE_MARK, /*cmd mark*/
	STAGE_DATAW, /*word data*/
	STAGE_DATAT, /*threshold data*/
	STAGE_CALLBACK, //
} xvcd_stage_t;

struct xvcd
{
	struct ftdi ftdi;
	struct tcp_socket sock;

	pthread_attr_t thread_attr;
	pthread_t thread;
	bool running;

	int max_vector_len;
	bool lock;
	struct dpmem rxbuf;
	struct dpmem txbuf;
	xvcd_stage_t stage;
	int cmd_index[8];
	const struct xvc_cmd *cmd_sel;
	int cmd_pos;
	int cmd_cb_index;
	uint32_t cmd_uval;
	uint32_t cmd_threshold;
};

/* Private define ------------------------------------------------------------*/
#define xvcd_info(xvcd, fmt, args...) \
	libdbg_info("xvcd-%d: " fmt, xvcd->sock.port, ##args)

#define xvcd_err(xvcd, fmt, args...) \
	libdbg_err("xvcd-%d: " fmt, xvcd->sock.port, ##args)

#define XVC_CMD(_cmd, ...) \
	{ \
		.cmd = _cmd, \
		.callbacks = (const xvc_cmd_callback[]){__VA_ARGS__, NULL}, \
	}

#define Bit2Byte(bit)				(((bit) + 7) / 8)

/* Private macro -------------------------------------------------------------*/
#define FT_TMS_MAX_SIZE				7
#define FT_CLK_FREQ					60e6
#define FT_IOBUFFER_SIZE			(1024*64)

#define FT_GPIO_INPUT				(0U)
#define FT_GPIO_OUTPUT				(1U)

#define FT_GPIO_TCK					(0U)
#define FT_GPIO_TDI					(1U)
#define FT_GPIO_TDO					(2U)
#define FT_GPIO_TMS					(3U)

#define DEFAULT_JTAG_TCK_FREQ		10e6

//#define DEBUG

/* Private function prototypes -----------------------------------------------*/
static int ftdi_open(struct ftdi *ftdi);
static void ftdi_close(struct ftdi *ftdi);
static int ftdi_set_tck_frequency(struct ftdi *ftdi, double freq);
static void ftdi_set_tms(struct ftdi *ftdi, bool tms);
static int ftdi_shift_out(struct ftdi *ftdi, uint8_t *tms, uint8_t *tdi,
		int bitlen);
static int ftdi_detach_sio(struct ftdi *ftdi);
static int ftdi_attach_sio(struct ftdi *ftdi);

static int tcp_socket_open(struct tcp_socket *sock);
static void tcp_socket_close(struct tcp_socket *sock);

static int xvcd_select(struct xvcd *xvcd, struct timeval *tv);
static void xvcd_connet(struct xvcd *xvcd);
static void xvcd_parser(struct xvcd *xvcd);
static void xvcd_packer(struct xvcd *xvcd, struct timeval *tv);

static int xvcd_com_open(struct xvcd *xvcd);
static void xvcd_com_close(struct xvcd *xvcd);

static int xvcd_getinfo_callback(struct xvcd *xvcd);
static int xvcd_settck_callback(struct xvcd *xvcd);
static int xvcd_shift1_callback(struct xvcd *xvcd);
static int xvcd_shift2_callback(struct xvcd *xvcd);
static int xvcd_lock_callback(struct xvcd *xvcd);

static void* xvcd_thread(void *handle);

/* Private variables ---------------------------------------------------------*/
#if defined(__WIN32__) && defined(__MINGW32__)
static int wsacount = 0;
#endif

/* command string tokens
 * @: for callback generation
 * #w: for data receive of 4 bytes
 * #?: for data receive of before date value
 * */
static const struct xvc_cmd xvc_cmds[] =
{
	XVC_CMD("getinfo:@", xvcd_getinfo_callback), //
	XVC_CMD("settck:#w@", xvcd_settck_callback), //
	XVC_CMD("shift:#w@#?@", xvcd_shift1_callback, xvcd_shift2_callback), //
	XVC_CMD("lock:#w@", xvcd_lock_callback), //
	XVC_CMD(NULL, NULL)

	};

/* Private functions ---------------------------------------------------------*/
#ifdef DEBUG
static void HexPrint(char *head, uint64_t phy, uint8_t *buf, int len, int align)
{
	int i, j, offset;
	int phy_dw = 0;
	uint64_t phy_end = phy + len - 1;

	for (phy_dw = 0; phy_end > 0; phy_end >>= 4, phy_dw++)
	{
	}

	printf("%s%*s", head, phy_dw + 2, "");

	for (i = 0; i < align; i++)
	{
		printf(" %2X", (unsigned int) i);
	}
	printf("\n");

	for (j = 0; j < ((phy % align) + len + align - 1) / align; j++)
	{
		printf("%s0x%0*lX", head, phy_dw,
				(unsigned long) (phy - (phy % align) + j * align));

		for (i = 0; i < align; i++)
		{
			offset = j * align + i - (phy % align);

			if ((offset >= 0) && (offset < len))
			{
				printf(" %02X", buf[offset]);
			} else
			{
				printf(" %2s", "");
			}
		}

		printf(" | ");
		for (i = 0; i < align; i++)
		{
			offset = j * align + i - (phy % align);

			if ((offset >= 0) && (offset < len))
			{
#include <ctype.h>
				printf("%c", isprint(buf[offset]) ? buf[offset] : '.');
			} else
			{
				printf(" ");
			}
		}

		printf("\n");
	}
}
#endif

static int ftdi_open(struct ftdi *ftdi)
{
	int ret = 0, status;
	DWORD cnt = 0;
	uint8_t wbuf[1024];
	uint8_t rbuf[1024];
	double freq;

	ret = ftdi_detach_sio(ftdi);
	if (ret < 0)
	{
		goto DONE;
	}

	if ((status = FT_Open(ftdi->index, &ftdi->dev)) != FT_OK)
	{
		libdbg_err("ftdi: open failed (-%d)\n", status);
		ret = -EBUSY;
		goto DONE;
	}

	if ((status = FT_ResetDevice(ftdi->dev)) != FT_OK)
	{
		libdbg_err("ftdi: reset failed (-%d)\n", status);
		ret = -EBUSY;
		goto DONE;
	}

	if (((status = FT_GetQueueStatus(ftdi->dev, &cnt)) == FT_OK) && (cnt > 0))
	{
		status = FT_Purge(ftdi->dev, FT_PURGE_RX | FT_PURGE_TX);
	}

	status += FT_SetUSBParameters(ftdi->dev, FT_IOBUFFER_SIZE, //
			FT_IOBUFFER_SIZE);
	status += FT_SetChars(ftdi->dev, 0, 0, 0, 0);
	status += FT_SetTimeouts(ftdi->dev, 1000, 1000);
	status += FT_SetLatencyTimer(ftdi->dev, 2);
	status += FT_SetBitMode(ftdi->dev, 0, FT_BITMODE_RESET);
	status += FT_SetBitMode(ftdi->dev, 0, FT_BITMODE_MPSSE);
	if (status != FT_OK)
	{
		libdbg_err("ftdi: initialize for mpsse mode failed (-%d)\n", status);
		ret = -EBUSY;
		goto DONE;
	}

	usleep(50 * 1000);

	/*mpsse synchronizing*/
	cnt = 0;
	wbuf[cnt++] = 0xAA;
	status = FT_Write(ftdi->dev, wbuf, cnt, &cnt);
	status += FT_Read(ftdi->dev, rbuf, 2, &cnt);
	if ((status != FT_OK) || (rbuf[0] != 0xFA) || (rbuf[1] != 0xAA))
	{
		libdbg_err("ftdi: mpsse synchronizing failed (-%d)\n", status);
		ret = -EPERM;
		goto DONE;
	}

	/*initialize*/
	cnt = 0;
	wbuf[cnt++] = FT_CMD_MCLK_DIV5_DISABLE;
	wbuf[cnt++] = FT_CMD_ADAPT_CLK_DISABLE;
	wbuf[cnt++] = FT_CMD_3PHASE_DATA_DISABLE;
	wbuf[cnt++] = FT_CMD_LOOPBACK_DISABLE;
	status = FT_Write(ftdi->dev, wbuf, cnt, &cnt);
	if (status != FT_OK)
	{
		libdbg_err("ftdi: write failed (-%d)\n", status);
		ret = -EBUSY;
		goto DONE;
	}

	ftdi->gpiol_base_dir = (FT_GPIO_OUTPUT << 7)
			| (FT_GPIO_OUTPUT << FT_GPIO_TMS) | (FT_GPIO_OUTPUT << FT_GPIO_TDI)
			| (FT_GPIO_OUTPUT << FT_GPIO_TCK);
	ftdi->gpiol_base_val = (1U << 7);

	// Set initial states of the MPSSE interface - low byte, both pin directions and output values
	// Pin name 	Signal 	Direction 	Config 	Initial State
	// ADBUS0 		TCK 	output 1 	low 	0
	// ADBUS1 		TDI 	output 1 	low 	0
	// ADBUS2 		TDO 	input 0 			0
	// ADBUS3 		TMS 	output 1 	high 	1
	// ADBUS4 		GPIOL0 	input 0 			0
	// ADBUS5 		GPIOL1 	input 0 			0
	// ADBUS6 		GPIOL2 	input 0 			0
	// ADBUS7 		GPIOL3 	output 1 	high	1
	cnt = 0;
	wbuf[cnt++] = FT_CMD_SET_GPIO_L;
	wbuf[cnt++] = ftdi->gpiol_base_val | (1U << FT_GPIO_TMS); //value
	wbuf[cnt++] = ftdi->gpiol_base_dir; //direction
	status = FT_Write(ftdi->dev, wbuf, cnt, &cnt);
	if (status != FT_OK)
	{
		libdbg_err("ftdi: write failed (-%d)\n", status);
		ret = -EBUSY;
		goto DONE;
	}

	// Set initial states of the MPSSE interface - high byte, both pin directions and output values
	// Pin name 	Signal 	Direction 	Config 	Initial State
	// ACBUS0 		GPIOH0 	input 0 			0
	// ACBUS1 		GPIOH1 	input 0 			0
	// ACBUS2 		GPIOH2 	input 0 			0
	// ACBUS3 		GPIOH3 	input 0 			0
	// ACBUS4 		GPIOH4 	input 0 			0
	// ACBUS5 		GPIOH5 	input 0 			0
	// ACBUS6 		GPIOH6 	input 0 			0
	// ACBUS7 		GPIOH7 	input 0 			0
	cnt = 0;
	wbuf[cnt++] = FT_CMD_SET_GPIO_H;
	wbuf[cnt++] = 0x00; //value
	wbuf[cnt++] = 0x00; //direction
	status = FT_Write(ftdi->dev, wbuf, cnt, &cnt);
	if (status != FT_OK)
	{
		libdbg_err("ftdi: write failed (-%d)\n", status);
		ret = -EBUSY;
		goto DONE;
	}

	ftdi->last_tms = true;
	ftdi->last_tdi = false;

	freq = ftdi->req_tck_frequency ?
			ftdi->req_tck_frequency : DEFAULT_JTAG_TCK_FREQ;

	ret = ftdi_set_tck_frequency(ftdi, freq);
	if (ret < 0)
	{
		goto DONE;
	}

	ftdi->cmd.size = FT_IOBUFFER_SIZE;
	ftdi->cmd.ptr = malloc(ftdi->cmd.size);
	if (ftdi->cmd.ptr == NULL)
	{
		libdbg_err("ftdi: command buffer malloc failed\n");
		ret = -EPERM;
		goto DONE;
	}
	ftdi->cmd.wpos = ftdi->cmd.rpos = 0;

	ftdi->stack.size = ftdi->cmd.size / 3;
	ftdi->stack.ptr = malloc(sizeof(struct ftdi_stack) * ftdi->stack.size);
	if (ftdi->stack.ptr == NULL)
	{
		libdbg_err("ftdi: command stack malloc failed\n");
		ret = -EPERM;
		goto DONE;
	}

	ftdi->stack.wpos = ftdi->stack.rpos = 0;

	DONE: if (ret < 0)
	{
		ftdi_close(ftdi);
	}

	return ret;
}

static void ftdi_close(struct ftdi *ftdi)
{
	if (ftdi->stack.ptr)
	{
		free(ftdi->stack.ptr);
		ftdi->stack.ptr = NULL;
	}

	if (ftdi->cmd.ptr)
	{
		free(ftdi->cmd.ptr);
		ftdi->cmd.ptr = NULL;
	}

	if (ftdi->dev)
	{
		FT_Close(ftdi->dev);
		ftdi->dev = NULL;
	}

	ftdi_attach_sio(ftdi);
}

static int ftdi_set_tck_frequency(struct ftdi *ftdi, double freq)
{
	int ret = 0;
	uint16_t div;
	uint8_t wbuf[3];
	DWORD cnt;
	int status;

	cnt = 0;
	div = (FT_CLK_FREQ / (freq * 2)) - 1;
	wbuf[cnt++] = FT_CMD_SET_CLK_DIV;
	wbuf[cnt++] = div & 0xFF;
	wbuf[cnt++] = div >> 8;

	status = FT_Write(ftdi->dev, wbuf, cnt, &cnt);
	if (status != FT_OK)
	{
		libdbg_err("ftdi: write failed(-%d)\n", status);
		ret = -EBUSY;
		goto DONE;
	}

	ftdi->actual_tck_frequency = FT_CLK_FREQ / ((div + 1) * 2);

	DONE: return ret;
}

static void ftdi_set_tms(struct ftdi *ftdi, bool tms)
{
	int pos = ftdi->cmd.wpos;
	uint8_t *buf = (uint8_t*) ftdi->cmd.ptr;
	bool tck = false;
	bool tdi = ftdi->last_tdi;

	if (ftdi->last_tms != tms)
	{
		// Set initial states of the MPSSE interface - low byte, both pin directions and output values
		// Pin name 	Signal 	Direction 	Config 	Initial State
		// ADBUS0 		TCK 	output 1 	low 	0
		// ADBUS1 		TDI 	output 1 	low 	0
		// ADBUS2 		TDO 	input 0 			0
		// ADBUS3 		TMS 	output 1 	high 	1
		// ADBUS4 		GPIOL0 	input 0 			0
		// ADBUS5 		GPIOL1 	input 0 			0
		// ADBUS6 		GPIOL2 	input 0 			0
		// ADBUS7 		GPIOL3 	output 1 	high	1
		buf[pos++] = FT_CMD_SET_GPIO_L;
		buf[pos++] = ftdi->gpiol_base_val | ((tck ? 1U : 0U) << FT_GPIO_TCK) | //
				((tdi ? 1U : 0U) << FT_GPIO_TDI) | //
				((tms ? 1U : 0U) << FT_GPIO_TMS); //value
		buf[pos++] = ftdi->gpiol_base_dir; //direction

		ftdi->last_tms = tms;
		ftdi->cmd.wpos = pos;
	}
}

static int ftdi_shift_out(struct ftdi *ftdi, uint8_t *tms, uint8_t *tdi,
		int bitlen)
{
	int ret = 0;
	int byteindex, bytetotal, bitindex, bittotal, cnt, pos, status;
	int cnt_margin = ((bitlen % 8) == 0) ? 0 : 1;
	uint8_t bits;
	uint8_t *buf = (uint8_t*) ftdi->cmd.ptr;
	DWORD iocnt;

	bytetotal = Bit2Byte(bitlen);
	ftdi->cmd.wpos = 0;

	byteindex = 0;
	while (byteindex < bytetotal)
	{
		if ((tms[byteindex] == 0x00) || (tms[byteindex] == 0xFF))
		{
			ftdi_set_tms(ftdi, tms[byteindex] == 0xFF);

			for (cnt = 1;
					((byteindex + cnt + cnt_margin) < bytetotal)
							&& (tms[byteindex + cnt] == tms[byteindex]); cnt++)
			{
			}

			if ((byteindex + cnt) == bytetotal)
			{
				bittotal = bitlen - (byteindex * 8);
			} else
			{
				bittotal = cnt * 8;
			}

			pos = ftdi->cmd.wpos;
			if (bittotal >= 8)
			{
				buf[pos++] = FT_CMD_SHIFT_DATA_BYTE;
				buf[pos++] = (cnt - 1) & 0xFF;
				buf[pos++] = (cnt - 1) >> 8;
			} else
			{
				buf[pos++] = FT_CMD_SHIFT_DATA_BIT;
				buf[pos++] = bittotal - 1;
			}
			memcpy(&buf[pos], &tdi[byteindex], cnt), pos += cnt;
			ftdi->cmd.wpos = pos;

			pos = ftdi->stack.wpos;
			((struct ftdi_stack*) ftdi->stack.ptr)[pos].bit_offset = 0;
			((struct ftdi_stack*) ftdi->stack.ptr)[pos].bit_len = bittotal;
			((struct ftdi_stack*) ftdi->stack.ptr)[pos].is_last = (byteindex
					+ cnt) == bytetotal;
			ftdi->stack.wpos = (pos + 1) % ftdi->stack.size;

			ftdi->last_tdi = (tdi[byteindex + cnt - 1] & 0x80) != 0;

			byteindex += cnt;
		} else
		{
			bitindex = 0;
			bits = tms[byteindex];

			if ((byteindex + 1) == bytetotal)
			{
				bittotal = bitlen - (byteindex * 8);
			} else
			{
				bittotal = 8;
			}

			while (bitindex < bittotal)
			{
				ftdi_set_tms(ftdi, (bits & 0x1) != 0);

				for (cnt = 1;
						((cnt + bitindex) < bittotal)
								&& ((((bits >> cnt) ^ bits) & 0x1) == 0); cnt++)
				{

				}

				pos = ftdi->cmd.wpos;
				buf[pos++] = FT_CMD_SHIFT_DATA_BIT;
				buf[pos++] = cnt - 1;
				buf[pos++] = (tdi[byteindex] >> bitindex) & ((1U << cnt) - 1);
				ftdi->cmd.wpos = pos;

				pos = ftdi->stack.wpos;
				((struct ftdi_stack*) ftdi->stack.ptr)[pos].bit_offset =
						bitindex;
				((struct ftdi_stack*) ftdi->stack.ptr)[pos].bit_len = cnt;
				((struct ftdi_stack*) ftdi->stack.ptr)[pos].is_last =
						((byteindex + 1) == bytetotal)
								&& ((bitindex + cnt) == bittotal);
				ftdi->stack.wpos = (pos + 1) % ftdi->stack.size;

				ftdi->last_tdi = (tdi[byteindex] & (1U << (bitindex + cnt - 1)))
						!= 0;

				bits = bits >> cnt;
				bitindex += cnt;
			}

			byteindex++;
		}
	}

	if (ftdi->cmd.wpos)
	{
#ifdef DEBUG
		printf("\t\tftdi command buffer:\n");
		HexPrint("\t\t", 0, ftdi->cmd.ptr, ftdi->cmd.wpos, 16);
#endif

		status = FT_Write(ftdi->dev, ftdi->cmd.ptr, ftdi->cmd.wpos, &iocnt);
		if (status != FT_OK)
		{
			libdbg_err("jtag-ftdi: write failed (-%d)\n", status);
			ret = -EBUSY;
			goto DONE;
		}
	}

	DONE: return ret;
}

#if (defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) || defined(__WIN64__))
static int ftdi_detach_sio(struct ftdi *ftdi)
{
	return 0;
}

static int ftdi_attach_sio(struct ftdi *ftdi)
{
	return 0;
}
#else
static int ftdi_detach_sio(struct ftdi *ftdi)
{
	int ret = 0;
	libusb_context *uctx = NULL;
	libusb_device *udev = NULL, **udev_list;
	int usb_index, dev_index, intf_index = 0;
	struct libusb_device_descriptor udev_desc;
	struct libusb_config_descriptor *ucfg_desc;
	struct libusb_device_handle *udev_handle;

	ret = libusb_init(&uctx);
	if (ret < 0)
	{
		libdbg_err("libusb init failed (%d)\n", ret);
		goto DONE;
	}

	ret = libusb_get_device_list(uctx, &udev_list);
	if (ret < 0)
	{
		libdbg_err("libusb get device list failed (%d)\n", ret);
		goto FREE1;
	}

	for (usb_index = 0, dev_index = 0; udev_list[usb_index]; usb_index++)
	{
		ret = libusb_get_device_descriptor(udev_list[usb_index], &udev_desc);
		if (ret < 0)
		{
			libdbg_err("libusb get device descriptor failed (%d)\n", ret);
			goto FREE2;
		}

		if ((udev_desc.idVendor == 0x403)
				&& (udev_desc.idProduct == 0x6001
						|| udev_desc.idProduct == 0x6010
						|| udev_desc.idProduct == 0x6011
						|| udev_desc.idProduct == 0x6014
						|| udev_desc.idProduct == 0x6015))
		{
			ret = libusb_get_config_descriptor(udev_list[usb_index], 0,
					&ucfg_desc);
			if (ret < 0)
			{
				libdbg_err("libusb get config descriptor failed (%d)\n", ret);
				goto FREE2;
			}

			for (intf_index = 0;
					intf_index < ucfg_desc->bNumInterfaces && !udev;
					intf_index++, dev_index++)
			{
				if (ftdi->index == dev_index)
				{
					udev = udev_list[usb_index];
					break;
				}
			}

			libusb_free_config_descriptor(ucfg_desc);

			if (udev)
			{
				break;
			}
		}
	}

	if (udev == NULL)
	{
		libdbg_err("find no specific index %d device\n", ftdi->index);
		ret = -EEXIST;
		goto FREE2;
	}

	ret = libusb_open(udev, &udev_handle);
	if (ret < 0)
	{
		libdbg_err("libusb open failed (%d)\n", ret);
		goto FREE2;
	}

	if (libusb_kernel_driver_active(udev_handle, intf_index) == 1)
	{
		ret = libusb_detach_kernel_driver(udev_handle, intf_index);
		if (ret < 0)
		{
			libdbg_warn("libusb detach sio driver failed (%d), " //
					"try to use root privilege\n", ret);
			goto FREE3;
		}
	}

	FREE3: libusb_close(udev_handle);
	FREE2: libusb_free_device_list(udev_list, 1);
	FREE1: libusb_exit(uctx);

	DONE: return ret;
}

static int ftdi_attach_sio(struct ftdi *ftdi)
{
	int ret = 0;
	libusb_context *uctx = NULL;
	libusb_device *udev = NULL, **udev_list;
	int usb_index, dev_index, intf_index = 0;
	struct libusb_device_descriptor udev_desc;
	struct libusb_config_descriptor *ucfg_desc;
	struct libusb_device_handle *udev_handle;

	ret = libusb_init(&uctx);
	if (ret < 0)
	{
		libdbg_err("libusb init failed (%d)\n", ret);
		goto DONE;
	}

	ret = libusb_get_device_list(uctx, &udev_list);
	if (ret < 0)
	{
		libdbg_err("libusb get device list failed (%d)\n", ret);
		goto FREE1;
	}

	for (usb_index = 0, dev_index = 0; udev_list[usb_index]; usb_index++)
	{
		ret = libusb_get_device_descriptor(udev_list[usb_index], &udev_desc);
		if (ret < 0)
		{
			libdbg_err("libusb get device descriptor failed (%d)\n", ret);
			goto FREE2;
		}

		if ((udev_desc.idVendor == 0x403)
				&& (udev_desc.idProduct == 0x6001
						|| udev_desc.idProduct == 0x6010
						|| udev_desc.idProduct == 0x6011
						|| udev_desc.idProduct == 0x6014
						|| udev_desc.idProduct == 0x6015))
		{
			ret = libusb_get_config_descriptor(udev_list[usb_index], 0,
					&ucfg_desc);
			if (ret < 0)
			{
				libdbg_err("libusb get config descriptor failed (%d)\n", ret);
				goto FREE2;
			}

			for (intf_index = 0;
					intf_index < ucfg_desc->bNumInterfaces && !udev;
					intf_index++, dev_index++)
			{
				if (ftdi->index == dev_index)
				{
					udev = udev_list[usb_index];
					break;
				}
			}

			libusb_free_config_descriptor(ucfg_desc);

			if (udev)
			{
				break;
			}
		}
	}

	if (udev == NULL)
	{
		libdbg_err("find no specific index %d device\n", ftdi->index);
		ret = -EEXIST;
		goto FREE2;
	}

	ret = libusb_open(udev, &udev_handle);
	if (ret < 0)
	{
		libdbg_err("libusb open failed (%d)\n", ret);
		goto FREE2;
	}

	if (libusb_kernel_driver_active(udev_handle, intf_index) == 0)
	{
		ret = libusb_attach_kernel_driver(udev_handle, intf_index);
		if (ret < 0)
		{
			libdbg_warn("libusb attach sio driver failed (%d)\n", ret);
			goto FREE3;
		}
	}

	FREE3: libusb_close(udev_handle);
	FREE2: libusb_free_device_list(udev_list, 1);
	FREE1: libusb_exit(uctx);

	DONE: return ret;
}

static int ftdi_get_sysfs_name(char *buf, size_t size, libusb_device *dev)
{
	int len = 0;
	uint8_t bnum = libusb_get_bus_number(dev);
	uint8_t pnums[7];
	int num_pnums;

	buf[0] = '\0';

	num_pnums = libusb_get_port_numbers(dev, pnums, sizeof(pnums));
	if (num_pnums == LIBUSB_ERROR_OVERFLOW)
	{
		return -1;
	} else if (num_pnums == 0)
	{
		/* Special-case root devices */
		return snprintf(buf, size, "usb%d", bnum);
	}

	len += snprintf(buf, size, "%d-", bnum);
	for (int i = 0; i < num_pnums; i++)
		len += snprintf(buf + len, size - len, i ? ".%d" : "%d", pnums[i]);

	return len;
}

static int ftdi_read_sysfs_prop(char *buf, char *sysfs_name, char *propname)
{
	int ret = 0, fd, len;
	char path[PATH_MAX];
	char prop[128];

	if ((buf == NULL) || (sysfs_name == NULL) || (propname == NULL))
	{
		goto DONE;
	}

	buf[0] = '\0';

	snprintf(path, sizeof(path), "/sys/bus/usb/devices/%s/%s", sysfs_name,
			propname);
	fd = open(path, O_RDONLY);
	if (fd < 0)
	{
		ret = -EPERM;
		goto DONE;
	}

	len = read(fd, prop, 128);

	if (len > 0)
	{
		memcpy(buf, prop, len - 1);
		buf[len - 1] = 0;
	}

	close(fd);

	DONE: return ret;
}
#endif

static int tcp_socket_open(struct tcp_socket *sock)
{
	int ret = 0;
	struct sockaddr_in local;

#ifdef WSA_PATCH
	WSADATA wsa_data;
	ULONG nonblk = 1;
	if (wsacount == 0)
	{
		if (WSAStartup(MAKEWORD(1, 1), &wsa_data) != 0)
		{
			libdbg_err("tcp-socket: wsa startup failed\n");
			ret = -EPERM;
			goto DONE;
		}
	}
	wsacount++;
#endif

	sock->local_sd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (IS_ERR(sock->local_sd))
	{
		libdbg_err("tcp-socket: local socket create failed\n");
		ret = -EPERM;
		goto DONE;
	}

	local.sin_port = htons(sock->port);
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = INADDR_ANY;
	ret = bind(sock->local_sd, (struct sockaddr*) &local,
			sizeof(struct sockaddr));
	if (ret < 0)
	{
		libdbg_err("tcp-socket: local socket bind failed (%d)\n", ret);
		goto CLOSE;
	}

	ret = listen(sock->local_sd, 2);
	if (ret < 0)
	{
		libdbg_err("tcp-socket: local socket listen failed (%d)\n", ret);
		goto CLOSE;
	}

#ifdef WSA_PATCH
	ret = ioctlsocket(sock->local_sd, FIONBIO, &nonblk);
	if (ret < 0)
	{
		libdbg_err("tcp-socket: switch to non-block mode failed (%d)\n", ret);
		goto CLOSE;
	}
#endif

	sock->remote_sd = SOCK_NONE;

	FD_ZERO(&sock->fds_raw);
	FD_ZERO(&sock->fds_selected);
	FD_SET(sock->local_sd, &sock->fds_raw);

	goto DONE;

	CLOSE:
	socket_close(sock->local_sd);

#ifdef WSA_PATCH
	wsacount--;
	if (wsacount == 0)
	{
		WSACleanup();
	}
#endif

	DONE: return ret;
}

static void tcp_socket_close(struct tcp_socket *sock)
{
	if (!IS_ERR(sock->remote_sd))
	{
		socket_close(sock->remote_sd);
	}

	if (!IS_ERR(sock->local_sd))
	{
		socket_close(sock->local_sd);

#ifdef WSA_PATCH
		wsacount--;
		if (wsacount == 0)
		{
			WSACleanup();
		}
#endif
	}
}

static int xvcd_select(struct xvcd *xvcd, struct timeval *tv)
{
	int maxfd = 0;

	xvcd->sock.fds_selected = xvcd->sock.fds_raw;
#ifndef WSA_PATCH
	if (!IS_ERR(xvcd->sock.local_sd) && (xvcd->sock.local_sd > maxfd))
	{
		maxfd = xvcd->sock.local_sd;
	}

	if (!IS_ERR(xvcd->sock.remote_sd) && (xvcd->sock.remote_sd > maxfd))
	{
		maxfd = xvcd->sock.remote_sd;
	}
#endif

	return select(maxfd + 1, &xvcd->sock.fds_selected, NULL, NULL, tv);
}

static void xvcd_connet(struct xvcd *xvcd)
{
	struct sockaddr_in old_remote, new_remote;
	socklen_t addr_len = sizeof(struct sockaddr_in);
	sock_t sd;

	if (IS_ERR(xvcd->sock.local_sd) || //
			!FD_ISSET(xvcd->sock.local_sd, &xvcd->sock.fds_selected))
	{
		return;
	}

	if (!xvcd->lock)
	{
		if (IS_ERR(xvcd->sock.remote_sd))
		{
			xvcd->sock.remote_sd = accept(xvcd->sock.local_sd,
					(struct sockaddr*) &new_remote, &addr_len);
			if (IS_ERR(xvcd->sock.remote_sd))
			{
				return;
			}

			if (getpeername(xvcd->sock.remote_sd,
					(struct sockaddr*) &new_remote, &addr_len) < 0)
			{
				memset(&new_remote, 0, sizeof(struct sockaddr_in));
			}

			xvcd_info(xvcd, "remote connected @%s:%d\n",
					inet_ntoa(new_remote.sin_addr), ntohs(new_remote.sin_port));

			if (xvcd_com_open(xvcd) < 0)
			{
				xvcd_err(xvcd, "xvcd device open failed\n");

				socket_close(xvcd->sock.remote_sd);
				xvcd->sock.remote_sd = SOCK_NONE;
			} else
			{
				FD_SET(xvcd->sock.remote_sd, &xvcd->sock.fds_raw);
			}
		} else
		{
			xvcd_com_close(xvcd);
		}
	} else
	{
		if (getpeername(xvcd->sock.remote_sd, (struct sockaddr*) &old_remote,
				&addr_len) < 0)
		{
			memset(&old_remote, 0, sizeof(struct sockaddr_in));
		}

		sd = accept(xvcd->sock.local_sd, (struct sockaddr*) &new_remote,
				&addr_len);
		close(sd);

		xvcd_info(xvcd, "reject connection from %s:%d (locked by %s:%d)\n",
				inet_ntoa(new_remote.sin_addr), ntohs(new_remote.sin_port),
				inet_ntoa(old_remote.sin_addr), ntohs(old_remote.sin_port));
	}
}

static void xvcd_packer(struct xvcd *xvcd, struct timeval *tv)
{
	struct ftdi *ftdi = &xvcd->ftdi;
	int ret = 0, status, pos, bitcnt, bytecnt, bitoffset;
	DWORD iocnt = 0, rdcnt;
	uint8_t temp, *dout, *buf = xvcd->txbuf.ptr + xvcd->txbuf.wpos;
	bool is_last;

	if (IS_ERR(xvcd->sock.remote_sd) || (ftdi->stack.wpos == ftdi->stack.rpos))
	{
		tv->tv_sec = 1;
		tv->tv_usec = 0;
		return;
	}

	status = FT_GetQueueStatus(ftdi->dev, &iocnt);
	if (status != FT_OK)
	{
		libdbg_err("jtag-ftdi: get queue status failed (-%d)\n", status);
		ret = -EBUSY;
		goto DONE;
	}

#ifdef DEBUG
	if (iocnt > 0)
	{
		printf("\tftdi %d byte recieved\n", (int) iocnt);
	}
#endif

	pos = ftdi->stack.rpos;
	while (iocnt > 0)
	{
		bitcnt = ((struct ftdi_stack*) ftdi->stack.ptr)[pos].bit_len;
		bytecnt = Bit2Byte(bitcnt);
		bitoffset = ((struct ftdi_stack*) ftdi->stack.ptr)[pos].bit_offset;
		is_last = ((struct ftdi_stack*) ftdi->stack.ptr)[pos].is_last;

#ifdef DEBUG
		printf("\tstack[%d]: length %d bits, offset %d bits, last%c\n", pos,
				bitcnt, bitoffset, is_last ? '+' : '-');
#endif

		if (iocnt < bytecnt)
		{
			break;
		}
		iocnt -= bytecnt;

		if (bitcnt < 8)
		{
			dout = &temp;

			if (bitoffset == 0)
			{
				*buf = 0;
			}
		} else
		{
			dout = buf;
		}

		status = FT_Read(ftdi->dev, dout, bytecnt, &rdcnt);
		if (status != FT_OK)
		{
			libdbg_err("jtag-ftdi: read failed (-%d)\n", status);
			ret = -EBUSY;
			goto DONE;
		}

		if (bitcnt < 8)
		{
			dout[0] >>= (8 - bitcnt);
		}

#ifdef DEBUG
		printf("\tftdi read buffer contents:\n");
		HexPrint("\t\t", 0, dout, bytecnt, 16);
#endif

		if (bitcnt < 8)
		{
			*buf |= (temp & ((1U << bitcnt) - 1)) << bitoffset;
		}

		if ((bitcnt < 8) && ((bitcnt + bitoffset) != 8) && !is_last)
		{
			bytecnt = 0;
		}

		buf += bytecnt;
		xvcd->txbuf.wpos += bytecnt;

		if (is_last)
		{
#ifdef DEBUG
			printf("\ttdo contents (%d bytes):\n", xvcd->txbuf.wpos);
			HexPrint("\t\t", 0, xvcd->txbuf.ptr, xvcd->txbuf.wpos,
					16);
#endif

			ret = socket_write(xvcd->sock.remote_sd, xvcd->txbuf.ptr,
					xvcd->txbuf.wpos);
			if (ret < 0)
			{
				break;
			}

			xvcd->txbuf.wpos = 0;
		}

		pos = (pos + 1) % ftdi->stack.size;
	}
	ftdi->stack.rpos = pos;

	if (ftdi->stack.wpos == ftdi->stack.rpos)
	{
		tv->tv_sec = 1;
		tv->tv_usec = 0;
	} else
	{
		tv->tv_sec = 0;
		tv->tv_usec = 0;
	}

	DONE: if (ret < 0)
	{
		xvcd_com_close(xvcd);
	}
}

static void xvcd_parser(struct xvcd *xvcd)
{
	int ret = 0;
	int last_stage, sb_avail, i;
	char cval;

	if (IS_ERR(xvcd->sock.remote_sd) || //
			!FD_ISSET(xvcd->sock.remote_sd, &xvcd->sock.fds_selected))
	{
		return;
	}

	ret = socket_read(xvcd->sock.remote_sd, xvcd->rxbuf.ptr + xvcd->rxbuf.wpos,
			xvcd->rxbuf.size - xvcd->rxbuf.wpos);
	if (ret <= 0)
	{
		ret = -EPERM;
		goto DONE;
	}

	xvcd->rxbuf.wpos += ret;

	do
	{
		sb_avail = xvcd->rxbuf.wpos - xvcd->rxbuf.rpos;
		last_stage = xvcd->stage;

		switch (xvcd->stage)
		{
			case STAGE_IDLE:
				if (sb_avail)
				{
					cval = *((char*) xvcd->rxbuf.ptr + xvcd->rxbuf.rpos);

					for (i = 0; xvc_cmds[i].cmd; i++)
					{
						if (cval == xvc_cmds[i].cmd[xvcd->cmd_index[i]])
						{
							xvcd->cmd_index[i]++;
						} else
						{
							xvcd->cmd_index[i] = 0;
						}
					}

					if (cval == ':')
					{
						for (i = 0; xvc_cmds[i].cmd; i++)
						{
							if (xvcd->cmd_index[i])
							{
								xvcd->stage = STAGE_CMD;

								xvcd->cmd_sel = &xvc_cmds[i];
								xvcd->cmd_pos = xvcd->cmd_index[i];
								xvcd->cmd_cb_index = 0;

								xvcd->cmd_index[i] = 0;
								break;
							}
						}
					}

					xvcd->rxbuf.rpos++;
				}
				break;
			case STAGE_CMD:
				switch (xvcd->cmd_sel->cmd[xvcd->cmd_pos])
				{
					case '@':
						xvcd->stage = STAGE_CALLBACK;
						break;
					case '#':
						xvcd->stage = STAGE_MARK;
						break;
					case 0:
						xvcd->stage = STAGE_IDLE;
						break;
				}
				xvcd->cmd_pos++;
				break;
			case STAGE_MARK:
				switch (xvcd->cmd_sel->cmd[xvcd->cmd_pos])
				{
					case 'w':
						xvcd->stage = STAGE_DATAW;
						break;
					case '?':
						xvcd->stage = STAGE_DATAT;
						break;
				}
				xvcd->cmd_pos++;
				break;
			case STAGE_DATAW:
				if (sb_avail >= sizeof(uint32_t))
				{
					memcpy(&xvcd->cmd_uval, xvcd->rxbuf.ptr + xvcd->rxbuf.rpos,
							sizeof(uint32_t));
					xvcd->stage = STAGE_CMD;
				} else
				{
					goto DONE;
				}
				break;
			case STAGE_DATAT:
				if (sb_avail >= xvcd->cmd_threshold)
				{
					xvcd->stage = STAGE_CMD;
				} else
				{
					goto DONE;
				}
				break;
			case STAGE_CALLBACK:
				ret = xvcd->cmd_sel->callbacks[xvcd->cmd_cb_index](xvcd);
				if (ret < 0)
				{
					xvcd_err(xvcd, "callback %d failed in command %s\n",
							xvcd->cmd_cb_index, xvcd->cmd_sel->cmd);
					xvcd->stage = STAGE_IDLE;
				} else
				{
					xvcd->cmd_cb_index++;
					xvcd->stage = STAGE_CMD;
				}
				break;
		}

	} while ((xvcd->rxbuf.wpos != xvcd->rxbuf.rpos)
			|| (last_stage != xvcd->stage));

	if ((xvcd->rxbuf.rpos != 0) && (sb_avail == 0))
	{
		xvcd->rxbuf.wpos = xvcd->rxbuf.rpos = 0;
	}

	DONE: if (ret < 0)
	{
		xvcd_com_close(xvcd);
	}
}

static int xvcd_com_open(struct xvcd *xvcd)
{
	int ret = 0;

	ret = ftdi_open(&xvcd->ftdi);
	if (ret < 0)
	{
		xvcd_err(xvcd, "ftdi open failed\n");
		goto DONE;
	}

	xvcd->rxbuf.size = (xvcd->max_vector_len * 2 + 10) * 2;
	xvcd->rxbuf.ptr = malloc(xvcd->rxbuf.size);
	if (xvcd->rxbuf.ptr == NULL)
	{
		ret = -ENOMEM;
		xvcd_err(xvcd, "command buffer alloc failed\n");
		goto FREE1;
	}

	memset(xvcd->rxbuf.ptr, 0, xvcd->rxbuf.size);
	xvcd->rxbuf.wpos = xvcd->rxbuf.rpos = 0;

	xvcd->txbuf.size = xvcd->max_vector_len * 2;
	xvcd->txbuf.ptr = malloc(xvcd->txbuf.size);
	if (xvcd->txbuf.ptr == NULL)
	{
		ret = -ENOMEM;
		xvcd_err(xvcd, "data buffer alloc failed\n");
		goto FREE2;
	}

	memset(xvcd->txbuf.ptr, 0, xvcd->txbuf.size);
	xvcd->txbuf.wpos = xvcd->txbuf.rpos = 0;

	xvcd->stage = STAGE_IDLE;
	xvcd->lock = false;

	goto DONE;

	FREE2: free(xvcd->rxbuf.ptr);

	FREE1: ftdi_close(&xvcd->ftdi);

	DONE: return ret;
}

static void xvcd_com_close(struct xvcd *xvcd)
{
	struct sockaddr_in remote;
	socklen_t addr_len = sizeof(struct sockaddr_in);

	if (!IS_ERR(xvcd->sock.remote_sd))
	{
		if (getpeername(xvcd->sock.remote_sd, (struct sockaddr*) &remote,
				&addr_len) < 0)
		{
			memset(&remote, 0, sizeof(struct sockaddr_in));
		}

		if (xvcd->lock)
		{
			xvcd->lock = false;
			xvcd_info(xvcd, "connection auto unlock from @%s:%d\n",
					inet_ntoa(remote.sin_addr), ntohs(remote.sin_port));
		}

		xvcd_info(xvcd, "remote closed @%s:%d\n", inet_ntoa(remote.sin_addr),
				ntohs(remote.sin_port));

		FD_CLR(xvcd->sock.remote_sd, &xvcd->sock.fds_raw);
		FD_CLR(xvcd->sock.remote_sd, &xvcd->sock.fds_selected);
		socket_close(xvcd->sock.remote_sd);
		xvcd->sock.remote_sd = SOCK_NONE;
	}

	if (xvcd->txbuf.ptr)
	{
		free(xvcd->txbuf.ptr);
		xvcd->txbuf.ptr = NULL;
	}

	if (xvcd->rxbuf.ptr)
	{
		free(xvcd->rxbuf.ptr);
		xvcd->rxbuf.ptr = NULL;
	}

	ftdi_close(&xvcd->ftdi);
}

static int xvcd_getinfo_callback(struct xvcd *xvcd)
{
	int ret = 0;
	char *sbuf = xvcd->txbuf.ptr;

	sbuf[0] = 0, snprintf(sbuf, xvcd->txbuf.size, "xvcServer_v1.0:%d\n",
			xvcd->max_vector_len);

	xvcd->txbuf.wpos = strlen(sbuf);
	memcpy(xvcd->txbuf.ptr, sbuf, xvcd->txbuf.wpos);

	ret = socket_write(xvcd->sock.remote_sd, xvcd->txbuf.ptr, xvcd->txbuf.wpos);
	xvcd->txbuf.wpos = 0;

	return ret;
}

static int xvcd_settck_callback(struct xvcd *xvcd)
{
	int ret = 0;
	double freq;

	if (xvcd->cmd_uval == 0)
	{
		xvcd_err(xvcd, "invalid tck period\n");
		ret = -EPERM;
		goto DONE;
	}

	if (xvcd->ftdi.req_tck_frequency == 0)
	{
		freq = 1e9 / xvcd->cmd_uval;
		ret = ftdi_set_tck_frequency(&xvcd->ftdi, freq);
		if (ret < 0)
		{
			goto DONE;
		}

		xvcd_info(xvcd, "set tck frequency to %0.f Hz\n",
				xvcd->ftdi.actual_tck_frequency);
	} else
	{
		//TCK frequency is fixed
		xvcd->cmd_uval = 1e9 / xvcd->ftdi.actual_tck_frequency;

		xvcd_info(xvcd, "use forced tck frequency %0.f Hz\n",
				xvcd->ftdi.actual_tck_frequency);
	}

	xvcd->rxbuf.rpos += sizeof(uint32_t);

	//send tck period(ns) to client
	memcpy(xvcd->txbuf.ptr, &xvcd->cmd_uval, sizeof(uint32_t));
	xvcd->txbuf.wpos = sizeof(uint32_t);

	ret = socket_write(xvcd->sock.remote_sd, xvcd->txbuf.ptr, xvcd->txbuf.wpos);
	xvcd->txbuf.wpos = 0;

	DONE: return ret;
}

static int xvcd_shift1_callback(struct xvcd *xvcd)
{
	int ret = 0;

	xvcd->rxbuf.rpos += sizeof(uint32_t);

	xvcd->cmd_threshold = ((xvcd->cmd_uval + 7) / 8) << 1;

	return ret;
}

static int xvcd_shift2_callback(struct xvcd *xvcd)
{
	int ret = 0;

#ifdef DEBUG
	static int shift_cnt = 0;

	printf("################################################\n");
	printf("[%d] total %d bits\n", shift_cnt++, xvcd->cmd_uval);
	printf("\ttms contents:\n");
	HexPrint("\t", 0, xvcd->rxbuf.ptr + xvcd->rxbuf.rpos,
			xvcd->cmd_threshold >> 1, 16);
	printf("\ttdi contents:\n");
	HexPrint("\t", 0,
			xvcd->rxbuf.ptr + xvcd->rxbuf.rpos
					+ (xvcd->cmd_threshold >> 1), xvcd->cmd_threshold >> 1, 16);
#endif

	ret = ftdi_shift_out(&xvcd->ftdi, xvcd->rxbuf.ptr + xvcd->rxbuf.rpos,
			xvcd->rxbuf.ptr + xvcd->rxbuf.rpos + (xvcd->cmd_threshold >> 1),
			xvcd->cmd_uval);
	if (ret < 0)
	{
		xvcd_err(xvcd, "ftdi-jtag shift failed (%d)\n", ret);
	}

	xvcd->rxbuf.rpos += xvcd->cmd_threshold;

	return ret;
}

static int xvcd_lock_callback(struct xvcd *xvcd)
{
	int ret = 0;
	struct sockaddr_in remote;
	socklen_t addr_len = sizeof(struct sockaddr_in);

	if (xvcd->cmd_uval == 0)
	{
		xvcd->lock = false;
	} else
	{
		xvcd->lock = true;
	}
	xvcd->rxbuf.rpos += sizeof(uint32_t);

	if (getpeername(xvcd->sock.remote_sd, (struct sockaddr*) &remote, &addr_len)
			< 0)
	{
		memset(&remote, 0, sizeof(struct sockaddr_in));
	}

	xvcd_info(xvcd, "connection %s by @%s:%d\n",
			(xvcd->lock ? "locked" : "unlocked"), inet_ntoa(remote.sin_addr),
			ntohs(remote.sin_port));

	memcpy(xvcd->txbuf.ptr, &xvcd->cmd_uval, sizeof(uint32_t));
	xvcd->txbuf.wpos = sizeof(uint32_t);

	ret = socket_write(xvcd->sock.remote_sd, xvcd->txbuf.ptr, xvcd->txbuf.wpos);
	xvcd->txbuf.wpos = 0;

	return ret;
}

static void* xvcd_thread(void *handle)
{
	struct xvcd *xvcd = (struct xvcd*) handle;
	struct timeval tv;

	while (xvcd->running)
	{
		xvcd_packer(xvcd, &tv);

		if (xvcd_select(xvcd, &tv) <= 0)
		{
			continue;
		}

		xvcd_connet(xvcd);

		xvcd_parser(xvcd);
	}

	return NULL;
}

/* Public functions ----------------------------------------------------------*/
int version(char *ver)
{
	int ret = 0;

	const char *arch = "";

	if (ver == NULL)
	{
		ret = -EINVAL;
		goto DONE;
	}

#if defined(WIN64)
	arch = "x64";
#elif defined(WIN32)
	arch = "x86";
#else
	arch = "x86/64";
#endif

	ver[0] = 0, sprintf(ver, "%s, v%s", arch, LIB_VERSION_STRING);

	DONE: return ret;
}

int ftdi_device_scan(void)
{
	int ret = 0;

	DWORD num = 0;

	if (FT_CreateDeviceInfoList(&num) == FT_OK)
	{
		ret = num;
	}

	return ret;
}

#if (defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) || defined(__WIN64__))
int ftdi_device_info(int index, char *chipname, uint32_t *id,
		char *serialnum, char *desc)
{
	int ret = 0;

	DWORD chiptype, chipid;

	if (FT_GetDeviceInfoDetail((DWORD) index, NULL, &chiptype, &chipid, //
			NULL, serialnum, desc, NULL) != FT_OK)
	{
		libdbg_err("ftdi get device info failed\n");
		ret = -EEXIST;
		goto DONE;
	}

	if (chipname)
	{
		switch (chiptype)
		{
			case FT_DEVICE_232H:
				strcpy(chipname, "ft232h");
				break;
			case FT_DEVICE_2232H:
				strcpy(chipname, "ft2232h");
				break;
			case FT_DEVICE_4232H:
				strcpy(chipname, "ft4232h");
				break;
			default:
				ret = -EPERM;
				goto DONE;
		}
	}

	if (id)
	{
		*id = (uint32_t) chipid;
	}

	DONE: return ret;
}
#else
int ftdi_device_info(int index, char *chipname, uint32_t *id, char *serialnum,
		char *desc)
{
	int ret = 0;
	libusb_context *uctx = NULL;
	libusb_device *udev = NULL, **udev_list;
	int usb_index, dev_index, intf_index = 0;
	struct libusb_device_descriptor udev_desc;
	struct libusb_config_descriptor *ucfg_desc;
	char sysfs_name[PATH_MAX];

	ret = libusb_init(&uctx);
	if (ret < 0)
	{
		libdbg_err("libusb init failed (%d)\n", ret);
		goto DONE;
	}

	ret = libusb_get_device_list(uctx, &udev_list);
	if (ret < 0)
	{
		libdbg_err("libusb get device list failed (%d)\n", ret);
		goto FREE1;
	}

	for (usb_index = 0, dev_index = 0; udev_list[usb_index]; usb_index++)
	{
		ret = libusb_get_device_descriptor(udev_list[usb_index], &udev_desc);
		if (ret < 0)
		{
			libdbg_err("libusb get device descriptor failed (%d)\n", ret);
			goto FREE2;
		}

		if ((udev_desc.idVendor == 0x403)
				&& (udev_desc.idProduct == 0x6001
						|| udev_desc.idProduct == 0x6010
						|| udev_desc.idProduct == 0x6011
						|| udev_desc.idProduct == 0x6014
						|| udev_desc.idProduct == 0x6015))
		{
			ret = libusb_get_config_descriptor(udev_list[usb_index], 0,
					&ucfg_desc);
			if (ret < 0)
			{
				libdbg_err("libusb get config descriptor failed (%d)\n", ret);
				goto FREE2;
			}

			for (intf_index = 0;
					intf_index < ucfg_desc->bNumInterfaces && !udev;
					intf_index++, dev_index++)
			{
				if (index == dev_index)
				{
					udev = udev_list[usb_index];
					break;
				}
			}

			libusb_free_config_descriptor(ucfg_desc);

			if (udev)
			{
				break;
			}
		}
	}

	if (udev == NULL)
	{
		libdbg_err("find no specific index %d device\n", index);
		ret = -EEXIST;
		goto FREE2;
	}

	if (chipname)
	{
		chipname[0] = 0;
		switch (udev_desc.idProduct)
		{
			case 0x6010:
				sprintf(chipname, "%s", "FT2232H");
				break;
			case 0x6011:
				sprintf(chipname, "%s", "FT4232H");
				break;
			case 0x6014:
				sprintf(chipname, "%s", "FT232H");
				break;
			default:
				sprintf(chipname, "%s", "unknown");
				break;
		}
	}

	if (id)
	{
		*id = (udev_desc.idVendor << 16) | udev_desc.idProduct;
	}

	if (ftdi_get_sysfs_name(sysfs_name, sizeof(sysfs_name), udev) >= 0)
	{
		ftdi_read_sysfs_prop(serialnum, sysfs_name, "serial");
		ftdi_read_sysfs_prop(desc, sysfs_name, "product");
	}

	FREE2: libusb_free_device_list(udev_list, 1);
	FREE1: libusb_exit(uctx);

	DONE: return ret;
}
#endif

void* xvcd_start(int index, int port, int max_vector_len, double freq)
{
	struct xvcd *xvcd = malloc(sizeof(struct xvcd));

	if (xvcd == NULL)
	{
		libdbg_err("xvcd handle alloc failed\n");
		goto DONE;
	}

	memset(xvcd, 0, sizeof(struct xvcd));

	xvcd->ftdi.parent = xvcd;
	xvcd->ftdi.index = index;
	xvcd->ftdi.req_tck_frequency = freq;

	xvcd->sock.parent = xvcd;
	xvcd->sock.port = port;
	if (tcp_socket_open(&xvcd->sock) < 0)
	{
		libdbg_err("tcp socket open failed\n");
		goto FREE1;
	}

	xvcd->max_vector_len = max_vector_len;

	if (pthread_attr_init(&xvcd->thread_attr) < 0)
	{
		libdbg_err("pthread attr init failed\n");
		goto FREE2;
	}

	xvcd->running = true;

	if (pthread_create(&xvcd->thread, &xvcd->thread_attr, xvcd_thread, xvcd)
			< 0)
	{
		libdbg_err("pthread create failed\n");
		goto FREE3;
	}

	goto DONE;

	FREE3: pthread_attr_destroy(&xvcd->thread_attr);

	FREE2: tcp_socket_close(&xvcd->sock);

	FREE1: free(xvcd), xvcd = NULL;

	DONE: return xvcd;
}

void xvcd_stop(void *handle)
{
	struct xvcd *xvcd = (struct xvcd*) handle;

	xvcd->running = false;
	pthread_cancel(xvcd->thread);
	pthread_join(xvcd->thread, NULL);
	pthread_attr_destroy(&xvcd->thread_attr);

	xvcd_com_close(xvcd);
	tcp_socket_close(&xvcd->sock);

	free(xvcd);
}

bool xvcd_connect_info(void *handle, char *ip, int *port, double *freq)
{
	bool ret = false;
	struct xvcd *xvcd = (struct xvcd*) handle;
	struct sockaddr_in remote;
	socklen_t addr_len = sizeof(struct sockaddr_in);

	if (!IS_ERR(xvcd->sock.remote_sd))
	{
		if (getpeername(xvcd->sock.remote_sd, (struct sockaddr*) &remote,
				&addr_len) < 0)
		{
			memset(&remote, 0, sizeof(struct sockaddr_in));
		}

		ip[0] = 0, sprintf(ip, "%s", inet_ntoa(remote.sin_addr));
		*port = ntohs(remote.sin_port);

		*freq = xvcd->ftdi.actual_tck_frequency;

		ret = true;
	}

	return ret;
}
