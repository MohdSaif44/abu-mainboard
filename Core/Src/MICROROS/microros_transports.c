//#define DMA_TRANSPORT // UART with dma
//#define IT_TRANSPORT // UART with interrupt
//#define UDP_TRANSPORT // Ethernet
#define USB_CDC_TRANSPORT


#ifdef USB_CDC_TRANSPORT

#include <rmw_microros/rmw_microros.h>

#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- USB CDC Handles ---
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;

// --- Reimplemented USB CDC callbacks ---
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len);

// Line coding: Rate: 115200bps; CharFormat: 1 Stop bit; Parity: None; Data: 8 bits
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

// --- micro-ROS Transports ---
#define USB_BUFFER_SIZE 2048
#define WRITE_TIMEOUT_MS 100U

volatile uint8_t storage_buffer[USB_BUFFER_SIZE] = {0};
volatile size_t it_head = 0;
volatile size_t it_tail = 0;
volatile bool g_write_complete = false;
bool initialized = false;

// Transmission completed callback
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
	(void) Buf;
	(void) Len;
	(void) epnum;

	g_write_complete = true;
	return USBD_OK;
}

// USB CDC requests callback
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
	switch(cmd)
	{
	case CDC_SET_LINE_CODING:
		memcpy(line_coding, pbuf, sizeof(line_coding));
		break;

	case CDC_GET_LINE_CODING:
		memcpy(pbuf, line_coding, sizeof(line_coding));
		break;

	case CDC_SEND_ENCAPSULATED_COMMAND:
	case CDC_GET_ENCAPSULATED_RESPONSE:
	case CDC_SET_COMM_FEATURE:
	case CDC_GET_COMM_FEATURE:
	case CDC_CLEAR_COMM_FEATURE:
	case CDC_SET_CONTROL_LINE_STATE:
	case CDC_SEND_BREAK:
	default:
		break;
	}

	return USBD_OK;
}

// Data received callback
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);

	// Circular buffer
	if ((it_tail + *Len) > USB_BUFFER_SIZE)
	{
		size_t first_section = USB_BUFFER_SIZE - it_tail;
		size_t second_section = *Len - first_section;

		memcpy((void*) &storage_buffer[it_tail] , Buf, first_section);
		memcpy((void*) &storage_buffer[0] , Buf, second_section);
		it_tail = second_section;
	}
	else
	{
		memcpy((void*) &storage_buffer[it_tail] , Buf, *Len);
		it_tail += *Len;
	}

	USBD_CDC_ReceivePacket(&hUsbDeviceFS);

	return (USBD_OK);
}

bool cubemx_transport_open(struct uxrCustomTransport * transport){

	if (!initialized)
	{
		// USB is initialized on generated main code: Replace default callbacks here
		USBD_Interface_fops_FS.Control = CDC_Control_FS;
		USBD_Interface_fops_FS.Receive = CDC_Receive_FS;
		USBD_Interface_fops_FS.TransmitCplt = CDC_TransmitCplt_FS;
		initialized = true;
	}

	return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
	return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
	uint8_t ret = CDC_Transmit_FS(buf, len);

	if (USBD_OK != ret)
	{
		return 0;
	}

	int64_t start = uxr_millis();
	while(!g_write_complete && (uxr_millis() -  start) < WRITE_TIMEOUT_MS)
	{
		vTaskDelay( 1 / portTICK_PERIOD_MS);
	}

	size_t writed = g_write_complete ? len : 0;
	g_write_complete = false;

	return writed;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){

	int64_t start = uxr_millis();
	size_t readed = 0;

	do
	{
		if (it_head != it_tail)
		{
			while ((it_head != it_tail) && (readed < len)){
				buf[readed] = storage_buffer[it_head];
				it_head = (it_head + 1) % USB_BUFFER_SIZE;
				readed++;
			}

			break;
		}

		vTaskDelay( 1 / portTICK_PERIOD_MS );
	} while ((uxr_millis() -  start) < timeout);

	return readed;
}

#endif

#endif


#ifdef DMA_TRANSPORT

#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

bool cubemx_transport_open(struct uxrCustomTransport * transport){
	UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
	HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
	return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
	UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
	HAL_UART_DMAStop(uart);
	return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
	UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

	HAL_StatusTypeDef ret;
	if (uart->gState == HAL_UART_STATE_READY){
		ret = HAL_UART_Transmit_DMA(uart, buf, len);
		while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY){
			osDelay(1);
		}

		return (ret == HAL_OK) ? len : 0;
	}else{
		return 0;
	}
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
	UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

	int ms_used = 0;
	do
	{
		__disable_irq();
		dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
		__enable_irq();
		ms_used++;
		osDelay(portTICK_RATE_MS);
	} while (dma_head == dma_tail && ms_used < timeout);

	size_t wrote = 0;
	while ((dma_head != dma_tail) && (wrote < len)){
		buf[wrote] = dma_buffer[dma_head];
		dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
		wrote++;
	}

	return wrote;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM

#endif

#ifdef IT_TRANSPORT

#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UART_IT_BUFFER_SIZE 2048

static uint8_t it_buffer[UART_IT_BUFFER_SIZE];
static uint8_t it_data;
static size_t it_head = 0, it_tail = 0;


bool cubemx_transport_open(struct uxrCustomTransport * transport){
	UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
	HAL_UART_Receive_IT(uart, &it_data, 1);
	return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
	UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
	HAL_UART_Abort_IT(uart);
	return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
	UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

	HAL_StatusTypeDef ret;
	if (uart->gState == HAL_UART_STATE_READY){
		ret = HAL_UART_Transmit_IT(uart, buf, len);
		while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY){
			osDelay(1);
		}

		return (ret == HAL_OK) ? len : 0;
	}else{
		return 0;
	}
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
	size_t wrote = 0;
	while ((it_head != it_tail) && (wrote < len)){
		buf[wrote] = it_buffer[it_head];
		it_head = (it_head + 1) % UART_IT_BUFFER_SIZE;
		wrote++;
	}

	return wrote;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(it_tail == UART_IT_BUFFER_SIZE)
		it_tail = 0;

	it_buffer[it_tail] = it_data;
	it_tail++;

	HAL_UART_Receive_IT(huart, &it_data, 1);
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM

#endif

#ifdef UDP_TRANSPORT

#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// --- LWIP ---
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include <lwip/sockets.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UDP_PORT        8888
static int sock_fd = -1;

bool cubemx_transport_open(struct uxrCustomTransport * transport){
	sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(UDP_PORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
	{
		return false;
	}

	return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
	if (sock_fd != -1)
	{
		closesocket(sock_fd);
		sock_fd = -1;
	}
	return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
	if (sock_fd == -1)
	{
		return 0;
	}
	const char * ip_addr = (const char*) transport->args;
	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(UDP_PORT);
	addr.sin_addr.s_addr = inet_addr(ip_addr);
	int ret = 0;
	ret = sendto(sock_fd, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	size_t writed = ret>0? ret:0;

	return writed;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){

	int ret = 0;
	//set timeout
	struct timeval tv_out;
	tv_out.tv_sec = timeout / 1000;
	tv_out.tv_usec = (timeout % 1000) * 1000;
	setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO,&tv_out, sizeof(tv_out));
	ret = recv(sock_fd, buf, len, MSG_WAITALL);
	size_t readed = ret > 0 ? ret : 0;
	return readed;
}

#endif

#endif
