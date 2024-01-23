#include "Logging.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <retarget_locks.h>
#include <stdio.h>
#include <ff.h>
#include <fatfs.h>
#include <modules/exti.hpp>

extern "C" uint32_t _fatfs_data_run_addr[];
extern "C" uint32_t _fatfs_bss_end[];
extern "C" uint32_t _app_data_run_addr[];
extern "C" uint32_t _app_bss_end[];

namespace Logging {

uint8_t retSD;    /* Return value for SD */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

uint8_t aBuf[16384] __attribute__((aligned(16)));
struct {
	UINT br_total;
	UINT bw_total;
	float sd_speed_r;
	float sd_speed_r_now;
	float sd_speed_w;
	float sd_speed_w_now;
} stat;

void testRead() {
	static const char path[] = "test.bin";
	retSD = f_open(&SDFile, path, FA_READ);
	uint32_t lastTime = xTaskGetTickCount();
	uint32_t startTime = lastTime;
	UINT last_br_total = stat.br_total;
	UINT br = 0;
	do {
		if ((retSD = f_read(&SDFile, aBuf, sizeof(aBuf), &br)) != FR_OK) {
			break;
		}
		stat.br_total += br;

		uint32_t timeNow = xTaskGetTickCount();
		if (timeNow - lastTime >= 1000) {
			stat.sd_speed_r_now = (stat.br_total - last_br_total) / (float)(1000.f * (timeNow - lastTime));
			stat.sd_speed_r = (stat.br_total) / (float)(1000.f * (timeNow - startTime));
			last_br_total = stat.br_total;
			lastTime = timeNow;
		}
	}
	while (br == sizeof(aBuf));
	retSD = f_close(&SDFile);
}

void testWrite() {
	static const char path[] = "write.bin";
	retSD = f_unlink(path);
	retSD = f_open(&SDFile, path, FA_WRITE | FA_CREATE_NEW);
	uint32_t lastTime = xTaskGetTickCount();
	uint32_t startTime = lastTime;
	UINT last_bw_total = stat.bw_total;
	UINT bw = 0;
	do {
		if ((retSD = f_write(&SDFile, aBuf, sizeof(aBuf), &bw)) != FR_OK) {
			break;
		}
		stat.bw_total += bw;

		uint32_t timeNow = xTaskGetTickCount();
		if (timeNow - lastTime >= 1000) {
			stat.sd_speed_w_now = (stat.bw_total - last_bw_total) / (float)(1000.f * (timeNow - lastTime));
			stat.sd_speed_w = (stat.bw_total) / (float)(1000.f * (timeNow - startTime));
			last_bw_total = stat.bw_total;
			lastTime = timeNow;
		}
	}
	while (bw == sizeof(aBuf) && stat.bw_total < 104857600);
	retSD = f_close(&SDFile);
}

static void taskLogging(void *pvParameters) {
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); //wait for parent task to finish initializing this task

	puts("test");

	retSD = f_mount(&SDFatFS, "0:/", 1);

	testRead();
	testWrite();

	for (;;) {
		vTaskDelay(1000);
	}
}

static portSTACK_TYPE xLoggingTaskStack[ 256 ] __attribute__((aligned(256*4))) __attribute__((section(".stack")));
static const TaskParameters_t xLoggingTaskDefinition =
{
	taskLogging,
	"Logging",
	(configSTACK_DEPTH_TYPE)sizeof(xLoggingTaskStack) / sizeof(portSTACK_TYPE),
    NULL,
    2,
	xLoggingTaskStack,
    {
        /* Base address   Length                    Parameters */
		{ (uint32_t*)(_fatfs_data_run_addr), (uint32_t)_fatfs_bss_end - (uint32_t)_fatfs_data_run_addr, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_CACHEABLE_BUFFERABLE },
		{ (uint32_t*)(_app_data_run_addr), (uint32_t)_app_bss_end - (uint32_t)_app_data_run_addr, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_CACHEABLE_BUFFERABLE },
        { (uint32_t*)(AHB1PERIPH_BASE), 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b10111111 << MPU_RASR_SRD_Pos) }, //GPIOG
		{ (uint32_t*)(AHB1PERIPH_BASE + 0x6000), 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11111101 << MPU_RASR_SRD_Pos) }, //DMA2
		{ (uint32_t*)(APB2PERIPH_BASE), 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b01111111 << MPU_RASR_SRD_Pos) }, //SDMMC2
    }
};

void Setup() {
	TaskHandle_t loggingTask;
	if (xTaskCreateRestricted(&xLoggingTaskDefinition, &loggingTask) != pdPASS) {
		Error_Handler();
	}

	fatfs_GrantAccess(loggingTask);
	retarget_locks_grant_access(loggingTask);
	modules::exti::GrantAccess(loggingTask);
	if (xTaskNotify(loggingTask, 0, eNoAction) != pdPASS) {
		Error_Handler();
	}
}

}