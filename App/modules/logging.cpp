#include "logging.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <retarget_locks.h>
#include <stdio.h>
#include <ff.h>
#include <fatfs.h>
#include <hw/exti.hpp>
#include <main.h>
#include <regions.h>
#include <util.hpp>
#include <config.hpp>
#include <Arduino.h>
#include <pins.hpp>

extern "C" uint32_t _fatfs_bss_run_addr[];
extern "C" uint32_t _logging_bss_run_addr[];
extern "C" uint32_t _shared_bss_run_addr[];

namespace modules {
namespace logging {

uint8_t buffers[config::logging_buffer_cnt][16384] ALIGN_CACHE __attribute__((section(".app")));

uint8_t retSD;    /* Return value for SD */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

struct {
	UINT br_total;
	UINT bw_total;
	float sd_speed_r;
	float sd_speed_r_now;
	float sd_speed_w;
	float sd_speed_w_now;
	TickType_t sd_latency_w_max;
	TickType_t sd_latency_w_now;
} stat;

void testRead() {
	static const char path[] = "test.bin";
	retSD = f_open(&SDFile, path, FA_READ);
	stat.br_total = 0;
	uint32_t lastTime = xTaskGetTickCount();
	uint32_t startTime = lastTime;
	UINT last_br_total = stat.br_total;
	UINT br = 0;
	do {
		if ((retSD = f_read(&SDFile, buffers[0], sizeof(buffers[0]), &br)) != FR_OK) {
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
	while (br == sizeof(buffers[0]));
	retSD = f_close(&SDFile);
}

void testWrite() {
	static const char path[] = "write.bin";
	retSD = f_unlink(path);
	retSD = f_open(&SDFile, path, FA_WRITE | FA_CREATE_NEW);
	stat.bw_total = 0;
	uint32_t lastTime = xTaskGetTickCount();
	uint32_t startTime = lastTime;
	UINT last_bw_total = stat.bw_total;
	UINT bw = 0;
	do {
		if ((retSD = f_write(&SDFile, buffers[0], sizeof(buffers[0]), &bw)) != FR_OK) {
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
	while (bw == sizeof(buffers[0]) && stat.bw_total < 104857600);
	retSD = f_close(&SDFile);
}

void testWriteThrottled() {
	static const char path[] = "writelog.bin";
	retSD = f_unlink(path);
	retSD = f_open(&SDFile, path, FA_WRITE | FA_CREATE_NEW);
	stat.bw_total = 0;
	UINT bw = 0;
	uint32_t cycleCnt = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	do {
		TickType_t timeBefore = xTaskGetTickCount();
		if ((retSD = f_write(&SDFile, buffers[0], sizeof(buffers[0]), &bw)) != FR_OK) {
			break;
		}

		stat.bw_total += bw;

		TickType_t timeAfter = xTaskGetTickCount();
		stat.sd_latency_w_now = timeAfter - timeBefore;
		stat.sd_latency_w_max = max(stat.sd_latency_w_max, stat.sd_latency_w_now);
		cycleCnt++;

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / config::logic_cycle_frequency));
	}
	while (bw == sizeof(buffers[0]) && pins::UI::SW_USER.Read());
	retSD = f_close(&SDFile);
}

static void taskLogging(void *pvParameters) {
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); //wait for parent task to finish initializing this task

	vTaskDelay(1000);
	retSD = f_mount(&SDFatFS, "0:/", 1);

	vTaskDelay(1000);
	testRead();

//	vTaskDelay(1000);
//	testWrite();

	vTaskDelay(1000);
	testWriteThrottled();

	vTaskSuspend(NULL);
}
static portSTACK_TYPE xLoggingTaskStack[ 256 ] __attribute__((aligned(256*4))) __attribute__((section(".stack")));
static constexpr MPU_REGION_REGISTERS xLoggingTaskExtendedRegions[] {
		util::mpuRegs(5, AHB1PERIPH_BASE, 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b10011111 << MPU_RASR_SRD_Pos)), //GPIOF(SW_USER) + GPIOG(SD_CARD_CD/WP)
		util::mpuRegs(5, AHB1PERIPH_BASE + 0x6000, 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11111101 << MPU_RASR_SRD_Pos)), //DMA2
		{0, 0}
};
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
		{ _fatfs_bss_run_addr, __fatfs_data_region_size__, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | CACHE_CONF(configTEX_S_C_B_SRAM) },
		{ _logging_bss_run_addr, __logging_data_region_size__, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | CACHE_CONF(configTEX_S_C_B_SRAM) },
		{ _shared_bss_run_addr, __shared_region_size__, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | CACHE_CONF(configTEX_S_C_B_TCMRAM) },
		{ (uint32_t*)(APB2PERIPH_BASE), 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b01111111 << MPU_RASR_SRD_Pos) }, //SDMMC2
		{0, 0, 0},
		{(void*)xLoggingTaskExtendedRegions}
    }
};

void Setup() {
	fatfs_Init();
	TaskHandle_t loggingTask;
	if (xTaskCreateRestricted(&xLoggingTaskDefinition, &loggingTask) != pdPASS) {
		Error_Handler();
	}

	fatfs_GrantAccess(loggingTask);
	retarget_locks_grant_access(loggingTask);
	hw::exti::GrantAccess(loggingTask);
	if (xTaskNotify(loggingTask, 0, eNoAction) != pdPASS) {
		Error_Handler();
	}
}

}
}
