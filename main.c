/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Simple user application.
 *
 * Copyright (c) 2016-2019, LEAPS. All rights reserved.
 *
 */

#include "dwm.h"
#include <stdio.h>
#include <stdlib.h>
void arrangeDevices(void);
void setupTag(void);
void setupAnchor(bool);
void processDistances(void);
uint8_t DEVICE_LOOKUP_COUNT = 0, DEVICE_UNLOCKED = 0, USER_DATA[4], USER_DATA_LEN = 4; // 1 => D1, 2 => D2, 3 => D3
bool SKIP_LOCALIZATION, KEY_ANCHOR, TO_BE_ANCHOR = 0, PRINT_DETAILED = 1, ENABLE_PRINT = 1;
uint16_t curAddr;
uint16_t acqIndex = 0;
#define ACQ_COUNT 101
#define FILTER_WIDTH 11
#define KEY0 0xB6
#define KEY1 0x20

/* Thread priority */
#ifndef THREAD_APP_PRIO
#define THREAD_APP_PRIO	20
#endif /* THREAD_APP_PRIO */

/* Thread stack size */
#ifndef THREAD_APP_STACK_SIZE
#define THREAD_APP_STACK_SIZE	(3 * 1024)
#endif /* THREAD_APP_STACK_SIZE */

#define APP_ERR_CHECK(err_code)	\
do {							\
	if ((err_code) != DWM_OK)	\
		printf("err: line(%u) code(%u)", __LINE__, (err_code));\
} while (0)						\

#define MSG_INIT	\
	"\n\n"	\
	"App   :  LUNAR\n"	\
        "Author:  Phatham Loahavilai\n"   \
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"

int64_t locX[4]; // scaled locations, by FILTER_WIDTH
int64_t locY[4];
int64_t locZ[4];
int64_t pDist[3];
uint16_t addrList[3];
int32_t rawDistances[3][ACQ_COUNT]; // a little danger but should be okay
bool addrSet = 0;
bool distancesProcessed = 0;

int cmpfunc (const void * a, const void * b)
{
        if ( *(int32_t*)a <  *(int32_t*)b ) return -1;
        if ( *(int32_t*)a >  *(int32_t*)b ) return 1;
        return 0;
}

int32_t deScale(int64_t value){
        bool isNeg = 0;
        int64_t absValue = value;
        int64_t result = 0;
        if (absValue < 0){
                isNeg = 1;
                absValue = -absValue;
        }
        result = absValue / FILTER_WIDTH;
        if (absValue % FILTER_WIDTH > FILTER_WIDTH / 2) {
                result += 1;
        }
        if (isNeg){
                result = -result;
        }
        return (int32_t) result;
}
void swapInt64(int64_t * a, int64_t * b){
        int64_t temp = *a;
        *a = *b;
        *b = temp;
}

void swapUInt16(uint16_t * a, uint16_t * b){
        uint16_t temp = *a;
        *a = *b;
        *b = temp;
}

void swapLocs(uint16_t indA, uint16_t indB){
        swapUInt16(&addrList[indA], &addrList[indB]);
        swapInt64(&locX[indA], &locX[indB]);
        swapInt64(&locY[indA], &locY[indB]);
        swapInt64(&locZ[indA], &locZ[indB]);
}

int64_t sqInt64(int64_t number){
        if (number < 0){
                printf("Domain Error! Square root of negative number\n");
                return -1;
        }
        if (number == 0) return 0;
        if (number == 1) return 1;
        //int64_t hi = number;
        //int64_t lo = 0;
        int64_t expShift;
        int64_t largeResult;
        int64_t result = 0;
        for (expShift = 31; expShift >= 0; expShift--){
                largeResult = (((int64_t)1) << expShift) | result;
                if (largeResult*largeResult <= number){
                        result = largeResult;
                }
        }
        /*if (((result + 1) & 0xFFFFFFFF )!= 0 && (result+1)*(result+1) - number < number - result*result){
                result = result + 1;
        }*/
        return result;
        /*int64_t mid = ( hi + lo ) >> 1;
        int64_t oldMid = hi + 1;
        int64_t mid2 = mid*mid;
        while( lo < hi-1 && mid2 != number && mid != oldMid) {
                if( mid2 < number ) {
                        lo = mid;
                }else{
                        hi = mid;
                }
                oldMid = mid;
                mid = ( hi + lo ) >> 1;
                mid2 = mid*mid;
        }
        return mid;*/
}

void arrangeDevices() {
        if (DEVICE_LOOKUP_COUNT == 1){ // D1
                return;
        }
        uint16_t i, j;
        uint16_t dev0, dev1;
        bool halt;
        /* Arrange D0, D1 pair for D2, D3 */
        for (i = 0, halt = 0; i < DEVICE_LOOKUP_COUNT && !halt; i++){
                for (j = i + 1; j < DEVICE_LOOKUP_COUNT; j++){
                        if (!locY[i] && !locY[j] && !locZ[i] && !locZ[j]){ // All are zeros
                                dev0 = i;
                                dev1 = j;
                                // Device 1 must be shifted (+x) from Device 0
                                // If the location xi is greater than xj, then xi is Device 1
                                if (locX[i] > locX[j]){
                                        dev0 = j;
                                        dev1 = i;
                                }
                                halt = 1;
                                break;
                        }
                }
        }
        if (dev0 == 1 && dev1 == 0){ // dev0 swapped with dev1
                swapLocs(1, 0);
                dev0 = 0;
                dev1 = 1;
        }
        if (dev0 != 0){ // dev0 swapped with somewhere else 0-2, 0-3
                swapLocs(dev0, 0);
        }
        if (dev1 != 1){ // dev1 swapped with somewhere else 1-2, 1-3
                swapLocs(dev1, 1);
        }
}
void setupAnchor(bool isInit){
        dwm_cfg_anchor_t anchorCfg;
        anchorCfg.common.ble_en = 0;
        anchorCfg.common.enc_en = 0;
        anchorCfg.common.fw_update_en = 0;
        anchorCfg.common.led_en = 1;
        anchorCfg.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
        anchorCfg.initiator = isInit;
        anchorCfg.bridge = 0;
        APP_ERR_CHECK(dwm_cfg_anchor_set(&anchorCfg));
        dwm_reset();
        while(1);
}
void setupTag(){
        dwm_cfg_tag_t tagCfg;
        tagCfg.common.ble_en = 0;
        tagCfg.common.enc_en = 0;
        tagCfg.common.fw_update_en = 0;
        tagCfg.common.led_en = 1;
        tagCfg.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
        tagCfg.loc_engine_en = 1;
        tagCfg.low_power_en = 0;
        tagCfg.stnry_en = 1;
        tagCfg.meas_mode = DWM_MEAS_MODE_TWR;
        APP_ERR_CHECK(dwm_cfg_tag_set(&tagCfg));
        printf("SET TAG\n");
        dwm_reset();
        while(1);
}
void processDistances(){
        int i, j, n, ind;
        int64_t offset;
        int64_t sumOfDelta, sumOfDelta2, variance, delta;
        int64_t minVariance;
        int64_t bestDistance; // With the scale of FILTER_WIDTH, MUST divide later
        dwm_pos_t resultPos;

        USER_DATA[1] = USER_DATA[1] | 0b1000;
        APP_ERR_CHECK(dwm_nvm_usr_data_set(USER_DATA, USER_DATA_LEN));

        for(ind = 0; ind < DEVICE_LOOKUP_COUNT; ind++) {
                minVariance = ((int64_t)1) << 62;
                bestDistance = 0;
                qsort(rawDistances[ind], ACQ_COUNT, sizeof(int32_t), cmpfunc);
                for (i = 0, n = (ACQ_COUNT - FILTER_WIDTH + 1); i < n; i++) {
                        offset = (int64_t) rawDistances[ind][i + ((ACQ_COUNT-1) / 2)];
                        sumOfDelta = 0;
                        sumOfDelta2 = 0;
                        for (j = 0; j < FILTER_WIDTH; j++){
                                delta = ((int64_t)rawDistances[ind][i + j]) - offset;
                                sumOfDelta += delta;
                                sumOfDelta2 += delta*delta;
                        }
                        variance = (sumOfDelta2*FILTER_WIDTH) - (sumOfDelta*sumOfDelta);
                        if (variance < minVariance) {
                                minVariance = variance;
                                bestDistance = (offset * FILTER_WIDTH) + sumOfDelta;
                        }
                }
                pDist[ind] = bestDistance;
        }
        if(TO_BE_ANCHOR){ // DEVICE_LOOKUP_COUNT == 3
                locX[3] = pDist[0];
                locY[3] = pDist[1];
                locZ[3] = pDist[2];
        }else if(DEVICE_LOOKUP_COUNT == 1){
                locX[1] = pDist[0];
                locY[1] = 0;
                locZ[1] = 0;
        }else if(DEVICE_LOOKUP_COUNT == 2){
                locX[2] = ((((pDist[0] + pDist[1])*(pDist[0] - pDist[1]))/locX[1]) + locX[1]) >> 1;
                locY[2] = sqInt64((pDist[0] + locX[2])*(pDist[0] - locX[2]));
                locZ[2] = 0;
        }else if(DEVICE_LOOKUP_COUNT == 3){
                locX[3] = ((((pDist[0] + pDist[1])*(pDist[0] - pDist[1]))/locX[1]) + locX[1]) >> 1;
                locY[3] = (((((pDist[0] + pDist[2])*(pDist[0] - pDist[2])) + (locX[2]*(locX[2]-(2*locX[3]))))/locY[2]) + locY[2]) >> 1;
                locZ[3] = sqInt64((pDist[0] - locY[3])*(pDist[0] + locY[3]) - (locX[3]*locX[3]));
                if (locZ[3] == 0) locZ[3] = 1;
        }
        
        resultPos.x = deScale(locX[DEVICE_LOOKUP_COUNT]);
        resultPos.y = deScale(locY[DEVICE_LOOKUP_COUNT]);
        resultPos.z = deScale(locZ[DEVICE_LOOKUP_COUNT]);
        resultPos.qf = 100;
        APP_ERR_CHECK(dwm_pos_set(&resultPos));
        printf("SET ANCHOR[%ld,%ld,%ld]\n", resultPos.x, resultPos.y, resultPos.z);
        setupAnchor(0);
}
/**
 * Event callback
 *
 * @param[in] p_evt  Pointer to event structure
 */
void on_dwm_evt(dwm_evt_t *p_evt)
{
	int i;
        int j;
	switch (p_evt->header.id) {
	/* New location data */
	case DWM_EVT_LOC_READY:
                if (ENABLE_PRINT){
                        if (PRINT_DETAILED){
                                printf("T:%lu ", dwm_systime_us_get());
                                if (!KEY_ANCHOR){
                                        if (p_evt->loc.pos_available) {
                                                printf("POS:[%ld,%ld,%ld,%u] ", p_evt->loc.pos.x,
                                                                p_evt->loc.pos.y, p_evt->loc.pos.z,
                                                                p_evt->loc.pos.qf);
                                        } else {
                                                printf("POS:N/A ");
                                        }
                                }
                                for (i = 0; i < p_evt->loc.anchors.dist.cnt; ++i) {
                                        printf("DIST%d:", i);

                                        printf("0x%04X", (unsigned int)(p_evt->loc.anchors.dist.addr[i] & 0xffff));
                                        if (i < p_evt->loc.anchors.an_pos.cnt) {
                                                printf("[%ld,%ld,%ld]",
                                                                p_evt->loc.anchors.an_pos.pos[i].x,
                                                                p_evt->loc.anchors.an_pos.pos[i].y,
                                                                p_evt->loc.anchors.an_pos.pos[i].z);
                                        }

                                        printf("=[%lu,%u] ", p_evt->loc.anchors.dist.dist[i],
                                                        p_evt->loc.anchors.dist.qf[i]);
                                }
                                printf("\n");
                        }else{
                                if (p_evt->loc.pos_available) {
                                        printf("%ld,%ld,%ld\n", p_evt->loc.pos.x, p_evt->loc.pos.y, p_evt->loc.pos.z);
                                } else {
                                        printf("None,None,None\n");
                                }
                        }
                }
                if(TO_BE_ANCHOR && acqIndex < ACQ_COUNT && p_evt->loc.pos_available && p_evt->loc.anchors.dist.cnt == 4){
                        rawDistances[0][acqIndex] = p_evt->loc.pos.x;
                        rawDistances[1][acqIndex] = p_evt->loc.pos.y;
                        rawDistances[2][acqIndex] = p_evt->loc.pos.z;
                        acqIndex++;
                        if (addrSet == 0){
                                addrSet = 1;
                                APP_ERR_CHECK(dwm_upd_rate_set(1, 1)); // Begin 10Hz acquicision
                        }
                }
                if(KEY_ANCHOR && acqIndex < ACQ_COUNT && p_evt->loc.anchors.dist.cnt == DEVICE_LOOKUP_COUNT && p_evt->loc.anchors.an_pos.cnt == DEVICE_LOOKUP_COUNT){
                        if (addrSet == 0){
                                for (i = 0; i < DEVICE_LOOKUP_COUNT; ++i) {
                                        addrList[i] = (uint16_t)(p_evt->loc.anchors.dist.addr[i] & 0xffff);
                                        locX[i] = ((int64_t)p_evt->loc.anchors.an_pos.pos[i].x) * FILTER_WIDTH;
                                        locY[i] = ((int64_t)p_evt->loc.anchors.an_pos.pos[i].y) * FILTER_WIDTH;
                                        locZ[i] = ((int64_t)p_evt->loc.anchors.an_pos.pos[i].z) * FILTER_WIDTH;
                                }
                                arrangeDevices();
                        }

                        for (i = 0; i < DEVICE_LOOKUP_COUNT; ++i) {
                                curAddr = (uint16_t)(p_evt->loc.anchors.dist.addr[i] & 0xffff);
                                for(j = 0; j < 4; j++){
                                        if(addrList[j] == curAddr) break;
                                }
                                rawDistances[j][acqIndex] = (int32_t) p_evt->loc.anchors.dist.dist[i]; // hopefully this will not overflow...
                        }

                        acqIndex++;
                        
                        if (addrSet == 0){
                                addrSet = 1;
                                APP_ERR_CHECK(dwm_upd_rate_set(1, 1)); // Begin 10Hz acquicision
                        }
                }
                if(KEY_ANCHOR && acqIndex < ACQ_COUNT && p_evt->loc.anchors.dist.cnt > DEVICE_LOOKUP_COUNT && p_evt->loc.anchors.an_pos.cnt > DEVICE_LOOKUP_COUNT){                        
                        USER_DATA[1] = USER_DATA[1] | 0b1000;
                        APP_ERR_CHECK(dwm_nvm_usr_data_set(USER_DATA, USER_DATA_LEN));
                        setupAnchor(0);
                }
                if((KEY_ANCHOR || TO_BE_ANCHOR) && acqIndex >= ACQ_COUNT && !distancesProcessed){ // full and not processed
                        distancesProcessed = 1;
                        processDistances();
                }
		break;
	default:
		break;
	}
}

/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */
void app_thread_entry(uint32_t data)
{
	dwm_cfg_t cfg;
	uint8_t i2cbyte;
	dwm_evt_t evt;
	int rv;
	// uint8_t label[DWM_LABEL_LEN_MAX];
	// uint8_t label_len = DWM_LABEL_LEN_MAX;
        uint16_t panID;
        dwm_pos_t pos;
        uint8_t SUB_MODE;

	/* Initial message */
	printf(MSG_INIT);

        APP_ERR_CHECK(dwm_nvm_usr_data_get(USER_DATA, &USER_DATA_LEN));
        
        panID = (USER_DATA[2] << 8) | USER_DATA[3];

        if ((USER_DATA[0] ^ KEY0) || ((USER_DATA[1] ^ KEY1) & 0xF0)){
                printf("NOT CONFIGURED\n");
        }else DEVICE_UNLOCKED = 1;
        
	APP_ERR_CHECK(dwm_cfg_get(&cfg));
        KEY_ANCHOR = (USER_DATA[1] & 0b0100) >> 2;
        SKIP_LOCALIZATION = (USER_DATA[1] & 0b1000) >> 3;
        TO_BE_ANCHOR = (USER_DATA[1] & 0b111) == 0b011;
        if(KEY_ANCHOR) { // Devices mode
                DEVICE_LOOKUP_COUNT = USER_DATA[1] & 0b0011;
                printf("Key Anchor D%d\n", DEVICE_LOOKUP_COUNT);
        }else{
                SUB_MODE = USER_DATA[1] & 0b0011;
                switch (SUB_MODE){
                case 0:
                        printf("Regular Tag\n");
                        break;
                case 1:
                        printf("Detailed Tag\n");
                        break;
                case 2:
                        printf("CSV Tag\n");
                        break;
                case 3:
                        printf("Auto Anchor\n");
                }
        }
        if(TO_BE_ANCHOR){
                DEVICE_LOOKUP_COUNT = 3;
        }
        if(KEY_ANCHOR || TO_BE_ANCHOR){
                // Update rate set to 5 seconds, stationary update rate set to 5 seconds
                // Start with slow acquicision to prevent jamming
                APP_ERR_CHECK(dwm_upd_rate_set(10, 10));
        }else{
                APP_ERR_CHECK(dwm_upd_rate_set(5, 10));
        }
        if (!KEY_ANCHOR && SUB_MODE == 0){ // also for to-be-anchor
                ENABLE_PRINT = 0;
        }
        if (!KEY_ANCHOR && SUB_MODE == 2){
                PRINT_DETAILED = 0;
        }
        if (cfg.common.uwb_mode != DWM_UWB_MODE_ACTIVE){
                printf("Setting up\n");
                setupTag();
        }
        if (!SKIP_LOCALIZATION && KEY_ANCHOR && DEVICE_LOOKUP_COUNT == 0){ // D0
                USER_DATA[1] = USER_DATA[1] | 0b1000;
                APP_ERR_CHECK(dwm_nvm_usr_data_set(USER_DATA, USER_DATA_LEN));
                pos.x = 0;
                pos.y = 0;
                pos.z = 0;
                pos.qf = 100;
                APP_ERR_CHECK(dwm_pos_set(&pos));
                printf("User data modified\n");
                setupAnchor(1);
        }else if(SKIP_LOCALIZATION && KEY_ANCHOR && DEVICE_LOOKUP_COUNT == 0){
                printf("Operating as Initiator Anchor\n");
        }else if (SKIP_LOCALIZATION && KEY_ANCHOR && cfg.mode != DWM_MODE_ANCHOR){
                printf("Switching to Anchor\n");
                setupAnchor(0);
        }else if (SKIP_LOCALIZATION && cfg.mode == DWM_MODE_ANCHOR){
                USER_DATA[1] = USER_DATA[1] & 0b11110111;
                APP_ERR_CHECK(dwm_nvm_usr_data_set(USER_DATA, USER_DATA_LEN));
                printf("Operating as Anchor (one-time)\n");
        }else if (!SKIP_LOCALIZATION && cfg.mode == DWM_MODE_ANCHOR){
                setupTag();
        }
        
	/* Get node configuration */
	APP_ERR_CHECK(dwm_panid_set(panID));


	/* Sensitivity for switching between stationary and normal update rate */
	APP_ERR_CHECK(dwm_stnry_cfg_set(DWM_STNRY_SENSITIVITY_NORMAL));

	/* Register event callback */
	dwm_evt_listener_register(
			DWM_EVT_LOC_READY | DWM_EVT_USR_DATA_READY |
			DWM_EVT_BH_INITIALIZED_CHANGED |
			DWM_EVT_UWBMAC_JOINED_CHANGED, NULL);

	/* Test the accelerometer */
	i2cbyte = 0x0f;
	rv = dwm_i2c_write(0x33 >> 1, &i2cbyte, 1, true);

	if (rv == DWM_OK) {
		rv = dwm_i2c_read(0x33 >> 1, &i2cbyte, 1);

		if (rv == DWM_OK) {
			printf("Accelerometer chip ID: %u\n", i2cbyte);
		} else {
			printf("i2c: read failed (%d)\n", rv);
		}
	} else {
		printf("i2c: write failed (%d)\n", rv);
	}

	/*rv = dwm_label_read(label, &label_len);

	if (rv == DWM_OK) {
		printf("LABEL(len=%d):", label_len);
		for (rv = 0; rv < label_len; ++rv) {
			printf(" %02x", label[rv]);
		}
		printf("\n");
	} else {
		printf("can't read label len=%d, error %d\n", label_len, rv);
	}*/

	while (1) {
		/* Thread loop */
		rv = dwm_evt_wait(&evt);

		if (rv != DWM_OK) {
			printf("dwm_evt_wait, error %d\n", rv);
		} else if (DEVICE_UNLOCKED && !SKIP_LOCALIZATION) {
			on_dwm_evt(&evt);
		}
	}
}

/**
 * Application entry point. Initialize application thread.
 *
 * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
 * USER THREADS CAN BE DONE IN THIS FUNCTION
 */
void dwm_user_start(void)
{
	uint8_t hndl;
	int rv;

	dwm_shell_compile();
	//Disabling ble by default as softdevice prevents debugging with breakpoints (due to priority)
	//dwm_ble_compile();
	dwm_le_compile();
	dwm_serial_spi_compile();

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}
