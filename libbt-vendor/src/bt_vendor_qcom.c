/*
 * Copyright 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 *
 *  Filename:      bt_vendor_brcm.c
 *
 *  Description:   Broadcom vendor specific library implementation
 *
 ******************************************************************************/

#define LOG_TAG "bt_vendor"

#include <utils/Log.h>
#include <cutils/properties.h>
#include <fcntl.h>
#include <termios.h>
#include "bt_vendor_qcom.h"
#include "userial_vendor_qcom.h"
#include "bt_vendor_ar3k.h"
#include "userial_vendor_ar3k.h"
#include "upio.h"
/******************************************************************************
**  Externs
******************************************************************************/
extern int hw_config(int nState);

extern int is_hw_ready();
static const tUSERIAL_CFG userial_init_cfg =
{
    (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1),
    USERIAL_BAUD_115200
};

/******************************************************************************
**  Variables
******************************************************************************/
int pFd[2] = {0,};
bt_hci_transport_device_type bt_hci_transport_device;

bt_vendor_callbacks_t *bt_vendor_cbacks = NULL;
uint8_t vnd_local_bd_addr[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#if (HW_NEED_END_WITH_HCI_RESET == TRUE)
void hw_epilog_process(void);
#endif

int btSocType=0;

/******************************************************************************
**  Local type definitions
******************************************************************************/


/******************************************************************************
**  Functions
******************************************************************************/

int get_bt_soc_type()
{
    int ret = 0;
    char bt_soc_type[PROPERTY_VALUE_MAX];

   ret = property_get("qcom.bluetooth.soc", bt_soc_type, NULL);
   ALOGI("qcom.bluetooth.soc set to %s\n", bt_soc_type);

    if (ret != 0) {
        ALOGI("qcom.bluetooth.soc set to %s\n", bt_soc_type);
        if (!strncasecmp(bt_soc_type, "ath3k", sizeof("ath3k"))) {
            ALOGI("qcom.bluetooth.soc: %s", bt_soc_type);
            return BT_SOC_ATH;
        }
        else if (!strncasecmp (bt_soc_type, "wcn2243", sizeof("wcn2243"))) {
            ALOGI("qcom.bluetooth.soc: %s", bt_soc_type);
            return BT_SOC_WCN;
        }
        else {
            ALOGI("qcom.bluetooth.soc not set, so using default.\n");
            return BT_SOC_IRIS;
        }
    }
    else {
        ALOGE("%s: Failed to get soc type", __FUNCTION__);
        ret = -1;
    }

    return ret;
}

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/

static int init(const bt_vendor_callbacks_t* p_cb, unsigned char *local_bdaddr)
{
    ALOGI("%s: ", __FUNCTION__);

    btSocType = get_bt_soc_type();
    if (btSocType < 0) {
        ALOGE("%s: Failed to detect BT SOC Type", __FUNCTION__);
        return -1;
    }

    if (p_cb == NULL)
    {
        ALOGE("init failed with no user callbacks!");
        return -1;
    }

    switch (btSocType)
    {
    case BT_SOC_IRIS:
        ALOGI("%s: No init required for SMD transport layer", __FUNCTION__);
        break;
    case BT_SOC_ATH:
    case BT_SOC_WCN:
        ALOGI("%s: Initializing UART transport layer", __FUNCTION__);
        userial_vendor_init();
        break;
    default:
        ALOGE("%s: SOC Type not supported!!!", __FUNCTION__);
        return -1;
    }

    /* store reference to user callbacks */
    bt_vendor_cbacks = (bt_vendor_callbacks_t *) p_cb;

    /* This is handed over from the stack */
    memcpy(vnd_local_bd_addr, local_bdaddr, 6);

    return 0;
}


/** Requested operations */
static int op(bt_vendor_opcode_t opcode, void *param)
{
    int retval = 0;
    int nCnt = 0;
    int nState = -1;
    int *state = (int *) param;
    char hciattach_serv[20];

    ALOGV("bt-vendor : op for %d", opcode);

    switch(opcode)
    {
    case BT_VND_OP_POWER_CTRL:
        switch (btSocType) {
        case BT_SOC_ATH:
            ALOGI("AR3002::BT_VND_OP_POWER_CTRL");
            if (*state == BT_VND_PWR_OFF) {
                ALOGI("AR3002 BT POWER-OFF");
                upio_set_bluetooth_power(UPIO_BT_POWER_OFF);
            }
            else if (*state == BT_VND_PWR_ON){
                ALOGI("AR3002 BT POWER-ON");
                upio_set_bluetooth_power(UPIO_BT_POWER_ON);
            }
            break;
        case BT_SOC_IRIS:
            ALOGI("WCN3660::BT_VND_OP_POWER_CTRL");
            nState = *(int *) param;
            retval = hw_config(nState);
            if(nState == BT_VND_PWR_ON && retval == 0 && is_hw_ready() == TRUE){
                    retval = 0;
            }
            else
               retval = -1;
            break;
        case BT_SOC_WCN:
            ALOGI("WCN2243::BT_VND_OP_POWER_CTRL");
            if (*state == BT_VND_PWR_OFF) {
                ALOGI("WCN2243 BT POWER-OFF");
                upio_set_bluetooth_power(UPIO_BT_POWER_OFF);
            }
            else if (*state == BT_VND_PWR_ON){
                ALOGI("WCN2243 BT POWER-ON");
                upio_set_bluetooth_power(UPIO_BT_POWER_ON);
            }
            break;
        } /* end of switch statement for BT SOC type*/

        break; /* break statement for BT_VND_OP_POWER_CTRL */

    case BT_VND_OP_FW_CFG:
        /* call hciattach to initalize the stack */
        if(bt_vendor_cbacks){
            ALOGI("Bluetooth Firmware and smd is initialized");
            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
        }
        else{
            ALOGE("Error : hci, smd initialization Error");
                   bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
            }
        break;

    case BT_VND_OP_SCO_CFG:
        bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS); //dummy
        break;

    case BT_VND_OP_USERIAL_OPEN:
        switch (btSocType) {
        case BT_SOC_WCN:
            ALOGI("WCN2243::BT_VND_OP_USERIAL_OPEN");
            /* Perform SOC Initialization */
            retval = hw_config(1);
            if (is_hw_ready() == TRUE && retval == 0) {
                ALOGI("%s: Initialized the BT SOC", __FUNCTION__);
                retval = 0;
            }
            else {
                ALOGE("%s: Failed to initialize the BT SOC", __FUNCTION__);
                retval = -1;
            }
        /* Deliberately fall through */
        ALOGI("%s: Deliberately falling thru to userial_vendor_open", __FUNCTION__);
        case BT_SOC_ATH:
            ALOGI("AR3002::BT_VND_OP_USERIAL_OPEN");
            int (*fd_array)[] = (int (*)[]) param;
            int fd, idx;
            fd = userial_vendor_open((tUSERIAL_CFG *) &userial_init_cfg);
            if (fd != -1) {
                for (idx=0; idx < CH_MAX; idx++)
                    (*fd_array)[idx] = fd;
                     retval = 1;
            }
            break;
        case BT_SOC_IRIS:
            ALOGI("WCN3660::BT_VND_OP_USERIAL_OPEN");
            if(bt_hci_init_transport(pFd) != -1){
                int (*fd_array)[] = (int (*) []) param;

                (*fd_array)[CH_CMD] = pFd[0];
                (*fd_array)[CH_EVT] = pFd[0];
                (*fd_array)[CH_ACL_OUT] = pFd[1];
                (*fd_array)[CH_ACL_IN] = pFd[1];
                retval = 0;
            }
            else
                retval = -1;
            break;
        }
        break;

    case BT_VND_OP_USERIAL_CLOSE:
        switch (btSocType)
        {
        case BT_SOC_WCN:
        case BT_SOC_ATH:
            ALOGI("UART transport deinit ::BT_VND_OP_USERIAL_CLOSE ");
            hw_config(0);
            userial_vendor_close();
            break;
        case BT_SOC_IRIS:
            ALOGI("SMD transport deinit ::BT_VND_OP_USERIAL_CLOSE ");
            bt_hci_deinit_transport(pFd);
            break;
        }
        break;

    case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:
        ALOGI("BT_VND_OP_GET_LPM_IDLE_TIMEOUT: Setting timeout to 1 sec");
        uint32_t *timeout_ms = (uint32_t *) param;
        *timeout_ms = 1000;
        break;

    case BT_VND_OP_LPM_SET_MODE:
        bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS); //dummy
        break;

    case BT_VND_OP_LPM_WAKE_SET_STATE:
        ALOGI("BT_VND_OP_LPM_WAKE_SET_STATE");
        uint8_t *state = (uint8_t *) param;
        uint8_t wake_assert = (*state == BT_VND_LPM_WAKE_ASSERT) ? \
            BT_VND_LPM_WAKE_ASSERT : BT_VND_LPM_WAKE_DEASSERT;

        if (wake_assert == 0)
            ALOGI("ASSERT: Waking up BT-Device");
        else if (wake_assert == 1)
            ALOGI("DEASSERT: Allowing BT-Device to Sleep");

        if(bt_vendor_cbacks){
            ALOGI("Invoking HCI H4 callback function");
           bt_vendor_cbacks->lpm_set_state_cb(wake_assert);
        }
        break;
    }

    return retval;
}

/** Closes the interface */
static void cleanup( void )
{
    ALOGI("cleanup");

    //upio_cleanup();

    bt_vendor_cbacks = NULL;
}

// Entry point of DLib
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE = {
    sizeof(bt_vendor_interface_t),
    init,
    op,
    cleanup
};
