/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/logging/log.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/openthread.h>
#include <openthread/coap.h>
#include <openthread/srp_client.h>
#include <openthread/ip6.h>
#include <openthread/message.h>
#include <openthread/thread.h>
#include <coap_server_client_interface.h>

#include "srp_utils.h"

LOG_MODULE_REGISTER(ot_srp_utils, CONFIG_OT_SRP_UTILS_LOG_LEVEL);


otSrpClientService lightService = {
	.mName = "_ccpeedsw._udp",
	.mPort = COAP_PORT,
	.mInstanceName = "CCPEED Switch", ///< The service instance name label (not the full name).
	.mSubTypeLabels = NULL,           ///< Array of service sub-type labels (must end with `NULL` or can be `NULL`).
    .mTxtEntries = NULL,              ///< Array of TXT entries (number of entries is given by `mNumTxtEntries`).
    .mNumTxtEntries = 0,              ///< Number of entries in the `mTxtEntries` array.
    .mPriority = 0,                   ///< The service priority.
    .mWeight = 0,                     ///< The service weight.
};



/**
 * @brief Registers us as a SRP service, so that the controllers on the network can discover us.
 * 
 * @return int standard OT error code.
 */
int srp_init()
{
	otError error;

	otInstance *ot = openthread_get_default_instance();

	otSrpClientEnableAutoStartMode(ot, NULL, NULL);
	
	// TODO use the deviceID as the hostname.
	error = otSrpClientSetHostName(ot, "ccpeedsw");
	if (error != OT_ERROR_NONE) {
		LOG_ERR("Failed to set SRP hostname. Error: %d", error);
		goto end;
	}

	error = otSrpClientEnableAutoHostAddress(ot);
	if (error != OT_ERROR_NONE) {
		LOG_ERR("Failed to set SRP addresses. Error: %d", error);
		goto end;
	}

	// TODO add some TXT records to represent the config (which group of lights it addresses, how many buttons, etc..)
	error = otSrpClientAddService(ot, &lightService);
	if (error != OT_ERROR_NONE) {
		LOG_ERR("Failed to add Service. Error: %d", error);
		goto end;
	}

end:
	return error == OT_ERROR_NONE ? 0 : 1;
}
