#include "BleOta.h"

/// @brief The characteristic used to control the OTA process.
NimBLECharacteristic* otaControl;
/// @brief The characteristic used to receive the OTA data.
NimBLECharacteristic* otaData;

/// @brief The size of the packets received
size_t packetSize;
/// @brief The partition that will be updated
const esp_partition_t* updatePartition;
/// @brief Handle to the current update
esp_ota_handle_t updateHandle;

/// @brief Check if the device is in OTA mode
bool updating = false;

/**
 * @brief Callbacks for the control characteristic.
 */
class OtaControlCallback: public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        //TODO: add error handling
        svr_chr_ota_control_val_t command = static_cast<svr_chr_ota_control_val_t>(pCharacteristic->getValue().data()[0]);

        switch (command) {
            case SVR_CHR_OTA_CONTROL_REQUEST:
                startOta();
                break;
            case SVR_CHR_OTA_CONTROL_DONE:
                endOta();
                break;
            case SVR_CHR_OTA_CONTROL_REBOOT:
                log_i("Rebooting...");
                delay(1000);
                esp_restart();
                break;
            default:
                log_w("Unknown OTA control data received: 0x%x, count: %d, raw data: 0x%x",
                    command, pCharacteristic->getValue().length(), pCharacteristic->getValue().data()[0]);
        }
    }
};

/**
 * @brief Callbacks for the data characteristic.
 */
class OtaDataCallback: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        const uint8_t* received = (uint8_t*) malloc(pCharacteristic->getValue().length());

        memcpy((void*)received, pCharacteristic->getValue().data(), pCharacteristic->getValue().length());

        log_d("received %d bytes", pCharacteristic->getValue().length());

        // for (int i = 0; i < pCharacteristic->getValue().length(); i++) {
        //     log_v("[%d] received data: 0x%x", i, received[i]);
        // }

        if (updating) {
            log_d("received data while in update mode");
            esp_ota_write(updateHandle, received, packetSize);
        } else {
            log_d("received data outside update mode");

            packetSize = received[1] << 8 | received[0];
            log_v("packet size: %d", packetSize);
        }

        free((void*)received);
    }
};

void logPartitions() {
    log_v("Partitions:");
    for (esp_partition_iterator_t it
        = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
        it != NULL;
        it = esp_partition_next(it)) {
            const esp_partition_t* partition = esp_partition_get(it);
            log_v("  %s, subtype: %d, size: %d, address: 0x%x",
                partition->label, partition->subtype, partition->size, partition->address);
    }
}

/**
 * @brief Check if a new OTA update was installed and verify it.
 */
void checkOta() {
    logPartitions();

    const esp_partition_t *partition = esp_ota_get_running_partition();

    log_v("Running partition: %s", partition->label);

    esp_ota_img_states_t otaState;
    esp_err_t err = esp_ota_get_state_partition(partition, &otaState);
    if (err == ESP_OK) {
        log_v("OTA state: 0x%x", otaState);

        if (otaState == ESP_OTA_IMG_PENDING_VERIFY) {
            log_i("OTA update pending");
            if (runDiagnostics()) {
                log_i("OTA update successful");
                esp_ota_mark_app_valid_cancel_rollback();
            } else {
                log_e("OTA update failed");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    } else {
        log_e("Failed to get OTA state, error %s", esp_err_to_name(err));
    }

    log_v("update state: %d", updating);
}

/**
 * @brief Run diagnostics for the newly installed update.
 * @return true if the update is valid, false otherwise
 */
bool runDiagnostics() {
    log_v("running diagnostics");
    return true;
}

/**
 * @brief Configure the OTA service.
 */
void setupOta() {
    otaControl->setCallbacks(new OtaControlCallback());
    otaData->setCallbacks(new OtaDataCallback());
}

/**
 * @brief Set the device into OTA mode and send acknowledgement to client.
 */
void startOta() {
    delay(1000);
    log_i("OTA has been requested via BLE.");
    updatePartition = esp_ota_get_next_update_partition(NULL);
    log_v("installing update to partition: %s", updatePartition->label);
    esp_err_t error = esp_ota_begin(updatePartition, OTA_SIZE_UNKNOWN, &updateHandle);

    log_d("updateHandle: %d", updateHandle);

    if (error == ESP_OK) {
        updating = true;

        log_i("OTA started");
        otaControl->setValue(OTA_CONTROL_REQUEST_ACK_MASK);
    } else {
        updating = false;
        log_e("OTA failed to start, error: %s", esp_err_to_name(error));

        error = esp_ota_abort(updateHandle);
        if (error == ESP_OK) {
            log_i("OTA aborted");
        } else {
            log_e("OTA failed to abort, error: %s", esp_err_to_name(error));
        }

        otaControl->setValue(OTA_CONTROL_REQUEST_NAK_MASK);
    }
    otaControl->notify();
}

/**
 * @brief Finish the update and send confirmation.
 */
void endOta() {
    updating = false;

    log_i("OTA finished");

    esp_err_t error = esp_ota_end(updateHandle);
    if (error != ESP_OK) {
        log_e("OTA failed to end, error: %s", esp_err_to_name(error));
    } else {
        error = esp_ota_set_boot_partition(updatePartition);
        if (error != ESP_OK) {
            log_e("OTA failed to set boot partition, error: %s", esp_err_to_name(error));
        }
    }

    esp_ota_img_states_t otaState;
    esp_err_t err = esp_ota_get_state_partition(updatePartition, &otaState);
    if (err == ESP_OK) {
        log_d("update partition state is 0x%x", otaState);
    } else {
        log_e("failed to get update partition state, error %s", esp_err_to_name(err));
    }

    if (error != ESP_OK) {
        otaControl->setValue(OTA_CONTROL_DONE_NAK_MASK);
    } else {
        otaControl->setValue(OTA_CONTROL_DONE_ACK_MASK);
    }
    otaControl->notify();
    log_i("OTA done, acknoledgement sent");
}