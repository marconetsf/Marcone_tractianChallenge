#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nvs_flash.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

const char TAG[] = "TRACTIAN_CHALLENGE";

void setup(void)
{
    esp_err_t ret;

    /*Será necessário iniciar o acesso a flash para acessar
    o arquivo de 500kb pois a memória ram das esp32 possui apenas 512kb */

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "NVS inicializado com sucesso");
}

void loop_routine(void* pvParameter)
{
    while(1)
    {
        vTaskDelay(100/portTICK_PERIOD_MS); //delay para evitar watchdog
    }
}

void app_main(void)
{
    setup();
    xTaskCreate(loop_routine, "BLUETOOTH_ROUTINE", 60*1024, NULL, 3, NULL);
}
