#include "usb_hid_host.h"
#include "esp_log.h"

static const char *USB_HID_HOST_TAG = "USB_HID_HOST";
static QueueHandle_t app_event_queue;

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event,
                              void *arg)
{
    const app_event_queue_t evt_queue = {
        .event_group = APP_EVENT_HID_HOST,
        // HID Host Device related info
        .hid_host_device.handle = hid_device_handle,
        .hid_host_device.event = event,
        .hid_host_device.arg = arg};

    if (app_event_queue)
    {
        xQueueSend(app_event_queue, &evt_queue, 0);
    }
}

QueueHandle_t create_queue(int queue_length, unsigned int item_size)
{
    return app_event_queue = xQueueCreate(queue_length, item_size);
}

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto)
{
    static hid_protocol_t prev_proto_output = -1;

    if (prev_proto_output != proto)
    {
        prev_proto_output = proto;
        printf("\r\n");
        if (proto == HID_PROTOCOL_MOUSE)
        {
            printf("Mouse\r\n");
        }
        else if (proto == HID_PROTOCOL_KEYBOARD)
        {
            printf("Keyboard\r\n");
        }
        else
        {
            printf("Generic\r\n");
        }
        fflush(stdout);
    }
}

/**
 * @brief USB HID Host Mouse Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_mouse_report_callback(const uint8_t *const data, const int length)
{
    hid_mouse_input_report_boot_t *mouse_report = (hid_mouse_input_report_boot_t *)data;

    if (length < sizeof(hid_mouse_input_report_boot_t))
    {
        return;
    }

    static int x_pos = 0;
    static int y_pos = 0;
    static int wheel_pos = 0;

    // Calculate absolute position from displacement
    x_pos += mouse_report->x_displacement;
    y_pos += mouse_report->y_displacement;
    wheel_pos += mouse_report->scrollwheel;

    hid_print_new_device_report_header(HID_PROTOCOL_MOUSE);

    printf("X: %06d\tY: %06d\tWheel: %06d\t|%c|%c|%c|\r",
           x_pos, y_pos, wheel_pos,
           (mouse_report->buttons.button1 ? 'o' : ' '),
           (mouse_report->buttons.button3 ? 'o' : ' '),
           (mouse_report->buttons.button2 ? 'o' : ' '));
    fflush(stdout);
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data, const int length)
{
    hid_print_new_device_report_header(HID_PROTOCOL_NONE);
    for (int i = 0; i < length; i++)
    {
        printf("%02X", data[i]);
    }
    putchar('\r');
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg)
{
    uint8_t data[64] = {0};
    size_t data_length = 0;
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event)
    {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hid_device_handle,
                                                                  data,
                                                                  64,
                                                                  &data_length));

        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class)
        {
            if (HID_PROTOCOL_KEYBOARD == dev_params.proto)
            {
                ESP_LOGI(USB_HID_HOST_TAG, "INSIDE hid_host_keyboard_report_callback");
                // hid_host_keyboard_report_callback(data, data_length);
            }
            else if (HID_PROTOCOL_MOUSE == dev_params.proto)
            {
                hid_host_mouse_report_callback(data, data_length);
            }
        }
        else
        {
            ESP_LOGI(USB_HID_HOST_TAG, "INSIDE hid_host_generic_report_callback");
            hid_host_generic_report_callback(data, data_length);
        }

        break;
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(USB_HID_HOST_TAG, "HID Device, protocol '%s' DISCONNECTED",
                 hid_proto_name_str[dev_params.proto]);
        ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
        break;
    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGI(USB_HID_HOST_TAG, "HID Device, protocol '%s' TRANSFER_ERROR",
                 hid_proto_name_str[dev_params.proto]);
        break;
    default:
        ESP_LOGE(USB_HID_HOST_TAG, "HID Device, protocol '%s' Unhandled event",
                 hid_proto_name_str[dev_params.proto]);
        break;
    }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, not used
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event,
                           void *arg)
{
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    /*
    hid_host_dev_info_t dev;
    hid_host_dev_info_t *info = &dev;
    hid_host_get_device_info(hid_device_handle, info);
    printf("MFG: %ls\n", info->iManufacturer);
    printf("PRODUCT: %ls\n", info->iProduct);
    printf("SERIAL: %ls\n", info->iSerialNumber);
    printf("PID: %d\n", info->PID);
    printf("VID: %d\n", info->VID);
    */

    switch (event)
    {
    case HID_HOST_DRIVER_EVENT_CONNECTED:
        ESP_LOGI(USB_HID_HOST_TAG, "HID Device, protocol '%s' CONNECTED", hid_proto_name_str[dev_params.proto]);

        const hid_host_device_config_t dev_config = {
            .callback = hid_host_interface_callback,
            .callback_arg = NULL};

        ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class)
        {
            ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT));
            if (HID_PROTOCOL_KEYBOARD == dev_params.proto)
            {
                ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle, 0, 0));
            }
        }
        ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
        break;
    default:
        break;
    }
}
