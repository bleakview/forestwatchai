//this code is based on code from 
//https://github.com/edgeimpulse/example-esp32-cam/tree/main/Examples/Basic-Image-Classification
//Include Edge impulse inferencing library

#include <forest_fire_inferencing.h>

#include "esp_http_server.h"

#include "img_converters.h"

#include "image_util.h"

#include "esp_camera.h"

#include <WiFi.h>

// Select camera model you must get this information with your 
// board manufacturer 
#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

//this also must come from board manufacturer
#include "camera_pins.h"

//In order to put our lora board to deep sleep 
//we use this pin when its high board is high 
//when its low board is low
const int LORA_PIN = 2;
//Since this board has a single UART module 
//I use the variables to get info or to send direct data
const int PRINT_RESULT = 49;
const int PRINT_DEBUG = 50;

int printState = PRINT_RESULT;
dl_matrix3du_t *resized_matrix = NULL;
size_t out_len = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
ei_impulse_result_t result = {
    0};

void setup()
{
    //Our camera and board setup
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    Serial.println();
    pinMode(LORA_PIN, OUTPUT);
    digitalWrite(LORA_PIN, HIGH);
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

#if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
#endif

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_vflip(s, 1);      // flip it back
        s->set_brightness(s, 1); // up the brightness just a bit
        s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif
}

int raw_feature_get_data(size_t offset, size_t out_len, float *signal_ptr)
{
    size_t pixel_ix = offset * 3;
    size_t bytes_left = out_len;
    size_t out_ptr_ix = 0;

    // read byte for byte
    while (bytes_left != 0)
    {
        // grab the values and convert to r/g/b
        uint8_t r, g, b;
        r = resized_matrix->item[pixel_ix];
        g = resized_matrix->item[pixel_ix + 1];
        b = resized_matrix->item[pixel_ix + 2];

        // then convert to out_ptr format
        float pixel_f = (r << 16) + (g << 8) + b;
        signal_ptr[out_ptr_ix] = pixel_f;

        // and go to the next pixel
        out_ptr_ix++;
        pixel_ix += 3;
        bytes_left--;
    }
    return 0;
}

void classify()
{
    if (printState == PRINT_DEBUG)
    {
        Serial.println("Getting signal...");
    }
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_WIDTH;
    signal.get_data = &raw_feature_get_data;
    if (printState == PRINT_DEBUG)
    {
        Serial.println("Run classifier...");
    }
    // Feed signal to the classifier
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false /* debug */);

    if (printState == PRINT_DEBUG)
    {
        // Returned error variable "res" while data object.array in "result"
        ei_printf("run_classifier returned: %d\n", res);
    }
    if (res != 0)
        return;
    if (printState == PRINT_DEBUG)
    {
        // print the predictions
        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                  result.timing.dsp, result.timing.classification, result.timing.anomaly);
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
        {
            ei_printf("    %s: \t%f\r\n", result.classification[ix].label, result.classification[ix].value);
        }
    }
    if (printState == PRINT_RESULT)
    {
        //Enable Lora module to send message
        digitalWrite(LORA_PIN, LOW);
        delay(50);
        // I choose 0.2 but you can select your own value
        if (result.classification[0].value > 0.2)
        {
            Serial.println("0x001fire");
        }
        else
        {
            Serial.println("0x001nofire");
        }
        //Disable Lora module to send message
        digitalWrite(LORA_PIN, HIGH);
    }
    if (printState == PRINT_DEBUG)
    {
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %f\r\n", result.anomaly);
#endif
    }
}

void disableWiFi()
{
    WiFi.disconnect(true); // Disconnect from the network
    WiFi.mode(WIFI_OFF);   // Switch WiFi off
}
void loop()
{
    //Disable wifi not needed
    disableWiFi();
    while (true)
    {
        //My board use 40 MHz XTAL so I can switch to maximum frequency of 
        //240 MHz to 10MHz in roder to conserve power.
        //you can get XTAL frequency with getXtalFrequencyMhz() function
         
        // Available frequencies
        //  240, 160, 80    <<< For all XTAL types
        //  40, 20, 10      <<< For 40MHz XTAL
        //  26, 13          <<< For 26MHz XTAL
        //  24, 12          <<< For 24MHz XTAL
        // https://deepbluembedded.com/esp32-change-cpu-speed-clock-frequency/

        //Also you can get Esp32 power comsumption from 
        //https://lastminuteengineers.com/esp32-sleep-modes-power-consumption/
        setCpuFrequencyMhz(240);
        try
        {
            esp_err_t res = ESP_OK;
            camera_fb_t *fb = NULL;
            if (printState == PRINT_DEBUG)
            {
                Serial.println("Capture image");
            }
            fb = esp_camera_fb_get();
            if (!fb)
            {
                if (printState == PRINT_DEBUG)
                {
                    Serial.println("Camera capture failed");
                }
            }

            // --- Convert frame to RGB888  ---
            if (printState == PRINT_DEBUG)
            {
                Serial.println("Converting to RGB888...");
            }
            // Allocate rgb888_matrix buffer
            dl_matrix3du_t *rgb888_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
            fmt2rgb888(fb->buf, fb->len, fb->format, rgb888_matrix->item);

            // --- Resize the RGB888 frame to 96x96 in this example ---
            if (printState == PRINT_DEBUG)
            {
                Serial.println("Resizing the frame buffer...");
            }
            resized_matrix = dl_matrix3du_alloc(1, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3);
            image_resize_linear(resized_matrix->item, rgb888_matrix->item, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3, fb->width, fb->height);

            // --- Free memory ---

            dl_matrix3du_free(rgb888_matrix);
            esp_camera_fb_return(fb);

            classify();

            // --- Convert back the resized RGB888 frame to JPG to send it back to the web app ---

            dl_matrix3du_free(resized_matrix);
        }
        catch (...)
        {
            Serial.println("Exception");
        }
        //lower frequency to reduce power usage
        setCpuFrequencyMhz(10);
        //this is the delay function which determines how many seconds 
        //we will sleep before we take and classify image again.
        //The power usage chart is based on this delay value        
        delay(10000);
    }
    delay(10);
}
