#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "../src/drivers/ssd1351.hpp"
#include "gfx_core.hpp"
#include "../fonts/Bm437_Acer_VGA_8x8.h"
#include "../fonts/Bm437_ACM_VGA_9x16.h"
#include "../fonts/Bm437_ATI_9x16.h"

using namespace espidf;
using namespace gfx;

#define LCD_WIDTH 128
#define LCD_HEIGHT 128
#define PARALLEL_LINES 16
#define DMA_CHAN SPI_DMA_CH_AUTO
#define PIN_NUM_MISO GPIO_NUM_NC
#define PIN_NUM_MOSI GPIO_NUM_6
#define PIN_NUM_CLK GPIO_NUM_7
#define PIN_NUM_CS GPIO_NUM_8
#define PIN_NUM_DC GPIO_NUM_9
#define PIN_NUM_RST GPIO_NUM_34

typedef struct
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef enum
{
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
} type_lcd_t;

spi_master spi_host(nullptr,
                    SPI3_HOST,
                    PIN_NUM_CLK,
                    PIN_NUM_MISO,
                    PIN_NUM_MOSI,
                    GPIO_NUM_NC,
                    GPIO_NUM_NC,
                    PARALLEL_LINES *LCD_WIDTH * 2 + 8,
                    DMA_CHAN);

using lcd_type = ssd1351<SPI3_HOST,
                         PIN_NUM_CS,
                         PIN_NUM_DC,
                         PIN_NUM_RST>;

lcd_type lcd;

using lcd_color = color<typename lcd_type::pixel_type>;

static const size16 bmp_size(32, 32);
using bmp_type = bitmap<typename lcd_type::pixel_type>;
using bmp_color = color<typename lcd_type::pixel_type>;

uint8_t bmp_buf[2048];
bmp_type bmp(bmp_size, bmp_buf);

template <typename Source>
void print_source(const Source &src)
{
    static const char *col_table = " .,-~;+=x!1%$O@#";
    using gsc4 = pixel<channel_traits<channel_name::L, 4>>;
    for (int y = 0; y < src.dimensions().height; ++y)
    {
        for (int x = 0; x < src.dimensions().width; ++x)
        {
            typename Source::pixel_type px;
            src.point(point16(x, y), &px);
            const auto px2 = convert<typename Source::pixel_type, gsc4>(px);
            size_t i = px2.template channel<0>();
            printf("%c", col_table[i]);
        }
        printf("\r\n");
    }
}

extern "C" void app_main()
{

    // check to make sure SPI was initialized successfully
    if (!spi_host.initialized())
    {
        printf("SPI host initialization error.\r\n");
        abort();
    }
    // // mount SPIFFS
    // esp_err_t ret;
    // esp_vfs_spiffs_conf_t conf = {};
    // conf.base_path = "/spiffs";
    // conf.format_if_mount_failed = false;
    // conf.max_files = 5;
    // conf.partition_label = "storage";
    // ret = esp_vfs_spiffs_register(&conf);
    // ESP_ERROR_CHECK(ret);
    //gfx_result rr;

lcd.clear(lcd.bounds());
    
    // draw stuff
bmp.clear(bmp.bounds());
    
    // bounding info for the face
    srect16 bounds(0,0,bmp_size.width-1,(bmp_size.height-1)/(4/3.0));
    rect16 ubounds(0,0,bounds.x2,bounds.y2);

    // draw the face
    draw::filled_ellipse(bmp,bounds,bmp_color::yellow);
    
    // draw the left eye
    srect16 eye_bounds_left(spoint16(bounds.width()/5,bounds.height()/5),ssize16(bounds.width()/5,bounds.height()/3));
    draw::filled_ellipse(bmp,eye_bounds_left,bmp_color::black);
    
    // draw the right eye
    srect16 eye_bounds_right(
        spoint16(
            bmp_size.width-eye_bounds_left.x1-eye_bounds_left.width(),
            eye_bounds_left.y1
        ),eye_bounds_left.dimensions());
    draw::filled_ellipse(bmp,eye_bounds_right,bmp_color::black);
    
    // draw the mouth
    srect16 mouth_bounds=bounds.inflate(-bounds.width()/7,-bounds.height()/8).normalize();
    // we need to clip part of the circle we'll be drawing
    srect16 mouth_clip(mouth_bounds.x1,mouth_bounds.y1+mouth_bounds.height()/(float)1.6,mouth_bounds.x2,mouth_bounds.y2);
    draw::ellipse(bmp,mouth_bounds,bmp_color::black,&mouth_clip);
    draw::bitmap(lcd,(srect16)bmp.bounds().center_horizontal(lcd.bounds()),bmp,bmp.bounds());
    const font& f = Bm437_Acer_VGA_8x8_FON;
    const char* text = "(C) 2021\r\nby HTCW";
    ssize16 text_size = f.measure_text((ssize16)lcd.dimensions(),text);
    srect16 text_rect = srect16(spoint16((lcd_type::width-text_size.width)/2,(lcd_type::height-text_size.height)/2),text_size);
    int16_t text_start = text_rect.x1;
    bool first=true;
    print_source(bmp);

    srect16 r(lcd_type::width/100.0,
                lcd_type::height/100.0,
                lcd_type::width*(lcd_type::width/100.0)-1,
                lcd_type::height*(lcd_type::height/100.0)-1);

    draw::line(lcd,srect16(lcd_type::width,r.y1,r.x1,0),lcd_color::red);

}