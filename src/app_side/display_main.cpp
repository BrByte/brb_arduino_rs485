#include "../main.h"

typedef enum
{
    DISPLAY_SCREEN_INFO,
    DISPLAY_SCREEN_CONTROL,
    DISPLAY_SCREEN_TEMP,
    DISPLAY_SCREEN_CONSUME,
    DISPLAY_SCREEN_LASTITEM
} BrbDisplayScreen;

static BrbGenericCBH BrbAppDisplay_Timer;

static BrbGenericCBH BrbAppDisplay_ScreenInfo;
static BrbGenericCBH BrbAppDisplay_ScreenControl;
static BrbGenericCBH BrbAppDisplay_ScreenConsume;
static BrbGenericCBH BrbAppDisplay_ScreenTemp;

static const BrbDisplayScreenPrototype glob_display_screen_prototype[] =
    {
        DISPLAY_SCREEN_INFO,
        "INFO",
        BrbAppDisplay_ScreenInfo,

        DISPLAY_SCREEN_CONTROL,
        "CONTROL",
        BrbAppDisplay_ScreenControl,

        DISPLAY_SCREEN_TEMP,
        "TEMP",
        BrbAppDisplay_ScreenConsume,

        DISPLAY_SCREEN_CONSUME,
        "CONSUME",
        BrbAppDisplay_ScreenTemp,

        DISPLAY_SCREEN_LASTITEM,
        NULL,
        NULL,
};

DHT dht_sensor(DHT_SENSOR_PIN, DHT11);
/**********************************************************************************************************************/
int BrbAppDisplay_Setup(BrbBase *brb_base)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)&glob_display_base;

    /* Clean up base */
    memset(&glob_display_base, 0, sizeof(BrbDisplayBase));

    display_base->brb_base = brb_base;
    // display_base->screen_cur = DISPLAY_SCREEN_INFO;
    display_base->screen_cur = DISPLAY_SCREEN_CONTROL;
    // display_base->screen_cur = DISPLAY_SCREEN_TEMP;
    // display_base->screen_cur = DISPLAY_SCREEN_CONSUME;

    display_base->pin_led = TFT_LED;
    display_base->pin_dc = TFT_DC;
    display_base->pin_rst = TFT_RST;
    display_base->pin_cs = TFT_CS;
    display_base->pin_miso = TFT_MISO;
    display_base->pin_mosi = TFT_MOSI;
    display_base->pin_clk = TFT_CLK;
    display_base->screen_arr_ptr = (BrbDisplayScreenPrototype *)&glob_display_screen_prototype;
    display_base->screen_arr_cnt = sizeof(glob_display_screen_prototype) / sizeof(BrbDisplayScreenPrototype);

    // display_base->tft = (ILI9341_due *)&tft;
    display_base->tft = new ILI9341_due(display_base->pin_cs, display_base->pin_dc, display_base->pin_rst);
    // display_base->tft = new TFT_eSPI();

    BrbDisplayBase_Init(display_base);
    BrbDisplayBase_ScreenAction(display_base, -1);

    BrbTimerAdd(display_base->brb_base, 5000, 0, BrbAppDisplay_Timer, display_base);

    return 0;
}
/**********************************************************************************************************************/
/* DISPLAY */
/**********************************************************************************************************************/
static int BrbAppDisplay_Timer(void *base_ptr, void *cb_data_ptr)
{
    // BrbTimer *timer = (BrbTimer *)base_ptr;
    BrbDisplayBase *display_base = (BrbDisplayBase *)cb_data_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    BrbDisplayBase_ScreenAction(display_base, -1);

    // display_base->screen_cur++;

    int delay = 5000;

    switch (gerador_base->state.code)
    {
    case GERADOR_STATE_START_INIT:
    case GERADOR_STATE_START_DELAY:
    case GERADOR_STATE_START_CHECK:
    case GERADOR_STATE_STOP_INIT:
    case GERADOR_STATE_STOP_DELAY:
    case GERADOR_STATE_STOP_CHECK:
    {
        delay = 1500;
        break;
    }
    case GERADOR_STATE_FAILURE:
    {
        delay = 2500;
        break;
    }
    case GERADOR_STATE_RUNNING:
    case GERADOR_STATE_NONE:
    default:
    {
        delay = 5000;
        break;
    }
    }

    BrbTimerAdd(&glob_brb_base, delay, 0, BrbAppDisplay_Timer, display_base);

    return 0;
}
/**********************************************************************************************************************/
int BrbAppDisplay_ScreenInfo(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Geral"), 10, 10);
    }

    pos_x = 10;
    pos_y = 50;

    display_base->tft->fillRect(pos_x, pos_y + 15, 90, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("Bateria"), gerador_base->info.battery, 1, PSTR("VDC"));

    display_base->tft->fillRect(pos_x + 160, pos_y + 15, 90, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x + 160, pos_y, PSTR("Energia"), gerador_base->info.power_ac, 1, PSTR("VAC"));

    pos_x = 10;
    pos_y = pos_y + 65;

    BrbDisplayBase_DrawArc(display_base, gerador_base->info.gas, 0, 100, pos_x, pos_y, 65, PSTR("Tanque"), DISPLAY_ARC_RED2GREEN);

    pos_x = pos_x + 160;

    BrbDisplayBase_DrawArc(display_base, gerador_base->info.load, 0, 30, pos_x, pos_y, 65, PSTR("Amp"), DISPLAY_ARC_GREEN2RED);

    return 0;
}
/**********************************************************************************************************************/
int BrbAppDisplay_ScreenTemp(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    // BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    /* This goes to the loop LIB */
    float dht_t = dht_sensor.readTemperature();
    float dht_h = dht_sensor.readHumidity();

    // Compute heat index in Fahrenheit (the default)
    // float dht_hif = dht.computeHeatIndex(f, h);

    // Compute heat index in Celsius (isFahreheit = false)
    float dht_hic = dht_sensor.computeHeatIndex(dht_t, dht_h, false);

    int pos_x;
    int pos_y;

    int sz_w = 224;
    int sz_h = 50;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Temperatura"), 10, 10);

        // display_base->tft->fillRect(0, 50, sz_w, 15, ILI9341_LIGHTSALMON);
        // display_base->tft->fillRect(sz_w, 50, sz_w, 15, ILI9341_LIMEGREEN);

        // display_base->tft->fillRect(0, 50, 320 - sz_w, 15, ILI9341_LIGHTSALMON);
        // display_base->tft->fillRect(sz_w, 50, 320 - sz_w, 15, ILI9341_LIMEGREEN);
    }

    pos_x = 20;
    pos_y = 50;

    BrbDisplayBase_DrawBarGraph(display_base, pos_x, pos_y, 130, isnan(dht_t) ? 0 : dht_t, -50, 150);

    pos_x = 90;
    pos_y = 50;

    display_base->tft->fillRect(pos_x, pos_y + 15, 90, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("TEMP"), isnan(dht_t) ? 0 : dht_t, 1, PSTR("C"));

    // BrbDisplayBase_DrawArcSeg(display_base, isnan(dht_t) ? 0 : dht_t, 0, 130, pos_x, pos_y, 100, PSTR("Celsius"), DISPLAY_ARC_GREEN2RED, 0, 3, 5);

    pos_x = sz_w;
    pos_y = sz_h;

    display_base->tft->fillRect(pos_x, pos_y + 15, 90, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("HUMIDADE"), isnan(dht_h) ? 0 : dht_h, 1, PSTR("%"));

    display_base->tft->fillRect(pos_x, pos_y + 75, 90, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y + 60, PSTR("HEAT INDEX"), isnan(dht_hic) ? 0 : dht_hic, 1, PSTR(""));

    return 0;
}
/****************************************************************************************************/
int BrbAppDisplay_ScreenControl(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    char sub_str[16] = {0};

    int pos_x;
    int pos_y;

    const char *title_ptr = NULL;
    const char *text_ptr = NULL;

    int retry_max = GERADOR_TIMER_START_RETRY_MAX;
    int color = ILI9341_ORANGERED;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Controle"), 10, 10);
    }

    if (!display_base->flags.on_action && display_base->flags.on_select)
    {
        display_base->flags.on_action = 1;
        display_base->action_code = -1;
    }

    switch (gerador_base->state.code)
    {
    case GERADOR_STATE_START_INIT:
    {
        retry_max = GERADOR_TIMER_START_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GERADOR_TIMER_START_WAIT_MS - gerador_base->state.delta) / 1000));
        color = ILI9341_ORANGERED;
        break;
    }
    case GERADOR_STATE_START_DELAY:
    {
        retry_max = GERADOR_TIMER_START_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GERADOR_TIMER_START_DELAY_MS - gerador_base->state.delta) / 1000));
        color = ILI9341_ORANGERED;
        break;
    }
    case GERADOR_STATE_START_CHECK:
    {
        retry_max = GERADOR_TIMER_START_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GERADOR_TIMER_START_CHECK_MS - gerador_base->state.delta) / 1000));
        color = ILI9341_ORANGERED;
        break;
    }
    case GERADOR_STATE_RUNNING:
    {
        retry_max = GERADOR_TIMER_START_RETRY_MAX;
        long delta_minutes = (gerador_base->state.delta / 1000) / 60;
        snprintf(sub_str, sizeof(sub_str) - 1, "%dh%02dm", (int)(delta_minutes / 60), (int)(delta_minutes % 60));
        color = ILI9341_ORANGERED;
        break;
    }
    case GERADOR_STATE_FAILURE:
    {
        retry_max = GERADOR_TIMER_START_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((gerador_base->state.delta) / 1000));
        break;
    }
    case GERADOR_STATE_STOP_INIT:
    {
        retry_max = GERADOR_TIMER_STOP_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GERADOR_TIMER_STOP_CHECK_MS - gerador_base->state.delta) / 1000));
        color = ILI9341_SEAGREEN;
        break;
    }
    case GERADOR_STATE_STOP_DELAY:
    {
        retry_max = GERADOR_TIMER_STOP_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GERADOR_TIMER_STOP_DELAY_MS - gerador_base->state.delta) / 1000));
        color = ILI9341_SEAGREEN;
        break;
    }
    case GERADOR_STATE_STOP_CHECK:
    {
        retry_max = GERADOR_TIMER_STOP_RETRY_MAX;
        snprintf(sub_str, sizeof(sub_str) - 1, "%d s", (int)((GERADOR_TIMER_STOP_CHECK_MS - gerador_base->state.delta) / 1000));
        color = ILI9341_SEAGREEN;
        break;
    }
    case GERADOR_STATE_NONE:
    {
        retry_max = 0;
        long delta_minutes = (gerador_base->state.delta / 1000) / 60;
        snprintf(sub_str, sizeof(sub_str) - 1, "%dh%02dm", (int)(delta_minutes / 60), (int)(delta_minutes % 60));
        color = ILI9341_SEAGREEN;
        break;
    }
    default:
    {
        /**/
    }
    }

    display_base->tft->fillRect(10, 50, 300, 60, ILI9341_WHITE);

    if (display_base->flags.on_action)
    {
        title_ptr = BrbGeradorBase_GetStateAction(gerador_base);
        text_ptr = BrbGeradorBase_GetFailure(gerador_base);

        if (!text_ptr)
        {
            text_ptr = BrbGeradorBase_GetState(gerador_base);
        }
    }
    else
    {
        title_ptr = BrbGeradorBase_GetState(gerador_base);
        text_ptr = BrbGeradorBase_GetFailure(gerador_base);

        // /* First check for failures */
        // title_ptr = BrbGeradorBase_GetFailure(gerador_base);

        // if (!title_ptr)
        // {
        //     title_ptr = BrbGeradorBase_GetStateAction(gerador_base);
        // }
        // else
        // {
        //     text_ptr = BrbGeradorBase_GetState(gerador_base);
        // }

        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(DISPLAY_FONT_BOX_SUB);
        display_base->tft->setTextScale(2);
        display_base->tft->printAtPivoted(sub_str, 310, 50, gTextPivotTopRight);

        sprintf(sub_str, "%d/%d", gerador_base->state.retry, retry_max);
        display_base->tft->printAtPivoted(sub_str, 310, 75, gTextPivotTopRight);
    }

    display_base->tft->setTextColor(color, ILI9341_WHITE);
    display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(10, 50);
    display_base->tft->println((const __FlashStringHelper *)title_ptr);

    if (text_ptr)
    {
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(DISPLAY_FONT_BOX_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(10, 80);
        display_base->tft->print((const __FlashStringHelper *)text_ptr);

        // if (gerador_base->state.fail > 0)
        // {
        //     display_base->tft->print(":");
        //     display_base->tft->cursorToXY(display_base->tft->getCursorX() + 5, 80);
        //     display_base->tft->println(gerador_base->state.fail);
        // }
    }

    pos_x = 10;
    pos_y = 110;

    BrbServo *servo_bb;

    servo_bb = BrbServoGrabByPin(gerador_base->brb_base, gerador_base->pin_servo);

    display_base->tft->fillRect(pos_x, pos_y + 15, 300, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("ENERGIA"), gerador_base->info.power_ac, 1, PSTR("VAC"));
    BrbDisplayBase_BoxSub(display_base, pos_x + 110, pos_y, PSTR("FREQUENCIA"), gerador_base->info.zero_value, 1, PSTR("Hz"));
    BrbDisplayBase_BoxSub(display_base, pos_x + 210, pos_y, PSTR("SERVO"), servo_bb ? servo_bb->pos_cur : 0, 1, PSTR("o"));
    // BrbDisplayBase_BoxMax(display_base, pos_x, pos_y, PSTR("Tentativas"), gerador_base->state.retry, retry_max);

    display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
    display_base->tft->setTextScale(1);

    if (display_base->flags.on_action)
    {
        if (display_base->action_code == BRB_BTN_SELECT)
        {
            if (display_base->user_int == 1)
            {
                switch (gerador_base->state.code)
                {
                case GERADOR_STATE_NONE:
                case GERADOR_STATE_STOP_INIT:
                case GERADOR_STATE_STOP_DELAY:
                case GERADOR_STATE_STOP_CHECK:
                {
                    BrbGeradorBase_Start(gerador_base);
                    break;
                }
                case GERADOR_STATE_START_INIT:
                case GERADOR_STATE_START_DELAY:
                case GERADOR_STATE_START_CHECK:
                case GERADOR_STATE_RUNNING:
                {
                    BrbGeradorBase_Stop(gerador_base);
                    break;
                }
                case GERADOR_STATE_FAILURE:
                {
                    BrbGeradorBase_FailureConfirm(gerador_base);
                    break;
                }
                default:
                {
                    break;
                }
                }
            }

            display_base->flags.on_action = 0;
            display_base->user_int = 0;
            display_base->screen_last = -1;

            BrbDisplayBase_ScreenAction(display_base, -1);

            return 0;
        }
        else if ((display_base->action_code == BRB_BTN_NEXT) || (display_base->action_code == BRB_BTN_PREV))
        {
            display_base->user_int = !display_base->user_int;
        }

        BrbDisplayBase_DrawBtn(display_base, 20, 170, 120, 60, PSTR("SIM"), display_base->user_int ? ILI9341_ORANGERED : ILI9341_LIGHTGREY, display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);
        BrbDisplayBase_DrawBtn(display_base, 170, 170, 120, 60, PSTR("NAO"), !display_base->user_int ? ILI9341_ORANGERED : ILI9341_LIGHTGREY, !display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);
    }
    else
    {

        const char *btn_text_ptr = BrbGeradorBase_GetStateButton(gerador_base);

        BrbDisplayBase_DrawBtn(display_base, 20, 170, 280, 60, btn_text_ptr, color, ILI9341_WHITE);
    }

    return 0;
}
/****************************************************************************************************/
int BrbAppDisplay_ScreenConsume(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Consumo"), 10, 10);
    }

    if (!display_base->flags.on_action && display_base->flags.on_select)
    {
        display_base->flags.on_action = 1;
        display_base->action_code = -1;
    }

    if (display_base->flags.on_action)
    {
        if (display_base->action_code == BRB_BTN_SELECT)
        {
            if (display_base->user_int == 1)
            {
                BrbGeradorBase_HourmeterReset(gerador_base);
            }

            display_base->flags.on_action = 0;
            display_base->user_int = 0;
            display_base->screen_last = -1;

            BrbDisplayBase_ScreenAction(display_base, -1);

            return 0;
        }
        else if ((display_base->action_code == BRB_BTN_NEXT) || (display_base->action_code == BRB_BTN_PREV))
        {
            display_base->user_int = !display_base->user_int;
        }

        display_base->tft->setFont(DISPLAY_FONT_TITLE);
        display_base->tft->setTextScale(2);
        display_base->tft->printAtPivoted(PSTR("Zerar Horimetro?"), 160, 80, gTextPivotMiddleCenter);

        display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
        display_base->tft->setTextScale(1);

        BrbDisplayBase_DrawBtn(display_base, 20, 170, 120, 60, PSTR("SIM"), display_base->user_int ? ILI9341_ORANGERED : ILI9341_LIGHTGREY, display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);
        BrbDisplayBase_DrawBtn(display_base, 170, 170, 120, 60, PSTR("NAO"), !display_base->user_int ? ILI9341_ORANGERED : ILI9341_LIGHTGREY, !display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);

        return 0;
    }

    pos_x = 10;
    pos_y = 50;

    double value_dec = ((double)(gerador_base->data.hourmeter_time) / 60.0);

    BrbDisplayBase_DrawArcSeg(display_base, value_dec, 0, GERADOR_HOURMETER_MAX, pos_x, pos_y, 100, PSTR("Horas"), DISPLAY_ARC_GREEN2RED, 0, 3, 5);

    pos_x = 220;
    pos_y = 50;

    // display_base->tft->fillRect(pos_x, pos_y + 20, 70, 30, ILI9341_PURPLE);
    // BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("SOBRA"), (GERADOR_HOURMETER_MAX - value_dec), 1, PSTR("Hrs"));

    // display_base->tft->fillRect(pos_x, pos_y + 80, 70, 30, ILI9341_PURPLE);
    // BrbDisplayBase_BoxSub(display_base, pos_x + 220, pos_y, PSTR("ZERAGEM"), gerador_base->data.hourmeter_reset, 0, PSTR("x"));

    pos_x = 10;
    pos_y = 180;

    display_base->tft->fillRect(pos_x, pos_y + 20, 300, 30, ILI9341_WHITE);
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("SOBRA"), (GERADOR_HOURMETER_MAX - value_dec), 1, PSTR("Hrs"));
    BrbDisplayBase_BoxSub(display_base, pos_x + 110, pos_y, PSTR("TOTAL"), (gerador_base->data.hourmeter_time / 60), 0, PSTR("Hrs"));
    BrbDisplayBase_BoxSub(display_base, pos_x + 220, pos_y, PSTR("ZERAGEM"), gerador_base->data.hourmeter_reset, 0, PSTR("x"));

    return 0;
}
/**********************************************************************************************************************/
