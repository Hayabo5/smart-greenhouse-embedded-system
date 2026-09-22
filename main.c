#include "system_sam3x.h"
#include "at91sam3x8.h"
#include <stdint.h>

/*
 * Greenhouse Controller (SAM3X / Arduino Due)
 * - LCD UI (menu + requirement views)
 * - Temperature sampling + 7-day log (fixed-size pool, no malloc)
 * - Day statistics (avg/min/max + timestamps)
 * - Light sensor + LED/servo control for light/dark simulation
 * - Temperature alarm with adjustable limits
 *
 */





#define LED_PIN         (1 << 3)


#define BUTTON_PIN      (1 << 1)


#define KP_BUF_OE_PIN   (1 << 2)


#define MAX_TEMP_RECORDS (7 * 24 * 60)  



#define ROW0 (1 << 2)   
#define ROW1 (1 << 3)   
#define ROW2 (1 << 4)   
#define ROW3 (1 << 5)   
#define ROW_MASK (ROW0 | ROW1 | ROW2 | ROW3)

#define COL0 (1 << 7)   
#define COL1 (1 << 8)   
#define COL2 (1 << 9)   
#define COL_MASK (COL0 | COL1 | COL2)


static const unsigned int row_bits[4] = { ROW0, ROW1, ROW3, ROW2 };
static const unsigned int col_bits[3] = { COL1, COL0, COL2 };


#define LCD_DB_MASK ((1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | \
                     (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9))


#define LCD_BUF_DIR (1 << 13)
#define LCD_BUF_OE  (1 << 12)
#define LCD_WR      (1 << 17)
#define LCD_RD      (1 << 16)
#define LCD_CE      (1 << 15)
#define LCD_CD      (1 << 14)
#define LCD_FS      (1 << 18)
#define LCD_RV      (1 << 19)
#define LCD_RESET   (1 << 0)


#define TEMP_PIN        (1 << 25)




#define SERVO_PWM_PIN   (1 << 17)


#define GLYPH_M  0x2D
#define GLYPH_E  0x25
#define GLYPH_N  0x2E
#define GLYPH_Y  0x39

#define GLYPH_R  0x32
#define GLYPH_Q  0x31
#define GLYPH_T  0x34

#define GLYPH_COLON  0x0E  
#define GLYPH_SPACE  0x00

#define GLYPH_F       0x26   
#define GLYPH_O       0x2F   

#define GLYPH_A  0x21
#define GLYPH_C  0x23
#define GLYPH_D  0x24
#define GLYPH_G  0x27
#define GLYPH_I  0x29
#define GLYPH_V  0x36
#define GLYPH_X  0x38
#define GLYPH_L  0x2C   
#define GLYPH_P  0x30   
#define GLYPH_S  0x33   
#define GLYPH_U  0x35   
#define GLYPH_H  0x28   
#define GLYPH_K  0x2B  
#define GLYPH_W  0x37
#define GLYPH_STAR   0x0A   
#define GLYPH_HASH   0x03   
#define GLYPH_DOT  0x0E





typedef struct {
    unsigned char sec;
    unsigned char min;
    unsigned char hour;
    unsigned char day;
    unsigned char month;
    unsigned short year;
} DateTime;




static const unsigned char days_in_month[12] =
    {31,28,31,30,31,30,31,31,30,31,30,31};

#define TEMPREC_NULL 0xFFFFu

typedef struct TempRecord {
    int16_t  temp10;   
    uint16_t next;     
} TempRecord;




static TempRecord g_temp_records_pool[MAX_TEMP_RECORDS];

static uint16_t g_free_head = TEMPREC_NULL;   
static uint16_t g_log_head  = TEMPREC_NULL;   
static uint16_t g_log_tail  = TEMPREC_NULL;   
static unsigned int g_log_count = 0;


typedef struct {
    unsigned short year;
    unsigned char  month;
    unsigned char  day;

    double minTemp;
    double maxTemp;
    double sumTemp;
    unsigned int count;

    DateTime minTime;   
    DateTime maxTime;   

    unsigned char valid;  
} DayStats;

#define MAX_DAYS_STATS 7

static DayStats g_day_stats[MAX_DAYS_STATS];
static int      g_num_days     = 0;  
static int      g_selected_day = 0;  
static int      g_log_selected_index = 0;  



/* SameDate function. */
static int SameDate(const DateTime *a, const DateTime *b)
{
    return (a->year  == b->year &&
            a->month == b->month &&
            a->day   == b->day);
}


/* DateEarlier function. */
static int DateEarlier(const DayStats *a, const DayStats *b)
{
    if (a->year  < b->year)  return 1;
    if (a->year  > b->year)  return 0;
    if (a->month < b->month) return 1;
    if (a->month > b->month) return 0;
    if (a->day   < b->day)   return 1;
    if (a->day   > b->day)   return 0;
    return 0; 
}
typedef enum {
    UI_VIEW_LIVE = 0,     
    UI_VIEW_DAY_STATS,    
    UI_VIEW_LIGHT_CTRL,   
    UI_VIEW_LOG_STATS,    
    UI_VIEW_MENU          
} UiView;

static DateTime g_log_start_time;
static int g_log_has_start_time = 0;
static int g_log_dropped_oldest = 0;   



volatile UiView g_ui_view = UI_VIEW_MENU;


typedef enum {
    UI_MODE_NORMAL = 0,
    UI_MODE_SET_CLOCK   
} UiMode;


typedef enum {
    TEMP_EDIT_MIN = 0,
    TEMP_EDIT_MAX
} TempEditField;

typedef enum {
    EDIT_HOUR = 0,
    EDIT_MIN,
    EDIT_SEC,
    EDIT_DAY,
    EDIT_MONTH,
    EDIT_YEAR
} EditField;


volatile UiMode   g_ui_mode    = UI_MODE_NORMAL;
volatile EditField g_edit_field = EDIT_HOUR;
volatile TempEditField g_temp_edit_field = TEMP_EDIT_MIN;

volatile int g_menu_needs_redraw = 1;

volatile int g_daystats_needs_redraw = 1;
volatile int g_logstats_needs_redraw = 1;
static int g_req5_needs_redraw = 1;



volatile double g_temp_limit_min = 20.0;   
volatile double g_temp_limit_max = 25.0;   


volatile int g_temp_alarm_active = 0;  
volatile int g_temp_alarm_high   = 0;  



volatile DateTime g_time = {0};


volatile unsigned int g_time_ms = 0;


volatile int g_systick_active         = 0;
volatile int g_systick_counter        = 0;
volatile int g_tc0_capture_done       = 0;
volatile int g_temp_measurement_ready = 1;
volatile double g_temperature_c       = 0.0;
volatile int    g_temp_has_valid_sample = 0;   


volatile double g_light_raw = 0.0;

volatile int g_fast_mode = 0;
volatile int g_live_show_temp = 0;   
volatile int g_logstats_count_changed = 0;





volatile unsigned long g_sunlight_seconds = 0;   
volatile unsigned long g_ledlight_seconds = 0;   
volatile unsigned long g_dark_seconds     = 0;   


volatile double g_sun_position_deg = 0.0;


volatile int g_led_on = 0;

volatile int g_shade_on = 0;



#define TARGET_LIGHT_SECONDS (16UL * 3600UL)   
#define TARGET_DARK_SECONDS  (8UL * 3600UL)    
#define TARGET_DAY_SECONDS   (24UL * 3600UL)   



#define LIGHT_THRESHOLD_SUN  1000.0



volatile int g_keypad_value = 0;


volatile int g_button_pressed = 0;




static void BusyDelay(int value);


void LED_Init(void);
void LED_Set(unsigned int on);


void Button_InitPolling(void);
void Button_Read(unsigned int *state);
void Button_InitInterrupt(void);


void Keypad_Config(void);
int  Keypad_Read(void);


unsigned char LCD_ReadStatus(void);
static void   LCD_WaitUntilReady(void);
static void   LCD_LoadBus(unsigned char value);
void          LCD_WriteCommand(unsigned char Command);
void          LCD_WriteData(unsigned char Data);


void LCD_ConfigPins(void);
void LCD_InitController(void);
void LCD_SetMemoryAddress(unsigned short addr);
void LCD_WriteGlyph(unsigned char ch);
void LCD_ClearTextMemory(void);
void LCD_WriteMenuText(const char *s);
static void DateTime_AddMinutes(DateTime *t, int minutes);



void   TempSensor_Init(void);
void   TempSensor_StartMeasurement(void);
double TempSensor_ReadCelsius(void);
void   LCD_PrintTemperature(double t);


/* Allocate a node from the pool (or reuse the oldest when full). */
static uint16_t TempLog_AllocateNodeIndex(void)
{
    uint16_t idx;
    g_log_dropped_oldest = 0;

    if (g_free_head != TEMPREC_NULL)
    {
        idx = g_free_head;
        g_free_head = g_temp_records_pool[idx].next;
    }
    else
    {
        /* Pool exhausted: reuse the oldest logged node (FIFO overwrite).
           This keeps memory bounded without malloc(). */
        idx = g_log_head;
        g_log_head = g_temp_records_pool[idx].next;

        g_log_dropped_oldest = 1;

        if (g_log_head == TEMPREC_NULL)
        {
            g_log_tail = TEMPREC_NULL;
        }
    }

    g_temp_records_pool[idx].next = TEMPREC_NULL;
    return idx;
}





/* Append one temperature sample to the log (one sample per simulated minute). */
void TempLog_AddSample(double temperature)
{
    /* Add a new temperature sample to the log (one entry per simulated minute).
       Storage is a fixed-size pool (no malloc). If full, we overwrite the oldest entry.
       We also maintain g_log_start_time so index->timestamp works even after overwrite. */

    uint16_t idx = TempLog_AllocateNodeIndex();
    
if (!g_log_has_start_time)
{
    g_log_start_time = g_time;
    g_log_start_time.sec = 0;  
    g_log_has_start_time = 1;
}


if (g_log_dropped_oldest)
{
    DateTime_AddMinutes(&g_log_start_time, 1);
}

    TempRecord *node = &g_temp_records_pool[idx];

    
    int temp10 = (int)(temperature * 10.0 + (temperature >= 0 ? 0.5 : -0.5));
    if (temp10 >  32767) temp10 =  32767;
    if (temp10 < -32768) temp10 = -32768;

    node->temp10 = (int16_t)temp10;


    node->next = TEMPREC_NULL;

    
    if (g_log_head == TEMPREC_NULL)
    {
        g_log_head = idx;
        g_log_tail = idx;
        g_log_count = 1;
    }
    else
    {
        g_temp_records_pool[g_log_tail].next = idx;
        g_log_tail = idx;

        if (g_log_count < MAX_TEMP_RECORDS)
            g_log_count++;
        
    }
   
g_logstats_count_changed = 1;


}



/* Compute min/max/average over the full log. */
int TempLog_GetStats(double *outMin, double *outMax, double *outAvg)
{
    if (g_log_head == TEMPREC_NULL || g_log_count == 0)
        return 0;

    uint16_t cur = g_log_head;

    
    double t0 = ((double)g_temp_records_pool[cur].temp10) / 10.0;
    double minT = t0;
    double maxT = t0;
    double sumT = 0.0;
    unsigned int n = 0;

    while (cur != TEMPREC_NULL)
    {
        double t = ((double)g_temp_records_pool[cur].temp10) / 10.0;

        if (t < minT) minT = t;
        if (t > maxT) maxT = t;

        sumT += t;
        n++;

        cur = g_temp_records_pool[cur].next;   
    }

    *outMin = minT;
    *outMax = maxT;
    *outAvg = sumT / (double)n;
    return (int)n;
}

/* Initialize the fixed-size node pool and free list for the temperature log. */
void TempLog_Init(void)
{
    
    g_free_head = 0;
    for (uint16_t i = 0; i < (uint16_t)(MAX_TEMP_RECORDS - 1); i++)
    {
        g_temp_records_pool[i].next = (uint16_t)(i + 1);
    }
    g_temp_records_pool[MAX_TEMP_RECORDS - 1].next = TEMPREC_NULL;

    g_log_head  = TEMPREC_NULL;
    g_log_tail  = TEMPREC_NULL;
    g_log_count = 0;
}


/* Reset all day-statistics slots (max 7 days). */
void DayStats_Init(void)
{
    for (int i = 0; i < MAX_DAYS_STATS; i++) {
        g_day_stats[i].valid = 0;
    }
    g_num_days     = 0;
    g_selected_day = 0;
}
/* Initialize temperature limits and alarm flags (Req.5). */
void TempAlarm_Init(void)
{
    g_temp_limit_min    = 20.0;   
    g_temp_limit_max    = 25.0;   
    g_temp_alarm_active = 0;
    g_temp_alarm_high   = 0;
    g_temp_edit_field   = TEMP_EDIT_MIN;
}

/* Get DayStats for a date; create or overwrite the oldest if full. */
static DayStats* DayStats_FindOrCreate(const DateTime *t)
{
    
    for (int i = 0; i < MAX_DAYS_STATS; i++) {
        if (g_day_stats[i].valid &&
            g_day_stats[i].year  == t->year &&
            g_day_stats[i].month == t->month &&
            g_day_stats[i].day   == t->day) {
            return &g_day_stats[i];
        }
    }

    
    for (int i = 0; i < MAX_DAYS_STATS; i++) {
        if (!g_day_stats[i].valid) {
            DayStats *ds = &g_day_stats[i];
            ds->year  = t->year;
            ds->month = t->month;
            ds->day   = t->day;
            ds->minTemp = 0.0;
            ds->maxTemp = 0.0;
            ds->sumTemp = 0.0;
            ds->count   = 0;
            ds->minTime = *t;
            ds->maxTime = *t;
            ds->valid   = 1;
            g_num_days++;
            return ds;
        }
    }

    
    int oldest_index = 0;
    for (int i = 1; i < MAX_DAYS_STATS; i++) {
        if (g_day_stats[i].valid &&
            DateEarlier(&g_day_stats[i], &g_day_stats[oldest_index])) {
            oldest_index = i;
        }
    }

    DayStats *ds = &g_day_stats[oldest_index];
    ds->year  = t->year;
    ds->month = t->month;
    ds->day   = t->day;
    ds->minTemp = 0.0;
    ds->maxTemp = 0.0;
    ds->sumTemp = 0.0;
    ds->count   = 0;
    ds->minTime = *t;
    ds->maxTime = *t;
    ds->valid   = 1;

    return ds;
}
/* Return pointer to the log entry at a given index (0 = oldest). */
TempRecord* TempLog_GetByIndex(int index)
{
    if (index < 0 || index >= (int)g_log_count)
        return 0;

    uint16_t cur = g_log_head;
    int i = 0;

    while (cur != TEMPREC_NULL && i < index)
    {
        cur = g_temp_records_pool[cur].next;
        i++;
    }

    if (cur == TEMPREC_NULL) return 0;
    return &g_temp_records_pool[cur];
}

/* Compute timestamp for a given log index (start time + index minutes). */
static void TempLog_GetTimestampByIndex(int index, DateTime *out)
{
    *out = g_log_start_time;
    DateTime_AddMinutes(out, index);
}



/* Update day min/max/avg counters when a new sample is logged. */
void DayStats_UpdateWithSample(const DateTime *t, double temp)
{
    DayStats *ds = DayStats_FindOrCreate(t);
    if (!ds) return;

    if (ds->count == 0) {
        ds->minTemp = temp;
        ds->maxTemp = temp;
        ds->sumTemp = temp;
        ds->count   = 1;
        ds->minTime = *t;
        ds->maxTime = *t;
    } else {
        ds->sumTemp += temp;
        ds->count++;

        if (temp < ds->minTemp) {
            ds->minTemp = temp;
            ds->minTime = *t;
        }
        if (temp > ds->maxTemp) {
            ds->maxTemp = temp;
            ds->maxTime = *t;
        }
    }
    
    if (g_ui_view == UI_VIEW_DAY_STATS)
    {
        g_daystats_needs_redraw = 1;
    }
}
 

/* Update alarm state based on current temperature (requires a valid sample). */
void TempAlarm_Update(double currentTemp)
{ 
   if (!g_temp_has_valid_sample)
        return;
   
    if (!g_temp_alarm_active)
    {
        
        if (currentTemp > g_temp_limit_max)
        {
            g_temp_alarm_active = 1;
            g_temp_alarm_high   = 1;   
        }

/**
 * if()
 * Short: Driver / helper function used by the greenhouse application.
 */
        else if (currentTemp < g_temp_limit_min)
        {
            g_temp_alarm_active = 1;
            g_temp_alarm_high   = 0;   
        }
    }
    else
    {
        
        if (currentTemp > g_temp_limit_max)
        {
            g_temp_alarm_high = 1;
        }

/**
 * if()
 * Short: Driver / helper function used by the greenhouse application.
 */
        else if (currentTemp < g_temp_limit_min)
        {
            g_temp_alarm_high = 0;
        }
    }
}



void   LightSensor_Init(void);
double LightSensor_Sample(void);
void   LCD_PrintLightValue(int value);


void Servo_InitPwm(void);
void Servo_UpdateFromKey(int key);



#define SERVO_MIN_DUTY 1837      
#define SERVO_MAX_DUTY 5197      

/* Set servo angle by mapping 0..180° into PWM duty cycle limits. */
void Servo_SetAngleDegrees(double angle)
{
    if (angle < 0.0)   angle = 0.0;
    if (angle > 180.0) angle = 180.0;

    double ratio = angle / 180.0;
    unsigned int duty = SERVO_MIN_DUTY +
                        (unsigned int)((SERVO_MAX_DUTY - SERVO_MIN_DUTY) * ratio + 0.5);

    *AT91C_PWMC_CH1_CDTYUPDR = duty;  /* Update PWM duty (takes effect next period) */
}



void Init_SysTick(void);
void Clock_SetDefault(void);
void Clock_IncrementField(EditField field, int delta);
void UI_HandleKeypadForClock(int key);
void Clock_AdvanceSeconds(unsigned int seconds);



void UI_PrintTime(void);
void UI_PrintDate(void);
static void LCD_ClearRow(unsigned short addr);
void UI_UpdateScreen(double temp, int light);
void UI_PrintDayStats(void);
void UI_PrintLightStatus(void);
void UI_PrintMenu(void);
void UI_PrintLogStats(void);
void UI_ClearBelowHeader(void);
void UI_PrintDayStatsTable(void);


void TempAlarm_Init(void);
void TempAlarm_Update(double currentTemp);
void UI_HandleKeypadForTempLimits(int key);
void UI_PrintTempAlarmMessage(void);
/* Keep min/max limits within allowed range and ensure min < max. */
static void TempLimits_Clamp(void)
{
    
    if (g_temp_limit_min < 0.0)  g_temp_limit_min = 0.0;
    if (g_temp_limit_max > 99.9) g_temp_limit_max = 99.9;

    
    if (g_temp_limit_min >= g_temp_limit_max)
    {
        g_temp_limit_max = g_temp_limit_min + 0.5;  
        if (g_temp_limit_max > 99.9) {
            g_temp_limit_max = 99.9;
            g_temp_limit_min = g_temp_limit_max - 0.5;
        }
    }
}

/* Handle keypad input for changing min/max limits and acknowledging alarm. */
void UI_HandleKeypadForTempLimits(int key)
{
    if (key == 0)
        return;

    
    if (!(g_ui_view == UI_VIEW_LIVE &&
          g_ui_mode == UI_MODE_NORMAL &&
          g_live_show_temp))
    {
        
        if (key == 9 && g_temp_alarm_active)
            g_temp_alarm_active = 0;
        return;
    }

    
    if (key == 11)
    {
        g_temp_edit_field = (g_temp_edit_field == TEMP_EDIT_MIN)
                                ? TEMP_EDIT_MAX
                                : TEMP_EDIT_MIN;
        return;
    }

    
    if (key == 7 || key == 8)
    {
        double step = 0.5;
        double delta = (key == 8) ? step : -step;

        if (g_temp_edit_field == TEMP_EDIT_MIN)
            g_temp_limit_min += delta;
        else
            g_temp_limit_max += delta;

        TempLimits_Clamp();
        return;
    }

    
    if (key == 9 && g_temp_alarm_active)
    {
        g_temp_alarm_active = 0;
        return;
    }
    if (key == 11)
{
    g_temp_edit_field = (g_temp_edit_field == TEMP_EDIT_MIN) ? TEMP_EDIT_MAX : TEMP_EDIT_MIN;
    g_req5_needs_redraw = 1;  
    return;
}

}







void SysTick_Handler(void);
void TC0_Handler(void);
void PIOD_Handler(void);




   /* System entry: initialize peripherals, then run the main loop (sampling + UI + keypad/menu). */
   int main(void)
{
    SystemInit();

    
    LED_Init();
    Button_InitInterrupt();       

    Keypad_Config();
    LCD_ConfigPins();
    LCD_InitController();
    LCD_ClearTextMemory();

    TempSensor_Init();
    LightSensor_Init();
    Servo_InitPwm();
    Servo_SetAngleDegrees(0);



    Clock_SetDefault();
    Init_SysTick();               

  
    TempLog_Init();               
    DayStats_Init();              
    TempAlarm_Init();             

    

   

    while (1)
    {
    /* ------------------------------------------------------------
       Main loop overview:
       - Kick off a new temperature conversion when ready
       - Read the last completed temperature sample (TC0 capture)
       - Sample light via ADC
       - Read keypad with simple "press -> wait release" latching
       - Handle menu navigation / view logic
       - Update the LCD depending on the current UI view
       ------------------------------------------------------------ */

        
        if (g_temp_measurement_ready)
        {
            g_temp_measurement_ready = 0;
            TempSensor_StartMeasurement();
        }

        
        double currentTemp = TempSensor_ReadCelsius();
        g_temperature_c = currentTemp;
        
        
        TempAlarm_Update(currentTemp);


        
        g_light_raw = LightSensor_Sample();
        int light_int = (int)g_light_raw;

       g_keypad_value = Keypad_Read(); 
        /* Keypad latch: accept a key once, then wait for release.
           This avoids repeated triggers from holding a button down. */
        static int last_key = 0;

int raw = g_keypad_value;


if (raw != 0 && last_key == 0)
{
    g_keypad_value = raw;

    
    do {
        BusyDelay(3000);           
        raw = Keypad_Read();
    } while (raw != 0);
}
else
{
    
    g_keypad_value = 0;
}

last_key = raw;


        
            

    
    if (g_ui_mode == UI_MODE_NORMAL && g_keypad_value == 12)
    {
        g_ui_view = UI_VIEW_MENU;
        g_menu_needs_redraw = 1;
    }

    
    if (g_ui_mode == UI_MODE_NORMAL &&
        g_keypad_value == 10 &&
        g_ui_view != UI_VIEW_MENU)
    {
        g_ui_view = UI_VIEW_MENU;
        g_menu_needs_redraw = 1;
    }

   if (g_ui_view == UI_VIEW_MENU && g_ui_mode == UI_MODE_NORMAL)
{
    switch (g_keypad_value)
    {
 case 1: 
      UI_ClearBelowHeader();          
    g_ui_view    = UI_VIEW_LIVE;
    g_ui_mode    = UI_MODE_NORMAL;  
    g_live_show_temp = 0;           
    break;


     case 2: 
        UI_ClearBelowHeader();                 
        g_ui_view = UI_VIEW_LOG_STATS;


        
        if (g_log_count > 0)
            g_log_selected_index = (int)g_log_count - 1;
        else
            g_log_selected_index = 0;
        
         g_logstats_needs_redraw = 1;  
        break;


        case 3: 
        
        g_ui_view = UI_VIEW_DAY_STATS;
        g_daystats_needs_redraw = 1;   
        break;


    case 4: 
        UI_ClearBelowHeader();
        g_ui_view = UI_VIEW_LIGHT_CTRL;
        break;

    case 5: 
         UI_ClearBelowHeader();
            g_ui_view = UI_VIEW_LIVE;
            g_live_show_temp = 1;           

           g_req5_needs_redraw = 1;


            
            break;
            
            case 6:  
    g_fast_mode = !g_fast_mode;
    g_menu_needs_redraw = 1;  
    break;

            
    }
}


           
    if (g_ui_view != UI_VIEW_MENU)
    {
        UI_HandleKeypadForClock(g_keypad_value);
        UI_HandleKeypadForTempLimits(g_keypad_value);
    }

        
TempAlarm_Update(currentTemp);


        
if (g_ui_mode == UI_MODE_NORMAL)
{
  

    
    if (g_ui_view == UI_VIEW_DAY_STATS) {
        if (g_keypad_value == 4 && g_selected_day > 0) {
            g_selected_day--;
        } else if (g_keypad_value == 6 && g_selected_day < (g_num_days - 1)) {
            g_selected_day++;
        }
    }
}
 if (g_ui_mode == UI_MODE_NORMAL)
{
    if (g_ui_view == UI_VIEW_DAY_STATS) {
        if (g_keypad_value == 4 && g_selected_day > 0) {
            g_selected_day--;
        } else if (g_keypad_value == 6 && g_selected_day < (g_num_days - 1)) {
            g_selected_day++;
        }
    }
 else if (g_ui_view == UI_VIEW_LOG_STATS) {

    if (g_log_count > 0) {

        if (g_keypad_value == 4 && g_log_selected_index > 0) {
            g_log_selected_index--;
            
        }
        else if (g_keypad_value == 6 &&
                 g_log_selected_index < (int)g_log_count - 1) {
            g_log_selected_index++;
            
        }

      
    }
}
  
}




        
        UI_UpdateScreen(currentTemp, light_int);

        BusyDelay(30000); 
    }
}




/* Simple busy-wait delay (used for short timing gaps / debouncing). */
static void BusyDelay(int value)
{
    for (volatile int i = 0; i < value; i++)
    {
        __NOP();
    }
}



/* Enable PIOD clock and configure the onboard LED pin as output. */
void LED_Init(void)
{
    *AT91C_PMC_PCER |= (1 << 14);     /* Enable peripheral clock in PMC */
    *AT91C_PIOD_PER |= LED_PIN;  /* Enable PIO control for these pins */
    *AT91C_PIOD_OER |= LED_PIN;  /* Configure pins as outputs */
    *AT91C_PIOD_CODR = LED_PIN;       /* Configure pins as inputs */
}

/* Turn the LED on/off by writing to PIOD set/clear registers. */
void LED_Set(unsigned int on)
{
    if (on)
        *AT91C_PIOD_SODR = LED_PIN;  /* Configure pins as inputs */
    else
        *AT91C_PIOD_CODR = LED_PIN;  /* Configure pins as inputs */
}



/* Configure hardware button pin as input with pull-up (polling version). */
void Button_InitPolling(void)
{
    *AT91C_PMC_PCER   |= (1 << 14);  /* Enable peripheral clock in PMC */
    *AT91C_PIOD_PER   |= BUTTON_PIN;  /* Enable PIO control for these pins */
    *AT91C_PIOD_PPUER |= BUTTON_PIN;  /* Enable pull-up resistors */
    *AT91C_PIOD_ODR   |= BUTTON_PIN;  /* Configure pins as inputs */
}

/* Read current button state (active-low). */
void Button_Read(unsigned int *state)
{
    if (((*AT91C_PIOD_PDSR) & BUTTON_PIN) == 0)  /* Read pin levels */
        *state = 1;   
    else
        *state = 0;
}


/* Configure hardware button pin to generate interrupts on PIOD. */
void Button_InitInterrupt(void)
{
    *AT91C_PMC_PCER |= (1 << 14);  /* Enable peripheral clock in PMC */

    *AT91C_PIOD_PER   |= BUTTON_PIN;  /* Enable PIO control for these pins */
    *AT91C_PIOD_PPUER |= BUTTON_PIN;  /* Enable pull-up resistors */
    *AT91C_PIOD_ODR   |= BUTTON_PIN;  /* Configure pins as inputs */

    *AT91C_PIOD_IER = BUTTON_PIN;     /* Enable PIO interrupt on these pins */

    NVIC_ClearPendingIRQ(PIOD_IRQn);
    NVIC_SetPriority(PIOD_IRQn, 1);
    NVIC_EnableIRQ(PIOD_IRQn);
}



/* Configure keypad row/column pins and enable the 74HC245 buffer (OE active-low). */
void Keypad_Config(void)
{
    *AT91C_PMC_PCER |= (1 << 13) | (1 << 14);   /* Enable peripheral clock in PMC */

    
    *AT91C_PIOD_PER  |= KP_BUF_OE_PIN;  /* Enable PIO control for these pins */
    *AT91C_PIOD_OER  |= KP_BUF_OE_PIN;  /* Configure pins as outputs */
    *AT91C_PIOD_CODR  = KP_BUF_OE_PIN;     /* Configure pins as inputs */

    
    *AT91C_PIOC_PER   |= ROW_MASK;  /* Enable PIO control for these pins */
    *AT91C_PIOC_ODR   |= ROW_MASK;  /* Configure pins as inputs */
    *AT91C_PIOC_PPUER |= ROW_MASK;  /* Enable pull-up resistors */

    
    *AT91C_PIOC_PER |= COL_MASK;  /* Enable PIO control for these pins */
    *AT91C_PIOC_ODR |= COL_MASK;  /* Configure pins as inputs */
}

/* Scan the 3x4 keypad matrix and return key number (1..12), or 0 if none. */
int Keypad_Read(void)
{
    int key_value = 0;

    
    *AT91C_PIOC_SODR = LCD_BUF_OE;  /* Configure pins as inputs */

    
    *AT91C_PIOD_CODR = KP_BUF_OE_PIN;  /* Configure pins as inputs */

    
    *AT91C_PIOC_OER |= COL_MASK;  /* Configure pins as outputs */
    *AT91C_PIOC_SODR = COL_MASK;  /* Configure pins as inputs */

    for (int col_index = 0; col_index < 3; col_index++)
    {
        /* Scan one column at a time:
           - Drive this column low
           - Read which row line goes low (pressed key) */
        unsigned int rows_snapshot;

        *AT91C_PIOC_SODR = COL_MASK;  /* Configure pins as inputs */
        *AT91C_PIOC_CODR = col_bits[col_index];  /* Configure pins as inputs */

        BusyDelay(3000);

        for (int row_index = 0; row_index < 4; row_index++)
        {
            rows_snapshot = *AT91C_PIOC_PDSR;  /* Read pin levels */

            if ((rows_snapshot & row_bits[row_index]) == 0)
            {
                key_value = row_index * 3 + col_index + 1;
            }
        }

        *AT91C_PIOC_SODR = col_bits[col_index];  /* Configure pins as inputs */
    }

    *AT91C_PIOC_ODR |= COL_MASK;    /* Configure pins as inputs */

    return key_value;
}



/* Read LCD controller status via the shared data bus (used for busy flag). */
unsigned char LCD_ReadStatus(void)
{
    unsigned char Temp;

    *AT91C_PIOD_SODR = KP_BUF_OE_PIN;       /* Configure pins as inputs */

    *AT91C_PIOC_PER |= LCD_DB_MASK;  /* Enable PIO control for these pins */
    *AT91C_PIOC_ODR |= LCD_DB_MASK;  /* Configure pins as inputs */

    *AT91C_PIOC_SODR = LCD_BUF_DIR;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_BUF_OE;  /* Configure pins as inputs */

    *AT91C_PIOC_SODR = LCD_CD;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_CE;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_RD;  /* Configure pins as inputs */

    BusyDelay(2000);

    Temp = (unsigned char)((*AT91C_PIOC_PDSR & LCD_DB_MASK) >> 2);  /* Read pin levels */

    *AT91C_PIOC_SODR = LCD_CE;  /* Configure pins as inputs */
    *AT91C_PIOC_SODR = LCD_RD;  /* Configure pins as inputs */
    *AT91C_PIOC_SODR = LCD_BUF_OE;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_BUF_DIR;  /* Configure pins as inputs */

    return Temp;
}

/* Wait until LCD controller reports ready (busy flags cleared). */
static void LCD_WaitUntilReady(void)
{
    while ((LCD_ReadStatus() & 0x03) != 0x03)
    {
        
    }
}


/**
 * LCD_LoadBus()
 * Short: Driver / helper function used by the greenhouse application.
 */
static void LCD_LoadBus(unsigned char value)
{
    *AT91C_PIOC_CODR = LCD_DB_MASK;  /* Configure pins as inputs */
    *AT91C_PIOC_SODR = ((unsigned int)value << 2) & LCD_DB_MASK;  /* Configure pins as inputs */
}

/* Write a command byte to the LCD controller. */
void LCD_WriteCommand(unsigned char Command)
{
    LCD_WaitUntilReady();
    LCD_LoadBus(Command);

    *AT91C_PIOC_CODR = LCD_BUF_DIR;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_BUF_OE;  /* Configure pins as inputs */
    *AT91C_PIOC_OER |= LCD_DB_MASK;  /* Configure pins as outputs */

    *AT91C_PIOC_SODR = LCD_CD;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_CE;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_WR;  /* Configure pins as inputs */

    BusyDelay(2000);

    *AT91C_PIOC_SODR = LCD_CE;  /* Configure pins as inputs */
    *AT91C_PIOC_SODR = LCD_WR;  /* Configure pins as inputs */
    *AT91C_PIOC_SODR = LCD_BUF_OE;  /* Configure pins as inputs */
    *AT91C_PIOC_ODR |= LCD_DB_MASK;  /* Configure pins as inputs */
}

/* Write a data byte (glyph) to the LCD controller. */
void LCD_WriteData(unsigned char Data)
{
    LCD_WaitUntilReady();
    LCD_LoadBus(Data);

    *AT91C_PIOC_CODR = LCD_BUF_DIR;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_BUF_OE;  /* Configure pins as inputs */
    *AT91C_PIOC_OER |= LCD_DB_MASK;  /* Configure pins as outputs */

    *AT91C_PIOC_CODR = LCD_CD;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_CE;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_WR;  /* Configure pins as inputs */

    BusyDelay(2000);

    *AT91C_PIOC_SODR = LCD_CE;  /* Configure pins as inputs */
    *AT91C_PIOC_SODR = LCD_WR;  /* Configure pins as inputs */
    *AT91C_PIOC_SODR = LCD_BUF_OE;  /* Configure pins as inputs */
    *AT91C_PIOC_ODR |= LCD_DB_MASK;  /* Configure pins as inputs */
}

/* Convert a small ASCII subset to LCD glyph codes and write it. */
void LCD_WriteMenuChar(char c)
{
    switch (c)
    {
        
        case 'M': LCD_WriteData(GLYPH_M); break;
        case 'E': LCD_WriteData(GLYPH_E); break;
        case 'N': LCD_WriteData(GLYPH_N); break;
        case 'Y': LCD_WriteData(GLYPH_Y); break;

        case 'R': LCD_WriteData(GLYPH_R); break;
        case 'e': LCD_WriteData(GLYPH_E); break;   
        case 'q': LCD_WriteData(GLYPH_Q); break;
        case 'F': LCD_WriteData(GLYPH_F); break;
        case 'O': LCD_WriteData(GLYPH_O); break;
        case 'T': LCD_WriteData(GLYPH_T); break;

        
        case ':': LCD_WriteData(GLYPH_COLON);  break;
        case ' ': LCD_WriteData(GLYPH_SPACE);  break;

        
        case '0': LCD_WriteData(0x10 + 0); break;
        case '1': LCD_WriteData(0x10 + 1); break;
        case '2': LCD_WriteData(0x10 + 2); break;
        case '3': LCD_WriteData(0x10 + 3); break;
        case '4': LCD_WriteData(0x10 + 4); break;
        case '5': LCD_WriteData(0x10 + 5); break;
        case '6': LCD_WriteData(0x10 + 6); break;
        case '7': LCD_WriteData(0x10 + 7); break;
        case '8': LCD_WriteData(0x10 + 8); break;
        case '9': LCD_WriteData(0x10 + 9); break;
        case 'A': LCD_WriteData(GLYPH_A); break;
        case 'C': LCD_WriteData(GLYPH_C); break;
        case 'D': LCD_WriteData(GLYPH_D); break;
        case 'G': LCD_WriteData(GLYPH_G); break;
        case 'I': LCD_WriteData(GLYPH_I); break;
        case 'V': LCD_WriteData(GLYPH_V); break;
        case 'X': LCD_WriteData(GLYPH_X); break;
        case 'L': LCD_WriteData(GLYPH_L); break;
        case 'P': LCD_WriteData(GLYPH_P); break;
        case 'S': LCD_WriteData(GLYPH_S); break;
        case 'U': LCD_WriteData(GLYPH_U); break;
        case 'H': LCD_WriteData(GLYPH_H); break;
        case 'K': LCD_WriteData(GLYPH_K); break;
        case 'W': LCD_WriteData(GLYPH_W); break;
     

        case '*': LCD_WriteData(GLYPH_STAR);break;

        case '#': LCD_WriteData(GLYPH_HASH); break;

        case '.': LCD_WriteData(GLYPH_DOT); break;
        case '/': LCD_WriteData(0x0F);         break;   




        default:
            LCD_WriteData(GLYPH_SPACE);  
            break;
    }

    
    LCD_WriteCommand(0xC0);
}





/* Configure all GPIO pins used by the LCD interface and buffers. */
void LCD_ConfigPins(void)
{
    *AT91C_PMC_PCER |= (1 << 13) | (1 << 14);   /* Enable peripheral clock in PMC */

    *AT91C_PIOC_PER |= LCD_WR | LCD_RD | LCD_CE | LCD_CD |  /* Enable PIO control for these pins */
                       LCD_FS | LCD_RV | LCD_BUF_DIR | LCD_BUF_OE;
    *AT91C_PIOC_OER |= LCD_WR | LCD_RD | LCD_CE | LCD_CD |  /* Configure pins as outputs */
                       LCD_FS | LCD_RV | LCD_BUF_DIR | LCD_BUF_OE;

    *AT91C_PIOD_PER |= LCD_RESET;  /* Enable PIO control for these pins */
    *AT91C_PIOD_OER |= LCD_RESET;  /* Configure pins as outputs */

    *AT91C_PIOC_PER |= LCD_DB_MASK;  /* Enable PIO control for these pins */
    *AT91C_PIOC_ODR |= LCD_DB_MASK;  /* Configure pins as inputs */

    *AT91C_PIOC_SODR = LCD_WR | LCD_RD | LCD_CE;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_CD;  /* Configure pins as inputs */

    *AT91C_PIOC_SODR = LCD_BUF_OE;  /* Configure pins as inputs */
    *AT91C_PIOC_CODR = LCD_BUF_DIR;  /* Configure pins as inputs */

    *AT91C_PIOC_CODR = LCD_FS | LCD_RV;  /* Configure pins as inputs */
}

/* Reset and initialize the LCD controller (text memory mapping). */
void LCD_InitController(void)
{
    *AT91C_PIOD_CODR = LCD_RESET;  /* Configure pins as inputs */
    BusyDelay(50000);
    *AT91C_PIOD_SODR = LCD_RESET;  /* Configure pins as inputs */

    LCD_WriteData(0x00);  LCD_WriteData(0x00);  LCD_WriteCommand(0x40);
    LCD_WriteData(0x00);  LCD_WriteData(0x40);  LCD_WriteCommand(0x42);
    LCD_WriteData(0x28);  LCD_WriteData(0x00);  LCD_WriteCommand(0x41);
    LCD_WriteData(0x28);  LCD_WriteData(0x00);  LCD_WriteCommand(0x43);

    LCD_WriteCommand(0x80);
    LCD_WriteCommand(0x94);
}

/* Set the LCD text memory address pointer. */
void LCD_SetMemoryAddress(unsigned short addr)
{
    LCD_WriteData((unsigned char)(addr & 0xFF));
    LCD_WriteData((unsigned char)((addr >> 8) & 0xFF));
    LCD_WriteCommand(0x24);
}

/* Write one glyph code to the LCD text memory (auto-increment). */
void LCD_WriteGlyph(unsigned char ch)
{
    LCD_WriteData(ch);
    LCD_WriteCommand(0xC0);
}

/* Clear the full LCD text memory area (16x40). */
void LCD_ClearTextMemory(void)
{
    LCD_SetMemoryAddress(0x0000);
    for (int i = 0; i < 40 * 16; i++)
    {
        LCD_WriteData(0x00);
        LCD_WriteCommand(0xC0);
    }
    LCD_SetMemoryAddress(0x0000);
}



/* Configure MAX6575 interface pins and TC0 capture interrupt for temperature timing. */
void TempSensor_Init(void)
{
    *AT91C_PMC_PCER |= (1 << 12);    /* Enable peripheral clock in PMC */
    *AT91C_PMC_PCER |= (1 << 27);    /* Enable peripheral clock in PMC */

    *AT91C_TC0_CMR = (0 << 0) | (0 << 1) | (0 << 2);  /* Configure Timer Counter mode */
    *AT91C_TC0_CCR = (1 << 2) | (1 << 0);  /* Start/stop/reset timer counter */
    *AT91C_TC0_CMR = (2 << 16) | (1 << 18);  /* Configure Timer Counter mode */

    *AT91C_PIOB_PER   |= TEMP_PIN;  /* Enable PIO control for these pins */
    *AT91C_PIOB_PPUDR |= TEMP_PIN;  /* Disable pull-ups */
    *AT91C_PIOB_ODR   |= TEMP_PIN;  /* Configure pins as inputs */

    (void)*AT91C_TC0_SR;  /* Read timer status to clear flags */

    NVIC_ClearPendingIRQ(TC0_IRQn);
    NVIC_SetPriority(TC0_IRQn, 0);
    NVIC_EnableIRQ(TC0_IRQn);
}

/* Trigger a new MAX6575 measurement sequence (uses SysTick delay). */
void TempSensor_StartMeasurement(void)
{
    *AT91C_PIOB_OER |= TEMP_PIN;  /* Configure pins as outputs */
    *AT91C_PIOB_SODR = TEMP_PIN;  /* Configure pins as inputs */
    *AT91C_PIOB_CODR = TEMP_PIN;  /* Configure pins as inputs */

    g_systick_active  = 1;
    g_systick_counter = 0;
    while (g_systick_active == 1) { }

    *AT91C_PIOB_SODR = TEMP_PIN;  /* Configure pins as inputs */
    *AT91C_PIOB_CODR = TEMP_PIN;  /* Configure pins as inputs */

    BusyDelay(25);

    *AT91C_PIOB_SODR = TEMP_PIN;  /* Configure pins as inputs */
    *AT91C_PIOB_ODR  = TEMP_PIN;  /* Configure pins as inputs */

    *AT91C_TC0_CCR = (1 << 2);  /* Start/stop/reset timer counter */
    (void)*AT91C_TC0_SR;  /* Read timer status to clear flags */
    *AT91C_TC0_IER = (1 << 6);  /* Enable timer interrupts */
}

/* Convert captured TC0 timing into Celsius and update global temperature value. */
double TempSensor_ReadCelsius(void)
{
    /* If TC0 has captured a pulse width (RA/RB), convert it to temperature.
       MAX6575 output is encoded as a pulse width -> we measure time between edges.
       The conversion constant (211.416) comes from the lab calibration formula. */

    if (g_tc0_capture_done)
    {
        int RA   = *AT91C_TC0_RA;  /* Capture register value */
        int RB   = *AT91C_TC0_RB;  /* Capture register value */
        int diff = RB - RA;

        g_temperature_c = ((double)diff / 211.416) - 273.15;

        g_tc0_capture_done        = 0;
        g_temp_measurement_ready  = 1;

        g_temp_has_valid_sample   = 1;   
    }
    return g_temperature_c;
}

/* Print temperature as NN.N (using LCD digit glyph codes). */
void LCD_PrintTemperature(double t)
{
    if (t < 0) t = 0;

    int whole  = (int)t;
    int tenths = (int)(t * 10.0) % 10;

    int tens =  (whole / 10) % 10;
    int ones =  whole % 10;

    LCD_WriteData(0x10 + tens);  LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + ones);  LCD_WriteCommand(0xC0);
    LCD_WriteData(0x0E);         LCD_WriteCommand(0xC0); 
    LCD_WriteData(0x10 + tenths);LCD_WriteCommand(0xC0);
}



/* Enable ADC and configure the analog input pin for the light sensor. */
void LightSensor_Init(void)
{
    *AT91C_PMC_PCER  |= (1 << 11);     /* Enable peripheral clock in PMC */
    *AT91C_PMC_PCER1 |= (1 << 5);      /* Enable peripheral clock in PMC */

    *AT91C_ADCC_MR = (2 << 8);  /* Configure ADC mode (prescaler, timing) */

    *AT91C_PIOA_PER   |= (1 << 3);  /* Enable PIO control for these pins */
    *AT91C_PIOA_PPUDR |= (1 << 3);  /* Disable pull-ups */
    *AT91C_PIOA_ODR   |= (1 << 3);  /* Configure pins as inputs */
}

/* Start an ADC conversion and return the latest light value (0..4095). */
double LightSensor_Sample(void)
{
    *AT91C_ADCC_CHER = (1 << 1);  /* Enable ADC channel */
    *AT91C_ADCC_CR   = (1 << 1);  /* Start ADC conversion */

    while (((*AT91C_ADCC_SR) & (1 << 24)) != (1 << 24))  /* ADC status (EOC flags) */
    {
        
    }

    g_light_raw = (double)(*AT91C_ADCC_LCDR & 0xFFF);  /* Read last ADC converted data */
    return g_light_raw;
}

/* Print a 4-digit light value to the LCD. */
void LCD_PrintLightValue(int value)
{
    if (value < 0)    value = 0;
    if (value > 9999) value = 9999;

    int thous = (value / 1000) % 10;
    int hunds = (value / 100)  % 10;
    int tens  = (value / 10)   % 10;
    int ones  = value % 10;

    LCD_WriteData(0x10 + thous); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + hunds); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + tens);  LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + ones);  LCD_WriteCommand(0xC0);
}








/* Configure PWM channel 1 on PB17 for servo control. */
void Servo_InitPwm(void)
{
    *AT91C_PMC_PCER  |= (1 << 12);    /* Enable peripheral clock in PMC */
    *AT91C_PMC_PCER1 |= (1 << 4);     /* Enable peripheral clock in PMC */

    
    *AT91C_PIOB_PDR  |= SERVO_PWM_PIN;  /* Disable PIO control (hand pins to peripheral) */

    
    *AT91C_PIOB_ABMR |= SERVO_PWM_PIN;     /* Select peripheral A/B for these pins */

    
    *AT91C_PWMC_MR = (32 << 0) | (0 << 8);     /* Configure PWM clock */

    
    *AT91C_PWMC_CH1_CMR   = 5;          /* Configure PWM clock */
    *AT91C_PWMC_CH1_CPRDR = 52500;      /* Set PWM period (counts per cycle) */
    *AT91C_PWMC_CH1_CDTYR = 1837;       /* Set PWM duty (initial) */

    
    *AT91C_PWMC_ENA = (1 << 1);  /* Enable PWM channel(s) */
}



/* Legacy helper: map keypad key (1..12) into a servo duty step. */
void Servo_UpdateFromKey(int key)
{
    if (key < 1 || key > 12)
        return;

    unsigned int duty = 1837 + (unsigned int)key * 280;
    *AT91C_PWMC_CH1_CDTYUPDR = duty;  /* Update PWM duty (takes effect next period) */
}



/* Configure SysTick to fire every 1 ms. */
void Init_SysTick(void)
{
    SysTick_Config(SystemCoreClock / 1000); 
}

/* Initialize the simulated clock to a known start date/time. */
void Clock_SetDefault(void)
{
    g_time.sec   = 0;
    g_time.min   = 0;
    g_time.hour  = 12;
    g_time.day   = 1;
    g_time.month = 1;
    g_time.year  = 2024;
}

/* Add a number of minutes to a DateTime (handles month/year rollover). */
static void DateTime_AddMinutes(DateTime *t, int minutes)
{
    while (minutes-- > 0)
    {
        t->min++;
        if (t->min >= 60) { t->min = 0; t->hour++; }
        if (t->hour >= 24) { t->hour = 0; t->day++; }

        unsigned char dim = days_in_month[t->month - 1];
        if (t->month == 2 && (t->year % 4) == 0) dim = 29;

        if (t->day > dim)
        {
            t->day = 1;
            t->month++;
            if (t->month > 12)
            {
                t->month = 1;
                t->year++;
            }
        }
    }
}

/* Advance simulated time and (once per simulated minute) log temperature + day stats. */
void Clock_AdvanceSeconds(unsigned int seconds)
{
    /* Software clock update.
       We increment g_time one second at a time so we can hook events on rollovers:
       - Each new minute: log temperature and update day stats
       - Each new hour/day/month/year: handle calendar rollover (incl. leap year) */

    while (seconds--)
    {
        g_time.sec++;

        
        if (g_time.sec >= 60)
        {
            g_time.sec = 0;
            g_time.min++;

            
            TempLog_AddSample(g_temperature_c);
            DayStats_UpdateWithSample((const DateTime*)&g_time, g_temperature_c);
        }

        if (g_time.min >= 60)
        {
            g_time.min = 0;
            g_time.hour++;
        }

        if (g_time.hour >= 24)
        {
            g_time.hour = 0;
            g_time.day++;

            
            unsigned char dim = days_in_month[g_time.month - 1];

            
            if (g_time.month == 2 &&
                (g_time.year % 4) == 0)
            {
                dim = 29;
            }

            if (g_time.day > dim)
            {
                g_time.day = 1;
                g_time.month++;
                if (g_time.month > 12)
                {
                    g_time.month = 1;
                    g_time.year++;
                }
            }
        }
    }
}





/* Increment one clock field while editing (with range checking). */
void Clock_IncrementField(EditField f, int delta)
{
    switch (f)
    {
    case EDIT_HOUR: {
        int h = (int)g_time.hour + delta;
        if (h < 0)  h = 23;
        if (h > 23) h = 0;
        g_time.hour = (unsigned char)h;
        break;
    }

    case EDIT_MIN: {
        int m = (int)g_time.min + delta;
        if (m < 0)  m = 59;
        if (m > 59) m = 0;
        g_time.min = (unsigned char)m;
        break;
    }

    case EDIT_SEC: {
        int s = (int)g_time.sec + delta;
        if (s < 0)  s = 59;
        if (s > 59) s = 0;
        g_time.sec = (unsigned char)s;
        break;
    }

    case EDIT_DAY: {
        int d = (int)g_time.day + delta;
        unsigned char dim = days_in_month[g_time.month - 1];

        if (g_time.month == 2 && (g_time.year % 4) == 0)
            dim = 29;

        if (d < 1)   d = 1;
        if (d > dim) d = dim;

        g_time.day = (unsigned char)d;
        break;
    }

    case EDIT_MONTH: {
        int mo = (int)g_time.month + delta;
        if (mo < 1)  mo = 1;
        if (mo > 12) mo = 12;
        g_time.month = (unsigned char)mo;
        break;
    }

    case EDIT_YEAR: {
        int y = (int)g_time.year + delta;
        if (y < 0) y = 0;
        if (y > 2099) y = 2099;
        g_time.year = (unsigned short)y;
        break;
    }
    }
}
/* State machine for entering/exiting and editing the clock (Req.1). */
void UI_HandleKeypadForClock(int key)
{
    if (key == 0)
        return;   

    
    if (g_ui_mode == UI_MODE_NORMAL)
    {
        
       if (g_ui_view == UI_VIEW_LIVE && !g_live_show_temp && key == 11)
{
    g_ui_mode    = UI_MODE_SET_CLOCK;
    g_edit_field = EDIT_HOUR;
}

        return;  
    }
    

    
    if (g_ui_mode != UI_MODE_SET_CLOCK)
        return;

    
    
    
    if (key == 11)
    {
        if (g_edit_field == EDIT_YEAR)
            g_edit_field = EDIT_HOUR;
        else
            g_edit_field++;
        return;
    }

    
    if (key == 10)
    {
        g_ui_mode = UI_MODE_NORMAL;
        g_ui_view = UI_VIEW_LIVE;  
        return;
    }

    
    if (key == 7)
    {
        Clock_IncrementField(g_edit_field, -1);
    }

/**
 * if()
 * Short: Driver / helper function used by the greenhouse application.
 */
    else if (key == 8)
    {
        Clock_IncrementField(g_edit_field, +1);
    }
}





/* Render HH:MM:SS on the first LCD row. */
void UI_PrintTime(void)
{
    int h_tens = g_time.hour / 10;
    int h_ones = g_time.hour % 10;
    int m_tens = g_time.min / 10;
    int m_ones = g_time.min % 10;
    int s_tens = g_time.sec / 10;
    int s_ones = g_time.sec % 10;

    LCD_WriteData(0x10 + h_tens); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + h_ones); LCD_WriteCommand(0xC0);

    LCD_WriteData(0x0E);          LCD_WriteCommand(0xC0); 

    LCD_WriteData(0x10 + m_tens); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + m_ones); LCD_WriteCommand(0xC0);

    LCD_WriteData(0x0E);          LCD_WriteCommand(0xC0); 

    LCD_WriteData(0x10 + s_tens); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + s_ones); LCD_WriteCommand(0xC0);
}

/* Render DD/MM/YYYY on the second LCD row. */
void UI_PrintDate(void)
{
    int d_tens = g_time.day / 10;
    int d_ones = g_time.day % 10;
    int m_tens = g_time.month / 10;
    int m_ones = g_time.month % 10;

    int y_thousands = (g_time.year / 1000) % 10;
    int y_hundreds = (g_time.year / 100)  % 10;
    int y_tens     = (g_time.year / 10)   % 10;
    int y_ones     =  g_time.year         % 10;

    
    LCD_WriteData(0x10 + d_tens); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + d_ones); LCD_WriteCommand(0xC0);

    
    LCD_WriteData(0x0F);          LCD_WriteCommand(0xC0); 

    
    LCD_WriteData(0x10 + m_tens); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + m_ones); LCD_WriteCommand(0xC0);

    
    LCD_WriteData(0x0F);          LCD_WriteCommand(0xC0);

    
    LCD_WriteData(0x10 + y_thousands); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + y_hundreds);  LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + y_tens);      LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + y_ones);      LCD_WriteCommand(0xC0);
}
  
/* Write the '#. MENU' hint at the bottom-right of the display. */
static void UI_PrintMenuHintBottomRight(void)
{
    

    LCD_SetMemoryAddress(0x0258);

    
    for (int i = 0; i < 33; i++)
    {
        LCD_WriteData(GLYPH_SPACE);
        LCD_WriteCommand(0xC0);
    }
    
    LCD_WriteMenuText("#. MENU");
}

/* Main UI renderer: prints different screens depending on the active view/mode. */
void UI_UpdateScreen(double temp, int light)
{
    /* Central screen refresh.
       Idea: only clear/redraw when needed to avoid visible flicker.
       - MENU: redraw only when g_menu_needs_redraw is set
       - Day stats: redraw table only when g_daystats_needs_redraw is set
       - Other views: update just the dynamic fields (time/date/temp/etc.) */


    /* Rendering strategy:
       - LCD writes are slow; clearing every loop causes flicker.
       - We clear only when switching views, otherwise overwrite only fields.
       - Addresses like 0x0000, 0x0028, ... are line starts in LCD text memory. */
    
    static UiView last_view = UI_VIEW_MENU;

    
    if (g_ui_view == UI_VIEW_MENU)
    {
        if (g_menu_needs_redraw)
        {
            UI_PrintMenu();
            g_menu_needs_redraw = 0;
        }
        
        last_view = g_ui_view;
        return;
    }

    
    if (g_ui_view == UI_VIEW_DAY_STATS)
    {
        if (g_daystats_needs_redraw)
        {
            UI_PrintDayStatsTable();
            g_daystats_needs_redraw = 0;
        }
        
        last_view = g_ui_view;
        return;
    }

    
    if (last_view == UI_VIEW_MENU)
    {
        
        LCD_SetMemoryAddress(0x0000);
        for (int i = 0; i < 40; i++)
        {
            LCD_WriteData(0x00);
            LCD_WriteCommand(0xC0);
        }

        
        LCD_SetMemoryAddress(0x0028);
        for (int i = 0; i < 40; i++)
        {
            LCD_WriteData(0x00);
            LCD_WriteCommand(0xC0);
        }
    }

    
    LCD_SetMemoryAddress(0x0000);
    UI_PrintTime();

    LCD_SetMemoryAddress(0x0028);
    UI_PrintDate();

    
    if (g_ui_view == UI_VIEW_LIVE)
    {
       

        
        if (g_live_show_temp)
        {
            LCD_SetMemoryAddress(0x0050);  

            
            LCD_PrintTemperature(temp);

            
            LCD_WriteData(0x00); LCD_WriteCommand(0xC0);
            LCD_WriteData(0x00); LCD_WriteCommand(0xC0);

            
            LCD_PrintTemperature(g_temp_limit_min);

            
            LCD_WriteData(0x00); LCD_WriteCommand(0xC0);
            LCD_WriteData(0x00); LCD_WriteCommand(0xC0);

            
            LCD_PrintTemperature(g_temp_limit_max);
        }

        
        if (!g_live_show_temp)
        {
            
            LCD_SetMemoryAddress(0x00A0);  
            LCD_WriteMenuText("0. PAUSE/NEXT FIELD");

            
            LCD_SetMemoryAddress(0x00C8);  
            LCD_WriteMenuText("7. MINUS");

            
            LCD_SetMemoryAddress(0x00F0);  
            LCD_WriteMenuText("8. PLUS");

            
            LCD_SetMemoryAddress(0x0118);  
            LCD_WriteMenuText("*. RESUME");
        }
else
{
    
    if (g_req5_needs_redraw)
    {
        g_req5_needs_redraw = 0;

        
        LCD_SetMemoryAddress(0x00A0);  
        for (int i = 0; i < 40; i++) { LCD_WriteData(0x00); LCD_WriteCommand(0xC0); }
        LCD_SetMemoryAddress(0x00A0);
        LCD_WriteMenuText("0: SELECT MIN/MAX");

        LCD_SetMemoryAddress(0x00C8);  
        for (int i = 0; i < 40; i++) { LCD_WriteData(0x00); LCD_WriteCommand(0xC0); }
        LCD_SetMemoryAddress(0x00C8);
        LCD_WriteMenuText("7: LOWER LIMIT");

        LCD_SetMemoryAddress(0x00F0);  
        for (int i = 0; i < 40; i++) { LCD_WriteData(0x00); LCD_WriteCommand(0xC0); }
        LCD_SetMemoryAddress(0x00F0);
        LCD_WriteMenuText("8: RAISE LIMIT");

        LCD_SetMemoryAddress(0x0118);  
        for (int i = 0; i < 40; i++) { LCD_WriteData(0x00); LCD_WriteCommand(0xC0); }
        LCD_SetMemoryAddress(0x0118);
        LCD_WriteMenuText("9: TEMP OK");
    }

    
}





    }
    

/**
 * if()
 * Short: Driver / helper function used by the greenhouse application.
 */
 else if (g_ui_view == UI_VIEW_LOG_STATS)
{
    static unsigned int last_ms = 0;

    if (!g_fast_mode)
    {
        UI_PrintLogStats();
    }
    else
    {
        
        if ((g_time_ms - last_ms) >= 250)
        {
            last_ms = g_time_ms;
            UI_PrintLogStats();
        }
    }
}


/**
 * if()
 * Short: Driver / helper function used by the greenhouse application.
 */
    else if (g_ui_view == UI_VIEW_LIGHT_CTRL)
    {
        UI_PrintLightStatus();
    }

    
    if (g_ui_view != UI_VIEW_MENU)
    {
        UI_PrintMenuHintBottomRight();
    }

    
    if (g_temp_alarm_active &&
        g_ui_view == UI_VIEW_LIVE &&
        g_ui_mode == UI_MODE_NORMAL &&
    g_live_show_temp)
    {
        UI_PrintTempAlarmMessage();
    }
    else
    {
        if (g_ui_view == UI_VIEW_LIVE)
        {
            LCD_SetMemoryAddress(0x0078);   
            for (int i = 0; i < 40; i++)
            {
                LCD_WriteData(0x00);
                LCD_WriteCommand(0xC0);
            }
        }
    }

    
    last_view = g_ui_view;
}



   
/* Clear the LCD area below the time/date header (rows 3..7). */
void UI_ClearBelowHeader(void)
{
    
    LCD_SetMemoryAddress(0x0050);  
    for (int i = 0; i < 200; i++)
    {
        LCD_WriteData(0x00);
        LCD_WriteCommand(0xC0);
    }
}




/* Utility: print a 2-digit number (00..99). */
static void LCD_PrintTwoDigits(int value)
{
    int tens = (value / 10) % 10;
    int ones = value % 10;
    LCD_WriteData(0x10 + tens); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + ones); LCD_WriteCommand(0xC0);
}
/* Utility: print a 5-digit number (00000..99999). */
static void LCD_PrintFiveDigits(int value)
{
    int d1 = (value / 10000) % 10;
    int d2 = (value / 1000)  % 10;
    int d3 = (value / 100)   % 10;
    int d4 = (value / 10)    % 10;
    int d5 =  value % 10;

    LCD_WriteData(0x10 + d1); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + d2); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + d3); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + d4); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + d5); LCD_WriteCommand(0xC0);
}




/* Utility: convert seconds to HH:MM and print. */
static void LCD_PrintHHMMFromSeconds(unsigned long sec)
{
    unsigned int hours   = (unsigned int)(sec / 3600UL);
    unsigned int minutes = (unsigned int)((sec % 3600UL) / 60UL);

    if (hours > 99)   hours = 99;   
    if (minutes > 59) minutes = 59;

    
    LCD_PrintTwoDigits(hours);

    
    LCD_WriteData(0x0E);    LCD_WriteCommand(0xC0);

    
    LCD_PrintTwoDigits(minutes);
}

/* Write a null-terminated ASCII string using LCD_WriteMenuChar mapping. */
void LCD_WriteMenuText(const char *s)
{
    while (*s)
    {
        LCD_WriteMenuChar(*s++);
    }
}




/* Utility: print HH:MM from a DateTime. */
static void LCD_PrintTime_HHMM(const DateTime *t)
{
    LCD_PrintTwoDigits(t->hour);
    LCD_WriteData(0x0E);          LCD_WriteCommand(0xC0); 
    LCD_PrintTwoDigits(t->min);
}




/* Render the 7-day statistics table (avg/min/max + times). */
void UI_PrintDayStatsTable(void)
{
    int row = 0;

    

    
    LCD_SetMemoryAddress(0x0000);
    for (int i = 0; i < 40; i++)
    {
        LCD_WriteData(GLYPH_SPACE);
        LCD_WriteCommand(0xC0);
    }

    
    LCD_SetMemoryAddress(0x0000);
    
    LCD_WriteMenuText("DATE    AVG     MIN     MAX");

    row = 1;  

    

    for (int i = 0; i < MAX_DAYS_STATS && row < 15; i++)
    {
        if (!g_day_stats[i].valid)
            continue;

        DayStats *ds = &g_day_stats[i];
        double avg = ds->sumTemp / (double)ds->count;

        
        unsigned short addr_data =
            0x0000 + row * 0x28;        
        unsigned short addr_time =
            0x0000 + (row + 1) * 0x28;  

        

        
        LCD_SetMemoryAddress(addr_data);
        for (int c = 0; c < 40; c++)
        {
            LCD_WriteData(GLYPH_SPACE);
            LCD_WriteCommand(0xC0);
        }

        
        LCD_SetMemoryAddress(addr_data);

        
        LCD_PrintTwoDigits(ds->day);
        LCD_WriteData(0x0F); LCD_WriteCommand(0xC0); 

        LCD_PrintTwoDigits(ds->month);

        
        for (int s = 0; s < 4; s++)
        {
            LCD_WriteData(GLYPH_SPACE);
            LCD_WriteCommand(0xC0);
        }

        
        LCD_PrintTemperature(avg);

        
        for (int s = 0; s < 3; s++)
        {
            LCD_WriteData(GLYPH_SPACE);
            LCD_WriteCommand(0xC0);
        }

        
        LCD_PrintTemperature(ds->minTemp);

        
        for (int s = 0; s < 3; s++)
        {
            LCD_WriteData(GLYPH_SPACE);
            LCD_WriteCommand(0xC0);
        }

        
        LCD_PrintTemperature(ds->maxTemp);

        

        
        LCD_SetMemoryAddress(addr_time);
        for (int c = 0; c < 40; c++)
        {
            LCD_WriteData(GLYPH_SPACE);
            LCD_WriteCommand(0xC0);
        }

        
        LCD_SetMemoryAddress(addr_time);

        
       for (int s = 0; s < 16; s++)
        {
            LCD_WriteData(GLYPH_SPACE);
            LCD_WriteCommand(0xC0);
        }

        
        LCD_PrintTime_HHMM(&ds->minTime);

        
      for (int s = 0; s < 2; s++)
        {
            LCD_WriteData(GLYPH_SPACE);
            LCD_WriteCommand(0xC0);
        }

        
        LCD_PrintTime_HHMM(&ds->maxTime);

        
        row += 2;
    }

    
    for (; row < 16; row++)
    {
        unsigned short addr = 0x0000 + row * 0x28;
        LCD_SetMemoryAddress(addr);
        for (int c = 0; c < 40; c++)
        {
            LCD_WriteData(GLYPH_SPACE);
            LCD_WriteCommand(0xC0);
        }
    } UI_PrintMenuHintBottomRight();
}



/* Render the temperature log viewer (selected sample + timestamp). */
void UI_PrintLogStats(void)
{
    static unsigned int last_count = 0;
    static int          last_index = -1;

    unsigned int count_now = g_log_count;
    int          index_now = g_log_selected_index;
    
static int follow_latest = 1;
if (g_log_count > 0)
{
    
    if (g_log_selected_index < (int)g_log_count - 1)
        follow_latest = 0;
    else
        follow_latest = 1;
}


if (g_logstats_count_changed && follow_latest && g_log_count > 0)
{
    g_log_selected_index = (int)g_log_count - 1;
    index_now = g_log_selected_index;
}


    if (count_now == 0 || g_log_head == TEMPREC_NULL)
    {
        LCD_SetMemoryAddress(0x0050);
        for (int i = 0; i < 5 * 40; i++)
        {
            LCD_WriteData(0x00);
            LCD_WriteCommand(0xC0);
        }
        LCD_SetMemoryAddress(0x0050);
        LCD_WriteMenuText("NO DATA");
        last_count = count_now;
        last_index = -1;
        return;
    }

    if (index_now < 0) index_now = 0;
    if (index_now >= (int)count_now) index_now = (int)count_now - 1;
    g_log_selected_index = index_now;

if (!g_logstats_needs_redraw &&
    index_now == last_index &&
    g_logstats_count_changed)
{
    
    LCD_SetMemoryAddress(0x0050 + 12);
    LCD_PrintFiveDigits((int)count_now);

    
    if (follow_latest)
    {
        LCD_SetMemoryAddress(0x0050 + 4);      
        LCD_PrintFiveDigits(index_now + 1);
    }

    last_count = count_now;
    g_logstats_count_changed = 0;
    return;
}


if (!g_logstats_needs_redraw &&
    count_now == last_count &&
    index_now == last_index)
{
    return;
}



    last_count = count_now;
    last_index = index_now;

    TempRecord *sel = TempLog_GetByIndex(index_now);
    if (!sel) return;
    
DateTime ts;
TempLog_GetTimestampByIndex(index_now, &ts);

    
    LCD_SetMemoryAddress(0x0050);
    for (int i = 0; i < 5 * 40; i++)
    {
        LCD_WriteData(0x00);
        LCD_WriteCommand(0xC0);
    }

    
    LCD_SetMemoryAddress(0x0050);
    LCD_WriteMenuText("LOG ");
   
{
    int v = index_now + 1;   

    int d1 = (v / 10000) % 10;
    int d2 = (v / 1000)  % 10;
    int d3 = (v / 100)   % 10;
    int d4 = (v / 10)    % 10;
    int d5 =  v % 10;

    LCD_WriteData(0x10 + d1); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + d2); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + d3); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + d4); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x10 + d5); LCD_WriteCommand(0xC0);
}

    LCD_WriteData(0x00); LCD_WriteCommand(0xC0);
    LCD_WriteData(0x0F); LCD_WriteCommand(0xC0); 
    LCD_WriteData(0x00); LCD_WriteCommand(0xC0);

    
    {
        int n = (int)count_now;
        int d1 = (n / 10000) % 10;
        int d2 = (n / 1000) % 10;
        int d3 = (n / 100) % 10;
        int d4 = (n / 10) % 10;
        int d5 = n % 10;
        LCD_WriteData(0x10 + d1); LCD_WriteCommand(0xC0);
        LCD_WriteData(0x10 + d2); LCD_WriteCommand(0xC0);
        LCD_WriteData(0x10 + d3); LCD_WriteCommand(0xC0);
        LCD_WriteData(0x10 + d4); LCD_WriteCommand(0xC0);
        LCD_WriteData(0x10 + d5); LCD_WriteCommand(0xC0);
    }

    
    LCD_SetMemoryAddress(0x0078);
    LCD_WriteMenuText("TEMP ");
    {
        double t = ((double)sel->temp10) / 10.0;
        LCD_PrintTemperature(t);
    }
    LCD_WriteData(0x00); LCD_WriteCommand(0xC0);
    LCD_WriteData('C');  LCD_WriteCommand(0xC0);

    
   LCD_SetMemoryAddress(0x00A0);
LCD_WriteMenuText("DATE ");
LCD_PrintTwoDigits(ts.day);
LCD_WriteData(0x0F); LCD_WriteCommand(0xC0); 
LCD_PrintTwoDigits(ts.month);


    
   LCD_SetMemoryAddress(0x00C8);
LCD_WriteMenuText("TIME ");
LCD_PrintTwoDigits(ts.hour);
LCD_WriteData(0x0E); LCD_WriteCommand(0xC0); 
LCD_PrintTwoDigits(ts.min);


    
    LCD_SetMemoryAddress(0x00F0);
    LCD_WriteMenuText("4: PREV   6: NEXT");

    g_logstats_needs_redraw = 0;
    g_logstats_count_changed = 0;

}




/* Blinking alarm message when temperature is out of limits (Req.5). */
void UI_PrintTempAlarmMessage(void)
{
    
    int blink_on = ((g_time_ms / 500) % 2) == 0;

    
    LCD_SetMemoryAddress(0x0078);   
    for (int i = 0; i < 40; i++)
    {
        LCD_WriteData(0x00);
        LCD_WriteCommand(0xC0);
    }

    
    if (!blink_on)
    {
        return;
    }

    
    LCD_SetMemoryAddress(0x0078);

    
    if (g_temp_alarm_high)
    {
        

        LCD_WriteData('H'); LCD_WriteCommand(0xC0);
        LCD_WriteData(0x00); LCD_WriteCommand(0xC0); 
        LCD_WriteData('T'); LCD_WriteCommand(0xC0);
        LCD_WriteData('E'); LCD_WriteCommand(0xC0);
        LCD_WriteData('M'); LCD_WriteCommand(0xC0);
        LCD_WriteData('P'); LCD_WriteCommand(0xC0);
    }
    else
    {
        

        LCD_WriteData('L'); LCD_WriteCommand(0xC0);
        LCD_WriteData(0x00); LCD_WriteCommand(0xC0); 
        LCD_WriteData('T'); LCD_WriteCommand(0xC0);
        LCD_WriteData('E'); LCD_WriteCommand(0xC0);
        LCD_WriteData('M'); LCD_WriteCommand(0xC0);
        LCD_WriteData('P'); LCD_WriteCommand(0xC0);
    }
}



/* Render sunlight/LED/dark counters and servo 'sun position' (Req.4). */
void UI_PrintLightStatus(void)
{
  
    
    
   

    
    LCD_SetMemoryAddress(0x0050);

    unsigned int angle = (unsigned int)(g_sun_position_deg + 0.5);
    if (angle > 180) angle = 180;

    unsigned int hundreds = angle / 100;
    unsigned int tens     = (angle / 10) % 10;
    unsigned int ones     = angle % 10;

    
    if (hundreds > 0)
    {
        LCD_WriteData(0x10 + hundreds);
        LCD_WriteCommand(0xC0);
    }
    else
    {
        LCD_WriteData(0x00); 
        LCD_WriteCommand(0xC0);
    }

    LCD_WriteData(0x10 + tens);
    LCD_WriteCommand(0xC0);

    LCD_WriteData(0x10 + ones);
    LCD_WriteCommand(0xC0);

    
    LCD_WriteData(0x0F);       
    LCD_WriteCommand(0xC0);

    LCD_WriteData(0x10 + 1);   
    LCD_WriteCommand(0xC0);

    LCD_WriteData(0x10 + 8);   
    LCD_WriteCommand(0xC0);

    LCD_WriteData(0x10 + 0);   
    LCD_WriteCommand(0xC0);
    
    
    LCD_SetMemoryAddress(0x0078); 

    
    LCD_WriteMenuText("SUN L ");
    LCD_PrintHHMMFromSeconds(g_sunlight_seconds);

    
    LCD_WriteData(0x00);  LCD_WriteCommand(0xC0);

    
    LCD_WriteMenuText("LED L ");
    LCD_PrintHHMMFromSeconds(g_ledlight_seconds);

    
    LCD_WriteData(0x00);  LCD_WriteCommand(0xC0);

    
    LCD_WriteMenuText("DARK ");
    LCD_PrintHHMMFromSeconds(g_dark_seconds);

   

    
    LCD_SetMemoryAddress(0x00F0);   

    if (g_shade_on)
    {
        LCD_WriteMenuText("SHADE ON");
    }
    else
    {
        
        for (int i = 0; i < 40; i++)
        {
            LCD_WriteData(0x00);
            LCD_WriteCommand(0xC0);
        }
    }
}





   /* Render the main menu screen (Req selection + fast mode indicator). */
   void UI_PrintMenu(void)
{
    
    LCD_SetMemoryAddress(0x0000);
    for (int i = 0; i < 40 * 16; i++)
    {
        LCD_WriteData(0x00);
        LCD_WriteCommand(0xC0);
    }

    
    LCD_SetMemoryAddress(0x0000);   
    LCD_WriteMenuText("MENU");

    
    LCD_SetMemoryAddress(0x0028);   
    LCD_WriteMenuText("1: TIMESTAMP");

    
    LCD_SetMemoryAddress(0x0050);   
    LCD_WriteMenuText("2: TEMP RECORD");

    
    LCD_SetMemoryAddress(0x0078);   
    LCD_WriteMenuText("3: LOGGED DATA");

    
    LCD_SetMemoryAddress(0x00A0);   
    LCD_WriteMenuText("4: DARK/LIGHT/SUN");

    
    LCD_SetMemoryAddress(0x00C8);   
    LCD_WriteMenuText("5: ALARM TEMP");
    
     

    
    LCD_SetMemoryAddress(0x00F0);   
    for (int i = 0; i < 40; i++)
    {
        LCD_WriteData(0x00);
        LCD_WriteCommand(0xC0);
    }

    
    LCD_SetMemoryAddress(0x00F0);
    LCD_WriteMenuText("6: FAST MODE ");

    if (g_fast_mode)
    {
        LCD_WriteMenuText("ON ");
    }
    else
    {
        LCD_WriteMenuText("OFF");
    }
     
    UI_PrintMenuHintBottomRight();

}









/* 1 ms interrupt: advances simulated time and runs Req.4 light/servo/LED logic once per second. */
void SysTick_Handler(void)
{
    /* SysTick runs at 1 kHz (1 ms tick).
       We use it for:
       - A short timing pulse needed by the MAX6575 sensor start sequence
       - A software real-time clock (g_time) that can be accelerated in fast mode
       - Periodic greenhouse simulation (Req 4) and logging (Req 2/3) */


    /* SysTick runs every 1 ms.
       We build a 1-second "heartbeat" using g_time_ms and then advance the
       simulated clock. In FAST MODE, one real second equals 30 simulated minutes. */
    
    if (g_systick_active)
    {
        g_systick_counter++;
        if (g_systick_counter >= 15)
        {
            g_systick_counter = 0;
            g_systick_active  = 0;
        }
    }

    g_time_ms++;
    if (g_time_ms >= 1000)
    {
        g_time_ms = 0;

        if (g_ui_mode == UI_MODE_SET_CLOCK)
        {
            return;
        }
        /* Req. 6  simulation speed control.
           sim_seconds tells Clock_AdvanceSeconds() how much simulated time
           to jump per real second tick. */
        unsigned int sim_seconds;
        if (g_fast_mode)
        {
            
            sim_seconds = 30U * 60U;
        }
        else
        {
            
            sim_seconds = 1; 
        }

        

        

        
        Clock_AdvanceSeconds(sim_seconds);

        

        
       if (g_ui_mode == UI_MODE_NORMAL)

        {
            
            if (g_time.hour == 0 && g_time.min == 0 && g_time.sec == 0)
            {
                g_sunlight_seconds = 0;
                g_ledlight_seconds = 0;
                g_dark_seconds     = 0;
                g_shade_on         = 0;   
            }

           
            
            int sun_present = 0;

            if (g_light_raw < LIGHT_THRESHOLD_SUN)
            {
                sun_present = 1;   
            }
            /* Servo "sun tracker" (Req. 4):
               When sun is detected, we sweep the servo and reverse direction
               if the light reading drops (simple max-search). */
            static double servo_angle = 90.0;      
static int    servo_dir   = +1;        
static double last_light  = -1.0;      

if (sun_present)
{
    double light_now = g_light_raw;

    
    if (last_light < 0.0)
        last_light = light_now;

    
    if (light_now < last_light)
        servo_dir = -servo_dir;

    
    servo_angle += servo_dir * 2.0;

    
    if (servo_angle < 0.0)  { servo_angle = 0.0;   servo_dir = +1; }
    if (servo_angle > 180.0){ servo_angle = 180.0; servo_dir = -1; }

    g_sun_position_deg = servo_angle;
    Servo_SetAngleDegrees(g_sun_position_deg);

    last_light = light_now;
}
else
{
    
    g_sun_position_deg = 0.0;
    Servo_SetAngleDegrees(g_sun_position_deg);

    
    last_light = -1.0;
}


           

            
            unsigned long total_light = g_sunlight_seconds + g_ledlight_seconds;

            if (sun_present)
            {
                

                if (total_light < TARGET_LIGHT_SECONDS)
                {
                    g_led_on   = 0;
                    g_shade_on = 0;
                    g_sunlight_seconds += sim_seconds;
                }
                else
                {
                    g_led_on   = 0;
                    g_shade_on = 1;
                    g_sunlight_seconds += sim_seconds;
                }
            }
            else
            {
                
                g_shade_on = 0;

                if (g_dark_seconds < TARGET_DARK_SECONDS)
                {
                    g_led_on = 0;
                    g_dark_seconds += sim_seconds;
                }
                else
                {
                    if (total_light < TARGET_LIGHT_SECONDS)
                    {
                        g_led_on = 1;
                        g_ledlight_seconds += sim_seconds;
                    }
                    else
                    {
                        g_led_on = 0;
                        g_dark_seconds += sim_seconds;
                    }
                }

                
                unsigned long total_req4_seconds =
                    g_sunlight_seconds +
                    g_ledlight_seconds +
                    g_dark_seconds;

                if (total_req4_seconds >= TARGET_DAY_SECONDS)
                {
                    g_sunlight_seconds = 0;
                    g_ledlight_seconds = 0;
                    g_dark_seconds     = 0;
                }
            }

            
            LED_Set(g_led_on);
        }
    }
}

/* TC0 capture interrupt: marks temperature capture complete. */
void TC0_Handler(void)
{
    g_tc0_capture_done = 1;
    *AT91C_TC0_IDR = (1 << 6);  /* Disable timer interrupts */
    (void)*AT91C_TC0_SR;  /* Read timer status to clear flags */
}


/* Button interrupt: toggles a flag on button press (can be used as acknowledge). */
void PIOD_Handler(void)
{
    unsigned int isr = *AT91C_PIOD_ISR;  /* Read/clear PIO interrupt status */

    if (isr & BUTTON_PIN)
    {
        
        if (((*AT91C_PIOD_PDSR) & BUTTON_PIN) == 0)  /* Read pin levels */
        {
            
            g_button_pressed = !g_button_pressed;
        }
    }
}
