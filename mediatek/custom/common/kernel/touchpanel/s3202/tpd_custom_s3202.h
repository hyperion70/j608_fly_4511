#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
// #define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE
#define TPD_I2C_NUMBER           0
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_DELAY                (2*HZ/100)
//#define TPD_RES_X                480
//#define TPD_RES_Y                800
#define TPD_CALIBRATION_MATRIX  {1980,0,0,0,1805,0,0,0};

#define TPD_KEY_COUNT   3
#define key_1           66,1021              //auto define
#define key_2           270,1021
#define key_3           478,1021
//#define key_4           420,850
#define TPD_KEYS        {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#define TPD_KEYS_DIM    {{key_1,60,40},{key_2,60,40},{key_3,60,40}}

//#define TPD_HAVE_CALIBRATION
//#define TPD_HAVE_BUTTON                   //report key as coordinate,Vibration feedback
#define TPD_HAVE_TOUCH_KEY            //report key as key_value
//#define TPD_HAVE_TREMBLE_ELIMINATION

//#define TPD_NO_GPIO
//#define TPD_RESET_PIN_ADDR   (PERICFG_BASE + 0xC000)

#endif /* TOUCHPANEL_H__ */
