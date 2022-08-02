/* Include files */

#include "arduino_imu_pitch_roll_sfun.h"
#include "c2_arduino_imu_pitch_roll.h"
#include "mwmathutil.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(S);
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

/* Forward Declarations */

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static emlrtMCInfo c2_emlrtMCI = { 82, /* lineNo */
  5,                                   /* colNo */
  "power",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pName */
};

static emlrtMCInfo c2_b_emlrtMCI = { 1,/* lineNo */
  1,                                   /* colNo */
  "SystemCore",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemCore.p"/* pName */
};

static emlrtMCInfo c2_c_emlrtMCI = { 13,/* lineNo */
  9,                                   /* colNo */
  "sqrt",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pName */
};

static emlrtRSInfo c2_emlrtRSI = { 22, /* lineNo */
  "matlabCodegenHandle",               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\matlabCodegenHandle.m"/* pathName */
};

static emlrtRSInfo c2_b_emlrtRSI = { 1,/* lineNo */
  "SystemCore",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemCore.p"/* pathName */
};

static emlrtRSInfo c2_c_emlrtRSI = { 171,/* lineNo */
  "sensorUnit",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Unit.m"                             /* pathName */
};

static emlrtRSInfo c2_d_emlrtRSI = { 47,/* lineNo */
  "device",                            /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\device"
  ".m"                                 /* pathName */
};

static emlrtRSInfo c2_e_emlrtRSI = { 110,/* lineNo */
  "sensorBoard",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Board.m"                            /* pathName */
};

static emlrtRSInfo c2_f_emlrtRSI = { 18,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#arduino_imu_pitch_roll:1"          /* pathName */
};

static emlrtRSInfo c2_g_emlrtRSI = { 21,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#arduino_imu_pitch_roll:1"          /* pathName */
};

static emlrtRSInfo c2_h_emlrtRSI = { 38,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#arduino_imu_pitch_roll:1"          /* pathName */
};

static emlrtRSInfo c2_i_emlrtRSI = { 41,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#arduino_imu_pitch_roll:1"          /* pathName */
};

static emlrtRSInfo c2_j_emlrtRSI = { 43,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#arduino_imu_pitch_roll:1"          /* pathName */
};

static emlrtRSInfo c2_k_emlrtRSI = { 44,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#arduino_imu_pitch_roll:1"          /* pathName */
};

static emlrtRSInfo c2_l_emlrtRSI = { 67,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#arduino_imu_pitch_roll:1"          /* pathName */
};

static emlrtRSInfo c2_m_emlrtRSI = { 69,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#arduino_imu_pitch_roll:1"          /* pathName */
};

static emlrtRSInfo c2_n_emlrtRSI = { 70,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#arduino_imu_pitch_roll:1"          /* pathName */
};

static emlrtRSInfo c2_o_emlrtRSI = { 26,/* lineNo */
  "arduino",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\arduinoio\\+coder\\arduino.m"/* pathName */
};

static emlrtRSInfo c2_p_emlrtRSI = { 31,/* lineNo */
  "arduino",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\arduinoio\\+coder\\arduino.m"/* pathName */
};

static emlrtRSInfo c2_q_emlrtRSI = { 32,/* lineNo */
  "arduino",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\arduinoio\\+coder\\arduino.m"/* pathName */
};

static emlrtRSInfo c2_r_emlrtRSI = { 33,/* lineNo */
  "arduino",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\arduinoio\\+coder\\arduino.m"/* pathName */
};

static emlrtRSInfo c2_s_emlrtRSI = { 35,/* lineNo */
  "arduino",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\arduinoio\\+coder\\arduino.m"/* pathName */
};

static emlrtRSInfo c2_t_emlrtRSI = { 49,/* lineNo */
  "arduino",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\arduinoio\\+coder\\arduino.m"/* pathName */
};

static emlrtRSInfo c2_u_emlrtRSI = { 66,/* lineNo */
  "arduino",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\arduinoio\\+coder\\arduino.m"/* pathName */
};

static emlrtRSInfo c2_v_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\hwsdk\\+matlabshared\\+coder\\+hwsdk\\controller.m"/* pathName */
};

static emlrtRSInfo c2_w_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\hwsdk\\+matlabshared\\+coder\\+dio\\controller.m"/* pathName */
};

static emlrtRSInfo c2_x_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\hwsdk\\+matlabshared\\+coder\\+adc\\controller.m"/* pathName */
};

static emlrtRSInfo c2_y_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\hwsdk\\+matlabshared\\+coder\\+pwm\\controller.m"/* pathName */
};

static emlrtRSInfo c2_ab_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\hwsdk\\+matlabshared\\+coder\\+i2c\\controller.m"/* pathName */
};

static emlrtRSInfo c2_bb_emlrtRSI = { 1,/* lineNo */
  "controller_base",                   /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\hwsdk\\+matlabshared\\+coder\\+i2c\\controller_base.m"/* pathName */
};

static emlrtRSInfo c2_cb_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\hwsdk\\+matlabshared\\+coder\\+spi\\controller.m"/* pathName */
};

static emlrtRSInfo c2_db_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\hwsdk\\+matlabshared\\+coder\\+serial\\controller.m"/* pathName */
};

static emlrtRSInfo c2_eb_emlrtRSI = { 1,/* lineNo */
  "SensorCodegenUtilities",            /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\Sensor"
  "CodegenUtilities.m"                 /* pathName */
};

static emlrtRSInfo c2_fb_emlrtRSI = { 1,/* lineNo */
  "ArduinoDigitalIO",                  /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\sharedarduino\\+arduinodriver\\ArduinoDigital"
  "IO.p"                               /* pathName */
};

static emlrtRSInfo c2_gb_emlrtRSI = { 1,/* lineNo */
  "DigitalIO",                         /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\target\\shared\\devicedrivers_coder\\+matlabshared\\+devicedrivers\\+coder\\Digit"
  "alIO.p"                             /* pathName */
};

static emlrtRSInfo c2_hb_emlrtRSI = { 1,/* lineNo */
  "ArduinoAnalogInput",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\sharedarduino\\+arduinodriver\\ArduinoAnalogI"
  "nput.p"                             /* pathName */
};

static emlrtRSInfo c2_ib_emlrtRSI = { 1,/* lineNo */
  "AnalogInSingle",                    /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\target\\shared\\devicedrivers_coder\\+matlabshared\\+devicedrivers\\+coder\\Analo"
  "gInSingle.p"                        /* pathName */
};

static emlrtRSInfo c2_jb_emlrtRSI = { 1,/* lineNo */
  "PWM",                               /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\target\\shared\\devicedrivers_coder\\+matlabshared\\+devicedrivers\\+coder\\PWM.p"/* pathName */
};

static emlrtRSInfo c2_kb_emlrtRSI = { 1,/* lineNo */
  "ArduinoSPI",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\sharedarduino\\+arduinodriver\\ArduinoSPI.p"/* pathName */
};

static emlrtRSInfo c2_lb_emlrtRSI = { 1,/* lineNo */
  "SPI",                               /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\target\\shared\\devicedrivers_coder\\+matlabshared\\+devicedrivers\\+coder\\SPI.p"/* pathName */
};

static emlrtRSInfo c2_mb_emlrtRSI = { 1,/* lineNo */
  "ArduinoPinMap",                     /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\target\\supportpackages\\arduinobase\\+codertarget\\+arduinobase\\+internal\\Ardu"
  "inoPinMap.p"                        /* pathName */
};

static emlrtRSInfo c2_nb_emlrtRSI = { 1,/* lineNo */
  "Uno",                               /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\target\\supportpackages\\arduinobase\\+codertarget\\+arduinobase\\+internal\\+pin"
  "Maps\\Uno.p"                        /* pathName */
};

static emlrtRSInfo c2_ob_emlrtRSI = { 1,/* lineNo */
  "ArduinoI2C",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\sharedarduino\\+arduinodriver\\ArduinoI2C.p"/* pathName */
};

static emlrtRSInfo c2_pb_emlrtRSI = { 1,/* lineNo */
  "I2C",                               /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\target\\shared\\devicedrivers_coder\\+matlabshared\\+devicedrivers\\+coder\\I2C.p"/* pathName */
};

static emlrtRSInfo c2_qb_emlrtRSI = { 110,/* lineNo */
  "lsm9ds1",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\lsm9ds1.m"/* pathName */
};

static emlrtRSInfo c2_rb_emlrtRSI = { 115,/* lineNo */
  "lsm9ds1",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\lsm9ds1.m"/* pathName */
};

static emlrtRSInfo c2_sb_emlrtRSI = { 30,/* lineNo */
  "sensorBoard",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Board.m"                            /* pathName */
};

static emlrtRSInfo c2_tb_emlrtRSI = { 39,/* lineNo */
  "sensorBase",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Base.m"                             /* pathName */
};

static emlrtRSInfo c2_ub_emlrtRSI = { 30,/* lineNo */
  "sensorInterface",                   /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Interface.m"                        /* pathName */
};

static emlrtRSInfo c2_vb_emlrtRSI = { 1,/* lineNo */
  "System",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\System.p"/* pathName */
};

static emlrtRSInfo c2_wb_emlrtRSI = { 1,/* lineNo */
  "SystemProp",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemProp.p"/* pathName */
};

static emlrtRSInfo c2_xb_emlrtRSI = { 39,/* lineNo */
  "sensorBoard",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Board.m"                            /* pathName */
};

static emlrtRSInfo c2_yb_emlrtRSI = { 71,/* lineNo */
  "sensorBoard",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Board.m"                            /* pathName */
};

static emlrtRSInfo c2_ac_emlrtRSI = { 63,/* lineNo */
  "sensorBoard",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Board.m"                            /* pathName */
};

static emlrtRSInfo c2_bc_emlrtRSI = { 231,/* lineNo */
  "lsm9ds1",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\lsm9ds1.m"/* pathName */
};

static emlrtRSInfo c2_cc_emlrtRSI = { 232,/* lineNo */
  "lsm9ds1",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\lsm9ds1.m"/* pathName */
};

static emlrtRSInfo c2_dc_emlrtRSI = { 233,/* lineNo */
  "lsm9ds1",                           /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\lsm9ds1.m"/* pathName */
};

static emlrtRSInfo c2_ec_emlrtRSI = { 47,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_fc_emlrtRSI = { 52,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_gc_emlrtRSI = { 17,/* lineNo */
  "accelerometer",                     /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\accele"
  "rometer.m"                          /* pathName */
};

static emlrtRSInfo c2_hc_emlrtRSI = { 18,/* lineNo */
  "gyroscope",                         /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\gyrosc"
  "ope.m"                              /* pathName */
};

static emlrtRSInfo c2_ic_emlrtRSI = { 39,/* lineNo */
  "sensorUnit",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Unit.m"                             /* pathName */
};

static emlrtRSInfo c2_jc_emlrtRSI = { 67,/* lineNo */
  "sensorUnit",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Unit.m"                             /* pathName */
};

static emlrtRSInfo c2_kc_emlrtRSI = { 129,/* lineNo */
  "sensorUnit",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Unit.m"                             /* pathName */
};

static emlrtRSInfo c2_lc_emlrtRSI = { 132,/* lineNo */
  "sensorUnit",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Unit.m"                             /* pathName */
};

static emlrtRSInfo c2_mc_emlrtRSI = { 136,/* lineNo */
  "sensorUnit",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Unit.m"                             /* pathName */
};

static emlrtRSInfo c2_nc_emlrtRSI = { 16,/* lineNo */
  "device",                            /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\device"
  ".m"                                 /* pathName */
};

static emlrtRSInfo c2_oc_emlrtRSI = { 19,/* lineNo */
  "device",                            /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\device"
  ".m"                                 /* pathName */
};

static emlrtRSInfo c2_pc_emlrtRSI = { 20,/* lineNo */
  "device",                            /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\device"
  ".m"                                 /* pathName */
};

static emlrtRSInfo c2_qc_emlrtRSI = { 73,/* lineNo */
  "controller",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\hwsdk\\+matlabshared\\+coder\\+i2c\\controller.m"/* pathName */
};

static emlrtRSInfo c2_rc_emlrtRSI = { 54,/* lineNo */
  "device",                            /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\device"
  ".m"                                 /* pathName */
};

static emlrtRSInfo c2_sc_emlrtRSI = { 85,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_tc_emlrtRSI = { 86,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_uc_emlrtRSI = { 75,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_vc_emlrtRSI = { 182,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_wc_emlrtRSI = { 183,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_xc_emlrtRSI = { 39,/* lineNo */
  "device",                            /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\device"
  ".m"                                 /* pathName */
};

static emlrtRSInfo c2_yc_emlrtRSI = { 28,/* lineNo */
  "device",                            /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\device"
  ".m"                                 /* pathName */
};

static emlrtRSInfo c2_ad_emlrtRSI = { 80,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_bd_emlrtRSI = { 199,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_cd_emlrtRSI = { 200,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_dd_emlrtRSI = { 44,/* lineNo */
  "mpower",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\matfun\\mpower.m"/* pathName */
};

static emlrtRSInfo c2_ed_emlrtRSI = { 71,/* lineNo */
  "power",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pathName */
};

static emlrtRSInfo c2_fd_emlrtRSI = { 54,/* lineNo */
  "sensorBase",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Base.m"                             /* pathName */
};

static emlrtRSInfo c2_gd_emlrtRSI = { 106,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_hd_emlrtRSI = { 107,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_id_emlrtRSI = { 42,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_jd_emlrtRSI = { 47,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_kd_emlrtRSI = { 22,/* lineNo */
  "magnetometer",                      /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\magnet"
  "ometer.m"                           /* pathName */
};

static emlrtRSInfo c2_ld_emlrtRSI = { 68,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_md_emlrtRSI = { 72,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_nd_emlrtRSI = { 73,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_od_emlrtRSI = { 74,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_pd_emlrtRSI = { 75,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_qd_emlrtRSI = { 159,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_rd_emlrtRSI = { 160,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_sd_emlrtRSI = { 166,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_td_emlrtRSI = { 107,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_ud_emlrtRSI = { 108,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_vd_emlrtRSI = { 144,/* lineNo */
  "sensorBoard",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Board.m"                            /* pathName */
};

static emlrtRSInfo c2_wd_emlrtRSI = { 124,/* lineNo */
  "sensorBoard",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Board.m"                            /* pathName */
};

static emlrtRSInfo c2_xd_emlrtRSI = { 88,/* lineNo */
  "sensorBoard",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Board.m"                            /* pathName */
};

static emlrtRSInfo c2_yd_emlrtRSI = { 142,/* lineNo */
  "sensorUnit",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Unit.m"                             /* pathName */
};

static emlrtRSInfo c2_ae_emlrtRSI = { 141,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_be_emlrtRSI = { 142,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_ce_emlrtRSI = { 114,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_de_emlrtRSI = { 123,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_ee_emlrtRSI = { 175,/* lineNo */
  "sensorUnit",                        /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+matlabshared\\+sensors\\+coder\\+matlab\\sensor"
  "Unit.m"                             /* pathName */
};

static emlrtRSInfo c2_fe_emlrtRSI = { 229,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_ge_emlrtRSI = { 230,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_he_emlrtRSI = { 231,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_ie_emlrtRSI = { 80,/* lineNo */
  "bitshift",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\ops\\bitshift.m"/* pathName */
};

static emlrtRSInfo c2_je_emlrtRSI = { 127,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_ke_emlrtRSI = { 136,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_le_emlrtRSI = { 238,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_me_emlrtRSI = { 239,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_ne_emlrtRSI = { 240,/* lineNo */
  "lsm9ds1_accel_gyro",                /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_accel_gyro.m"/* pathName */
};

static emlrtRSInfo c2_oe_emlrtRSI = { 126,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_pe_emlrtRSI = { 113,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_qe_emlrtRSI = { 122,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_re_emlrtRSI = { 177,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_se_emlrtRSI = { 178,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtRSInfo c2_te_emlrtRSI = { 179,/* lineNo */
  "lsm9ds1_mag",                       /* fcnName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\shared\\sensors\\+sensors\\+internal\\lsm9ds1_mag.m"/* pathName */
};

static emlrtBCInfo c2_emlrtBCI = { 0,  /* iFirst */
  0,                                   /* iLast */
  271,                                 /* lineNo */
  45,                                  /* colNo */
  "",                                  /* aName */
  "arduino",                           /* fName */
  "C:\\ProgramData\\MATLAB\\SupportPackages\\R2022a\\toolbox\\matlab\\hardware\\supportpackages\\arduinoio\\+coder\\arduino.m",/* pName */
  0                                    /* checkKind */
};

static char_T c2_cv[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
  't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e', 'd',
  'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o', 'd', 'e',
  'g', 'e', 'n' };

static char_T c2_cv1[51] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
  't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e', 'd',
  'W', 'h', 'e', 'n', 'L', 'o', 'c', 'k', 'e', 'd', 'R', 'e', 'l', 'e', 'a', 's',
  'e', 'd', 'C', 'o', 'd', 'e', 'g', 'e', 'n' };

/* Function Declarations */
static void initialize_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void initialize_params_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void mdl_start_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void mdl_terminate_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void mdl_setup_runtime_resources_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void mdl_cleanup_runtime_resources_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void enable_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void disable_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void sf_gateway_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void ext_mode_exec_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void c2_update_jit_animation_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void c2_do_animation_call_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void set_sim_state_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance, const mxArray
   *c2_st);
static void initSimStructsc2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static c2_lsm9ds1 *c2_lsm9ds1_lsm9ds1(SFc2_arduino_imu_pitch_rollInstanceStruct *
  chartInstance, const emlrtStack *c2_sp, c2_lsm9ds1 *c2_obj, c2_arduino
  *c2_varargin_1);
static c2_matlabshared_sensors_coder_matlab_device *c2_device_device
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance, const emlrtStack
   *c2_sp, c2_matlabshared_sensors_coder_matlab_device *c2_obj, c2_arduino
   *c2_parent, uint8_T c2_deviceAddress);
static void c2_sensorBase_set_SampleRate
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance,
   c2_sensors_internal_lsm9ds1_accel_gyro *c2_obj);
static void c2_b_sensorBase_set_SampleRate
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance,
   c2_sensors_internal_lsm9ds1_mag *c2_obj);
static void c2_sensorBoard_read(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const emlrtStack *c2_sp, c2_lsm9ds1 *c2_obj, real_T
  c2_varargout_1[3], real_T c2_varargout_2[3], real_T c2_varargout_3[3]);
static void c2_SystemCore_setupAndReset
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance, const emlrtStack
   *c2_sp, c2_lsm9ds1 *c2_obj);
static void c2_SystemCore_step(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const emlrtStack *c2_sp,
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_obj, real_T c2_varargout_1[3],
  real_T c2_varargout_2[3]);
static void c2_b_SystemCore_step(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const emlrtStack *c2_sp, c2_sensors_internal_lsm9ds1_mag
  *c2_obj, real_T c2_varargout_1[3]);
static real_T c2_sqrt(SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance,
                      const emlrtStack *c2_sp, real_T c2_x);
static real_T c2_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_b_pitch, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static real_T c2_c_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_b_pitchAccelOffset, const char_T
  *c2_identifier, boolean_T *c2_svPtr);
static real_T c2_d_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T *c2_svPtr);
static uint8_T c2_e_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_arduino_imu_pitch_roll, const
  char_T *c2_identifier);
static uint8_T c2_f_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_slStringInitializeDynamicBuffers
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance);
static void c2_chart_data_browse_helper
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance, int32_T
   c2_ssIdNumber, const mxArray **c2_mxData, uint8_T *c2_isValueTooBig);
static void c2_b_sqrt(SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance,
                      const emlrtStack *c2_sp, real_T *c2_x);
static void init_dsm_address_info(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_arduinoObj_not_empty = false;
  chartInstance->c2_imuObj_not_empty = false;
  chartInstance->c2_prevTimestamp_not_empty = false;
  chartInstance->c2_pitchAngle_not_empty = false;
  chartInstance->c2_rollAngle_not_empty = false;
  chartInstance->c2_pitchAccelOffset_not_empty = false;
  chartInstance->c2_rollAccelOffset_not_empty = false;
  chartInstance->c2_pitchGyroOffset_not_empty = false;
  chartInstance->c2_rollGyroOffset_not_empty = false;
  chartInstance->c2_is_active_c2_arduino_imu_pitch_roll = 0U;
  chartInstance->c2_imuObj._pobj1.matlabCodegenIsDeleted = true;
  chartInstance->c2_imuObj._pobj0.matlabCodegenIsDeleted = true;
  chartInstance->c2_imuObj.matlabCodegenIsDeleted = true;
}

static void initialize_params_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void mdl_start_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void mdl_terminate_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void mdl_setup_runtime_resources_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  static const uint32_T c2_decisionTxtEndIdx = 0U;
  static const uint32_T c2_decisionTxtStartIdx = 0U;
  setDebuggerFlag(chartInstance->S, true);
  setDataBrowseFcn(chartInstance->S, (void *)&c2_chart_data_browse_helper);
  chartInstance->c2_RuntimeVar = sfListenerCacheSimStruct(chartInstance->S);
  sfListenerInitializeRuntimeVars(chartInstance->c2_RuntimeVar,
    &chartInstance->c2_IsDebuggerActive,
    &chartInstance->c2_IsSequenceViewerPresent, 0, 0,
    &chartInstance->c2_mlFcnLineNumber, &chartInstance->c2_IsHeatMapPresent, 0);
  sim_mode_is_external(chartInstance->S);
  covrtCreateStateflowInstanceData(chartInstance->c2_covrtInstance, 1U, 0U, 1U,
    4U);
  covrtChartInitFcn(chartInstance->c2_covrtInstance, 0U, false, false, false);
  covrtStateInitFcn(chartInstance->c2_covrtInstance, 0U, 0U, false, false, false,
                    0U, &c2_decisionTxtStartIdx, &c2_decisionTxtEndIdx);
  covrtTransInitFcn(chartInstance->c2_covrtInstance, 0U, 0, NULL, NULL, 0U, NULL);
  covrtEmlInitFcn(chartInstance->c2_covrtInstance, "", 4U, 0U, 1U, 0U, 6U, 0U,
                  0U, 0U, 1U, 0U, 0U, 0U);
  covrtEmlFcnInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 0U,
                     "eML_blk_kernel", 0, -1, 2858);
  covrtEmlIfInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 0U, 493, 515, -1,
                    547, false);
  covrtEmlIfInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 1U, 548, 566, -1,
                    604, false);
  covrtEmlIfInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 2U, 745, 773, -1,
                    1620, false);
  covrtEmlIfInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 3U, 989, 1002, 1094,
                    1144, false);
  covrtEmlIfInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 4U, 1622, 1644, -1,
                    1668, false);
  covrtEmlIfInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 5U, 1670, 1691, -1,
                    1714, false);
  covrtEmlForInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 0U, 969, 984, 1451);
  covrtEmlRelationalInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 0U, 992,
    1002, -1, 0U);
}

static void mdl_cleanup_runtime_resources_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  c2_arduinodriver_ArduinoI2C *c2_bb_obj;
  c2_arduinodriver_ArduinoI2C *c2_db_obj;
  c2_arduinodriver_ArduinoI2C *c2_t_obj;
  c2_arduinodriver_ArduinoI2C *c2_y_obj;
  c2_lsm9ds1 *c2_c_obj;
  c2_lsm9ds1 *c2_d_obj;
  c2_lsm9ds1 *c2_h_obj;
  c2_lsm9ds1 *c2_j_obj;
  c2_lsm9ds1 *c2_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_cb_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_r_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_w_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_x_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_e_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_i_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_k_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_m_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_o_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_q_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_s_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_u_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_ab_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_b_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_f_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_g_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_l_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_n_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_p_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_v_obj;
  c2_obj = &chartInstance->c2_imuObj;
  if (!c2_obj->matlabCodegenIsDeleted) {
    c2_obj->matlabCodegenIsDeleted = true;
    c2_c_obj = c2_obj;
    c2_d_obj = c2_c_obj;
    if (c2_d_obj->isInitialized == 1) {
      c2_d_obj->isInitialized = 2;
      c2_h_obj = c2_d_obj;
      if (c2_h_obj->isSetupComplete) {
        c2_j_obj = c2_h_obj;
        c2_m_obj = c2_j_obj->SensorObjects.f1;
        if (c2_m_obj->isInitialized == 1) {
          c2_m_obj->isInitialized = 2;
          c2_q_obj = c2_m_obj;
          if (c2_q_obj->isSetupComplete) {
            c2_u_obj = c2_q_obj;
            c2_u_obj->SamplesRead = 0.0;
            c2_x_obj = c2_u_obj->Device;
            c2_bb_obj = c2_x_obj->InterfaceObj;
            MW_I2C_Close(c2_bb_obj->MW_I2C_HANDLE);
          }
        }

        c2_p_obj = c2_j_obj->SensorObjects.f2;
        if (c2_p_obj->isInitialized == 1) {
          c2_p_obj->isInitialized = 2;
          c2_v_obj = c2_p_obj;
          if (c2_v_obj->isSetupComplete) {
            c2_ab_obj = c2_v_obj;
            c2_ab_obj->SamplesRead = 0.0;
            c2_cb_obj = c2_ab_obj->Device;
            c2_db_obj = c2_cb_obj->InterfaceObj;
            MW_I2C_Close(c2_db_obj->MW_I2C_HANDLE);
          }
        }

        c2_j_obj->SamplesRead = 0.0;
      }
    }
  }

  c2_b_obj = &chartInstance->c2_imuObj._pobj0;
  if (!c2_b_obj->matlabCodegenIsDeleted) {
    c2_b_obj->matlabCodegenIsDeleted = true;
    c2_f_obj = c2_b_obj;
    c2_g_obj = c2_f_obj;
    if (c2_g_obj->isInitialized == 1) {
      c2_g_obj->isInitialized = 2;
      c2_l_obj = c2_g_obj;
      if (c2_l_obj->isSetupComplete) {
        c2_n_obj = c2_l_obj;
        c2_n_obj->SamplesRead = 0.0;
        c2_r_obj = c2_n_obj->Device;
        c2_t_obj = c2_r_obj->InterfaceObj;
        MW_I2C_Close(c2_t_obj->MW_I2C_HANDLE);
      }
    }
  }

  c2_e_obj = &chartInstance->c2_imuObj._pobj1;
  if (!c2_e_obj->matlabCodegenIsDeleted) {
    c2_e_obj->matlabCodegenIsDeleted = true;
    c2_i_obj = c2_e_obj;
    c2_k_obj = c2_i_obj;
    if (c2_k_obj->isInitialized == 1) {
      c2_k_obj->isInitialized = 2;
      c2_o_obj = c2_k_obj;
      if (c2_o_obj->isSetupComplete) {
        c2_s_obj = c2_o_obj;
        c2_s_obj->SamplesRead = 0.0;
        c2_w_obj = c2_s_obj->Device;
        c2_y_obj = c2_w_obj->InterfaceObj;
        MW_I2C_Close(c2_y_obj->MW_I2C_HANDLE);
      }
    }
  }

  sfListenerLightTerminate(chartInstance->c2_RuntimeVar);
  covrtDeleteStateflowInstanceData(chartInstance->c2_covrtInstance);
}

static void enable_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void sf_gateway_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  static char_T c2_b_cv[4] = { 's', 't', 'e', 'p' };

  c2_arduino *c2_b_obj;
  c2_arduino *c2_b_this;
  c2_arduino *c2_c_obj;
  c2_arduino *c2_c_this;
  c2_arduino *c2_d_obj;
  c2_arduino *c2_d_this;
  c2_arduino *c2_e_obj;
  c2_arduino *c2_e_this;
  c2_arduino *c2_f_this;
  c2_arduino *c2_g_this;
  c2_arduino *c2_h_obj;
  c2_arduino *c2_h_this;
  c2_arduino *c2_j_obj;
  c2_arduino *c2_k_obj;
  c2_arduino *c2_l_obj;
  c2_arduino *c2_m_obj;
  c2_arduino *c2_n_obj;
  c2_arduino *c2_obj;
  c2_arduino *c2_this;
  c2_arduinodriver_ArduinoI2C *c2_rv[1];
  c2_arduinodriver_ArduinoI2C *c2_e_r;
  c2_arduinodriver_ArduinoI2C *c2_i_this;
  c2_arduinodriver_ArduinoI2C *c2_o_obj;
  c2_arduinodriver_ArduinoI2C *c2_p_obj;
  c2_cell_wrap_11 c2_localOutputs[2];
  c2_lsm9ds1 *c2_f_obj;
  c2_lsm9ds1 *c2_g_obj;
  c2_lsm9ds1 *c2_i_obj;
  emlrtStack c2_b_st;
  emlrtStack c2_c_st;
  emlrtStack c2_d_st;
  emlrtStack c2_e_st;
  emlrtStack c2_st = { NULL,           /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  const mxArray *c2_e_y = NULL;
  const mxArray *c2_f_y = NULL;
  const mxArray *c2_h_y = NULL;
  real_T c2_a__1[3];
  real_T c2_a__2[3];
  real_T c2_accel[3];
  real_T c2_gyro[3];
  real_T c2_unusedExpr[3];
  real_T c2_a;
  real_T c2_accelPitchAngle;
  real_T c2_accelRollAngle;
  real_T c2_b;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_b_i;
  real_T c2_b_pitch;
  real_T c2_b_r;
  real_T c2_b_roll;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_c_r;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_d_r;
  real_T c2_d_x;
  real_T c2_d_y;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_g_y;
  real_T c2_gyroPitchAngleDiff;
  real_T c2_gyroRollAngleDiff;
  real_T c2_h_x;
  real_T c2_i_y;
  real_T c2_j_y;
  real_T c2_k_y;
  real_T c2_r;
  real_T c2_x;
  real_T c2_y;
  int32_T c2_c_i;
  int32_T c2_i;
  int32_T c2_i1;
  int32_T c2_i2;
  int32_T c2_i3;
  c2_st.tls = chartInstance->c2_fEmlrtCtx;
  c2_b_st.prev = &c2_st;
  c2_b_st.tls = c2_st.tls;
  c2_c_st.prev = &c2_b_st;
  c2_c_st.tls = c2_b_st.tls;
  c2_d_st.prev = &c2_c_st;
  c2_d_st.tls = c2_c_st.tls;
  c2_e_st.prev = &c2_d_st;
  c2_e_st.tls = c2_d_st.tls;
  chartInstance->c2_JITTransitionAnimation[0] = 0U;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_sfEvent = CALL_EVENT;
  covrtEmlFcnEval(chartInstance->c2_covrtInstance, 4U, 0, 0);
  if (covrtEmlIfEval(chartInstance->c2_covrtInstance, 4U, 0, 0,
                     !chartInstance->c2_arduinoObj_not_empty)) {
    c2_b_st.site = &c2_f_emlrtRSI;
    c2_obj = &chartInstance->c2_arduinoObj;
    c2_b_obj = c2_obj;
    c2_c_st.site = &c2_o_emlrtRSI;
    c2_c_obj = c2_b_obj;
    c2_b_obj = c2_c_obj;
    c2_d_st.site = &c2_v_emlrtRSI;
    c2_this = c2_b_obj;
    c2_b_obj = c2_this;
    c2_c_st.site = &c2_o_emlrtRSI;
    c2_d_obj = c2_b_obj;
    c2_b_obj = c2_d_obj;
    c2_d_st.site = &c2_v_emlrtRSI;
    c2_b_this = c2_b_obj;
    c2_b_obj = c2_b_this;
    c2_c_st.site = &c2_o_emlrtRSI;
    c2_e_obj = c2_b_obj;
    c2_b_obj = c2_e_obj;
    c2_d_st.site = &c2_v_emlrtRSI;
    c2_c_this = c2_b_obj;
    c2_b_obj = c2_c_this;
    c2_c_st.site = &c2_o_emlrtRSI;
    c2_h_obj = c2_b_obj;
    c2_b_obj = c2_h_obj;
    c2_d_st.site = &c2_v_emlrtRSI;
    c2_d_this = c2_b_obj;
    c2_b_obj = c2_d_this;
    c2_c_st.site = &c2_o_emlrtRSI;
    c2_j_obj = c2_b_obj;
    c2_b_obj = c2_j_obj;
    c2_d_st.site = &c2_ab_emlrtRSI;
    c2_k_obj = c2_b_obj;
    c2_b_obj = c2_k_obj;
    c2_e_st.site = &c2_bb_emlrtRSI;
    c2_e_this = c2_b_obj;
    c2_b_obj = c2_e_this;
    c2_c_st.site = &c2_o_emlrtRSI;
    c2_l_obj = c2_b_obj;
    c2_b_obj = c2_l_obj;
    c2_d_st.site = &c2_v_emlrtRSI;
    c2_f_this = c2_b_obj;
    c2_b_obj = c2_f_this;
    c2_c_st.site = &c2_o_emlrtRSI;
    c2_m_obj = c2_b_obj;
    c2_b_obj = c2_m_obj;
    c2_d_st.site = &c2_v_emlrtRSI;
    c2_g_this = c2_b_obj;
    c2_b_obj = c2_g_this;
    c2_c_st.site = &c2_o_emlrtRSI;
    c2_n_obj = c2_b_obj;
    c2_b_obj = c2_n_obj;
    c2_d_st.site = &c2_eb_emlrtRSI;
    c2_h_this = c2_b_obj;
    c2_b_obj = c2_h_this;
    c2_c_st.site = &c2_p_emlrtRSI;
    c2_d_st.site = &c2_fb_emlrtRSI;
    c2_e_st.site = &c2_gb_emlrtRSI;
    c2_d_st.site = &c2_fb_emlrtRSI;
    c2_c_st.site = &c2_q_emlrtRSI;
    c2_d_st.site = &c2_hb_emlrtRSI;
    c2_e_st.site = &c2_ib_emlrtRSI;
    c2_d_st.site = &c2_hb_emlrtRSI;
    c2_c_st.site = &c2_r_emlrtRSI;
    c2_d_st.site = &c2_jb_emlrtRSI;
    c2_c_st.site = &c2_s_emlrtRSI;
    c2_d_st.site = &c2_kb_emlrtRSI;
    c2_e_st.site = &c2_lb_emlrtRSI;
    c2_c_st.site = &c2_t_emlrtRSI;
    c2_d_st.site = &c2_mb_emlrtRSI;
    c2_d_st.site = &c2_mb_emlrtRSI;
    c2_e_st.site = &c2_nb_emlrtRSI;
    c2_b_obj->Aref = 5.0;
    c2_c_st.site = &c2_u_emlrtRSI;
    c2_o_obj = &c2_b_obj->_pobj0;
    c2_e_r = c2_o_obj;
    c2_d_st.site = &c2_ob_emlrtRSI;
    c2_p_obj = c2_e_r;
    c2_e_r = c2_p_obj;
    c2_e_st.site = &c2_pb_emlrtRSI;
    c2_i_this = c2_e_r;
    c2_e_r = c2_i_this;
    c2_e_r->MW_I2C_HANDLE = NULL;
    c2_rv[0] = c2_e_r;
    c2_b_obj->I2CDriverObj[0] = c2_rv[0];
    chartInstance->c2_arduinoObj_not_empty = true;
  }

  if (covrtEmlIfEval(chartInstance->c2_covrtInstance, 4U, 0, 1,
                     !chartInstance->c2_imuObj_not_empty)) {
    c2_b_st.site = &c2_g_emlrtRSI;
    c2_lsm9ds1_lsm9ds1(chartInstance, &c2_b_st, &chartInstance->c2_imuObj,
                       &chartInstance->c2_arduinoObj);
    chartInstance->c2_imuObj_not_empty = true;
  }

  if (covrtEmlIfEval(chartInstance->c2_covrtInstance, 4U, 0, 2,
                     !chartInstance->c2_pitchAccelOffset_not_empty)) {
    chartInstance->c2_pitchAccelOffset = 0.0;
    chartInstance->c2_pitchAccelOffset_not_empty = true;
    chartInstance->c2_pitchGyroOffset = 0.0;
    chartInstance->c2_pitchGyroOffset_not_empty = true;
    chartInstance->c2_rollAccelOffset = 0.0;
    chartInstance->c2_rollAccelOffset_not_empty = true;
    chartInstance->c2_rollGyroOffset = 0.0;
    chartInstance->c2_rollGyroOffset_not_empty = true;
    chartInstance->c2_prevTimestamp = 0.0;
    chartInstance->c2_prevTimestamp_not_empty = true;
    for (c2_i = 0; c2_i < 100; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      covrtEmlForEval(chartInstance->c2_covrtInstance, 4U, 0, 0, 1);
      if (covrtEmlIfEval(chartInstance->c2_covrtInstance, 4U, 0, 3,
                         covrtRelationalopUpdateFcn
                         (chartInstance->c2_covrtInstance, 4U, 0U, 0U, c2_b_i,
                          100.0, -1, 0U, c2_b_i == 100.0))) {
        c2_b_st.site = &c2_h_emlrtRSI;
        c2_sensorBoard_read(chartInstance, &c2_b_st, &chartInstance->c2_imuObj,
                            c2_accel, c2_gyro, c2_a__1);
        chartInstance->c2_prevTimestamp = 0.0;
      } else {
        c2_b_st.site = &c2_i_emlrtRSI;
        c2_f_obj = &chartInstance->c2_imuObj;
        c2_c_st.site = &c2_wd_emlrtRSI;
        c2_g_obj = c2_f_obj;
        if (c2_g_obj->isInitialized == 2) {
          c2_e_y = NULL;
          sf_mex_assign(&c2_e_y, sf_mex_create("y", c2_cv, 10, 0U, 1U, 0U, 2, 1,
            45), false);
          c2_f_y = NULL;
          sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_cv, 10, 0U, 1U, 0U, 2, 1,
            45), false);
          c2_h_y = NULL;
          sf_mex_assign(&c2_h_y, sf_mex_create("y", c2_b_cv, 10, 0U, 1U, 0U, 2,
            1, 4), false);
          sf_mex_call(&c2_c_st, &c2_b_emlrtMCI, "error", 0U, 2U, 14, c2_e_y, 14,
                      sf_mex_call(&c2_c_st, NULL, "getString", 1U, 1U, 14,
            sf_mex_call(&c2_c_st, NULL, "message", 1U, 2U, 14, c2_f_y, 14,
                        c2_h_y)));
        }

        if (c2_g_obj->isInitialized != 1) {
          c2_d_st.site = &c2_b_emlrtRSI;
          c2_SystemCore_setupAndReset(chartInstance, &c2_d_st, c2_g_obj);
        }

        c2_d_st.site = &c2_b_emlrtRSI;
        c2_i_obj = c2_g_obj;
        c2_e_st.site = &c2_xd_emlrtRSI;
        c2_SystemCore_step(chartInstance, &c2_e_st, c2_i_obj->SensorObjects.f1,
                           c2_a__1, c2_a__2);
        for (c2_c_i = 0; c2_c_i < 3; c2_c_i++) {
          c2_localOutputs[0].f1[c2_c_i] = c2_a__1[c2_c_i];
        }

        for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
          c2_localOutputs[1].f1[c2_i1] = c2_a__2[c2_i1];
        }

        for (c2_i2 = 0; c2_i2 < 3; c2_i2++) {
          c2_accel[c2_i2] = c2_localOutputs[0].f1[c2_i2];
        }

        for (c2_i3 = 0; c2_i3 < 3; c2_i3++) {
          c2_gyro[c2_i3] = c2_localOutputs[1].f1[c2_i3];
        }

        c2_e_st.site = &c2_xd_emlrtRSI;
        c2_b_SystemCore_step(chartInstance, &c2_e_st, c2_i_obj->SensorObjects.f2,
                             c2_unusedExpr);
        c2_i_obj->SamplesRead++;
      }

      c2_d_y = c2_accel[0];
      c2_d_x = c2_accel[1] * c2_accel[1] + c2_accel[2] * c2_accel[2];
      c2_b_st.site = &c2_j_emlrtRSI;
      c2_b_sqrt(chartInstance, &c2_b_st, &c2_d_x);
      c2_c_a = c2_d_y;
      c2_c_b = c2_d_x;
      c2_i_y = c2_c_a;
      c2_f_x = c2_c_b;
      c2_c_r = muDoubleScalarAtan2(c2_i_y, c2_f_x);
      chartInstance->c2_pitchAccelOffset += c2_c_r;
      c2_j_y = -c2_accel[1];
      c2_g_x = c2_accel[0] * c2_accel[0] + c2_accel[2] * c2_accel[2];
      c2_b_st.site = &c2_k_emlrtRSI;
      c2_b_sqrt(chartInstance, &c2_b_st, &c2_g_x);
      c2_d_a = c2_j_y;
      c2_d_b = c2_g_x;
      c2_k_y = c2_d_a;
      c2_h_x = c2_d_b;
      c2_d_r = muDoubleScalarAtan2(c2_k_y, c2_h_x);
      chartInstance->c2_rollAccelOffset += c2_d_r;
      chartInstance->c2_pitchGyroOffset += c2_gyro[1];
      chartInstance->c2_rollGyroOffset += c2_gyro[0];
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    covrtEmlForEval(chartInstance->c2_covrtInstance, 4U, 0, 0, 0);
    chartInstance->c2_pitchAccelOffset /= 100.0;
    chartInstance->c2_rollAccelOffset /= 100.0;
    chartInstance->c2_pitchGyroOffset /= 100.0;
    chartInstance->c2_rollGyroOffset /= 100.0;
  }

  if (covrtEmlIfEval(chartInstance->c2_covrtInstance, 4U, 0, 4,
                     !chartInstance->c2_pitchAngle_not_empty)) {
    chartInstance->c2_pitchAngle = 0.0;
    chartInstance->c2_pitchAngle_not_empty = true;
  }

  if (covrtEmlIfEval(chartInstance->c2_covrtInstance, 4U, 0, 5,
                     !chartInstance->c2_rollAngle_not_empty)) {
    chartInstance->c2_rollAngle = 0.0;
    chartInstance->c2_rollAngle_not_empty = true;
  }

  c2_b_st.site = &c2_l_emlrtRSI;
  c2_sensorBoard_read(chartInstance, &c2_b_st, &chartInstance->c2_imuObj,
                      c2_accel, c2_gyro, c2_a__2);
  c2_y = c2_accel[0];
  c2_x = c2_accel[1] * c2_accel[1] + c2_accel[2] * c2_accel[2];
  c2_b_st.site = &c2_m_emlrtRSI;
  c2_b_sqrt(chartInstance, &c2_b_st, &c2_x);
  c2_a = c2_y;
  c2_b = c2_x;
  c2_b_y = c2_a;
  c2_b_x = c2_b;
  c2_r = muDoubleScalarAtan2(c2_b_y, c2_b_x);
  c2_accelPitchAngle = c2_r - chartInstance->c2_pitchAccelOffset;
  c2_c_y = -c2_accel[1];
  c2_c_x = c2_accel[0] * c2_accel[0] + c2_accel[2] * c2_accel[2];
  c2_b_st.site = &c2_n_emlrtRSI;
  c2_b_sqrt(chartInstance, &c2_b_st, &c2_c_x);
  c2_b_a = c2_c_y;
  c2_b_b = c2_c_x;
  c2_g_y = c2_b_a;
  c2_e_x = c2_b_b;
  c2_b_r = muDoubleScalarAtan2(c2_g_y, c2_e_x);
  c2_accelRollAngle = c2_b_r - chartInstance->c2_rollAccelOffset;
  c2_gyro[0] -= chartInstance->c2_rollGyroOffset;
  c2_gyro[1] -= chartInstance->c2_pitchGyroOffset;
  c2_gyroPitchAngleDiff = c2_gyro[1] * (0.0 - chartInstance->c2_prevTimestamp);
  c2_gyroRollAngleDiff = c2_gyro[0] * (0.0 - chartInstance->c2_prevTimestamp);
  chartInstance->c2_prevTimestamp = 0.0;
  chartInstance->c2_pitchAngle = 0.98 * (chartInstance->c2_pitchAngle +
    c2_gyroPitchAngleDiff) + 0.020000000000000018 * c2_accelPitchAngle;
  chartInstance->c2_rollAngle = 0.98 * (chartInstance->c2_rollAngle +
    c2_gyroRollAngleDiff) + 0.020000000000000018 * c2_accelRollAngle;
  c2_b_pitch = chartInstance->c2_pitchAngle * 180.0 / 3.1415926535897931;
  c2_b_roll = chartInstance->c2_rollAngle * 180.0 / 3.1415926535897931;
  *chartInstance->c2_pitch = c2_b_pitch;
  *chartInstance->c2_roll = c2_b_roll;
  c2_do_animation_call_c2_arduino_imu_pitch_roll(chartInstance);
  covrtSigUpdateFcn(chartInstance->c2_covrtInstance, 0U,
                    *chartInstance->c2_pitch);
  covrtSigUpdateFcn(chartInstance->c2_covrtInstance, 1U, *chartInstance->c2_roll);
}

static void ext_mode_exec_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_update_jit_animation_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_do_animation_call_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  sfDoAnimationWrapper(chartInstance->S, false, true);
  sfDoAnimationWrapper(chartInstance->S, false, false);
}

static const mxArray *get_sim_state_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  const mxArray *c2_d_y = NULL;
  const mxArray *c2_e_y = NULL;
  const mxArray *c2_f_y = NULL;
  const mxArray *c2_g_y = NULL;
  const mxArray *c2_h_y = NULL;
  const mxArray *c2_i_y = NULL;
  const mxArray *c2_j_y = NULL;
  const mxArray *c2_k_y = NULL;
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(10, 1), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", chartInstance->c2_pitch, 0, 0U, 0U,
    0U, 0), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", chartInstance->c2_roll, 0, 0U, 0U,
    0U, 0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_d_y = NULL;
  if (!chartInstance->c2_pitchAccelOffset_not_empty) {
    sf_mex_assign(&c2_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_d_y, sf_mex_create("y",
      &chartInstance->c2_pitchAccelOffset, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_e_y = NULL;
  if (!chartInstance->c2_pitchAccelOffset_not_empty) {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", &chartInstance->c2_pitchAngle, 0,
      0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_f_y = NULL;
  if (!chartInstance->c2_pitchAccelOffset_not_empty) {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", &chartInstance->c2_pitchGyroOffset,
      0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_g_y = NULL;
  if (!chartInstance->c2_pitchAccelOffset_not_empty) {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", &chartInstance->c2_prevTimestamp,
      0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_h_y = NULL;
  if (!chartInstance->c2_pitchAccelOffset_not_empty) {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", &chartInstance->c2_rollAccelOffset,
      0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 6, c2_h_y);
  c2_i_y = NULL;
  if (!chartInstance->c2_pitchAccelOffset_not_empty) {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", &chartInstance->c2_rollAngle, 0,
      0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 7, c2_i_y);
  c2_j_y = NULL;
  if (!chartInstance->c2_pitchAccelOffset_not_empty) {
    sf_mex_assign(&c2_j_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_j_y, sf_mex_create("y", &chartInstance->c2_rollGyroOffset,
      0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 8, c2_j_y);
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y",
    &chartInstance->c2_is_active_c2_arduino_imu_pitch_roll, 3, 0U, 0U, 0U, 0),
                false);
  sf_mex_setcell(c2_y, 9, c2_k_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance, const mxArray
   *c2_st)
{
  const mxArray *c2_u;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  *chartInstance->c2_pitch = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 0)), "pitch");
  *chartInstance->c2_roll = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 1)), "roll");
  chartInstance->c2_pitchAccelOffset = c2_c_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 2)), "pitchAccelOffset",
    &chartInstance->c2_pitchAccelOffset_not_empty);
  chartInstance->c2_pitchAngle = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 3)), "pitchAngle",
    &chartInstance->c2_pitchAngle_not_empty);
  chartInstance->c2_pitchGyroOffset = c2_c_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 4)), "pitchGyroOffset",
    &chartInstance->c2_pitchGyroOffset_not_empty);
  chartInstance->c2_prevTimestamp = c2_c_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 5)), "prevTimestamp",
    &chartInstance->c2_prevTimestamp_not_empty);
  chartInstance->c2_rollAccelOffset = c2_c_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 6)), "rollAccelOffset",
    &chartInstance->c2_rollAccelOffset_not_empty);
  chartInstance->c2_rollAngle = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 7)), "rollAngle",
    &chartInstance->c2_rollAngle_not_empty);
  chartInstance->c2_rollGyroOffset = c2_c_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 8)), "rollGyroOffset",
    &chartInstance->c2_rollGyroOffset_not_empty);
  chartInstance->c2_is_active_c2_arduino_imu_pitch_roll = c2_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 9)),
     "is_active_c2_arduino_imu_pitch_roll");
  sf_mex_destroy(&c2_u);
  sf_mex_destroy(&c2_st);
}

static void initSimStructsc2_arduino_imu_pitch_roll
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

const mxArray *sf_c2_arduino_imu_pitch_roll_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  const char_T *c2_data[51] = {
    "789ced9d5b6c2bc779c7a9c0866f3d8ed236ae5b146d121b4510b9aa444947e7186d51921275a80b8fc48b6e517066458ea8d559eeae962bead2b4a50d373e8e"
    "2fc7a963c3f0439e0a1b085c202d50a0401bf8d54f456bc0a9ddc4ed79ec5bdc37374f5d727728ee8ae3a1b9a3d9ddd9d91769f971e6ffcd0cf7371f3fceee24",
    "46722b238944e2d1847dfca4f8a5cedf2bcef9a8f3f74b09f7e1b58f387f65cf393aee4fdce72a87ec779dbf154d35e189699fa8521d764b56b5baac4aaa593a"
    "d561c2800d4d69c26ac7b2272bb024d761b1f724df3eab677b4cdd93b6a9fd7f661f566e178fea0963bf71eea1d27bd2ed8f8f30edbd6fc0fe6878ce139ef721",
    "fbb77399ef649ede5935b49a21d5e72453da5949959653e99de291ae6b86b92a556e4b35d8d8292427924969c7d43465573bd9a94ba622edeeec4b46f55832e0"
    "4ec37ebb8ede6ebd7e24ab9aaced8c55b42a34d00be375573beff86ce713847622bbed6ec3721756c73b0e8def1f37aab7c7db1f0043531468b8fc0243fae53d",
    "707ea103e9dd1b520fd5ff97043d64a737de9daedce974e2ce586ff7a2011fb36de71ddc1e7a52ff7e79c0f67aff9ebfffc1cedf3ffee17f744cacf45a7ffaf7"
    "75967ae8084aef0453dfa09fd7c7307aa31e7bbd69e6f7ccd2494ac9eecf5cadcc55cdd5c3ecdcb91fab041d921f09cc39abfa05e7076be7d709ed44f63e9caf",
    "ca9a97f2d1e5fc77097ac8ce92f356077b28cf2fe7137f76b7c254cf3978e7fc9aa6964ce55039399c9d4c9e2c97aa2b7ae3705e705e70defdbecfe1bc54ad08"
    "ce5f26e7ad0e8e0de75ba9fa164b3d74f0cef999cda9c55d68e4575616f70bcdfd89ad4ca6b47e43705e70defdbecfe1bc7e5c179cbf4cce5b1d1c1fceeb1ffe",
    "034b3d7470cff9d9c303453696b760f6283bafef4e66d76b798ee2f97b98f251beeee5243ebef3cbfb3f20b417d9fbf0de72ab87f7b776a5060c8ef7ff33a41e"
    "aaff19821eb20737ee9d0e76667b7eb99ff8e0e56799ea3907efdc3fdc9e2eaab9b3e583a5c2dcb5e65469b1b4bd5759e087fb22be1fac9d3ee27b37ef9df789",
    "f89ecd3c0f30ed892ae75b6f3f7dcc520f1d51e5fc57317aa31efb815182ba797dbe944f559aa9f9b99564764e4e08ce0bcebbdff7399c6fe8b2e0fc6572deea"
    "e0d8703e71e785ff66aae71c51e5fca0f1fc96542e55a5edf2e275bd74a6ceaaa632030b229e8f1de79f24b413d9fb711e1ab2a478501f55ceff15410fd99972",
    "bed3c16ed473cbf9775efb98a99e73f0cef935b87cb23f7fb861547227e9cde24a6d6ea698c908ce478df36ffa6ce735423b91ddc5f906541b9ad170786f9bc6"
    "8b9d1733d64b35a8964d59914d193682e2fe6743eaa1fadf20e8213b6dee3b5deb257ff765670a704af7ef7396ebee173f623c1f7ca77ec454cf39789f0feaf2",
    "c9f1d5e27ad9c86cec9637b7a715282f15395a770f30e52f8b3ffa907a239ed7717ac8ee2c85b4e7864470ed8dcb7c0b30ed103c1d50cf3978e76971663179d8"
    "9c9092d5bdb3eb66f5606fa9a465d282a7de6350be0cab87ea7f00ab675b1041836adfbf0da987ea57087ac8ce809f2e70b2bace33bfff115b6e26ef7c97a99e",
    "73f0cecdd935a84fe7b6e5357965feb8b9279dcd4f54758ed68bfb5d4f76c7739ef0bc0fd92fed3ab7bf913ad7f8ce98f34fd5909b56cc94b2cfe6e49a6c4a4a"
    "eee6b88edafdaacf767f8bd06e6477e529aab02957a0ed1bca56747db3df1f14ef3f1d520fd5ff7d821eb20ffd393025a3064d949670f5e42d571202a5285c6f",
    "414174cf27c1dd7e80691f2d7ebdce388e6e7d3cf29f2cf51e2efd8c6dfb9e9abddaabc76a3eb8512c4ce8cb4ba9a9f4ead656ae54924f669a69f17b64e4be37"
    "b77cb6f3f708ed4476d77c34ee9d8fa2bb8e3caaf33ec0b48b16977ece9af34fc00f58ea8dfd88713ea8f8d641af1e2bce4bcd4c69f168725781d3a9f2d45121",
    "09371a1b597e381ffa788fc6f59f522545abe554fdc8f4c67b7ee3ff3f22b41fd989f13ff2b128ab35054636febfeb39f7ea217bc0f1bfbbb7ad0f05aff17f22"
    "23335da7c2fa7782d643bf78b7578fd5bca03492b9d9d9e526dc573737d5956ca12c652b1ccd0b22fe1fac9d5f23b413d9fbc6ff3df35254791ff5f91f60da27",
    "be070ca637faa30fd9ce679fbcf8b55e3d56bc3717d3e6e6cae9ac7afdba7275feb03971a048331cad43890bef03597fde2fde5fdd58e9f50b0ce997f76095f7"
    "798ea087ec01c7f9562fdbc4e736bed7df629adfffe775b6f9fdc413738ff7eab1baaf74b53eb3727b61664f353756d7eac5fde3fc9e3a9d10bc8f1befbf4168",
    "27b2bbe37be7cca24faa5a5d96770dc9384db8fc0243fae53d58c5f92f11f4903d9838bf5f7fb3e53eef71fe073719ffaefb174f28bd7aacb83fb9a06e67a797"
    "9726b313a92d6363f534a5ceab097eb8ef37ee7b1653ffa8c71ee8f7fde26acefb3d3f3471bfe55baf5f6048bfbc8788fb5d717ff71320e27e3a7a0f1c31cef3",
    "bcb1f0935e3d56799e85a5f9bdccd519756e6b352b55f7568cd9e47241e4792217f7fbfdbc3c4e6827b2f7cdeb5bf4892adfa33abf034cbb447c3f98debfbcc9"
    "b87d7ff22bb9578f15dff717b5f26efa48353756d29b1bf327d999f58d8278be40e4f8feaacf768e13da89ec1d1fec981465753acfa59555131aaaa420e4afca",
    "ea8aa4d3bb2fec3ecff9b97fb6a5627560fb6fe4d6edf41fee769f3a03eebc71cc6541ddbde3ea6e86eb763e61cdfffb9ffe57967ae8e0fd3eaedc61a9b85bd8"
    "340cfd46fa742d97d9d8bbb67ac4d17d5c71e780df79e12942fb919d382fe81dcf1ae3e5cee30f229bf77f85a087ec417f1ec69cfedeb1fa7b7c807938aaf342",
    "e2c7fff721533de7e07e5ed8de5f58cc6777a7e0fcf2da762e7558ce1fe6399a17629117c82533a1cdfb5bbef5fa0586f4cb7b88bcbf2befdffd0488bc3f1dbd"
    "b8ace79f9a5e3a3b2b2f2f6e6f578e6f1f64a55db3602e88f5fc91cb0b059af7b7e81355be47757e07987689bcff607ac99f33fe5df75753eff5eab15ad7a36c",
    "e7d4935c656b552b4d143632bb49ad52384bf0c377bfdff3d9dfc733cc7365ed97d33d1bc3a1f6fb7ddef024a1fdc84e7edeb0fd62ae9d98d8932a30e1f6130c"
    "e9a7f76095ffb9f47c20c5cf45b7cf193e777eea7718af8bbcef4df17cb704fdf961babcd0d85b9cbf61a870fa78e364a3343f5dbd9910f303aaef454cfda31e",
    "7b183890d6ac4a28ff5e3ce8ef0283ce0fe9ceeeb1d19d17a21e2f004cfb6871ec3dc6eb6912afac33cd0ba123aaf3c2a079a1d2f2cd835ab3acdda8ceced70e"
    "37a7a74b1367758ef60f7f1f537ed07e9430f58f7aec97c501a551bf5e6d4c5ed827da2fefff90d02e641f98f7ed3929115dde477dfe0798f6d1e2d35b7fcb38",
    "4ff45ffff83e4b3d74f0cefb8583aabe543b9b93f2ba5999af6cdd48abfba71cad0f0598f2517d3ebec3ffae9eee53ef8ae73ce1791fb2e3f637e1753e0518bf"
    "a3fa3ba3d85fc43e68e751eaeab1bab4b77ff5faf1fc6c725fad49c999ab3ce55100a67c58f9495a472eabb24953ef8bf29addef96379906a6ae487404f5727c",
    "78c3bb1eab78b4bc9d5fbfba509c3bc8df382c1fd44e57a5a9d50a47cf990798f297c55356f360c5809209ed7d30cbd6d5dfc8d57525111c67eff9d46b61f5dc"
    "f64be36c37301d71fedbe9dbc571e10fef7a82af74eabf87293f683f06b5cf7c37bceafed3bdd5c3b9fe6f49950a546ed54e0dade7f77ebf79dfa19ee7de37ef",
    "db710f1a5a1d5a7e4736effb32410fd983cdfbbafaba93bb60c5add7fe9cf1f310e756524cf59c83f779402e258fcf366bd9f56bbb37a6ae6f6d9d1ce9a7758e"
    "d67fdfc39417f340fff68e11da8bece479a0ed58a3a2e9f64ac0a8ce032f10f4903dd879a0dbd7ccf3d7ace781d67bb9cf58eaa123aaf3c0a0dfdb4b1307b352",
    "697973716d6d7e617261737369e95a29c1cf3ce09703515af7d54e0a78d77df99d17c43a4077fd51fa3c887580625e18f6fb01efeb00ef61ca8bef07fddb4b7b"
    "1e2853fa1dd67b887960b0b80060da478b5b8f7dccf8fbc14f7ff9304b3d74f03e0fac4de45539bfa46d4ee6af19d542ea744d5f3fe4284fc4ebfa35bfcf8179",
    "82d02e644788ef3e07ece22ce4f24bf7e9d715cfb9d72f64c7ad57bce7533f6af33ec0b4871697b4771973feebefbcc5520f1dbc73bebc763cb9a6e42bbb33c9"
    "aa5e9676f7abb7951a47cf810798f2bcac630c2beffdae777c86e017b207b3ded18eac7b963bc6864bbceb89f53874ea0798f2515fef588366aa29c916191498",
    "4b66d2478ddc5c23119e38f7339ffaaf11f4919d3a77f78f1bd5db5eea3a590d3959d9a968aa69688ad24e288de046213e9ce25d4f70984efd0053feb238dc1a"
    "520fd5ffdb043d64ef4b0067ed79af3f41f3d86fbef915823eb253e4f1c00fa4ec8f617b10e2c229def50487e9d40f30e5798f875ff5e9c738c10f641f76ffa2",
    "0747fcf9f736c13f641f9ecf97b391082e848e0bd778d713dca653bfdff82deaeb05fce697bf49683fb293d78fd8fb208425aef79bdffe1e411fd983fd5cd87d"
    "defe4cb0e2d88bac9f2ff5ceb3ffcb520f1dbccf0b05b85a49e7d462d32c5eddd85fcba4e6cc93498ed68f004cf9cb8ae7fdf2eecb9e73af1eb25b61a1150dce",
    "75b623b8b97bc04d1ee565823eb2079ed776f57e7cb8c4bb9e88c7e9d40f30e5c39a47f90d821eb27baefc6e263b2cfcf51befdef19c7bf5913da83cb6b7f3e3"
    "c225def50477e9d41f977dd5fce63bbe416827b2e3f6554b55abcbf2ae2119a709975f6048bfbcc7a0f39edf78fb25821eb207bdcfda797fb3dd4f93f7fdd6be",
    "7297ed7e6bad4feeb99e13c08afb2ba96ded644a9a5cd06b7a259bafe4afeed67739da4f1960ca8775fdf4fd583ddbd23025c34cd0e3fd65e5b7239bef182ec1"
    "3ce21d1780691f2f7129ef7a22eea6533fc0940f6b9ef961821eb26b3a54ed65076efdb0ec6bcceef73dbfeb426cc0ca5a031acd8bbbc7cb5a4591a16aee8ce9",
    "d090f57d68484ac3de3e7ea4770ce2c225def50477e9d4eff7fa7f0e53ffa8c73ef4f75ff7e56fc75136cd1ab75c31162281eb2d280eebee344e2bfe7ed0737e"
    "de6edb525c9f2b9deab011d4fcf64f43eaa1facb043d64a734ae8d667507755967a078cd4f24feeedbb758eabdf9c2bfb36ddf23573eeed563c5e989ccf4f589",
    "dbf595196d5ed797f7a6b21be56b471cdddf1d574efbcd5b3c496837b2bbf2162ed79cec85e559af5f6048bfbcc7a03c8febf8034cbb68f1ea75d6fcd7df62fa"
    "3c3fd6fbb0b51efac5bbbd7aacf83f35bd7476565e5edcdeae1cdf3ec84abb66c15c10ebf02e1cacf2238f7acebd7ac8de7eb4c45c8704eefbe9c2fabc0fbff9",
    "eabb04bf90fdf2f2d5833d6069c4333071c92bf0ae27f22674ea0798f23cf0d8de824bf0b8d71e0a1ef70c4c5c78c5bb9ee0319dfa01a67c58d74b0f7adf79fb"
    "b24ff5ee86e5d97732ac5cf6fb7ca637087e217b28b8dc6780e2c22fdef5049fe9d40f30e5c3cae7af10f490bd01edabbf20a935983e35a1c70f5ef9fc3704bf",
    "903d703e6306282efce25d4ff0994efd00533eacf98c5f23e821bb01a56a01d6e4467bd3de043d2e5fd67a68bf5cfe21c12f640fcb7a68dcf8004c3b79e116ef"
    "7a82cb74ea0798f261e5f2a0cfdf309c6b3e97cc142c04846e5db4dfbcf28b043f903dc875d1de31681f71e113ef7a82bf74ea0798f261e5ef23043d6437a463",
    "ebb2df3064d3bd8f7958f8eb77bddbf3043f903d50fef68c415cb8c4bb9ee02e9dfa01a67c58b93be8fd80f635df0db7b8e36e14ee07ec1d83b87089773dc15d"
    "3af5034cf9b072f78ae7dcab87ecc7ed28ab27d1c87b1ef875825fc81e963c306e7c00a69dbc708b773dc1653af5034cf9b07219d7be51cfdf9e1c64e78b70d8",
    "e261bf79e0c19f0f178a3c70372114173ef1ae27f84ba77e80291f56fe0e9a076e6f8fd45885c67231edd20febbab5f0f398d2ba35cfc0c48557bceb091ed3a9"
    "1f60ca477d3d71fb7682058b008d8aa6c33efbbc8695cbb1594f8c19a0b8f08b773dc1673af5034cf9b0c6cb83e62b1ab073f577ef26e09dcb3f20f885ec8173",
    "d93b308e5f71e116ef7a82cb74ea0798f261e5f217c8632c883c4638f3180b228fc19d9ee0319dfa01a67cd4f31856385694ac6fc7b02099f086a6ddf6f8c12b"
    "975f21f885ecacd75dd87fd35203dab1b27770e2c22ddef50497e9d40f30e5c31a270fbafed8baf46fce152ee62d79e5f1e5df87472f7fd13b3071e115ef7a82",
    "c774eaf7bb2feb89e73ce1791fb2d3dba773600cd4a5da781db5f3559fed1c27b413d9c9eb912db75468dacf340b6e5e649727097219746f5fdb1bf5b2e255b5"
    "f5b311967a89daf32f32d5730edef95f970bcbebe96b4b1395ed5a69726a6b7deb20bf9816fc8f1bff9f22b413d9c9fcb75f2cab727bbbd2c8f2fffb043d640f",
    "96ffe77ded6cd3ce8c578f7dcc96ffad9ffef261967ae8e09dff6b137955ce2f699b93f96b46b5903a5dd3d70f39daffe47d4cf941fb51c2d43feab15f16071c"
    "d8a3cbbbdbae96cf76fd2ea15dc88ecdbb58b34faf3f7ef350573ce75e7f90dd9e6a9cbb6f12f19be701a61db438a4bdcb38ae9f7d5263aae71cbc735d99d294",
    "d2617d663b7bbc621e56f21bc5894292a3bc0ec094bfac787258bd11cffb2eead916d9132fb77cead1e6bbdfe77b3c43f007d983f97dd38ea047d02024e2c323"
    "def5441e9d4efd00533eacbf6b3eea39f7ea213b6e1fc1964ffdb0f1f78ee7dceb0fb207f17b6627c0f56c1c18173ef1ae27f84ba77e8029cf037ffbed1bd8f2",
    "a92ff83b0c7fcf47222e7ce25d4ff0974efd00533eacebad7f93a087ecedab7ea56715015a4816560e7fead39fe0d7590fc2e18b2312175ef1ae27784ca77e80"
    "291fd67878d0fd4d1ab07df19fdf8dcc2b875f20f883ecc171d83b12f611174ef1ae27384ca77e80291f560e7f81fbc257fadd17def2a91fb6bcc45f13fc41f6",
    "e038ec1e89b8f089773dc15f3af5034cf9b0e62568dd07def2e947d8e2e1e0f312e2feef38eb091ed3a91f60ca87351ef67bff77cba77ed8e2e1e708fe207ba0"
    "79899e91880b9f78d713fca5533fc094e73d1e063efd7800eb876d711824e25dc72f80691f2f3ce25d4ff0964efd00539ed77817f8d4ffa29c8dfebab3cfe5ac",
    "669572c5b471e10fef7a82af74ea0798f251bdcfcd7076b08e2a4f9fc5eab9edc1f2d4b8b04f38c0b48b17def0ae27784aa77e80291fd678f521821eb25b11d4"
    "91de67df27e0539f355f9fc7eab9ed81c7ab4e6fc7853fbceb09bed2a91f60ca479daf06b4ae79c15766f12beaedb8f087773dc1573af5034cf9a8ffbe558366",
    "fea87ef3c8d48fcc460838fba94fbdbb583db73d58ce5eecf5b8f088773dc15b3af5034cf9b0e65f1ff49c9febd9968609f5417e67096bfcfa3dac9edb1e707e"
    "c0e9e5f8f086773dc1533af5034cf9a8e70770f9d7b0ee8fc437879de739f60c4a5c38c5bb9ee0309dfa01a67cd4398ccbd30a0e07c8e19e41890ba778d7131c",
    "a6533fc094e73d9f1b561efbcdfb86fbbe0687c77d06272edce25d4f70994efd00539e97bc6f58f91bfde7d90c9297e8a68763c325def50477e9d40f30e5c31a"
    "0fff3a410fd9db2bf2eda76bb75173f1beb2b0f2f8339f7ebd46f00bd983789ec279939d5b26fa0c505cf8c5bb9ee0339dfa01a67c58f93ce873d0db977faacd",
    "036848a6aca9dee7abf0cae7d7097e217b28f8dc6780e2c22fdef5049fe9d40f30e5c3fabb1eae7da39ebfedcbbf006bb2f5d5b913a0859dcb7ef3c82f13fc42"
    "f6807fd7730f4a223ebce25d4ff0984efd00533eea3cae686a131a6627228b028ffdc6c93f20f885ec81c7c9de8171fc8a0bb778d7135ca6533fc0940f6b1ee3",
    "b7087ac8def99aacd68e14c958878a5691cd53d77d5dbcf2f94d825fc81e389f310314177ef1ae27f84ca77e80291fd6b879d0fddd9cf06cc18241273a0b3b97"
    "fde6315e25f885ec8173d9333071e115ef7a82c774ea0798f261e5b1dffbf65a3ef5c3b6af4524ee1311f7eb71a727f84ba77e80291f75fee2eed76bf9d417fc",
    "15f7e9093dc1dfa8f2372cf7e9b57cfa21f6d914f7e7093dc1e3a8f378583d54ff17bd3fafe5532f6cf1afb82fcffbfe70f088773dc15b3af5034cf9b0c6bfb4"
    "eecb6bf9f4236cf1ef4b047f903db8fd8dfb8f485c78c5bb9ee0319dfa01a67c5879fc55821eb2b7affe15a9a64253ae6465a8543df158cba71f61e371f0cf89",
    "1f84c71747242ebce25d4ff0984efd00533eacbfcfe1da37eaf98bbbefaee5533f6c1c7e99e00fb28bfbed84de65e8090ed3a91f60ca8795c38f7acebd7ac8ee"
    "2c4fb50231fbc20f2b87a3bf5f3d291e768f445cf8c4bb9ee02f9dfa01a67cd4f95b8366e6c830a06a96e43aa4a77fc573eed547f68a5685c6b8c5ac2359d57a",
    "fc7a70c49ffedb047d64671df78e751a8cac3b76423863bd56836ad99415d99461c35e2ad1775c00a6ddbcf08a773dc1633af5034cf9b0e68907fdddce7ddd5f"
    "fcdd2e682e47709f66fbed3a7abbd32659432c765eb8005d910fe6484f70974efd00533eac71f023043d6437a002a506bcf07cdc964f7d9187182a1fdc1d8cb8",
    "f089773dc15f3af5034cf9a8f3b7a268d6059fccccc1a64bdfeff323be49d047f65e2a8d2318db41b06d1aafc2a65c81c8afe83f3f7ea0fc84dde8767618333e"
    "00d34e5eb8c5bb9ee0329dfa01a63c2f5c4e1f355cfa7e3f378f13f491ddf9725e35e4a685e2947d66f9139d7c842919356822fcca56771acd73c222feca5a45",
    "91a16aee8ce9d090f57d68484a63c76ae8b83ed23b0671e112ef7a82bb74ea0798f251e72e2e1f11d6e7a989bc84db5f5e38c5bb9ee0309dfa01a63caf1c063e"
    "f51fc0eadb1607b7d4d601bf80d573db83e16b5ab34ab901ebf813170ef1ae2738ebaffeff07302f6861",
    "" };

  c2_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c2_data[0], 152520U, &c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static c2_lsm9ds1 *c2_lsm9ds1_lsm9ds1(SFc2_arduino_imu_pitch_rollInstanceStruct *
  chartInstance, const emlrtStack *c2_sp, c2_lsm9ds1 *c2_obj, c2_arduino
  *c2_varargin_1)
{
  c2_arduino *c2_b_parent;
  c2_arduino *c2_b_varargin_1;
  c2_arduino *c2_c_parent;
  c2_arduino *c2_c_varargin_1;
  c2_arduino *c2_d_varargin_1;
  c2_arduino *c2_e_varargin_1;
  c2_arduino *c2_h_varargin_1;
  c2_arduino *c2_i_varargin_1;
  c2_arduino *c2_parent;
  c2_arduinodriver_ArduinoI2C *c2_fb_obj;
  c2_arduinodriver_ArduinoI2C *c2_gb_obj;
  c2_arduinodriver_ArduinoI2C *c2_hb_obj;
  c2_arduinodriver_ArduinoI2C *c2_jb_obj;
  c2_arduinodriver_ArduinoI2C *c2_kb_obj;
  c2_arduinodriver_ArduinoI2C *c2_lc_obj;
  c2_arduinodriver_ArduinoI2C *c2_mc_obj;
  c2_arduinodriver_ArduinoI2C *c2_ob_obj;
  c2_arduinodriver_ArduinoI2C *c2_oc_obj;
  c2_arduinodriver_ArduinoI2C *c2_pb_obj;
  c2_arduinodriver_ArduinoI2C *c2_pc_obj;
  c2_arduinodriver_ArduinoI2C *c2_qb_obj;
  c2_arduinodriver_ArduinoI2C *c2_sb_obj;
  c2_arduinodriver_ArduinoI2C *c2_sc_obj;
  c2_arduinodriver_ArduinoI2C *c2_tb_obj;
  c2_arduinodriver_ArduinoI2C *c2_tc_obj;
  c2_arduinodriver_ArduinoI2C *c2_uc_obj;
  c2_arduinodriver_ArduinoI2C *c2_wc_obj;
  c2_arduinodriver_ArduinoI2C *c2_xc_obj;
  c2_cell_1 c2_r;
  c2_lsm9ds1 *c2_ad_obj;
  c2_lsm9ds1 *c2_b_obj;
  c2_lsm9ds1 *c2_b_this;
  c2_lsm9ds1 *c2_bd_obj;
  c2_lsm9ds1 *c2_c_obj;
  c2_lsm9ds1 *c2_cd_obj;
  c2_lsm9ds1 *c2_d_obj;
  c2_lsm9ds1 *c2_dd_obj;
  c2_lsm9ds1 *c2_e_obj;
  c2_lsm9ds1 *c2_ed_obj;
  c2_lsm9ds1 *c2_f_obj;
  c2_lsm9ds1 *c2_fd_obj;
  c2_lsm9ds1 *c2_g_obj;
  c2_lsm9ds1 *c2_gd_obj;
  c2_lsm9ds1 *c2_h_obj;
  c2_lsm9ds1 *c2_hd_obj;
  c2_lsm9ds1 *c2_i_obj;
  c2_lsm9ds1 *c2_j_obj;
  c2_lsm9ds1 *c2_k_obj;
  c2_lsm9ds1 *c2_l_obj;
  c2_lsm9ds1 *c2_this;
  c2_matlabshared_sensors_coder_matlab_device *c2_c_iobj_0;
  c2_matlabshared_sensors_coder_matlab_device *c2_d_iobj_0;
  c2_matlabshared_sensors_coder_matlab_device *c2_eb_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_ib_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_kc_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_nb_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_nc_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_rb_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_rc_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_vc_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_ab_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_accelGyro;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_b_iobj_1;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_bb_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_c_this;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_cb_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_d_this;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_db_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_e_this;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_f_this;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_iobj_1;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_lb_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_m_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_mb_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_n_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_o_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_p_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_q_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_r_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_s_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_t_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_u_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_v_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_w_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_x_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_y_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_ac_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_b_iobj_0;
  c2_sensors_internal_lsm9ds1_mag *c2_bc_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_cc_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_dc_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_ec_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_fc_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_g_this;
  c2_sensors_internal_lsm9ds1_mag *c2_gc_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_h_this;
  c2_sensors_internal_lsm9ds1_mag *c2_hc_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_i_this;
  c2_sensors_internal_lsm9ds1_mag *c2_ic_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_iobj_0;
  c2_sensors_internal_lsm9ds1_mag *c2_jc_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_magneto;
  c2_sensors_internal_lsm9ds1_mag *c2_qc_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_ub_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_vb_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_wb_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_xb_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_yb_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_yc_obj;
  emlrtStack c2_b_st;
  emlrtStack c2_c_st;
  emlrtStack c2_d_st;
  emlrtStack c2_e_st;
  emlrtStack c2_st;
  int32_T c2_i;
  int32_T c2_i1;
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  uint8_T c2_d_data[2];
  uint8_T c2_a;
  uint8_T c2_b_a;
  uint8_T c2_b_c;
  uint8_T c2_b_data;
  uint8_T c2_b_output;
  uint8_T c2_b_slaveAddress;
  uint8_T c2_b_status;
  uint8_T c2_b_val;
  uint8_T c2_c;
  uint8_T c2_c_a;
  uint8_T c2_c_c;
  uint8_T c2_c_data;
  uint8_T c2_c_output;
  uint8_T c2_c_slaveAddress;
  uint8_T c2_c_status;
  uint8_T c2_c_val;
  uint8_T c2_d_a;
  uint8_T c2_d_slaveAddress;
  uint8_T c2_data;
  uint8_T c2_e_a;
  uint8_T c2_e_data;
  uint8_T c2_e_slaveAddress;
  uint8_T c2_f_a;
  uint8_T c2_f_data;
  uint8_T c2_f_slaveAddress;
  uint8_T c2_f_varargin_1;
  uint8_T c2_g_data;
  uint8_T c2_g_slaveAddress;
  uint8_T c2_g_varargin_1;
  uint8_T c2_h_data;
  uint8_T c2_h_slaveAddress;
  uint8_T c2_i_data;
  uint8_T c2_i_slaveAddress;
  uint8_T c2_j_data;
  uint8_T c2_j_slaveAddress;
  uint8_T c2_j_varargin_1;
  uint8_T c2_k_slaveAddress;
  uint8_T c2_l_slaveAddress;
  uint8_T c2_m_slaveAddress;
  uint8_T c2_n_slaveAddress;
  uint8_T c2_o_slaveAddress;
  uint8_T c2_output;
  uint8_T c2_p_slaveAddress;
  uint8_T c2_q_slaveAddress;
  uint8_T c2_r_slaveAddress;
  uint8_T c2_s_slaveAddress;
  uint8_T c2_slaveAddress;
  uint8_T c2_status;
  uint8_T c2_val;
  boolean_T c2_b_flag;
  boolean_T c2_c_flag;
  boolean_T c2_d_flag;
  boolean_T c2_e_flag;
  boolean_T c2_flag;
  c2_st.prev = c2_sp;
  c2_st.tls = c2_sp->tls;
  c2_b_st.prev = &c2_st;
  c2_b_st.tls = c2_st.tls;
  c2_c_st.prev = &c2_b_st;
  c2_c_st.tls = c2_b_st.tls;
  c2_d_st.prev = &c2_c_st;
  c2_d_st.tls = c2_c_st.tls;
  c2_e_st.prev = &c2_d_st;
  c2_e_st.tls = c2_d_st.tls;
  c2_b_obj = c2_obj;
  c2_st.site = &c2_qb_emlrtRSI;
  c2_c_obj = c2_b_obj;
  c2_b_obj = c2_c_obj;
  c2_b_st.site = &c2_sb_emlrtRSI;
  c2_d_obj = c2_b_obj;
  c2_b_obj = c2_d_obj;
  c2_c_st.site = &c2_tb_emlrtRSI;
  c2_e_obj = c2_b_obj;
  c2_b_obj = c2_e_obj;
  c2_d_st.site = &c2_ub_emlrtRSI;
  c2_f_obj = c2_b_obj;
  c2_b_obj = c2_f_obj;
  c2_e_st.site = &c2_vb_emlrtRSI;
  c2_g_obj = c2_b_obj;
  c2_b_obj = c2_g_obj;
  c2_this = c2_b_obj;
  c2_b_obj = c2_this;
  c2_e_st.site = &c2_vb_emlrtRSI;
  c2_h_obj = c2_b_obj;
  c2_b_obj = c2_h_obj;
  c2_b_this = c2_b_obj;
  c2_b_obj = c2_b_this;
  c2_b_obj->isInitialized = 0;
  c2_i_obj = c2_b_obj;
  for (c2_i = 0; c2_i < 2; c2_i++) {
    c2_i_obj->tunablePropertyChanged[c2_i] = false;
  }

  c2_st.site = &c2_rb_emlrtRSI;
  c2_j_obj = c2_b_obj;
  c2_b_varargin_1 = c2_varargin_1;
  c2_iobj_0 = &c2_b_obj->_pobj0;
  c2_iobj_1 = &c2_b_obj->_pobj1;
  c2_b_st.site = &c2_xb_emlrtRSI;
  c2_k_obj = c2_j_obj;
  c2_parent = c2_b_varargin_1;
  c2_k_obj->Parent = c2_parent;
  c2_b_st.site = &c2_ac_emlrtRSI;
  c2_l_obj = c2_j_obj;
  c2_c_varargin_1 = c2_b_varargin_1;
  c2_b_iobj_0 = c2_iobj_0;
  c2_b_iobj_1 = c2_iobj_1;
  c2_c_st.site = &c2_bc_emlrtRSI;
  c2_m_obj = c2_b_iobj_1;
  c2_d_varargin_1 = c2_c_varargin_1;
  c2_accelGyro = c2_m_obj;
  c2_d_st.site = &c2_ec_emlrtRSI;
  c2_n_obj = c2_accelGyro;
  c2_accelGyro = c2_n_obj;
  c2_e_st.site = &c2_gc_emlrtRSI;
  c2_c_this = c2_accelGyro;
  c2_accelGyro = c2_c_this;
  c2_d_st.site = &c2_ec_emlrtRSI;
  c2_o_obj = c2_accelGyro;
  c2_accelGyro = c2_o_obj;
  c2_e_st.site = &c2_hc_emlrtRSI;
  c2_d_this = c2_accelGyro;
  c2_accelGyro = c2_d_this;
  c2_d_st.site = &c2_ec_emlrtRSI;
  c2_p_obj = c2_accelGyro;
  c2_accelGyro = c2_p_obj;
  c2_e_st.site = &c2_ic_emlrtRSI;
  c2_q_obj = c2_accelGyro;
  c2_accelGyro = c2_q_obj;
  c2_r_obj = c2_accelGyro;
  c2_accelGyro = c2_r_obj;
  c2_s_obj = c2_accelGyro;
  c2_accelGyro = c2_s_obj;
  c2_t_obj = c2_accelGyro;
  c2_accelGyro = c2_t_obj;
  c2_e_this = c2_accelGyro;
  c2_accelGyro = c2_e_this;
  c2_u_obj = c2_accelGyro;
  c2_accelGyro = c2_u_obj;
  c2_accelGyro->isInitialized = 0;
  c2_f_this = c2_accelGyro;
  c2_accelGyro = c2_f_this;
  c2_accelGyro->isInitialized = 0;
  c2_v_obj = c2_accelGyro;
  for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
    c2_v_obj->tunablePropertyChanged[c2_i1] = false;
  }

  c2_d_st.site = &c2_fc_emlrtRSI;
  c2_w_obj = c2_accelGyro;
  c2_e_varargin_1 = c2_d_varargin_1;
  c2_c_iobj_0 = &c2_accelGyro->_pobj0;
  c2_e_st.site = &c2_jc_emlrtRSI;
  c2_x_obj = c2_w_obj;
  c2_b_parent = c2_e_varargin_1;
  c2_x_obj->Parent = c2_b_parent;
  c2_e_st.site = &c2_kc_emlrtRSI;
  c2_y_obj = c2_w_obj;
  c2_ab_obj = c2_y_obj;
  c2_flag = (c2_ab_obj->isInitialized == 1);
  if (c2_flag) {
    c2_y_obj->TunablePropsChanged = true;
    c2_y_obj->tunablePropertyChanged[0] = true;
  }

  c2_e_st.site = &c2_kc_emlrtRSI;
  c2_w_obj->Device = c2_device_device(chartInstance, &c2_e_st, c2_c_iobj_0,
    c2_w_obj->Parent, 107U);
  c2_e_st.site = &c2_lc_emlrtRSI;
  c2_bb_obj = c2_w_obj;
  c2_cb_obj = c2_bb_obj;
  c2_db_obj = c2_cb_obj;
  c2_eb_obj = c2_db_obj->Device;
  c2_fb_obj = c2_eb_obj->InterfaceObj;
  c2_slaveAddress = c2_eb_obj->DeviceAddress;
  c2_val = 0U;
  c2_gb_obj = c2_fb_obj;
  c2_b_slaveAddress = c2_slaveAddress;
  c2_data = 32U;
  c2_status = MW_I2C_MasterWrite(c2_gb_obj->MW_I2C_HANDLE, c2_b_slaveAddress,
    &c2_data, 1U, true, false);
  if (c2_status == 0) {
    c2_hb_obj = c2_fb_obj;
    c2_c_slaveAddress = c2_slaveAddress;
    MW_I2C_MasterRead(c2_hb_obj->MW_I2C_HANDLE, c2_c_slaveAddress, &c2_output,
                      1U, false, true);
    c2_val = c2_output;
  }

  c2_f_varargin_1 = c2_val;
  c2_a = c2_f_varargin_1;
  c2_b_a = c2_a;
  c2_c = c2_b_a;
  c2_ib_obj = c2_db_obj->Device;
  c2_b_data = c2_c;
  c2_jb_obj = c2_ib_obj->InterfaceObj;
  c2_d_slaveAddress = c2_ib_obj->DeviceAddress;
  c2_c_data = c2_b_data;
  c2_d_data[0] = 32U;
  c2_d_data[1] = c2_c_data;
  c2_kb_obj = c2_jb_obj;
  c2_e_slaveAddress = c2_d_slaveAddress;
  MW_I2C_MasterWrite(c2_kb_obj->MW_I2C_HANDLE, c2_e_slaveAddress, &c2_d_data[0],
                     2U, false, false);
  c2_cb_obj->AccelerometerResolution = 6.103515625E-5;
  c2_lb_obj = c2_bb_obj;
  c2_mb_obj = c2_lb_obj;
  c2_nb_obj = c2_mb_obj->Device;
  c2_ob_obj = c2_nb_obj->InterfaceObj;
  c2_f_slaveAddress = c2_nb_obj->DeviceAddress;
  c2_b_val = 0U;
  c2_pb_obj = c2_ob_obj;
  c2_g_slaveAddress = c2_f_slaveAddress;
  c2_e_data = 16U;
  c2_b_status = MW_I2C_MasterWrite(c2_pb_obj->MW_I2C_HANDLE, c2_g_slaveAddress,
    &c2_e_data, 1U, true, false);
  if (c2_b_status == 0) {
    c2_qb_obj = c2_ob_obj;
    c2_h_slaveAddress = c2_f_slaveAddress;
    MW_I2C_MasterRead(c2_qb_obj->MW_I2C_HANDLE, c2_h_slaveAddress, &c2_b_output,
                      1U, false, true);
    c2_b_val = c2_b_output;
  }

  c2_g_varargin_1 = c2_b_val;
  c2_c_a = c2_g_varargin_1;
  c2_d_a = c2_c_a;
  c2_b_c = c2_d_a;
  c2_rb_obj = c2_mb_obj->Device;
  c2_f_data = c2_b_c;
  c2_sb_obj = c2_rb_obj->InterfaceObj;
  c2_i_slaveAddress = c2_rb_obj->DeviceAddress;
  c2_g_data = c2_f_data;
  c2_d_data[0] = 16U;
  c2_d_data[1] = c2_g_data;
  c2_tb_obj = c2_sb_obj;
  c2_j_slaveAddress = c2_i_slaveAddress;
  MW_I2C_MasterWrite(c2_tb_obj->MW_I2C_HANDLE, c2_j_slaveAddress, &c2_d_data[0],
                     2U, false, false);
  c2_lb_obj->GyroscopeResolution = 0.00875;
  c2_e_st.site = &c2_mc_emlrtRSI;
  c2_sensorBase_set_SampleRate(chartInstance, c2_w_obj);
  c2_accelGyro->matlabCodegenIsDeleted = false;
  c2_c_st.site = &c2_cc_emlrtRSI;
  c2_ub_obj = c2_b_iobj_0;
  c2_h_varargin_1 = c2_c_varargin_1;
  c2_magneto = c2_ub_obj;
  c2_d_st.site = &c2_id_emlrtRSI;
  c2_vb_obj = c2_magneto;
  c2_magneto = c2_vb_obj;
  c2_e_st.site = &c2_kd_emlrtRSI;
  c2_g_this = c2_magneto;
  c2_magneto = c2_g_this;
  c2_d_st.site = &c2_id_emlrtRSI;
  c2_wb_obj = c2_magneto;
  c2_magneto = c2_wb_obj;
  c2_e_st.site = &c2_ic_emlrtRSI;
  c2_xb_obj = c2_magneto;
  c2_magneto = c2_xb_obj;
  c2_yb_obj = c2_magneto;
  c2_magneto = c2_yb_obj;
  c2_ac_obj = c2_magneto;
  c2_magneto = c2_ac_obj;
  c2_bc_obj = c2_magneto;
  c2_magneto = c2_bc_obj;
  c2_h_this = c2_magneto;
  c2_magneto = c2_h_this;
  c2_cc_obj = c2_magneto;
  c2_magneto = c2_cc_obj;
  c2_magneto->isInitialized = 0;
  c2_i_this = c2_magneto;
  c2_magneto = c2_i_this;
  c2_magneto->isInitialized = 0;
  c2_dc_obj = c2_magneto;
  for (c2_i2 = 0; c2_i2 < 3; c2_i2++) {
    c2_dc_obj->tunablePropertyChanged[c2_i2] = false;
  }

  c2_d_st.site = &c2_jd_emlrtRSI;
  c2_ec_obj = c2_magneto;
  c2_i_varargin_1 = c2_h_varargin_1;
  c2_d_iobj_0 = &c2_magneto->_pobj0;
  c2_e_st.site = &c2_jc_emlrtRSI;
  c2_fc_obj = c2_ec_obj;
  c2_c_parent = c2_i_varargin_1;
  c2_fc_obj->Parent = c2_c_parent;
  c2_e_st.site = &c2_kc_emlrtRSI;
  c2_gc_obj = c2_ec_obj;
  c2_hc_obj = c2_gc_obj;
  c2_b_flag = (c2_hc_obj->isInitialized == 1);
  if (c2_b_flag) {
    c2_gc_obj->TunablePropsChanged = true;
    c2_gc_obj->tunablePropertyChanged[0] = true;
  }

  c2_e_st.site = &c2_kc_emlrtRSI;
  c2_ec_obj->Device = c2_device_device(chartInstance, &c2_e_st, c2_d_iobj_0,
    c2_ec_obj->Parent, 30U);
  c2_e_st.site = &c2_lc_emlrtRSI;
  c2_ic_obj = c2_ec_obj;
  c2_jc_obj = c2_ic_obj;
  c2_kc_obj = c2_jc_obj->Device;
  c2_lc_obj = c2_kc_obj->InterfaceObj;
  c2_k_slaveAddress = c2_kc_obj->DeviceAddress;
  c2_mc_obj = c2_lc_obj;
  c2_l_slaveAddress = c2_k_slaveAddress;
  for (c2_i3 = 0; c2_i3 < 2; c2_i3++) {
    c2_d_data[c2_i3] = (uint8_T)(34U + (uint32_T)(uint8_T)(-34 * (uint8_T)c2_i3));
  }

  MW_I2C_MasterWrite(c2_mc_obj->MW_I2C_HANDLE, c2_l_slaveAddress, &c2_d_data[0],
                     2U, false, false);
  c2_nc_obj = c2_jc_obj->Device;
  c2_oc_obj = c2_nc_obj->InterfaceObj;
  c2_m_slaveAddress = c2_nc_obj->DeviceAddress;
  c2_pc_obj = c2_oc_obj;
  c2_n_slaveAddress = c2_m_slaveAddress;
  for (c2_i4 = 0; c2_i4 < 2; c2_i4++) {
    c2_d_data[c2_i4] = (uint8_T)(32U + (uint32_T)(uint8_T)(-32 * (uint8_T)c2_i4));
  }

  MW_I2C_MasterWrite(c2_pc_obj->MW_I2C_HANDLE, c2_n_slaveAddress, &c2_d_data[0],
                     2U, false, false);
  c2_qc_obj = c2_jc_obj;
  c2_rc_obj = c2_qc_obj->Device;
  c2_sc_obj = c2_rc_obj->InterfaceObj;
  c2_o_slaveAddress = c2_rc_obj->DeviceAddress;
  c2_c_val = 0U;
  c2_tc_obj = c2_sc_obj;
  c2_p_slaveAddress = c2_o_slaveAddress;
  c2_h_data = 33U;
  c2_c_status = MW_I2C_MasterWrite(c2_tc_obj->MW_I2C_HANDLE, c2_p_slaveAddress,
    &c2_h_data, 1U, true, false);
  if (c2_c_status == 0) {
    c2_uc_obj = c2_sc_obj;
    c2_q_slaveAddress = c2_o_slaveAddress;
    MW_I2C_MasterRead(c2_uc_obj->MW_I2C_HANDLE, c2_q_slaveAddress, &c2_c_output,
                      1U, false, true);
    c2_c_val = c2_c_output;
  }

  c2_j_varargin_1 = c2_c_val;
  c2_e_a = c2_j_varargin_1;
  c2_f_a = c2_e_a;
  c2_c_c = c2_f_a;
  c2_vc_obj = c2_qc_obj->Device;
  c2_i_data = c2_c_c;
  c2_wc_obj = c2_vc_obj->InterfaceObj;
  c2_r_slaveAddress = c2_vc_obj->DeviceAddress;
  c2_j_data = c2_i_data;
  c2_d_data[0] = 33U;
  c2_d_data[1] = c2_j_data;
  c2_xc_obj = c2_wc_obj;
  c2_s_slaveAddress = c2_r_slaveAddress;
  MW_I2C_MasterWrite(c2_xc_obj->MW_I2C_HANDLE, c2_s_slaveAddress, &c2_d_data[0],
                     2U, false, false);
  c2_yc_obj = c2_jc_obj;
  c2_yc_obj->MagnetometerResolution = 0.00014000000000000001;
  c2_e_st.site = &c2_mc_emlrtRSI;
  c2_b_sensorBase_set_SampleRate(chartInstance, c2_ec_obj);
  c2_magneto->matlabCodegenIsDeleted = false;
  c2_c_st.site = &c2_dc_emlrtRSI;
  c2_ad_obj = c2_l_obj;
  c2_d_st.site = &c2_wb_emlrtRSI;
  c2_bd_obj = c2_ad_obj;
  c2_c_flag = (c2_bd_obj->isInitialized == 1);
  if (c2_c_flag) {
    c2_ad_obj->TunablePropsChanged = true;
    c2_ad_obj->tunablePropertyChanged[0] = true;
  }

  c2_r.f1 = c2_accelGyro;
  c2_r.f2 = c2_magneto;
  c2_l_obj->SensorObjects = c2_r;
  c2_b_st.site = &c2_yb_emlrtRSI;
  c2_cd_obj = c2_j_obj;
  c2_c_st.site = &c2_fd_emlrtRSI;
  c2_dd_obj = c2_cd_obj;
  c2_d_st.site = &c2_vd_emlrtRSI;
  c2_sensorBase_set_SampleRate(chartInstance, c2_dd_obj->SensorObjects.f1);
  c2_d_st.site = &c2_vd_emlrtRSI;
  c2_ed_obj = c2_dd_obj;
  c2_e_st.site = &c2_wb_emlrtRSI;
  c2_fd_obj = c2_ed_obj;
  c2_d_flag = (c2_fd_obj->isInitialized == 1);
  if (c2_d_flag) {
    c2_ed_obj->TunablePropsChanged = true;
    c2_ed_obj->tunablePropertyChanged[0] = true;
  }

  c2_d_st.site = &c2_vd_emlrtRSI;
  c2_b_sensorBase_set_SampleRate(chartInstance, c2_dd_obj->SensorObjects.f2);
  c2_d_st.site = &c2_vd_emlrtRSI;
  c2_gd_obj = c2_dd_obj;
  c2_e_st.site = &c2_wb_emlrtRSI;
  c2_hd_obj = c2_gd_obj;
  c2_e_flag = (c2_hd_obj->isInitialized == 1);
  if (c2_e_flag) {
    c2_gd_obj->TunablePropsChanged = true;
    c2_gd_obj->tunablePropertyChanged[0] = true;
  }

  c2_b_obj->matlabCodegenIsDeleted = false;
  return c2_b_obj;
}

static c2_matlabshared_sensors_coder_matlab_device *c2_device_device
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance, const emlrtStack
   *c2_sp, c2_matlabshared_sensors_coder_matlab_device *c2_obj, c2_arduino
   *c2_parent, uint8_T c2_deviceAddress)
{
  MW_I2C_Mode_Type c2_modename;
  c2_arduino *c2_d_obj;
  c2_arduino *c2_e_obj;
  c2_arduinodriver_ArduinoI2C *c2_g_obj;
  c2_arduinodriver_ArduinoI2C *c2_i2cDriverObj;
  c2_matlabshared_sensors_coder_matlab_device *c2_b_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_c_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_f_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_this;
  emlrtStack c2_b_st;
  emlrtStack c2_st;
  int32_T c2_i;
  uint32_T c2_u;
  uint8_T c2_b_busNum;
  uint8_T c2_busNum;
  uint8_T c2_i2cModule;
  c2_st.prev = c2_sp;
  c2_st.tls = c2_sp->tls;
  c2_b_st.prev = &c2_st;
  c2_b_st.tls = c2_st.tls;
  c2_b_obj = c2_obj;
  c2_st.site = &c2_nc_emlrtRSI;
  c2_this = c2_b_obj;
  c2_b_obj = c2_this;
  c2_c_obj = c2_b_obj;
  c2_c_obj->Bus = 0U;
  c2_b_obj->DeviceAddress = c2_deviceAddress;
  c2_st.site = &c2_oc_emlrtRSI;
  c2_d_obj = c2_parent;
  c2_busNum = c2_b_obj->Bus;
  c2_b_st.site = &c2_qc_emlrtRSI;
  c2_e_obj = c2_d_obj;
  c2_b_busNum = c2_busNum;
  c2_u = (uint32_T)c2_b_busNum + 1U;
  if (c2_u > 255U) {
    c2_u = 255U;
    sf_data_saturate_error(chartInstance->S, 1U, 0, 0);
  }

  c2_i = (uint8_T)c2_u - 1;
  if ((c2_i < 0) || (c2_i > 0)) {
    emlrtDynamicBoundsCheckR2012b(c2_i, 0, 0, &c2_emlrtBCI, &c2_b_st);
  }

  c2_i2cDriverObj = c2_e_obj->I2CDriverObj[c2_i];
  c2_b_obj->InterfaceObj = c2_i2cDriverObj;
  c2_st.site = &c2_pc_emlrtRSI;
  c2_f_obj = c2_b_obj;
  c2_g_obj = c2_f_obj->InterfaceObj;
  c2_i2cModule = c2_f_obj->Bus;
  c2_modename = MW_I2C_MASTER;
  c2_g_obj->MW_I2C_HANDLE = MW_I2C_Open(c2_i2cModule, c2_modename);
  return c2_b_obj;
}

static void c2_sensorBase_set_SampleRate
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance,
   c2_sensors_internal_lsm9ds1_accel_gyro *c2_obj)
{
  c2_arduinodriver_ArduinoI2C *c2_d_obj;
  c2_arduinodriver_ArduinoI2C *c2_e_obj;
  c2_arduinodriver_ArduinoI2C *c2_f_obj;
  c2_arduinodriver_ArduinoI2C *c2_h_obj;
  c2_arduinodriver_ArduinoI2C *c2_i_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_c_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_g_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_b_obj;
  uint8_T c2_d_data[2];
  uint8_T c2_a;
  uint8_T c2_b_a;
  uint8_T c2_b_data;
  uint8_T c2_b_slaveAddress;
  uint8_T c2_c;
  uint8_T c2_c_data;
  uint8_T c2_c_slaveAddress;
  uint8_T c2_d_slaveAddress;
  uint8_T c2_data;
  uint8_T c2_e_slaveAddress;
  uint8_T c2_output;
  uint8_T c2_slaveAddress;
  uint8_T c2_status;
  uint8_T c2_val;
  uint8_T c2_varargin_1;
  (void)chartInstance;
  c2_b_obj = c2_obj;
  c2_c_obj = c2_b_obj->Device;
  c2_d_obj = c2_c_obj->InterfaceObj;
  c2_slaveAddress = c2_c_obj->DeviceAddress;
  c2_val = 0U;
  c2_e_obj = c2_d_obj;
  c2_b_slaveAddress = c2_slaveAddress;
  c2_data = 16U;
  c2_status = MW_I2C_MasterWrite(c2_e_obj->MW_I2C_HANDLE, c2_b_slaveAddress,
    &c2_data, 1U, true, false);
  if (c2_status == 0) {
    c2_f_obj = c2_d_obj;
    c2_c_slaveAddress = c2_slaveAddress;
    MW_I2C_MasterRead(c2_f_obj->MW_I2C_HANDLE, c2_c_slaveAddress, &c2_output, 1U,
                      false, true);
    c2_val = c2_output;
  }

  c2_varargin_1 = c2_val;
  c2_a = c2_varargin_1;
  c2_b_a = c2_a;
  c2_c = (uint8_T)(c2_b_a | 193);
  c2_g_obj = c2_b_obj->Device;
  c2_b_data = c2_c;
  c2_h_obj = c2_g_obj->InterfaceObj;
  c2_d_slaveAddress = c2_g_obj->DeviceAddress;
  c2_c_data = c2_b_data;
  c2_d_data[0] = 16U;
  c2_d_data[1] = c2_c_data;
  c2_i_obj = c2_h_obj;
  c2_e_slaveAddress = c2_d_slaveAddress;
  MW_I2C_MasterWrite(c2_i_obj->MW_I2C_HANDLE, c2_e_slaveAddress, &c2_d_data[0],
                     2U, false, false);
  c2_b_obj->GyroscopeBandwidth = 40.0;
  c2_b_obj->AccelerometerODR = 952.0;
  c2_b_obj->GyroscopeODR = 952.0;
}

static void c2_b_sensorBase_set_SampleRate
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance,
   c2_sensors_internal_lsm9ds1_mag *c2_obj)
{
  c2_arduinodriver_ArduinoI2C *c2_d_obj;
  c2_arduinodriver_ArduinoI2C *c2_e_obj;
  c2_arduinodriver_ArduinoI2C *c2_f_obj;
  c2_arduinodriver_ArduinoI2C *c2_h_obj;
  c2_arduinodriver_ArduinoI2C *c2_i_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_c_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_g_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_b_obj;
  uint8_T c2_d_data[2];
  uint8_T c2_a;
  uint8_T c2_b_a;
  uint8_T c2_b_data;
  uint8_T c2_b_slaveAddress;
  uint8_T c2_c;
  uint8_T c2_c_data;
  uint8_T c2_c_slaveAddress;
  uint8_T c2_d_slaveAddress;
  uint8_T c2_data;
  uint8_T c2_e_slaveAddress;
  uint8_T c2_output;
  uint8_T c2_slaveAddress;
  uint8_T c2_status;
  uint8_T c2_val;
  uint8_T c2_varargin_1;
  (void)chartInstance;
  c2_b_obj = c2_obj;
  c2_c_obj = c2_b_obj->Device;
  c2_d_obj = c2_c_obj->InterfaceObj;
  c2_slaveAddress = c2_c_obj->DeviceAddress;
  c2_val = 0U;
  c2_e_obj = c2_d_obj;
  c2_b_slaveAddress = c2_slaveAddress;
  c2_data = 32U;
  c2_status = MW_I2C_MasterWrite(c2_e_obj->MW_I2C_HANDLE, c2_b_slaveAddress,
    &c2_data, 1U, true, false);
  if (c2_status == 0) {
    c2_f_obj = c2_d_obj;
    c2_c_slaveAddress = c2_slaveAddress;
    MW_I2C_MasterRead(c2_f_obj->MW_I2C_HANDLE, c2_c_slaveAddress, &c2_output, 1U,
                      false, true);
    c2_val = c2_output;
  }

  c2_varargin_1 = c2_val;
  c2_a = c2_varargin_1;
  c2_b_a = c2_a;
  c2_c = (uint8_T)(c2_b_a | 28);
  c2_g_obj = c2_b_obj->Device;
  c2_b_data = c2_c;
  c2_h_obj = c2_g_obj->InterfaceObj;
  c2_d_slaveAddress = c2_g_obj->DeviceAddress;
  c2_c_data = c2_b_data;
  c2_d_data[0] = 32U;
  c2_d_data[1] = c2_c_data;
  c2_i_obj = c2_h_obj;
  c2_e_slaveAddress = c2_d_slaveAddress;
  MW_I2C_MasterWrite(c2_i_obj->MW_I2C_HANDLE, c2_e_slaveAddress, &c2_d_data[0],
                     2U, false, false);
  c2_b_obj->MagnetometerODR = 80.0;
}

static void c2_sensorBoard_read(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const emlrtStack *c2_sp, c2_lsm9ds1 *c2_obj, real_T
  c2_varargout_1[3], real_T c2_varargout_2[3], real_T c2_varargout_3[3])
{
  static char_T c2_b_cv[4] = { 's', 't', 'e', 'p' };

  c2_cell_wrap_11 c2_localOutputs[2];
  c2_cell_wrap_11 c2_b_localOutputs[1];
  c2_lsm9ds1 *c2_b_obj;
  c2_lsm9ds1 *c2_c_obj;
  emlrtStack c2_b_st;
  emlrtStack c2_c_st;
  emlrtStack c2_st;
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  const mxArray *c2_y = NULL;
  real_T c2_dv[3];
  real_T c2_dv1[3];
  int32_T c2_i;
  int32_T c2_i1;
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  c2_st.prev = c2_sp;
  c2_st.tls = c2_sp->tls;
  c2_b_st.prev = &c2_st;
  c2_b_st.tls = c2_st.tls;
  c2_c_st.prev = &c2_b_st;
  c2_c_st.tls = c2_b_st.tls;
  c2_st.site = &c2_wd_emlrtRSI;
  c2_b_obj = c2_obj;
  if (c2_b_obj->isInitialized == 2) {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_cv, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_cv, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call(&c2_st, &c2_b_emlrtMCI, "error", 0U, 2U, 14, c2_y, 14,
                sf_mex_call(&c2_st, NULL, "getString", 1U, 1U, 14, sf_mex_call
      (&c2_st, NULL, "message", 1U, 2U, 14, c2_b_y, 14, c2_c_y)));
  }

  if (c2_b_obj->isInitialized != 1) {
    c2_b_st.site = &c2_b_emlrtRSI;
    c2_SystemCore_setupAndReset(chartInstance, &c2_b_st, c2_b_obj);
  }

  c2_b_st.site = &c2_b_emlrtRSI;
  c2_c_obj = c2_b_obj;
  c2_c_st.site = &c2_xd_emlrtRSI;
  c2_SystemCore_step(chartInstance, &c2_c_st, c2_c_obj->SensorObjects.f1, c2_dv,
                     c2_dv1);
  for (c2_i = 0; c2_i < 3; c2_i++) {
    c2_localOutputs[0].f1[c2_i] = c2_dv[c2_i];
  }

  for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
    c2_localOutputs[1].f1[c2_i1] = c2_dv1[c2_i1];
  }

  for (c2_i2 = 0; c2_i2 < 3; c2_i2++) {
    c2_varargout_1[c2_i2] = c2_localOutputs[0].f1[c2_i2];
  }

  for (c2_i3 = 0; c2_i3 < 3; c2_i3++) {
    c2_varargout_2[c2_i3] = c2_localOutputs[1].f1[c2_i3];
  }

  c2_c_st.site = &c2_xd_emlrtRSI;
  c2_b_SystemCore_step(chartInstance, &c2_c_st, c2_c_obj->SensorObjects.f2,
                       c2_b_localOutputs[0].f1);
  for (c2_i4 = 0; c2_i4 < 3; c2_i4++) {
    c2_varargout_3[c2_i4] = c2_b_localOutputs[0].f1[c2_i4];
  }

  c2_c_obj->SamplesRead++;
}

static void c2_SystemCore_setupAndReset
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance, const emlrtStack
   *c2_sp, c2_lsm9ds1 *c2_obj)
{
  static char_T c2_b_cv[5] = { 's', 'e', 't', 'u', 'p' };

  c2_lsm9ds1 *c2_b_obj;
  c2_lsm9ds1 *c2_c_obj;
  emlrtStack c2_st;
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  const mxArray *c2_y = NULL;
  (void)chartInstance;
  c2_st.prev = c2_sp;
  c2_st.tls = c2_sp->tls;
  c2_st.site = &c2_b_emlrtRSI;
  c2_b_obj = c2_obj;
  c2_b_obj->isSetupComplete = false;
  if (c2_b_obj->isInitialized != 0) {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv1, 10, 0U, 1U, 0U, 2, 1, 51),
                  false);
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_cv1, 10, 0U, 1U, 0U, 2, 1, 51),
                  false);
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_cv, 10, 0U, 1U, 0U, 2, 1, 5),
                  false);
    sf_mex_call(&c2_st, &c2_b_emlrtMCI, "error", 0U, 2U, 14, c2_y, 14,
                sf_mex_call(&c2_st, NULL, "getString", 1U, 1U, 14, sf_mex_call
      (&c2_st, NULL, "message", 1U, 2U, 14, c2_b_y, 14, c2_c_y)));
  }

  c2_b_obj->isInitialized = 1;
  c2_b_obj->isSetupComplete = true;
  c2_st.site = &c2_b_emlrtRSI;
  c2_c_obj = c2_obj;
  c2_c_obj->SamplesRead = 0.0;
}

static void c2_SystemCore_step(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const emlrtStack *c2_sp,
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_obj, real_T c2_varargout_1[3],
  real_T c2_varargout_2[3])
{
  static char_T c2_b_cv1[5] = { 's', 'e', 't', 'u', 'p' };

  static char_T c2_b_cv[4] = { 's', 't', 'e', 'p' };

  c2_arduinodriver_ArduinoI2C *c2_i_obj;
  c2_arduinodriver_ArduinoI2C *c2_k_obj;
  c2_arduinodriver_ArduinoI2C *c2_l_obj;
  c2_arduinodriver_ArduinoI2C *c2_q_obj;
  c2_arduinodriver_ArduinoI2C *c2_r_obj;
  c2_arduinodriver_ArduinoI2C *c2_s_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_h_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_p_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_b_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_c_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_d_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_e_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_f_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_g_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_j_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_m_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_n_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_o_obj;
  c2_sensors_internal_lsm9ds1_accel_gyro *c2_t_obj;
  emlrtStack c2_b_st;
  emlrtStack c2_st;
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  const mxArray *c2_d_y = NULL;
  const mxArray *c2_e_y = NULL;
  const mxArray *c2_f_y = NULL;
  const mxArray *c2_y = NULL;
  real_T c2_d_data[6];
  real_T c2_dataAccel[3];
  real_T c2_dataAngularVelocity[3];
  real_T c2_accel_x;
  real_T c2_accel_y;
  real_T c2_accel_z;
  real_T c2_gyro_x;
  real_T c2_gyro_y;
  real_T c2_gyro_z;
  int32_T c2_i;
  int32_T c2_i1;
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  int16_T c2_a;
  int16_T c2_b;
  int16_T c2_b_a;
  int16_T c2_b_b;
  int16_T c2_b_c;
  int16_T c2_b_varargin_1;
  int16_T c2_b_varargin_2;
  int16_T c2_c;
  int16_T c2_c_a;
  int16_T c2_c_b;
  int16_T c2_c_c;
  int16_T c2_c_varargin_1;
  int16_T c2_c_varargin_2;
  int16_T c2_d_a;
  int16_T c2_d_b;
  int16_T c2_d_c;
  int16_T c2_d_varargin_1;
  int16_T c2_d_varargin_2;
  int16_T c2_e_a;
  int16_T c2_e_b;
  int16_T c2_e_c;
  int16_T c2_e_varargin_1;
  int16_T c2_e_varargin_2;
  int16_T c2_f_a;
  int16_T c2_f_b;
  int16_T c2_f_c;
  int16_T c2_f_varargin_1;
  int16_T c2_f_varargin_2;
  int16_T c2_g_a;
  int16_T c2_g_b;
  int16_T c2_g_c;
  int16_T c2_h_a;
  int16_T c2_h_b;
  int16_T c2_h_c;
  int16_T c2_i_a;
  int16_T c2_i_b;
  int16_T c2_i_c;
  int16_T c2_j_a;
  int16_T c2_j_b;
  int16_T c2_j_c;
  int16_T c2_k_a;
  int16_T c2_k_b;
  int16_T c2_k_c;
  int16_T c2_l_a;
  int16_T c2_l_b;
  int16_T c2_l_c;
  int16_T c2_m_a;
  int16_T c2_n_a;
  int16_T c2_o_a;
  int16_T c2_p_a;
  int16_T c2_q_a;
  int16_T c2_r_a;
  int16_T c2_s_a;
  int16_T c2_t_a;
  int16_T c2_u_a;
  int16_T c2_v_a;
  int16_T c2_varargin_1;
  int16_T c2_varargin_2;
  int16_T c2_w_a;
  int16_T c2_x_a;
  uint8_T c2_data[6];
  uint8_T c2_b_data;
  uint8_T c2_b_slaveAddress;
  uint8_T c2_b_status;
  uint8_T c2_c_data;
  uint8_T c2_c_slaveAddress;
  uint8_T c2_d_slaveAddress;
  uint8_T c2_e_slaveAddress;
  uint8_T c2_f_slaveAddress;
  uint8_T c2_slaveAddress;
  uint8_T c2_status;
  (void)chartInstance;
  c2_st.prev = c2_sp;
  c2_st.tls = c2_sp->tls;
  c2_b_st.prev = &c2_st;
  c2_b_st.tls = c2_st.tls;
  if (c2_obj->isInitialized == 2) {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_cv, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_cv, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call(c2_sp, &c2_b_emlrtMCI, "error", 0U, 2U, 14, c2_y, 14,
                sf_mex_call(c2_sp, NULL, "getString", 1U, 1U, 14, sf_mex_call
      (c2_sp, NULL, "message", 1U, 2U, 14, c2_b_y, 14, c2_c_y)));
  }

  if (c2_obj->isInitialized != 1) {
    c2_st.site = &c2_b_emlrtRSI;
    c2_b_obj = c2_obj;
    c2_b_st.site = &c2_b_emlrtRSI;
    c2_d_obj = c2_b_obj;
    c2_d_obj->isSetupComplete = false;
    if (c2_d_obj->isInitialized != 0) {
      c2_d_y = NULL;
      sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_cv1, 10, 0U, 1U, 0U, 2, 1, 51),
                    false);
      c2_e_y = NULL;
      sf_mex_assign(&c2_e_y, sf_mex_create("y", c2_cv1, 10, 0U, 1U, 0U, 2, 1, 51),
                    false);
      c2_f_y = NULL;
      sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_b_cv1, 10, 0U, 1U, 0U, 2, 1,
        5), false);
      sf_mex_call(&c2_b_st, &c2_b_emlrtMCI, "error", 0U, 2U, 14, c2_d_y, 14,
                  sf_mex_call(&c2_b_st, NULL, "getString", 1U, 1U, 14,
        sf_mex_call(&c2_b_st, NULL, "message", 1U, 2U, 14, c2_e_y, 14, c2_f_y)));
    }

    c2_d_obj->isInitialized = 1;
    c2_d_obj->isSetupComplete = true;
    c2_b_st.site = &c2_b_emlrtRSI;
    c2_j_obj = c2_b_obj;
    c2_j_obj->SamplesRead = 0.0;
  }

  c2_st.site = &c2_b_emlrtRSI;
  c2_c_obj = c2_obj;
  c2_b_st.site = &c2_yd_emlrtRSI;
  c2_e_obj = c2_c_obj;
  c2_f_obj = c2_e_obj;
  c2_g_obj = c2_f_obj;
  c2_h_obj = c2_g_obj->Device;
  c2_i_obj = c2_h_obj->InterfaceObj;
  c2_slaveAddress = c2_h_obj->DeviceAddress;
  for (c2_i = 0; c2_i < 6; c2_i++) {
    c2_data[c2_i] = 0U;
  }

  c2_k_obj = c2_i_obj;
  c2_b_slaveAddress = c2_slaveAddress;
  c2_b_data = 40U;
  c2_status = MW_I2C_MasterWrite(c2_k_obj->MW_I2C_HANDLE, c2_b_slaveAddress,
    &c2_b_data, 1U, true, false);
  if (c2_status == 0) {
    c2_l_obj = c2_i_obj;
    c2_c_slaveAddress = c2_slaveAddress;
    MW_I2C_MasterRead(c2_l_obj->MW_I2C_HANDLE, c2_c_slaveAddress, &c2_data[0],
                      6U, false, true);
  }

  c2_m_obj = c2_f_obj;
  c2_a = c2_data[1];
  c2_b_a = c2_a;
  c2_c = (int16_T)(c2_b_a << 8);
  c2_varargin_1 = c2_data[0];
  c2_varargin_2 = c2_c;
  c2_c_a = c2_varargin_1;
  c2_b = c2_varargin_2;
  c2_d_a = c2_c_a;
  c2_b_b = c2_b;
  c2_b_c = (int16_T)(c2_d_a | c2_b_b);
  c2_accel_x = (real_T)c2_b_c;
  c2_e_a = c2_data[3];
  c2_f_a = c2_e_a;
  c2_c_c = (int16_T)(c2_f_a << 8);
  c2_b_varargin_1 = c2_data[2];
  c2_b_varargin_2 = c2_c_c;
  c2_g_a = c2_b_varargin_1;
  c2_c_b = c2_b_varargin_2;
  c2_h_a = c2_g_a;
  c2_d_b = c2_c_b;
  c2_d_c = (int16_T)(c2_h_a | c2_d_b);
  c2_accel_y = (real_T)c2_d_c;
  c2_i_a = c2_data[5];
  c2_j_a = c2_i_a;
  c2_e_c = (int16_T)(c2_j_a << 8);
  c2_c_varargin_1 = c2_data[4];
  c2_c_varargin_2 = c2_e_c;
  c2_k_a = c2_c_varargin_1;
  c2_e_b = c2_c_varargin_2;
  c2_l_a = c2_k_a;
  c2_f_b = c2_e_b;
  c2_f_c = (int16_T)(c2_l_a | c2_f_b);
  c2_accel_z = (real_T)c2_f_c;
  c2_dataAccel[0] = c2_m_obj->AccelerometerResolution * c2_accel_x;
  c2_dataAccel[1] = c2_m_obj->AccelerometerResolution * c2_accel_y;
  c2_dataAccel[2] = c2_m_obj->AccelerometerResolution * c2_accel_z;
  for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
    c2_dataAccel[c2_i1] *= 9.81;
  }

  c2_n_obj = c2_e_obj;
  c2_o_obj = c2_n_obj;
  c2_p_obj = c2_o_obj->Device;
  c2_q_obj = c2_p_obj->InterfaceObj;
  c2_d_slaveAddress = c2_p_obj->DeviceAddress;
  for (c2_i2 = 0; c2_i2 < 6; c2_i2++) {
    c2_data[c2_i2] = 0U;
  }

  c2_r_obj = c2_q_obj;
  c2_e_slaveAddress = c2_d_slaveAddress;
  c2_c_data = 24U;
  c2_b_status = MW_I2C_MasterWrite(c2_r_obj->MW_I2C_HANDLE, c2_e_slaveAddress,
    &c2_c_data, 1U, true, false);
  if (c2_b_status == 0) {
    c2_s_obj = c2_q_obj;
    c2_f_slaveAddress = c2_d_slaveAddress;
    MW_I2C_MasterRead(c2_s_obj->MW_I2C_HANDLE, c2_f_slaveAddress, &c2_data[0],
                      6U, false, true);
  }

  c2_t_obj = c2_n_obj;
  c2_m_a = c2_data[1];
  c2_n_a = c2_m_a;
  c2_g_c = (int16_T)(c2_n_a << 8);
  c2_d_varargin_1 = c2_data[0];
  c2_d_varargin_2 = c2_g_c;
  c2_o_a = c2_d_varargin_1;
  c2_g_b = c2_d_varargin_2;
  c2_p_a = c2_o_a;
  c2_h_b = c2_g_b;
  c2_h_c = (int16_T)(c2_p_a | c2_h_b);
  c2_gyro_x = (real_T)c2_h_c;
  c2_q_a = c2_data[3];
  c2_r_a = c2_q_a;
  c2_i_c = (int16_T)(c2_r_a << 8);
  c2_e_varargin_1 = c2_data[2];
  c2_e_varargin_2 = c2_i_c;
  c2_s_a = c2_e_varargin_1;
  c2_i_b = c2_e_varargin_2;
  c2_t_a = c2_s_a;
  c2_j_b = c2_i_b;
  c2_j_c = (int16_T)(c2_t_a | c2_j_b);
  c2_gyro_y = (real_T)c2_j_c;
  c2_u_a = c2_data[5];
  c2_v_a = c2_u_a;
  c2_k_c = (int16_T)(c2_v_a << 8);
  c2_f_varargin_1 = c2_data[4];
  c2_f_varargin_2 = c2_k_c;
  c2_w_a = c2_f_varargin_1;
  c2_k_b = c2_f_varargin_2;
  c2_x_a = c2_w_a;
  c2_l_b = c2_k_b;
  c2_l_c = (int16_T)(c2_x_a | c2_l_b);
  c2_gyro_z = (real_T)c2_l_c;
  c2_dataAngularVelocity[0] = c2_t_obj->GyroscopeResolution * c2_gyro_x;
  c2_dataAngularVelocity[1] = c2_t_obj->GyroscopeResolution * c2_gyro_y;
  c2_dataAngularVelocity[2] = c2_t_obj->GyroscopeResolution * c2_gyro_z;
  for (c2_i3 = 0; c2_i3 < 3; c2_i3++) {
    c2_dataAngularVelocity[c2_i3] *= 3.1415926535897931;
  }

  for (c2_i4 = 0; c2_i4 < 3; c2_i4++) {
    c2_dataAngularVelocity[c2_i4] /= 180.0;
  }

  for (c2_i5 = 0; c2_i5 < 3; c2_i5++) {
    c2_d_data[c2_i5] = c2_dataAccel[c2_i5];
  }

  for (c2_i6 = 0; c2_i6 < 3; c2_i6++) {
    c2_d_data[c2_i6 + 3] = c2_dataAngularVelocity[c2_i6];
  }

  for (c2_i7 = 0; c2_i7 < 3; c2_i7++) {
    c2_varargout_1[c2_i7] = c2_d_data[c2_i7];
  }

  for (c2_i8 = 0; c2_i8 < 3; c2_i8++) {
    c2_varargout_2[c2_i8] = c2_d_data[c2_i8 + 3];
  }

  c2_c_obj->SamplesRead++;
}

static void c2_b_SystemCore_step(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const emlrtStack *c2_sp, c2_sensors_internal_lsm9ds1_mag
  *c2_obj, real_T c2_varargout_1[3])
{
  static char_T c2_b_cv1[5] = { 's', 'e', 't', 'u', 'p' };

  static char_T c2_b_cv[4] = { 's', 't', 'e', 'p' };

  c2_arduinodriver_ArduinoI2C *c2_i_obj;
  c2_arduinodriver_ArduinoI2C *c2_k_obj;
  c2_arduinodriver_ArduinoI2C *c2_l_obj;
  c2_matlabshared_sensors_coder_matlab_device *c2_h_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_b_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_c_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_d_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_e_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_f_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_g_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_j_obj;
  c2_sensors_internal_lsm9ds1_mag *c2_m_obj;
  emlrtStack c2_b_st;
  emlrtStack c2_st;
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  const mxArray *c2_d_y = NULL;
  const mxArray *c2_e_y = NULL;
  const mxArray *c2_f_y = NULL;
  const mxArray *c2_y = NULL;
  real_T c2_mag_x;
  real_T c2_mag_y;
  real_T c2_mag_z;
  int32_T c2_i;
  int32_T c2_i1;
  int16_T c2_a;
  int16_T c2_b;
  int16_T c2_b_a;
  int16_T c2_b_b;
  int16_T c2_b_c;
  int16_T c2_b_varargin_1;
  int16_T c2_b_varargin_2;
  int16_T c2_c;
  int16_T c2_c_a;
  int16_T c2_c_b;
  int16_T c2_c_c;
  int16_T c2_c_varargin_1;
  int16_T c2_c_varargin_2;
  int16_T c2_d_a;
  int16_T c2_d_b;
  int16_T c2_d_c;
  int16_T c2_e_a;
  int16_T c2_e_b;
  int16_T c2_e_c;
  int16_T c2_f_a;
  int16_T c2_f_b;
  int16_T c2_f_c;
  int16_T c2_g_a;
  int16_T c2_h_a;
  int16_T c2_i_a;
  int16_T c2_j_a;
  int16_T c2_k_a;
  int16_T c2_l_a;
  int16_T c2_varargin_1;
  int16_T c2_varargin_2;
  uint8_T c2_magData[6];
  uint8_T c2_b_slaveAddress;
  uint8_T c2_c_slaveAddress;
  uint8_T c2_data;
  uint8_T c2_slaveAddress;
  uint8_T c2_status;
  (void)chartInstance;
  c2_st.prev = c2_sp;
  c2_st.tls = c2_sp->tls;
  c2_b_st.prev = &c2_st;
  c2_b_st.tls = c2_st.tls;
  if (c2_obj->isInitialized == 2) {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_cv, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_cv, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call(c2_sp, &c2_b_emlrtMCI, "error", 0U, 2U, 14, c2_y, 14,
                sf_mex_call(c2_sp, NULL, "getString", 1U, 1U, 14, sf_mex_call
      (c2_sp, NULL, "message", 1U, 2U, 14, c2_b_y, 14, c2_c_y)));
  }

  if (c2_obj->isInitialized != 1) {
    c2_st.site = &c2_b_emlrtRSI;
    c2_b_obj = c2_obj;
    c2_b_st.site = &c2_b_emlrtRSI;
    c2_d_obj = c2_b_obj;
    c2_d_obj->isSetupComplete = false;
    if (c2_d_obj->isInitialized != 0) {
      c2_d_y = NULL;
      sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_cv1, 10, 0U, 1U, 0U, 2, 1, 51),
                    false);
      c2_e_y = NULL;
      sf_mex_assign(&c2_e_y, sf_mex_create("y", c2_cv1, 10, 0U, 1U, 0U, 2, 1, 51),
                    false);
      c2_f_y = NULL;
      sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_b_cv1, 10, 0U, 1U, 0U, 2, 1,
        5), false);
      sf_mex_call(&c2_b_st, &c2_b_emlrtMCI, "error", 0U, 2U, 14, c2_d_y, 14,
                  sf_mex_call(&c2_b_st, NULL, "getString", 1U, 1U, 14,
        sf_mex_call(&c2_b_st, NULL, "message", 1U, 2U, 14, c2_e_y, 14, c2_f_y)));
    }

    c2_d_obj->isInitialized = 1;
    c2_d_obj->isSetupComplete = true;
    c2_b_st.site = &c2_b_emlrtRSI;
    c2_j_obj = c2_b_obj;
    c2_j_obj->SamplesRead = 0.0;
  }

  c2_st.site = &c2_b_emlrtRSI;
  c2_c_obj = c2_obj;
  c2_b_st.site = &c2_yd_emlrtRSI;
  c2_e_obj = c2_c_obj;
  c2_f_obj = c2_e_obj;
  c2_g_obj = c2_f_obj;
  c2_h_obj = c2_g_obj->Device;
  c2_i_obj = c2_h_obj->InterfaceObj;
  c2_slaveAddress = c2_h_obj->DeviceAddress;
  for (c2_i = 0; c2_i < 6; c2_i++) {
    c2_magData[c2_i] = 0U;
  }

  c2_k_obj = c2_i_obj;
  c2_b_slaveAddress = c2_slaveAddress;
  c2_data = 40U;
  c2_status = MW_I2C_MasterWrite(c2_k_obj->MW_I2C_HANDLE, c2_b_slaveAddress,
    &c2_data, 1U, true, false);
  if (c2_status == 0) {
    c2_l_obj = c2_i_obj;
    c2_c_slaveAddress = c2_slaveAddress;
    MW_I2C_MasterRead(c2_l_obj->MW_I2C_HANDLE, c2_c_slaveAddress, &c2_magData[0],
                      6U, false, true);
  }

  c2_m_obj = c2_f_obj;
  c2_a = c2_magData[1];
  c2_b_a = c2_a;
  c2_c = (int16_T)(c2_b_a << 8);
  c2_varargin_1 = c2_magData[0];
  c2_varargin_2 = c2_c;
  c2_c_a = c2_varargin_1;
  c2_b = c2_varargin_2;
  c2_d_a = c2_c_a;
  c2_b_b = c2_b;
  c2_b_c = (int16_T)(c2_d_a | c2_b_b);
  c2_mag_x = (real_T)c2_b_c;
  c2_e_a = c2_magData[3];
  c2_f_a = c2_e_a;
  c2_c_c = (int16_T)(c2_f_a << 8);
  c2_b_varargin_1 = c2_magData[2];
  c2_b_varargin_2 = c2_c_c;
  c2_g_a = c2_b_varargin_1;
  c2_c_b = c2_b_varargin_2;
  c2_h_a = c2_g_a;
  c2_d_b = c2_c_b;
  c2_d_c = (int16_T)(c2_h_a | c2_d_b);
  c2_mag_y = (real_T)c2_d_c;
  c2_i_a = c2_magData[5];
  c2_j_a = c2_i_a;
  c2_e_c = (int16_T)(c2_j_a << 8);
  c2_c_varargin_1 = c2_magData[4];
  c2_c_varargin_2 = c2_e_c;
  c2_k_a = c2_c_varargin_1;
  c2_e_b = c2_c_varargin_2;
  c2_l_a = c2_k_a;
  c2_f_b = c2_e_b;
  c2_f_c = (int16_T)(c2_l_a | c2_f_b);
  c2_mag_z = (real_T)c2_f_c;
  c2_varargout_1[0] = c2_mag_x * c2_m_obj->MagnetometerResolution;
  c2_varargout_1[1] = c2_mag_y * c2_m_obj->MagnetometerResolution;
  c2_varargout_1[2] = c2_mag_z * c2_m_obj->MagnetometerResolution;
  for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
    c2_varargout_1[c2_i1] *= 100.0;
  }

  c2_c_obj->SamplesRead++;
}

static real_T c2_sqrt(SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance,
                      const emlrtStack *c2_sp, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_sqrt(chartInstance, c2_sp, &c2_b_x);
  return c2_b_x;
}

static real_T c2_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_b_pitch, const char_T *c2_identifier)
{
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  c2_thisId.fIdentifier = (const char_T *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_pitch), &c2_thisId);
  sf_mex_destroy(&c2_b_pitch);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_d;
  real_T c2_y;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static real_T c2_c_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_b_pitchAccelOffset, const char_T
  *c2_identifier, boolean_T *c2_svPtr)
{
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  c2_thisId.fIdentifier = (const char_T *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_pitchAccelOffset),
    &c2_thisId, c2_svPtr);
  sf_mex_destroy(&c2_b_pitchAccelOffset);
  return c2_y;
}

static real_T c2_d_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T *c2_svPtr)
{
  real_T c2_d;
  real_T c2_y;
  (void)chartInstance;
  if (mxIsEmpty(c2_u)) {
    *c2_svPtr = false;
  } else {
    *c2_svPtr = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static uint8_T c2_e_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_arduino_imu_pitch_roll, const
  char_T *c2_identifier)
{
  emlrtMsgIdentifier c2_thisId;
  uint8_T c2_y;
  c2_thisId.fIdentifier = (const char_T *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_arduino_imu_pitch_roll), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_arduino_imu_pitch_roll);
  return c2_y;
}

static uint8_T c2_f_emlrt_marshallIn(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_b_u;
  uint8_T c2_y;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_b_u, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_b_u;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_slStringInitializeDynamicBuffers
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_chart_data_browse_helper
  (SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance, int32_T
   c2_ssIdNumber, const mxArray **c2_mxData, uint8_T *c2_isValueTooBig)
{
  real_T c2_d;
  real_T c2_d1;
  *c2_mxData = NULL;
  *c2_mxData = NULL;
  *c2_isValueTooBig = 0U;
  switch (c2_ssIdNumber) {
   case 6U:
    c2_d = *chartInstance->c2_pitch;
    sf_mex_assign(c2_mxData, sf_mex_create("mxData", &c2_d, 0, 0U, 0U, 0U, 0),
                  false);
    break;

   case 7U:
    c2_d1 = *chartInstance->c2_roll;
    sf_mex_assign(c2_mxData, sf_mex_create("mxData", &c2_d1, 0, 0U, 0U, 0U, 0),
                  false);
    break;
  }
}

static void c2_b_sqrt(SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance,
                      const emlrtStack *c2_sp, real_T *c2_x)
{
  static char_T c2_b_cv[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  static char_T c2_b_cv1[4] = { 's', 'q', 'r', 't' };

  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  const mxArray *c2_y = NULL;
  real_T c2_b_x;
  boolean_T c2_b_p;
  boolean_T c2_p;
  (void)chartInstance;
  c2_b_x = *c2_x;
  if (c2_b_x < 0.0) {
    c2_p = true;
  } else {
    c2_p = false;
  }

  c2_b_p = c2_p;
  if (c2_b_p) {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_b_cv, 10, 0U, 1U, 0U, 2, 1, 30),
                  false);
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_cv, 10, 0U, 1U, 0U, 2, 1, 30),
                  false);
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_cv1, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call(c2_sp, &c2_c_emlrtMCI, "error", 0U, 2U, 14, c2_y, 14,
                sf_mex_call(c2_sp, NULL, "getString", 1U, 1U, 14, sf_mex_call
      (c2_sp, NULL, "message", 1U, 2U, 14, c2_b_y, 14, c2_c_y)));
  }

  *c2_x = muDoubleScalarSqrt(*c2_x);
}

static void init_dsm_address_info(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc2_arduino_imu_pitch_rollInstanceStruct
  *chartInstance)
{
  chartInstance->c2_covrtInstance = (CovrtStateflowInstance *)
    sfrtGetCovrtInstance(chartInstance->S);
  chartInstance->c2_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c2_pitch = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_roll = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SFunction Glue Code */
void sf_c2_arduino_imu_pitch_roll_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2277634816U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4067716958U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(840549687U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2389936964U);
}

mxArray *sf_c2_arduino_imu_pitch_roll_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,4);
  mxSetCell(mxcell3p, 0, mxCreateString("arduinodriver.ArduinoDigitalIO"));
  mxSetCell(mxcell3p, 1, mxCreateString("arduinodriver.ArduinoAnalogInput"));
  mxSetCell(mxcell3p, 2, mxCreateString("arduinodriver.arduinoPWMAddLibrary"));
  mxSetCell(mxcell3p, 3, mxCreateString("arduinodriver.ArduinoI2CAddLibrary"));
  return(mxcell3p);
}

mxArray *sf_c2_arduino_imu_pitch_roll_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("late");
  mxArray *fallbackReason = mxCreateString("ir_function_calls");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("MW_I2C_Close");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c2_arduino_imu_pitch_roll_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_arduino_imu_pitch_roll(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  mxArray *mxVarInfo = sf_mex_decode(
    "eNrllr1OhEAQgBfCXYREjtLC4hIbS2NlJ1aGwthY2BHCLboJf+Hv4M7CR/MR7hF8BB/BXXY5uA2"
    "R4khuDZsMk5lkhpn9mAxAsp4APjqWa/yYY32GRQb0zJgtYdGYpn5l719iyaoYEn+auNYK69AJat"
    "vJSyv0ojr/HWjzz3vyS538KvPT83N/XPyFyccrPfGzTrzB7Bhl7ju7n+aeTtfHZd3HzUAfykEfC"
    "kgi3x+nfsM8Lp6+3xyo3+A4GA2HB9eF/rPnpTAjeXbg7+9V4r5XmdkkxgcIxD319PUjc/0YTTO7"
    "0qS6GuVe/gsfjeOj7fmEbz5s85yeT0H5xOtJ8VlwfBYNn8cqiej4iMHne8PmZzspPuccH2LHCSx"
    "eUADTzAliUebHzCkfo5j8/JAd2lk/gsxPxeZnMyk+KsdHbfi060cMPp9rNj/lpPjoHB+d8emsH0"
    "HmZ0v5fH0Iwmc5Sh2vA3yuOD7ERqntuBkqoO3e2k6yylEY2SjI7frHwSb4Dvr8BagegNo="
    );
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_arduino_imu_pitch_roll_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "sB9vJXpaDgOfbfj2ApBx5rE";
}

static void sf_opaque_initialize_c2_arduino_imu_pitch_roll(void
  *chartInstanceVar)
{
  initialize_params_c2_arduino_imu_pitch_roll
    ((SFc2_arduino_imu_pitch_rollInstanceStruct*) chartInstanceVar);
  initialize_c2_arduino_imu_pitch_roll
    ((SFc2_arduino_imu_pitch_rollInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_arduino_imu_pitch_roll(void *chartInstanceVar)
{
  enable_c2_arduino_imu_pitch_roll((SFc2_arduino_imu_pitch_rollInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_arduino_imu_pitch_roll(void *chartInstanceVar)
{
  disable_c2_arduino_imu_pitch_roll((SFc2_arduino_imu_pitch_rollInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_arduino_imu_pitch_roll(void *chartInstanceVar)
{
  sf_gateway_c2_arduino_imu_pitch_roll
    ((SFc2_arduino_imu_pitch_rollInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_arduino_imu_pitch_roll
  (SimStruct* S)
{
  return get_sim_state_c2_arduino_imu_pitch_roll
    ((SFc2_arduino_imu_pitch_rollInstanceStruct *)sf_get_chart_instance_ptr(S));/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_arduino_imu_pitch_roll(SimStruct* S,
  const mxArray *st)
{
  set_sim_state_c2_arduino_imu_pitch_roll
    ((SFc2_arduino_imu_pitch_rollInstanceStruct*)sf_get_chart_instance_ptr(S),
     st);
}

static void sf_opaque_cleanup_runtime_resources_c2_arduino_imu_pitch_roll(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_arduino_imu_pitch_rollInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_arduino_imu_pitch_roll_optimization_info();
    }

    mdl_cleanup_runtime_resources_c2_arduino_imu_pitch_roll
      ((SFc2_arduino_imu_pitch_rollInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_mdl_start_c2_arduino_imu_pitch_roll(void *chartInstanceVar)
{
  mdl_start_c2_arduino_imu_pitch_roll((SFc2_arduino_imu_pitch_rollInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_mdl_terminate_c2_arduino_imu_pitch_roll(void
  *chartInstanceVar)
{
  mdl_terminate_c2_arduino_imu_pitch_roll
    ((SFc2_arduino_imu_pitch_rollInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_arduino_imu_pitch_roll
    ((SFc2_arduino_imu_pitch_rollInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_arduino_imu_pitch_roll(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  sf_warn_if_symbolic_dimension_param_changed(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_arduino_imu_pitch_roll
      ((SFc2_arduino_imu_pitch_rollInstanceStruct*)sf_get_chart_instance_ptr(S));
    initSimStructsc2_arduino_imu_pitch_roll
      ((SFc2_arduino_imu_pitch_rollInstanceStruct*)sf_get_chart_instance_ptr(S));
  }
}

const char* sf_c2_arduino_imu_pitch_roll_get_post_codegen_info(void)
{
  int i;
  const char* encStrCodegen [105] = {
    "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",
    ""
  };

  static char newstr [7789] = "";
  newstr[0] = '\0';
  for (i = 0; i < 105; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c2_arduino_imu_pitch_roll(SimStruct *S)
{
  const char* newstr = sf_c2_arduino_imu_pitch_roll_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(4090320639U));
  ssSetChecksum1(S,(429190951U));
  ssSetChecksum2(S,(1409661113U));
  ssSetChecksum3(S,(3477566380U));
}

static void mdlRTW_c2_arduino_imu_pitch_roll(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlSetupRuntimeResources_c2_arduino_imu_pitch_roll(SimStruct *S)
{
  SFc2_arduino_imu_pitch_rollInstanceStruct *chartInstance;
  chartInstance = (SFc2_arduino_imu_pitch_rollInstanceStruct *)utMalloc(sizeof
    (SFc2_arduino_imu_pitch_rollInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc2_arduino_imu_pitch_rollInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  if (ssGetSampleTime(S, 0) == CONTINUOUS_SAMPLE_TIME && ssGetOffsetTime(S, 0) ==
      0 && ssGetNumContStates(ssGetRootSS(S)) > 0 &&
      !supportsLegacyBehaviorForPersistentVarInContinuousTime(S)) {
    sf_error_out_about_continuous_sample_time_with_persistent_vars(S);
  }

  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.mdlStart =
    sf_opaque_mdl_start_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.mdlTerminate =
    sf_opaque_mdl_terminate_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_arduino_imu_pitch_roll;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartEventFcn = NULL;
  chartInstance->chartInfo.chartStateSetterFcn = NULL;
  chartInstance->chartInfo.chartStateGetterFcn = NULL;
  chartInstance->S = S;
  chartInstance->chartInfo.dispatchToExportedFcn = NULL;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0,
    chartInstance->c2_JITStateAnimation,
    chartInstance->c2_JITTransitionAnimation);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_setup_runtime_resources_c2_arduino_imu_pitch_roll(chartInstance);
}

void c2_arduino_imu_pitch_roll_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c2_arduino_imu_pitch_roll(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_arduino_imu_pitch_roll(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_arduino_imu_pitch_roll(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_arduino_imu_pitch_roll_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
