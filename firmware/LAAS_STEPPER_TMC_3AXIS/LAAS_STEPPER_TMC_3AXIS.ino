/*
Thomas Flayols LAAS CNRS April 2025

Control stepper motor via TMC2130 drivers connected in SPI + Step/DIR
Receive simple commands via USB (emulated Serial)
Send back motor status every (FEEDBACK_PERIOD_MS) ms

Format:
    Commands "X 100 1000 5000\n" //Axis 'X' goto position 100 with max speed set to 1000 and max axeleration set to 5000
    State: "X 95 65 Y 0 0 Z 0 0\n" //X position is 95, velocity is 65. Y position is 0 etc...
*/
#include <TMCStepper.h>
#include <AccelStepper.h>

using namespace TMC2130_n;

// Software SPI pins
#define SW_MOSI 19
#define SW_MISO 16
#define SW_SCK  18

// Timing and hardware constants
#define FEEDBACK_PERIOD_MS 100
#define R_SENSE            0.11f

// Axis limits (steps, steps/s, steps/s^2)
#define X_MIN_POS_STEPS 0
#define X_MAX_POS_STEPS (17000)
#define X_MAX_VEL_STEPS (10000)
#define X_MAX_ACC_STEPS (200000)

#define Y_MIN_POS_STEPS 0
#define Y_MAX_POS_STEPS (15000)
#define Y_MAX_VEL_STEPS (10000)
#define Y_MAX_ACC_STEPS (200000)

#define Z_MIN_POS_STEPS 0
#define Z_MAX_POS_STEPS (72000)
#define Z_MAX_VEL_STEPS (10000)
#define Z_MAX_ACC_STEPS (200000)

// Axis-specific pin and parameter defines
// X axis
#define X_CS_PIN      17
#define X_EN_PIN      22
#define X_DIR_PIN     21
#define X_STEP_PIN    20
#define X_DIAG0_PIN    5 //2
#define X_MICROSTEPS  16
#define X_CURRENT_MA 500
#define X_STALL_VALUE  3

// Y axis
#define Y_CS_PIN       6
#define Y_EN_PIN       7
#define Y_DIR_PIN      8
#define Y_STEP_PIN     9
#define Y_DIAG0_PIN    3
#define Y_MICROSTEPS   16
#define Y_CURRENT_MA 500
#define Y_STALL_VALUE  3

// Z axis
#define Z_CS_PIN      10
#define Z_EN_PIN      11
#define Z_DIR_PIN     12
#define Z_STEP_PIN    13
#define Z_DIAG0_PIN    4
#define Z_MICROSTEPS   16
#define Z_CURRENT_MA 500
#define Z_STALL_VALUE 13

// Axis configuration struct
typedef struct {
    char     name;          // Axis identifier
    uint8_t  cs;            // Chip select pin
    uint8_t  en;            // Enable pin
    uint8_t  dir;           // Direction pin
    uint8_t  step;          // Step pin
    int8_t   diag0;         // StallGuard / homing pin (-1 if unused)
    uint32_t microsteps;    // Microstep setting
    uint16_t current_mA;    // RMS current setting
    int16_t stall_value;    // StallGuard threshold
    int32_t  min_pos;       // Minimum position (steps)
    int32_t  max_pos;       // Maximum position (steps)
    uint32_t max_vel;       // Maximum velocity (steps/s)
    uint32_t max_acc;       // Maximum acceleration (steps/s^2)
} AxisConfig;

// Runtime data for each axis
typedef struct {
    TMC2130Stepper *driver;
    AccelStepper   *stepper;
} AxisData;

// Global driver instances (accessible from core 2)
TMC2130Stepper driverX(X_CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
TMC2130Stepper driverY(Y_CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
TMC2130Stepper driverZ(Z_CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

// Global stepper instances (accessible from core 2)
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// Axis configurations using defined constants
AxisConfig axisConfigs[] = {
    { 'X', X_CS_PIN, X_EN_PIN, X_DIR_PIN, X_STEP_PIN, X_DIAG0_PIN, X_MICROSTEPS, X_CURRENT_MA, X_STALL_VALUE, X_MIN_POS_STEPS, X_MAX_POS_STEPS, X_MAX_VEL_STEPS, X_MAX_ACC_STEPS },
    { 'Y', Y_CS_PIN, Y_EN_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_DIAG0_PIN, Y_MICROSTEPS, Y_CURRENT_MA, Y_STALL_VALUE, Y_MIN_POS_STEPS, Y_MAX_POS_STEPS, Y_MAX_VEL_STEPS, Y_MAX_ACC_STEPS },
    { 'Z', Z_CS_PIN, Z_EN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_DIAG0_PIN, Z_MICROSTEPS, Z_CURRENT_MA, Z_STALL_VALUE, Z_MIN_POS_STEPS, Z_MAX_POS_STEPS, Z_MAX_VEL_STEPS, Z_MAX_ACC_STEPS }
};
AxisData axes[3];

unsigned long lastFeedback = 0;

// Initialize hardware for one axis
void axis_init(int idx) {
    AxisConfig *cfg = &axisConfigs[idx];
    pinMode(cfg->cs, OUTPUT);
    digitalWrite(cfg->cs, HIGH);
    pinMode(cfg->en, OUTPUT);
    digitalWrite(cfg->en, LOW);
    if (cfg->diag0 >= 0) pinMode(cfg->diag0, INPUT);

    // Assign global driver and configure
    TMC2130Stepper *drv = (idx == 0 ? &driverX : idx == 1 ? &driverY : &driverZ);
    axes[idx].driver = drv;
    drv->begin();
    drv->rms_current(cfg->current_mA);
    drv->microsteps(cfg->microsteps);
    drv->en_pwm_mode(true);
    drv->pwm_autoscale(true);
    if (cfg->diag0 >= 0) {
        drv->sgt(cfg->stall_value);
        drv->diag0_stall(true);
        drv->diag0_int_pushpull(true);
    }

    // Assign global stepper and configure
    AccelStepper *stp = (idx == 0 ? &stepperX : idx == 1 ? &stepperY : &stepperZ);
    axes[idx].stepper = stp;
    stp->setEnablePin(cfg->en);
    stp->setPinsInverted(false, false, true);
    stp->enableOutputs();
    stp->setMaxSpeed(cfg->max_vel);
    stp->setAcceleration(cfg->max_acc);
}

// Perform homing for an axis (only X by default)
void axis_home(int idx, bool direction = 0, int home_position = 0) {
    AxisConfig *cfg = &axisConfigs[idx];
    if (cfg->diag0 < 0) return;
    TMC2130Stepper *drv = axes[idx].driver;
    AccelStepper   *stp = axes[idx].stepper;

    drv->en_pwm_mode(false);
    drv->pwm_autoscale(false);
    drv->TCOOLTHRS(0xFFFFF); // 20bit max
    drv->THIGH(0);
    delay(100);
    digitalWrite(cfg->dir, direction);
    tone(cfg->step, 2000);
    delay(100);
    while (!gpio_get(cfg->diag0)); //Time critical, todo disable interupt? move this check to a PIO program?
    noTone(cfg->step);
    drv->TCOOLTHRS(0);
    drv->en_pwm_mode(true);
    drv->pwm_autoscale(true);
    stp->setCurrentPosition(home_position);
}

// Set motion parameters and target
void axis_setMotion(int idx, int32_t pos, uint32_t vel, uint32_t acc) {
    AxisConfig *cfg = &axisConfigs[idx];
    AccelStepper *stp = axes[idx].stepper;

    // Clamp position, velocity, and acceleration to allowed range
    if (pos < cfg->min_pos) pos = cfg->min_pos;
    if (pos > cfg->max_pos) pos = cfg->max_pos;
    if (vel > cfg->max_vel) vel = cfg->max_vel;
    if (acc > cfg->max_acc) acc = cfg->max_acc;

    stp->setMaxSpeed(vel);
    stp->setAcceleration(acc);
    stp->moveTo(pos);
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    SPI.begin();

    axis_init(0);
    axis_home(0);

    axis_init(1);
    axis_home(1);

    axis_init(2);
    axis_home(2,1,72000);
}

void loop() {
    // Command handling
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        char axis; uint32_t pos, vel, acc;
        if (sscanf(cmd.c_str(), "%c %lu %lu %lu", &axis, &pos, &vel, &acc) == 4) {
            for (int i = 0; i < 3; ++i) {
                if (axisConfigs[i].name == axis) {
                    axis_setMotion(i, pos, vel, acc);
                    Serial.println(i);
                    break;
                }
            }
        }
    }

    // Update steppers directly
    axes[0].stepper->run();
    axes[1].stepper->run();
    axes[2].stepper->run();

    // Periodic feedback inline
    if (millis() - lastFeedback >= FEEDBACK_PERIOD_MS) {
        lastFeedback = millis();
        Serial.printf(
            "X %ld %f Y %ld %f Z %ld %f\n",
            axes[0].stepper->currentPosition(), axes[0].stepper->speed(),
            axes[1].stepper->currentPosition(), axes[1].stepper->speed(),
            axes[2].stepper->currentPosition(), axes[2].stepper->speed()
        );
    }
}
