//Prateek
//https://justdoelectronics.com

#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include "constants.h"

const int8_t pot_map_radius[3] = { 5, 5, 5 };
const uint16_t nom_vals[3][3] = { { 500, 300, 25 }, { 5, 5, 5 }, { 12, 25, 50 } };
const uint8_t val_inc[3][3] = { { 10, 5, 1 }, { 1, 1, 1 }, { 1, 1, 2 } };


const uint16_t base_delays[3] = { 950, 550, 200 };

const char clear_value[] = "        ";
const char *setting_titles[3] = { "Volume: ", "Pressure: ", "Breath Rate: " };
const char *age_titles[3] = { "Adult", "Child", "Infant" };
const char *unit[3] = { " mL", "/10", " br/m" };

bool ventilating = false;
bool time_to_squeeze = true;
bool time_to_release = true;
long squeeze_time = -100;
long release_time = -100;
bool compression_state = 0;
bool halt = false;
bool warn_user = false;

bool can_read_age_button = true;
long age_button_time = -100;
bool can_read_start_button = true;
long start_button_time = -100;

int8_t pot_vals[3] = { 0, 0, 0 };
uint8_t age_state = 0;

long red_led_flash = 0;
bool red_led_flash_on = false;

double sensitivity[] = {
  0.185,
  0.100,
  0.066
};
double voltage;
double current_sum = 0;

Servo servo;
LiquidCrystal_I2C lcd(0x27, 20, 4);

int map_pot_val(uint16_t unmapped_pot_val, int8_t pot_index) {
  return map(unmapped_pot_val, 0, 1024, pot_map_radius[pot_index], -pot_map_radius[pot_index]);
}

int16_t map_servo_pos(int16_t pos) {
  return map(pos, 0, MAX_SERVO_POS, 180, 0);
}

void lcd_update(uint8_t pot_index) {
  lcd.setCursor(LCD_H_SPACING - 1, pot_index + POT_LCD_OFFSET);
  lcd.print(clear_value);
  lcd.setCursor(LCD_H_SPACING, pot_index + POT_LCD_OFFSET);
  int16_t val = nom_vals[pot_index][age_state] + val_inc[pot_index][age_state] * pot_vals[pot_index];
  lcd.print(val);
  lcd.print(unit[pot_index]);
}

void lcd_update_age() {
  lcd.setCursor(LCD_H_SPACING - 1, 0);
  lcd.print(clear_value);
  lcd.setCursor(LCD_H_SPACING, 0);
  lcd.print(age_titles[age_state]);
  for (uint8_t i = 0; i < 3; ++i) {
    lcd_update(i);
  }
}

void lcd_startup() {
  lcd.clear();
  lcd.setCursor(8, 0);
  lcd.print("YOUR");
  lcd.setCursor(0, 2);
  lcd.print("Automatic Inhalation");
  lcd.setCursor(4, 3);
  lcd.print("VENTILATOR");
}

void lcd_init_titles() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Age Group:");
  lcd.setCursor(0, 1);
  lcd.print("Volume:");
  lcd.setCursor(0, 2);
  lcd.print("Pressure:");
  lcd.setCursor(0, 3);
  lcd.print("Breath Rate:");
}

void lcd_init() {
  lcd_init_titles();
  lcd_update_age();
}



void check_pot_vals() {
  for (uint8_t i = 0; i < 3; ++i) {
    if (map_pot_val(analogRead(i + POT_PIN_OFFSET), i) != pot_vals[i]) {
      pot_vals[i] = map_pot_val(analogRead(i + POT_PIN_OFFSET), i);
      lcd_update(i);
    }
  }
}

void check_age_setting() {
  if (!digitalRead(P_SETTING_BUTTON) && can_read_age_button) {
    age_button_time = millis();
    can_read_age_button = false;
    age_state = (age_state + 1) % 3;
    lcd_update_age();
  }
}

void check_start_stop() {
  if (!digitalRead(P_START_BUTTON) && can_read_start_button) {
    if (warn_user) {
      red_led(true);
      warn_user = false;
      lcd_init();
    } else {
      start_button_time = millis();
      can_read_start_button = false;
      ventilating = !ventilating;
      red_led(!ventilating);
      green_led(ventilating);
    }
  }
}

bool check_halt() {
  if (ventilating && warn_user) {
    return true;
  }
  bool was_ventilating = ventilating;
  check_start_stop();
  if (was_ventilating && !ventilating) {
    return true;
  }
  return false;
}

uint8_t get_servo_delay() {
  uint16_t des_delay = nom_vals[PRES][age_state];
  des_delay += val_inc[PRES][age_state] * pot_vals[PRES];
  des_delay = map(des_delay, nom_vals[PRES][age_state] - pot_map_radius[PRES] * val_inc[PRES][age_state], nom_vals[PRES][age_state] + pot_map_radius[PRES] * val_inc[PRES][age_state], MIN_SPEED, MAX_SPEED);
  return des_delay;
}

void drive_servo(uint16_t desired_pos) {
  uint16_t servo_pos = servo.read();
  if (servo_pos < desired_pos) {
    for (int16_t pos = servo_pos; pos < desired_pos; pos += 1) {
      halt = check_halt();
      if (halt) {
        break;
      }
      servo.write(pos);
      delay(15);
    }
  }

  else {
    for (int16_t pos = servo_pos; pos > desired_pos; pos -= 1) {
      halt = check_halt();
      if (halt) {
        break;
      }
      servo.write(pos);
      update_current();
      uint8_t del = get_servo_delay();
      delay(del);
    }
  }
}

uint16_t get_ms_per_breath() {
  uint8_t breath_per_min = nom_vals[RATE][age_state] + val_inc[RATE][age_state] * pot_vals[RATE];
  return round(1000 / (breath_per_min / 60.0));
}

uint16_t vol_to_ang(uint16_t vol) {
  return 4.89427e-7 * pow(vol, 3) - 8.40105e-4 * pow(vol, 2) + 0.64294 * vol + 28.072327;
}

void red_led(bool on) {

  analogWrite(P_RED_LED, on ? 5 : 0);
}

void green_led(bool on) {
  digitalWrite(P_GREEN_LED, on ? HIGH : LOW);
}

void flash_red() {
  if ((millis() - red_led_flash) > 200) {
    red_led(!red_led_flash_on);
    red_led_flash_on = !red_led_flash_on;
    red_led_flash = millis();
  }
}

double predict_current() {

  uint16_t v = nom_vals[0][age_state] + val_inc[0][age_state] * pot_vals[0];
  uint8_t p = nom_vals[1][age_state] + val_inc[1][age_state] * pot_vals[1];
  return -1.041041e3 + 2.35386 * v + 2.316309e-3 * pow(v, 2) - 3.887507e1 * p + 7.7198644 * pow(p, 2) - 4.2099326e-2 * v * p;
}

void update_current() {

  uint16_t potentiometer_raw = analogRead(P_CURRENT);
  double voltage_raw = (5.0 / 1023.0) * analogRead(VIN);
  voltage = voltage_raw - QOV + 0.012;
  double current = voltage / sensitivity[MODEL] * -1;
  current_sum += current;
}

void setup() {

  Serial.begin(9600);
  pinMode(P_RED_LED, OUTPUT);
  pinMode(P_GREEN_LED, OUTPUT);
  pinMode(P_SETTING_BUTTON, INPUT_PULLUP);
  pinMode(P_START_BUTTON, INPUT_PULLUP);
  servo.attach(P_SERVO);
  servo.write(map_servo_pos(START_POS));
  lcd.init();
  lcd.backlight();
  lcd_startup();
  red_led(true);
  green_led(true);
  delay(3000);
  lcd_init();
  green_led(false);
}

void loop() {
  if (!can_read_age_button && millis() - age_button_time >= 500) {
    can_read_age_button = true;
  }
  if (!can_read_start_button && millis() - start_button_time >= 500) {
    can_read_start_button = true;
  }

  if (!warn_user) {
    if (!ventilating) {
      check_age_setting();
    }
    check_pot_vals();
  }

  if (halt) {
    compression_state = 0;
    ventilating = false;
    drive_servo(map_servo_pos(START_POS));
    release_time = millis();
  }
  halt = check_halt();

  if (warn_user) {
    flash_red();
  }

  if (ventilating) {
    uint16_t ms_per_breath = get_ms_per_breath();
    int16_t post_breath_delay = ms_per_breath - base_delays[age_state];
    if (post_breath_delay < 0) {
      post_breath_delay = 1000;
    }

    if (time_to_squeeze && !compression_state) {
      compression_state = 1;
      uint16_t des_vol = nom_vals[VOL][age_state];
      des_vol += val_inc[VOL][age_state] * pot_vals[VOL];
      uint16_t dest = vol_to_ang(des_vol);
      if (age_state == 0) {

        dest = map(dest, 193, 208, 180, 250);
      }
      drive_servo(map_servo_pos(dest));
      squeeze_time = millis();
    } else if (time_to_release && compression_state) {

      if (current_sum >= predict_current() * 1.2 && age_state == 0) {
        halt = true;
        green_led(false);
        warn_user = true;

      } else {
        compression_state = 0;
        drive_servo(map_servo_pos(START_POS));
        release_time = millis();
      }
      current_sum = 0;
    }

    if (compression_state) {
      update_current();
    }

    time_to_squeeze = millis() - release_time > post_breath_delay;
    time_to_release = millis() - squeeze_time > base_delays[age_state];
  }
}