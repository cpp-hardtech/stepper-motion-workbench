#include <cinttypes>

#include "StepperMotion.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "stepper_motion";
constexpr uint8_t STEP_PIN = 17;
constexpr uint8_t DIR_PIN = 18;
constexpr uint8_t ENABLE_PIN = 26;
constexpr int32_t TARGET_STEPS = 200;
StepperMotionEngine engine;

}  // namespace

extern "C" void app_main() {
  engine.init();
  StepperMotion* axis = engine.stepperConnectToPin(STEP_PIN);
  if (axis == nullptr) {
    ESP_LOGE(TAG, "No pulse output available on GPIO %u", STEP_PIN);
    return;
  }

  axis->setDirectionPin(DIR_PIN);
  axis->setEnablePin(ENABLE_PIN, true);
  axis->setAutoEnable(true);
  if (axis->setSpeedInHz(400) != 0 || axis->setAcceleration(800) != 0) {
    ESP_LOGE(TAG, "Motion settings rejected");
    return;
  }

  ESP_LOGI(TAG, "Moving to %" PRId32 " steps", TARGET_STEPS);
  const MoveResultCode result = axis->moveTo(TARGET_STEPS);
  if (result != MoveResultCode::OK) {
    ESP_LOGE(TAG, "Move rejected (%d)", static_cast<int>(result));
    return;
  }
  while (axis->isRunning()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  ESP_LOGI(TAG, "Motion ended at %" PRId32 " commanded steps",
           axis->getCurrentPosition());
}
