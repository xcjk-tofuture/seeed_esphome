#include "esphome/core/component.h"

#include <cinttypes>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {

static const char *const TAG = "component";

// Global vectors for component data that doesn't belong in every instance.
// Using vector instead of unordered_map for both because:
// - Much lower memory overhead (8 bytes per entry vs 20+ for unordered_map)
// - Linear search is fine for small n (typically < 5 entries)
// - These are rarely accessed (setup only or error cases only)

// Component error messages - only stores messages for failed components
// Lazy allocated since most configs have zero failures
// Note: We don't clear this vector because:
// 1. Components are never destroyed in ESPHome
// 2. Failed components remain failed (no recovery mechanism)
// 3. Memory usage is minimal (only failures with custom messages are stored)

// Using namespace-scope static to avoid guard variables (saves 16 bytes total)
// This is safe because ESPHome is single-threaded during initialization
namespace {
// Error messages for failed components
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::unique_ptr<std::vector<std::pair<const Component *, const char *>>> component_error_messages;
// Setup priority overrides - freed after setup completes
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::unique_ptr<std::vector<std::pair<const Component *, float>>> setup_priority_overrides;
}  // namespace

namespace setup_priority {

const float BUS = 1000.0f;
const float IO = 900.0f;
const float HARDWARE = 800.0f;
const float DATA = 600.0f;
const float PROCESSOR = 400.0;
const float BLUETOOTH = 350.0f;
const float AFTER_BLUETOOTH = 300.0f;
const float WIFI = 250.0f;
const float ETHERNET = 250.0f;
const float BEFORE_CONNECTION = 220.0f;
const float AFTER_WIFI = 200.0f;
const float AFTER_CONNECTION = 100.0f;
const float LATE = -100.0f;

}  // namespace setup_priority

// Component state uses bits 0-2 (8 states, 5 used)
const uint8_t COMPONENT_STATE_MASK = 0x07;
const uint8_t COMPONENT_STATE_CONSTRUCTION = 0x00;
const uint8_t COMPONENT_STATE_SETUP = 0x01;
const uint8_t COMPONENT_STATE_LOOP = 0x02;
const uint8_t COMPONENT_STATE_FAILED = 0x03;
const uint8_t COMPONENT_STATE_LOOP_DONE = 0x04;
// Status LED uses bits 3-4
const uint8_t STATUS_LED_MASK = 0x18;
const uint8_t STATUS_LED_OK = 0x00;
const uint8_t STATUS_LED_WARNING = 0x08;  // Bit 3
const uint8_t STATUS_LED_ERROR = 0x10;    // Bit 4

const uint16_t WARN_IF_BLOCKING_OVER_MS = 50U;       ///< Initial blocking time allowed without warning
const uint16_t WARN_IF_BLOCKING_INCREMENT_MS = 10U;  ///< How long the blocking time must be larger to warn again

uint32_t global_state = 0;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

float Component::get_loop_priority() const { return 0.0f; }

float Component::get_setup_priority() const { return setup_priority::DATA; }

void Component::setup() {}

void Component::loop() {}

void Component::set_interval(const std::string &name, uint32_t interval, std::function<void()> &&f) {  // NOLINT
  App.scheduler.set_interval(this, name, interval, std::move(f));
}

void Component::set_interval(const char *name, uint32_t interval, std::function<void()> &&f) {  // NOLINT
  App.scheduler.set_interval(this, name, interval, std::move(f));
}

bool Component::cancel_interval(const std::string &name) {  // NOLINT
  return App.scheduler.cancel_interval(this, name);
}

bool Component::cancel_interval(const char *name) {  // NOLINT
  return App.scheduler.cancel_interval(this, name);
}

void Component::set_retry(const std::string &name, uint32_t initial_wait_time, uint8_t max_attempts,
                          std::function<RetryResult(uint8_t)> &&f, float backoff_increase_factor) {  // NOLINT
  App.scheduler.set_retry(this, name, initial_wait_time, max_attempts, std::move(f), backoff_increase_factor);
}

bool Component::cancel_retry(const std::string &name) {  // NOLINT
  return App.scheduler.cancel_retry(this, name);
}

void Component::set_timeout(const std::string &name, uint32_t timeout, std::function<void()> &&f) {  // NOLINT
  App.scheduler.set_timeout(this, name, timeout, std::move(f));
}

void Component::set_timeout(const char *name, uint32_t timeout, std::function<void()> &&f) {  // NOLINT
  App.scheduler.set_timeout(this, name, timeout, std::move(f));
}

bool Component::cancel_timeout(const std::string &name) {  // NOLINT
  return App.scheduler.cancel_timeout(this, name);
}

bool Component::cancel_timeout(const char *name) {  // NOLINT
  return App.scheduler.cancel_timeout(this, name);
}

void Component::call_loop() { this->loop(); }
void Component::call_setup() { this->setup(); }
void Component::call_dump_config() {
  this->dump_config();
  if (this->is_failed()) {
    // Look up error message from global vector
    const char *error_msg = "unspecified";
    if (component_error_messages) {
      for (const auto &pair : *component_error_messages) {
        if (pair.first == this) {
          error_msg = pair.second;
          break;
        }
      }
    }
    ESP_LOGE(TAG, "  %s is marked FAILED: %s", this->get_component_source(), error_msg);
  }
}

uint8_t Component::get_component_state() const { return this->component_state_; }
void Component::call() {
  uint8_t state = this->component_state_ & COMPONENT_STATE_MASK;
  switch (state) {
    case COMPONENT_STATE_CONSTRUCTION:
      // State Construction: Call setup and set state to setup
      this->component_state_ &= ~COMPONENT_STATE_MASK;
      this->component_state_ |= COMPONENT_STATE_SETUP;
      this->call_setup();
      break;
    case COMPONENT_STATE_SETUP:
      // State setup: Call first loop and set state to loop
      this->component_state_ &= ~COMPONENT_STATE_MASK;
      this->component_state_ |= COMPONENT_STATE_LOOP;
      this->call_loop();
      break;
    case COMPONENT_STATE_LOOP:
      // State loop: Call loop
      this->call_loop();
      break;
    case COMPONENT_STATE_FAILED:  // NOLINT(bugprone-branch-clone)
      // State failed: Do nothing
      break;
    case COMPONENT_STATE_LOOP_DONE:  // NOLINT(bugprone-branch-clone)
      // State loop done: Do nothing, component has finished its work
      break;
    default:
      break;
  }
}
const char *Component::get_component_source() const {
  if (this->component_source_ == nullptr)
    return "<unknown>";
  return this->component_source_;
}
bool Component::should_warn_of_blocking(uint32_t blocking_time) {
  if (blocking_time > this->warn_if_blocking_over_) {
    // Prevent overflow when adding increment - if we're about to overflow, just max out
    if (blocking_time + WARN_IF_BLOCKING_INCREMENT_MS < blocking_time ||
        blocking_time + WARN_IF_BLOCKING_INCREMENT_MS > std::numeric_limits<uint16_t>::max()) {
      this->warn_if_blocking_over_ = std::numeric_limits<uint16_t>::max();
    } else {
      this->warn_if_blocking_over_ = static_cast<uint16_t>(blocking_time + WARN_IF_BLOCKING_INCREMENT_MS);
    }
    return true;
  }
  return false;
}
void Component::mark_failed() {
  ESP_LOGE(TAG, "%s was marked as failed", this->get_component_source());
  this->component_state_ &= ~COMPONENT_STATE_MASK;
  this->component_state_ |= COMPONENT_STATE_FAILED;
  this->status_set_error();
  // Also remove from loop since failed components shouldn't loop
  App.disable_component_loop_(this);
}
void Component::disable_loop() {
  if ((this->component_state_ & COMPONENT_STATE_MASK) != COMPONENT_STATE_LOOP_DONE) {
    ESP_LOGVV(TAG, "%s loop disabled", this->get_component_source());
    this->component_state_ &= ~COMPONENT_STATE_MASK;
    this->component_state_ |= COMPONENT_STATE_LOOP_DONE;
    App.disable_component_loop_(this);
  }
}
void Component::enable_loop() {
  if ((this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_LOOP_DONE) {
    ESP_LOGVV(TAG, "%s loop enabled", this->get_component_source());
    this->component_state_ &= ~COMPONENT_STATE_MASK;
    this->component_state_ |= COMPONENT_STATE_LOOP;
    App.enable_component_loop_(this);
  }
}
void IRAM_ATTR HOT Component::enable_loop_soon_any_context() {
  // This method is thread and ISR-safe because:
  // 1. Only performs simple assignments to volatile variables (atomic on all platforms)
  // 2. No read-modify-write operations that could be interrupted
  // 3. No memory allocation, object construction, or function calls
  // 4. IRAM_ATTR ensures code is in IRAM, not flash (required for ISR execution)
  // 5. Components are never destroyed, so no use-after-free concerns
  // 6. App is guaranteed to be initialized before any ISR could fire
  // 7. Multiple ISR/thread calls are safe - just sets the same flags to true
  // 8. Race condition with main loop is handled by clearing flag before processing
  this->pending_enable_loop_ = true;
  App.has_pending_enable_loop_requests_ = true;
}
void Component::reset_to_construction_state() {
  if ((this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_FAILED) {
    ESP_LOGI(TAG, "%s is being reset to construction state", this->get_component_source());
    this->component_state_ &= ~COMPONENT_STATE_MASK;
    this->component_state_ |= COMPONENT_STATE_CONSTRUCTION;
    // Clear error status when resetting
    this->status_clear_error();
  }
}
bool Component::is_in_loop_state() const {
  return (this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_LOOP;
}
void Component::defer(std::function<void()> &&f) {  // NOLINT
  App.scheduler.set_timeout(this, static_cast<const char *>(nullptr), 0, std::move(f));
}
bool Component::cancel_defer(const std::string &name) {  // NOLINT
  return App.scheduler.cancel_timeout(this, name);
}
void Component::defer(const std::string &name, std::function<void()> &&f) {  // NOLINT
  App.scheduler.set_timeout(this, name, 0, std::move(f));
}
void Component::defer(const char *name, std::function<void()> &&f) {  // NOLINT
  App.scheduler.set_timeout(this, name, 0, std::move(f));
}
void Component::set_timeout(uint32_t timeout, std::function<void()> &&f) {  // NOLINT
  App.scheduler.set_timeout(this, static_cast<const char *>(nullptr), timeout, std::move(f));
}
void Component::set_interval(uint32_t interval, std::function<void()> &&f) {  // NOLINT
  App.scheduler.set_interval(this, static_cast<const char *>(nullptr), interval, std::move(f));
}
void Component::set_retry(uint32_t initial_wait_time, uint8_t max_attempts, std::function<RetryResult(uint8_t)> &&f,
                          float backoff_increase_factor) {  // NOLINT
  App.scheduler.set_retry(this, "", initial_wait_time, max_attempts, std::move(f), backoff_increase_factor);
}
bool Component::is_failed() const { return (this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_FAILED; }
bool Component::is_ready() const {
  return (this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_LOOP ||
         (this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_LOOP_DONE ||
         (this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_SETUP;
}
bool Component::can_proceed() { return true; }
bool Component::status_has_warning() const { return this->component_state_ & STATUS_LED_WARNING; }
bool Component::status_has_error() const { return this->component_state_ & STATUS_LED_ERROR; }
void Component::status_set_warning(const char *message) {
  // Don't spam the log. This risks missing different warning messages though.
  if ((this->component_state_ & STATUS_LED_WARNING) != 0)
    return;
  this->component_state_ |= STATUS_LED_WARNING;
  App.app_state_ |= STATUS_LED_WARNING;
  ESP_LOGW(TAG, "%s set Warning flag: %s", this->get_component_source(), message);
}
void Component::status_set_error(const char *message) {
  if ((this->component_state_ & STATUS_LED_ERROR) != 0)
    return;
  this->component_state_ |= STATUS_LED_ERROR;
  App.app_state_ |= STATUS_LED_ERROR;
  ESP_LOGE(TAG, "%s set Error flag: %s", this->get_component_source(), message);
  if (strcmp(message, "unspecified") != 0) {
    // Lazy allocate the error messages vector if needed
    if (!component_error_messages) {
      component_error_messages = std::make_unique<std::vector<std::pair<const Component *, const char *>>>();
    }
    // Check if this component already has an error message
    for (auto &pair : *component_error_messages) {
      if (pair.first == this) {
        pair.second = message;
        return;
      }
    }
    // Add new error message
    component_error_messages->emplace_back(this, message);
  }
}
void Component::status_clear_warning() {
  if ((this->component_state_ & STATUS_LED_WARNING) == 0)
    return;
  this->component_state_ &= ~STATUS_LED_WARNING;
  ESP_LOGW(TAG, "%s cleared Warning flag", this->get_component_source());
}
void Component::status_clear_error() {
  if ((this->component_state_ & STATUS_LED_ERROR) == 0)
    return;
  this->component_state_ &= ~STATUS_LED_ERROR;
  ESP_LOGE(TAG, "%s cleared Error flag", this->get_component_source());
}
void Component::status_momentary_warning(const std::string &name, uint32_t length) {
  this->status_set_warning();
  this->set_timeout(name, length, [this]() { this->status_clear_warning(); });
}
void Component::status_momentary_error(const std::string &name, uint32_t length) {
  this->status_set_error();
  this->set_timeout(name, length, [this]() { this->status_clear_error(); });
}
void Component::dump_config() {}
float Component::get_actual_setup_priority() const {
  // Check if there's an override in the global vector
  if (setup_priority_overrides) {
    // Linear search is fine for small n (typically < 5 overrides)
    for (const auto &pair : *setup_priority_overrides) {
      if (pair.first == this) {
        return pair.second;
      }
    }
  }
  return this->get_setup_priority();
}
void Component::set_setup_priority(float priority) {
  // Lazy allocate the vector if needed
  if (!setup_priority_overrides) {
    setup_priority_overrides = std::make_unique<std::vector<std::pair<const Component *, float>>>();
    // Reserve some space to avoid reallocations (most configs have < 10 overrides)
    setup_priority_overrides->reserve(10);
  }

  // Check if this component already has an override
  for (auto &pair : *setup_priority_overrides) {
    if (pair.first == this) {
      pair.second = priority;
      return;
    }
  }

  // Add new override
  setup_priority_overrides->emplace_back(this, priority);
}

bool Component::has_overridden_loop() const {
#if defined(USE_HOST) || defined(CLANG_TIDY)
  bool loop_overridden = true;
  bool call_loop_overridden = true;
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpmf-conversions"
  bool loop_overridden = (void *) (this->*(&Component::loop)) != (void *) (&Component::loop);
  bool call_loop_overridden = (void *) (this->*(&Component::call_loop)) != (void *) (&Component::call_loop);
#pragma GCC diagnostic pop
#endif
  return loop_overridden || call_loop_overridden;
}

PollingComponent::PollingComponent(uint32_t update_interval) : update_interval_(update_interval) {}

void PollingComponent::call_setup() {
  // Let the polling component subclass setup their HW.
  this->setup();

  // init the poller
  this->start_poller();
}

void PollingComponent::start_poller() {
  // Register interval.
  this->set_interval("update", this->get_update_interval(), [this]() { this->update(); });
}

void PollingComponent::stop_poller() {
  // Clear the interval to suspend component
  this->cancel_interval("update");
}

uint32_t PollingComponent::get_update_interval() const { return this->update_interval_; }
void PollingComponent::set_update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }

WarnIfComponentBlockingGuard::WarnIfComponentBlockingGuard(Component *component, uint32_t start_time)
    : started_(start_time), component_(component) {}
uint32_t WarnIfComponentBlockingGuard::finish() {
  uint32_t curr_time = millis();

  uint32_t blocking_time = curr_time - this->started_;
  bool should_warn;
  if (this->component_ != nullptr) {
    should_warn = this->component_->should_warn_of_blocking(blocking_time);
  } else {
    should_warn = blocking_time > WARN_IF_BLOCKING_OVER_MS;
  }
  if (should_warn) {
    const char *src = component_ == nullptr ? "<null>" : component_->get_component_source();
    ESP_LOGW(TAG, "%s took a long time for an operation (%" PRIu32 " ms)", src, blocking_time);
    ESP_LOGW(TAG, "Components should block for at most 30 ms");
  }

  return curr_time;
}

WarnIfComponentBlockingGuard::~WarnIfComponentBlockingGuard() {}

void clear_setup_priority_overrides() {
  // Free the setup priority map completely
  setup_priority_overrides.reset();
}

}  // namespace esphome
