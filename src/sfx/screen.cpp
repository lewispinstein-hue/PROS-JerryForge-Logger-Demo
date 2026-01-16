#include "sfx/screen.hpp" // parent header
#include "pros/screen.hpp"
#include <cstdint>
#include <vector>

namespace sfx {

namespace screen {

void Manager::clearScreen() {
  MutexGuard m(sharedMutex);
  textLines.clear();
  pros::screen::erase_rect(0, 0, 480, 272);
  pros::screen::erase();
}

void Manager::drawCapsule(int x, int y, int width, int height,
                          uint32_t fillColor, uint32_t borderColor) {
  MutexGuard m(sharedMutex); // Lock

  int radius = height / 2;

  // Base Fill
  pros::screen::set_pen(fillColor);
  pros::screen::set_eraser(fillColor); // Set the fill color

  pros::screen::fill_circle(x + radius, y + radius, radius);
  pros::screen::fill_circle(x + width - radius, y + radius, radius);
  pros::screen::fill_rect(x + radius, y, x + width - radius, y + height);

  // --- STEP 2: The Outlines ---
  pros::screen::set_pen(borderColor);

  // Draw the full circle outlines at both ends
  pros::screen::draw_circle(x + radius, y + radius, radius);
  pros::screen::draw_circle(x + width - radius, y + radius, radius);

  // Draw the top and bottom connecting lines
  pros::screen::draw_line(x + radius, y, x + width - radius, y);
  pros::screen::draw_line(x + radius, y + height, x + width - radius,
                          y + height);

  // This is the "Soft Rectangle" part. We use the FILL color to overwrite
  // the inner halves of the circles we just drew in Step 2.
  pros::screen::set_pen(fillColor);
  pros::screen::set_eraser(fillColor);

  // We offset by +1 and -1 so we don't erase the top/bottom border lines.
  // This removes the vertical 'seams' inside the button.
  pros::screen::fill_rect(x + radius + 1, y + 1, x + width - radius - 1,
                          y + height - 1);
}

void Manager::drawBottomButtons(bool redraw) {
  MutexGuard m(sharedMutex);
  if (redraw)
    clearScreen();

  auto color = pros::screen::get_pen();

  drawCapsule(getX(ButtonId::LEFT), BTN_Y, BTN_WIDTH, BTN_HEIGHT,
              COLOR_BTN_FILL, COLOR_BTN_BORDER);
  drawCapsule(getX(ButtonId::MIDDLE), BTN_Y, BTN_WIDTH, BTN_HEIGHT,
              COLOR_BTN_FILL, COLOR_BTN_BORDER);
  drawCapsule(getX(ButtonId::RIGHT), BTN_Y, BTN_WIDTH, BTN_HEIGHT,
              COLOR_BTN_FILL, COLOR_BTN_BORDER);

  pros::screen::set_pen(color);
}

/**
 * @brief returns if the given coords are inside of the given box. Specific for
 * Manager class
 */

bool Manager::isInside(int px, int py, int bx, int by, int w, int h) {
  return (px >= (bx - PADDING) && px <= (bx + w + PADDING) &&
          py >= (by - PADDING) && py <= (by + h + PADDING));
}

ButtonId Manager::waitForBottomButtonTap(uint32_t timeout) {
  int last_release_count = pros::screen::touch_status().release_count;
  uint32_t startTime = pros::millis();
  int final_x = -1, final_y = -1;

  while ((uint32_t)pros::millis() - startTime < timeout) {
    pros::screen_touch_status_s touch = pros::screen::touch_status();

    if (touch.touch_status == pros::E_TOUCH_PRESSED ||
        touch.touch_status == pros::E_TOUCH_HELD) {
      final_x = touch.x;
      final_y = touch.y;
    }

    if (touch.release_count > last_release_count) {
      last_release_count = touch.release_count;
      if (final_x != -1 && final_y != -1) {
        if (isInside(final_x, final_y, getX(ButtonId::LEFT), BTN_Y, BTN_WIDTH,
                     BTN_HEIGHT))
          return ButtonId::LEFT;
        if (isInside(final_x, final_y, getX(ButtonId::MIDDLE), BTN_Y, BTN_WIDTH,
                     BTN_HEIGHT))
          return ButtonId::MIDDLE;
        if (isInside(final_x, final_y, getX(ButtonId::RIGHT), BTN_Y, BTN_WIDTH,
                     BTN_HEIGHT))
          return ButtonId::RIGHT;
      }
      final_x = -1;
      final_y = -1;
    }
    pros::delay(20);
  }
  return ButtonId::NONE;
}

bool Manager::waitForScreenTouch(uint32_t timeoutMs, bool detectTouchOnly) {
  uint32_t start = pros::millis();
  int32_t initialRelease = pros::screen::touch_status().release_count;

  while (((uint32_t)pros::millis() - start) < timeoutMs) {
    auto status = pros::screen::touch_status();
    if (detectTouchOnly && status.touch_status == pros::E_TOUCH_HELD)
      return true;
    if (!detectTouchOnly && status.release_count > initialRelease)
      return true;
    pros::delay(20);
  }
  return false;
}
} // namespace screen
} // namespace sfx