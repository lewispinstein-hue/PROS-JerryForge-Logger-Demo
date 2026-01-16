#pragma once

#include "fmt/format.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cstdint>
#include <vector>

namespace sfx {

namespace screen {

/// Identifiers for the three interactive bottom buttons
enum class ButtonId { NONE = -1, LEFT = 0, MIDDLE = 1, RIGHT = 2 };
/**
 * @brief Manages a specific UI layout with scrolling text and three bottom
 * buttons.
 */
class Manager {
public:
  // --- Configuration Constants ---

  /// Standard V5 Brain screen width
  inline static constexpr int SCREEN_WIDTH = 480;
  /// Usable height for this UI layout (excluding status bar area and bottom
  /// buttons)
  inline static constexpr int SCREEN_HEIGHT = 210;
  /// Height of the bottom interactive buttons
  inline static constexpr int BTN_HEIGHT = 40;
  /// Width of the bottom interactive buttons
  inline static constexpr int BTN_WIDTH = 100;
  /// Y-coordinate for the top edge of the button row
  inline static constexpr int BTN_Y = 190;
  /// Extra pixel tolerance for touch detection around buttons
  inline static constexpr int PADDING = 5;
  ///< approx width per char in pixels
  static constexpr int CHAR_WIDTH = 8;

  // UI Styling

  /// Button background color (dark gray)
  inline static constexpr uint32_t COLOR_BTN_FILL = 0x3D3D3D;
  /// Button outline color (light gray)
  inline static constexpr uint32_t COLOR_BTN_BORDER = 0x707070;
  /// Maximum text lines retained in the scrolling buffer
  inline static constexpr int MAX_ROWS = 8;
  /// Pixel spacing between text lines
  inline static constexpr int ROW_HEIGHT = 20;

  Manager() = default;

  /**
   * @brief RAII wrapper for PROS mutexes.
   * @details Automatically takes the mutex on construction and gives it on
   * destruction.
   */
  struct unique_lock {
    pros::Mutex &m;
    explicit unique_lock(pros::Mutex &m) : m(m) { m.take(); }
    explicit unique_lock(pros::Mutex &m, uint32_t timeout) : m(m) {
      m.take(timeout);
    }
    ~unique_lock() { m.give(); }
  };

  /**
   * @brief Renders the three interactive capsule buttons at the bottom of the
   * screen.
   * @param redraw If @c true, clears the entire screen before drawing buttons.
   */
  void drawBottomButtons(bool redraw = true);

  /**
   * @brief Blocks until one of the bottom buttons is tapped and released.
   * @param timeout Maximum wait time in milliseconds.
   * @return The @ref ButtonId of the tapped button, or @c ButtonId::NONE if
   * timed out.
   */
  ButtonId waitForBottomButtonTap(uint32_t timeout = UINT32_MAX);

  /**
   * @brief Blocks execution until a screen touch event occurs.
   * * @param timeoutMs Maximum time to wait in milliseconds. Default is
   * infinite.
   * @param detectTouchOnly If @c true, returns on any contact. If @c false,
   * waits for a press and release.
   * @return @c true if touched within timeout, @c false if timed out.
   * * @example
   * // Wait 2 seconds for a user to tap the screen
   * if (waitForScreenTouch(2000)) startAuton();
   * @endcode
   */
  bool waitForScreenTouch(uint32_t timeoutMs = UINT32_MAX,
                          bool detectTouchOnly = false);

  /**
   * @brief Clears the screen and resets the internal line buffer.
   * @note Calls @ref pros::screen::erase internally.
   */
  void clearScreen();

  /// Maximum number of rows displayed before scrolling occurs. For
  /// printToScreen
  static constexpr int maxRows = 11;
  /// Vertical spacing between text lines in pixels. For printToScreen
  static constexpr int rowHeight = 18;
  /// Left margin for text in pixels. For printToScreen
  static constexpr int xPosition = 20;

  /**
   * @brief Formats and displays scrolling text on the Robot Brain.
   * @details Manages a scrolling buffer of text lines. If the screen fills up,
   * the oldest line is removed.
   * * @param clearScreen If @c true, clears the screen and internal text buffer
   * before printing.
   * @param name Optional prefix (e.g. system name). If provided, prepends
   * "Name: " to the message.
   * @param format printf-style format string.
   * @param ... Arguments for the format string.
   * * @example
   * // Result: "Chassis: Position: 10.5"
   * printToScreen(false, "Chassis", "Position: %.1f", 10.5);
   * @endcode
   */
  template <typename... Args>
  void printToScreen(bool clear, const char *name, const char *format,
                     Args &&...args) {
    unique_lock m(sharedMutex);

    // Clear screen area if requested
    if (clear) {
      textLines.clear();
      pros::screen::erase_rect(0, 0, 480, MAX_ROWS * ROW_HEIGHT);
    }

    // Format the string safely using fmt
    std::string formatted =
        fmt::format(fmt::runtime(format), std::forward<Args>(args)...);

    // Prepend the name if provided
    if (name != nullptr && name[0] != '\0') {
      formatted = std::string(name) + ": " + formatted;
    }

    // Automatic text wrapping (naive, by character count)
    size_t maxCharsPerLine = SCREEN_WIDTH / CHAR_WIDTH;
    size_t start = 0;
    while (start < formatted.size()) {
      std::string line = formatted.substr(start, maxCharsPerLine);
      textLines.push_back(line);
      start += maxCharsPerLine;
    }

    // Clamp the number of lines
    while (textLines.size() > MAX_ROWS) {
      textLines.erase(textLines.begin());
    }

    // Redraw all lines
    for (size_t i = 0; i < textLines.size(); ++i) {
      pros::screen::print(pros::E_TEXT_SMALL, 10,
                          static_cast<int>(i * ROW_HEIGHT), "%s",
                          textLines[i].c_str());
    }
  }

  template <typename... Args>
  void printToScreen(const char *format, Args&&...args) {
    // Convenience wrapper: forwards to full overload
    printToScreen(false, "", format, std::forward<Args>(args)...);
  }

private:
  std::vector<std::string> textLines;
  pros::Mutex sharedMutex;

  // Internal coordinate calculation
  inline static constexpr int getGap() {
    return (SCREEN_WIDTH - (3 * BTN_WIDTH)) / 4;
  }
  inline static constexpr int getX(ButtonId id) {
    return (getGap() * (static_cast<int>(id) + 1)) +
           (BTN_WIDTH * static_cast<int>(id));
  }

  /**
   * @brief Helper to draw a capsule (rounded rectangle) shape.
   * @details Used internally to render the buttons.
   */
  void drawCapsule(int x, int y, int width, int height, uint32_t fillColor,
                   uint32_t borderColor);

  /// Checks if a coordinate pair lies within a specified bounding box
  bool isInside(int px, int py, int bx, int by, int w, int h);
};

} // namespace screen
} // namespace sfx