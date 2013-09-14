#ifndef SMOGCONTROLLER_HPP
#define SMOGCONTROLLER_HPP

// STD
#include <string>
// Smog
#include "SmogMainWindow.hpp"

/**
 * UI class of the main window.
 */
namespace Ui {
class SmogMainWindow;
}

/**
 * Controller class of the application
 * @brief The SmogController class
 */
class SmogController
{
public:
    /**
     * Constructor of controller
     * @brief SmogController constructor
     */
    SmogController(Ui::SmogMainWindow* ui);

    /**
     * @brief Load cloud.
     * @param filename path to the file to load.
     */
    void loadCloud(const std::string& filename);
private:
};

#endif // SMOGCONTROLLER_HPP
