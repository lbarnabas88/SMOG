#ifndef SMOGMAINWINDOW_HPP
#define SMOGMAINWINDOW_HPP

// STD
#include <memory>
// QT
#include <QMainWindow>
// Smog
#include "SmogController.hpp"

/**
 * UI class of the main window.
 */
namespace Ui {
class SmogMainWindow;
}

/**
 * Main window class of the application.
 * @brief The SmogMainWindow class.
 */
class SmogMainWindow : public QMainWindow
{
    Q_OBJECT
public:
    /**
     * Constructor of the main window.
     * @brief SmogMainWindow constructor.
     * @param parent the parend widget of the window.
     */
    explicit SmogMainWindow(QWidget *parent = 0);

    /**
     * Destructor of the main window.
     */
    ~SmogMainWindow();
    
private slots:

    /**
     * @brief Called when quit action's triggered
     */
    void on_actionQuit_triggered();

    /**
     * @brief Called when the load action's triggered
     */
    void on_actionLoad_Cloud_triggered();

private:
    /**
     * @brief User interface object.
     */
    Ui::SmogMainWindow *ui;
};

#endif // SMOGMAINWINDOW_HPP
