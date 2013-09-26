#ifndef SMOGMAINWINDOW_HPP
#define SMOGMAINWINDOW_HPP

// Std
#include <memory>
// Qt
#include <QMainWindow>
// Backend
#include "CloudModel.hpp"
#include "CloudEntry.hpp"

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

    void on_actionLoad_Cloud_triggered();

    void on_actionIncrease_point_size_triggered();

    void on_actionDecrease_point_size_triggered();

    void on_actionBackground_Color_triggered();

    void cloudModelChanged(const QModelIndex& from, const QModelIndex& to);

    void on_actionClose_Cloud_triggered();

private:
    // User interface object
    Ui::SmogMainWindow *ui;

    // Update cloud on visualizer
    void updateOnVisibility(CloudEntry::Ptr cloudEntry);

    // Qt model for cloud list
    std::shared_ptr<CloudModel> mCloudModel;
};

#endif // SMOGMAINWINDOW_HPP
