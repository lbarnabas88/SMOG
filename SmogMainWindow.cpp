#include "SmogMainWindow.hpp"
#include "ui_SmogMainWindow.h"
// QT
#include <QFileDialog>
#include <QTextStream>

/**
 * Constructor of the main window.
 * @brief SmogMainWindow constructor.
 * @param parent the parend widget of the window.
 */
SmogMainWindow::SmogMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SmogMainWindow)
{
    // Setup the ui
    ui->setupUi(this);
    // Start maximized
    showMaximized();
}

/**
 * Destructor of the main window.
 */
SmogMainWindow::~SmogMainWindow()
{
    // Delete user interface.
    delete ui;
}

/**
 * @brief Called when quit action's triggered
 */
void SmogMainWindow::on_actionQuit_triggered()
{
    // Quit app
    qApp->quit();
}

/**
 * @brief Called when the load action's triggered
 */
void SmogMainWindow::on_actionLoad_Cloud_triggered()
{
    // Get file path to load
    QString filename = QFileDialog::getOpenFileName(this, "Load file");
    // If valid, log
    if(!filename.isNull())
        QTextStream(stdout) << "[MainWindow] Load file: " << filename << '\n';
}
