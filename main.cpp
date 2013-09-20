// Smog
#include "SmogMainWindow.hpp"
// QT
#include <QApplication>

#include "CloudStore.hpp"

int main(int argc, char *argv[])
{
    // Create qt application
    QApplication a(argc, argv);
    // Create main window
    SmogMainWindow w;
    // Show main window
    w.show();
    // Execute qt application
    return a.exec();
}
