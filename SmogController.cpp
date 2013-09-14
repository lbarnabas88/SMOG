#include "SmogController.hpp"
// QT
#include <QTextStream>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

SmogController::SmogController(Ui::SmogMainWindow* ui)
{
}

/**
 * @brief Load cloud.
 * @param filename path to the file to load.
 */
void SmogController::loadCloud(const std::string& filename)
{
    // Output stream
    QTextStream out(stdout);
    // Print what to load
    out << "[SmogController] Load file: " << filename.c_str() << '\n';
}
