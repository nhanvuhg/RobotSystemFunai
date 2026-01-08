#include "ros2_qml_gui/cam_provider.hpp"
#include <QColor>

CamProvider::CamProvider()
    : QQuickImageProvider(QQuickImageProvider::Image) {}

void CamProvider::setImage(const QImage &img)
{
    QMutexLocker locker(&mutex);
    image = img;
}

QImage CamProvider::requestImage(const QString &, QSize *size, const QSize &)
{
    QMutexLocker locker(&mutex);
    if (image.isNull())
    {
        if (size)
            *size = QSize(1, 1);
        QImage fallback(1, 1, QImage::Format_RGB888);
        fallback.fill(QColor("#2a2a2a"));
        return fallback;
    }

    if (size)
        *size = image.size();
    return image;
}
