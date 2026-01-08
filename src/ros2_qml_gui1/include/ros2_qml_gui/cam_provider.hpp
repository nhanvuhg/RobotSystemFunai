#ifndef CAM_PROVIDER_HPP
#define CAM_PROVIDER_HPP

#include <QQuickImageProvider>
#include <QImage>
#include <QMutex>

class CamProvider : public QQuickImageProvider {
public:
    CamProvider();
    void setImage(const QImage &img);
    QImage requestImage(const QString &id, QSize *size, const QSize &) override;

private:
    QImage image;
    QMutex mutex;
};

#endif // CAM_PROVIDER_HPP
