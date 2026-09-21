#pragma once
#include "core.h"
#include <QJsonObject>
#include <QString>
namespace m2m {
QJsonObject loadConfig(const QString& root,const QString& mode,const QString& overridePath);
void validateConfig(const QJsonObject& config,bool interaction);
// Keeps OpenCV/Open3D's original two viewports and uses Qt's event loop for application lifetime.
int runApplication(const QString& root,const QJsonObject& config,bool interaction,
                   int maximumFrames=0,bool synthetic=false,const QString& capturePath={},bool hidden=false,
                   const QString& videoPath={},double benchmarkSeconds=0);
}
