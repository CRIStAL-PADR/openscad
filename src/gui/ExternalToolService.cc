#include "gui/ExternalToolService.h"
#include <QDesktopServices>

std::unique_ptr<ExternalToolInterface> ExternalToolService::create(
        PrintServiceType serviceType, const QString& serviceName, FileFormat fileFormat)
{
  switch (serviceType) {
  case PrintServiceType::NONE:
    // TODO: Print warning
    return nullptr;
    break;
  case PrintServiceType::PRINT_SERVICE: {
    if (const auto printService = PrintService::getPrintService(serviceName.toStdString())) {
      return createExternalPrintService(printService, fileFormat);
    }
    LOG("Unknown print service \"%1$s\"", serviceName.toStdString());
    return nullptr;
    break;
  }
  case PrintServiceType::OCTOPRINT:
    return createOctoPrintService(fileFormat);
    break;
  case PrintServiceType::LOCAL_APPLICATION:
    return createLocalProgramService(fileFormat);
    break;
  }
  return {};
}

void ExternalToolService::send(ExternalToolInterface& service,
                               const QString& filename_,
                               const std::shared_ptr<const Geometry>& geometry,
                               Camera* camera,
                               std::function<bool (double)> cb)
{
  QString filename = filename_;
  if (filename.isEmpty())
      filename = "Untitled.scad";
  // TODO: Replace suffix to match exported file format?

  filename = filename + QString::fromStdString("." + fileformat::toSuffix(service.fileFormat()));

  if (!service.exportTemporaryFile(geometry, filename, camera)) {
    return;
  }

  if (!service.process(filename.toStdString(), cb)) {
    return;
  }
//  updateStatusBar(nullptr);

//  this->progresswidget = new ProgressWidget(this);
//  connect(this->progresswidget, &ProgressWidget::requestShow, this, &MainWindow::showProgress);

//  const bool process_status = service.process(filename.toStdString(), [this](double permille) {
//    return network_progress_func(permille);
//  });

//  updateStatusBar(nullptr);

  const auto url = service.getURL();
  if (!url.empty()) {
    QDesktopServices::openUrl(QUrl{QString::fromStdString(url)});
  }
}
