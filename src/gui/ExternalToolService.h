#pragma once

#include "gui/ExternalToolInterface.h"

class ExternalToolService
{
public:

    static std::unique_ptr<ExternalToolInterface> create(PrintServiceType serviceType,
                                                         const QString& serviceName,
                                                         FileFormat fileFormat);

    static void send(ExternalToolInterface& service,
              const QString& filename,
              const std::shared_ptr<const Geometry>& geometry,
              Camera* camera,
              std::function<bool (double)>);
};
