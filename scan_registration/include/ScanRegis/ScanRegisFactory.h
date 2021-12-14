#ifndef _SCANREGISFACTORY_H
#define _SCANREGISFACTORY_H

#include <iostream>
#include <string>
#include "ScanRegis/ScanRegisBase.h"
#include "ScanRegis/ScanRegisWithPLICP.h"

namespace ScanRegisFactory
{
    enum ScanRegisMode
    {
        PLICP,
        ICP
    };

    std::shared_ptr<ScanRegisBase> CreateScanRegisMethod(ScanRegisMode mode);

};


#endif